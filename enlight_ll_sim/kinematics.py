"""Offline kinematics for the two Enlight-LL arms in flexiv_description.

The transforms come from the generated URDF that RViz also displays.  This
module does not import the Flexiv SDK or communicate with a robot.
"""

from dataclasses import dataclass
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


# The nominal joint postures shipped in flexiv_description/config/Enlight-L.
_LEFT_HOME = np.array([0.0, -0.69813, 0.0, 1.570796, 0.0, -0.872665, 0.0])
_RIGHT_HOME = np.array([0.0, 0.69813, 0.0, -1.570796, 0.0, 0.872665, 0.0])


@dataclass(frozen=True)
class _Joint:
    name: str
    kind: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


def _vector(value, default):
    return np.fromstring(value if value is not None else default, sep=" ", dtype=float)


def _joint_from_xml(element):
    origin_element = element.find("origin")
    xyz = _vector(origin_element.get("xyz") if origin_element is not None else None, "0 0 0")
    rpy = _vector(origin_element.get("rpy") if origin_element is not None else None, "0 0 0")
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    transform[:3, 3] = xyz

    axis_element = element.find("axis")
    axis = _vector(axis_element.get("xyz") if axis_element is not None else None, "1 0 0")
    axis /= np.linalg.norm(axis)
    limit = element.find("limit")
    lower = float(limit.get("lower")) if limit is not None and limit.get("lower") else -2 * np.pi
    upper = float(limit.get("upper")) if limit is not None and limit.get("upper") else 2 * np.pi
    return _Joint(element.get("name"), element.get("type"), transform, axis, lower, upper)


class ArmChain:
    """A world-to-flange URDF chain for one seven-joint arm."""

    def __init__(self, joints, side):
        self._joints = joints
        self.joint_names = [joint.name for joint in joints if joint.kind != "fixed"]
        if len(self.joint_names) != 7:
            raise ValueError(f"Expected seven {side} arm joints; found {self.joint_names}")
        moving = [joint for joint in joints if joint.kind != "fixed"]
        self.lower = np.array([joint.lower for joint in moving])
        self.upper = np.array([joint.upper for joint in moving])
        home = _LEFT_HOME if side == "left" else _RIGHT_HOME
        self.default_q = np.clip(home.copy(), self.lower, self.upper)

    def forward(self, q):
        """Return T_world_flange for seven joint angles in radians."""
        q = np.asarray(q, dtype=float)
        transform = np.eye(4)
        moving_index = 0
        for joint in self._joints:
            transform = transform @ joint.origin
            if joint.kind != "fixed":
                rotation = np.eye(4)
                rotation[:3, :3] = Rotation.from_rotvec(q[moving_index] * joint.axis).as_matrix()
                transform = transform @ rotation
                moving_index += 1
        return transform

    def solve(self, target, seed):
        """Find a continuous joint posture near ``seed`` for T_world_flange.

        Returns ``(q, position_error_m, orientation_error_rad)``.  The caller
        decides whether the resulting Cartesian error is acceptable.
        """
        target = np.asarray(target, dtype=float)
        seed = np.clip(np.asarray(seed, dtype=float), self.lower, self.upper)

        def pose_error(q):
            actual = self.forward(q)
            position = actual[:3, 3] - target[:3, 3]
            orientation = Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).as_rotvec()
            return position, orientation

        def residual(q):
            position, orientation = pose_error(q)
            # Metres and radians have different scales.  A light seed term
            # keeps the redundant seventh axis on the same branch.
            return np.r_[position, 0.2 * orientation, 1e-4 * (q - seed)]

        result = least_squares(residual, seed, bounds=(self.lower, self.upper), max_nfev=100)
        position, orientation = pose_error(result.x)
        return result.x, float(np.linalg.norm(position)), float(np.linalg.norm(orientation))


def parse_chain(urdf_xml, arm, serial=None):
    """Read one Enlight-LL arm chain from the same URDF used in RViz.

    ``arm`` accepts ``ARM_1``/``left`` or ``ARM_2``/``right``.  ``serial`` is
    optional because the arm prefix can be found in the generated URDF.
    """
    side = {"ARM_1": "left", "ARM_2": "right", "left": "left", "right": "right"}[arm]
    robot = ET.fromstring(urdf_xml)
    child_to_joint = {joint.find("child").get("link"): joint for joint in robot.findall("joint")}
    if serial is None:
        flange = next(link for link in child_to_joint if link.startswith(side + "_") and link.endswith("_flange"))
    else:
        flange = f"{side}_{serial}_flange"

    joints = []
    link = flange
    while link != "world":
        element = child_to_joint[link]
        joints.append(_joint_from_xml(element))
        link = element.find("parent").get("link")
    joints.reverse()
    return ArmChain(joints, side)
