"""Analyze the existing recording offline; never import the robot SDK."""

from pathlib import Path
import csv
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


root = Path(__file__).resolve().parent
source = root / "whole_arm_contact.csv"
with source.open(newline="") as stream:
    reader = csv.DictReader(stream)
    fields = reader.fieldnames
    rows = list(reader)
selected = [r for r in rows if r["joint_group"] == "ARM_1"]
extracted = root / "whole_arm_contact_arm1_extracted.csv"
with extracted.open("w", newline="", encoding="utf-8") as stream:
    writer = csv.DictWriter(stream, fieldnames=fields)
    writer.writeheader()
    writer.writerows(selected)

keys = ["force_x_N", "force_y_N", "force_z_N", "position_x_m", "position_y_m", "position_z_m"]
sids = sorted({int(r["sample"]) for r in rows})
arrays = {}
for group in sorted({r["joint_group"] for r in rows}):
    subset = sorted((r for r in rows if r["joint_group"] == group),
                    key=lambda r: (int(r["sample"]), int(r["link_index"])))
    links = sorted({int(r["link_index"]) for r in subset})
    arrays[group] = np.array([[float(r[k]) for k in keys] for r in subset]).reshape(len(sids), len(links), 6)
a = arrays["ARM_1"]
concatenated = np.concatenate([arrays["ARM_1"], arrays["ARM_2"]], axis=1)
firsts = {int(r["sample"]): r for r in selected if int(r["link_index"]) == 0}
host = np.array([int(firsts[i]["host_time_ns"]) for i in sids], dtype=np.int64)
robot = np.array([int(firsts[i]["robot_time_sec"]) * 10**9 + int(firsts[i]["robot_time_nsec"])
                  for i in sids], dtype=np.int64)
t = (host - host[0]) / 1e9
norms = np.linalg.norm(a[:, :, :3], axis=2)
start = t < 5
end = t > t[-1] - 5
summary = {
    "source": source.name,
    "extracted_arm1_csv": extracted.name,
    "samples": len(sids), "arm1_rows": len(selected), "links": a.shape[1],
    "span_s": float(t[-1]), "rate_hz": float((len(sids) - 1) / t[-1]),
    "sample_interval_median_ms": float(np.median(np.diff(host)) / 1e6),
    "sample_interval_max_ms": float(np.max(np.diff(host)) / 1e6),
    "robot_span_s": float((robot[-1] - robot[0]) / 1e9),
    "robot_and_host_initial_clock_difference_s": float((robot[0] - host[0]) / 1e9),
    "group_link_counts": {k: int(v.shape[1]) for k, v in arrays.items()},
    "ALL_matches_ARMS_exactly": bool(np.array_equal(arrays["ALL"], arrays["ARMS"])),
    "ALL_matches_concatenated_single_arms_exactly": bool(np.array_equal(arrays["ALL"], concatenated)),
    "ALL_vs_ARMS_max_absolute_difference": float(np.max(np.abs(arrays["ALL"] - arrays["ARMS"]))),
    "ALL_vs_concatenated_arms_max_absolute_difference": float(np.max(np.abs(arrays["ALL"] - concatenated))),
    "unit_caveat": "N and m are nominal recorder units; v2.1 multi-contact docs do not explicitly annotate units.",
    "link_statistics": [],
}
for link in range(a.shape[1]):
    f, p, n = a[:, link, :3], a[:, link, 3:], norms[:, link]
    summary["link_statistics"].append({
        "link_index": link, "samples": len(n),
        "norm_mean": float(n.mean()), "norm_std": float(n.std()),
        "norm_min": float(n.min()), "norm_max": float(n.max()),
        "zero_force_and_position_percent": float(np.mean(np.all(a[:, link, :] == 0, axis=1)) * 100),
        "force_component_mean": f.mean(axis=0).tolist(),
        "force_component_std": f.std(axis=0).tolist(),
        "position_mean_m_nominal": p.mean(axis=0).tolist(),
        "position_std_mm_nominal": (p.std(axis=0) * 1000).tolist(),
        "first5s_force_mean": f[start].mean(axis=0).tolist(),
        "last5s_force_mean": f[end].mean(axis=0).tolist(),
        "last5s_minus_first5s_norm_mean": float(n[end].mean() - n[start].mean()),
        "max_step_change_force_vector_norm": float(np.linalg.norm(np.diff(f, axis=0), axis=1).max()),
    })
(root / "whole_arm_contact_arm1_analysis.json").write_text(
    json.dumps(summary, indent=2, ensure_ascii=False) + "\n")

fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True, constrained_layout=True)
colors = plt.get_cmap("tab10").colors
for link in range(a.shape[1]):
    axes[0].plot(t, norms[:, link], label=f"SDK link_index {link}", color=colors[link], lw=1)
    axes[1].plot(t, norms[:, link] - norms[start, link].mean(), color=colors[link], lw=1)
axes[0].set_title("ARM_1: per-link multi-contact force estimates (original 60 s recording)")
axes[0].set_ylabel("Force magnitude (nominal N)")
axes[0].legend(ncol=3, fontsize=9)
axes[1].set_title("Magnitude change relative to the first 5 s mean; this is not a verified no-contact baseline", fontsize=10)
axes[1].set_ylabel("Magnitude change (nominal N)")
axes[1].set_xlabel("Time from first sample (s)")
for ax in axes:
    ax.grid(alpha=0.25)
    ax.set_xlim(t[0], t[-1])
fig.savefig(root / "whole_arm_contact_arm1_force.png", dpi=160)
fig.savefig(root / "whole_arm_contact_arm1_force.pdf")
plt.close(fig)
print(json.dumps(summary, indent=2, ensure_ascii=False))
