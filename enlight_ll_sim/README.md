# Enlight LL 离线 3D 动画

在 Windows PowerShell 运行：

```powershell
wsl -d Ubuntu-22.04 -- bash /mnt/e/desktop/MS/lab/rdk/enlight_ll_sim/run_offline.sh --arm ARM_1 --delta 0.01 0 -0.01 --tilt 0 15 0
```

这会打开 RViz，显示两只机械臂。`ARM_1` 对应左臂，`ARM_2` 对应右臂；指定的手臂沿世界坐标 X 增加 10 mm、Z 减少 10 mm，并绕起始工具 Y 轴倾斜 15°。画面会往返播放，便于观察。按 `Ctrl+C` 结束。

启动器使用 WSL 中已有的 `~/flexiv_description` 和 `urdf_creation:latest`。它导入本地 `intermediate4_non_realtime_cartesian_motion_force_control.py` 的 `build_plan()` 和 `MotionPlan.sample()`，再用同一份 Enlight LL URDF 求逆运动学并发布 `/joint_states`。不创建 Flexiv RDK 机器人连接。

只校验轨迹、坐标和逆解，不打开窗口：

```powershell
wsl -d Ubuntu-22.04 -- bash /mnt/e/desktop/MS/lab/rdk/enlight_ll_sim/run_offline.sh --check-only --arm ARM_1 --delta 0.01 0 -0.01 --tilt 0 15 0
```

可把 `ARM_1` 改成 `ARM_2`，或把 `--delta`、`--tilt` 改为原示例支持的 `--position`、`--rpy`。如已知起始七轴关节角，可增加 `--start-joints q1 q2 q3 q4 q5 q6 q7`，单位为弧度；否则使用模型的默认关节姿态。

这是离线参考轨迹。虚拟世界坐标取自 URDF，暂以法兰作为 TCP。真实 NRT/MoveL 只发送终点，实际中间轨迹由控制器生成；没有真机状态和工具标定时，无法保证画面逐帧对应真实关节或实际 TCP。

离线入口默认把原示例针对现场的 TCP 位置检查范围设为虚拟的 ±2 m，方便查看任一侧手臂；这不改变 `--delta`、`--tilt` 或计算出的目标。如需检查指定范围，可显式传入原示例的 `--position-min` 和 `--position-max`。
