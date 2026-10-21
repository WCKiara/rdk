# Enlight LL → Isaac Sim 离线回放

`isaac_bundle/` 包含 `robot.urdf`、所需网格文件和 `motion.json`。轨迹来自 `rdk/flexiv_enlight_readonly_setup/third_party/flexiv_rdk/example_py/intermediate4_non_realtime_cartesian_motion_force_control.py` 的 `build_plan()` 与 `MotionPlan.sample()`；导出器将每帧目标法兰位姿逆解为 Enlight LL 的关节角。这个过程不连接机械臂。

## 准备轨迹和模型

当前目录已有一份左臂斜向运动的 `isaac_bundle/`。如果修改运动参数，在 Windows PowerShell 中重新导出：

```powershell
wsl -d Ubuntu-22.04 -- bash /mnt/e/desktop/MS/lab/rdk/enlight_ll_sim/prepare_isaac_bundle.sh --arm ARM_1 --delta 0.01 0 -0.01 --tilt 0 15 0
```

该脚本使用 WSL 中的 `~/flexiv_description` 和 `urdf_creation:latest`，调用同目录的 `export_isaac_bundle.py`。`--arm ARM_2` 可改为右臂；`--delta`、`--tilt`、`--start-joints` 等参数沿用 `play_intermediate4.py`。

## 在当前电脑上检查文件

以下命令只读取模型与轨迹，不启动 Isaac Sim，也不需要 Isaac Sim 的 Python：

```powershell
python E:\desktop\MS\lab\rdk\enlight_ll_sim\isaac_replay.py --bundle E:\desktop\MS\lab\rdk\enlight_ll_sim\isaac_bundle --check-bundle
```

## 以后在装有 Isaac Sim 的电脑上回放

把 `rdk/enlight_ll_sim/` 中的回放脚本和 `isaac_bundle/` 一起带到那台电脑，并把下面路径改成那台电脑的实际路径。在 PowerShell 中运行：

```powershell
& "C:\path\to\isaac-sim\python.bat" "D:\path\to\rdk\enlight_ll_sim\isaac_replay.py" --bundle "D:\path\to\rdk\enlight_ll_sim\isaac_bundle"
```

默认往返循环播放；加 `--once` 则正向播放一次并停在终点，关闭窗口退出。脚本将 URDF 导入 Isaac Sim，在关节名对应后逐帧设置关节角。画面显示的是离线参考轨迹，不运行 Flexiv 控制器，也不向真机发送命令。

## 本机 Ubuntu 启动

本机的安装目录约定为 `/mnt/data/kiara/isaacsim/6.0.0`，使用数据盘保存安装文件和默认缓存。

```bash
cd /home/kiara/rdk
./enlight_ll_sim/run_isaac_offline.sh --check-bundle
./enlight_ll_sim/run_isaac_offline.sh
```

启动器只运行本目录的离线回放脚本，使用现有本地模型和轨迹，不运行 RDK 控制示例。可以用 `ISAAC_SIM_ROOT` 指定其他安装目录。窗口渲染分辨率为 960×540。

如果 `nvidia-smi` 失败，启动器会停止。本机的 `6.8.1-1058-realtime` 内核无法加载当前 NVIDIA 模块；已安装的 `6.8.0-138-generic` 内核有匹配的 NVIDIA 580.178.04 模块。切换内核需要保存工作并重启，在 GRUB 的 `Advanced options for Ubuntu` 中选择 `Ubuntu, with Linux 6.8.0-138-generic`。

本机 RTX 2080 8 GB 显存、16 GB 内存低于 [Isaac Sim 6.0.0 官方最低配置](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/requirements.html)。这里只准备简单的本地参考轨迹回放，实际运行效果仍需在驱动可用后验证。

## 坐标对应范围

Isaac Sim 的 `/World/EnlightLL` 放在世界原点，长度单位为米，不额外旋转模型；两臂安装位姿取自同一份 Enlight LL URDF。`motion.json` 的 `world_frame` 为 `world`，位置单位为米，关节角单位为弧度，姿态四元数顺序为 `[qw, qx, qy, qz]`。当前将对应手臂的 `flange` 当作 TCP。

因此，模型和导出的关节轨迹使用同一套 URDF 坐标。要与以后接入的实体机械臂逐帧对齐，还需要确认真实底座在世界中的位姿、实际工具 TCP 标定，以及控制器执行时的关节状态；原示例的控制器可能生成与离线参考不同的中间轨迹。

这台电脑上尚未运行 Isaac Sim；URDF 导入、渲染和回放效果需要在装有 Isaac Sim 的电脑上验证。
