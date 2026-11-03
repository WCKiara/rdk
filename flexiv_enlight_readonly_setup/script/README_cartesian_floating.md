# RT 6-DoF Cartesian floating

`intermediate8_realtime_cartesian_floating.cpp` 是所附方案（RT 六维笛卡尔浮动）按本机重写的版本。方案原文件 `third_party/flexiv_rdk/example/intermediate8_realtime_cartesian_floating.cpp` 和 `example/CMakeLists.txt` 不动，只编译 `script/` 下这份。

## 控制方法：照方案

- ARM_1 六个笛卡尔轴都做位置控制。目标位姿固定为进入实时模式时的位姿，不跟随实测位姿，也不自己解 IK。
- 刚度降到 `K_x_nom × --scale`，0 就是六维零刚度浮动。阻尼比保持 RDK 名义值 0.7。
- `--passive`：六轴被动零力控制，目标力 0。
- 先用名义刚度成功发满 100 次实时指令，再降刚度。
- ARM_1 的监测阈值照方案（见下表）。

## 运行流程：照 `arm1_transparent_arm2_hold_rt.cpp`

1. 检查、上伺服：有故障就退出（加 `--clear-fault` 先清）。要有 ARM_1、ARM_2，都是 7 轴；没有外部轴；机器人静止。
2. IDLE 下设置 passive force control，切到 `NRT_PRIMITIVE_EXECUTION`。
3. ① 回正：一条原语指令 `{ARM_1: Home, ARM_2: Hold}`（`--no-home` 跳过）。
4. ② 受力回 0：一条原语指令 `{ARM_1: ZeroFTSensor, ARM_2: Hold}`。
5. ③ 切到 `RT_CARTESIAN_MOTION_FORCE`，从这里到结束一直是实时模式，不再切换。每 1 ms 的 `StreamCartesianMotionForce` 同时带两条臂：ARM_1 的浮动指令，ARM_2 的 hold 位姿（6 轴纯位置控制、名义刚度，目标是进入 ③ 时的位姿）。
6. 发满 100 次后，在实时模式里把 ARM_1 刚度降下来（或开六轴被动力控）。这是参数设置，不切换控制模式，两条臂的指令照常每 1 ms 发。
7. Ctrl+C 或任一检查不通过：先停实时任务，再 `Stop()`，机器人进入 IDLE 并保持当前位置。最后打印 ③ 期间 ARM_2 的最大偏离和速度。

每次给 ARM_1 发指令都同时给 ARM_2 发 hold，不单独控制一条臂。

另外两处：

- `--passive` 在上伺服之前检查 ARM_1 的 `raw_ft_sensor`（RDK：没装力传感器时为 0）。这台机器 Rx/Ry/Rz 力矩控制会被拒绝，检查到 0 就直接退出，机器人不动。
- `--log=FILE.csv`：浮动期间由主线程按 100 Hz 记录，实时任务里不读写文件。

## 编译

在 RDK 2.1 已安装到 `.rdk-v2.1` 的 Ubuntu 机器上，从 `rdk/flexiv_enlight_readonly_setup` 运行。`script/CMakeLists.txt` 新加了这个目标，第一次要重新配置：

```bash
cmake -S script -B .build/script && cmake --build .build/script --target intermediate8_realtime_cartesian_floating
```

## 运行

一次只运行一条，结束后检查机器人状态再跑下一条。手放在 E-stop 旁边。① 回正前确认 ARM_1 路径上没有东西；② 期间不要碰 ARM_1；③ 降刚度和程序结束时扶住 ARM_1。

```bash
sudo .build/script/intermediate8_realtime_cartesian_floating Enlight-LL-062034 --scale=0.10
sudo .build/script/intermediate8_realtime_cartesian_floating Enlight-LL-062034 --scale=0.05
sudo .build/script/intermediate8_realtime_cartesian_floating Enlight-LL-062034 --scale=0.02
sudo .build/script/intermediate8_realtime_cartesian_floating Enlight-LL-062034 --scale=0.00
```

| 选项 | 作用 |
|---|---|
| `--scale=S` | ARM_1 六轴刚度 = `K_x_nom × S`，范围 [0, 1]，默认 0.10 |
| `--nullspace=W` | 肘部（零空间）拉回进入 ③ 时姿态的权重，范围 [0.1, 1]，默认 0.5。0.1 是 RDK 允许的最小值，不等于关闭零空间控制 |
| `--passive` | 六轴被动零力控制，此时 `--scale` 不起作用。需要六维力传感器，本机没有 |
| `--log=FILE.csv` | 浮动期间 100 Hz 记录 ARM_1 的 `tcp_pose`、`tcp_twist`、`tcp_wrench`、`q`、`dtheta`。文件属主是 root |
| `--no-home` | 跳过 ① 回正，在当前姿态受力回 0 后直接浮动 |
| `--clear-fault` | 启动时有故障先尝试清除；不加则直接退出 |
| `-h`, `--help` | 打印用法，不连接机器人 |

例如零刚度、最弱零空间跟踪，并记录数据：

```bash
sudo .build/script/intermediate8_realtime_cartesian_floating Enlight-LL-062034 --scale=0.00 --nullspace=0.1 --log=logs/i8_scale0_ns010.csv
```

在本机比较被动力控的手感，用 `arm1_transparent_arm2_hold_rt_cartesian --passive`：X Y Z 被动零力，Rx Ry Rz 零转动刚度，见 [COMMANDS.md](COMMANDS.md)。

## 监测阈值

示例值，不是 Flexiv 认证的安全限值。

| 项目 | 阈值 | 何时检查 |
|---|---|---|
| ARM_1 末端线速度 / 角速度 | 0.20 m/s / 0.70 rad/s | ③ |
| ARM_1 末端离进入 ③ 时的位置 | 0.20 m | ③ |
| ARM_1 关节离位置极限 | 0.12 rad（进入 ③ 时就不到则不开始） | ③ |
| ARM_1 关节速度 | `dq_max` 的 30% | ③ |
| ARM_2 关节离开 hold 位置 | 0.01 rad | ①②③ |
| 实时任务心跳 | 100 ms 没有成功发出指令 | ③ |
| 断线、E-stop、故障、未就绪、控制模式被切走、实时指令迟到过多 | 检测到即停 | ①②③ |

正常 Ctrl+C 结束时没有"已停止"这一行，退出码 0。"已停止：……"后面是停机原因，退出码 1。参数错误或没有 root，退出码 2。

## 用日志比较透明度

`tcp_wrench` 是世界坐标系下的末端外力，本机由关节力矩估算，② 已清零。按所附方案第 7 节比较三项：

- 起动拖动力：缓慢推动时的 `tcp_wrench`
- 运动阻力：相近 TCP 速度下的 `tcp_wrench`
- 松手漂移：松手后的 `tcp_pose` 和 `tcp_twist`
