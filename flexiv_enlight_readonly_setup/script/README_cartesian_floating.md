# RT 6-DoF Cartesian floating

`intermediate8_realtime_cartesian_floating.cpp` 是所附方案（RT 六维笛卡尔浮动）按本机重写的版本。方案原文件 `third_party/flexiv_rdk/example/intermediate8_realtime_cartesian_floating.cpp` 和 `example/CMakeLists.txt` 不动，只编译 `script/` 下这份。

## 控制方法：照方案，只有一处不同

- ARM_1 六个笛卡尔轴都做位置控制，不自己解 IK。
- **和方案不同：** 降刚度之后，ARM_1 的目标位姿每 1 ms 跟随实测位姿。方案的目标位姿一直锁在进入实时模式时的位姿，2026-11-03 实测刚度 0.10 时 ARM_1 像被弹簧拴在起点，推不动、松手弹回。跟随之后弹簧力约为 0，只剩阻尼：哪里都能拖，松手停在原地；没人推时是匀速漂，不会越漂越快。预热阶段（名义刚度）目标仍然锁在起点。
- 刚度降到 `K_x_nom × --scale`，阻尼比用 `--zeta`（默认 RDK 名义值 0.7），两者都可以每个轴单独设。RDK 的笛卡尔模式没有逐关节（J1–J7）阻尼。因为目标跟随实测位姿，`--scale` 实际决定的是阻尼（阻尼 = 2 × Z_x × √(刚度 × 惯量)），越小越透明。0 时阻尼也是 0，没人推会越漂越快；程序不设速度限位，只能靠 E-stop 和控制器自带的保护停。
- ARM_2 的零空间跟踪权重设为上限 1.0（RDK 默认 0.5）。11-03 实测默认值下 ARM_2 的 J1、J3 在零空间里反向漂，约 4 秒超过当时 0.01 rad 的停机检查（这个检查现已去掉）。
- **每个轴单独锁定（2026-11-03 加 Rx Ry Rz 锁定，11-06 拆成每轴一个参数）：** 源码里是 `kLockX`、`kLockY`、`kLockZ`、`kLockRx`、`kLockRy`、`kLockRz`，改了要重新编译。
  - `true` 是锁定：这个轴的目标一直是进入 ③ 时的值，不跟随实测值，刚度保持名义刚度 `K_x_nom`（RDK 允许的最大值）。
  - `false` 是能拖。
  - 默认 X Y Z 能拖、Rx Ry Rz 锁定，和原来的 `kLockRotation = true` 完全一样。
  - 轴都是世界坐标系的，RDK 的笛卡尔运动控制一直用世界坐标系。
  - 只锁一部分转动轴时，程序把"从起始姿态转到当前姿态"的旋转向量在锁定轴上的分量拉回 0，能拖的轴上不加弹簧力（`FollowPose`）。
- `--passive`：能拖的轴做被动零力控制，目标力 0；锁定的轴照样位置控制。Rx Ry Rz 里有能拖的轴时需要六维力传感器，本机没有。
- 先用名义刚度成功发满 100 次实时指令，再降刚度。
- **和方案不同：** 不设方案里的速度、位置限位类停机检查（2026-11-03 按要求去掉），见下方"停机条件"。

## 运行流程：照 `arm1_transparent_arm2_hold_rt.cpp`

1. 检查、上伺服：有故障就退出（加 `--clear-fault` 先清）。要有 ARM_1、ARM_2，都是 7 轴；没有外部轴；机器人静止。
2. IDLE 下设置 passive force control，切到 `NRT_PRIMITIVE_EXECUTION`。
3. ① 回正：一条原语指令 `{ARM_1: Home, ARM_2: Hold}`（`--no-home` 跳过）。
4. ② 受力回 0：一条原语指令 `{ARM_1: ZeroFTSensor, ARM_2: Hold}`。
5. ③ 切到 `RT_CARTESIAN_MOTION_FORCE`，从这里到结束一直是实时模式，不再切换。每 1 ms 的 `StreamCartesianMotionForce` 同时带两条臂：ARM_1 的浮动指令，ARM_2 的 hold 位姿（6 轴纯位置控制、名义刚度，目标是进入 ③ 时的位姿）。
6. 发满 100 次后，ARM_1 能拖的轴的目标改成跟随实测值（锁定的轴不跟随），再在实时模式里把能拖的轴的刚度降下来（或给能拖的轴开被动力控）。这是参数设置，不切换控制模式，两条臂的指令照常每 1 ms 发。
7. Ctrl+C 或机器人不能继续（见"停机条件"）：先停实时任务，再 `Stop()`，机器人进入 IDLE 并保持当前位置。最后打印 ③ 期间 ARM_2 的最大偏离和速度，以及拖动期间的控制模式检查结果。

每次给 ARM_1 发指令都同时给 ARM_2 发 hold，不单独控制一条臂。

另外两处：

- `--passive` 在上伺服之前检查 ARM_1 的 `raw_ft_sensor`（RDK：没装力传感器时为 0）。这台机器 Rx/Ry/Rz 力矩控制会被拒绝，检查到 0 就直接退出，机器人不动。只在 Rx Ry Rz 里有能拖的轴（`kLockRx`、`kLockRy`、`kLockRz` 有 `false`）时检查：三个都锁定时 Rx Ry Rz 不做力控，用不到六维力传感器。
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
| `--scale=S` | ARM_1 能拖的轴的刚度 = `K_x_nom × S`，范围 [0, 1]，默认 0。锁定的轴不受影响。1 个数 = 6 个轴都用它；逗号分开的 6 个数 = X,Y,Z,Rx,Ry,Rz 各一个 |
| `--zeta=Z` | ARM_1 能拖的轴的阻尼比 Z_x，范围 [0.3, 0.8]，默认 0.7（锁定的轴固定 0.7）。刚度为 0 的轴不起作用；1 个数或 6 个数，同 `--scale`。`--passive` 时不起作用 |
| `--nullspace=W` | 肘部（零空间）拉回进入 ③ 时姿态的权重，范围 [0.1, 1]，默认 0.5。0.1 是 RDK 允许的最小值，不等于关闭零空间控制 |
| `--passive` | 能拖的轴做被动零力控制，此时 `--scale` 不起作用；锁定的轴照样锁定。Rx Ry Rz 里有能拖的轴时需要六维力传感器，本机没有 |
| `--active` | 能拖的 X Y Z 轴做主动零力控制：闭环用关节力矩估算的末端力，控制器主动带着关节走，能抵掉一部分关节摩擦和惯量（J1 J2 变轻主要靠它）。Rx Ry Rz 照样位置控制，不需要六维力传感器。末端力估算有偏差时没人推也会自己走，最快到 `kActiveMaxLinearVel`（默认 1.0 m/s）。不能和 `--passive` 同时用 |
| `--nullspace-follow` | 肘部参考姿态每 10 ms 跟随实测关节位置，肘部不再被拉回进入 ③ 时的姿态（J1 J2 J3 一起转、末端不动的那种动作变轻）。没人推时肘部可能自己漂 |
| `--log=FILE.csv` | 浮动期间 100 Hz 记录 ARM_1 的 `tcp_pose`、`tcp_twist`、`tcp_wrench`、`q`、`dtheta`。文件属主是 root |
| `--no-home` | 跳过 ① 回正，在当前姿态受力回 0 后直接浮动 |
| `--clear-fault` | 启动时有故障先尝试清除；不加则直接退出 |
| `-h`, `--help` | 打印用法，不连接机器人 |

例如零刚度、最弱零空间跟踪，并记录数据：

```bash
sudo .build/script/intermediate8_realtime_cartesian_floating Enlight-LL-062034 --scale=0.00 --nullspace=0.1 --log=logs/i8_scale0_ns010.csv
```

在本机比较被动力控的手感，用 `arm1_transparent_arm2_hold_rt_cartesian --passive`：X Y Z 被动零力，Rx Ry Rz 零转动刚度，见 [COMMANDS.md](COMMANDS.md)。

## 停机条件

程序不设速度、位置限位类的停机检查。ARM_1 末端速度、离起点的位移、关节离极限的余量、关节速度，以及 ARM_2 偏离 hold 位置，都不检查。ARM_2 的偏离仍然记录，结束时打印。拖动时手放在 E-stop 旁边。控制器自带的保护（关节硬极限、安全设置里的限速、碰撞检测等）RDK 关不掉，照常有效。

程序只在机器人自己不能继续时停：

| 项目 | 条件 | 何时检查 |
|---|---|---|
| 断线、E-stop、故障、未就绪、实时指令迟到过多 | 检测到即停 | ①②③ |
| 控制模式被切走 | ①② 不是 `NRT_PRIMITIVE_EXECUTION`，③ 不是 `RT_CARTESIAN_MOTION_FORCE` | ①②③ |
| 实时任务心跳 | 100 ms 没有成功发出指令 | ③ |

## 拖动期间全程实时模式

只有 ①② 用非实时模式 `NRT_PRIMITIVE_EXECUTION`（Home、ZeroFTSensor 只有原语版本），这时 ARM_1 由原语控制，不能拖。拖动只在 ③：

- ③ 开头切到 `RT_CARTESIAN_MOTION_FORCE` 之后，直到结束时 `Stop()`（进入 IDLE），程序没有再切换控制模式。
- ARM_1 的指令由 1 kHz 实时调度器每 1 ms 发一次 `StreamCartesianMotionForce`（实时接口）。降刚度用的 `SetCartesianImpedance` 只是实时模式里的参数设置，不切换控制模式。
- 主线程每 10 ms 查一次控制模式，不是 `RT_CARTESIAN_MOTION_FORCE` 就停机，并打印"控制模式被切到 ……"。
- 结束时打印一行"拖动 X 秒期间主线程每 10 ms 查一次控制模式，N 次都是 RT_CARTESIAN_MOTION_FORCE；③ 实时任务共发出 M 次 StreamCartesianMotionForce"。M 大约是 ③ 的秒数 × 1000。

正常 Ctrl+C 结束时没有"已停止"这一行，退出码 0。"已停止：……"后面是停机原因，退出码 1。参数错误或没有 root，退出码 2。

## 用日志比较透明度

`tcp_wrench` 是世界坐标系下的末端外力，本机由关节力矩估算，② 已清零。按所附方案第 7 节比较三项：

- 起动拖动力：缓慢推动时的 `tcp_wrench`
- 运动阻力：相近 TCP 速度下的 `tcp_wrench`
- 松手漂移：松手后的 `tcp_pose` 和 `tcp_twist`
