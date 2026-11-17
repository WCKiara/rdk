# RT 笛卡尔导纳控制（intermediate8）

`intermediate8_realtime_cartesian_floating.cpp`：ARM_1 末端导纳控制，可以手拖；ARM_2 全程不动。

- 2026-11-15 从"降刚度浮动"（阻抗控制）改成导纳控制。文件名、可执行文件名没改。
- 改之前的版本原样保留为 `intermediate8_realtime_cartesian_floating_original.cpp`，只作对照，不编译。那个版本的说明在 git 历史里（提交 6a09332 的这个文件）。
- 所附方案的原文件 `third_party/flexiv_rdk/example/intermediate8_realtime_cartesian_floating.cpp` 和 `example/CMakeLists.txt` 不动，只编译 `script/` 下这份。
- **状态：** 11-15 改的时候只做了离线检查：用 RDK 2.1 头文件编译通过，导纳的离散公式和解析解一致。还没有链接 RDK 库编译，也没有上真机。

## 控制方法：导纳

ARM_1 本身一直是硬的位置控制，手感由程序里的虚拟质量-阻尼模型决定：

1. ARM_1 六个轴全程纯位置控制：名义刚度 `K_x_nom`、阻尼比 0.7（RDK 默认），不降刚度、不开力控，不自己解 IK。
2. 每 1 ms 读末端外力 F = `tcp_wrench`：世界坐标系，手加在 TCP 上的力和力矩，本机由关节力矩估算。
3. F 先过死区：力、力矩各自按合起来的大小算，不到死区当作 0，只有超过的部分进模型，所以过死区时不跳。
4. 能拖的轴按 M·dv/dt + D·v = F 算出目标速度 v，积分出目标位姿，连同目标速度一起发出去，由名义刚度的位置控制跟踪。离散用 1 ms 的精确解（F 在 1 ms 内不变），M、D 取多大都不会数值发散。

手感只由 M（`--mass`）、D（`--damping`）和死区决定，和机器人本身的刚度、阻尼无关：

- 一直用力 F 推，最后匀速 (|F| − 死区)/D。D 越小越轻、越快。
- M 越小，起动、变向越轻。太小容易抖，因为 `tcp_wrench` 是估算值，有噪声和滤波滞后。
- 松手后速度按时间常数 M/D 衰减到 0，再滑 速度 × M/D 那么远停下，不弹回。默认 M/D：平移、转动都是 0.5 s。
- 死区就是起动要用的最小力。② 清零的零点误差超出死区的部分，会让末端以 超出量/D 的速度一直走。转动的 D 小，最容易漂。

和以前的降刚度浮动不同：以前目标跟随实测位姿、刚度降到 0，手直接推机器人，`tcp_wrench` 只记录不用。现在 `tcp_wrench` 是控制输入，机器人本身一直是硬的位置控制。

其他：

- **每个轴锁不锁：** 由源码里的 `kLockX`、`kLockY`、`kLockZ`、`kLockRx`、`kLockRy`、`kLockRz` 决定，改了要重新编译。默认 6 个轴都能拖。
  - `true` 是锁定：目标速度一直是 0，目标保持进入 ③ 时的值，用名义刚度锁住，和 ARM_2 hold 一样硬。这个轴上的外力不进导纳模型。
  - 都是世界坐标系的轴：RDK 的笛卡尔运动控制一直用世界坐标系，和 `tcp_wrench` 同一个顺序。
  - 只锁一部分转动轴时，绕两个世界轴的转动叠起来，第三个轴上也会带出一点。所以程序把"从起始姿态转到目标姿态"的旋转向量在锁定轴上的分量清零。
- **外力是估算的：** 碰肘部等末端以外的地方，估算出的末端外力也会让末端动。
- **肘部（零空间）：** 不归末端导纳管，由零空间姿态控制拉回进入 ③ 时的关节姿态。权重默认 0.1，是 RDK 允许的最小值，不等于关掉。`--nullspace-follow` 时参考姿态每 10 ms 跟随实测关节位置，基本不再拉回。
- **开始导纳：** 先把目标锁在进入 ③ 时的位姿，成功发满 100 次实时指令，再开始导纳。从当前目标、速度 0 开始积分，不改刚度、不切换控制模式，目标不跳。
- **ARM_2：** 6 轴纯位置控制、名义刚度，目标位姿一直是进入 ③ 时的末端位姿。零空间权重 1.0：11-03 实测默认 0.5 时，ARM_2 的 J1、J3 在零空间里反向漂。
- **被动力控：** IDLE 下把 ARM_1 的被动力控关掉。以前的程序（降刚度浮动的 `--passive` 等）可能开过，导纳用不到力控。

## 默认值怎么来的

取最透明的一组：用 `compare_transparency.py` 分析 `logs/` 里降刚度浮动的日志，拟合 外力 = 阻尼×速度 + 惯量×加速度 + 常数，默认值取各次结果的最小值附近。用到的日志是 `i8_ns010*`、`i8_nsfollow`、`i8_passive*`；`i8_active*` 阻尼大得多，没算进来。

| 参数 | 默认 | 日志里的值 |
|---|---|---|
| 平移质量 M | 3 kg | 平移惯量项 2.6–9.8 kg |
| 转动惯量 M | 0.05 kg·m² | 转动惯量项 0.04–0.44 kg·m² |
| 平移阻尼 D | 5 N·s/m | 平移阻尼 4.2–15.7 N·s/m |
| 转动阻尼 D | 0.1 N·m·s/rad | 只有 i8_ns010、i8_nsfollow 拟合出正值（0.14、0.10），其余是负的 |
| 力死区 | 0.5 N | 静止时 \|F\| 中位数大多 0.02–0.30 N，起动力中位数 0.38–0.69 N |
| 力矩死区 | 0.05 N·m | 静止时 \|M\| 的 95% 分位 0.01–0.03 N·m（② 清零后、home 附近；离 home 远了零点误差会变大） |
| 零空间 | 权重 0.1，不跟随 | i8_nsfollow 和 i8_ns010 互有高低，没有明显更透明 |

透明的代价：起动、刹车都靠手，松手后还会滑一段。抖或漂得受不了，就调大 `--mass`、`--damping`。

## 运行流程：照 `arm1_transparent_arm2_hold_rt.cpp`

1. 检查、上伺服：有故障就退出（加 `--clear-fault` 先清）。要有 ARM_1、ARM_2，都是 7 轴；没有外部轴；机器人静止。
2. IDLE 下关掉 ARM_1 的被动力控，切到 `NRT_PRIMITIVE_EXECUTION`。
3. ① 回正：一条原语指令 `{ARM_1: Home, ARM_2: Hold}`（`--no-home` 跳过）。
4. ② 受力回 0：一条原语指令 `{ARM_1: ZeroFTSensor, ARM_2: Hold}`。导纳的输入就是这个外力，零点不准末端会自己走，这期间不要碰 ARM_1。
5. ③ 切到 `RT_CARTESIAN_MOTION_FORCE`，从这里到结束一直是实时模式，不再切换。两条臂都设成 6 轴纯位置控制、名义刚度，之后不再改。每 1 ms 的 `StreamCartesianMotionForce` 同时带两条臂。
6. 发满 100 次后开始导纳，打印"ARM_1 导纳控制已开始"。
7. Ctrl+C 或碰到停机条件（见下）：先停实时任务，再 `Stop()`，机器人进入 IDLE 并保持当前位置。最后打印：
   - ③ 期间 ARM_2 的最大偏离和速度；
   - 导纳期间 ARM_1 的最大跟踪误差；
   - 拖动期间的控制模式检查结果。

每次给 ARM_1 发指令都同时给 ARM_2 发 hold，不单独控制一条臂。`--log=FILE.csv` 由主线程按 100 Hz 记录，实时任务里不读写文件。

## 编译

在 RDK 2.1 已安装到 `.rdk-v2.1` 的 Ubuntu 机器上，从 `rdk/flexiv_enlight_readonly_setup` 运行：

```bash
cmake -S script -B .build/script && cmake --build .build/script --target intermediate8_realtime_cartesian_floating
```

## 运行

一次只运行一条，结束后检查机器人状态再跑下一条。手放在 E-stop 旁边。

- ① 回正前确认 ARM_1 路径上没有东西。
- ② 期间不要碰 ARM_1。
- 看到"ARM_1 导纳控制已开始"后，如果零点误差大，末端可能自己慢慢走，手放在旁边随时扶住。

先用默认值：

```bash
sudo .build/script/intermediate8_realtime_cartesian_floating Enlight-LL-062034
```

觉得抖或者太飘，就把质量、阻尼调大一些。1 个数只改平移 X Y Z，转动保持默认的 0.15、0.3，扭末端时 J7 照样转得动：

```bash
sudo .build/script/intermediate8_realtime_cartesian_floating Enlight-LL-062034 --mass=8 --damping=15
```

转动的单位和平移不同：手握在离 TCP 约 r = 0.1 m 处时，转动和平移手感差不多要 M_转 ≈ M·r²、D_转 ≈ D·r²，也就是平移的 1/100 左右。11-16 以前 1 个数会用到 6 个轴，`--mass=5 --damping=10` 让转动阻尼变成 10 N·m·s/rad（当时默认 0.1 的 100 倍），扭末端时 J7 几乎不转（23 秒只转了 7–12°），所以现在 1 个数只改 X Y Z。

默认值 11-16 调过一次：原来最透明的那组（平移 3 kg、5 N·s/m，转动 0.05 kg·m²、0.1 N·m·s/rad）真机上手一碰 1.2 s 内就失控，约 2 Hz 越摆越大，外力到 16 N、末端 0.34 m/s、J7 1.37 rad/s（`logs/i8_adm_m3_d5_20261116_174025.csv`）。平移 5 kg、10 N·s/m 的 3 次都稳，所以默认平移改成 5、10，转动改成原来的 3 倍 0.15、0.3。往小调要一点一点试，开始摆就调回去。

平移、转动分开设，6 个数按 X,Y,Z,Rx,Ry,Rz：

```bash
sudo .build/script/intermediate8_realtime_cartesian_floating Enlight-LL-062034 --mass=5,5,5,0.3,0.3,0.3 --damping=10,10,10,0.6,0.6,0.6
```

| 选项 | 作用 |
|---|---|
| `--mass=M` | 虚拟质量，必须 > 0。X Y Z 单位 kg，Rx Ry Rz 单位 kg·m²，默认 5,5,5,0.15,0.15,0.15。越小起动、变向越轻，太小会抖。1 个数 = 只改 X Y Z，Rx Ry Rz 保持默认；逗号分开的 6 个数 = X,Y,Z,Rx,Ry,Rz 各一个 |
| `--damping=D` | 虚拟阻尼，必须 > 0。X Y Z 单位 N·s/m，Rx Ry Rz 单位 N·m·s/rad，默认 10,10,10,0.3,0.3,0.3。一直用力 F 推最后匀速 (\|F\| − 死区)/D：越小越轻、越快，松手后也滑得越远。1 个数（只改 X Y Z）或 6 个数，同 `--mass` |
| `--nullspace=W` | 肘部（零空间）拉回进入 ③ 时姿态的权重，范围 [0.1, 1]，默认 0.1 |
| `--nullspace-follow` | 肘部参考姿态每 10 ms 跟随实测关节位置，肘部不再被拉回进入 ③ 时的姿态（J1 J2 J3 一起转、末端不动的那种动作变轻）。没人推时肘部可能自己漂 |
| `--log=FILE.csv` | 导纳期间 100 Hz 记录 ARM_1 的 `tcp_pose`、`tcp_twist`、`tcp_wrench`、`q`、`dtheta`。文件属主是 root。文件已经存在时程序不运行，不会覆盖旧记录 |
| `--no-home` | 跳过 ① 回正，在当前姿态受力回 0 后直接开始 |
| `--clear-fault` | 启动时有故障先尝试清除；不加则直接退出 |
| `-h`, `--help` | 打印用法，不连接机器人 |

以前的 `--scale`、`--zeta`、`--passive`、`--active` 都去掉了：导纳不降刚度，也不用力控。

下面这些没有命令行参数，改源码后要重新编译：

- 死区：`kForceDeadband`、`kMomentDeadband`
- 锁定：`kLockX` … `kLockRz`
- 跟踪误差上限：`kMaxTrackPositionError`、`kMaxTrackAngleError`

要比较力控（被动零力控制）的手感，用 `arm1_transparent_arm2_hold_rt_cartesian --passive`，见 [COMMANDS.md](COMMANDS.md)。

## 停机条件

程序不设速度、位置限位类的停机检查。下面这些都不检查：ARM_1 末端速度、离起点的位移、关节离极限的余量、关节速度，以及 ARM_2 偏离 hold 位置。ARM_2 的偏离仍然记录，结束时打印。拖动时手放在 E-stop 旁边。控制器自带的保护（关节硬极限、安全设置里的限速、碰撞检测等）RDK 关不掉，照常有效。

程序只在机器人自己不能继续时停：

| 项目 | 条件 | 何时检查 |
|---|---|---|
| 断线、E-stop、故障、未就绪、实时指令迟到过多 | 检测到即停 | ①②③ |
| 控制模式被切走 | ①② 不是 `NRT_PRIMITIVE_EXECUTION`，③ 不是 `RT_CARTESIAN_MOTION_FORCE` | ①②③ |
| 实时任务心跳 | 100 ms 没有成功发出指令 | ③ |
| ARM_1 跟不上导纳目标 | 实测位姿离上一次发出的目标超过 10 mm 或 0.03 rad | ③ 导纳期间 |
| `tcp_wrench` 不是有限值 | 检测到即停 | ③ 导纳期间 |

"跟不上导纳目标"这一条是改成导纳时新加的，原因是导纳目标由模型积分出来，不看机器人实际在哪：

- 机器人被挡住、到关节极限、被控制器限速时跟不上，目标还会接着往前走，名义刚度会推得越来越狠。
- 10 mm、0.03 rad 在名义刚度下大约是 150 N、60 N·m。
- 它不是速度、位置限位：拖得多快、多远都不查。
- 结束时会打印导纳期间的最大跟踪误差，按实测再调 `kMaxTrackPositionError`、`kMaxTrackAngleError`。

## 拖动期间全程实时模式

只有 ①② 用非实时模式 `NRT_PRIMITIVE_EXECUTION`（Home、ZeroFTSensor 只有原语版本），这时 ARM_1 由原语控制，不能拖。拖动只在 ③：

- ③ 开头切到 `RT_CARTESIAN_MOTION_FORCE` 之后，直到结束时 `Stop()`（进入 IDLE），程序没有再切换控制模式。
- 刚度和力控轴只在实时流开始之前设一次。导纳期间，实时任务每 1 ms 只更新 ARM_1 的目标位姿和目标速度。
- `--nullspace-follow` 时，主线程每 10 ms 调一次 `SetNullSpacePosture`。这是实时模式里的参数设置，不切换控制模式。
- 主线程每 10 ms 查一次控制模式，不是 `RT_CARTESIAN_MOTION_FORCE` 就停机，并打印"控制模式被切到 ……"。
- 结束时打印一行"拖动 X 秒期间主线程每 10 ms 查一次控制模式，N 次都是 RT_CARTESIAN_MOTION_FORCE；③ 实时任务共发出 M 次 StreamCartesianMotionForce"。M 大约是 ③ 的秒数 × 1000。

退出码：

- 正常 Ctrl+C 结束：没有"已停止"这一行，退出码 0。
- "已停止：……"：后面是停机原因，退出码 1。
- 参数错误或没有 root：退出码 2。

## 用日志比较透明度

`--log` 的格式没变：从导纳开始起 100 Hz 记一行。每行是 `t_s`（导纳开始后的秒数）和 ARM_1 的 `tcp_pose`、`tcp_twist`、`tcp_wrench`、`q`、`dtheta`，都在世界坐标系；`tcp_wrench` 是 ② 清零之后的读数。

`compare_transparency.py` 照样能用，拖法和命令见 [COMMANDS.md](COMMANDS.md) 的"量化对比透明度"。

导纳下拟合出的结果可以直接对照参数。理想情况下：

- 平移阻尼 ≈ `--damping`
- 惯量项 ≈ `--mass`
- 常数项 ≈ 力死区 0.5 N

差得多，说明 ARM_1 没跟上导纳目标，或者外力估算有偏差。以前降刚度浮动的日志（`logs/i8_ns010*`、`logs/i8_passive*` 等）也可以放在一起比。
