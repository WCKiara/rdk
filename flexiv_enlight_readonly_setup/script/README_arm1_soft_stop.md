# ARM_1 软停止 + ARM_2 持续 Hold

按 [arm1_soft_stop_implementation_instructions.md](arm1_soft_stop_implementation_instructions.md) 修改 `arm1_transparent_arm2_hold_rt_cartesian.cpp`。

改动在分支 `feature/arm1-soft-stop` 上，从 `tansparent` 的 6a09332 拉出，**还没有提交**。

**状态：** 离线测试（Test A）已通过。主程序用 RDK 2.1 头文件做了编译检查，但没有链接 RDK 库，也没有在实验室电脑上编译、上过真机。Test B–E 都还没做。

| 文件 | 作用 |
|---|---|
| `arm1_transparent_arm2_hold_rt_cartesian.cpp` | 改动后的程序。透明手拖、ARM_2 hold、实时发送、原有停机检查都保留 |
| `arm1_soft_stop.hpp` | 状态机、二次轨迹、T 的规则、刚度渐增公式、日志行、无锁队列。只用标准库 |
| `arm1_soft_stop_test.cpp` | 离线单元测试（Test A），加 1 kHz 闭环模拟。不用 RDK |
| `plot_soft_stop.py` | 画指令第 8 节的 6 张图，并打印验收数字 |
| `CMakeLists.txt` | 新加 `arm1_soft_stop_test` 目标 |
| `COMMANDS.md` | 新选项和软停止的运行命令 |

## 1. 改之前的现状（指令第 1 步）

行号指改之前的版本。查看那个版本：`git show tansparent:flexiv_enlight_readonly_setup/script/arm1_transparent_arm2_hold_rt_cartesian.cpp`

| 要查清的事 | 结论 |
|---|---|
| Robot、RDK 版本 | 444 行 `std::make_unique<rdk::Robot>(params.robot_sn)`。`CMakeLists.txt` 要求 `flexiv_rdk 2.1 EXACT`（`.rdk-v2.1`），`third_party/flexiv_rdk` 也是 2.1 |
| 控制模式 | 495 行切到 `RT_CARTESIAN_MOTION_FORCE` |
| ARM_1 透明怎么实现 | 467 行在 IDLE 下设 `SetPassiveForceControl`。497–500 行：力控坐标系 WORLD；X Y Z 力控，目标力 0（291 行）；Rx Ry Rz 位置控制，转动刚度 `kRotStiffness = 0`，自由浮动。X Y Z 设的刚度是 K_x_nom，只对锁住的轴起作用。没有外部跟随算法 |
| ARM_2 hold | 505 行在进入实时循环前采样一次 `tcp_pose`，292 行发 `RtCartesianCmd(arm2_pose)`（wrench、twist、acc 都是 0）。501 行设成 6 轴纯位置控制。不是每周期重新采样，不会漂 |
| 实时循环 | `rdk::Scheduler`，527–528 行 `AddTask(..., 1, max_priority())`，1 kHz，要 root |
| `StreamCartesianMotionForce` | 只有 290–293 行一处，两条臂一起发 |
| Stop Sign 从哪来 | **原来没有**。只有 Ctrl+C、SIGTERM、SIGHUP（214 行 `OnSignal`），作用是结束程序 |
| 一起停两条臂的旧逻辑 | `g_stop` → 实时任务不再发（268 行）→ `scheduler->Stop()`（550 行）→ `robot->Stop()`（558 行），两条臂一起进 IDLE |
| 原有保护 | 实时任务：故障（272 行）、ARM_2 关节离 hold > 0.01 rad（283 行）、ARM_1 关节速度 > 1 rad/s（284 行）。主线程每 10 ms：断线、E-stop、故障、未就绪、模式被切走、实时指令迟到（534 行）。没有日志文件 |

## 2. 和指令文档不一样的地方（先看这里）

1. **速度字段名。** 本机 `data.hpp` 里是 `RobotStates::tcp_twist`，不是指令里写的 `tcp_vel`。

2. **在透明配置下，只发减速轨迹不会让 ARM_1 减速。**
   - X Y Z 是力控，不跟踪 pose、twist、acc 指令。
   - Rx Ry Rz 的刚度是 0，阻尼也是 0（阻尼 = 2ζ√(K·M)），同样不跟踪。

   所以实时任务锁存 SOFT_STOP 之后，主线程会把 ARM_1 切回运动控制，这一步不在 1 kHz 回调里：先调一次 `SetCartesianImpedance(ARM_1, K_stop)`，再调一次 `SetForceControlAxis(ARM_1, 全 false)`。ARM_2 不碰。

   - 这不是刚度渐增，但同样是在实时流运行期间调用阻塞接口。
   - 支持这样做的依据：官方例子 `third_party/flexiv_rdk/example/intermediate5_realtime_cartesian_pure_motion_control.cpp` 在实时流运行期间调用 `SetCartesianImpedance`（112、130 行）。本项目的 `intermediate8_realtime_cartesian_floating.cpp` 从 11-03 起，一直在主线程里、实时流运行期间调用 `SetForceControlAxis`、`SetCartesianImpedance`、`SetNullSpacePosture`。
   - **还没有厂家书面确认**：另一个线程并发调用、对同一个 `Robot` 实例是否线程安全、允许多快的调用频率。Test B 要用日志里的 `impedance_request_latency_ms` 和 `rt_cycle_us` 确认这次切换不破坏 1 kHz 实时流。
   - 切换完成之前的几十毫秒里，ARM_1 仍按透明方式滑行。轨迹从锁存那一刻算起，所以切换时目标和实际只差很少：闭环模拟中延迟 30 ms、速度 0.1 m/s 时差 0.3 mm。

3. **原来没有 Stop Sign 入口。** 按指令不能擅自改成 Ctrl+C，所以加了三个普通 Stop Sign 来源：终端按 Enter、`SIGUSR1`、`--stop-di N`。Ctrl+C 仍是原来的受控退出。如果实际的 Stop Sign 来自别处（视觉程序、网络），可以用 `kill -USR1` 接进来，或者改 `StopSignInput`。

4. **显式复位 = 重新运行。** 程序里没有 HOLD → TRANSPARENT 的路径。要恢复透明手拖，Ctrl+C 退出（整机 `Stop()`），再重新运行。

5. **"交给保护停止"用的是本程序原有的停机。** 实时任务不再发 → `scheduler->Stop()` → `robot->Stop()`，**两条臂都进 IDLE**，保持当前位置。这是原来的行为，没有改。只有实测值无效、没停稳、跟踪误差太大时才走这条路，速度大不会。

6. **速度大也照样软停止（按 2026-11-15 的要求）。** 指令 5.3 节要求：停止距离、转角超限，或没有可行的 T 时，不执行二次轨迹、交给保护停止。这里按要求去掉了这些上限，T 只按减速度上限来定。
   - 速度大时，T 和停止距离跟着变大：停止距离 = |v0|²/(2 a_max)，0.5 m/s 时约 25 cm，1 m/s 时约 50 cm。
   - 剩下的保护：跟踪误差 > 50 mm 停机、原有的 ARM_1 关节速度 > 1 rad/s 停机、控制器自带的保护、E-stop。
   - 实测速度读数如果异常偏大，ARM_1 也会照着走更远。

## 3. 现在的行为

```text
                  普通 Stop Sign（Enter / SIGUSR1 / DI）
                          |
ARM_1:   TRANSPARENT ----> SOFT_STOP ----> HOLD
              ^                |              |
              |                +---- 实测值无效 / 没停稳 / 跟踪误差太大 ----> 原有停机（两臂 Stop()）
              +---- 只有 Ctrl+C 退出后重新运行 ----+
ARM_2:   HOLD ===============================> HOLD（每周期同一条指令）
```

**实时任务每 1 ms：**

1. 原有检查不变：故障、ARM_2 偏离、ARM_1 关节速度。
2. 读 ARM_1 实测值：`tcp_pose`、`tcp_twist`（最近 5 个周期的平均）、`tcp_wrench`，以及 `timestamp` 有没有更新。
3. 状态机算出 ARM_1 的指令：
   - TRANSPARENT：发原来的透明指令，逐字节相同。
   - 第一次看到 Stop Sign 的周期：锁存实测位姿 p0、R0 和速度 v0、ω0，算出 T，转入 SOFT_STOP。之后再来 Stop Sign 不重新开始。
   - SOFT_STOP：发二次轨迹，见下方公式。
   - HOLD：发终点，速度、加速度为 0。
4. ARM_2 的指令只在进入 ③ 时构造一次，之后不改。
5. 一次 `StreamCartesianMotionForce` 同时发两条臂。
6. 加了 `--log` 时，把这一周期写成一行放进无锁队列。

**主线程每 10 ms：**

1. 原有检查：断线、E-stop、模式等。
2. 查三个 Stop Sign 来源，查到了只把计数加 1。第一次时最多等 100 ms，让实时任务锁存。
3. 实时任务锁存后，把 ARM_1 切回运动控制（第 2 节第 2 条）。开了 `--stiffness-ramp` 时，之后每 50 ms 再设一次刚度。
4. "SOFT_STOP START""已切回运动控制""HOLD"各打印一次。
5. 把日志队列里的行写进文件。

**轨迹。** 设 t = 实际单调时钟时间 − t0：

- 位置 p = p0 + v0 (t − t²/2T)，速度 v = v0 (1 − t/T)，加速度 a = −v0/T。
- t ≥ T 时停在 p0 + v0 T/2，速度、加速度为 0。
- 姿态：θ = ω0 (t − t²/2T)，R = Exp(θ)·R0（ω0 是世界系角速度，所以左乘），ω = ω0 (1 − t/T)，α = −ω0/T。
- 原来锁住的轴（`--force-axis` 为 0）保持原来的目标，不减速。
- 起点和终点加速度有跳变，不是 jerk-limited 的停止。

**T 的规则：** T = max(T_min, |v0|/a_max, |ω0|/α_max)，即满足减速度上限的最短时间。速度再大也照样执行，不设停止距离、转角、T 的上限，T 和停止距离 ½|v0|T 跟着速度变大。

默认参数下的例子：

| \|v0\| | T | 停止距离 |
|---|---|---|
| 0.01 m/s | 0.3 s | 1.5 mm |
| 0.10 m/s | 0.3 s | 15 mm |
| 0.20 m/s | 0.4 s | 40 mm |
| 0.50 m/s | 1.0 s | 250 mm |
| 1.0 m/s | 2.0 s | 500 mm |

**SOFT_STOP → HOLD：** t ≥ T 之后，下面五项**连续满足 0.2 s** 才进 HOLD：

- 实测线速度 ≤ 5 mm/s
- 角速度 ≤ 0.02 rad/s
- 离终点 ≤ 5 mm
- 姿态差 ≤ 0.03 rad
- 末端外力 ≤ 20 N

T 之后 3 s 还没停稳，就走原有停机。另外，SOFT_STOP 和 HOLD 期间实测离目标超过 50 mm 或 0.3 rad，或者实测状态 20 个周期没更新，也走原有停机。程序不会无限提高刚度去硬拉。

**刚度渐增（第二阶段，默认关，`--stiffness-ramp`）：**

- 公式：K(t) = K0 + (Kf − K0)(t/T)²。
- K0 = 透明时的实际刚度：力控的 X Y Z 按 0，锁住的 X Y Z 用 K_x_nom，转动轴用 `kRotStiffness`。
- Kf = 停止刚度。
- 主线程每 50 ms 调一次阻塞的 `SetCartesianImpedance`，所以实际刚度是阶梯，不是连续曲线。
- 刚度从 0 起步时，ARM_1 一开始几乎不跟踪轨迹，阻尼也是 0。只在 Test D 里用。

## 4. 参数

都在源码"ARM_1 软停止：在这里调"一段。下面的值是研发候选值，**不是厂家安全认证值**。

| 常量 | 默认 | 含义 |
|---|---|---|
| `kStopMaxLinearDecel` / `kStopMaxAngularDecel` | 0.5 m/s² / 1.0 rad/s² | a_max、α_max。调大就停得更快、停止距离更短 |
| `kStopMinTime` | 0.3 s | T 的下限。没有上限，也没有停止距离、转角的上限 |
| `kTwistFilterSamples` | 5 | v0、ω0 平均的周期数，延迟约 2 ms |
| `kMaxStateStaleTicks` | 20 | `timestamp` 连续这么多个周期没变，算状态过期 |
| `kHoldLinearVel` / `kHoldAngularVel` | 5 mm/s / 0.02 rad/s | 停稳条件 |
| `kHoldPositionTol` / `kHoldAngleTol` | 5 mm / 0.03 rad | 停稳条件 |
| `kHoldMaxForce` | 20 N | 停稳条件 |
| `kHoldSettleTime` / `kHoldTimeout` | 0.2 s / 3 s | 停稳要保持的时间、T 之后最多等多久 |
| `kMaxTrackPositionError` / `kMaxTrackAngleError` | 50 mm / 0.3 rad | 跟踪误差保护 |
| `kStopStiffnessScale` | 1.0（`--stop-stiffness-scale`） | 停止刚度 = K_x_nom × 它，不低于透明时的刚度；锁住的轴保持 K_x_nom |
| `kStiffnessRamp` / `kStiffnessRampPeriod` | 关（`--stiffness-ramp`）/ 50 ms | 刚度渐增 |

## 5. 编译和离线测试（Test A）

在项目根目录 `rdk/flexiv_enlight_readonly_setup` 运行。`CMakeLists.txt` 新加了目标，第一次要重新配置：

```bash
cmake -S script -B .build/script && cmake --build .build/script
.build/script/arm1_soft_stop_test
```

没装 RDK 的电脑：

```bash
g++ -std=c++20 -O2 -Wall -Wextra -o arm1_soft_stop_test script/arm1_soft_stop_test.cpp && ./arm1_soft_stop_test
```

测试覆盖：

- 二次轨迹边界：t = 0、t = T、t > T、数值导数
- 零速度，正、负速度
- 原来锁住的轴
- T 的规则：速度大（到 2 m/s、3 rad/s）也照样执行，T、停止距离跟着变大
- 姿态：终点、单位四元数、符号连续、角速度，转角超过半圈也正确
- Stop Sign 连续 100 次只锁存一次
- HOLD 不自动恢复透明
- 实测值 NaN、无穷大、四元数不是单位长度、状态过期
- 时间 NaN、时间倒退、大段丢周期
- 停稳超时、跟踪误差
- 刚度公式、速度滤波、日志列数、无锁队列、参数检查
- 1 kHz 闭环模拟：抖动 −0.2/+0.3 ms，每 400 个周期丢 5 个，切运动控制延迟 30 ms。一般速度（约 0.1 m/s）和大速度（约 0.62 m/s）各跑一次，各自不开、开刚度渐增。检查 ARM_2 指令逐字节不变、没有 NaN、只锁存一次、进入 HOLD

加 `--csv FILE.csv` 可以写出模拟日志，用来离线试 `plot_soft_stop.py`（模拟日志里没有关节数据）。

## 6. 真机测试（Test B–E，不能跳级）

每次只运行一条命令，手放在 E-stop 旁边，两臂远离人、障碍物和彼此。**不要用手去接运动中的机械臂来测停止。** 日志目录要先建好（`mkdir -p logs`）。

### Test B：静止、不接触

```bash
sudo .build/script/arm1_transparent_arm2_hold_rt_cartesian Enlight-LL-062034 --log logs/soft_stop_B_$(date +%Y%m%d_%H%M%S).csv
```

- [ ] 看到"③ … 已开始"后，不碰 ARM_1，按 Enter。应依次看到"SOFT_STOP START（|v0| ≈ 0）""ARM_1 已切回运动控制（… ms）""ARM_1 HOLD"。
- [ ] 如果按 Enter 后程序整机停，并打印"实测状态已经 … 个周期没更新"：说明这台的 `RobotStates::timestamp` 不是每个周期都更新。看日志里的 `state_stale_ticks`，调大 `kMaxStateStaleTicks` 后再试。
- [ ] ARM_2 不动，程序没有退出，没有调用 `Stop()`。
- [ ] 再按一次 Enter：应提示"已经锁存，不重新开始"。
- [ ] 另开终端运行 `sudo kill -USR1 <pid>`：同上，只提示，不重新开始。
- [ ] Ctrl+C 正常退出。用 `plot_soft_stop.py` 检查下面几项：
  - 状态序列是 TRANSPARENT → SOFT_STOP → HOLD
  - ARM_2 偏离和锁存前一样
  - 实时周期 p99、最大值、迟到次数和不发 Stop Sign 时一样
  - `SetCartesianImpedance`、`SetForceControlAxis` 的用时
  - 日志里的 `arm1_*` 确实是 ARM_1 的数据（和示教器显示的 TCP 对一下）
- [ ] 刚度渐增保持关闭。

### Test C：ARM_1 低速自由空间减速

- [ ] 移走工具和工件。轻推 ARM_1 后松手，确认手已离开，再由另一人按 Enter 或运行 `kill -USR1`。
  - 主动力控（默认）下，松手后 ARM_1 自己会慢慢停，速度可能太小，测不出效果。
  - 被动力控（`--passive`）下，松手后会一直滑行，更容易测，但没有限速，推得要更轻。
- [ ] 从很小的速度开始（几 cm/s），换几个方向、几个速度重复，检查每次结果是否一致。
- [ ] 每次看图 1、图 2：实际速度没有突跳、没有反向，实际停止距离接近计划值，最后进入 HOLD。
- [ ] 刚度渐增仍然关闭，这样轨迹的问题和刚度的问题能分开看。

### Test D：刚度渐增（先要有 Test B 关于并发调用的结论）

- [ ] `--stiffness-ramp --stop-stiffness-scale 0.2` 起步，再分级加大比例。
- [ ] 监控实时抖动（图 6）、跟踪误差、图 3 的关节速度。有任何异常抖动或阻塞，立即停止这项实验。
- [ ] 多次复现，确认和纯轨迹版本相比没有引入额外的风险。

### Test E：回归

- [ ] 不发 Stop Sign 时，手拖手感和改之前一样。
- [ ] 原有停机照常有效：E-stop、ARM_1 关节速度超限、ARM_2 偏离、断线、模式被切走。
- [ ] 速度大的时候发 Stop Sign 也照样软停止，不会整机停：T、停止距离按 |v0|²/(2 a_max) 变大，最后进入 HOLD。事先确认前方空间够。
- [ ] HOLD 之后不会自己恢复透明。

## 7. 日志和画图

`--log FILE.csv` 由实时任务每周期写一行，放进预分配的无锁队列（8192 行，约 8 秒），主线程每 10 ms 写盘，实时任务里不读写文件。文件已经存在时程序不运行。

- 大约 1 MB/s。
- 文件属主是 root。
- 队列满时会丢行，结束时打印丢了几行。

日志共 91 列，和指令 8.1 节的对应关系：

| 列名 | 含义 |
|---|---|
| `time_s`、`rt_cycle_us`、`stop_sign`、`arm1_state`、`stop_elapsed_s`、`stop_T_s` | 时间、实时周期间隔、Stop Sign 次数、状态、锁存后的 t、T |
| `arm1_p*`、`arm1_q[wxyz]`、`arm1_v*`、`arm1_w*`、`arm1_f*` | ARM_1 实测位姿、速度、外力 |
| `arm1_target_*` | 发给 ARM_1 的目标位姿、速度、加速度（含角加速度 `alpha*`） |
| `arm1_q1..7`、`arm1_dtheta1..7` | ARM_1 关节位置和速度。速度用 `dtheta`（电机侧，噪声小） |
| `arm2_p*`、`arm2_q1..7`、`arm2_dtheta1..7` | ARM_2 末端位置、关节位置和速度 |
| `arm2_hold_position_error`、`arm2_hold_joint_error` | ARM_2 离 hold 位置：末端 [m]、关节最大值 [rad] |
| `requested_Kx_arm1_*`、`set_Kx_arm1_*` | ARM_1 应有的刚度、最近一次 `SetCartesianImpedance` 设成的刚度 |
| `impedance_request_latency_ms`、`motion_ctrl`、`state_stale_ticks` | 最近一次设刚度的用时、是否已切回运动控制、状态几个周期没更新 |
| `stream_status`、`fault_status` | 这一周期是否发送成功、机器人是否故障 |

画图用 `.venv/bin/python script/plot_soft_stop.py FILE.csv`，要先 `.venv/bin/pip install matplotlib`。

- 画出 6 张 PNG，放在 CSV 旁边的 `<文件名>_plots/`。
- 打印状态序列、和公式的偏差、跟踪误差、有没有反向速度、ARM_2 偏离、实时周期 p95/p99/最大、设刚度的用时。
- 只看数字加 `--no-plots`。

## 8. 测试记录

| 日期 | 项目 | 环境 | 结果 |
|---|---|---|---|
| 2026-11-15 | Test A 离线单元测试 | WSL Ubuntu 22.04、g++ 11.4，`-O2 -Wall -Wextra -Wpedantic -Wshadow` | 2841 项检查全部通过，没有编译警告 |
| 2026-11-15 | Test A，CMake 方式 | Windows、MinGW g++ 13.2，`script/CMakeLists.txt` 配置时用替身的 RDK 包，Release 编译 `arm1_soft_stop_test` | 全部通过，结果和上一行相同 |
| 2026-11-15 | 闭环模拟，一般速度，不开刚度渐增 | 同上。质量-弹簧-阻尼模型，假设 K_x_nom = 4000 N/m、300 Nm/rad；\|v0\| = 0.096 m/s、\|ω0\| = 0.27 rad/s | T = 0.3 s，计划停止距离 14.47 mm、实际 14.47 mm，锁存后 0.50 s 进 HOLD，最大跟踪误差 0.30 mm |
| 2026-11-15 | 闭环模拟，一般速度，开刚度渐增 | 同上，50 ms 一级 | 锁存后 0.52 s 进 HOLD，最大跟踪误差 0.97 mm，设刚度 7 次 |
| 2026-11-15 | 闭环模拟，大速度，不开刚度渐增 | 同上；\|v0\| = 0.62 m/s、\|ω0\| = 0.88 rad/s | 照样软停止：T = 1.23 s，计划停止距离 380 mm、实际 380 mm，锁存后 1.43 s 进 HOLD，最大跟踪误差 0.42 mm |
| 2026-11-15 | 闭环模拟，大速度，开刚度渐增 | 同上，50 ms 一级 | 锁存后 1.43 s 进 HOLD，最大跟踪误差 2.97 mm，设刚度 25 次 |
| 2026-11-15 | 主程序编译检查 | WSL g++ 11.4，`-std=gnu++20 -fsyntax-only -Wall -Wextra -Wpedantic -Wshadow`，RDK 2.1 头文件（Eigen 用替身） | 0 警告；改之前的版本同样 0 警告。**没有链接 RDK 库，没有在实验室电脑上编译** |
| 2026-11-15 | `plot_soft_stop.py` | WSL python3、numpy 2.2、matplotlib 3.10，用模拟日志 | 6 张图正常，数字和模拟一致 |
| — | Test B–E | 真机 | 未做 |

记录真机测试时请写上：

- RDK 2.1
- 控制器软件版本：程序启动时打印"控制器软件版本 …"
- 序列号
- 命令行参数
- 软停止参数（启动时打印"ARM_1 软停止：…"）
- 刚度渐增开没开
- 日志文件名

模拟只说明状态机和轨迹的逻辑没问题，不代表真机的跟踪效果。

## 9. 遗留问题

- 第 2 节第 2 条：实时流运行期间由主线程调用阻塞接口，还没有厂家书面确认。刚度渐增也一样。
- 按要求不设停止距离上限：速度大时 ARM_1 会走很远（1 m/s 约 50 cm）。前方空间够不够，只能靠操作者确认和 E-stop。
- T 只按末端线速度、角速度和减速度上限来定，下面这些都没有评估：
  - 各关节速度、加速度（只有原有的 1 rad/s 关节速度停机检查）
  - 奇异位形
  - 碰撞预测
  - 两臂间距
  - 工作空间余量
- `tcp_twist` 的 5 点平均没有用真机数据评估过。
- `RobotStates::timestamp` 的更新频率未知，`kMaxStateStaleTicks = 20` 可能要按真机调。
- 停稳容差、20 N 外力上限、50 mm 跟踪误差上限都是候选值，要结合负载和风险评估来定。
- 起点和终点的加速度有跳变。是否改成 S 曲线或五次多项式，要等二次轨迹验收之后再评估。
- 刚度渐增从 0 起步时，开始阶段没有阻尼，ARM_1 会先滑行一段。
- 力控坐标系设成 TCP 且有锁住的轴时，锁住的方向不是世界系的轴。这种情况下软停止把三个平移轴都从实测位姿开始减速。默认配置不受影响。

## 10. 回滚

原文件在 `tansparent` 分支的提交 6a09332 里：

```bash
git restore --source=tansparent -- flexiv_enlight_readonly_setup/script/arm1_transparent_arm2_hold_rt_cartesian.cpp
```

在 `rdk` 目录运行。或者直接 `git switch tansparent`，但要先处理掉未提交的改动。
