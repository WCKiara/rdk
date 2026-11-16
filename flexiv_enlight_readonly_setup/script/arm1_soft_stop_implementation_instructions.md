# Arm1 平滑停止 + Arm2 持续 Hold：RT Cartesian 控制修改流程指令

> **目标文件**：`arm1_transparent_arm2_hold_rt_cartesian.cpp`  
> **目标平台**：Flexiv 双臂系统，按 RDK C++ **v2.1** API 设计（实施前先核实实际版本）  
> **任务类型**：给工程师 / 编码 Agent 的分步实施指令，而不是未经验证可直接驱动真机的完整程序。  
> **首要原则**：只改变 Arm1 的普通操作停止行为；Arm2 始终 Hold；不能绕过或延迟安全急停、保护停止、限位和碰撞保护。

---

## 0. 任务定义与最终行为

在 `arm1_transparent_arm2_hold_rt_cartesian.cpp` 正常运行时：

- **Arm1（ARM_1）**：维持当前 Transparent（低笛卡尔刚度/可被外力带动）行为；收到已有的 **Stop Sign** 后，锁存一次停止事件，读取当前实测 TCP 位姿和速度，生成**二次位置减速轨迹**，直至速度降为零，随后在停止目标位姿持续 Hold。
- **Arm2（ARM_2）**：程序启动时锁定一个固定 Hold 位姿，并在每次实时发送中持续发送 **Arm2 固定目标位姿 + 零速度 + 零加速度**。收到 Stop Sign 前后均**不得改变 Arm2 的模式、刚度、目标位姿、速度指令或退出发送**。
- **循环**：继续以现有 RT 频率（目标 1 kHz）使用 `StreamCartesianMotionForce()` 一次提交包含 Arm1 和 Arm2 的命令映射。Soft Stop 不意味着调用 `robot.Stop()`、退出 RT 循环或停止给 Arm2 发包。
- **刚度**：本文件处于 `RT_CARTESIAN_MOTION_FORCE` 模式；严格说可改的是 **Arm1 笛卡尔刚度 `K_x`**，不是关节刚度 `K_q`。在完成轨迹软停止验证后，才可按第 7 节加入逐级提高 **Arm1 笛卡尔刚度**的功能。`SetCartesianImpedance()` 是**阻塞 API**，不可在 1 kHz 回调中按采样周期调用。

### 安全语义必须先确认

这里的 `Stop Sign` 专指**非安全相关的普通操作停止请求**（例如操作者按钮、视觉算法发出的普通暂停命令）。

- **真正的急停、安全保护停止、人员进入危险区、超限/碰撞故障**：必须继续使用机器人厂家认证的安全停止链，不能改成软停止，不能为了二次轨迹延迟保护动作。
- 本方案生成的是**参考轨迹**，不是保证实际机械臂关节一定按二次函数停止。透明模式下尤其可能受外力、低刚度、阻尼、饱和与跟踪误差影响。
- 真机测试必须由有权限的操作人员在可随时执行硬件安全停止的条件下完成；不得用手接触运动的机器人来测试停止性能。

### 最终状态机

```text
                  ordinary Stop Sign
                          |
                          v
Arm1:   TRANSPARENT ----> SOFT_STOP ----> HOLD
             ^                  |            |
             |                  +--fault----> PROTECTIVE RESPONSE
             |                               (原有安全机制)
             +----- 仅经过显式复位/重新使能 ------+

Arm2:   HOLD ==================================> HOLD
        每个 RT 周期持续发送固定目标位姿、零 twist、零 acc
```

**验收的关键事实**：停止只针对 Arm1；不会误让 Arm2 自由浮动、重新捕获 Arm2 的 Hold 点，或停止对 Arm2 的持续指令流。

---

## 1. 第一步：定位源文件、创建安全分支和备份

在已有 Flexiv 项目的根目录执行（如果项目目录不同，先 `cd` 到对应目录）：

```bash
find . -name 'arm1_transparent_arm2_hold_rt_cartesian.cpp' -print

git status --short
git switch -c feature/arm1-soft-stop
```

如项目不使用 Git，则直接备份源文件：

```bash
cp path/to/arm1_transparent_arm2_hold_rt_cartesian.cpp \
   path/to/arm1_transparent_arm2_hold_rt_cartesian.cpp.bak
```

**不要直接覆盖现有可用版本。**

### 阅读源文件时必须查清 9 件事

用编辑器或下列命令定位：

```bash
grep -nE 'RT_CARTESIAN|StreamCartesian|SetCartesianImpedance|SetJointImpedance|Stop\(|SIGINT|signal|stop|ARM_1|ARM_2|states\(' \
  path/to/arm1_transparent_arm2_hold_rt_cartesian.cpp
```

- [ ] 当前创建 `flexiv::rdk::Robot` 的位置、RDK 版本、构造参数及机械臂连接方式。
- [ ] 当前是否确实进入 `Mode::RT_CARTESIAN_MOTION_FORCE`。
- [ ] `ARM_1` 的透明控制是如何实现的：笛卡尔刚度是否为零/低值？是否开启力控轴？是否存在外部跟随算法？
- [ ] `ARM_2` 原有 Hold 目标是固定采样一次，还是每次使用实测 pose（后者会造成漂移）。
- [ ] 主 RT 循环的频率、计时方式、是否已经采用 realtime scheduler。
- [ ] 所有对 `StreamCartesianMotionForce()` 的调用位置；确保只有**一个**负责双臂命令的实时发送路径。
- [ ] `stop sign` 从哪里来（视觉程序/网络/键盘/数字 IO）；必须保留实际入口而不是擅自改成 `Ctrl+C`。
- [ ] 是否存在 `break`、`return`、`robot.Stop()`、`SetMode(IDLE)` 或共同停止双臂的旧逻辑。
- [ ] 是否已有日志与保护条件（超限、失联、异常、工作空间边界、最大力矩、外力等）。

**输出**：形成一段简短的现状说明，记录停止信号入口和 RT 发送入口的具体函数、行号。由于尚未提供源代码，本文件无法填写准确行号。

---

## 2. 第二步：核实控制模式和 RDK v2.1 API

官方 v2.1 的关键接口：

| 目的 | API / 数据 | 约束 |
|---|---|---|
| 实时笛卡尔发送 | `Robot::StreamCartesianMotionForce(std::map<JointGroup, RtCartesianCmd>)` | 仅 `RT_CARTESIAN_MOTION_FORCE`；连续约 1 kHz 发送 |
| 读两臂状态 | `Robot::states()` | 按 `JointGroup::ARM_1` 与 `ARM_2` 获取 `RobotStates` |
| 获取位姿 | `RobotStates::tcp_pose` | `[x,y,z,qw,qx,qy,qz]`（世界系） |
| 获取速度 | `RobotStates::tcp_vel` | `[vx,vy,vz,wx,wy,wz]`（世界系） |
| 设置 Arm1 笛卡尔刚度 | `Robot::SetCartesianImpedance(JointGroup::ARM_1, K_x, Z_x)` | **阻塞**；只允许合法刚度范围；需要匹配控制模式 |
| 关节阻抗模式专用 | `Robot::SetJointImpedance(JointGroup::ARM_1, K_q, Z_q)` | 只能用于 Joint Impedance 模式，**不能**在当前 Cartesian 模式直接使用 |

`RtCartesianCmd` 在 v2.1 中的构造参数顺序为：

```cpp
RtCartesianCmd(pose_d, wrench_d, twist_d, acc_d)
```

即目标位姿、目标力/力矩、目标速度、目标加速度。实际代码请以本机安装的 `flexiv/rdk/data.hpp` 头文件为准。

**严禁：** 为了得到“关节刚度渐增”，在 Arm1 运动的瞬间将整套双臂控制模式切换到 `RT_JOINT_IMPEDANCE`；这会改变双臂的控制接口和原有 Arm2 Hold，必须另立项目专门验证。

---

## 3. 第三步：把 Arm2 固定 Hold 从其他逻辑中独立出来

### 3.1 程序初始化时

1. 确认双臂都已经处于可控的、无故障状态，并且 TCP 和负载配置正确。
2. **只在进入 RT 循环之前**记录 `arm2_hold_pose = states()[ARM_2].tcp_pose`。
3. 保留原代码中已经验证过的 Arm2 Hold 刚度/阻尼和安全限制；不要为实现 Arm1 Soft Stop 而修改 Arm2 的配置。
4. 使用 `arm2_hold_pose` 构造一个固定的 Arm2 参考指令：`wrench=0`、`twist=0`、`acc=0`，仅在确认 Arm2 为纯运动控制轴时才按此设置；如果原代码使用混合力控，先核查原力控语义，不可盲目置零。
5. 记录该 Hold 位姿作为日志的 Arm2 位置误差基准。

### 3.2 每个 RT 周期都要做

```cpp
// 结构示意，不是可直接替换的完整 C++ 程序
std::map<JointGroup, RtCartesianCmd> commands;
commands[JointGroup::ARM_1] = make_arm1_command(arm1_state);
commands[JointGroup::ARM_2] = make_arm2_fixed_hold_command(arm2_hold_pose);
robot.StreamCartesianMotionForce(commands);
```

**要点**：`make_arm2_fixed_hold_command` 不可读取 Arm1 的 Stop 状态来切换目标，不可每周期重新用 `states()[ARM_2].tcp_pose` 更新 Hold 位姿。

---

## 4. 第四步：新增 Arm1 三态状态机

建议增加以下状态与数据（变量名可按现有代码风格调整）：

```cpp
enum class Arm1State {
    TRANSPARENT,
    SOFT_STOP,
    HOLD
};

struct SoftStopContext {
    bool initialized = false;
    double elapsed = 0.0;
    double duration = 0.0;
    // 缓存触发时的 ARM_1 TCP pose / twist
    // 缓存计算后的停止目标 pose
    // 保存触发时间戳和触发来源
};
```

### 4.1 事件入口

- 现有 Stop Sign 检测路径只负责设置线程安全的**一次性请求标志**，不在检测线程里操作机械臂。
- 在 RT 线程里接收事件并由 `TRANSPARENT` 原子式进入 `SOFT_STOP`。
- **事件锁存**：停止开始后，即使 Stop Sign 连续多帧出现，也**不得重置** `t=0` 或重新测量 `v0`，否则永远无法停稳。
- `SOFT_STOP` 和 `HOLD` 中不得自动回到 `TRANSPARENT`；恢复必须是独立的显式复位操作，且先确认安全条件。

### 4.2 停止启动瞬间记录

只执行一次：

1. 读取 Arm1 当前**实测** `tcp_pose` 和 `tcp_vel`；检查数据时间戳、新鲜度、有限数值及单位。
2. 对速度使用事先评估过的低延迟滤波；不要把长时间平均导致的延迟误差当作即时速度。
3. 读取/检查当前发出的 Arm1 命令，验证与实测速度、位姿没有不可接受的跳变；否则不启动软件轨迹。
4. 将当前实测平移位置记录为 `p0`，实测线速度记录为 `v0`；姿态记录为 `R0`，实测角速度记为 `omega0`。
5. 检查预计制动距离和工作空间余量，确定合法停止时间 `T`，并生成停止终点。
6. 打印一次 `SOFT_STOP START`，随后按固定频率采样日志，不允许每 1 ms 大量同步写终端。

---

## 5. 第五步：Arm1 二次位置减速轨迹（第一阶段必须先实现）

### 5.1 平移轨迹

设从收到 Stop Sign 起经过时间 `t`，设 `s = clamp(t / T, 0, 1)`。

对于 Arm1 TCP 的 `x/y/z` 三轴：

\[
\boxed{p_d(t)=p_0+v_0\left(t-\frac{t^2}{2T}\right)}
\]

\[
\boxed{v_d(t)=v_0\left(1-\frac{t}{T}\right)}
\]

\[
\boxed{a_d(t)=-\frac{v_0}{T}},\qquad 0\le t<T
\]

`t >= T` 时：

\[
p_d=p_{stop}=p_0+\frac{1}{2}v_0T,\quad v_d=0,\quad a_d=0
\]

**说明**：

- 目标**位置**是二次函数，目标**速度**线性下降；“速度是二次函数”与“位置是二次函数”不是同一件事。
- 初始与结束时加速度存在跳变，因此这种算法不等于 jerk-limited 最平顺停止。先完成二次轨迹验收，再评估是否改成 S 曲线或五次多项式。
- 使用 `t` 的**实际单调时钟时间**，不要简单使用 `循环次数 × 0.001`，否则 RT 抖动或掉周期将改变制动时间。

### 5.2 姿态（不可忽略角速度）

`tcp_pose` 的后四项是单位四元数，**不能按位置的二次公式逐元素相加**。

推荐逐步实施：

1. **最初低速验证**：只有在 Stop Sign 时实测角速度已经非常小、经检查固定姿态不会引起突然旋转时，暂时保持 `R0` 不变。
2. **正式 6D 实施**：对世界坐标系角速度 `omega0` 使用相同的减速规律。旋转向量可取 `theta(t) = omega0 * (t - t^2/(2T))`，再以 SO(3) 指数映射构造 `R_d(t) = Exp([theta(t)]x) * R0`，并转成归一化四元数。
3. 配套向 `RtCartesianCmd` 发送 `omega_d(t)=omega0*(1-t/T)` 和 `alpha_d(t)=-omega0/T`，结束后都设为零。
4. 校核四元数符号连续性、角速度参考坐标系、角速度上限和角加速度上限。

### 5.3 停止时间 `T` 不允许只填固定常数

给定初始线速度 `|v0|`、允许减速度上限 `a_max`、该运动方向上的可用停止距离 `d_free`：

\[
T \ge \frac{\lVert v_0\rVert}{a_{max}}
\]

\[
\frac{1}{2}\lVert v_0\rVert T \le d_{free}
\]

还需对**角速度/角加速度**、各轴关节速度、关节加速度、奇异位形、预测碰撞和二臂间距分别评估。任一限制无可行 `T` 时，**不得强行执行二次轨迹**，应交给已验证的保护停止路径。

例如：当初始速度只有 `0.01 m/s` 且 `T=1 s`，平移理论制动距离约 `5 mm`；若速度为 `0.10 m/s`，同样的 `T` 会造成约 `50 mm` 的继续运动。因此 **不能假设 1 秒总是安全**。

---

## 6. 第六步：构造命令与进入 Hold

### 6.1 在实时循环中按状态计算 Arm1 指令

```text
每个 RT tick：
  获取最新状态与普通停止请求
  若发生安全故障：保留原有认证安全响应；不得由 soft-stop 屏蔽
  ARM_1:
    TRANSPARENT -> 使用原来的 transparent 命令（不要改动原功能）
    SOFT_STOP   -> 使用第 5 节的目标 pose / twist / acc
    HOLD        -> 固定停止目标 pose，twist=0，acc=0
  ARM_2:
    始终使用启动时锁存的 arm2_hold_pose，twist=0，acc=0
  一次 StreamCartesianMotionForce({ARM_1: cmd1, ARM_2: cmd2})
  记录轻量日志到预分配缓冲区
```

### 6.2 `SOFT_STOP -> HOLD` 的条件

不得只在 `t == T` 时认为机械臂实际已经停止。进入 Hold 前必须同时检查：

- 已达到计划轨迹结束时刻，目标速度已为零。
- Arm1 实测 TCP 线速度、角速度低于项目设定阈值，并连续保持指定时间窗口。
- Arm1 实测位置/姿态相对终点误差在允许范围内。
- 无异常接触力、保护报警或失联迹象。

若超出合理等待时间仍不能满足条件，不应无限提高刚度“硬拉回”终点；保持适当参考指令并触发既有的保护/故障处理流程，具体动作依据设备安全规程确定。

**停止后**仍需持续发送 `{ARM_1:HOLD, ARM_2:HOLD}`，不可随意 `break`/`return`/`robot.Stop()`。仅当操作人员通过已确认的受控退出流程切换模式时，才能结束 RT 流。

---

## 7. 第七步：Arm1 刚度渐增（第二阶段，独立验收）

> **重要约束**：该文件处于 RT Cartesian 模式，因此实现的是 `K_x`（笛卡尔刚度）的渐增，**不是** `K_q`（关节刚度）。如果科研目标必须证明“每个关节的刚度单独渐增”，需要另开 `RT_JOINT_IMPEDANCE` 版本，重新设计并验证**双臂**命令，不能在运动中直接热切换。

### 7.1 目标函数

对于 Arm1 的六个 Cartesian 方向，要求：

\[
K_x(t)=K_{x,0} + (K_{x,f}-K_{x,0})\left(\frac{t}{T}\right)^2
\]

其中：

- `K_x,0`：运行阶段已配置的低刚度值，必须记录实际设定值。
- `K_x,f`：停止后 Hold 的目标刚度；各元素不应超过当前机型 `RobotInfo::K_x_nom`，必须以真机配置与厂家约束为准。
- `Z_x`：初期维持原有/厂家名义阻尼比，默认文档值 `0.7`；不要同时在实验中改动阻尼。
- `ARM_2`：不调用 `SetCartesianImpedance(ARM_2, ...)`。

### 7.2 为什么不能每 1 ms 调用 SetCartesianImpedance

RDK v2.1 将 `SetCartesianImpedance()` 定义为 **Blocking**。如果在 1 kHz RT 发送线程里持续调用，可能导致命令流延迟、周期超时、掉包或运动突变。

### 7.3 推荐的阶段化落地策略

1. **默认 `enable_stiffness_ramp = false`**：先仅实现轨迹软停止，确保 Arm2 Hold 和整个双臂 RT 流正确。
2. 向当前设备对应的 Flexiv RDK 版本/厂家确认：是否支持在双臂 `StreamCartesianMotionForce` 运行期间，**并发、非 RT 地**调用 `SetCartesianImpedance(ARM_1, ...)`，是否对同一 `Robot` 实例线程安全，以及允许的变更速率。
3. 只有获得确认后，设计一个**不运行在 RT 回调里**的刚度更新工作器；目标刚度由 RT 状态机发布为快照，工作器按经过验证的低频步进逐次执行阻塞 API，并记录每次完成时间。**不要**在 RT 线程内等待工作器，不要使用会阻塞 RT 发包的共享互斥锁。
4. 必须验证这一接口和 RT 发送同时工作不会破坏 1 kHz 数据流；若无法验证，**禁用在线刚度渐增**，退回纯轨迹 Soft Stop，不得声称已经实现了逐渐增加刚度。
5. 开启后，初期设置较小的 `K_x,f - K_x,0` 并分级测试，监控实际 TCP 速度、接触力、Arm1 末端的突跳与 Arm2 漂移。
6. 记录每次刚度命令的**请求时间、完成时间、实际请求值**；由于调用阻塞，实际刚度变化可能是**阶梯状**，不是数学意义上的连续二次曲线。

**工程判断**：仅增大刚度不能保证软停止；主要停速机制始终是轨迹与机器人内部阻尼控制。特别是起始刚度接近零的 Transparent 模式，轨迹跟踪并不必然有效。不可在接触人员或接近障碍物时将刚度提高当成安全制动手段。

---

## 8. 第八步：日志与可视化验证

### 8.1 建议 CSV 字段

最少记录：

```text
time_s,stop_sign,arm1_state,
arm1_px,arm1_py,arm1_pz,arm1_vx,arm1_vy,arm1_vz,
arm1_wx,arm1_wy,arm1_wz,
arm1_target_px,arm1_target_py,arm1_target_pz,
arm1_target_vx,arm1_target_vy,arm1_target_vz,
arm1_target_ax,arm1_target_ay,arm1_target_az,
arm1_joint_q,arm1_joint_dq,
arm2_px,arm2_py,arm2_pz,arm2_joint_q,arm2_joint_dq,
arm2_hold_position_error,
stop_elapsed_s,requested_Kx_arm1,impedance_request_latency_ms,
rt_cycle_us,stream_status,fault_status
```

关节数组建议拆成单独列 `q1..q7`、`dq1..dq7`；若 RDK 当前版本数据结构不同，按实际结构取值。同步写日志不要阻塞 RT 循环，应缓存后离线写盘。

### 8.2 必须画出的 6 张图

1. Arm1 目标/实际平移位置 `p(t)`，检查二次曲线与跟踪误差。
2. Arm1 目标/实际线速度 `v(t)`，检查目标是否线性下降和实际是否无反向冲击。
3. Arm1 实测关节速度 `dq_i(t)`（7 个关节），检查是否发生个别关节突跳。
4. Arm1 目标刚度 `K_x(t)` 与每次 API 生效请求时间（仅第二阶段）。
5. Arm2 实测 TCP 位置与启动 Hold 位姿之差，以及关节位置误差。
6. RT 循环间隔（包括最大值、p95/p99、丢周期和异常间隔）。

### 8.3 建议验收指标（项目自定，**不是厂家安全认证值**）

| 测项 | 第一轮建议检查项 | 未通过时 |
|---|---|---|
| Arm1 目标平移轨迹 | 位置符合二次公式；目标速度单调降至零 | 检查 `s`、时钟、触发锁存 |
| Arm1 实测运动 | 无速度突跳；减速后实际速度低于自定阈值 | 降低测试速度并检查控制模式、刚度与外力 |
| Arm2 Hold | 无 Stop 前后目标变化，实测位姿漂移不异常 | 检查是否错误重采样、丢发、变更 Arm2 刚度 |
| 实时稳定性 | 满足现有项目已验收的 RT 抖动/超时要求 | 禁用刚度工作器，排查阻塞调用 |
| 安全逻辑 | 硬件安全停止始终独立有效 | 停止试验，恢复原始安全行为 |
| 恢复路径 | 不会自动恢复 Transparent；只有显式复位 | 修正状态机和 stop 标志锁存 |

如尚无既定容差，可在**不运行真实运动**时先用目标容差评估（如 Arm2 平移漂移小于 `1 mm` 作为研发候选指标）；真实容差必须结合载荷、结构、当前控制器表现和风险评估制定，不能把例子直接当为安全阈值。

---

## 9. 第九步：逐级验证顺序（禁止跳级）

### Test A：纯软件计算 / 离线单元测试（零真机运动）

- [ ] 输入给定 `p0,v0,T`，检查 `t=0` 时 `p=p0`、`v=v0`；`t=T` 时 `p=p0+v0*T/2`、`v=0`。
- [ ] 验证 `t>T` 后命令恒定 Hold。
- [ ] 速度为零时不产生位移；正负速度均能正确停止。
- [ ] Stop Sign 连续重复 100 次，状态只从 Transparent 跳转一次。
- [ ] 模拟时间抖动、丢周期、异常速度和超限距离，验证无 `NaN`、越界和循环卡死。
- [ ] Arm2 的参考指令在所有测试中逐字节/逐字段恒定（除时间戳等不相关字段）。

### Test B：只读/无接触、低风险工况

- [ ] 启动程序，Arm2 Hold，Arm1 尽量从低速/静止开始。
- [ ] 验证 Stop Sign 事件正确进入 RT 线程而不调用 `robot.Stop()`。
- [ ] **首先关闭刚度渐增**，仅检查保持发送的命令和 Arm2 是否保持。
- [ ] 验证实测速度和位姿日志确实来自 Arm1，而不是 Arm2 或全系统错误索引。

### Test C：Arm1 低速自由空间减速

- [ ] 两臂远离人体、工作空间边界和相互碰撞区域，移除工具和工件。
- [ ] 给 Arm1 一个很小的可测初速，确认停止距离足够。
- [ ] 触发普通 Stop Sign，观测 Arm1 二次目标与实际停速；Arm2 持续 Hold。
- [ ] 重复至少若干次不同方向和不同小速度，检查一致性。
- [ ] 暂不加入变刚度，以明确区分轨迹问题与刚度问题。

### Test D：第二阶段变刚度实验

- [ ] 先获得 API 并发和实时性验证结果。
- [ ] 仅 Arm1 小幅、低频刚度变化；Arm2 保持原配置。
- [ ] 监控 RT 抖动、TCP/关节速度、接触力；发现任何意外抖动或阻塞立即禁用这一实验功能，按既有安全规程处理。
- [ ] 多次复现并保存日志，确认加入刚度更新后比纯轨迹版本没有引入额外风险或异常。

### Test E：回归测试

- [ ] 无 Stop Sign 时仍可正常 Transparent。
- [ ] Stop Sign 后 Arm1 进入 Soft Stop 再 Hold。
- [ ] Arm2 在全部状态持续 Hold。
- [ ] 原有故障处理、保护停止、断网保护和受控退出行为仍有效。
- [ ] 反复 Stop Sign 不重置轨迹；Hold 后不能未经复位自动恢复。
- [ ] 代码编译无警告或至少没有新增严重警告；日志有足够证据支持结论。

---

## 10. 第十步：编译与运行流程

**先确认 CMake 工程路径与目标名**；以下是典型 CMake 命令，不代表源文件肯定直接属于某个可执行目标：

```bash
# 在实际项目根目录
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 4
```

若项目已自定义 `CMakeLists.txt` 或使用现成 Flexiv 示例构建系统，应先按原方式编译；不要为 Soft Stop 重构整个工程。

运行前必须：

1. 由操作员确认 RDK/固件兼容性、RT 网络链路、机型及双臂顺序。
2. 检查 Arm2 Hold 初始目标、Arm1 工具和 TCP、负载、限速、限位、碰撞空间与安全停止装置。
3. 首先运行**不驱动真机的轨迹单元测试**，再按 Test B → C → D 逐级验证。
4. 遇到异常，按既有安全程序处置，不要依赖 RT 应用程序能在异常时继续执行柔和减速。

---

## 11. 修改任务交付清单

工程师 / Agent 应交付：

1. **改动后的 C++ 源文件**：默认仅启用 Arm1 二次轨迹软停止；Arm2 保留固定 Hold；变刚度由显式开关控制且默认关闭。
2. **最小代码差异**：保留原透明控制、原 Stop Sign 输入源、原双臂实时发送、原安全处理，不重写不相关内容。
3. **单元测试或可复现离线验证脚本**：覆盖 Stop Sign 锁存、二次轨迹边界、Arm2 恒定 Hold、时间异常。
4. **CSV 日志与绘图脚本**：可对照第 8 节绘图。
5. **README / 测试记录**：列明使用的 RDK 版本、固件、配置、`T` 的计算规则、是否启用刚度渐增、测试结果及遗留问题。
6. **可回滚版本**：一旦新版本编译、运行或真机验证失败，可恢复原 `arm1_transparent_arm2_hold_rt_cartesian.cpp`。

### 给代码修改 Agent 的执行指令（可直接复制）

```text
请修改当前工程的 arm1_transparent_arm2_hold_rt_cartesian.cpp，严格按这份
arm1_soft_stop_implementation_instructions.md 执行。

先阅读源文件和本机 RDK 头文件，说明当前 Stop Sign 入口、Arm1 Transparent、
Arm2 Hold、模式和唯一 RT 发送调用的位置。禁止凭空假设变量或函数名称。

第一阶段只实现：
1. Arm1: TRANSPARENT -> SOFT_STOP -> HOLD 状态机；
2. 收到普通 Stop Sign 后锁存 Arm1 的实测 TCP pose/twist；
3. 根据合法停止时间 T，连续生成二次位置、线性速度、恒定减速度的 RT 参考轨迹；
4. Arm2 从启动起固定 Hold，并在每个周期与 Arm1 一同发送；
5. Stop Sign 事件不调用 robot.Stop()、不退出 RT 循环、不修改 Arm2 参数；
6. 保留原安全急停/保护停止/故障处理；
7. 添加日志、单元测试、编译检查，给出改动 diff。

第二阶段单独处理 Arm1 笛卡尔刚度 K_x 渐增，不是关节 K_q。
SetCartesianImpedance 在 v2.1 为 blocking API，默认禁用在线渐增；
先确认当前 RDK 与设备支持异步并发调用且不会影响 1 kHz RT 流。
确认前不许在 RT 循环中调用它，也不许擅自切换为 Joint Impedance。

不要在没有真机和源代码的情况下宣称该改动已经验证通过。
```

---

## 12. 技术来源（Flexiv 官方文档）

- Flexiv RDK v2.x：控制模式与 1 kHz 频率说明：  
  https://flexiv.us/software/rdk/manual/v2.x/control_modes.html
- Flexiv RDK v2.1：`Robot` API，含 `StreamCartesianMotionForce`、`SetCartesianImpedance`、`SetJointImpedance`：  
  https://www.flexiv.com/software/rdk/api/v2.1/classflexiv_1_1rdk_1_1_robot.html
- Flexiv RDK v2.1：`RtCartesianCmd` 成员与参数顺序：  
  https://www.flexiv.com/software/rdk/api/v2.1/structflexiv_1_1rdk_1_1_rt_cartesian_cmd-members.html
- Flexiv RDK v2.1：`data.hpp`、`JointGroup`：  
  https://www.flexiv.com/software/rdk/api/v2.1/data_8hpp.html
- Flexiv RDK v2.x：常见问题、`Robot::states()`：  
  https://www.flexiv.com/software/rdk/manual/v2.x/faq.html

**文档状态**：根据用户描述和官方 RDK v2.1 API 编写。由于尚未获得本地 `arm1_transparent_arm2_hold_rt_cartesian.cpp` 源文件，这是一份具体实施与验收规范，不是已经针对原始代码完成的修改。
