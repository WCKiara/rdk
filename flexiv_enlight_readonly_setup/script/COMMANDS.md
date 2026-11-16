# 常用命令

下面的命令都在项目根目录运行：

```bash
cd ~/rdk/flexiv_enlight_readonly_setup
```

**一次只粘贴一条运行命令。** 如果一次粘贴好几行，按 Ctrl+C 停掉前一条之后，下一条会马上自动启动，机器人也会接着动。

## 编译

第一次编译，或者改了 `script/CMakeLists.txt` 之后：

```bash
cmake -S script -B .build/script && cmake --build .build/script
```

只改了 `.cpp` 里的参数：

```bash
cmake --build .build/script
```

用命令行选项临时覆盖参数的话，不用重新编译。

## 运行 ARM_1 透明手拖、ARM_2 不动

两个程序都要用 `sudo` 运行，因为实时调度器需要 root。两个程序的流程一样：

1. **① 回正**：ARM_1 自己动到 home 姿态。开始前先确认路径上没有东西。加 `--no-home` 可以跳过这一步。
2. **② 受力回 0**：力/力矩传感器清零，这期间不要碰 ARM_1。
3. **③ 手拖**：进入实时模式。看到"③ 实时模式 … 已开始"之后就可以拖 ARM_1，按 Ctrl+C 结束。

ARM_2 全程保持不动。运行时手放在 E-stop 旁边。

### 关节空间版

```bash
sudo .build/script/arm1_transparent_arm2_hold_rt Enlight-LL-062034
```

| 选项 | 作用 |
|---|---|
| `--damping D1 … D7` | J1–J7 关节阻尼 [Nm·s/rad]，越小越透明。阻尼为 0 时，没人推臂会越漂越快。例如 `--damping 0 0 0 0 0 0 0` |
| `--friction-comp-scale S` | 摩擦补偿百分比，范围 0–100，100 最透明 |
| `--no-home` | 跳过回正 |
| `--clear-fault` | 启动时有故障先尝试清除 |

### 笛卡尔空间版

```bash
sudo .build/script/arm1_transparent_arm2_hold_rt_cartesian Enlight-LL-062034
```

| 选项 | 作用 |
|---|---|
| `--passive` | 被动力控：更透明，但没人推时会自己漂。不加这个选项就是主动力控 |
| `--force-axis AX AY AZ` | X Y Z 哪几个方向能拖，1 能拖，0 锁住。例如 `--force-axis 1 1 0` |
| `--rot-stiffness R1 R2 R3` | Rx Ry Rz 转动刚度 [Nm/rad]，0 是自由转动。例如 `--rot-stiffness 5 5 5` |
| `--max-linear-vel V1 V2 V3` | X Y Z 末端线速度上限 [m/s]，范围 0.005–2，只在主动力控下有效 |
| `--null-space-tracking W` | 肘部被拉回原姿态的权重，范围 0.1–1，越小肘部越自由 |
| `--no-home` | 跳过回正 |
| `--clear-fault` | 启动时有故障先尝试清除 |
| `--stop-di N` | 数字输入 DI N（0–19）变高也算 Stop Sign，见下方"ARM_1 软停止" |
| `--stop-stiffness-scale S` | 软停止后 ARM_1 的刚度占 K_x_nom 的比例，范围 (0, 1]，默认 1 |
| `--stiffness-ramp` | 软停止时刚度按 (t/T)² 渐增。第二阶段实验，默认关 |
| `--log FILE.csv` | ③ 期间每个实时周期记一行（约 1 MB/s）。文件已经存在时程序不运行 |

选项可以组合使用，例如：

```bash
sudo .build/script/arm1_transparent_arm2_hold_rt_cartesian Enlight-LL-062034 --passive --rot-stiffness 5 5 5
```

### ARM_1 软停止（Stop Sign，只有笛卡尔空间版有）

③ 开始后，用下面任一种方式发普通 Stop Sign：

- 在运行程序的终端里按 Enter；
- 另开一个终端运行 `sudo kill -USR1 <pid>`，pid 在"发 Stop Sign"那一行里。视觉等别的程序也用这种方式；
- 加了 `--stop-di N` 时，让 DI N 变高。

收到之后，ARM_1 从当时的速度按二次位置曲线减速到 0，然后 HOLD。ARM_2 照旧 hold，程序也不退出。

- 要恢复透明手拖：Ctrl+C 结束程序，再重新运行。
- 速度再大也照样软停止，只是减速时间和停止距离更长：停止距离约为 速度² ÷ (2 × 0.5 m/s²)，0.5 m/s 时约 25 cm。要停得更快就调大 `kStopMaxLinearDecel`。
- Stop Sign 不是急停，急停用 E-stop。

记日志：

```bash
sudo .build/script/arm1_transparent_arm2_hold_rt_cartesian Enlight-LL-062034 --log logs/soft_stop_$(date +%Y%m%d_%H%M%S).csv
```

画图前先装一次 matplotlib（`.venv/bin/pip install matplotlib`），然后把文件名换成实际的：

```bash
.venv/bin/python script/plot_soft_stop.py logs/soft_stop_20261115_120000.csv
```

离线单元测试不连机器人：

```bash
cmake --build .build/script --target arm1_soft_stop_test && .build/script/arm1_soft_stop_test
```

参数、原理和真机测试顺序见 [README_arm1_soft_stop.md](README_arm1_soft_stop.md)。

两个程序都可以加 `--help` 查看全部选项。

### 改默认值

要长期改默认值，就在源码里找"透明度参数：在这里调"那一段，改完重新编译：

- 关节版 `arm1_transparent_arm2_hold_rt.cpp`：`kFloatingDamping`、`kFrictionCompScale`
- 笛卡尔版 `arm1_transparent_arm2_hold_rt_cartesian.cpp`：`kPassiveForceControl`、`kForceAxis`、`kRotStiffness`、`kMaxLinearVel`、`kNullSpaceRefTracking`、`kForceCtrlFrame`
- 笛卡尔版的软停止：同一个文件里"ARM_1 软停止：在这里调"一段，例如 `kStopMaxLinearDecel`、`kStopMaxAngularDecel`、`kStopMinTime`、`kStopStiffnessScale`

### 停下来之后

输出最后一行如果是"已停止：……"，后面写的就是停机原因。没有这一行，说明是 Ctrl+C 正常结束的。

## 运行 RT 六维笛卡尔浮动（intermediate8）

ARM_1 末端每个轴能不能拖，由源码里的 `kLockX`、`kLockY`、`kLockZ`、`kLockRx`、`kLockRy`、`kLockRz` 分别决定：`true` 锁定，`false` 能拖。默认 X Y Z 能拖、Rx Ry Rz 锁定，改了要重新编译。ARM_2 全程不动。流程和上面两个程序一样：① 回正 → ② 受力回 0 → ③ 实时模式。每次给 ARM_1 发指令都同时给 ARM_2 发 hold；③ 从开始到 Ctrl+C 一直是实时模式，不切换。

③ 开始后 ARM_1 先用名义刚度保持约 0.1 秒，然后降刚度。看到"ARM_1 已浮动"之后就可以拖。降刚度那一下要扶住 ARM_1。浮动时 ARM_1 的目标跟随实测位姿：拖到哪里，松手就停在哪里，不会弹回起点。`--scale` 越小，阻力越小。

这个程序不设速度、位置限位类的停机检查：拖得多快、多远都不会被程序停下，ARM_2 偏离 hold 位置也不停机（结束时照样打印偏离）。只有 E-stop 和控制器自带的保护能停住机器人，拖动时手放在 E-stop 旁边。拖动期间全程是实时模式 `RT_CARTESIAN_MOTION_FORCE`，结束时会打印一行控制模式检查结果确认这一点。

第一次编译前先重新配置（`CMakeLists.txt` 新加了这个目标）。按 `--scale` 从高到低（阻力从大到小），一次跑一条：

```bash
sudo .build/script/intermediate8_realtime_cartesian_floating Enlight-LL-062034 --scale=0.10
```

```bash
sudo .build/script/intermediate8_realtime_cartesian_floating Enlight-LL-062034 --scale=0.00
```

中间依次试 `--scale=0.05`、`--scale=0.02`。

| 选项 | 作用 |
|---|---|
| `--scale=S` | ARM_1 能拖的轴的刚度占名义刚度的比例，范围 0–1，默认 0（锁定的轴不受影响）。目标跟随实测位置，所以它决定的是阻尼（阻尼 ∝ √比例），越小越透明。0 是无阻尼浮动，没人推会越漂越快。1 个数 = 6 个轴都用它；逗号分开的 6 个数 = X,Y,Z,Rx,Ry,Rz 各一个，例如 `--scale=0.05,0.05,0.02,0,0,0` |
| `--zeta=Z` | ARM_1 能拖的轴的阻尼比，范围 0.3–0.8，默认 0.7（RDK 默认；锁定的轴固定 0.7）。只能把阻尼调到 0.7 时的 0.43–1.14 倍，刚度为 0 的轴不起作用。1 个数或 6 个数，同 `--scale`。`--passive`、`--active` 时对 X Y Z 不起作用 |
| `--nullspace=W` | 肘部被拉回起始姿态的权重，范围 0.1–1，默认 0.5 |
| `--active` | 能拖的 X Y Z 轴做主动零力控制。**2026-11-06 实测比不开重得多**：平移阻尼 141–202 N·s/m，不开时 7–10 N·s/m，是主动力控多出来的阻尼，不要用来变轻。Rx Ry Rz 照样位置控制，不需要六维力传感器。末端力估算有偏差时没人推也会自己走，最快到 `kActiveMaxLinearVel`（默认 1.0 m/s）。不能和 `--passive` 同时用 |
| `--nullspace-follow` | 肘部参考姿态每 10 ms 跟随实测关节位置，肘部不再被拉回进入 ③ 时的姿态（J1 J2 J3 一起转、末端不动的那种动作变轻）。没人推时肘部可能自己漂 |
| `--log=FILE.csv` | 浮动期间 100 Hz 记录 ARM_1 的末端位姿、速度、外力和关节状态。文件已经存在时程序不运行，不会覆盖旧记录 |
| `--passive` | 能拖的 X Y Z 做被动零力控制，这几个轴上 `--scale`、`--zeta` 不起作用；Rx Ry Rz 不做力控，能拖的 Rx Ry Rz 照样按 `--scale`、`--zeta` 浮动（`--scale=0` 时零刚度自由转），不需要六维力传感器。锁定的轴照样锁定。不能和 `--active` 同时用 |
| `--no-home` | 跳过回正 |
| `--clear-fault` | 启动时有故障先尝试清除 |

要长期改默认值，改源码里"透明度参数：在这里调"一段的 `kStiffnessScale`、`kDampingRatio`、`kNullSpaceRefTracking`、`kPassiveForceControl`、`kActiveForceControl`、`kActiveMaxLinearVel`、`kNullSpaceFollow`、`kLockX` … `kLockRz`。停下来之后怎么看原因，同上面的"停下来之后"。详细说明见 [README_cartesian_floating.md](README_cartesian_floating.md)。

### 量化对比透明度

每次运行加 `--log=` 记一份日志，文件名写上这次的参数和日期时间。下面的命令用 `$(date +%Y%m%d_%H%M%S)` 自动加上日期时间，每次都是新文件，不会覆盖以前的记录；文件已经存在时程序也会直接退出。**每次的拖法要一样**，数字才能比：

1. 看到"ARM_1 已浮动"后，先手离开 2 秒（手悬在旁边随时能扶住）。脚本用日志开头这 2 秒算松手漂移。
2. 沿一个方向慢慢来回推 10 秒左右，有慢有快。
3. 如果转动轴放开了，再慢慢转几下工具。
4. Ctrl+C 结束。

```bash
sudo .build/script/intermediate8_realtime_cartesian_floating Enlight-LL-062034 --scale=0.10 --log=logs/i8_s010_$(date +%Y%m%d_%H%M%S).csv
```

```bash
sudo .build/script/intermediate8_realtime_cartesian_floating Enlight-LL-062034 --scale=0.00 --log=logs/i8_s000_$(date +%Y%m%d_%H%M%S).csv
```

跑完之后，把几份日志放在一起比（不需要 sudo，不连机器人）：

```bash
.venv/bin/python script/compare_transparency.py logs/i8_s010_2026*.csv logs/i8_s000_2026*.csv
```

`2026*` 会匹配所有带日期的同名日志。只想比某一次，就写完整文件名，程序结束时会打印"日志已写入 …"。

最后的对比表里，平移阻尼、转动阻尼、平移阻力、起动力越小越透明；松手最大漂移越小越稳。松手漂移只看日志开头 2 秒，要换成别的秒数就加 `--hands-off=3`。

## 清除故障

```bash
.build/rdk-v2.1-examples/basics2_clear_fault Enlight-LL-062034
```

这个程序不需要 `sudo`，也不会让机器人动。清除之前先松开 E-stop，并排除造成故障的原因。严重故障可能要等最多 30 秒，有的必须重启机器人电源才能清掉。

要重新编译这个官方例子：

```bash
cmake --build .build/rdk-v2.1-examples --target basics2_clear_fault
```
