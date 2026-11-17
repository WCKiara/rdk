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

## 运行 RT 笛卡尔导纳控制（intermediate8）

ARM_1 用导纳控制手拖，ARM_2 全程不动。流程和上面两个程序一样：① 回正 → ② 受力回 0 → ③ 实时模式。每次给 ARM_1 发指令都同时给 ARM_2 发 hold；③ 从开始到 Ctrl+C 一直是实时模式，不切换。

- ARM_1 一直是名义刚度的位置控制，不降刚度、不开力控。程序每 1 ms 读末端外力，按虚拟质量 `--mass`、虚拟阻尼 `--damping` 算出目标往哪走，所以手感只由这两个数决定。
- ③ 开始后 ARM_1 先锁在当前位姿约 0.1 秒。看到"ARM_1 导纳控制已开始"之后就可以拖：推它就走，松手后减速停下，不弹回。
- 一直用同样的力推，最后匀速：速度 =（力 − 0.5 N）÷ 阻尼。松手后还会滑一段，大约 速度 × 质量 ÷ 阻尼 那么远。
- 小于 0.5 N（力矩 0.05 N·m）的外力当作 0，这是起动要用的最小力。② 清零不准时，末端可能自己慢慢走，拖动时手放在旁边。
- 外力是由关节力矩估算的：碰肘部等末端以外的地方，末端也会动。
- 默认 6 个轴都能拖。每个轴锁不锁，由源码里的 `kLockX`、`kLockY`、`kLockZ`、`kLockRx`、`kLockRy`、`kLockRz` 决定：`true` 锁定，`false` 能拖，改了要重新编译。

这个程序不设速度、位置限位类的停机检查：拖得多快、多远都不会被程序停下，ARM_2 偏离 hold 位置也不停机（结束时照样打印偏离）。

- 只有一个例外：ARM_1 跟不上导纳目标超过 10 mm 或 0.03 rad（被挡住、到关节极限、被控制器限速）时整机停。结束时打印导纳期间的最大跟踪误差。
- 除此之外只有 E-stop 和控制器自带的保护能停住机器人，拖动时手放在 E-stop 旁边。
- 拖动期间全程是实时模式 `RT_CARTESIAN_MOTION_FORCE`，结束时会打印一行控制模式检查结果确认这一点。

先用默认值：

```bash
sudo .build/script/intermediate8_realtime_cartesian_floating Enlight-LL-062034
```

觉得抖或者太飘，就把质量、阻尼调大一些。1 个数只改平移 X Y Z，转动保持默认的 0.15、0.3，扭末端时 J7 照样转得动，例如：

```bash
sudo .build/script/intermediate8_realtime_cartesian_floating Enlight-LL-062034 --mass=8 --damping=15
```

| 选项 | 作用 |
|---|---|
| `--mass=M` | 虚拟质量，必须 > 0。X Y Z 单位 kg，Rx Ry Rz 单位 kg·m²，默认 5,5,5,0.15,0.15,0.15。越小起动、变向越轻，太小会抖。1 个数 = 只改 X Y Z，Rx Ry Rz 保持默认；逗号分开的 6 个数 = X,Y,Z,Rx,Ry,Rz 各一个，例如 `--mass=5,5,5,0.3,0.3,0.3` |
| `--damping=D` | 虚拟阻尼，必须 > 0。X Y Z 单位 N·s/m，Rx Ry Rz 单位 N·m·s/rad，默认 10,10,10,0.3,0.3,0.3。越小越轻、越快，松手后也滑得越远。1 个数（只改 X Y Z）或 6 个数，同 `--mass` |
| `--nullspace=W` | 肘部被拉回起始姿态的权重，范围 0.1–1，默认 0.1 |
| `--nullspace-follow` | 肘部参考姿态每 10 ms 跟随实测关节位置，肘部不再被拉回进入 ③ 时的姿态（J1 J2 J3 一起转、末端不动的那种动作变轻）。没人推时肘部可能自己漂 |
| `--log=FILE.csv` | 导纳期间 100 Hz 记录 ARM_1 的末端位姿、速度、外力和关节状态。文件已经存在时程序不运行，不会覆盖旧记录 |
| `--no-home` | 跳过回正 |
| `--clear-fault` | 启动时有故障先尝试清除 |

以前的 `--scale`、`--zeta`、`--passive`、`--active` 都去掉了：导纳不降刚度，也不用力控。以前的降刚度浮动版本保留为 `intermediate8_realtime_cartesian_floating_original.cpp`，不编译。

要长期改默认值，改源码里"导纳参数：在这里调"那一段：`kAdmittanceMass`、`kAdmittanceDamping`、`kForceDeadband`、`kMomentDeadband`、`kNullSpaceRefTracking`、`kNullSpaceFollow`、`kLockX` … `kLockRz`。其中死区和锁定没有命令行参数。停下来之后怎么看原因，同上面的"停下来之后"。详细说明见 [README_cartesian_floating.md](README_cartesian_floating.md)。

### 量化对比透明度

每次运行加 `--log=` 记一份日志，文件名写上这次的参数和日期时间。下面的命令用 `$(date +%Y%m%d_%H%M%S)` 自动加上日期时间，每次都是新文件，不会覆盖以前的记录；文件已经存在时程序也会直接退出。**每次的拖法要一样**，数字才能比：

1. 看到"ARM_1 导纳控制已开始"后，先手离开 2 秒（手悬在旁边随时能扶住）。脚本用日志开头这 2 秒算松手漂移。
2. 沿一个方向慢慢来回推 10 秒左右，有慢有快。
3. 如果转动轴放开了，再慢慢转几下工具。
4. Ctrl+C 结束。

```bash
sudo .build/script/intermediate8_realtime_cartesian_floating Enlight-LL-062034 --log=logs/i8_adm_m5_d10_r015_03_$(date +%Y%m%d_%H%M%S).csv
```

```bash
sudo .build/script/intermediate8_realtime_cartesian_floating Enlight-LL-062034 --mass=8 --damping=15 --log=logs/i8_adm_m8_d15_$(date +%Y%m%d_%H%M%S).csv
```

跑完之后，把几份日志放在一起比（不需要 sudo，不连机器人）：

```bash
.venv/bin/python script/compare_transparency.py logs/i8_adm_m5_d10_r015_03_2026*.csv logs/i8_adm_m8_d15_2026*.csv
```

`2026*` 会匹配所有带日期的同名日志。只想比某一次，就写完整文件名，程序结束时会打印"日志已写入 …"。

最后的对比表里：

- 平移阻尼、转动阻尼、平移阻力、起动力越小越透明；松手最大漂移越小越稳。
- 导纳下拟合出的阻尼应该接近 `--damping`，惯量项接近 `--mass`，常数接近力死区 0.5 N。差得多，说明 ARM_1 没跟上导纳目标，或者外力估算有偏差。
- 以前降刚度浮动的日志（`logs/i8_ns010*`、`logs/i8_passive*` 等）也可以一起比。
- 松手漂移只看日志开头 2 秒，要换成别的秒数就加 `--hands-off=3`。

## 清除故障

```bash
.build/rdk-v2.1-examples/basics2_clear_fault Enlight-LL-062034
```

这个程序不需要 `sudo`，也不会让机器人动。清除之前先松开 E-stop，并排除造成故障的原因。严重故障可能要等最多 30 秒，有的必须重启机器人电源才能清掉。

要重新编译这个官方例子：

```bash
cmake --build .build/rdk-v2.1-examples --target basics2_clear_fault
```
