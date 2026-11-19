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

**注意：** 11-16 真机用默认参数进入 ③ 的 4 次（`logs/soft_stop_B_*`），都是没人碰 ARM_1，③ 开始后 0.6–6 秒它就自己转起来，越转越快。程序现在不查 ARM_1 速度，一直转到姿态离目标 90° 时机器人故障停机（301005 Orientation error exceeds 90 degrees），之后要先清除故障（见最后的"清除故障"，或者下次运行加 `--clear-fault`）。运行时手一直放在 E-stop 上。记录见 [README_arm1_soft_stop.md](README_arm1_soft_stop.md) 第 8 节。

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

画图前先装一次 matplotlib（`.venv/bin/pip install matplotlib`），然后把文件名换成实际的。不装 matplotlib、只看数字就加 `--no-plots`：

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

程序打印的最后一行如果是"已停止：……"，后面写的就是停机原因。它后面可能还有 RDK 自己打印的几行（带 `[info]`、`[error]`），不用管。

- 停机原因是"机器人故障"时，看 RDK 打印的 `[error] [Event Log] [故障码] …` 那一行，例如 `[301005] Orientation error exceeds 90 degrees.`。这时通常还有一行"Stop() 没有执行：…"，要先按下面的"清除故障"清掉才能再运行。
- 没有"已停止"这一行，说明是 Ctrl+C 正常结束的。

## 运行 RT 笛卡尔导纳控制（intermediate8）

ARM_1 用导纳控制手拖，ARM_2 全程不动。流程和上面两个程序一样：① 回正 → ② 受力回 0 → ③ 实时模式。每次给 ARM_1 发指令都同时给 ARM_2 发 hold；③ 从开始到 Ctrl+C 一直是实时模式，不切换。

- ARM_1 一直是名义刚度的位置控制，不降刚度、不开力控。程序每 1 ms 读末端外力，按虚拟质量 `--mass`、虚拟阻尼 `--damping` 算出目标往哪走，所以手感只由这两个数决定。
- ③ 开始后 ARM_1 先锁在当前位姿约 0.1 秒。看到"ARM_1 导纳控制已开始"之后就可以拖：推它就走，松手后减速停下，不弹回。
- 一直用同样的力推，最后匀速：速度 =（力 − 0.5 N）÷ 阻尼。松手后还会滑一段，大约 速度 × 质量 ÷ 阻尼 那么远。
- 小于 0.5 N（力矩 0.05 N·m）的外力当作 0，这是起动要用的最小力。② 清零不准时，末端可能自己慢慢走，拖动时手放在旁边。
- 外力是由关节力矩估算的：碰肘部等末端以外的地方，末端也会动。
- 默认 6 个轴都能拖。每个轴锁不锁，由源码里的 `kLockX`、`kLockY`、`kLockZ`、`kLockRx`、`kLockRy`、`kLockRz` 决定：`true` 锁定，`false` 能拖，改了要重新编译。

安全限制（11-18 加的，只在导纳期间起作用）：速度、角速度有上限，只限速不停机；实测值超过停机线就整机停。

| 量 | 上限（只限速） | 停机线（实测超过就停） |
|---|---|---|
| 末端速度 | 0.25 m/s（`--max-linear-speed`） | 0.5 m/s（上限的 2 倍） |
| 末端角速度 | 0.5 rad/s（`--max-angular-speed`） | 1 rad/s（上限的 2 倍） |
| 末端外力 | 无 | 20 N（`--stop-force`） |
| 末端外力矩 | 无 | 3 N·m（`--stop-moment`） |

- 手再用力也不会比上限快；顶着上限使劲推，外力超过 20 N 也会停机。
- 停机时最后一行是"已停止：ARM_1 末端外力 … 超过安全停机线 …"。结束时打印实测最大值和上限起作用的时间，按实测再调。
- 原理和数据依据见 [README_cartesian_floating.md](README_cartesian_floating.md) 的"安全限制"。

除了安全限制，不设位置限位：拖得多远都不会被程序停下，ARM_2 偏离 hold 位置也不停机（结束时照样打印偏离）。

- 另外，ARM_1 跟不上导纳目标超过 10 mm 或 0.03 rad（被挡住、到关节极限、被控制器限速）时整机停。结束时打印导纳期间的最大跟踪误差。
- 除此之外只有 E-stop 和控制器自带的保护能停住机器人，拖动时手放在 E-stop 旁边。
- 拖动期间全程是实时模式 `RT_CARTESIAN_MOTION_FORCE`，结束时会打印一行控制模式检查结果确认这一点。

先用默认值：

```bash
sudo .build/script/intermediate8_realtime_cartesian_floating Enlight-LL-062034
```

现在的默认值（阻尼 5,5,5,0.1,0.1,0.1，目标角速度上限 0.5 rad/s）还没上机试过。11-16 真机跑了 5 次的是阻尼 10,10,10,0.3,0.3,0.3（质量和现在一样，那时还没有角速度上限）。一开始摆，就先回到这组：

```bash
sudo .build/script/intermediate8_realtime_cartesian_floating Enlight-LL-062034 --damping=10,10,10,0.3,0.3,0.3
```

觉得抖或者太飘，就把质量、阻尼调大一些。1 个数只改平移 X Y Z，转动保持默认的 0.15、0.1，扭末端时 J7 照样转得动，例如：

```bash
sudo .build/script/intermediate8_realtime_cartesian_floating Enlight-LL-062034 --mass=8 --damping=15
```

| 选项 | 作用 |
|---|---|
| `--mass=M` | 虚拟质量，必须 > 0。X Y Z 单位 kg，Rx Ry Rz 单位 kg·m²，默认 5,5,5,0.15,0.15,0.15。越小起动、变向越轻，太小会抖。1 个数 = 只改 X Y Z，Rx Ry Rz 保持默认；逗号分开的 6 个数 = X,Y,Z,Rx,Ry,Rz 各一个，例如 `--mass=5,5,5,0.3,0.3,0.3` |
| `--damping=D` | 虚拟阻尼，必须 > 0。X Y Z 单位 N·s/m，Rx Ry Rz 单位 N·m·s/rad，默认 5,5,5,0.1,0.1,0.1。越小越轻、越快，松手后也滑得越远。1 个数（只改 X Y Z）或 6 个数，同 `--mass` |
| `--max-linear-speed=V` | ARM_1 目标速度 \|v\| 的上限 [m/s]，必须 > 0，默认 0.25。超过就按比例缩到它、方向不变，只限速、不停机。实测速度超过 2 倍上限就停机 |
| `--max-angular-speed=W` | ARM_1 目标角速度 \|ω\| 的上限 [rad/s]，必须 > 0，默认 0.5。超过就按比例缩到它、方向不变，只限速、不停机。工具朝下时扭末端主要是 J7 在转。实测角速度超过 2 倍上限就停机 |
| `--stop-force=F` | ARM_1 实测外力 \|F\| 的安全停机线 [N]，默认 20，要大于力死区 0.5 N |
| `--stop-moment=M` | ARM_1 实测外力矩 \|M\| 的安全停机线 [N·m]，默认 3，要大于力矩死区 0.05 N·m |
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

文件名里的 `m5_d5` 是平移质量 5、阻尼 5，`r015_01` 是转动质量 0.15、阻尼 0.1，`w05` 是角速度上限 0.5。1 个数的 `--mass`、`--damping` 只改平移，所以第二条的转动部分还是 `r015_01_w05`：

```bash
sudo .build/script/intermediate8_realtime_cartesian_floating Enlight-LL-062034 --log=logs/i8_adm_m5_d5_r015_01_w05_$(date +%Y%m%d_%H%M%S).csv
```

```bash
sudo .build/script/intermediate8_realtime_cartesian_floating Enlight-LL-062034 --mass=8 --damping=15 --log=logs/i8_adm_m8_d15_r015_01_w05_$(date +%Y%m%d_%H%M%S).csv
```

跑完之后，把几份日志放在一起比（不需要 sudo，不连机器人）：

```bash
.venv/bin/python script/compare_transparency.py logs/i8_adm_m5_d5_r015_01_w05_2026*.csv logs/i8_adm_m8_d15_r015_01_w05_2026*.csv
```

`2026*` 会匹配所有带日期的同名日志。只想比某一次，就写完整文件名，程序结束时会打印"日志已写入 …"。

没进入导纳就停了的那次（例如 ①② 期间 Ctrl+C），日志是 0 字节的空文件，`2026*` 也会把它匹配进来，`compare_transparency.py` 读到空文件就报错退出。11-16 的 `logs/i8_adm_m5_d10_r015_03_20261116_180734.csv` 就是这样。比之前先找出空文件，删掉，或者写完整文件名跳过它：

```bash
find logs -name '*.csv' -empty
```

最后的对比表里：

- 平移阻尼、转动阻尼、平移阻力、起动力越小越透明；松手最大漂移越小越稳。
- 导纳下拟合出的阻尼应该接近 `--damping`，惯量项接近 `--mass`，常数接近力死区 0.5 N。差得多，说明 ARM_1 没跟上导纳目标，或者外力估算有偏差。
- 以前降刚度浮动的日志也可以一起比，例如 `logs/i8_ns010*.csv`、`logs/i8_passive_2026*.csv`。要写上 `.csv`，不然会匹配到 `*_terminal.txt`；也不要写 `logs/i8_passive*.csv`，它会匹配到空文件 `i8_passive_.csv`。
- 松手漂移只看日志开头 2 秒，要换成别的秒数就加 `--hands-off=3`。

## 工具负载标定（calibrate_tool_payload）

用 RDK 官方的 `Tool::CalibratePayloadParams` 标定法兰上工具的质量、质心、惯量，和 `suyi Nov12.stl` 算出的参考值对比，确认后写成工具 `suyi_Nov12` 并在该臂启用。不用 `sudo`。

```bash
.build/script/calibrate_tool_payload Enlight-LL-062034
```

称过整个工具（含螺丝、夹具）的话，加 `--weighed-mass=<kg>` 和标定结果对比，它不写入。程序每一步都等你按 Enter：

1. 打印已有工具和两条臂当前的工具，把 ARM_1 切到 Flange。
2. **③ 装着工具标定**：按 Enter 后机器人自己动。RDK 没说双臂时动哪条臂，所以两条臂周围都要清空，手放在 E-stop 旁边。
3. **④ 拆下工具再标定一次**：本机没有法兰力传感器，官方建议做这一步。拆工具前可以先拍 E-stop，松开后程序会自己上伺服。输入 `s` 回车跳过。
4. **⑤ 对比表**：每次标定的结果和 STL 参考值。输入 `y` 回车才写入并启用；不写入时恢复原来的工具。

标定期间程序记录两条臂各动了多少。只有 `--arm` 那条臂动、另一条没动，才允许写入。结果都写到 `logs/tool_payload_<日期时间>.json`。

**很轻的工具不用标定。** 末端力由关节力矩估算，11-16 日志里静止时的力噪声中位数 0.02–0.11 N（p90 到 0.31 N）、力矩噪声 0.007–0.023 N·m。`suyi_Nov12` 称重 35.891 g，重力只有 0.35 N，质心力矩 0.012 N·m，和噪声同一个量级，标不出质心。这种工具直接写称重质量 + STL 质心 + STL 惯量，机器人不动：

```bash
.build/script/calibrate_tool_payload Enlight-LL-062034 --no-calibration --weighed-mass=0.035891
```

| 选项 | 作用 |
|---|---|
| `--no-calibration` | 不标定，机器人不动：写 `--weighed-mass` + STL 质心 + STL 形状 × 称重质量 |
| `--arm=2` | 工具装在 ARM_2 上，默认 ARM_1 |
| `--tool-name=NAME` | 工具名，默认 `suyi_Nov12`。已有同名工具就更新它 |
| `--tcp-mm=X,Y,Z` | TCP 位置 [mm]，姿态与法兰相同。不给时：更新已有工具保持原来的 TCP，新建工具 TCP 在法兰中心 |
| `--stl-inertia` | 写入的惯量改用 STL 形状 × 标定质量 |
| `--mounted-only` | 跳过拆下工具后的第二次标定 |
| `--log=FILE.json` | 换记录文件名；文件已经存在时不运行 |
| `--clear-fault` | 启动时有故障先尝试清除 |

## 清除故障

```bash
.build/rdk-v2.1-examples/basics2_clear_fault Enlight-LL-062034
```

这个程序不需要 `sudo`，也不会让机器人动。清除之前先松开 E-stop，并排除造成故障的原因。严重故障可能要等最多 30 秒，有的必须重启机器人电源才能清掉。

还没有 `.build/rdk-v2.1-examples` 时（例如换了电脑），先配置再编译这个官方例子：

```bash
cmake -S third_party/flexiv_rdk/example -B .build/rdk-v2.1-examples -Dflexiv_rdk_DIR="$PWD/.rdk-v2.1/lib/cmake/flexiv_rdk" && cmake --build .build/rdk-v2.1-examples --target basics2_clear_fault
```

已经配置过，只要重新编译：

```bash
cmake --build .build/rdk-v2.1-examples --target basics2_clear_fault
```
