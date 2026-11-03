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

选项可以组合使用，例如：

```bash
sudo .build/script/arm1_transparent_arm2_hold_rt_cartesian Enlight-LL-062034 --passive --rot-stiffness 5 5 5
```

两个程序都可以加 `--help` 查看全部选项。

### 改默认值

要长期改默认值，就在源码里找"透明度参数：在这里调"那一段，改完重新编译：

- 关节版 `arm1_transparent_arm2_hold_rt.cpp`：`kFloatingDamping`、`kFrictionCompScale`
- 笛卡尔版 `arm1_transparent_arm2_hold_rt_cartesian.cpp`：`kPassiveForceControl`、`kForceAxis`、`kRotStiffness`、`kMaxLinearVel`、`kNullSpaceRefTracking`、`kForceCtrlFrame`

### 停下来之后

输出最后一行如果是"已停止：……"，后面写的就是停机原因。没有这一行，说明是 Ctrl+C 正常结束的。

## 运行 RT 六维笛卡尔浮动（intermediate8）

ARM_1 末端 X Y Z、Rx Ry Rz 六个方向都能拖；ARM_2 全程不动。流程和上面两个程序一样：① 回正 → ② 受力回 0 → ③ 实时模式。每次给 ARM_1 发指令都同时给 ARM_2 发 hold；③ 从开始到 Ctrl+C 一直是实时模式，不切换。

③ 开始后 ARM_1 先用名义刚度保持约 0.1 秒，然后降刚度。看到"ARM_1 已浮动"之后就可以拖。降刚度那一下要扶住 ARM_1。

第一次编译前先重新配置（`CMakeLists.txt` 新加了这个目标）。按刚度从高到低，一次跑一条：

```bash
sudo .build/script/intermediate8_realtime_cartesian_floating Enlight-LL-062034 --scale=0.10
```

```bash
sudo .build/script/intermediate8_realtime_cartesian_floating Enlight-LL-062034 --scale=0.00
```

中间依次试 `--scale=0.05`、`--scale=0.02`。

| 选项 | 作用 |
|---|---|
| `--scale=S` | ARM_1 六轴刚度占名义刚度的比例，范围 0–1，默认 0.10。0 是零刚度浮动 |
| `--nullspace=W` | 肘部被拉回起始姿态的权重，范围 0.1–1，默认 0.5 |
| `--log=FILE.csv` | 浮动期间 100 Hz 记录 ARM_1 的末端位姿、速度、外力和关节状态 |
| `--passive` | 六轴被动零力。需要六维力传感器，这台没有，会在上伺服之前直接退出 |
| `--no-home` | 跳过回正 |
| `--clear-fault` | 启动时有故障先尝试清除 |

要长期改默认值，改源码里"透明度参数：在这里调"一段的 `kStiffnessScale`、`kNullSpaceRefTracking`、`kPassiveForceControl`。停下来之后怎么看原因，同上面的"停下来之后"。详细说明见 [README_cartesian_floating.md](README_cartesian_floating.md)。

## 清除故障

```bash
.build/rdk-v2.1-examples/basics2_clear_fault Enlight-LL-062034
```

这个程序不需要 `sudo`，也不会让机器人动。清除之前先松开 E-stop，并排除造成故障的原因。严重故障可能要等最多 30 秒，有的必须重启机器人电源才能清掉。

要重新编译这个官方例子：

```bash
cmake --build .build/rdk-v2.1-examples --target basics2_clear_fault
```
