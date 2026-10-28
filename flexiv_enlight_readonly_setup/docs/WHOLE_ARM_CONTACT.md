# 全臂接触估计的读取与记录

RDK 2.1 在 `third_party/flexiv_rdk/include/flexiv/rdk/model.hpp` 提供
`Model.multi_contact_forces()` 和 `Model.multi_contact_positions()`。它们利用关节中的力/力矩传感器，按关节组和连杆顺序返回三维接触力与接触位置估计；两个向量都使用对应连杆的局部坐标系。它们不是触觉皮肤的原始阵列数据。SDK 中没有单独的“开启全臂触觉”函数；本项目的读取路径不调用运动或伺服命令。

项目脚本 `scripts/record_whole_arm_contact.py` 先读取 `robot.states()[JointGroup.ALL]`，用整机 `q`、`dq` 更新 `Model`，然后采集上述两组数据。模型更新需要整机状态，但 CSV 只写入 `--arm` 选择的单臂，默认是 `ARM_1`；它遍历所选臂返回的全部连杆条目。它只建立连接、读取状态和模型结果、写本地 CSV。代码包含逐行中文注释。未连接机械臂时可以先看参数：

```bash
python scripts/record_whole_arm_contact.py --help
```

连接机械臂并使用项目的 RDK 2.1 Python 环境后，从项目根目录运行：

```bash
.venv/bin/python scripts/record_whole_arm_contact.py '<完整机器人序列号>' \
  --arm ARM_1 \
  --rate 20 \
  --duration 60 \
  --output logs/whole_arm_contact_arm1.csv
```

需要在终端同时看每条记录时加 `--show-live`；记录另一条臂使用 `--arm ARM_2`。同名输出文件会被覆盖。所选臂没有数据、连杆力和位置数组为空或长度不一致、数值非有限、连接断开时脚本报错退出。

## CSV 数据说明

CSV 的一行对应一个采样时刻、所选臂中的一个连杆索引；包含主机时间、机器人时间、三维接触力和三维接触位置。`link_index` 是返回数组内从 0 开始的索引；SDK 文档没有给出该索引与 `Model.link_names()` 的逐组对应关系，不能直接给索引附会肩、肘、腕等物理名称。不同连杆的力和位置使用不同局部坐标系，不能直接按 XYZ 分量相加。`force_*_N` 与 `position_*_m` 是脚本采用的常规物理单位列名；该接口的头文件没有单独标注单位，CSV 列名不等于已完成单位或力值标定。

机器人时间戳来自 `states()` 返回的状态；力和位置分别由两个 SDK 访问器读取，接口没有提供三者原子快照的保证。主机与控制器时钟可能未同步，应分别使用各自的相对时间分析采样间隔。

`Model` 需要 RDK Professional 许可，且只有部分机型支持多点接触估计。如果记录结束后 CSV 只有表头，应先核对机型支持情况和许可。`robot.states()[group].tau_ext` 另提供各关节的外力矩估计（Nm），可辅助判断机械臂本体受到外部接触，但它本身不给出接触点位置。

2026-10-20 的原始 60 秒记录中，ARM_1 有 1198 次采样、每次 7 个条目，共 8386 行。旧脚本同时记录的 `ALL`、`ARMS` 均与 `ARM_1` 和 `ARM_2` 的拼接完全相同，因此存在重复记录。该次 ARM_1 的索引 4、6 的力和位置始终为零，其余索引存在较大的持续输出；这些现象本身不能证明实际接触、定位有效性或全部连杆的触碰响应。需核对无接触基线、物理索引映射、单位和模型/传感器配置，再做已知接触的验证。新脚本的单臂过滤和数据检查已离线验证，未为本次修改重新运行硬件采集。

## 离线可视化

`scripts/visualize_whole_arm_contact.py` 只读取已保存的 CSV，不导入机器人 SDK。使用装有 NumPy 和 Matplotlib 的 Python 环境运行：

```bash
python3 scripts/visualize_whole_arm_contact.py logs/<采集文件>.csv
```

它在 CSV 旁生成 `.overview.png`（完整曲线和运动片段放大）、`.components.png`（各连杆的力与位置 XYZ 分量）、两页 `.report.pdf` 和 `.visualization.json` 统计。安装 Plotly 的环境可加 `--interactive`，另生成内嵌绘图库、可离线打开的 `.visualization.html`，支持缩放、悬停读取数值和切换连杆。

可视化前检查各采样的连杆集合一致、数值有限、主机和机器人时间戳递增。存在同名前缀的 `.motion.log` / `.recorder.log` 时，使用 America/Los_Angeles 日志时间与主机时间对齐事件；蓝色阴影表示日志中的 Home 阶段，红虚线表示运动端检测到故障。控制器时间不直接与主机时间混用。约 20 Hz 的采样无法解析几十毫秒内的故障过程。

力曲线保留原始零值。位置图在力和位置均全零的采样处留空，避免把占位零值画成接触点移动；原始 CSV 不修改。全程零输出不等于已证明没有接触，曲线的幅值也不等于已完成传感器标定。

接口依据：[Flexiv RDK v2.1 Model API](https://www.flexiv.com/software/rdk/api/v2.1/classflexiv_1_1rdk_1_1_model.html)。
