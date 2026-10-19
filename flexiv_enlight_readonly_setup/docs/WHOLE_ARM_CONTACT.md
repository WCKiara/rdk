# 全臂接触估计的读取与记录

RDK 2.1 在 `third_party/flexiv_rdk/include/flexiv/rdk/model.hpp` 提供
`Model.multi_contact_forces()` 和 `Model.multi_contact_positions()`。它们利用关节中的力/力矩传感器，按关节组和连杆顺序返回三维接触力与接触位置估计；两个向量都使用对应连杆的局部坐标系。它们不是触觉皮肤的原始阵列数据。SDK 中没有单独的“开启全臂触觉”函数；本项目的读取路径不调用运动或伺服命令。

项目脚本 `scripts/record_whole_arm_contact.py` 先读取 `robot.states()[JointGroup.ALL]`，用整机 `q`、`dq` 更新 `Model`，然后采集上述两组数据。它只建立连接、读取状态和模型结果、写本地 CSV。未连接机械臂时可以先看参数：

```bash
python scripts/record_whole_arm_contact.py --help
```

连接机械臂并使用项目的 RDK 2.1 Python 环境后，从项目根目录运行：

```bash
.venv/bin/python scripts/record_whole_arm_contact.py '<完整机器人序列号>' --rate 20 --duration 30 --output logs/whole_arm_contact.csv
```

需要在终端同时看每条记录时加 `--show-live`。CSV 的一行对应一个采样时刻、一个关节组中的一个连杆索引；包含主机时间、机器人时间、三维接触力和三维接触位置。`link_index` 是返回数组内从 0 开始的索引；SDK 文档没有给出该索引与 `Model.link_names()` 的逐组对应关系。`force_*_N` 与 `position_*_m` 是脚本采用的常规物理单位列名；该接口的头文件没有单独标注单位。

`Model` 需要 RDK Professional 许可，且只有部分机型支持多点接触估计。如果记录结束后 CSV 只有表头，应先核对机型支持情况和许可。`robot.states()[group].tau_ext` 另提供各关节的外力矩估计（Nm），可辅助判断机械臂本体受到外部接触，但它本身不给出接触点位置。
