"""按连杆记录所选机械臂的全臂接触估计，默认只记录 ARM_1。"""

import argparse  # 解析命令行参数。
import csv  # 写入 CSV 表格。
import math  # 检查参数和数据是否为有限数。
import time  # 计时、生成主机时间戳和控制采样间隔。
from pathlib import Path  # 处理输出文件路径。


def main(argv=None):  # 定义程序入口；argv 可供离线检查传入参数。
    parser = argparse.ArgumentParser(description=__doc__)  # 创建参数解析器。
    parser.add_argument("robot_sn", help="Actual full robot serial number")  # 实际完整序列号。
    parser.add_argument("--arm", choices=("ARM_1", "ARM_2"), default="ARM_1")  # 选择记录哪条臂。
    parser.add_argument("--output", type=Path, default=Path("logs/whole_arm_contact.csv"))  # CSV 路径。
    parser.add_argument("--rate", type=float, default=20.0, help="Sampling rate in Hz")  # 采样频率。
    parser.add_argument("--duration", type=float, default=30.0, help="Recording length in seconds")  # 时长。
    parser.add_argument("--show-live", action="store_true")  # 加此参数时同时打印各连杆数据。
    args = parser.parse_args(argv)  # 读取并验证命令行参数。
    if not all(math.isfinite(v) and v > 0 for v in (args.rate, args.duration)):  # 拒绝非正数、NaN 和无穷大。
        parser.error("--rate and --duration must be finite and positive")  # 参数无效时退出。

    import flexivrdk  # 延迟导入 SDK，使 --help 无需加载 SDK。

    methods = ("multi_contact_forces", "multi_contact_positions")  # 所需的两项全臂接口。
    if not all(hasattr(flexivrdk.Model, name) for name in methods):  # 检查 SDK 是否提供接口。
        raise RuntimeError("This SDK does not expose multi-contact Model methods")  # 缺失则退出。
    group = getattr(flexivrdk.JointGroup, args.arm)  # 把 ARM_1/ARM_2 转成 SDK 的关节组枚举。
    robot = flexivrdk.Robot(args.robot_sn)  # 连接机器人；不发送运动命令。
    model = flexivrdk.Model(robot)  # 创建读取模型与接触估计的接口。
    period = 1.0 / args.rate  # 目标采样周期：20 Hz 对应 0.05 秒。
    sample = 0  # 已完成的采样次数，从 0 开始编号。
    rows = 0  # 已写入的连杆记录行数。

    args.output.parent.mkdir(parents=True, exist_ok=True)  # 输出目录不存在时创建。
    with args.output.open("w", newline="", encoding="utf-8") as stream:  # 打开 CSV；同名文件会覆盖。
        writer = csv.writer(stream)  # 创建 CSV 写入器。
        writer.writerow([  # 写入列名；下面的每组对应一种数据。
            "sample", "host_time_ns", "robot_time_sec", "robot_time_nsec",  # 采样编号和两种时钟。
            "joint_group", "link_index",  # 关节组与 SDK 返回数组内的连杆索引。
            "force_x_N", "force_y_N", "force_z_N",  # 连杆局部坐标系中的三维力估计。
            "position_x_m", "position_y_m", "position_z_m",  # 同一局部坐标系中的接触位置估计。
        ])  # 结束表头。
        stop_at = time.monotonic() + args.duration  # 用不受系统校时影响的时钟确定结束时刻。
        while time.monotonic() < stop_at:  # 未达到记录时长就继续采样。
            started = time.monotonic()  # 记录本轮开始时刻，用于补足采样周期。
            if not robot.connected():  # 防止断线后继续写入缓存数据。
                raise RuntimeError("Robot connection lost")  # 断线时停止并关闭文件。
            states = robot.states()  # 获取机器人最新缓存状态。
            all_states = states[flexivrdk.JointGroup.ALL]  # 提取整机状态，满足模型的输入要求。
            model.Update(all_states.q, all_states.dq)  # 用整机关节角和角速度更新本地模型。
            forces_by_group = model.multi_contact_forces()  # 获取各组、各连杆的三维接触力估计。
            positions_by_group = model.multi_contact_positions()  # 获取对应的接触位置估计。
            if group not in forces_by_group or group not in positions_by_group:  # 检查所选臂是否有数据。
                raise RuntimeError(f"No multi-contact data for {args.arm}")  # 缺失时退出，不生成成功记录。
            forces = forces_by_group[group]  # 只保留所选臂；默认 ARM_1。
            positions = positions_by_group[group]  # 取出同一条臂的接触位置。
            if not forces or len(forces) != len(positions):  # 拒绝空数组和无法一一对应的数据。
                raise RuntimeError("Empty or mismatched multi-contact data")  # 数据结构异常时退出。
            host_time_ns = time.time_ns()  # 获取主机时间戳，单位为纳秒。
            sec, nsec = all_states.timestamp  # 获取状态的机器人时间戳：秒与秒内纳秒。
            for link_index, (force, position) in enumerate(zip(forces, positions)):  # 遍历所选臂的全部条目。
                values = [float(v) for v in (*force, *position)]  # 按 Fx、Fy、Fz、Px、Py、Pz 排列。
                if len(values) != 6 or not all(math.isfinite(v) for v in values):  # 检查维度和有限性。
                    raise RuntimeError("Invalid multi-contact vector")  # 拒绝异常数据。
                writer.writerow([sample, host_time_ns, sec, nsec, args.arm, link_index, *values])  # 一条目一行。
                rows += 1  # 累计写入行数。
                if args.show_live:  # 按参数决定是否打印当前条目。
                    print(f"{args.arm} link {link_index}: force={list(force)} position={list(position)}", flush=True)  # 实时输出。
            sample += 1  # 全部连杆记录完成后，采样编号加一。
            stream.flush()  # 将本次记录从 Python 缓冲区提交给操作系统。
            time.sleep(max(0.0, period - (time.monotonic() - started)))  # 扣除计算耗时后等待，尽量保持目标频率。

    print(f"Recorded {sample} samples and {rows} {args.arm} link rows to {args.output}")  # 正常结束时输出统计。


if __name__ == "__main__":  # 仅在直接运行此脚本时启动采集。
    main()  # 执行入口函数。
