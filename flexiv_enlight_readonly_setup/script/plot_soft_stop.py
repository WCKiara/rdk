"""画 arm1_transparent_arm2_hold_rt_cartesian --log 记录的 ARM_1 软停止，并打印验收用的数字（软停止指令第 8 节）。

用法（项目根目录；画图要 matplotlib：.venv/bin/pip install matplotlib，只打印数字加 --no-plots）：
  .venv/bin/python script/plot_soft_stop.py logs/soft_stop_20261115_120000.csv
  .venv/bin/python script/plot_soft_stop.py logs/soft_stop_20261115_120000.csv --out logs/soft_stop_plots
没有真机时可以先画 arm1_soft_stop_test --csv sim.csv 写出的模拟日志（没有关节数据）。

6 张图存成 PNG，默认放在 CSV 旁边的 <文件名>_plots/：
  1_position.png        ARM_1 目标/实际平移位置（相对锁存时）：目标应是二次曲线，看跟踪误差
  2_velocity.png        ARM_1 目标/实际线速度：目标线性降到 0，实际不应反向冲
  3_joint_velocity.png  ARM_1 七个关节速度 dθ：看有没有个别关节突跳
  4_stiffness.png       ARM_1 应有的刚度和 SetCartesianImpedance 实际设的刚度（开刚度渐增时是阶梯）
  5_arm2_hold.png       ARM_2 末端、关节离 hold 位置的偏差（整段日志）
  6_rt_cycle.png        实时周期间隔：最大值、p95、p99、迟到
图上的竖线：实线 = 锁存（收到 Stop Sign），虚线 = t0 + T，点线 = 进 HOLD。图里的文字用英文，免得没有中文字体时显示成方块。
打印的数字只用来对照 README_arm1_soft_stop.md 的验收项，不是厂家安全认证值。
"""

import argparse  # 解析命令行参数。
import os  # 输出目录。
import sys  # 出错时打印原因并以非零状态退出。

import numpy as np

# ================================ 分析参数：一般不用改 ================================
LATE_US = 2000.0  # 实时周期间隔超过它算迟到 [µs]，同 C++ 的 kLateCycleMs。
REVERSE_SPEED = 0.002  # 锁存后实测速度在 v0 方向上的分量低于 -REVERSE_SPEED 算反向运动 [m/s]，更小的当噪声。
WINDOW_BEFORE = 0.5  # 图 1–4 从锁存前多少秒开始画 [s]。
WINDOW_AFTER_HOLD = 1.0  # 图 1–4 画到进 HOLD 后多少秒 [s]；没进 HOLD 时画到 t0 + T 后 3 秒。
# ====================================================================================

STATES = {0: "TRANSPARENT", 1: "SOFT_STOP", 2: "HOLD"}
AXES = ("x", "y", "z")
K_AXES = ("x", "y", "z", "rx", "ry", "rz")


def load(path):  # 读一个日志文件，返回 列名 -> 数组。
    with open(path, encoding="utf-8") as f:
        names = f.readline().strip().split(",")
    data = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    if data.shape[0] < 2:
        raise ValueError(f"只有 {data.shape[0]} 行数据，太短")
    if data.shape[1] != len(names):
        raise ValueError(f"表头 {len(names)} 列、数据 {data.shape[1]} 列，对不上")
    return {name: data[:, i] for i, name in enumerate(names)}


def vec(log, prefix, suffixes):  # 把几列拼成 N×k 数组，例如 vec(log, "arm1_p", AXES)。
    return np.column_stack([log[prefix + s] for s in suffixes])


def norm(rows):  # 每行的长度。
    return np.linalg.norm(rows, axis=1)


def fmt(value, digits=2):  # 数字转文字；没有数据时是 "—"。
    return "—" if value is None or not np.isfinite(value) else f"{value:.{digits}f}"


def events(log):  # 锁存、进 HOLD 的行号和时刻，以及 T；没有的是 None。
    t = log["time_s"]
    state = log["arm1_state"]
    ev = {"i_latch": None, "t_latch": None, "T": None, "i_hold": None, "t_hold": None}
    left = np.flatnonzero(state >= 1)
    if left.size:
        i = int(left[0])
        ev.update(i_latch=i, t_latch=float(t[i]), T=float(log["stop_T_s"][i]))
    hold = np.flatnonzero(state == 2)
    if hold.size:
        ev.update(i_hold=int(hold[0]), t_hold=float(t[hold[0]]))
    return ev


def state_sequence(state):  # 状态依次出现的顺序，例如 [0, 1, 2]。
    valid = state[np.isfinite(state)].astype(int)
    seq = [int(valid[0])] if valid.size else []
    for s in valid[1:]:
        if s != seq[-1]:
            seq.append(int(s))
    return seq


def summarize(log, ev):  # 打印验收用的数字；返回有没有发现问题。
    t = log["time_s"]
    ok = True
    seq = state_sequence(log["arm1_state"])
    print(f"  时长 {fmt(t[-1] - t[0], 1)} s，{len(t)} 行；Stop Sign {int(np.nanmax(log['stop_sign']))} 次；"
          f"ARM_1 状态 {' → '.join(STATES.get(s, str(s)) for s in seq)}")
    if 0 in seq[1:]:
        print("  问题：离开 TRANSPARENT 之后又回到了 TRANSPARENT（应该只能重新运行程序才恢复）")
        ok = False
    if seq.count(1) > 1:
        print("  问题：SOFT_STOP 出现了不止一次，锁存可能被重置")
        ok = False

    if ev["i_latch"] is not None:
        i, T = ev["i_latch"], ev["T"]
        p_d = vec(log, "arm1_target_p", AXES)
        v_d = vec(log, "arm1_target_v", AXES)
        p = vec(log, "arm1_p", AXES)
        v = vec(log, "arm1_v", AXES)
        p0, v0 = p_d[i], v_d[i]
        speed0 = float(np.linalg.norm(v0))
        tau = log["stop_elapsed_s"]
        in_stop = np.isfinite(tau) & (tau <= T)
        tc = np.clip(tau[in_stop], 0.0, T)
        formula = p0 + np.outer(tc - tc ** 2 / (2.0 * T), v0)
        formula_error = float(norm(p_d[in_stop] - formula).max())
        target_speed = norm(v_d[in_stop])
        monotonic = bool(np.all(np.diff(target_speed) <= 1e-9))
        print(f"  锁存于 {fmt(ev['t_latch'], 3)} s：|v0| = {fmt(speed0, 4)} m/s，T = {fmt(T, 3)} s，"
              f"计划停止距离 {fmt(0.5 * speed0 * T * 1000.0)} mm")
        print(f"  目标轨迹：和公式 p0 + v0 (t - t²/2T) 最多差 {formula_error * 1000.0:.2e} mm；"
              f"目标速度{'单调降到 0' if monotonic else '不是单调下降'}")
        ok = ok and monotonic and formula_error < 1e-6
        after = slice(i, None)
        track = float(norm(p[after] - p_d[after]).max())
        reverse = None
        if speed0 > 1e-9:
            reverse = float((v[after] @ (v0 / speed0)).min())
        motion = np.flatnonzero((log["motion_ctrl"] >= 0.5) & (np.arange(len(t)) >= i))
        delay = (t[motion[0]] - ev["t_latch"]) * 1000.0 if motion.size else None
        print(f"  实际：锁存后最大跟踪误差 {fmt(track * 1000.0)} mm；沿 v0 方向最小速度 {fmt(reverse, 4)} m/s"
              f"（低于 -{REVERSE_SPEED} 算反向）；锁存后 {fmt(delay, 1)} ms 切回运动控制")
        if reverse is not None and reverse < -REVERSE_SPEED:
            print("  问题：锁存后实测速度反向，可能有回弹或被推")
            ok = False
        if ev["i_hold"] is not None:
            j = ev["i_hold"]
            print(f"  锁存后 {fmt(ev['t_hold'] - ev['t_latch'], 3)} s 进 HOLD：实际走了 "
                  f"{fmt(np.linalg.norm(p[j] - p[i]) * 1000.0)} mm，离终点 {fmt(np.linalg.norm(p[j] - p_d[j]) * 1000.0)} mm；"
                  f"日志结束时离终点 {fmt(np.linalg.norm(p[-1] - p_d[-1]) * 1000.0)} mm")
        else:
            print("  没有进 HOLD")

    arm2_pos = log["arm2_hold_position_error"]
    arm2_joint = log["arm2_hold_joint_error"]
    if np.isfinite(arm2_pos).any():
        before = np.arange(len(t)) < (ev["i_latch"] if ev["i_latch"] is not None else len(t))
        parts = [f"全程最大 {fmt(np.nanmax(arm2_pos) * 1000.0, 3)} mm"]
        if ev["i_latch"] is not None and before.any():
            parts.append(f"锁存前 {fmt(np.nanmax(arm2_pos[before]) * 1000.0, 3)} mm、"
                         f"锁存后 {fmt(np.nanmax(arm2_pos[~before]) * 1000.0, 3)} mm")
        print(f"  ARM_2 末端离 hold 位置：{'，'.join(parts)}；关节最大偏离 {fmt(np.nanmax(arm2_joint), 5)} rad")

    cycle = log["rt_cycle_us"][np.isfinite(log["rt_cycle_us"])]
    if cycle.size:
        p95, p99 = np.percentile(cycle, [95, 99])
        late = int((cycle > LATE_US).sum())
        print(f"  实时周期间隔：平均 {fmt(cycle.mean(), 1)} µs，p95 {fmt(p95, 1)}，p99 {fmt(p99, 1)}，最大 "
              f"{fmt(cycle.max(), 1)} µs；超过 {LATE_US:g} µs 的 {late} 次")
    failed_sends = int((log["stream_status"] == 0).sum())
    faults = int((log["fault_status"] == 1).sum())
    if failed_sends or faults:
        print(f"  问题：{failed_sends} 个周期发送失败，{faults} 个周期机器人故障")
        ok = False
    latency = log["impedance_request_latency_ms"]
    latency = latency[np.isfinite(latency)]
    if latency.size:
        calls = int(np.count_nonzero(np.diff(latency))) + 1
        print(f"  SetCartesianImpedance 用时：约 {calls} 次，最长 {fmt(latency.max(), 1)} ms")
    return ok


def plot_all(log, ev, out_dir):  # 6 张图存成 PNG，返回文件路径。
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = log["time_s"]
    latched = ev["t_latch"] is not None
    origin = ev["t_latch"] if latched else t[0]
    x = t - origin
    if latched:
        end = ev["t_hold"] - origin + WINDOW_AFTER_HOLD if ev["t_hold"] is not None else ev["T"] + 3.0
        window = (x >= -WINDOW_BEFORE) & (x <= end)
        xlabel = "time since Stop Sign latch [s]"
    else:
        window = np.ones_like(x, dtype=bool)
        xlabel = "time [s]"
    files = []

    def marks(ax):  # 锁存、t0 + T、进 HOLD 的竖线。
        if not latched:
            return
        ax.axvline(0.0, color="k", lw=0.8)
        ax.axvline(ev["T"], color="k", lw=0.8, ls="--")
        if ev["t_hold"] is not None:
            ax.axvline(ev["t_hold"] - origin, color="k", lw=0.8, ls=":")

    def save(fig, name):
        path = os.path.join(out_dir, name)
        fig.tight_layout()
        fig.savefig(path, dpi=120)
        plt.close(fig)
        files.append(path)

    w = window
    p_ref = vec(log, "arm1_target_p", AXES)[ev["i_latch"]] if latched else vec(log, "arm1_p", AXES)[0]

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 8))
    for k, a in enumerate(AXES):
        axes[k].plot(x[w], (log[f"arm1_target_p{a}"][w] - p_ref[k]) * 1000.0, lw=1.6, label="target")
        axes[k].plot(x[w], (log[f"arm1_p{a}"][w] - p_ref[k]) * 1000.0, lw=1.0, label="measured")
        axes[k].set_ylabel(f"Δp{a} [mm]")
        axes[k].grid(alpha=0.3)
        marks(axes[k])
    axes[0].set_title("1  ARM_1 TCP position relative to latch: quadratic target vs measured\n"
                      "(before latch the target is the transparent pose, unused on force-controlled axes)", fontsize=10)
    axes[0].legend(fontsize=8)
    axes[-1].set_xlabel(xlabel)
    save(fig, "1_position.png")

    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(9, 9))
    for k, a in enumerate(AXES):
        axes[k].plot(x[w], log[f"arm1_target_v{a}"][w], lw=1.6, label="target")
        axes[k].plot(x[w], log[f"arm1_v{a}"][w], lw=1.0, label="measured")
        axes[k].set_ylabel(f"v{a} [m/s]")
    axes[3].plot(x[w], norm(vec(log, "arm1_target_v", AXES))[w], lw=1.6, label="target")
    axes[3].plot(x[w], norm(vec(log, "arm1_v", AXES))[w], lw=1.0, label="measured")
    axes[3].set_ylabel("|v| [m/s]")
    for ax in axes:
        ax.grid(alpha=0.3)
        marks(ax)
    axes[0].set_title("2  ARM_1 TCP linear velocity: target decreases linearly to 0")
    axes[0].legend(fontsize=8)
    axes[-1].set_xlabel(xlabel)
    save(fig, "2_velocity.png")

    fig, ax = plt.subplots(figsize=(9, 5))
    joints = vec(log, "arm1_dtheta", [str(i) for i in range(1, 8)])
    if np.isfinite(joints).any():
        for i in range(7):
            ax.plot(x[w], joints[w, i], lw=1.0, label=f"J{i + 1}")
        ax.legend(fontsize=8, ncol=7)
    else:
        ax.text(0.5, 0.5, "no joint data in this log (simulation?)", ha="center", va="center", transform=ax.transAxes)
    ax.set_title("3  ARM_1 joint velocities dtheta")
    ax.set_ylabel("[rad/s]")
    ax.set_xlabel(xlabel)
    ax.grid(alpha=0.3)
    marks(ax)
    save(fig, "3_joint_velocity.png")

    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(9, 7))
    for k, a in enumerate(K_AXES):
        ax = axes[0 if k < 3 else 1]
        line = ax.plot(x[w], log[f"requested_Kx_arm1_{a}"][w], lw=1.6, label=f"{a} requested")[0]
        ax.plot(x[w], log[f"set_Kx_arm1_{a}"][w], lw=1.0, ls="--", drawstyle="steps-post", color=line.get_color(),
                label=f"{a} set")
    axes[0].set_ylabel("translational [N/m]")
    axes[1].set_ylabel("rotational [Nm/rad]")
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, ncol=3)
        marks(ax)
    axes[0].set_title("4  ARM_1 Cartesian stiffness: requested K(t) vs SetCartesianImpedance (stairs)")
    axes[-1].set_xlabel(xlabel)
    save(fig, "4_stiffness.png")

    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
    axes[0].plot(x, log["arm2_hold_position_error"] * 1000.0, lw=1.0)
    axes[0].set_ylabel("TCP error [mm]")
    axes[1].plot(x, log["arm2_hold_joint_error"], lw=1.0)
    axes[1].set_ylabel("max joint error [rad]")
    for ax in axes:
        ax.grid(alpha=0.3)
        marks(ax)
    axes[0].set_title("5  ARM_2 deviation from the hold pose (whole log)")
    axes[-1].set_xlabel(xlabel)
    save(fig, "5_arm2_hold.png")

    cycle = log["rt_cycle_us"]
    valid = np.isfinite(cycle)
    fig, axes = plt.subplots(2, 1, figsize=(9, 7))
    axes[0].plot(x[valid], cycle[valid], lw=0.6)
    axes[0].axhline(LATE_US, color="r", lw=0.8, ls="--")
    axes[0].set_ylabel("cycle [µs]")
    axes[0].set_xlabel(xlabel)
    marks(axes[0])
    if valid.any():
        p95, p99 = np.percentile(cycle[valid], [95, 99])
        axes[0].set_title(f"6  RT cycle interval: p95 {p95:.0f} µs, p99 {p99:.0f} µs, max {cycle[valid].max():.0f} µs, "
                          f"> {LATE_US:g} µs: {int((cycle[valid] > LATE_US).sum())}")
        axes[1].hist(cycle[valid], bins=100, log=True)
    axes[1].set_xlabel("cycle [µs]")
    axes[1].set_ylabel("count")
    for ax in axes:
        ax.grid(alpha=0.3)
    save(fig, "6_rt_cycle.png")
    return files


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", help="arm1_transparent_arm2_hold_rt_cartesian --log 写出的 CSV 文件")
    parser.add_argument("--out", help="PNG 输出目录，默认 CSV 旁边的 <文件名>_plots/")
    parser.add_argument("--no-plots", action="store_true", help="只打印数字，不画图（不需要 matplotlib）")
    args = parser.parse_args(argv)
    try:
        log = load(args.log)
    except (OSError, ValueError) as error:
        sys.exit(f"读不了 {args.log}：{error}")
    ev = events(log)
    print(f"== {args.log}")
    ok = summarize(log, ev)
    if not args.no_plots:
        try:
            import matplotlib  # noqa: F401  只检查装没装。
        except ImportError:
            sys.exit("画图要 matplotlib：.venv/bin/pip install matplotlib；只看数字加 --no-plots")
        out_dir = args.out or os.path.splitext(args.log)[0] + "_plots"
        os.makedirs(out_dir, exist_ok=True)
        for path in plot_all(log, ev, out_dir):
            print(f"  图：{path}")
    print("  结论：" + ("没有发现上面列出的问题" if ok else "有问题，见上面"))


if __name__ == "__main__":
    main()
