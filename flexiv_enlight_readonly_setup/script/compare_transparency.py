"""比较 intermediate8 --log 记录的透明度：每次运行用同样的拖法，数值越小越透明。

用法（项目根目录，每个文件是一次运行的日志）：
  .venv/bin/python script/compare_transparency.py logs/i8_adm_m3_d5.csv logs/i8_adm_m5_d10.csv

日志里的外力 tcp_wrench 是机器人估算的"人施加在末端的力"（世界坐标系，② 已清零），所以：
  阻力 = 外力在运动方向上的分量：要用多大的力才能推着它走。
  阻尼 = 阻力 ÷ 速度（最小二乘拟合，同时拟合出惯量项和常数项）：每快 1 m/s 要多用多少力。
  起动力 = 从静止开始推动时用到的最大外力。
  松手漂移 = 日志开头 --hands-off 秒（默认 2 秒）里末端的最大速度和漂了多远：看到"ARM_1 导纳控制已开始"后先别碰它
    （11-15 之前降刚度浮动的日志，对应的提示是"ARM_1 已浮动"）。
    不按外力大小判断有没有人碰：越透明，推它用的力越小，会把正在拖的时段误当成松手。
导纳控制（11-15 之后的 intermediate8）下，拟合出的阻尼、惯量项应该接近 --damping、--mass，常数项接近力死区 0.5 N；
差得多说明 ARM_1 没跟上导纳目标，或者外力估算有偏差。以前降刚度浮动的日志格式一样，可以放在一起比。
"""

import argparse  # 解析命令行参数。
import math  # 无穷大。
import sys  # 出错时打印原因并以非零状态退出。
import unicodedata  # 中文按两格宽对齐。

import numpy as np

# ================================ 分析参数：一般不用改 ================================
LIN_MIN_SPEED = 0.02  # 算平移阻力的最低末端线速度 [m/s]：更慢时外力主要是噪声和零点误差。
ANG_MIN_SPEED = 0.05  # 算转动阻力的最低角速度 [rad/s]。
LIN_BINS = [0.02, 0.05, 0.10, 0.20, math.inf]  # 平移阻力按线速度分段取中位数 [m/s]。
ANG_BINS = [0.05, 0.10, 0.30, math.inf]  # 转动阻力按角速度分段取中位数 [rad/s]。
STILL_SPEED = 0.005  # 低于这个线速度算静止 [m/s]。
ONSET_SPEED = 0.01  # 静止之后线速度升过它，算一次起动 [m/s]。
STILL_BEFORE = 0.3  # 起动前至少静止这么久，才算一次起动 [s]。
HANDS_OFF = 2.0  # 日志开头没人碰的秒数，用来量松手漂移 [s]；命令行 --hands-off 可改。
TOUCH_WARN_FORCE = 3.0  # 这段时间外力超过它就提示可能有人碰了 [N]。
SMOOTH_SAMPLES = 5  # 求加速度前对速度做滑动平均的点数（100 Hz 下 50 ms）。
# ====================================================================================


def load(path):  # 读一个日志文件，返回 列名 -> 数组。
    data = np.genfromtxt(path, delimiter=",", names=True)
    if data.size < 10:
        raise ValueError(f"{path} 只有 {data.size} 行，太短")
    return {name: np.asarray(data[name], dtype=float) for name in data.dtype.names}


def vectors(log, names):  # 把 3 列拼成 N×3 数组。
    return np.column_stack([log[n] for n in names])


def smooth(x):  # 按列滑动平均。
    kernel = np.ones(SMOOTH_SAMPLES) / SMOOTH_SAMPLES
    return np.column_stack([np.convolve(x[:, i], kernel, mode="same") for i in range(x.shape[1])])


def resistance(t, velocity, load_vec, min_speed, bins):  # 阻力分段中位数，以及 阻力 = 阻尼×速度 + 惯量×加速度 + 常数 的拟合。
    speed = np.linalg.norm(velocity, axis=1)
    moving = speed >= min_speed
    result = {"samples": int(moving.sum()), "bins": [], "fit": None}
    if result["samples"] < 20:
        return result
    direction = velocity[moving] / speed[moving, None]
    along = np.sum(load_vec[moving] * direction, axis=1)  # 外力（矩）在运动方向上的分量。
    accel = np.gradient(smooth(velocity), t, axis=0)
    accel_along = np.sum(accel[moving] * direction, axis=1)
    for low, high in zip(bins[:-1], bins[1:]):
        in_bin = (speed[moving] >= low) & (speed[moving] < high)
        result["bins"].append((low, high, int(in_bin.sum()), float(np.median(along[in_bin])) if in_bin.any() else None))
    design = np.column_stack([speed[moving], accel_along, np.ones(len(along))])
    coef, *_ = np.linalg.lstsq(design, along, rcond=None)
    residual = along - design @ coef
    r2 = 1.0 - residual.var() / along.var() if along.var() > 0 else float("nan")
    result["fit"] = {"damping": coef[0], "inertia": coef[1], "offset": coef[2], "r2": r2}
    return result


def onsets(t, speed, force_norm):  # 每次从静止起动时用到的最大外力 [N]。
    forces = []
    still_since = None  # 当前这段静止从哪个时刻开始；不在静止时为 None。
    last_still = None  # 最近一个静止样本的序号。
    for i in range(len(t)):
        if speed[i] < STILL_SPEED:
            still_since = t[i] if still_since is None else still_since
            last_still = i
            continue
        if still_since is not None and speed[i] >= ONSET_SPEED:
            if t[last_still] - still_since >= STILL_BEFORE:
                forces.append(float(force_norm[last_still:i + 1].max()))
            still_since = None
        elif speed[i] >= ONSET_SPEED:
            still_since = None
    return forces


def analyze(path, hands_off):  # 一个日志的全部指标。
    log = load(path)
    t = log["t_s"]
    position = vectors(log, ["x", "y", "z"])
    velocity = vectors(log, ["vx", "vy", "vz"])
    omega = vectors(log, ["wx", "wy", "wz"])
    force = vectors(log, ["fx", "fy", "fz"])
    moment = vectors(log, ["mx", "my", "mz"])
    speed = np.linalg.norm(velocity, axis=1)
    force_norm = np.linalg.norm(force, axis=1)
    still = speed < STILL_SPEED
    window = t <= t[0] + hands_off  # 日志开头没人碰的时段。
    end = int(np.flatnonzero(window)[-1])
    drift = {
        "seconds": float(t[end] - t[0]),
        "max_speed": float(speed[window].max()),
        "max_omega": float(np.linalg.norm(omega[window], axis=1).max()),
        "distance": float(np.linalg.norm(position[end] - position[0])),
        "max_force": float(force_norm[window].max()),
    }
    return {
        "duration": float(t[-1] - t[0]),
        "rows": len(t),
        "max_speed": float(speed.max()),
        "still_force": float(np.median(force_norm[still])) if still.any() else None,
        "linear": resistance(t, velocity, force, LIN_MIN_SPEED, LIN_BINS),
        "angular": resistance(t, omega, moment, ANG_MIN_SPEED, ANG_BINS),
        "onsets": onsets(t, speed, force_norm),
        "drift": drift,
    }


def fmt(value, digits=2):  # 数字转文字；没有数据时是 "—"。
    return "—" if value is None or (isinstance(value, float) and math.isnan(value)) else f"{value:.{digits}f}"


def pad(text, width):  # 按显示宽度补空格：中文字符占两格。
    shown = sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in text)
    return text + " " * max(width - shown, 0)


def print_detail(path, r):  # 一个文件的详细结果。
    print(f"\n== {path}")
    print(f"  时长 {fmt(r['duration'], 1)} s，{r['rows']} 行，最大线速度 {fmt(r['max_speed'], 3)} m/s，"
          f"静止时外力读数中位数 {fmt(r['still_force'])} N（零点误差，越小越准）")
    for name, key, unit, rate, what in (("平移", "linear", "N", "m/s", "外力"), ("转动", "angular", "Nm", "rad/s", "外力矩")):
        res = r[key]
        print(f"  {name}阻力（运动方向上的{what}，{res['samples']} 个运动样本）：")
        for low, high, count, median in res["bins"]:
            span = f"{low:g} {rate} 以上" if math.isinf(high) else f"{low:g}–{high:g} {rate}"
            print(f"    {span}：中位数 {fmt(median)} {unit}（{count} 个样本）")
        fit = res["fit"]
        if fit:
            print(f"    拟合：阻尼 {fmt(fit['damping'])} {unit}·s/{rate.split('/')[0]}，惯量项 {fmt(fit['inertia'])}，"
                  f"常数 {fmt(fit['offset'])} {unit}，R² {fmt(fit['r2'])}")
        else:
            print("    运动样本太少，没有拟合")
    forces = r["onsets"]
    print(f"  起动力：{len(forces)} 次从静止起动，最大外力中位数 {fmt(float(np.median(forces)) if forces else None)} N")
    d = r["drift"]
    print(f"  松手漂移（日志开头 {fmt(d['seconds'], 1)} s）：最大线速度 {fmt(d['max_speed'], 3)} m/s、最大角速度 "
          f"{fmt(d['max_omega'], 3)} rad/s，漂了 {fmt(d['distance'] * 1000, 1)} mm")
    if d["max_force"] > TOUCH_WARN_FORCE:
        print(f"    注意：这段时间外力最大 {fmt(d['max_force'])} N，可能有人碰了，漂移数不准")


def print_summary(paths, results):  # 所有文件的关键指标放一起比。
    rows = [
        ("平移阻尼 [N·s/m]", lambda r: r["linear"]["fit"] and r["linear"]["fit"]["damping"]),
        ("0.05–0.1 m/s 平移阻力 [N]", lambda r: r["linear"]["bins"][1][3] if len(r["linear"]["bins"]) > 1 else None),
        ("转动阻尼 [Nm·s/rad]", lambda r: r["angular"]["fit"] and r["angular"]["fit"]["damping"]),
        ("起动力中位数 [N]", lambda r: float(np.median(r["onsets"])) if r["onsets"] else None),
        ("松手最大漂移 [m/s]", lambda r: r["drift"]["max_speed"]),
    ]
    width = 28
    names = [p.rsplit("/", 1)[-1] for p in paths]
    print("\n== 对比（同样的拖法下，越小越透明；松手漂移越小越稳）")
    print(pad("", width) + "".join(f"{n:>22}" for n in names))
    for label, get in rows:
        values = []
        for r in results:
            value = get(r)
            values.append(fmt(value, 3 if "漂移" in label else 2) if value is not None else "—")
        print(pad(label, width) + "".join(f"{v:>22}" for v in values))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("logs", nargs="+", help="intermediate8 --log 写出的 CSV 文件，一次运行一个")
    parser.add_argument("--hands-off", type=float, default=HANDS_OFF,
                        help=f"日志开头没人碰的秒数，用来量松手漂移，默认 {HANDS_OFF}")
    args = parser.parse_args(argv)
    results = []
    for path in args.logs:
        try:
            results.append(analyze(path, args.hands_off))
        except (OSError, ValueError) as error:
            sys.exit(f"读不了 {path}：{error}")
    for path, r in zip(args.logs, results):
        print_detail(path, r)
    print_summary(args.logs, results)


if __name__ == "__main__":
    main()
