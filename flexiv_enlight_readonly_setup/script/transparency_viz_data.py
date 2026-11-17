"""把 intermediate8 --log 记录的手拖日志算成对比图要用的数据：每次运行一行、每种设置一行、两条特性曲线。

用法（项目根目录，不连机器人）：
  .venv/bin/python script/transparency_viz_data.py OUT_DIR
读 logs/i8_*.csv，写 OUT_DIR/runs.json、modes.json、speed_curve.json、accel_curve.json，
每个文件是 {"rows": [...], "meta": [...]}，meta 里带本脚本的全文，方便核对和重算。

定义（外力 tcp_wrench 是机器人估算的人手施加在末端的力，世界坐标系，② 已清零）：
  一笔推动 = 末端线速度连续 ≥ 0.05 m/s 的一段；运动方向上的外力 = 外力在速度方向上的分量。
  匀速样本 = |沿运动方向的加速度| < 0.15 m/s²：这时外力几乎全用来克服阻力。
  加减速样本 = |加速度| > 0.3 m/s²：拟合 外力 = 等效质量 × 加速度 + 阻尼 × 速度 + 常数，等效质量就是推着加速时有多重。
  95% 区间 = 按"一笔推动"整段重抽样（bootstrap）。同一笔里的样本彼此相关，整段抽才不会把区间算窄。
设置（mode）按文件名判断；有 *_terminal.txt 的按终端打印的控制方式核对，两者不一致时以终端为准。
"""

import glob  # 找日志文件。
import json  # 写结果。
import os  # 文件时间、路径。
import re  # 读终端记录和文件名里的时间。
import sys  # 命令行参数。
import time  # 时间格式。

import numpy as np

# ================================ 分析参数 ================================
MOVING_SPEED = 0.05  # 推动中的最低末端线速度 [m/s]。
STEADY_ACCEL = 0.15  # 匀速样本：|加速度| 低于它 [m/s²]。
FIT_ACCEL = 0.3  # 拟合等效质量用的样本：|加速度| 高于它 [m/s²]。
SMOOTH_SAMPLES = 5  # 求加速度前对速度做滑动平均的点数（100 Hz 下 50 ms）。
SPEED_BINS = [0.05, 0.075, 0.10, 0.125, 0.15, 0.20, 0.25, 0.35]  # 阻力-速度曲线的速度分段 [m/s]。
ACCEL_BINS = [-1.2, -0.8, -0.5, -0.25, -0.08, 0.08, 0.25, 0.5, 0.8, 1.2]  # 外力-加速度曲线的加速度分段 [m/s²]。
MIN_BIN_SAMPLES = 30  # 曲线上一个点至少要这么多样本。
MIN_STROKES = 3  # 并且来自至少这么多笔推动；区间也要至少这么多笔。
BOOTSTRAP = 2000  # 重抽样次数。
CHUNK = 200  # 重抽样分批算，省内存。
SEED = 20261112  # 固定随机种子：同样的日志每次算出同样的区间。
NAME_TIME_SLACK = 120  # 文件名里的时间和终端记录的开始时间差超过它 [s]，就在备注里写出来。
# 设置：文件名前缀（先长后短）、显示名、说明。
MODES = [
    ("i8_active_nsfollow", "active_nsfollow", "active+nsfollow（主动力控 + 肘部跟随）", "X Y Z 主动零力控制，肘部参考姿态跟随（--active --nullspace-follow）"),
    ("i8_active", "active", "active（主动力控）", "X Y Z 主动零力控制（--active）"),
    ("i8_nsfollow", "nsfollow", "nsfollow（肘部跟随）", "零刚度位置控制，肘部参考姿态跟随（--nullspace-follow）"),
    ("i8_ns010", "ns010", "ns010（零刚度位置控制）", "六轴零刚度位置控制，肘部权重 0.1（--scale=0 --nullspace=0.1）"),
    ("i8_passive", "passive", "passive（被动零力）", "X Y Z 被动零力控制，Rx Ry Rz 零刚度（--passive）"),
]
# ========================================================================


def load(path):  # 读一个日志，返回 列名 -> 数组。
    data = np.genfromtxt(path, delimiter=",", names=True)
    return {name: np.asarray(data[name], dtype=float) for name in data.dtype.names}


def smooth(x):  # 按列滑动平均。
    kernel = np.ones(SMOOTH_SAMPLES) / SMOOTH_SAMPLES
    return np.column_stack([np.convolve(x[:, i], kernel, mode="same") for i in range(x.shape[1])])


def mode_from_name(name):
    for prefix, mode, _, _ in MODES:
        if name.startswith(prefix):
            return mode
    return None


def read_terminal(path):  # 终端记录：(设置, 开始时间 [s since epoch, 本地])；没有终端记录或没打印设置返回 (None, None)。
    if not os.path.exists(path):
        return None, None
    with open(path, encoding="utf-8", errors="replace") as f:
        text = f.read()
    start = None
    if m := re.search(r"\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d)\.\d+\] \[info\] \[flexiv::rdk::Robot\] Connected", text):
        start = time.mktime(time.strptime(m.group(1), "%Y-%m-%d %H:%M:%S"))
    if "ARM_1：" not in text:
        return None, start
    follow = "跟随实测关节位置" in text
    if "主动零力控制" in text:
        return ("active_nsfollow" if follow else "active"), start
    if "被动零力控制" in text:
        return "passive", start
    return ("nsfollow" if follow else "ns010"), start


def run_samples(path):  # 一次运行的逐样本量，以及每个样本属于第几笔推动（不在推动中为 -1）。
    log = load(path)
    t = log["t_s"]
    v = np.column_stack([log[c] for c in ("vx", "vy", "vz")])
    f = np.column_stack([log[c] for c in ("fx", "fy", "fz")])
    speed = np.linalg.norm(v, axis=1)
    accel = np.gradient(smooth(v), t, axis=0)
    moving = speed >= MOVING_SPEED
    direction = np.zeros_like(v)
    direction[moving] = v[moving] / speed[moving, None]
    stroke = np.full(len(t), -1)
    stroke[moving] = np.cumsum(np.diff(np.concatenate([[0], moving.astype(int)])) == 1)[moving] - 1
    return {"t": t, "speed": speed, "moving": moving, "stroke": stroke,
            "f_along": np.sum(f * direction, axis=1), "a_along": np.sum(accel * direction, axis=1)}


def pool(samples):  # 几次运行的推动样本合在一起，笔号重新编成 0..k-1。
    keys = ("speed", "f_along", "a_along", "stroke")
    parts, offset = {k: [] for k in keys}, 0
    for s in samples:
        m = s["moving"]
        for k in keys:
            parts[k].append(s[k][m] + (offset if k == "stroke" else 0))
        offset += int(s["stroke"].max()) + 1 if m.any() else 0
    g = {k: np.concatenate(parts[k]) for k in keys}
    g["strokes"] = offset
    return g


def counts_for(g, rng):  # 每次重抽样里每笔推动被抽到几次（有放回地抽 k 笔）：BOOTSTRAP × k。
    k = g["strokes"]
    return rng.multinomial(k, np.full(k, 1.0 / k), size=BOOTSTRAP) if k else np.zeros((BOOTSTRAP, 0))


def median_ci(values, strokes, counts):  # 按笔重抽样后的中位数 95% 区间。
    if len(np.unique(strokes)) < MIN_STROKES:
        return None, None
    order = np.argsort(values)
    v, s = values[order], strokes[order]
    meds = []
    for start in range(0, len(counts), CHUNK):
        cw = np.cumsum(counts[start:start + CHUNK][:, s], axis=1)
        total = cw[:, -1]
        idx = np.argmax(cw >= (total / 2.0)[:, None], axis=1)
        meds.append(v[idx][total > 0])
    lo, hi = np.percentile(np.concatenate(meds), [2.5, 97.5])
    return float(lo), float(hi)


def design(g, mask):  # 外力 = 质量×加速度 + 阻尼×速度 + 常数 的设计矩阵。
    return np.column_stack([g["a_along"][mask], g["speed"][mask], np.ones(int(mask.sum()))]), g["f_along"][mask]


def mass_fit(g, mask):
    if len(np.unique(g["stroke"][mask])) < MIN_STROKES:
        return None
    x, y = design(g, mask)
    return float(np.linalg.lstsq(x, y, rcond=None)[0][0])


def mass_ci(g, mask, counts):  # 每笔推动的 XᵀX、Xᵀy 按抽到的次数加权求和，再解出质量。
    if len(np.unique(g["stroke"][mask])) < MIN_STROKES:
        return None, None
    x, y = design(g, mask)
    s = g["stroke"][mask]
    xtx = np.zeros((g["strokes"], 3, 3))
    xty = np.zeros((g["strokes"], 3))
    np.add.at(xtx, s, x[:, :, None] * x[:, None, :])
    np.add.at(xty, s, x * y[:, None])
    a = np.einsum("bk,kij->bij", counts, xtx)
    b = np.einsum("bk,ki->bi", counts, xty)
    ok = np.abs(np.linalg.det(a)) > 1e-9
    masses = np.linalg.solve(a[ok], b[ok][..., None])[:, 0, 0]
    lo, hi = np.percentile(masses, [2.5, 97.5])
    return float(lo), float(hi)


def curve(g, mask, key, edges, counts):  # 按 key 分段，每段外力中位数和 95% 区间。
    points = []
    for lo_edge, hi_edge in zip(edges[:-1], edges[1:]):
        m = mask & (g[key] >= lo_edge) & (g[key] < hi_edge)
        if m.sum() < MIN_BIN_SAMPLES or len(np.unique(g["stroke"][m])) < MIN_STROKES:
            continue
        lo, hi = median_ci(g["f_along"][m], g["stroke"][m], counts)
        points.append({"x": float(np.median(g[key][m])), "median": float(np.median(g["f_along"][m])),
                       "lo": lo, "hi": hi, "samples": int(m.sum()), "from": lo_edge, "to": hi_edge})
    return points


def r(value, digits):  # 四舍五入，None 保持 None（JSON 里没有 NaN）。
    return None if value is None else round(float(value), digits)


def summarize(g, counts):  # 一组样本的匀速阻力和等效质量，各带 95% 区间。
    steady = np.abs(g["a_along"]) < STEADY_ACCEL
    fit = np.abs(g["a_along"]) > FIT_ACCEL
    res_lo, res_hi = median_ci(g["f_along"][steady], g["stroke"][steady], counts)
    mass_lo, mass_hi = mass_ci(g, fit, counts)
    return {
        "steady_n": r(np.median(g["f_along"][steady]), 2) if steady.any() else None,
        "steady_lo_n": r(res_lo, 2), "steady_hi_n": r(res_hi, 2), "steady_samples": int(steady.sum()),
        "mass_kg": r(mass_fit(g, fit), 1), "mass_lo_kg": r(mass_lo, 1), "mass_hi_kg": r(mass_hi, 1),
        "accel_samples": int(fit.sum()), "strokes": int(g["strokes"]),
    }


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(2)
    out_dir = sys.argv[1]
    os.makedirs(out_dir, exist_ok=True)
    rng = np.random.default_rng(SEED)
    labels = {mode: (label, note) for _, mode, label, note in MODES}

    runs, by_mode, skipped = [], {}, []
    for path in sorted(glob.glob("logs/i8_*.csv"), key=os.path.getmtime):
        name = os.path.basename(path)[:-4]
        if os.path.getsize(path) == 0:
            skipped.append(f"{name}.csv（空文件）")
            continue
        named = mode_from_name(name)
        printed, start = read_terminal(f"logs/{name}_terminal.txt")
        mode = printed or named
        if mode is None:
            skipped.append(f"{name}.csv（认不出设置）")
            continue
        s = run_samples(path)
        by_mode.setdefault(mode, []).append(s)
        g = pool([s])
        duration = float(s["t"][-1] - s["t"][0])
        end = time.localtime(os.path.getmtime(path))
        notes = []
        if named != mode:
            notes.append(f"文件名是 {named}，终端记录是 {mode}，按终端记录算")
        if start is not None and (m := re.search(r"_(\d{8})_(\d{6})$", name)):
            stamp = time.mktime(time.strptime(m.group(1) + m.group(2), "%Y%m%d%H%M%S"))
            if abs(stamp - start) > NAME_TIME_SLACK:
                notes.append(f"实际 {time.strftime('%H:%M:%S', time.localtime(start))} 开始，"
                             f"文件名里的时间沿用了上一次")
        runs.append({
            "run": name, "mode": mode, "mode_label": labels[mode][0],
            "date": time.strftime("%Y-%m-%d", end), "end_time": time.strftime("%H:%M", end),
            "drag_s": r(duration, 1), **summarize(g, counts_for(g, rng)),
            "note": "；".join(notes),
        })

    modes, speed_rows, accel_rows = [], [], []
    for _, mode, label, setting in MODES:
        if mode not in by_mode:
            continue
        g = pool(by_mode[mode])
        counts = counts_for(g, rng)
        modes.append({"mode": mode, "mode_label": label, "setting": setting, "runs": len(by_mode[mode]),
                      "drag_s": r(sum(float(s["t"][-1] - s["t"][0]) for s in by_mode[mode]), 0),
                      **summarize(g, counts)})
        steady = np.abs(g["a_along"]) < STEADY_ACCEL
        for p in curve(g, steady, "speed", SPEED_BINS, counts):
            speed_rows.append({"mode": mode, "mode_label": label, "speed_ms": r(p["x"], 4), "force_n": r(p["median"], 3),
                               "lo_n": r(p["lo"], 3), "hi_n": r(p["hi"], 3), "samples": p["samples"],
                               "bin": f"{p['from']}–{p['to']} m/s"})
        for p in curve(g, np.ones(len(g["speed"]), bool), "a_along", ACCEL_BINS, counts):
            accel_rows.append({"mode": mode, "mode_label": label, "accel_ms2": r(p["x"], 3), "force_n": r(p["median"], 3),
                               "lo_n": r(p["lo"], 3), "hi_n": r(p["hi"], 3), "samples": p["samples"],
                               "bin": f"{p['from']}–{p['to']} m/s²"})

    with open(os.path.abspath(__file__), encoding="utf-8") as f:
        code = f.read()
    inputs = "logs/ 下的 intermediate8 --log 日志：" + "、".join(f"{x['run']}.csv" for x in runs)
    if skipped:
        inputs += "；没用：" + "、".join(skipped)
    meta = [{"label": "transparency_viz_data.py", "value": code, "type": "code", "lang": "python"},
            {"label": "Input", "value": inputs}]
    for name, rows in [("runs", runs), ("modes", modes), ("speed_curve", speed_rows), ("accel_curve", accel_rows)]:
        with open(os.path.join(out_dir, f"{name}.json"), "w", encoding="utf-8") as f:
            json.dump({"rows": rows, "meta": meta}, f, ensure_ascii=False, allow_nan=False)
        print(f"{name}.json：{len(rows)} 行")


if __name__ == "__main__":
    main()
