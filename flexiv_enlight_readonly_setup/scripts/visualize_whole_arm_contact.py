"""Plot recorded per-link contact estimates offline; never connect to the robot.

Use a Python environment with numpy and matplotlib. --interactive additionally
needs plotly and embeds its JavaScript into a standalone HTML file.
"""

import argparse
import csv
from datetime import datetime
import html
import json
from pathlib import Path
import re
from zoneinfo import ZoneInfo

import matplotlib
matplotlib.use("Agg")
from matplotlib import font_manager
from matplotlib.backends.backend_pdf import PdfPages
import matplotlib.pyplot as plt
import numpy as np


XYZ_COLORS = ("#2563eb", "#d97706", "#9333ea")
LINK_COLORS = ("#64748b", "#94a3b8", "#2563eb", "#cbd5e1", "#ea580c", "#0d9488", "#c026d3")
ZONE = ZoneInfo("America/Los_Angeles")


def load_csv(path, arm):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = [r for r in csv.DictReader(stream) if r["joint_group"] == arm]
    if not rows:
        raise ValueError(f"No data for {arm}")
    grouped = {}
    for row in rows:
        grouped.setdefault(int(row["sample"]), []).append(row)
    sample_ids = sorted(grouped)
    links = sorted(int(r["link_index"]) for r in grouped[sample_ids[0]])
    if len(set(links)) != len(links):
        raise ValueError("Duplicate link rows in first sample")
    keys = ("force_x_N", "force_y_N", "force_z_N", "position_x_m", "position_y_m", "position_z_m")
    values, host, robot = [], [], []
    for sample in sample_ids:
        block = sorted(grouped[sample], key=lambda r: int(r["link_index"]))
        if [int(r["link_index"]) for r in block] != links:
            raise ValueError(f"Incomplete or duplicate links in sample {sample}")
        stamps = {(r["host_time_ns"], r["robot_time_sec"], r["robot_time_nsec"]) for r in block}
        if len(stamps) != 1:
            raise ValueError(f"Inconsistent timestamps in sample {sample}")
        values.append([[float(r[k]) for k in keys] for r in block])
        host.append(int(block[0]["host_time_ns"]))
        robot.append(int(block[0]["robot_time_sec"]) * 10**9 + int(block[0]["robot_time_nsec"]))
    a = np.asarray(values)
    host, robot = np.asarray(host, dtype=np.int64), np.asarray(robot, dtype=np.int64)
    if len(host) < 2 or not np.all(np.isfinite(a)) or not np.all(np.diff(host) > 0) or not np.all(np.diff(robot) > 0):
        raise ValueError("Need finite data and strictly advancing host/robot timestamps")
    return links, a, host, robot


def log_time(line):
    match = re.search(r"\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d+)\]", line)
    if match:
        value = datetime.strptime(match.group(1), "%Y-%m-%d %H:%M:%S.%f").replace(tzinfo=ZONE)
        return round(value.timestamp() * 10**9)


def load_events(source, origin_ns):
    result = {}
    motion = source.with_suffix(".motion.log")
    if motion.exists():
        for line in motion.read_text().splitlines():
            stamp = log_time(line)
            if stamp is None:
                continue
            relative = (stamp - origin_ns) / 1e9
            result.setdefault("program_start", relative)
            if "Control mode switched to [NRT_PRIMITIVE_EXECUTION]" in line:
                result.setdefault("home_start", relative)
            if "TILT_PLAN" in line:
                result.setdefault("home_end", relative)
            if "CONTROL_FAILED" in line:
                result.setdefault("fault_detected", relative)
            result["program_end"] = relative
    recorder = source.with_suffix(".recorder.log")
    if recorder.exists():
        for line in recorder.read_text().splitlines():
            if "[303010]" in line and log_time(line) is not None:
                result["event_303010_received"] = (log_time(line) - origin_ns) / 1e9
    return result


def style_axis(ax):
    ax.set_facecolor("white")
    ax.grid(color="#e2e8f0", linewidth=0.7)
    ax.set_axisbelow(True)
    ax.spines[["top", "right"]].set_visible(False)
    for name in ("bottom", "left"):
        ax.spines[name].set_color("#cbd5e1")
    ax.tick_params(colors="#475569", labelsize=9)


def event_marks(ax, events, shift=0):
    if "home_start" in events and "home_end" in events:
        ax.axvspan(events["home_start"]-shift, events["home_end"]-shift, color="#2563eb", alpha=0.09)
    if "program_start" in events:
        ax.axvline(events["program_start"]-shift, color="#2563eb", lw=1.1, ls="--", alpha=0.8)
    if "fault_detected" in events:
        ax.axvline(events["fault_detected"]-shift, color="#dc2626", lw=1.3, ls="--")


def norm_traces(t, norms, links):
    zero = [link for j, link in enumerate(links) if np.all(norms[:, j] == 0)]
    traces = []
    if zero:
        traces.append((t, np.zeros_like(t), "索引 " + "/".join(map(str, zero)) + "（全程为零）", "#94a3b8", "--"))
    for j, link in enumerate(links):
        if link not in zero:
            traces.append((t, norms[:, j], f"索引 {link}", LINK_COLORS[link % len(LINK_COLORS)], "-"))
    return traces, zero


def overview(t, norms, links, events, arm, stats):
    fig = plt.figure(figsize=(13.8, 9.5), facecolor="#f8fafc")
    fig.text(0.07, 0.95, f"{arm}  /  全臂接触估计", fontsize=24, weight="bold", color="#0f172a")
    fig.text(0.07, 0.912, f"{stats['start_local'][:10]}  ·  {stats['start_local'][11:19]}–{stats['end_local'][11:19]}  America/Los_Angeles", fontsize=11, color="#64748b")
    for x, value, label in zip((0.07, 0.31, 0.55, 0.79),
                               (f"{t[-1]:.2f} s", f"{stats['rate_hz']:.2f} Hz", f"{len(t):,}", str(len(links))),
                               ("记录时长", "实际采样频率", "采样时刻数", "连杆条目数")):
        fig.text(x, 0.855, value, fontsize=21, weight="bold", color="#0f172a")
        fig.text(x, 0.823, label, fontsize=10, color="#64748b")
    gs = fig.add_gridspec(2, 1, left=0.07, right=0.96, top=0.745, bottom=0.14, hspace=0.62)
    ax, zoom = fig.add_subplot(gs[0]), fig.add_subplot(gs[1])
    traces, zero = norm_traces(t, norms, links)
    for x, y, name, color, ls in traces:
        ax.plot(x, y, label=name, color=color, lw=1.2, ls=ls)
    ax.set_title("完整记录 · 各连杆力估计模长", loc="left", fontsize=13, pad=13, weight="bold")
    ax.legend(loc="upper left", ncol=5, fontsize=8.5, frameon=False, bbox_to_anchor=(0, 1.02))
    ax.set_xlim(t[0], t[-1])
    ax.set_ylim(-1, float(norms.max())*1.17 + 1)
    ax.set_ylabel("力模长（名义 N）", fontsize=10)
    ax.set_xlabel("距采集开始的主机时间（s）", fontsize=10)
    event_marks(ax, events)
    if "program_start" in events:
        shift = events["program_start"]
        end = events.get("program_end", shift + 2)
        ax.axvspan(shift-1, end+1, color="#64748b", alpha=0.07)
        ax.annotate("运动程序所在片段 ↓", (shift, norms.max()*1.08), ha="right", fontsize=9, color="#475569")
        for x, y, _, color, ls in traces:
            zoom.plot(x-shift, y, color=color, lw=1.5, ls=ls, marker=".", markersize=3)
        zoom.set_xlim(-1, end-shift+1)
        event_marks(zoom, events, shift)
        zoom.set_xlabel("相对运动程序启动的主机时间（s）", fontsize=10)
        fault = events.get("fault_detected")
        if fault is not None:
            zoom.annotate(f"故障检测：{fault-shift:.3f} s\n303010 · 非实时轨迹不可行",
                          xy=(fault-shift, 0.93), xycoords=("data", "axes fraction"),
                          xytext=(0.99, 1.04), textcoords="axes fraction", ha="right", va="bottom",
                          color="#b91c1c", fontsize=9,
                          arrowprops={"arrowstyle": "-", "color": "#dc2626", "lw": 0.8})
    else:
        for x, y, _, color, ls in traces:
            zoom.plot(x, y, color=color, lw=1, ls=ls)
        zoom.set_xlabel("距采集开始的主机时间（s）")
    zoom.set_title("运动片段放大 · 圆点为原始采样；蓝色阴影为 Home 阶段", loc="left", fontsize=12, pad=17, weight="bold")
    zoom.set_ylabel("力模长（名义 N）", fontsize=10)
    for axis in (ax, zoom):
        style_axis(axis)
    fig.text(0.07, 0.073, "数据为 SDK 接触估计；单位沿用 CSV 的名义 N / m，尚未标定。各连杆使用各自局部坐标系。", fontsize=9, color="#64748b")
    fig.text(0.07, 0.046, "零输出不能证明没有接触；连杆索引尚未对应到具体部位。日志事件按主机时钟对齐，故障瞬间可能短于约 50 ms 的采样间隔。", fontsize=8.7, color="#64748b")
    return fig


def components(t, a, links, events, arm):
    fig, axes = plt.subplots(len(links), 2, figsize=(14, 16), sharex=True, facecolor="white")
    fig.subplots_adjust(left=0.08, right=0.97, top=0.905, bottom=0.075, hspace=0.45, wspace=0.22)
    fig.suptitle(f"{arm}  /  各连杆 XYZ 分量", fontsize=23, weight="bold", x=0.08, ha="left", y=0.973)
    fig.text(0.08, 0.938, "左：力估计（名义 N）    右：接触位置估计（名义 mm，连杆局部坐标）", fontsize=11, color="#64748b")
    for j, link in enumerate(links):
        force, position = a[:, j, :3], a[:, j, 3:].copy()*1000
        no_output = np.all(a[:, j, :] == 0, axis=1)
        position[no_output] = np.nan
        for axis, color in enumerate(XYZ_COLORS):
            axes[j, 0].plot(t, force[:, axis], color=color, lw=0.9, label="F"+"xyz"[axis])
            axes[j, 1].plot(t, position[:, axis], color=color, lw=0.9, label="P"+"xyz"[axis])
        axes[j, 0].set_ylabel(f"索引 {link}\n名义 N", fontsize=10)
        axes[j, 1].set_ylabel("名义 mm", fontsize=9)
        if no_output.all():
            axes[j, 0].text(0.5, 0.65, "全程零输出", transform=axes[j, 0].transAxes, ha="center", color="#64748b", fontsize=10)
            axes[j, 1].text(0.5, 0.5, "全程全零；无可用位置输出", transform=axes[j, 1].transAxes, ha="center", color="#64748b", fontsize=10)
        for ax in axes[j]:
            style_axis(ax)
            event_marks(ax, events)
            ax.set_xlim(0, t[-1])
        if j == 0:
            for ax in axes[j]:
                ax.legend(loc="upper right", ncol=3, frameon=False, fontsize=9)
    for ax in axes[-1]:
        ax.set_xlabel("距采集开始的主机时间（s）", fontsize=10)
    fig.text(0.08, 0.038, "位置图在六个值全零的采样处留空，避免将零占位误画成接触点移动；原始 CSV 保留全部数值。", fontsize=9, color="#64748b")
    fig.text(0.08, 0.022, "蓝色阴影：Home 阶段；红虚线：故障检测。不同连杆的 XYZ 分量不可直接相加。", fontsize=9, color="#64748b")
    return fig


def interactive_page(path, t, a, norms, links, events, arm, stats):
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    titles = ("完整记录 · 力估计模长（可框选放大、点击图例筛选）", "运动片段放大",
              "所选连杆 · 力估计 XYZ（用右上方下拉菜单切换索引）", "所选连杆 · 接触位置 XYZ（局部坐标）")
    fig = make_subplots(rows=4, cols=1, vertical_spacing=0.085, subplot_titles=titles)
    traces, _ = norm_traces(t, norms, links)
    for row in (1, 2):
        for x, y, name, color, ls in traces:
            fig.add_trace(go.Scatter(x=x.tolist(), y=y.tolist(), name=name,
                                    legendgroup=name, showlegend=row == 1,
                                    line={"color": color, "width": 1.4, "dash": "dash" if ls == "--" else "solid"},
                                    hovertemplate="%{x:.3f} s<br>%{y:.3f}（名义 N）<extra>%{fullData.name}</extra>"), row=row, col=1)
    detail_indices = list(range(len(fig.data), len(fig.data)+6))
    default_link = 5 if 5 in links else links[0]
    def detail_values(j):
        position = a[:, j, 3:].copy()*1000
        position[np.all(a[:, j, :] == 0, axis=1)] = np.nan
        return [a[:, j, k].tolist() for k in range(3)] + [[None if not np.isfinite(v) else float(v) for v in position[:, k]] for k in range(3)]
    detail = detail_values(links.index(default_link))
    for k, y in enumerate(detail):
        fig.add_trace(go.Scatter(x=t.tolist(), y=y, name=("F" if k < 3 else "P")+"xyz"[k % 3],
                                connectgaps=False, line={"color": XYZ_COLORS[k % 3], "width": 1.3}), row=3 if k < 3 else 4, col=1)
    fig.update_layout(updatemenus=[{"buttons": [{"label": f"索引 {link}", "method": "restyle", "args": [{"y": detail_values(j)}, detail_indices]} for j, link in enumerate(links)],
                                   "active": links.index(default_link), "x": 1, "y": 1.06, "xanchor": "right", "yanchor": "bottom"}])
    for row in range(1, 5):
        fig.update_xaxes(title_text="距采集开始的主机时间（s）", showgrid=True, gridcolor="#e2e8f0", row=row, col=1)
        fig.update_yaxes(title_text="名义 mm" if row == 4 else "名义 N", gridcolor="#e2e8f0", row=row, col=1)
        if "home_start" in events and "home_end" in events:
            fig.add_vrect(x0=events["home_start"], x1=events["home_end"], fillcolor="#2563eb", opacity=0.08, line_width=0, row=row, col=1)
        if "fault_detected" in events:
            fig.add_vline(x=events["fault_detected"], line_dash="dash", line_color="#dc2626", row=row, col=1)
    if "program_start" in events:
        fig.update_xaxes(range=[events["program_start"]-1, events.get("program_end", events["program_start"]+2)+1], row=2, col=1)
    fig.update_layout(height=1420, template="plotly_white", font={"family": "Noto Sans CJK SC, sans-serif", "color": "#0f172a"},
                      margin={"l": 75, "r": 35, "t": 100, "b": 50}, legend={"orientation": "h", "y": 1.07},
                      hovermode="x unified", paper_bgcolor="white")
    body = fig.to_html(full_html=False, include_plotlyjs=True, config={"displaylogo": False, "scrollZoom": True})
    zero_links = [str(item["link_index"]) for item in stats["links"] if item["all_values_zero"]]
    zero_note = f"索引 {'、'.join(zero_links)} 本次全程为零；" if zero_links else ""
    cards = "".join(f'<div class="card"><b>{value}</b><span>{label}</span></div>' for value, label in
                    ((f"{stats['duration_s']:.2f} s", "记录时长"), (f"{stats['rate_hz']:.2f} Hz", "实际频率"), (f"{stats['samples']:,}", "采样时刻"), (str(len(links)), "连杆条目")))
    document = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>{html.escape(arm)} 全臂接触估计</title><style>body{{font-family:system-ui,sans-serif;background:#f1f5f9;color:#0f172a;margin:0;padding:28px}}main{{max-width:1450px;margin:auto}}h1{{margin:0 0 8px}}p{{line-height:1.7;color:#475569}}.cards{{display:grid;grid-template-columns:repeat(4,1fr);gap:16px;margin:24px 0}}.card{{padding:20px;background:white;border-radius:12px}}.card b{{display:block;font-size:27px}}.card span{{color:#64748b;font-size:13px}}.chart{{background:white;border-radius:14px;overflow:hidden}}@media(max-width:700px){{body{{padding:12px}}.cards{{grid-template-columns:repeat(2,1fr)}}}}</style>
<main><h1>{html.escape(arm)} · 全臂接触估计</h1><p>{html.escape(stats['start_local'])} 至 {html.escape(stats['end_local'])}<br>离线数据；可缩放、悬停查看数值，使用下拉菜单切换连杆。蓝色阴影为 Home 阶段，红虚线为故障检测。</p>
<div class="cards">{cards}</div><div class="chart">{body}</div><p>{zero_note}零输出不能证明没有接触。各连杆使用各自局部坐标，不能直接相加。单位沿用 CSV 名义 N/m，尚未标定。位置图在力和位置全零的采样处留空。事件按主机时钟对齐，采样间隔约 50 ms。</p></main></html>'''
    path.write_text(document, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--arm", default="ARM_1", choices=("ARM_1", "ARM_2"))
    parser.add_argument("--interactive", action="store_true")
    args = parser.parse_args()
    font = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    if font.exists():
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=str(font)).get_name()
    # Type 3 embeds glyph outlines and supports this system's CJK font collection.
    plt.rcParams.update({"axes.unicode_minus": False, "pdf.fonttype": 3, "savefig.facecolor": "auto"})
    links, a, host, robot = load_csv(args.csv, args.arm)
    t = (host-host[0])/1e9
    norms = np.linalg.norm(a[:, :, :3], axis=2)
    events = load_events(args.csv, int(host[0]))
    stats = {
        "source": str(args.csv.resolve()), "arm": args.arm, "samples": len(t), "rows": len(t)*len(links),
        "duration_s": float(t[-1]), "rate_hz": float((len(t)-1)/t[-1]),
        "start_local": datetime.fromtimestamp(int(host[0])/1e9, ZONE).isoformat(),
        "end_local": datetime.fromtimestamp(int(host[-1])/1e9, ZONE).isoformat(),
        "host_interval_median_ms": float(np.median(np.diff(host))/1e6),
        "host_interval_max_ms": float(np.max(np.diff(host))/1e6),
        "robot_timestamps_increase": bool(np.all(np.diff(robot)>0)),
        "events_seconds_from_capture_start": events,
        "links": [{"link_index": link, "all_values_zero": bool(np.all(a[:, j, :]==0)),
                   "force_norm_max_nominal_N": float(norms[:, j].max()),
                   "force_norm_mean_nominal_N": float(norms[:, j].mean()),
                   "force_norm_peak_time_s": float(t[norms[:, j].argmax()])} for j, link in enumerate(links)],
    }
    figs = [overview(t, norms, links, events, args.arm, stats), components(t, a, links, events, args.arm)]
    output = {}
    for name, fig in zip(("overview", "components"), figs):
        path = args.csv.with_suffix(f".{name}.png")
        fig.savefig(path, dpi=180)
        output[name] = str(path.resolve())
    pdf = args.csv.with_suffix(".report.pdf")
    with PdfPages(pdf) as pages:
        for fig in figs:
            pages.savefig(fig)
            plt.close(fig)
    output["pdf"] = str(pdf.resolve())
    if args.interactive:
        page = args.csv.with_suffix(".visualization.html")
        interactive_page(page, t, a, norms, links, events, args.arm, stats)
        output["interactive"] = str(page.resolve())
    stats["outputs"] = output
    args.csv.with_suffix(".visualization.json").write_text(json.dumps(stats, ensure_ascii=False, indent=2)+"\n")
    print(json.dumps(stats, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
