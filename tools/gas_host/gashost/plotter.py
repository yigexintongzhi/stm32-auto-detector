"""实时曲线绘图：STREAM 采样逐点刷新，数据集整段绘制并标记异常。

异常点判定：固件在 Modbus 通信失败时沿用上一次读数，
因此连续 >=5 个相同值视为“通信失败区间”，红圈标出。
"""

from typing import List, Optional

import matplotlib.pyplot as plt

from .protocol import Dataset, Event, Result, Sample

FINAL_INDEX = {"A": 119, "B": 19}
PLATEAU_MIN = 5


def find_plateaus(values: List[int], min_len: int = PLATEAU_MIN):
    """返回等值区间 [(start, end_exclusive), ...]，长度 >= min_len。"""
    runs = []
    i = 0
    n = len(values)
    while i < n:
        j = i + 1
        while j < n and values[j] == values[i]:
            j += 1
        if j - i >= min_len:
            runs.append((i, j))
        i = j
    return runs


class LivePlotter:
    def __init__(self, title: str = "gas detector live capture") -> None:
        self.fig, self.ax = plt.subplots(figsize=(9, 5), facecolor="#fcfcfb")
        self.ax.set_facecolor("#fcfcfb")
        self.ax.set_xlabel("sample index (1 s per point)")
        self.ax.set_ylabel("sensor raw value (y)")
        self.ax.grid(True, color="#e1e0d9", linewidth=0.8, linestyle="-")
        self.fig.canvas.manager.set_window_title(title)

        self.live_line, = self.ax.plot([], [], "o-", color="#2a78d6",
                                       markersize=6, linewidth=2,
                                       label="streamed samples")
        self.ds_line, = self.ax.plot([], [], "-", color="#eb6834", linewidth=2,
                                     label="dataset burst")
        self.final_marker = None
        self.suspect_dots = None
        self.ax.legend(loc="upper left")
        self._title_base = title
        self._auto_scaled = False

    def on_events(self, events: List[Event]) -> None:
        for ev in events:
            if isinstance(ev, Sample):
                xs, ys = self.live_line.get_xdata(), self.live_line.get_ydata()
                xs = list(xs) + [ev.index]
                ys = list(ys) + [ev.value]
                self.live_line.set_xdata(xs)
                self.live_line.set_ydata(ys)
                self._autoscale(xs, ys)
            elif isinstance(ev, Dataset):
                self._plot_dataset(ev)
            elif isinstance(ev, Result):
                self.ax.set_title(f"{self._title_base}  |  Result {ev.stage}: {ev.text}",
                                  fontsize=10)

    def _plot_dataset(self, ds: Dataset) -> None:
        xs = list(range(len(ds.values)))
        self.ds_line.set_xdata(xs)
        self.ds_line.set_ydata(ds.values)

        # 固件实际参与判定的最终采样点（A: 索引 119，B: 索引 19）
        if self.final_marker is not None:
            self.final_marker.remove()
        final_idx = FINAL_INDEX.get(ds.stage, len(ds.values) - 1)
        if final_idx < len(ds.values):
            self.final_marker, = self.ax.plot(
                [final_idx], [ds.values[final_idx]], "v", color="#d03b3b", markersize=9,
                label=f"final sample idx={final_idx}")

        # 通信失败区间（固件沿用旧值的平台段）
        if self.suspect_dots is not None:
            self.suspect_dots.remove()
            self.suspect_dots = None
        plateaus = find_plateaus(ds.values)
        if plateaus:
            px = [i for (a, b) in plateaus for i in range(a, b)]
            py = [ds.values[i] for i in px]
            self.suspect_dots, = self.ax.plot(
                px, py, "o", mfc="none", mec="#d03b3b", mew=1.5, markersize=8,
                label=f"comm-fail plateau x{len(plateaus)}")

        self._autoscale(xs, ds.values)
        if self.final_marker is not None or self.suspect_dots is not None:
            self.ax.legend(loc="upper left")

    def _autoscale(self, xs, ys) -> None:
        if not xs:
            return
        self.ax.set_xlim(min(xs) - 2, max(max(xs), FINAL_INDEX.get("A", 120)) + 5)
        lo, hi = min(ys), max(ys)
        margin = max(5, (hi - lo) * 0.15)
        self.ax.set_ylim(lo - margin, hi + margin)

    def tick(self) -> None:
        self.fig.canvas.draw_idle()
        plt.pause(0.01)

    def save(self, path: str) -> None:
        self.fig.savefig(path, dpi=120, bbox_inches="tight")
        print(f"[PLOT] saved -> {path}")
