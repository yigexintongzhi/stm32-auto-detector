"""标定拟合：分段线性回归 y = K*x + B，输出 K/B/R² 并与固件系数对比。

输入 CSV 格式（标定实验记录）：
    x_ppm,y_adc
    3.0,27.3
    6.0,74.1
    ...
量程 A 取 3~18 ppm 的点，量程 B 取 15~65 ppm 的点，分别拟合。
"""

import csv
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

# 与 Core/Inc/app_config.h 保持一致的固件标定系数
FIRMWARE_CAL: Dict[str, Dict[str, float]] = {
    "A": {"K": 15.59213, "B": -19.48466, "min": 3.0, "max": 18.0},
    "B": {"K": 2.49739,  "B": 96.09863,  "min": 15.0, "max": 65.0},
}


@dataclass
class FitResult:
    stage: str
    K: float
    B: float
    r_squared: float
    points_used: int

    def firmware_delta(self) -> Tuple[float, float]:
        fw = FIRMWARE_CAL[self.stage]
        return self.K - fw["K"], self.B - fw["B"]


def load_points(path: str) -> List[Tuple[float, float]]:
    """读取标定点 CSV，返回 [(x_ppm, y_adc), ...]。"""
    points = []
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        for row in reader:
            row = [c.strip() for c in row if c.strip()]
            if len(row) < 2 or row[0].startswith("#") or row[0].lower().startswith("x"):
                continue
            try:
                points.append((float(row[0]), float(row[1])))
            except ValueError:
                continue
    return points


def linear_fit(stage: str, points: List[Tuple[float, float]]) -> Optional[FitResult]:
    """对指定量程内的点做一元线性拟合（y = K*x + B）。"""
    cfg = FIRMWARE_CAL[stage]
    xs = np.array([p[0] for p in points if cfg["min"] <= p[0] <= cfg["max"]],
                  dtype=float)
    ys = np.array([p[1] for p in points if cfg["min"] <= p[0] <= cfg["max"]],
                  dtype=float)
    if len(xs) < 2:
        return None
    K, B = np.polyfit(xs, ys, 1)
    y_hat = K * xs + B
    ss_res = float(np.sum((ys - y_hat) ** 2))
    ss_tot = float(np.sum((ys - np.mean(ys)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 1.0
    return FitResult(stage=stage, K=float(K), B=float(B),
                     r_squared=r2, points_used=len(xs))


def demo_points(seed: int = 42) -> List[Tuple[float, float]]:
    """按固件系数生成带噪声的仿真标定点，用于演示拟合可复现固件系数。"""
    rng = np.random.default_rng(seed)
    points: List[Tuple[float, float]] = []
    for stage, lo, hi in (("A", 3.0, 14.0), ("B", 19.0, 65.0)):
        cfg = FIRMWARE_CAL[stage]
        for x in np.linspace(lo, hi, 8):
            y = cfg["K"] * x + cfg["B"] + rng.normal(0.0, 1.5)
            points.append((float(x), float(y)))
    return points


def report(points: List[Tuple[float, float]]) -> None:
    print(f"{'stage':<6} {'K':>10} {'B':>10} {'R^2':>8} {'n':>3} "
          f"{'dK(vs FW)':>10} {'dB(vs FW)':>10}")
    for stage in ("A", "B"):
        fit = linear_fit(stage, points)
        if fit is None:
            print(f"{stage:<6} (not enough points in "
                  f"{FIRMWARE_CAL[stage]['min']}~{FIRMWARE_CAL[stage]['max']} ppm)")
            continue
        dK, dB = fit.firmware_delta()
        print(f"{fit.stage:<6} {fit.K:>10.4f} {fit.B:>10.4f} "
              f"{fit.r_squared:>8.4f} {fit.points_used:>3} {dK:>+10.4f} {dB:>+10.4f}")
