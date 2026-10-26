"""设备模拟器：无实物时生成与固件输出格式一致的串口字节流。

场景：
    normal    量程 A 正常测量（5~14 ppm），结果 ok
    over      量程 A 超量程（>18 ppm）→ 触发稀释 → 量程 B 出结果
    comm_fail 中段模拟 Modbus 失败（固件沿用旧值形成平台段），结果仍按末点判定
"""

import time
from typing import List, Optional, Tuple

import numpy as np

from .calib import FIRMWARE_CAL
from .capture import ByteStream

STAGE_POINTS = {"A": 120, "B": 20}


def _adc_series(stage: str, x_ppm: float, noise: float,
                rng: np.random.Generator,
                comm_fail_span: Optional[Tuple[int, int]] = None) -> List[int]:
    """由浓度真值按固件标定方程生成 ADC 序列（含噪声/通信失败平台）。"""
    cfg = FIRMWARE_CAL[stage]
    n = STAGE_POINTS[stage]
    base = cfg["K"] * x_ppm + cfg["B"]
    drift = np.linspace(0.0, cfg["K"] * 0.3, n)   # 轻微上升漂移，模拟真实采样
    ys = base + drift + rng.normal(0.0, noise, n)
    values = [int(round(v)) for v in ys]
    if comm_fail_span:
        a, b = comm_fail_span
        for i in range(a, min(b, n)):
            values[i] = values[a - 1] if a > 0 else values[0]
    return values


def build_script(scenario: str, fast: bool,
                 seed: int = 7) -> List[Tuple[float, bytes]]:
    """生成 (相对时间秒, 字节块) 脚本。fast=True 时 1 s 压缩为 0.01 s。"""
    rng = np.random.default_rng(seed)
    tick = 0.01 if fast else 1.0
    script: List[Tuple[float, bytes]] = []
    t = 0.0

    def emit(text: str, dt: float = 0.0) -> None:
        nonlocal t
        t += dt
        script.append((t, text.encode("ascii")))

    if scenario == "normal":
        x_true = 11.5
        values = _adc_series("A", x_true, noise=2.0, rng=rng)
        stage_plan = [("A", values)]
    elif scenario == "over":
        x_true = 24.0
        values_a = _adc_series("A", x_true, noise=2.0, rng=rng)
        values_b = _adc_series("B", x_true, noise=2.0, rng=rng)
        stage_plan = [("A", values_a), ("B", values_b)]
    elif scenario == "comm_fail":
        x_true = 9.0
        values = _adc_series("A", x_true, noise=2.0, rng=rng,
                             comm_fail_span=(40, 48))
        stage_plan = [("A", values)]
    else:
        raise ValueError(f"unknown scenario: {scenario}")

    for stage, values in stage_plan:
        stage_no = 1 if stage == "A" else 2
        for i, v in enumerate(values):
            emit(f"STREAM,{stage_no},{i},{v}\r\n", tick)
        burst = ",".join(str(v) for v in values)
        emit("\r\n--- Countdown Data ---\r\n")
        emit(f"{burst}\r\n")
        emit("--- End of Data ---\r\n")

        cfg = FIRMWARE_CAL[stage]
        y = values[-1]
        x = (y - cfg["B"]) / cfg["K"]
        if scenario == "comm_fail" and stage == "A":
            emit(f"Result A: x = {x:.2f} ppm, y={y}\r\n")
        elif x < cfg["min"]:
            emit(f"Result {stage}: x < {cfg['min']:.0f} ppm (LOD), y={y}\r\n")
        elif x <= cfg["max"]:
            emit(f"Result {stage}: x = {x:.2f} ppm, y={y}\r\n")
        elif stage == "A":
            emit(f"Result A: x > 18 ppm ({x:.1f}), goto dilution\r\n")
        else:
            emit(f"Result B: x > 65 ppm (over range), y={y}\r\n")
        emit("\r\n", 0.2)

    return script


class SimulatedSerial(ByteStream):
    """按脚本吐字节的假串口，行为对齐 pyserial 的 read/close。"""

    def __init__(self, scenario: str = "normal", fast: bool = True) -> None:
        self._script = build_script(scenario, fast)
        self._pos = 0
        self._cursor = 0
        self._start = time.monotonic()
        self._open = True

    def read(self, size: int = 256) -> bytes:
        if not self._open or self._pos >= len(self._script):
            return b""
        now = time.monotonic() - self._start
        due = self._script[self._pos][0]
        if now < due:
            time.sleep(min(0.005, max(0.0, due - now)))
            return b""
        _, payload = self._script[self._pos]
        chunk = payload[self._cursor:self._cursor + size]
        self._cursor += len(chunk)
        if self._cursor >= len(payload):
            self._pos += 1
            self._cursor = 0
        return chunk

    @property
    def eof(self) -> bool:
        return self._pos >= len(self._script)

    def close(self) -> None:
        self._open = False
