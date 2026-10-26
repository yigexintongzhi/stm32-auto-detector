"""离线复现：用固件同一套阈值逻辑回放历史数据集，交叉验证判定一致性。

复现的规则与 Core/Src/app.c 中 STATE_RESULT 完全一致：
    量程 A: y = (index>=120) ? values[119] : 最后一个点
            x = (y - B_A) / K_A
            x < 3      -> LOD
            x <= 18    -> ok
            否则       -> over（触发稀释流程，切换到量程 B）
    量程 B: y = (index>=20) ? values[19] : 最后一个点
            x = (y - B_B) / K_B
            x > 65     -> over range
            x >= 15    -> ok
            否则       -> error
数据集 CSV 首行可写预期结果用于自动比对：# expect=ok
（也可使用 lod / over / error）
"""

import os
from dataclasses import dataclass
from typing import List, Optional, Tuple

from .calib import FIRMWARE_CAL

FINAL_INDEX = {"A": 119, "B": 19}


@dataclass
class ReplayResult:
    path: str
    stage: str
    category: str       # ok / lod / over / error
    x: float
    y: int
    expect: Optional[str] = None

    @property
    def matched(self) -> Optional[bool]:
        return None if self.expect is None else self.expect == self.category

    def line(self) -> str:
        s = (f"{self.path}: Result {self.stage} [{self.category}] "
             f"x={self.x:.2f} ppm (y={self.y})")
        if self.expect is not None:
            s += f"  expect={self.expect} -> {'MATCH' if self.matched else 'MISMATCH'}"
        return s


def load_dataset_csv(path: str) -> Tuple[List[int], Optional[str]]:
    """读取 capture 保存的数据集 CSV，返回 (values, expect)。"""
    values: List[int] = []
    expect: Optional[str] = None
    with open(path, newline="", encoding="utf-8") as f:
        for raw in f:
            raw = raw.strip()
            if raw.startswith("#"):
                if raw.startswith("# expect="):
                    expect = raw.split("=", 1)[1].strip()
                continue
            if raw.lower().startswith("index"):
                continue
            parts = [p.strip() for p in raw.split(",") if p.strip()]
            if len(parts) >= 2:
                try:
                    values.append(int(parts[1]))
                except ValueError:
                    pass
    return values, expect


def firmware_result(values: List[int], stage: str) -> ReplayResult:
    """与固件 STATE_RESULT 分支一一对应的离线复现。"""
    cfg = FIRMWARE_CAL[stage]
    final = FINAL_INDEX[stage]
    y = values[final] if len(values) >= final + 1 else (values[-1] if values else 0)
    x = (y - cfg["B"]) / cfg["K"]
    if stage == "A":
        if x < cfg["min"]:
            cat = "lod"
        elif x <= cfg["max"]:
            cat = "ok"
        else:
            cat = "over"
    else:
        if x > cfg["max"]:
            cat = "over"
        elif x >= cfg["min"]:
            cat = "ok"
        else:
            cat = "error"
    return ReplayResult(path="", stage=stage, category=cat, x=x, y=y)


def replay_file(path: str) -> ReplayResult:
    values, expect = load_dataset_csv(path)
    stage = "A" if len(values) >= 100 else "B"
    result = firmware_result(values, stage)
    result.path = os.path.basename(path)
    result.expect = expect
    return result


def replay_dir(dir_path: str) -> List[ReplayResult]:
    results = []
    for name in sorted(os.listdir(dir_path)):
        if name.endswith(".csv"):
            results.append(replay_file(os.path.join(dir_path, name)))
    return results


def report(results: List[ReplayResult]) -> None:
    checked = [r for r in results if r.expect is not None]
    for r in results:
        print(r.line())
    if checked:
        n_match = sum(1 for r in checked if r.matched)
        print(f"\nconsistency: {n_match}/{len(checked)} matched")
