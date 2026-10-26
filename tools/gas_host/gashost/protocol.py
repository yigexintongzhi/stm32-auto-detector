"""Firmware UART text protocol parser.

帧格式（与 Core/Src/console.c 保持一致）：
    采样流（LON 开启后倒计时期间每秒一行）:
        STREAM,<stage>,<index>,<value>\\r\\n
    数据集突发（倒计时结束后一次性发送）:
        --- Countdown Data ---\\r\\n
        v0,v1,...,vN\\r\\n
        --- End of Data ---\\r\\n
    结果行:
        Result A: x = 12.34 ppm, y=234\\r\\n
        Result A: x < 3 ppm (LOD), y=..\\r\\n
        Result A: x > 18 ppm (8.7), goto dilution\\r\\n
        Result B: x = .. / x > 65 ppm (over range) / ERROR x < 15 (..)
"""

from dataclasses import dataclass, field
import re
from typing import List, Optional, Union

DATA_START = "--- Countdown Data ---"
DATA_END = "--- End of Data ---"
STREAM_PREFIX = "STREAM,"

RESULT_RE = re.compile(r"^Result ([AB]): (.*)$")


@dataclass
class Sample:
    """单个流式采样点（倒计时期间每秒一个）。"""
    stage: int      # 1 = 低量程流程 A，2 = 高量程流程 B
    index: int      # 倒计时内 0 起始的采样序号
    value: int      # 传感器寄存器原始值（y）


@dataclass
class Dataset:
    """一段完整采集序列（突发帧）。"""
    values: List[int]
    stage: str = "?"    # 按长度推断：120 点 = A，20 点 = B

    def infer_stage(self) -> str:
        if len(self.values) >= 100:
            return "A"
        if 1 <= len(self.values) <= 40:
            return "B"
        return "?"


@dataclass
class Result:
    """结果判定行。"""
    stage: str          # 'A' 或 'B'
    text: str           # "Result X: " 之后的原文


@dataclass
class Text:
    """其他文本行（OK:/Busy: 等回显）。"""
    line: str


Event = Union[Sample, Dataset, Result, Text]


class FrameParser:
    """增量式行解析器：feed 任意字节块，返回解析出的事件列表。"""

    def __init__(self) -> None:
        self._buf = ""
        self._in_data = False
        self._data_values: List[int] = []

    def feed(self, chunk: bytes) -> List[Event]:
        text = chunk.decode("ascii", errors="replace")
        # 统一换行：先合并 \r\n，再按 \n 切行
        self._buf += text.replace("\r\n", "\n")
        events: List[Event] = []
        while True:
            idx = self._buf.find("\n")
            if idx < 0:
                break
            line = self._buf[:idx].rstrip("\r")
            self._buf = self._buf[idx + 1:]
            produced = self._handle_line(line)
            if produced is None:
                continue
            if isinstance(produced, list):
                events.extend(produced)
            else:
                events.append(produced)
        return events

    def _handle_line(self, line: str) -> Optional[Union[Event, List[Event]]]:
        stripped = line.strip()

        if self._in_data:
            if stripped == DATA_END:
                self._in_data = False
                ds = Dataset(values=self._data_values)
                ds.stage = ds.infer_stage()
                self._data_values = []
                return ds
            parts = [p for p in stripped.split(",") if p.strip()]
            try:
                self._data_values.extend(int(p) for p in parts)
            except ValueError:
                pass    # 数据帧内 tolerate 坏行
            return None

        if stripped == DATA_START:
            self._in_data = True
            self._data_values = []
            return None

        if stripped.startswith(STREAM_PREFIX):
            fields = stripped.split(",")
            if len(fields) == 4:
                try:
                    return Sample(stage=int(fields[1]),
                                  index=int(fields[2]),
                                  value=int(fields[3]))
                except ValueError:
                    pass
            return Text(line=line)

        m = RESULT_RE.match(stripped)
        if m:
            return Result(stage=m.group(1), text=m.group(2))

        if stripped == "":
            return None
        return Text(line=line)
