"""串口采集会话：读字节 → 解析 → 落盘（原始 log + 数据集 CSV）。"""

import csv
import os
from datetime import datetime
from typing import Callable, List, Optional

from .protocol import Dataset, Event, FrameParser, Result, Sample, Text

# 串口数据源协议：read(size) -> bytes，close()。真实串口和模拟器都实现它。
class ByteStream:
    def read(self, size: int) -> bytes:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError


def default_out_dirs(base_dir: str):
    out_dir = os.path.join(base_dir, "data")
    log_dir = os.path.join(base_dir, "logs")
    os.makedirs(out_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)
    return out_dir, log_dir


class CaptureSession:
    """一次采集会话。事件通过 on_event 回调交给上层（绘图/打印）。"""

    def __init__(self,
                 stream: ByteStream,
                 out_dir: str,
                 log_dir: str,
                 on_event: Optional[Callable[[Event], None]] = None,
                 echo_text: bool = True) -> None:
        self.stream = stream
        self.parser = FrameParser()
        self.on_event = on_event
        self.echo_text = echo_text
        self.datasets: List[Dataset] = []
        self.results: List[Result] = []
        self.running = False

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = os.path.join(log_dir, f"capture_{stamp}.log")
        self.out_dir = out_dir
        self._log_file = open(self.log_path, "wb")

    def _dispatch(self, event: Event) -> None:
        if isinstance(event, Dataset):
            self.datasets.append(event)
            self._write_dataset_csv(event)
        elif isinstance(event, Result):
            self.results.append(event)
        elif isinstance(event, Text) and self.echo_text:
            print(f"[MCU] {event.line}")
        if self.on_event is not None:
            self.on_event(event)

    def _write_dataset_csv(self, ds: Dataset) -> None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.out_dir, f"dataset_{ds.stage}_{stamp}.csv")
        with open(path, "w", newline="", encoding="utf-8") as f:
            f.write(f"# stage={ds.stage} points={len(ds.values)}\n")
            f.write("index,value\n")
            for i, v in enumerate(ds.values):
                f.write(f"{i},{v}\n")
        print(f"[SAVE] {len(ds.values)} points -> {path}")

    def run(self, duration_s: Optional[float] = None) -> None:
        """阻塞读循环。duration_s=None 表示直到 Ctrl+C 或流关闭。"""
        import time
        self.running = True
        start = time.monotonic()
        print(f"[CAPTURE] logging raw bytes to {self.log_path} (Ctrl+C to stop)")
        try:
            while self.running:
                if duration_s is not None and time.monotonic() - start >= duration_s:
                    break
                chunk = self.stream.read(256)
                if chunk:
                    self._log_file.write(chunk)
                    self._log_file.flush()
                    for event in self.parser.feed(chunk):
                        self._dispatch(event)
                elif getattr(self.stream, "eof", False):
                    break
        except KeyboardInterrupt:
            print("\n[CAPTURE] interrupted by user")
        finally:
            self.stop()

    def stop(self) -> None:
        self.running = False
        try:
            self.stream.close()
        except Exception:
            pass
        self._log_file.close()
