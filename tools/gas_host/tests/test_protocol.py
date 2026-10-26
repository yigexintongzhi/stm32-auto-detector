"""协议解析器单元测试：逐字节喂入也必须得到与整块喂入相同的事件。"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gashost.protocol import Dataset, FrameParser, Result, Sample, Text


class TestFrameParser(unittest.TestCase):
    def setUp(self):
        self.parser = FrameParser()

    def test_stream_line(self):
        events = self.parser.feed(b"STREAM,1,42,328\r\n")
        self.assertEqual(len(events), 1)
        self.assertIsInstance(events[0], Sample)
        self.assertEqual((events[0].stage, events[0].index, events[0].value),
                         (1, 42, 328))

    def test_result_line(self):
        events = self.parser.feed(b"Result A: x = 12.34 ppm, y=171\r\n")
        self.assertIsInstance(events[0], Result)
        self.assertEqual(events[0].stage, "A")
        self.assertIn("12.34", events[0].text)

    def test_dataset_burst(self):
        raw = ("\r\n--- Countdown Data ---\r\n"
               "10,11,12\r\n"
               "--- End of Data ---\r\n").encode()
        events = self.parser.feed(raw)
        ds = [e for e in events if isinstance(e, Dataset)]
        self.assertEqual(len(ds), 1)
        self.assertEqual(ds[0].values, [10, 11, 12])

    def test_byte_by_byte_feed(self):
        raw = b"STREAM,2,7,99\r\n--- Countdown Data ---\r\n1,2\r\n--- End of Data ---\r\n"
        p = FrameParser()
        all_events = []
        for i in range(len(raw)):
            all_events.extend(p.feed(raw[i:i + 1]))
        kinds = [type(e).__name__ for e in all_events]
        self.assertEqual(kinds, ["Sample", "Dataset"])
        self.assertEqual(all_events[1].values, [1, 2])

    def test_stage_inference(self):
        p = FrameParser()
        events = p.feed(b"--- Countdown Data ---\r\n" +
                        b",".join([b"100"] * 120) +
                        b"\r\n--- End of Data ---\r\n")
        self.assertEqual(events[0].stage, "A")

    def test_text_line(self):
        events = self.parser.feed(b"OK: Pump ON\r\n")
        self.assertIsInstance(events[0], Text)


if __name__ == "__main__":
    unittest.main()
