"""标定、回放、异常检测与模拟协议的逻辑测试。"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gashost.calib import demo_points, linear_fit
from gashost.plotter import find_plateaus
from gashost.protocol import Dataset, FrameParser, Result, Sample
from gashost.replay import firmware_result
from gashost.simulator import build_script


class TestAnalysis(unittest.TestCase):
    def test_calibration_demo_recovers_firmware_coefficients(self):
        points = demo_points()
        fit_a = linear_fit("A", points)
        fit_b = linear_fit("B", points)
        self.assertIsNotNone(fit_a)
        self.assertIsNotNone(fit_b)
        self.assertGreater(fit_a.r_squared, 0.99)
        self.assertGreater(fit_b.r_squared, 0.99)
        self.assertLess(abs(fit_a.K - 15.59213), 0.2)
        self.assertLess(abs(fit_b.K - 2.49739), 0.02)

    def test_firmware_threshold_categories(self):
        self.assertEqual(firmware_result([0] * 120, "A").category, "lod")
        self.assertEqual(firmware_result([136] * 120, "A").category, "ok")
        self.assertEqual(firmware_result([300] * 120, "A").category, "over")
        self.assertEqual(firmware_result([120] * 20, "B").category, "error")
        self.assertEqual(firmware_result([171] * 20, "B").category, "ok")
        self.assertEqual(firmware_result([272] * 20, "B").category, "over")

    def test_plateau_detection(self):
        values = [1, 2, 3, 3, 3, 3, 3, 4]
        self.assertEqual(find_plateaus(values, min_len=5), [(2, 7)])
        self.assertEqual(find_plateaus(values, min_len=6), [])

    def test_normal_simulator_protocol(self):
        raw = b"".join(payload for _, payload in build_script("normal", fast=True))
        events = FrameParser().feed(raw)
        self.assertEqual(sum(isinstance(e, Sample) for e in events), 120)
        datasets = [e for e in events if isinstance(e, Dataset)]
        results = [e for e in events if isinstance(e, Result)]
        self.assertEqual(len(datasets), 1)
        self.assertEqual(datasets[0].stage, "A")
        self.assertEqual(len(datasets[0].values), 120)
        self.assertEqual(len(results), 1)

    def test_over_simulator_switches_to_stage_b(self):
        raw = b"".join(payload for _, payload in build_script("over", fast=True))
        events = FrameParser().feed(raw)
        datasets = [e for e in events if isinstance(e, Dataset)]
        results = [e for e in events if isinstance(e, Result)]
        self.assertEqual([d.stage for d in datasets], ["A", "B"])
        self.assertEqual([r.stage for r in results], ["A", "B"])
        self.assertEqual(sum(isinstance(e, Sample) for e in events), 140)


if __name__ == "__main__":
    unittest.main()
