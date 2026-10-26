#!/usr/bin/env python3
"""gas_host: 气体检测仪 PC 上位机工具。

子命令：
    gui        启动图形上位机（不带子命令时也默认启动 GUI）
    capture    串口采集 + 实时曲线 + CSV 落盘（--simulate 可无设备演示）
    calibrate  标定点分段线性拟合，输出 K/B/R^2 并与固件系数对比
    replay     用固件同一套阈值逻辑离线回放数据集，交叉验证判定一致性
    selftest   运行协议解析单元测试

示例：
    python gas_host.py capture --simulate --scenario over
    python gas_host.py capture --port COM3 --out runs/run1
    python gas_host.py calibrate --csv calib_points.csv
    python gas_host.py calibrate --demo
    python gas_host.py replay --dir runs/run1/data
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gashost.capture import CaptureSession, default_out_dirs


def cmd_gui(args) -> None:
    from gashost.gui import run_gui
    run_gui(demo=args.demo)


def cmd_capture(args) -> None:
    if args.screenshot:
        import matplotlib
        matplotlib.use("Agg")
    from gashost.plotter import LivePlotter

    out_dir, log_dir = default_out_dirs(args.out)

    if args.simulate:
        from gashost.simulator import SimulatedSerial
        stream = SimulatedSerial(scenario=args.scenario, fast=True)
        print(f"[CAPTURE] simulate scenario={args.scenario} (no device)")
    else:
        if not args.port:
            raise SystemExit("error: --port is required (or use --simulate)")
        import serial
        stream = serial.Serial(args.port, args.baud, timeout=0.2)

    plotter = LivePlotter(title=f"gas detector  ({args.scenario or args.port})")
    session = CaptureSession(stream, out_dir, log_dir,
                             on_event=lambda ev: plotter.on_events([ev]))

    if args.simulate:
        # 模拟模式：读循环跑完脚本自然结束，不依赖 Ctrl+C
        import threading
        t = threading.Thread(target=session.run, daemon=True)
        t.start()
        while t.is_alive():
            if args.screenshot:
                t.join(timeout=0.05)
            else:
                plotter.tick()
        t.join()
    else:
        session.run()

    if args.screenshot:
        plotter.save(args.screenshot)
    else:
        try:
            import matplotlib.pyplot as plt
            print("[CAPTURE] done. close the plot window to exit.")
            plt.show(block=True)
        except KeyboardInterrupt:
            pass


def cmd_calibrate(args) -> None:
    from gashost.calib import demo_points, load_points, report
    points = demo_points() if args.demo else load_points(args.csv)
    if not points:
        raise SystemExit("error: no calibration points")
    report(points)


def cmd_replay(args) -> None:
    from gashost.replay import replay_dir, replay_file, report
    if args.dir:
        report(replay_dir(args.dir))
    elif args.csv:
        report([replay_file(args.csv)])
    else:
        raise SystemExit("error: --csv or --dir is required")


def cmd_selftest(args) -> None:
    import unittest
    tests = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tests")
    suite = unittest.defaultTestLoader.discover(tests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    raise SystemExit(0 if result.wasSuccessful() else 1)


def main() -> None:
    if len(sys.argv) == 1:
        from gashost.gui import run_gui
        run_gui()
        return

    parser = argparse.ArgumentParser(description="gas detector host tool")
    sub = parser.add_subparsers(dest="command", required=True)

    p_gui = sub.add_parser("gui", help="launch the graphical host application")
    p_gui.add_argument("--demo", choices=["normal", "over", "comm_fail"],
                       help="start a simulated scenario after launch")
    p_gui.set_defaults(func=cmd_gui)

    p_cap = sub.add_parser("capture", help="serial capture + live plot + CSV logging")
    p_cap.add_argument("--port", help="serial port, e.g. COM3")
    p_cap.add_argument("--baud", type=int, default=115200)
    p_cap.add_argument("--out", default="runs/demo", help="output base dir")
    p_cap.add_argument("--simulate", action="store_true", help="use simulated device")
    p_cap.add_argument("--scenario", default="normal",
                       choices=["normal", "over", "comm_fail"],
                       help="simulated scenario")
    p_cap.add_argument("--screenshot", help="save final plot to png and exit")
    p_cap.set_defaults(func=cmd_capture)

    p_cal = sub.add_parser("calibrate", help="piecewise linear calibration fit")
    p_cal.add_argument("--csv", help="calibration points csv: x_ppm,y_adc")
    p_cal.add_argument("--demo", action="store_true",
                       help="fit synthetic points generated from firmware constants")
    p_cal.set_defaults(func=cmd_calibrate)

    p_rep = sub.add_parser("replay", help="offline firmware-logic replay")
    p_rep.add_argument("--csv", help="single dataset csv")
    p_rep.add_argument("--dir", help="directory of dataset csvs")
    p_rep.set_defaults(func=cmd_replay)

    p_test = sub.add_parser("selftest", help="run protocol parser unit tests")
    p_test.set_defaults(func=cmd_selftest)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
