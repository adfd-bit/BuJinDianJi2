"""Capture and validate real STM32 reports; no simulated data is injected."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time

import serial

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.obstacle_protocol import LineDecoder


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    decoder = LineDecoder()
    reports = []
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with serial.Serial(args.port, 230400, timeout=0.1) as port, args.output.open("w", encoding="ascii") as output:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            for report in decoder.feed(port.read(1024)):
                reports.append(report)
                encoded = json.dumps({"v": 1, **asdict(report)}, separators=(",", ":"))
                output.write(encoded + "\n")
                if len(reports) <= 3:
                    print(encoded)
    valid = [report for report in reports if report.valid]
    summary = {
        "port": args.port,
        "reports": len(reports),
        "valid_reports": len(valid),
        "distinct_valid_scans": len({report.scan_seq for report in valid}),
        "max_obstacles": max((len(report.obstacles) for report in valid), default=0),
        "line_errors": decoder.errors,
        "first_rx_frames": reports[0].rx_frames if reports else None,
        "last_rx_frames": reports[-1].rx_frames if reports else None,
        "last_bad_frames": reports[-1].bad_frames if reports else None,
        "last_rx_dropped": reports[-1].rx_dropped if reports else None,
        "last_uart_errors": reports[-1].uart_errors if reports else None,
    }
    print(json.dumps(summary, separators=(",", ":")))
    return 0 if valid else 2 if reports else 1


if __name__ == "__main__":
    raise SystemExit(main())
