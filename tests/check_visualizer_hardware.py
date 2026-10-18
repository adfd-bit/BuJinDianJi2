"""Verify both Tk views against live reports without injecting snapshots."""

import argparse
from pathlib import Path
import sys
import time
import tkinter as tk

from PIL import ImageGrab

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.radar_visualizer import RadarVisualizer


def capture(root, app, path, view, after_reports):
    root.attributes("-topmost", True)
    root.lift()
    try:
        root.update()
        # Keep the reader from replacing the verified report during the grab.
        with app.data_lock:
            report = app.snapshot.latest
            if (app.received_reports <= after_reports
                    or not app.snapshot.is_fresh(time.monotonic())
                    or not report.obstacles
                    or app.view_var.get() != view
                    or app.values["reports"].get() != str(app.received_reports)
                    or app.values["scan_seq"].get() != str(report.scan_seq)):
                return None
            icons = app.canvas.find_withtag("obstacle")
            if not icons:
                return None
            screen = ImageGrab.grab()
            if not app.snapshot.is_fresh(time.monotonic()):
                return None
            assert app.canvas.find_withtag("obstacle"), f"{view}: no obstacle icons at capture"
            sx = screen.width / root.winfo_screenwidth()
            sy = screen.height / root.winfo_screenheight()
            x, y = root.winfo_rootx(), root.winfo_rooty()
            bounds = tuple(round(value) for value in (
                x * sx, y * sy, (x + root.winfo_width()) * sx, (y + root.winfo_height()) * sy,
            ))
            result = (report.seq, report.scan_seq, len(icons))
        screen.crop(bounds).save(path)
        return result
    finally:
        root.attributes("-topmost", False)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    args = parser.parse_args()
    output = Path(__file__).parent / "artifacts"
    output.mkdir(exist_ok=True)
    root = tk.Tk()
    app = RadarVisualizer(root)
    app.port_var.set(args.port)
    app.connect()
    try:
        for view in ("vehicle", "field"):
            app.view_var.set(view)
            with app.data_lock:
                after_reports = app.received_reports
            path = output / f"hardware-visualizer-{view}.png"
            captured = None
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                captured = capture(root, app, path, view, after_reports)
                if captured is not None:
                    break
                time.sleep(0.02)
            assert captured is not None, f"{view}: no fresh real report with rendered obstacle icons"
            seq, scan_seq, icon_count = captured
            print(f"{path}: seq={seq}, scan_seq={scan_seq}, icons={icon_count}")
        app.disconnect()
        root.update()
        assert not app.canvas.find_withtag("obstacle"), "disconnect left stale icons"
        print("Live COM reports rendered in both views; disconnect cleared icons.")
    finally:
        app.on_close()


if __name__ == "__main__":
    main()
