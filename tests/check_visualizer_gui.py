"""Isolated Tk layout checks; never opens a serial port."""

import json
from pathlib import Path
import time
import tkinter as tk
from unittest.mock import patch

from PIL import ImageGrab

from tools.obstacle_protocol import parse_snapshot
from tools.radar_visualizer import RadarVisualizer


def leaf_widgets(widget):
    for child in widget.winfo_children():
        yield from leaf_widgets(child)
    if not widget.winfo_children():
        yield widget


def main():
    root = tk.Tk()
    root.attributes("-topmost", True)
    output = Path(__file__).parent / "artifacts"
    output.mkdir(exist_ok=True)
    try:
        with patch("tools.radar_visualizer.list_ports.comports", return_value=[]):
            app = RadarVisualizer(root)
        app.running = True
        app.connection_var.set("测试数据")
        root.minsize(940, 620)
        seq = 0
        for width, height in ((940, 640), (1240, 800)):
            root.geometry(f"{width}x{height}+0+0")
            root.update()
            for view in ("field", "vehicle"):
                seq += 1
                report = {
                    "v": 1, "seq": seq, "scan_seq": seq, "valid": True,
                    "age_ms": 0, "front_m": 0.5, "blocked": False,
                    "obstacles": [{"x_m": 0.4 + i * 0.02, "y_m": 0.4 + i * 0.02} for i in range(8)],
                    "rx_frames": 100, "bad_frames": 0, "rx_dropped": 0, "uart_errors": 0,
                }
                with app.data_lock:
                    app.snapshot.update(parse_snapshot(json.dumps(report).encode()), time.monotonic())
                app.view_var.set(view)
                app.refresh_display()
                root.lift()
                root.update()
                time.sleep(0.1)
                labels = app.canvas.find_withtag("obstacle-label")
                assert len(labels) == 8, (width, height, view, len(labels))
                boxes = [app.canvas.bbox(label) for label in labels]
                for index, box in enumerate(boxes):
                    assert 0 <= box[0] < box[2] <= app.canvas.winfo_width(), box
                    assert 0 <= box[1] < box[3] <= app.canvas.winfo_height(), box
                    for other in boxes[index + 1:]:
                        assert box[2] < other[0] or box[0] > other[2] or box[3] < other[1] or box[1] > other[3], (box, other)
                x, y = root.winfo_rootx(), root.winfo_rooty()
                for widget in leaf_widgets(root):
                    if not widget.winfo_ismapped():
                        continue
                    wx, wy = widget.winfo_rootx(), widget.winfo_rooty()
                    assert x <= wx and y <= wy, widget
                    assert wx + widget.winfo_width() <= x + root.winfo_width(), widget
                    assert wy + widget.winfo_height() <= y + root.winfo_height(), widget
                artifact = output / f"visualizer-{width}x{height}-{view}.png"
                screen = ImageGrab.grab()
                sx = screen.width / root.winfo_screenwidth()
                sy = screen.height / root.winfo_screenheight()
                bounds = tuple(round(value) for value in (x * sx, y * sy, (x + root.winfo_width()) * sx, (y + root.winfo_height()) * sy))
                screen.crop(bounds).save(artifact)
                print(artifact)
        app.disconnect()
        assert not app.canvas.find_withtag("obstacle"), "disconnect left obstacle icons"
        print("Both views and window sizes passed; labels fit and do not overlap.")
    finally:
        root.destroy()


if __name__ == "__main__":
    main()
