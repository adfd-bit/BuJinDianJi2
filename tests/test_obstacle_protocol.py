import json
import queue
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from tools.obstacle_protocol import (
    LineDecoder, MAX_LINE_BYTES, SnapshotState, parse_snapshot, vehicle_to_field,
)


def report_bytes(**changes):
    report = {
        "v": 1, "seq": 1, "scan_seq": 1, "valid": True, "age_ms": 0,
        "front_m": 0.5, "blocked": False,
        "obstacles": [{"x_m": 0.2, "y_m": 0.4}],
        "rx_frames": 100, "bad_frames": 0, "rx_dropped": 0, "uart_errors": 0,
    }
    report.update(changes)
    return json.dumps(report, separators=(",", ":")).encode("utf-8")


class ProtocolTests(unittest.TestCase):
    def test_all_split_positions_and_multiple_reports(self):
        wire = report_bytes() + b"\r\n" + report_bytes(seq=2, scan_seq=2) + b"\n"
        for split in range(len(wire) + 1):
            decoder = LineDecoder()
            reports = decoder.feed(wire[:split]) + decoder.feed(wire[split:])
            self.assertEqual([item.seq for item in reports], [1, 2], split)
            self.assertEqual(decoder.errors, 0)

    def test_invalid_lines_do_not_hide_next_report(self):
        decoder = LineDecoder()
        reports = decoder.feed(b"debug text\n\xff\n{}\n" + report_bytes() + b"\n")
        self.assertEqual(len(reports), 1)
        self.assertEqual(decoder.errors, 3)

    def test_exact_line_limit_and_oversize_discard(self):
        decoder = LineDecoder()
        payload = report_bytes()
        maximum = payload + b" " * (MAX_LINE_BYTES - len(payload))
        self.assertEqual(len(decoder.feed(maximum + b"\n")), 1)
        self.assertEqual(decoder.feed(b"x" * (MAX_LINE_BYTES + 1)), [])
        self.assertTrue(decoder.discarding)
        self.assertEqual(decoder.feed(report_bytes() + b"\n"), [])
        self.assertEqual(len(decoder.feed(report_bytes(seq=2) + b"\n")), 1)
        self.assertEqual(decoder.errors, 1)

    def test_strict_fields_and_numeric_types(self):
        invalid_changes = (
            {"v": True}, {"seq": True}, {"seq": -1}, {"scan_seq": 2**32},
            {"valid": 1}, {"age_ms": -1}, {"front_m": True},
            {"blocked": 0}, {"front_m": float("nan")},
            {"front_m": 10**400}, {"rx_frames": 1.0},
            {"bad_frames": None}, {"obstacles": {}},
            {"obstacles": [{"x_m": float("inf"), "y_m": 1}]},
            {"obstacles": [{"x_m": False, "y_m": 1}]},
            {"obstacles": [{"x_m": 6.21, "y_m": 1}]},
            {"obstacles": [{"x_m": 0, "y_m": -6.21}]},
            {"obstacles": [{"x_m": 0, "y_m": 1, "id": 2}]},
            {"obstacles": [{"x_m": 0, "y_m": 1}] * 9}, {"extra": 1},
            {"age_ms": None}, {"valid": False}, {"front_m": None}, {"blocked": None},
        )
        for changes in invalid_changes:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                parse_snapshot(report_bytes(**changes))
        with self.assertRaises(ValueError):
            parse_snapshot(report_bytes().replace(b'"seq":1', b'"seq":1,"seq":2'))
        with self.assertRaises(ValueError):
            parse_snapshot(b"[]")

    def test_nulls_and_uint32_boundary_are_valid(self):
        report = parse_snapshot(report_bytes(
            seq=0xFFFFFFFF, scan_seq=0, valid=False, age_ms=None,
            front_m=None, blocked=None, obstacles=[],
        ))
        self.assertEqual(report.seq, 0xFFFFFFFF)
        self.assertEqual(report.obstacles, ())

    def test_incomplete_line_cannot_update(self):
        decoder = LineDecoder()
        self.assertEqual(decoder.feed(report_bytes()), [])
        self.assertEqual(len(decoder.feed(b"\n")), 1)

    def test_rounding_does_not_recompute_blocked(self):
        snapshot = parse_snapshot(report_bytes(front_m=0.5, blocked=True))
        self.assertTrue(snapshot.blocked)


class FreshnessTests(unittest.TestCase):
    def test_host_elapsed_is_added_to_mcu_age(self):
        state = SnapshotState()
        state.update(parse_snapshot(report_bytes(age_ms=400)), 10.0)
        self.assertTrue(state.obstacles(10.099))
        self.assertFalse(state.obstacles(10.101))

    def test_repeat_scan_cannot_extend_lifetime(self):
        state = SnapshotState()
        state.update(parse_snapshot(report_bytes()), 10.0)
        state.update(parse_snapshot(report_bytes(seq=2, age_ms=0)), 10.4)
        self.assertTrue(state.obstacles(10.49))
        self.assertFalse(state.obstacles(10.51))
        state.update(parse_snapshot(report_bytes(seq=3, age_ms=0)), 11.0)
        self.assertFalse(state.obstacles(11.0))
        state.update(parse_snapshot(report_bytes(seq=4, scan_seq=2)), 11.1)
        self.assertTrue(state.obstacles(11.1))

    def test_larger_mcu_age_is_respected(self):
        state = SnapshotState()
        state.update(parse_snapshot(report_bytes()), 10.0)
        state.update(parse_snapshot(report_bytes(seq=2, age_ms=480)), 10.1)
        self.assertFalse(state.obstacles(10.13))

    def test_invalid_empty_disconnect_and_reconnect_clear(self):
        state = SnapshotState()
        state.update(parse_snapshot(report_bytes()), 0.0)
        self.assertEqual(len(state.obstacles(0.0)), 1)
        state.update(parse_snapshot(report_bytes(seq=2, obstacles=[])), 0.05)
        self.assertFalse(state.obstacles(0.05))
        state.update(parse_snapshot(report_bytes(seq=3, valid=False, obstacles=[], front_m=None, blocked=None)), 0.1)
        self.assertFalse(state.obstacles(0.1))
        state.clear()
        self.assertIsNone(state.latest)
        state.update(parse_snapshot(report_bytes(scan_seq=1)), 20.0)
        self.assertTrue(state.obstacles(20.0))

    def test_never_scanned_stays_empty(self):
        state = SnapshotState()
        state.update(parse_snapshot(report_bytes(valid=False, age_ms=None, obstacles=[], front_m=None, blocked=None)), 0.0)
        self.assertIsNone(state.age_ms(1.0))
        self.assertFalse(state.obstacles(1.0))

    def test_projection_has_no_second_mounting_offset(self):
        x, y = vehicle_to_field(0.106, 1.106, 0.168, 0.168, 0)
        self.assertAlmostEqual(x, 0.274)
        self.assertAlmostEqual(y, 1.274)
        x, y = vehicle_to_field(1, 0, 0.168, 0.168, 90)
        self.assertAlmostEqual(x, 0.168)
        self.assertAlmostEqual(y, 1.168)


class ReaderSessionTests(unittest.TestCase):
    def make_reader(self):
        return SimpleNamespace(
            session=2, running=True, data_lock=threading.Lock(),
            snapshot=SnapshotState(), bytes_since_rate=0, report_errors=0,
            received_reports=0, reports_since_rate=0, events=queue.Queue(),
        )

    def test_late_read_from_old_connection_cannot_replace_snapshot(self):
        from tools.radar_visualizer import RadarVisualizer

        reader = self.make_reader()
        reader.snapshot.update(parse_snapshot(report_bytes(seq=99)), 0)
        port = Mock()
        port.read.return_value = report_bytes(seq=1) + b"\n"
        RadarVisualizer._read_serial(reader, port, threading.Event(), 1)
        self.assertEqual(reader.snapshot.latest.seq, 99)
        self.assertEqual(reader.received_reports, 0)

    def test_reader_keeps_latest_report_and_tags_disconnect(self):
        import serial
        from tools.radar_visualizer import RadarVisualizer

        reader = self.make_reader()
        port = Mock()
        port.read.side_effect = [
            report_bytes(seq=1) + b"\n" + report_bytes(seq=2, scan_seq=2) + b"\n",
            serial.SerialException("test disconnect"),
        ]
        RadarVisualizer._read_serial(reader, port, threading.Event(), 2)
        self.assertEqual(reader.snapshot.latest.seq, 2)
        self.assertEqual(reader.received_reports, 2)
        self.assertEqual(reader.events.get_nowait(), (2, "test disconnect"))


if __name__ == "__main__":
    unittest.main()
