"""STM32 obstacle snapshots and their host-side freshness clock."""

from dataclasses import dataclass
import json
import math
from typing import Optional, Tuple


MAX_LINE_BYTES = 1023
MAX_OBSTACLES = 8
MAX_COORDINATE_M = 6.2
STALE_MS = 500
UINT32_MAX = 0xFFFFFFFF
FIELDS = {
    "v", "seq", "scan_seq", "valid", "age_ms", "front_m", "blocked",
    "obstacles", "rx_frames", "bad_frames", "rx_dropped", "uart_errors",
}


@dataclass(frozen=True)
class Obstacle:
    x_m: float
    y_m: float


@dataclass(frozen=True)
class Snapshot:
    seq: int
    scan_seq: int
    valid: bool
    age_ms: Optional[int]
    front_m: Optional[float]
    blocked: Optional[bool]
    obstacles: Tuple[Obstacle, ...]
    rx_frames: int
    bad_frames: int
    rx_dropped: int
    uart_errors: int


def _uint32(value):
    return type(value) is int and 0 <= value <= UINT32_MAX


def _number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key")
        result[key] = value
    return result


def parse_snapshot(line):
    """Reject invalid reports before they can replace the displayed snapshot."""
    if len(line) > MAX_LINE_BYTES:
        raise ValueError("report exceeds line limit")
    try:
        report = json.loads(line.decode("utf-8"), object_pairs_hook=_object)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("invalid JSON report") from exc
    if type(report) is not dict or set(report) != FIELDS:
        raise ValueError("unexpected report fields")
    if type(report["v"]) is not int or report["v"] != 1:
        raise ValueError("unsupported report version")
    for name in ("seq", "scan_seq", "rx_frames", "bad_frames", "rx_dropped", "uart_errors"):
        if not _uint32(report[name]):
            raise ValueError("invalid counter: " + name)
    if type(report["valid"]) is not bool:
        raise ValueError("invalid valid flag")
    if report["age_ms"] is not None and not _uint32(report["age_ms"]):
        raise ValueError("invalid age")
    if report["front_m"] is not None and (not _number(report["front_m"]) or report["front_m"] < 0):
        raise ValueError("invalid front distance")
    if report["blocked"] is not None and type(report["blocked"]) is not bool:
        raise ValueError("invalid blocked flag")
    if (report["front_m"] is None) != (report["blocked"] is None):
        raise ValueError("incomplete front state")
    raw_obstacles = report["obstacles"]
    if type(raw_obstacles) is not list or len(raw_obstacles) > MAX_OBSTACLES:
        raise ValueError("invalid obstacle list")
    obstacles = []
    for point in raw_obstacles:
        if type(point) is not dict or set(point) != {"x_m", "y_m"}:
            raise ValueError("invalid obstacle fields")
        if not _number(point["x_m"]) or not _number(point["y_m"]):
            raise ValueError("invalid obstacle coordinate")
        if abs(point["x_m"]) > MAX_COORDINATE_M or abs(point["y_m"]) > MAX_COORDINATE_M:
            raise ValueError("obstacle coordinate outside measurement range")
        obstacles.append(Obstacle(float(point["x_m"]), float(point["y_m"])))
    if report["valid"] and report["age_ms"] is None:
        raise ValueError("valid report has no scan age")
    if not report["valid"] and (obstacles or report["front_m"] is not None or report["blocked"] is not None):
        raise ValueError("invalid scan contains obstacle state")
    return Snapshot(
        seq=report["seq"], scan_seq=report["scan_seq"], valid=report["valid"],
        age_ms=report["age_ms"], front_m=report["front_m"], blocked=report["blocked"],
        obstacles=tuple(obstacles), rx_frames=report["rx_frames"],
        bad_frames=report["bad_frames"], rx_dropped=report["rx_dropped"],
        uart_errors=report["uart_errors"],
    )


class LineDecoder:
    def __init__(self):
        self.buffer = bytearray()
        self.discarding = False
        self.errors = 0

    def feed(self, chunk):
        snapshots = []
        for byte in chunk:
            if self.discarding:
                if byte == 10:
                    self.discarding = False
                continue
            if byte == 10:
                line = bytes(self.buffer)
                self.buffer.clear()
                if line.endswith(b"\r"):
                    line = line[:-1]
                try:
                    snapshots.append(parse_snapshot(line))
                except (ValueError, OverflowError, RecursionError):
                    self.errors += 1
            elif len(self.buffer) == MAX_LINE_BYTES:
                self.buffer.clear()
                self.discarding = True
                self.errors += 1
            else:
                self.buffer.append(byte)
        return snapshots


class SnapshotState:
    def __init__(self):
        self.clear()

    def clear(self):
        self.latest = None
        self.scan_seq = None
        self.reference_age_ms = None
        self.reference_at = 0.0

    def update(self, snapshot, received_at):
        age = snapshot.age_ms
        if self.scan_seq == snapshot.scan_seq:
            current_age = self.age_ms(received_at)
            if current_age is not None:
                age = current_age if age is None else max(current_age, age)
        self.scan_seq = snapshot.scan_seq
        self.reference_age_ms = age
        self.reference_at = received_at
        self.latest = snapshot

    def age_ms(self, now):
        if self.reference_age_ms is None:
            return None
        return self.reference_age_ms + max(0.0, now - self.reference_at) * 1000.0

    def is_fresh(self, now):
        age = self.age_ms(now)
        return self.latest is not None and self.latest.valid and age is not None and age < STALE_MS

    def obstacles(self, now):
        return self.latest.obstacles if self.is_fresh(now) else ()


def vehicle_to_field(x_m, y_m, vehicle_x_m, vehicle_y_m, heading_deg):
    """Project car coordinates (right, forward); mounting offsets are in the MCU."""
    heading = math.radians(heading_deg)
    return (
        vehicle_x_m + x_m * math.cos(heading) - y_m * math.sin(heading),
        vehicle_y_m + x_m * math.sin(heading) + y_m * math.cos(heading),
    )
