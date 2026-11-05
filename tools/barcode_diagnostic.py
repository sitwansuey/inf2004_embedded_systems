"""Decode the optional v1 barcode debug channel; EDR16 telemetry is unchanged."""
import struct

TOPIC = "Robopico/diagnostics/barcode/v1"
SIZE = 192
REASONS = ("none", "ok", "incomplete", "noise", "overflow", "timeout", "pattern", "unknown")
CAPTURE_STATES = ("waiting for white", "ready", "scanning", "held")
NAVIGATION = ("none", "left", "right", "forward", "U-turn")

def decode(payload):
    if not isinstance(payload, (bytes, bytearray)) or len(payload) != SIZE:
        raise ValueError("Barcode diagnostic must contain exactly192 bytes")
    if payload[:2] != b"\xbc\x01" or payload[2] not in (1, 2) or payload[3] > 7:
        raise ValueError("Invalid barcode diagnostic version, kind or status")
    phase, fault, power, nav, symbol, flags, capture, capture_runs, mask, stored, valid, algorithm = payload[64:76]
    if (phase > 12 or fault > 7 or power > 100 or nav > 4 or
        symbol and not 65 <= symbol <= 90 or (flags & 128 and not flags & 64) or capture > 3 or
        mask > 7 or stored > 26 or valid > 7 or algorithm > 6):
        raise ValueError("Invalid barcode diagnostic fields")
    names = ("sequence", "time_ms", "result_id", "result_ms", "started_ms", "rejected_count",
             "decoded_count", "max_gap_ms", "run_count", "max_run_ms", "dropped", "sample_ms")
    result = dict(zip(names, struct.unpack_from("<12I", payload, 4)))
    acquisition, left, right = struct.unpack_from("<3i", payload, 52)
    runs = list(struct.unpack_from("<29I", payload, 76))
    result.update(version=1, kind="result" if payload[2] == 2 else "status",
        status=payload[3], reason=REASONS[payload[3]], acquisition_error=acquisition,
        left_counts=left, right_counts=right, phase=phase, fault=fault, power=power,
        navigation=NAVIGATION[nav], decoded_symbol=chr(symbol) if symbol else None,
        stored_symbol=chr(64+stored) if stored else None, reverse=bool(flags & 1),
        junction_abort=bool(flags & 2), scanning=bool(flags & 4),
        black=bool(flags & 8) if valid & 4 else None, fresh_ir=bool(flags & 16),
        armed=bool(flags & 32), capture_state=CAPTURE_STATES[capture], capture_runs=capture_runs,
        scan_line_correction=("left" if flags & 128 else "right") if flags & 64 else "none",
        line_mask=mask, valid_mask=valid, algorithm_error=algorithm,
        runs_ms=runs[:min(result["run_count"], 29)],
        sample_age_ms=(result["time_ms"]-result["sample_ms"]) & 0xFFFFFFFF)
    return result
