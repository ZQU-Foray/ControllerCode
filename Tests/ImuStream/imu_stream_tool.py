#!/usr/bin/env python3
"""BMI088 原始数据出口的主机接收与验收工具。

用法：
  # 采集（需要 pyserial）
  python3 imu_stream_tool.py capture --port COM5 --out raw.bin --seconds 600
  # 离线解码并生成验收报告
  python3 imu_stream_tool.py decode --bin raw.bin --csv raw.csv

帧格式与 Libraries/Protocol/imu/ImuFrame.hpp 严格一致。
验收不允许放宽校验、跳过异常帧或用最好片段代替整段结果。

退出码：0 = 验收 PASS；1 = 验收 FAIL；2 = 参数/环境错误。
"""

from __future__ import annotations

import argparse
import csv
import struct
import sys
import time
from pathlib import Path

MAGIC = 0x4952
VERSION = 1
HEADER_SIZE = 40
RECORD_SIZE = 36
CRC_SIZE = 4
MAX_FRAME_SIZE = 512

SENSOR_NAMES = {0: "accel", 1: "gyro"}

HEADER_STRUCT = struct.Struct("<HBBIIIHHIIIfB3x")
RECORD_STRUCT = struct.Struct("<IIIIIIhhhHB3x")

assert HEADER_STRUCT.size == HEADER_SIZE, HEADER_STRUCT.size
assert RECORD_STRUCT.size == RECORD_SIZE, RECORD_STRUCT.size

_CRC32_TABLE: list[int] = []
for _index in range(256):
    _value = _index
    for _ in range(8):
        _value = (_value >> 1) ^ 0xEDB88320 if (_value & 1) else (_value >> 1)
    _CRC32_TABLE.append(_value)


def crc32(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for byte in data:
        crc = _CRC32_TABLE[(crc ^ byte) & 0xFF] ^ (crc >> 8)
    return crc ^ 0xFFFFFFFF


def config_id_fields(config_id: int) -> dict[str, int]:
    return {
        "gyro_bandwidth_register": (config_id >> 24) & 0xFF,
        "gyro_range_register": (config_id >> 16) & 0xFF,
        "accel_conf_register": (config_id >> 8) & 0xFF,
        "accel_range_register": config_id & 0xFF,
    }


class FrameError(Exception):
    """魔数正确但帧内容非法（版本/类型/长度/CRC）。"""


class ResyncNeeded(Exception):
    """当前位置不是帧头，需要按魔数重新同步。"""


def decode_frame(buffer: bytes, offset: int) -> tuple[dict, int]:
    remaining = len(buffer) - offset
    if remaining < HEADER_SIZE + CRC_SIZE:
        raise ResyncNeeded("尾部长度不足")

    if struct.unpack_from("<H", buffer, offset)[0] != MAGIC:
        raise ResyncNeeded("魔数不匹配")

    (magic, version, frame_type, config_id, tick_hz, frame_sequence,
     sample_count, flags, dropped, temperature_sequence, temperature_tick,
     temperature_celsius, temperature_valid) = HEADER_STRUCT.unpack_from(
        buffer, offset)

    if version != VERSION or frame_type != 0:
        raise FrameError(f"版本/类型不匹配：version={version} type={frame_type}")

    frame_size = HEADER_SIZE + sample_count * RECORD_SIZE + CRC_SIZE
    if frame_size > MAX_FRAME_SIZE:
        raise FrameError(f"帧长度越界：{frame_size}")
    if frame_size > remaining:
        raise FrameError("帧数据不完整")

    expected_crc = struct.unpack_from("<I", buffer,
                                      offset + frame_size - CRC_SIZE)[0]
    if crc32(buffer[offset:offset + frame_size - CRC_SIZE]) != expected_crc:
        raise FrameError("CRC 校验失败")

    records = []
    cursor = offset + HEADER_SIZE
    for _ in range(sample_count):
        (sequence, drdy_sequence, drdy_tick, start_tick, completed_tick,
         harvested_tick, x, y, z, record_flags, sensor) = \
            RECORD_STRUCT.unpack_from(buffer, cursor)
        records.append({
            "sequence": sequence,
            "drdy_sequence": drdy_sequence,
            "drdy_tick": drdy_tick,
            "start_tick": start_tick,
            "completed_tick": completed_tick,
            "harvested_tick": harvested_tick,
            "x": x,
            "y": y,
            "z": z,
            "flags": record_flags,
            "sensor": sensor,
        })
        cursor += RECORD_SIZE

    header = {
        "config_id": config_id,
        "tick_hz": tick_hz,
        "frame_sequence": frame_sequence,
        "sample_count": sample_count,
        "flags": flags,
        "dropped": dropped,
        "temperature_sequence": temperature_sequence,
        "temperature_tick": temperature_tick,
        "temperature_celsius": temperature_celsius,
        "temperature_valid": temperature_valid,
    }
    return {"header": header, "records": records}, frame_size


def find_next_magic(buffer: bytes, offset: int) -> int:
    found = buffer.find(struct.pack("<H", MAGIC), offset)
    return len(buffer) if found < 0 else found


def capture(args: argparse.Namespace) -> int:
    try:
        import serial  # type: ignore
    except ImportError:
        print("capture 需要 pyserial：pip install pyserial", file=sys.stderr)
        return 2

    output = Path(args.out)
    with serial.Serial(args.port, args.baud, timeout=0.1) as port:
        if args.pause_first:
            port.write(b"P")
            time.sleep(0.1)
        port.write(b"R")
        port.flush()
        deadline = time.monotonic() + args.seconds
        received = 0
        with output.open("wb") as handle:
            while time.monotonic() < deadline:
                chunk = port.read(4096)
                if not chunk:
                    continue
                handle.write(chunk)
                received += len(chunk)
        port.write(b"P")
        port.flush()

    print(f"已写入 {output}，共 {received} 字节")
    return 0


def decode(args: argparse.Namespace) -> int:
    buffer = Path(args.bin).read_bytes()

    crc_errors = 0
    malformed_frames = 0
    resync_events = 0
    resync_after_first = 0
    truncated_tail_frames = 0
    frame_count = 0
    sequence_gaps = 0
    record_sequence_gaps = {"gyro": 0, "accel": 0}
    record_drdy_gaps = {"gyro": 0, "accel": 0}
    previous_frame_sequence: int | None = None
    tick_hz = 0
    config_id = 0
    sensor_counts = {"gyro": 0, "accel": 0}
    dt_ticks: dict[str, list[int]] = {"gyro": [], "accel": []}
    last_drdy: dict[str, int | None] = {"gyro": None, "accel": None}
    last_sequence: dict[str, int | None] = {"gyro": None, "accel": None}
    last_drdy_sequence: dict[str, int | None] = {"gyro": None, "accel": None}
    dropped_first: int | None = None
    dropped_last: int | None = None
    row_count = 0

    csv_handle = None
    writer = None
    if args.csv:
        csv_handle = Path(args.csv).open("w", newline="", encoding="utf-8")
        writer = csv.writer(csv_handle)
        writer.writerow([
            "frame_sequence", "sensor", "sequence", "drdy_sequence", "drdy_tick",
            "start_tick", "completed_tick", "harvested_tick", "x", "y", "z",
            "flags",
        ])

    offset = 0
    while offset < len(buffer):
        try:
            frame, frame_size = decode_frame(buffer, offset)
        except ResyncNeeded as error:
            if str(error) == "尾部长度不足":
                # 文件末尾剩余字节不足一个帧头：deadline 截断的固有尾部残留。
                truncated_tail_frames += 1
                break
            resync_events += 1
            if frame_count > 0:
                resync_after_first += 1
            next_offset = find_next_magic(buffer, offset + 1)
            offset = next_offset if next_offset > offset else offset + 1
            continue
        except FrameError as error:
            if str(error).startswith("CRC"):
                crc_errors += 1
            elif (str(error) == "帧数据不完整"
                  and len(buffer) - offset < MAX_FRAME_SIZE):
                # 最后一帧被采集 deadline 拦腰截断：捕获固有现象，不计入失败。
                truncated_tail_frames += 1
                break
            else:
                malformed_frames += 1
            next_offset = find_next_magic(buffer, offset + 1)
            offset = next_offset if next_offset > offset else offset + 1
            continue

        header = frame["header"]
        frame_count += 1
        tick_hz = header["tick_hz"]
        config_id = header["config_id"]
        if previous_frame_sequence is not None:
            expected = (previous_frame_sequence + 1) & 0xFFFFFFFF
            if header["frame_sequence"] != expected:
                sequence_gaps += 1
        previous_frame_sequence = header["frame_sequence"]

        if dropped_first is None:
            dropped_first = header["dropped"]
        dropped_last = header["dropped"]

        for record in frame["records"]:
            name = SENSOR_NAMES.get(record["sensor"])
            if name is None:
                malformed_frames += 1
                continue
            sensor_counts[name] += 1
            previous = last_drdy[name]
            if previous is not None:
                dt_ticks[name].append(
                    (record["drdy_tick"] - previous) & 0xFFFFFFFF)
            else:
                pass
            # 逐样本序号连续性（回绕安全）：对应验收 Q1 的 sequence gap。
            prev_seq = last_sequence[name]
            if prev_seq is not None and ((record["sequence"] - prev_seq) & 0xFFFFFFFF) != 1:
                record_sequence_gaps[name] += 1
            last_sequence[name] = record["sequence"]
            prev_drdy_seq = last_drdy_sequence[name]
            if prev_drdy_seq is not None and ((record["drdy_sequence"] - prev_drdy_seq) & 0xFFFFFFFF) != 1:
                record_drdy_gaps[name] += 1
            last_drdy_sequence[name] = record["drdy_sequence"]
            last_drdy[name] = record["drdy_tick"]
            row_count += 1
            if writer is not None:
                writer.writerow([
                    header["frame_sequence"], name, record["sequence"],
                    record["drdy_sequence"], record["drdy_tick"],
                    record["start_tick"], record["completed_tick"],
                    record["harvested_tick"], record["x"], record["y"],
                    record["z"], record["flags"],
                ])

        offset += frame_size

    if csv_handle is not None:
        csv_handle.close()

    print(f"文件：{args.bin}（{len(buffer)} 字节）")
    print(f"有效帧：{frame_count}，样本行：{row_count}")
    print(f"CRC 错误：{crc_errors}，结构非法帧：{malformed_frames}，"
          f"重同步事件：{resync_events}（首帧之后 {resync_after_first}），"
          f"尾部截断帧：{truncated_tail_frames}")
    print(f"帧序号缺口：{sequence_gaps}")
    print(f"tickHz：{tick_hz}，配置 ID：0x{config_id:08X} "
          f"{config_id_fields(config_id)}")
    dropped_delta = 0
    if dropped_first is not None and dropped_last is not None:
        dropped_delta = (dropped_last - dropped_first) & 0xFFFFFFFF
    print(f"累计丢弃：首帧 {dropped_first} → 末帧 {dropped_last}"
          f"（窗口内 +{dropped_delta}）")

    failures = []
    if frame_count == 0:
        failures.append("没有任何有效帧")
    if crc_errors:
        failures.append(f"CRC 错误 {crc_errors}")
    if malformed_frames:
        failures.append(f"结构非法帧 {malformed_frames}")
    if truncated_tail_frames > 1:
        failures.append(f"尾部截断帧 {truncated_tail_frames} > 1")
    if resync_after_first:
        failures.append(f"首帧之后重同步 {resync_after_first} 次")
    if sequence_gaps:
        failures.append(f"帧序号缺口 {sequence_gaps}")
    if dropped_delta:
        failures.append(f"窗口内丢弃 {dropped_delta}")

    for name, nominal_hz in (("gyro", 2000.0), ("accel", 1600.0)):
        if sensor_counts[name] == 0:
            failures.append(f"{name} 无样本")
            continue
        # 回绕安全的时长：逐样本 dt 之和（长捕获会多次绕过 32 位 tick 空间）。
        span_ticks = sum(dt_ticks[name])
        span_seconds = span_ticks / tick_hz if tick_hz else 0.0
        rate = (sensor_counts[name] - 1) / span_seconds if span_seconds > 0 else 0.0
        print(f"{name}：样本 {sensor_counts[name]}，时长 {span_seconds:.3f} s，"
              f"实测速率 {rate:.1f} Hz（标称 {nominal_hz:.0f} Hz）")
        print(f"  样本序号缺口 {record_sequence_gaps[name]}，"
              f"DRDY 序号缺口 {record_drdy_gaps[name]}")
        if record_sequence_gaps[name]:
            failures.append(
                f"{name} 样本序号缺口 {record_sequence_gaps[name]}")
        if record_drdy_gaps[name]:
            failures.append(
                f"{name} DRDY 序号缺口 {record_drdy_gaps[name]}")
        if span_seconds <= 0 or abs(rate - nominal_hz) / nominal_hz > 0.01:
            failures.append(f"{name} 速率偏离标称超过 1%")

        nominal_dt = tick_hz / nominal_hz
        samples = sorted(dt_ticks[name])
        if samples:
            inside = sum(1 for dt in samples
                         if 0.9 * nominal_dt <= dt <= 1.1 * nominal_dt)
            ratio = inside / len(samples)
            print(f"  dt 中位数 {samples[len(samples) // 2] / tick_hz * 1e3:.4f} ms，"
                  f"±10% 内占比 {ratio * 100:.3f}%")
            if ratio < 0.999:
                failures.append(
                    f"{name} dt 落在标称 ±10% 内的比例 {ratio * 100:.3f}% < 99.9%")

    if failures:
        print("验收：FAIL")
        for failure in failures:
            print(f"  - {failure}")
        return 1

    print("验收：PASS")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    capture_parser = subparsers.add_parser("capture", help="从 USB CDC 采集原始字节流")
    capture_parser.add_argument("--port", required=True)
    capture_parser.add_argument("--baud", type=int, default=115200)
    capture_parser.add_argument("--out", required=True)
    capture_parser.add_argument("--seconds", type=float, default=600.0)
    capture_parser.add_argument("--pause-first", action="store_true",
                                help="先发送 'P' 关闭出口再发送 'R'，用于测量开启瞬间")
    capture_parser.set_defaults(func=capture)

    decode_parser = subparsers.add_parser("decode", help="解码并验收原始字节流")
    decode_parser.add_argument("--bin", required=True)
    decode_parser.add_argument("--csv", default=None)
    decode_parser.set_defaults(func=decode)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
