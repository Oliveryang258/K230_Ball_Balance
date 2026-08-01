"""PC端工具：把K230 TF卡中的ball_run.bin转换为CSV。

本文件只在Windows/PC Python中运行，使用的csv、struct、argparse、pathlib
均为Python标准库；不要把它复制到K230的/src目录。

用法：
    python tools/decode_k230_telemetry.py ball_run.bin
    python tools/decode_k230_telemetry.py ball_run.bin ball_run.csv
"""

import argparse
import csv
import struct
from pathlib import Path


HEADER = b"TM"
VERSION_V2 = 2
VERSION_V3 = 3
PACKET_SIZE_V2 = 64
PACKET_SIZE_V3 = 106
CRC_OFFSET_V2 = 62
CRC_OFFSET_V3 = 104

COLUMNS_V2 = (
    "telemetry_sequence",
    "stm32_tick_ms",
    "vision_frame_id",
    "ball_x_px",
    "error_px",
    "velocity_px_s",
    "p_term_us",
    "i_term_us",
    "d_term_us",
    "control_offset_us",
    "servo_target_us",
    "servo_current_us",
    "guard_state",
    "measurement_status",
    "control_flags",
    "motion_state",
    "motion_flags",
    "motion_link_valid",
    "acc_track_mg",
    "yaw_rate_dps",
    "vibration_level_mg",
    "line_error",
    "left_speed",
    "right_speed",
    "turn_command",
    "motion_sequence",
    "motion_age_ms",
    "lost_grace",
    "lost_age_ms",
    "lost_recovery",
)

COLUMNS_V3 = COLUMNS_V2 + (
    "pid_sum_raw_us",
    "pid_sum_directed_us",
    "acc_filtered_mg",
    "af_raw_us",
    "af_clamped_us",
    "af_slewed_us",
    "yaw_raw_dps",
    "speed_average",
    "speed_scale_x1000",
    "yf_raw_us",
    "yf_clamped_us",
    "yf_slewed_us",
    "hold_pwm_effective_us",
    "feedforward_total_us",
    "servo_prelimit_us",
    "servo_flags",
    "motion_bias_active_us",
    "turn_scale",
    "yaw_handover",
    "turn_preview_raw_us",
    "turn_preview_slewed_us",
)


def crc16_ccitt_false(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def decode_packet(packet):
    """校验并解码一条V2(64字节)或V3(98字节)记录；无效返回None。"""
    if len(packet) < 4:
        return None
    if packet[0:2] != HEADER:
        return None

    version = packet[2]
    pkt_size = packet[3]
    if version == VERSION_V2 and pkt_size == PACKET_SIZE_V2:
        crc_off = CRC_OFFSET_V2
    elif version == VERSION_V3 and pkt_size == PACKET_SIZE_V3:
        crc_off = CRC_OFFSET_V3
    else:
        return None

    if len(packet) < pkt_size:
        return None

    expected_crc = struct.unpack_from("<H", packet, crc_off)[0]
    actual_crc = crc16_ccitt_false(packet[2:crc_off])
    if expected_crc != actual_crc:
        return None

    row = {
        "telemetry_sequence": struct.unpack_from("<H", packet, 4)[0],
        "stm32_tick_ms": struct.unpack_from("<I", packet, 6)[0],
        "vision_frame_id": struct.unpack_from("<H", packet, 10)[0],
        "ball_x_px": struct.unpack_from("<h", packet, 12)[0],
        "error_px": struct.unpack_from("<h", packet, 14)[0],
        "velocity_px_s": struct.unpack_from("<h", packet, 16)[0],
        "p_term_us": struct.unpack_from("<h", packet, 18)[0],
        "i_term_us": struct.unpack_from("<h", packet, 20)[0],
        "d_term_us": struct.unpack_from("<h", packet, 22)[0],
        "control_offset_us": struct.unpack_from("<h", packet, 24)[0],
        "servo_target_us": struct.unpack_from("<H", packet, 26)[0],
        "servo_current_us": struct.unpack_from("<H", packet, 28)[0],
        "guard_state": packet[30],
        "measurement_status": packet[31],
        "control_flags": packet[32],
        "motion_state": packet[33],
        "motion_flags": packet[34],
        "motion_link_valid": packet[35],
        "acc_track_mg": struct.unpack_from("<h", packet, 36)[0],
        "yaw_rate_dps": struct.unpack_from("<h", packet, 38)[0] * 0.1,
        "vibration_level_mg": struct.unpack_from("<H", packet, 40)[0],
        "line_error": struct.unpack_from("<h", packet, 42)[0],
        "left_speed": struct.unpack_from("<h", packet, 44)[0],
        "right_speed": struct.unpack_from("<h", packet, 46)[0],
        "turn_command": struct.unpack_from("<h", packet, 48)[0],
        "motion_sequence": struct.unpack_from("<H", packet, 50)[0],
        "motion_age_ms": struct.unpack_from("<H", packet, 52)[0],
        "lost_grace": packet[54],
        "lost_age_ms": struct.unpack_from("<H", packet, 55)[0],
        "lost_recovery": struct.unpack_from("<H", packet, 57)[0],
    }

    if version == VERSION_V3:
        row.update({
            "pid_sum_raw_us": struct.unpack_from("<h", packet, 64)[0],
            "pid_sum_directed_us": struct.unpack_from("<h", packet, 66)[0],
            "acc_filtered_mg": struct.unpack_from("<h", packet, 68)[0],
            "af_raw_us": struct.unpack_from("<h", packet, 70)[0],
            "af_clamped_us": struct.unpack_from("<h", packet, 72)[0],
            "af_slewed_us": struct.unpack_from("<h", packet, 74)[0],
            "yaw_raw_dps": struct.unpack_from("<h", packet, 76)[0] * 0.1,
            "speed_average": struct.unpack_from("<h", packet, 78)[0],
            "speed_scale_x1000": struct.unpack_from("<H", packet, 80)[0],
            "yf_raw_us": struct.unpack_from("<h", packet, 82)[0],
            "yf_clamped_us": struct.unpack_from("<h", packet, 84)[0],
            "yf_slewed_us": struct.unpack_from("<h", packet, 86)[0],
            "hold_pwm_effective_us": struct.unpack_from("<H", packet, 88)[0],
            "feedforward_total_us": struct.unpack_from("<h", packet, 90)[0],
            "servo_prelimit_us": struct.unpack_from("<h", packet, 92)[0],
            "servo_flags": packet[94],
            "motion_bias_active_us": struct.unpack_from("<b", packet, 95)[0],
            "turn_scale": struct.unpack_from("<H", packet, 96)[0] * 0.001,
            "yaw_handover": struct.unpack_from("<H", packet, 98)[0] * 0.001,
            "turn_preview_raw_us": struct.unpack_from("<h", packet, 100)[0],
            "turn_preview_slewed_us": struct.unpack_from("<h", packet, 102)[0],
        })
    return row


def iter_packets(data):
    """从文件内容中寻找有效帧；支持V2(64字节)和V3(98字节)混合记录。"""
    offset = 0
    invalid_candidates = 0
    while offset + 4 <= len(data):
        if data[offset : offset + 2] != HEADER:
            offset += 1
            continue

        # 先读4字节确定包大小
        if offset + 4 > len(data):
            break
        ver = data[offset + 2]
        sz = data[offset + 3]
        if ver == VERSION_V2 and sz == PACKET_SIZE_V2:
            pkt = PACKET_SIZE_V2
        elif ver == VERSION_V3 and sz == PACKET_SIZE_V3:
            pkt = PACKET_SIZE_V3
        else:
            offset += 1
            continue

        if offset + pkt > len(data):
            break

        row = decode_packet(data[offset : offset + pkt])
        if row is None:
            invalid_candidates += 1
            offset += 1
            continue

        yield row
        offset += pkt

    return invalid_candidates


def convert(input_path, output_path):
    data = input_path.read_bytes()
    rows = list(iter_packets(data))
    columns = COLUMNS_V3

    with output_path.open("w", newline="", encoding="utf-8-sig") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)

    print("input:  {}".format(input_path.resolve()))
    print("output: {}".format(output_path.resolve()))
    print("bytes:  {}".format(len(data)))
    print("valid records: {}".format(len(rows)))
    if rows:
        duration_ms = rows[-1]["stm32_tick_ms"] - rows[0]["stm32_tick_ms"]
        print("duration: {:.3f} s".format(duration_ms / 1000.0))


def main():
    parser = argparse.ArgumentParser(
        description="Decode K230/STM32 64-byte telemetry log to CSV."
    )
    parser.add_argument("input", type=Path, help="TF-card ball_run.bin")
    parser.add_argument(
        "output",
        nargs="?",
        type=Path,
        help="output CSV; default is input name with .csv suffix",
    )
    args = parser.parse_args()

    output_path = args.output or args.input.with_suffix(".csv")
    convert(args.input, output_path)


if __name__ == "__main__":
    main()
