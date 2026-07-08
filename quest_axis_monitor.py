#!/usr/bin/env python3
"""
Quest-only wrist axis monitor for RB teleoperation calibration.

This script does not connect to the robot. It binds the same Quest input ports
used by Open-Teach, receives raw Unity world-space hand keypoints, and prints
which Unity axis changes when you move your wrist.

Usage:
    cd ~/Open-Teach
    conda activate rbpodo
    python quest_axis_monitor.py --host 192.168.50.49

Quest flow:
    1. Start the Quest APK and connect/stream to this PC IP.
    2. Green / hand mode: confirm connected and raw wrist coordinates.
    3. Move your wrist to the intended start point.
    4. Blue / arm mode: first frame becomes the delta origin.
    5. Move forward / left / up and watch Unity dX/dY/dZ.
"""

import argparse
import csv
import math
import re
import signal
import sys
import time
from pathlib import Path

import numpy as np
import zmq

DEFAULT_HOST = "192.168.50.49"
DEFAULT_KEYPOINT_PORT = 8087
DEFAULT_BUTTON_PORT = 8095
DEFAULT_TELEOP_PORT = 8100
DEFAULT_LOG = "/tmp/quest_axis_log.csv"
NUM_KEYPOINTS = 24
WRIST_INDEX = 0

GREEN = "\033[92m"
BLUE = "\033[94m"
YELLOW = "\033[93m"
RED = "\033[91m"
DIM = "\033[2m"
RESET = "\033[0m"

# Quest(Unity 왼손계) → Robot base(오른손계) 좌표 변환
# Robot: X+전방  Y+좌  Z+상
# Quest: Z+전방  X-좌  Y+상
R_QUEST_TO_ROBOT = np.array([
    [ 0,  0,  1],
    [-1,  0,  0],
    [ 0,  1,  0],
], dtype=float)


def color(text, code):
    return f"{code}{text}{RESET}"


def make_pull_socket(ctx, host, port, name):
    sock = ctx.socket(zmq.PULL)
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.setsockopt(zmq.RCVTIMEO, 1)
    endpoint = f"tcp://{host}:{port}"
    try:
        sock.bind(endpoint)
    except zmq.ZMQError as exc:
        raise RuntimeError(
            f"Failed to bind {name} socket at {endpoint}. "
            f"Is teleop.py or another monitor already running?"
        ) from exc
    return sock


def parse_keypoints(packet):
    text = packet.decode().strip()
    if ":" not in text:
        raise ValueError(f"Unexpected keypoint packet: {text[:80]!r}")

    prefix, payload = text.split(":", 1)
    data_type = "absolute" if prefix.startswith("absolute") else "relative"

    # Unity SerializeVector3List emits "x,y,z|...|x,y,z:" with a trailing colon.
    # Extract numeric tokens directly so trailing separators or format drift do not
    # break calibration while the user is wearing the headset.
    numeric_tokens = re.findall(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", payload)
    values = [float(v) for v in numeric_tokens]

    if len(values) < NUM_KEYPOINTS * 3:
        raise ValueError(f"Expected {NUM_KEYPOINTS} keypoints, got {len(values) // 3}")
    return data_type, np.asarray(values[: NUM_KEYPOINTS * 3], dtype=float).reshape(NUM_KEYPOINTS, 3)


def decode_mode(packet, previous):
    if packet is None:
        return previous
    token = packet.decode(errors="ignore").strip()
    # Open-Teach maps Low -> STOP, anything else -> CONT.
    if token == "Low":
        return "STOP"
    if token:
        return "CONT"
    return previous


def axis_summary(delta):
    axes = ["X", "Y", "Z"]
    idx = int(np.argmax(np.abs(delta)))
    sign = "+" if delta[idx] >= 0 else "-"
    return axes[idx], sign, float(delta[idx])


def format_vec(vec, unit="m"):
    return f"X={vec[0]:+.4f}{unit}  Y={vec[1]:+.4f}{unit}  Z={vec[2]:+.4f}{unit}"


def main():
    parser = argparse.ArgumentParser(description="Quest-only Unity wrist axis monitor")
    parser.add_argument("--host", default=DEFAULT_HOST, help="PC WiFi IP that Quest connects to")
    parser.add_argument("--keypoint-port", type=int, default=DEFAULT_KEYPOINT_PORT)
    parser.add_argument("--button-port", type=int, default=DEFAULT_BUTTON_PORT)
    parser.add_argument("--teleop-port", type=int, default=DEFAULT_TELEOP_PORT)
    parser.add_argument("--log", default=DEFAULT_LOG, help="CSV log path")
    parser.add_argument("--print-hz", type=float, default=10.0, help="Console refresh rate")
    args = parser.parse_args()

    ctx = zmq.Context()
    stop = False

    def handle_signal(_sig, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    key_sock = make_pull_socket(ctx, args.host, args.keypoint_port, "keypoint")
    btn_sock = make_pull_socket(ctx, args.host, args.button_port, "button")
    teleop_sock = make_pull_socket(ctx, args.host, args.teleop_port, "teleop")

    log_path = Path(args.log)
    log_file = log_path.open("w", newline="")
    writer = csv.writer(log_file)
    writer.writerow([
        "time_s", "event", "data_type", "mode",
        "wrist_x_m", "wrist_y_m", "wrist_z_m",
        "origin_x_m", "origin_y_m", "origin_z_m",
        "quest_dx_cm", "quest_dy_cm", "quest_dz_cm",
        "robot_dx_cm", "robot_dy_cm", "robot_dz_cm",
        "dominant_axis", "dominant_sign", "dominant_cm",
    ])

    print(color("Quest axis monitor ready", GREEN))
    print(f"host={args.host}  keypoints={args.keypoint_port}  button={args.button_port}  teleop={args.teleop_port}")
    print(f"CSV log: {log_path}")
    print()
    print("Expected flow:")
    print("  1. Quest Stream on -> green CONNECTED appears")
    print("  2. Index/green hand mode -> move wrist to desired origin")
    print("  3. Middle/blue arm mode -> first frame becomes origin")
    print("  4. Move wrist forward / left / up and read dX/dY/dZ")
    print()

    connected = False
    stream_origin = None
    arm_origin = None
    last_mode = "UNKNOWN"
    mode = "UNKNOWN"
    last_print = 0.0
    last_frame_time = 0.0
    sample_count = 0
    start = time.time()

    try:
        while not stop:
            now = time.time()
            key_packet = None
            btn_packet = None
            teleop_packet = None

            try:
                key_packet = key_sock.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                pass
            try:
                btn_packet = btn_sock.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                pass
            try:
                teleop_packet = teleop_sock.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                pass

            mode = decode_mode(teleop_packet, mode)
            _button_mode = decode_mode(btn_packet, None)

            if key_packet is None:
                if now - last_print >= 1.0:
                    age = now - last_frame_time if last_frame_time else math.inf
                    status = "waiting for Quest packets" if not connected else f"no keypoint packet for {age:.1f}s"
                    print("\r" + color(status.ljust(90), YELLOW), end="", flush=True)
                    last_print = now
                time.sleep(0.002)
                continue

            try:
                data_type, keypoints = parse_keypoints(key_packet)
            except Exception as exc:
                print("\n" + color(f"Parse error: {exc}", RED), flush=True)
                continue

            wrist = keypoints[WRIST_INDEX]
            sample_count += 1
            last_frame_time = now

            if not connected:
                connected = True
                stream_origin = wrist.copy()
                print("\n" + color("CONNECTED: first Quest keypoint packet received", GREEN))
                print(f"stream origin wrist: {format_vec(stream_origin)}")
                print(color("Switch to blue / arm mode when you want to set the delta origin.", BLUE))

            event = "sample"
            if mode != last_mode:
                print()
                if mode == "STOP":
                    print(color("GREEN/HAND mode detected. Move wrist to your desired origin.", GREEN))
                    arm_origin = None
                    event = "mode_stop"
                elif mode == "CONT":
                    arm_origin = wrist.copy()
                    print(color("BLUE/ARM mode detected. Delta origin set from this first frame.", BLUE))
                    print(f"arm origin wrist:    {format_vec(arm_origin)}")
                    event = "mode_cont_origin"
                else:
                    print(color(f"Mode changed: {mode}", YELLOW))
                    event = "mode_unknown"
                last_mode = mode

            origin = arm_origin if arm_origin is not None else stream_origin
            delta = wrist - origin
            delta_robot = R_QUEST_TO_ROBOT @ delta
            dom_axis, dom_sign, dom_value = axis_summary(delta_robot)
            elapsed = now - start

            writer.writerow([
                f"{elapsed:.4f}", event, data_type, mode,
                f"{wrist[0]:.6f}", f"{wrist[1]:.6f}", f"{wrist[2]:.6f}",
                f"{origin[0]:.6f}", f"{origin[1]:.6f}", f"{origin[2]:.6f}",
                f"{delta[0] * 100:.3f}", f"{delta[1] * 100:.3f}", f"{delta[2] * 100:.3f}",
                f"{delta_robot[0] * 100:.3f}", f"{delta_robot[1] * 100:.3f}", f"{delta_robot[2] * 100:.3f}",
                dom_axis, dom_sign, f"{dom_value * 100:.3f}",
            ])

            if now - last_print >= 1.0 / args.print_hz:
                mode_label = color("BLUE/ARM", BLUE) if mode == "CONT" else color("GREEN/HAND", GREEN) if mode == "STOP" else color(mode, YELLOW)
                line = (
                    f"{mode_label}  "
                    f"Quest Δ X={delta[0]*100:+6.1f}cm Y={delta[1]*100:+6.1f}cm Z={delta[2]*100:+6.1f}cm  |  "
                    f"Robot Δ X={delta_robot[0]*100:+6.1f}cm Y={delta_robot[1]*100:+6.1f}cm Z={delta_robot[2]*100:+6.1f}cm  "
                    f"[{dom_axis}{dom_sign} {dom_value * 100:+.1f}cm]"
                )
                # ANSI color codes make length awkward; clear line generously.
                print("\r" + line + " " * 20, end="", flush=True)
                last_print = now

            time.sleep(0.002)
    finally:
        print("\n")
        print(f"samples: {sample_count}")
        print(f"CSV log saved: {log_path}")
        log_file.close()
        key_sock.close(0)
        btn_sock.close(0)
        teleop_sock.close(0)
        ctx.term()


if __name__ == "__main__":
    main()
