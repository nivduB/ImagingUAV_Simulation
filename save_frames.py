"""
EPC901 Polar Frame Capture — RAM buffer mode.

Flow:
  1. Press S → sends 0x01 to transmitter → starts capturing to RAM
  2. Sensor captures 108 frames at ~6,500 fps (one full rotation at 60Hz)
  3. Press D → sends 0x02 to transmitter → dumps all frames over BLE
  4. Python receives 108 × 1024 bytes, saves as .npy files
  5. Run plot_polar.py to reconstruct 360° image

UART frame format from receiver:
  [0xAA] [0x55] [len_lo] [len_hi] [data...]

Each frame = 1024 raw 8-bit bytes (no unpacking needed).

Usage:
  python3 save_frames.py
  python3 save_frames.py --port /dev/tty.usbmodem0010507939193
"""

import serial
import numpy as np
import os
import shutil
import argparse
import time
import sys
import select

# --- Config ---
PORT             = '/dev/tty.usbmodem0010507939193'
BAUD             = 115200
PIXELS_PER_FRAME = 1024
BYTES_PER_FRAME  = 1024   # 8-bit pixels — 1 byte per pixel
MAX_FRAMES       = 108
OUTPUT_DIR       = 'frames'


def clear_frames_dir():
    if os.path.exists(OUTPUT_DIR) and os.listdir(OUTPUT_DIR):
        confirm = input(f"Clear {len(os.listdir(OUTPUT_DIR))} existing frames? (y/n): ")
        if confirm.lower() == 'y':
            shutil.rmtree(OUTPUT_DIR)
    os.makedirs(OUTPUT_DIR, exist_ok=True)


def read_packet(ser: serial.Serial, timeout_s: float = 10.0) -> bytes | None:
    """
    Scan stream for 0xAA 0x55 sync marker, read length-prefixed payload.
    """
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        byte = ser.read(1)
        if not byte:
            continue
        if byte == b'\xAA':
            if ser.read(1) == b'\x55':
                length_bytes = ser.read(2)
                if len(length_bytes) < 2:
                    continue
                length = int.from_bytes(length_bytes, 'little')
                if 1 <= length <= 244:
                    payload = ser.read(length)
                    if len(payload) == length:
                        return payload
    return None


def collect_frame(ser: serial.Serial) -> np.ndarray | None:
    """Collect packets until a full 1024-byte frame is assembled."""
    frame_buffer = bytearray()
    while len(frame_buffer) < BYTES_PER_FRAME:
        packet = read_packet(ser)
        if packet is None:
            print("  [timeout waiting for packet]")
            return None
        frame_buffer.extend(packet)

    # Raw 8-bit pixels — no unpacking needed
    return np.frombuffer(bytes(frame_buffer[:BYTES_PER_FRAME]), dtype=np.uint8).copy()


def send_start(ser: serial.Serial):
    """Send 0x01 — start continuous capture to RAM."""
    ser.write(b'\x01')
    ser.flush()
    print(">>> Sent START (0x01) — sensor capturing to RAM...")


def send_stop(ser: serial.Serial):
    """Send 0x02 — stop capture and dump all frames over BLE."""
    ser.write(b'\x02')
    ser.flush()
    print(">>> Sent STOP (0x02) — waiting for BLE dump...")


def save_frame(pixels: np.ndarray, frame_num: int) -> str:
    path = os.path.join(OUTPUT_DIR, f"frame_{frame_num:05d}.npy")
    np.save(path, pixels)
    return path


def kbhit():
    """Non-blocking check if a key was pressed."""
    return select.select([sys.stdin], [], [], 0)[0] != []


def main():
    parser = argparse.ArgumentParser(description='EPC901 polar frame capture')
    parser.add_argument('--port', type=str, default=PORT, help='Serial port')
    args = parser.parse_args()

    clear_frames_dir()

    ser = serial.Serial(args.port, BAUD, rtscts=False, dsrdtr=False, timeout=1)
    print(f"Connected to {args.port}")
    print(f"Saving to ./{OUTPUT_DIR}/")
    print()
    print("Controls:")
    print("  S + Enter → start capture (spin the sensor first)")
    print("  D + Enter → stop capture and dump frames over BLE")
    print("  Ctrl+C    → exit")
    print()

    frame_num   = 0
    capturing   = False
    dumping     = False

    try:
        while True:
            # Check for keypress
            cmd = input("Command (S=start, D=dump): ").strip().upper()

            if cmd == 'S':
                send_start(ser)
                capturing = True
                dumping   = False
                frame_num = 0

            elif cmd == 'D':
                if not capturing:
                    print("Not currently capturing — start first with S.")
                    continue
                send_stop(ser)
                capturing = False
                dumping   = True

                print(f"Receiving up to {MAX_FRAMES} frames...")
                start_time = time.time()

                while frame_num < MAX_FRAMES:
                    pixels = collect_frame(ser)
                    if pixels is None:
                        print(f"  Timeout after {frame_num} frames.")
                        break

                    path = save_frame(pixels, frame_num)
                    print(f"  frame {frame_num:03d} — "
                          f"min={pixels.min():3d} max={pixels.max():3d} "
                          f"mean={pixels.mean():5.1f} → {path}")
                    frame_num += 1

                elapsed = time.time() - start_time
                print(f"\nReceived {frame_num} frames in {elapsed:.2f}s.")
                print("Run: python3 plot_polar.py")
                dumping = False

    except KeyboardInterrupt:
        print(f"\nStopped. {frame_num} frames saved.")
    finally:
        ser.close()


if __name__ == '__main__':
    main()
