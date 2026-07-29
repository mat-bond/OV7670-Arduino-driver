import sys
import time
from datetime import datetime
from pathlib import Path

import serial
from PIL import Image


PORT = "/dev/cu.usbmodem112201"  # Replace with port to Arduino
BAUD_RATE = 2_000_000

WIDTH = 640
HEIGHT = 480
FRAME_SIZE = WIDTH * HEIGHT * 2

READY_MARKER = b"*RDY*"
IMAGE_MARKER = b"*IMG*"

MARKER_TIMEOUT = 180.0
IDLE_TIMEOUT = 45.0
OVERALL_FRAME_TIMEOUT = 360.0

OUTPUT_DIR = Path(__file__).resolve().parent


def wait_for_serial_marker(
    camera: serial.Serial,
    marker: bytes,
    timeout_seconds: float,
    initial_data: bytes = b"",
) -> bytes:
    received = bytearray(initial_data)
    deadline = time.monotonic() + timeout_seconds

    while time.monotonic() < deadline:
        position = received.find(marker)
        if position >= 0:
            return bytes(received[position + len(marker):])

        available = camera.in_waiting
        chunk = camera.read(available if available else 1)

        if not chunk:
            continue

        received.extend(chunk)

        if b"*BADID*" in received:
            raise RuntimeError("Arduino could not read OV7670 PID/VER.")
        if b"*ERR*" in received:
            raise RuntimeError(
                "Arduino reported a camera initialisation error."
            )
        if b"*INIT*" in received:
            print("*INIT* received.")
            received = received.replace(b"*INIT*", b"")

        if len(received) > 4096:
            received = received[-128:]

    raise RuntimeError(f"Did not receive {marker!r}.")


def receive_complete_rgb565_frame(
    camera: serial.Serial,
    initial_data: bytes,
) -> bytes:
    frame = bytearray(initial_data[:FRAME_SIZE])
    started = time.monotonic()
    last_data = started
    last_percent = -1

    while len(frame) < FRAME_SIZE:
        now = time.monotonic()

        if now - last_data > IDLE_TIMEOUT:
            raise RuntimeError(
                f"Transmission stopped at {len(frame)}/{FRAME_SIZE} bytes."
            )

        if now - started > OVERALL_FRAME_TIMEOUT:
            raise RuntimeError(
                f"Overall timeout at {len(frame)}/{FRAME_SIZE} bytes."
            )

        remaining = FRAME_SIZE - len(frame)
        available = camera.in_waiting
        chunk = camera.read(
            min(remaining, available if available else 1)
        )

        if not chunk:
            continue

        frame.extend(chunk)
        last_data = time.monotonic()

        percent = len(frame) * 100 // FRAME_SIZE
        if percent != last_percent:
            print(
                f"\r{len(frame)}/{FRAME_SIZE} bytes ({percent}%)",
                end="",
                flush=True,
            )
            last_percent = percent

    print()
    return bytes(frame)


def decode_rgb565_with_swapped_bytes(frame: bytes) -> Image.Image:
    if len(frame) != FRAME_SIZE:
        raise ValueError(
            f"Expected {FRAME_SIZE} bytes, received {len(frame)}."
        )

    rgb = bytearray(WIDTH * HEIGHT * 3)
    output_index = 0

    for input_index in range(0, FRAME_SIZE, 2):
        first = frame[input_index]
        second = frame[input_index + 1]

        # Interpret the second received byte as the RGB565 high byte.
        pixel = (second << 8) | first

        red5 = (pixel >> 11) & 0x1F
        green6 = (pixel >> 5) & 0x3F
        blue5 = pixel & 0x1F

        red = (red5 << 3) | (red5 >> 2)
        green = (green6 << 2) | (green6 >> 4)
        blue = (blue5 << 3) | (blue5 >> 2)

        rgb[output_index:output_index + 3] = bytes(
            (red, green, blue)
        )
        output_index += 3

    return Image.frombytes("RGB", (WIDTH, HEIGHT), bytes(rgb))


def capture_and_save_vga_image() -> None:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_file = OUTPUT_DIR / f"OV7670_VGA_{timestamp}.png"

    print("Opening camera...")

    with serial.Serial(
        port=PORT,
        baudrate=BAUD_RATE,
        timeout=0.10,
        write_timeout=2.0,
    ) as camera:
        print("Waiting for *RDY*...")
        trailing = wait_for_serial_marker(
            camera,
            READY_MARKER,
            MARKER_TIMEOUT,
        )

        print("*RDY* received. Sending G...")
        camera.write(b"G")
        camera.flush()

        trailing = wait_for_serial_marker(
            camera,
            IMAGE_MARKER,
            MARKER_TIMEOUT,
            initial_data=trailing,
        )

        print("*IMG* received. Reading 640x480 RGB565 frame...")
        frame = receive_complete_rgb565_frame(camera, trailing)

    image = decode_rgb565_with_swapped_bytes(frame)
    image.save(output_file)
    image.show()

    print(f"Saved VGA image to:\n  {output_file}")


if __name__ == "__main__":
    try:
        capture_and_save_vga_image()
    except Exception as error:
        print(f"\nError: {error}", file=sys.stderr)
        sys.exit(1)