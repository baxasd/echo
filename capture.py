"""Capture raw ADC data from an IWR6843 + DCA1000EVM.

    python capture.py

Writes the raw ADC bytes exactly as streamed (int16 LE). Lost packets are
filled with zeros so frames stay aligned. Frame layout follows radar.cfg.
"""

import socket
import struct
import time

import serial

PORT = "COM6"
SECONDS = 10
OUT_PATH = "capture.bin"
CFG_PATH = "radar.cfg"

PC_IP = "192.168.33.30"
DCA_ADDRESS = ("192.168.33.180", 4096)
CONFIG_PORT = 4096
DATA_PORT = 4098


def dca_command(sock, code, payload=b""):
    """Send one DCA1000 command and return the status from its reply (0 = OK)."""
    packet = struct.pack("<HHH", 0xA55A, code, len(payload)) + payload + struct.pack("<H", 0xEEAA)
    sock.sendto(packet, DCA_ADDRESS)
    reply = sock.recv(2048)
    return struct.unpack("<H", reply[4:6])[0]


def radar_command(uart, line):
    """Send one CLI line and return the radar's reply (waits for Done/Error)."""
    uart.reset_input_buffer()
    uart.write((line + "\n").encode())
    reply = b""
    deadline = time.time() + 3
    while time.time() < deadline and b"Done" not in reply and b"Error" not in reply:
        reply += uart.read(256)
    return reply.decode(errors="ignore")


def capture(port, seconds, out_path, cfg_path="radar.cfg"):
    """Record `seconds` of raw ADC data to `out_path`. Returns (bytes, lost_bytes)."""
    with open(cfg_path) as f:
        commands = [l.strip() for l in f if l.strip() and not l.startswith("%")]

    uart = serial.Serial(port, 115200, timeout=0.2)
    config_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    config_socket.bind((PC_IP, CONFIG_PORT))
    config_socket.settimeout(2)
    data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16 * 1024 * 1024)
    data_socket.bind((PC_IP, DATA_PORT))
    data_socket.settimeout(0.2)

    expected = 0
    lost = 0
    try:
        # DCA1000: stop leftover run, connect, 2-lane LVDS / Ethernet / 16-bit, 1472 B packets.
        dca_command(config_socket, 0x06)
        for code, payload in [(0x09, b""),
                              (0x03, bytes([1, 2, 1, 2, 3, 30])),
                              (0x0B, struct.pack("<HHH", 1472, 3125, 0))]:
            if dca_command(config_socket, code, payload) != 0:
                raise RuntimeError(f"DCA1000 command 0x{code:02X} failed")

        # Radar: send every cfg line except sensorStart.
        radar_command(uart, "sensorStop")
        for line in commands:
            if line.startswith("sensorStart"):
                continue
            reply = radar_command(uart, line)
            if "Done" not in reply:
                raise RuntimeError(f"Radar rejected '{line}': {reply.strip()}")

        # Start recording before the radar starts chirping, so no frame is missed.
        with open(out_path, "wb") as out:
            if dca_command(config_socket, 0x05) != 0:
                raise RuntimeError("DCA1000 record start failed")
            reply = radar_command(uart, "sensorStart")
            if "Done" not in reply:
                raise RuntimeError(f"sensorStart failed: {reply.strip()}")

            end_time = time.time() + seconds
            while time.time() < end_time:
                try:
                    packet = data_socket.recv(2048)
                except socket.timeout:
                    continue
                # Header: 4-byte sequence number, 6-byte count of bytes sent before this packet.
                byte_count = int.from_bytes(packet[4:10], "little")
                if byte_count < expected:
                    continue  # duplicate / late packet
                if byte_count > expected:
                    out.write(bytes(byte_count - expected))  # lost packets -> zeros
                    lost += byte_count - expected
                out.write(packet[10:])
                expected = byte_count + len(packet) - 10
    finally:
        radar_command(uart, "sensorStop")
        dca_command(config_socket, 0x06)
        uart.close()
        config_socket.close()
        data_socket.close()

    return expected, lost


if __name__ == "__main__":
    total, lost = capture(PORT, SECONDS, OUT_PATH, CFG_PATH)
    print(f"saved {total} bytes, lost {lost} bytes -> {OUT_PATH}")
