# src/kuka_mujoco_sim/sri_codec.py
"""SRI M8128-style binary frame encoder, matching sri_mock_server.cpp.

Frame (31 bytes): AA 55 | 00 1B | PN_H PN_L | 24 bytes (6x float32 LE) | SUM.
SUM = sum of the 24 data bytes (buf[6:30]) & 0xFF. We are the TCP server;
sri_ft_driver connects and (optionally) sends AT+GSD before streaming.
"""
import struct

START_ACK = b'ACK+GSD=OK\r\n'


def build_sri_frame(wrench6, pn):
    """wrench6 = (Fx, Fy, Fz, Mx, My, Mz) in N/Nm. pn = packet number."""
    buf = bytearray(31)
    buf[0] = 0xAA
    buf[1] = 0x55
    buf[2] = 0x00
    buf[3] = 0x1B  # payload length 27 = PN(2) + data(24) + sum(1)
    pn &= 0xFFFF
    buf[4] = (pn >> 8) & 0xFF
    buf[5] = pn & 0xFF
    struct.pack_into('<6f', buf, 6, *wrench6)
    buf[30] = sum(buf[6:30]) & 0xFF
    return bytes(buf)


def is_start_command(data):
    return b'AT+GSD' in data
