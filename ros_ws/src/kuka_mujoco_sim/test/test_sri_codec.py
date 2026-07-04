# test/test_sri_codec.py
import struct
from kuka_mujoco_sim.sri_codec import build_sri_frame, is_start_command, START_ACK

def test_frame_length_and_header():
    f = build_sri_frame((1.0, 2.0, 3.0, 0.1, 0.2, 0.3), 1)
    assert len(f) == 31
    assert f[0] == 0xAA and f[1] == 0x55
    assert f[2] == 0x00 and f[3] == 0x1B  # payload length 27

def test_frame_payload_floats_little_endian():
    w = (1.5, -2.5, 3.5, 0.25, -0.5, 0.75)
    f = build_sri_frame(w, 7)
    # PN big-endian in bytes 4,5
    assert f[4] == 0x00 and f[5] == 0x07
    decoded = struct.unpack('<6f', bytes(f[6:30]))
    for got, exp in zip(decoded, w):
        assert abs(got - exp) < 1e-6

def test_frame_checksum():
    f = build_sri_frame((1.0, 2.0, 3.0, 0.1, 0.2, 0.3), 1)
    expected = sum(f[6:30]) & 0xFF
    assert f[30] == expected

def test_is_start_command():
    assert is_start_command(b'AT+GSD\r\n')
    assert is_start_command(b'junk AT+GSD more')
    assert not is_start_command(b'hello')

def test_start_ack_value():
    assert START_ACK == b'ACK+GSD=OK\r\n'
