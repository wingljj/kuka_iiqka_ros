# test/test_sri_endpoint.py
import socket
import struct
import time
from kuka_mujoco_sim.sri_endpoint import SriEndpoint

def _connect(port):
    c = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    c.settimeout(2.0)
    c.connect(('127.0.0.1', port))
    return c

def test_ack_and_stream():
    ep = SriEndpoint(listen_port=0, require_start=True)  # 0 = ephemeral
    ep.start()
    try:
        port = ep.port
        c = _connect(port)
        assert ep.wait_for_client(2.0)
        c.sendall(b'AT+GSD\r\n')
        # read ACK
        ack = c.recv(64)
        assert b'ACK+GSD=OK' in ack
        # drive a frame
        ep.set_wrench((1.0, 2.0, 3.0, 0.1, 0.2, 0.3))
        for _ in range(5):
            ep.send_frame_if_streaming()
            time.sleep(0.01)
        data = c.recv(256)
        # find a frame header and decode floats
        i = data.find(b'\xAA\x55')
        assert i >= 0
        frame = data[i:i+31]
        floats = struct.unpack('<6f', frame[6:30])
        assert abs(floats[0] - 1.0) < 1e-5
        c.close()
    finally:
        ep.stop()

def test_no_stream_before_start_command():
    ep = SriEndpoint(listen_port=0, require_start=True)
    ep.start()
    try:
        c = _connect(ep.port)
        assert ep.wait_for_client(2.0)
        ep.set_wrench((5, 0, 0, 0, 0, 0))
        ep.send_frame_if_streaming()  # should NOT send (no AT+GSD yet)
        c.settimeout(0.3)
        got = b''
        try:
            got = c.recv(64)
        except socket.timeout:
            pass
        assert got == b''
        c.close()
    finally:
        ep.stop()
