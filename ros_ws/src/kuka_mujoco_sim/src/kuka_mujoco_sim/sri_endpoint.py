# src/kuka_mujoco_sim/sri_endpoint.py
"""SRI TCP server endpoint. sri_ft_driver connects, optionally sends AT+GSD,
then we stream 31-byte FT frames driven by the physics world's sensor wrench.
"""
import socket
import threading
import time

from .sri_codec import build_sri_frame, is_start_command, START_ACK


class SriEndpoint:
    def __init__(self, listen_ip='127.0.0.1', listen_port=4008,
                 require_start=True):
        self._ip = listen_ip
        self._req_port = listen_port
        self.port = listen_port
        self._require_start = require_start
        self._lock = threading.Lock()
        self._wrench = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._pn = 0
        self._client = None
        self._streaming = False
        self._listen = None
        self._running = False
        self._thread = None
        self.stats = {'frames_sent': 0, 'clients': 0}

    def start(self):
        self._listen = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listen.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listen.bind((self._ip, self._req_port))
        self.port = self._listen.getsockname()[1]
        self._listen.listen(1)
        self._listen.settimeout(0.1)
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        with self._lock:
            if self._client:
                self._client.close()
                self._client = None
        if self._listen:
            self._listen.close()

    def wait_for_client(self, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self._client is not None:
                    return True
            time.sleep(0.01)
        return False

    def set_wrench(self, wrench6):
        with self._lock:
            self._wrench = tuple(float(v) for v in wrench6)

    def send_frame_if_streaming(self):
        with self._lock:
            if self._client is None or not self._streaming:
                return
            self._pn += 1
            frame = build_sri_frame(self._wrench, self._pn)
            try:
                self._client.sendall(frame)
                self.stats['frames_sent'] += 1
            except OSError:
                self._drop_client_locked()

    def _drop_client_locked(self):
        if self._client:
            try:
                self._client.close()
            except OSError:
                pass
        self._client = None
        self._streaming = False

    def _run(self):
        while self._running:
            with self._lock:
                have_client = self._client is not None
            if not have_client:
                try:
                    conn, _ = self._listen.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break
                conn.settimeout(0.05)
                with self._lock:
                    self._client = conn
                    self._streaming = not self._require_start
                    self.stats['clients'] += 1
                continue
            # have client: poll for AT+GSD
            with self._lock:
                conn = self._client
            try:
                data = conn.recv(256)
                if not data:
                    with self._lock:
                        self._drop_client_locked()
                    continue
                if is_start_command(data):
                    with self._lock:
                        conn.sendall(START_ACK)
                        self._streaming = True
            except socket.timeout:
                pass
            except OSError:
                with self._lock:
                    self._drop_client_locked()
