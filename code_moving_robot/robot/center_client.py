"""중앙 컨트롤 TCP 클라이언트. 상자 브리지와 같은 한 줄 프로토콜.

라즈베리 파이의 wasd/g/b 키와 동시에 쓴다. 센터는 명령을 보내고,
여기서는 같은 주행 루프에 넣는다. 웹에 올릴 PROG 도 이 소켓으로 나간다.
"""

from __future__ import annotations

import queue
import socket
import threading
import time


class CenterClient:
    def __init__(self, host: str, port: int = 9000):
        self.host = host
        self.port = int(port)
        self._cmds: queue.Queue = queue.Queue()
        self._alive = True
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self.ok = False

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._alive = False
        sock = self._sock
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
        self._sock = None
        self.ok = False

    def poll(self):
        try:
            return self._cmds.get_nowait()
        except queue.Empty:
            return None

    def emit(self, line: str) -> None:
        payload = (line.strip() + "\n").encode("ascii", errors="ignore")
        with self._lock:
            sock = self._sock
            if sock is None:
                return
            try:
                sock.sendall(payload)
            except OSError:
                pass

    def _run(self) -> None:
        while self._alive:
            try:
                self._serve_one()
            except Exception:
                self.ok = False
                self._sock = None
                time.sleep(2.5)

    def _serve_one(self) -> None:
        sock = socket.create_connection((self.host, self.port), timeout=5.0)
        sock.settimeout(0.4)
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        with self._lock:
            self._sock = sock
        self.ok = True
        self.emit("H robot")
        buf = ""
        try:
            while self._alive:
                try:
                    chunk = sock.recv(256)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buf += chunk.decode("ascii", errors="ignore")
                while "\n" in buf:
                    raw, buf = buf.split("\n", 1)
                    self._on_line(raw.strip())
        finally:
            self.ok = False
            with self._lock:
                if self._sock is sock:
                    self._sock = None
            try:
                sock.close()
            except OSError:
                pass

    def _on_line(self, line: str) -> None:
        if not line or line.startswith("#"):
            return
        parts = line.split()
        kind = parts[0]
        if kind not in {"C", "Q"} or len(parts) < 3:
            return
        msg_id = parts[1]
        cmd = parts[2]
        self._cmds.put({"kind": kind, "id": msg_id, "cmd": cmd})
