"""상자와의 전송 경로. 내용은 전부 같은 한 줄 프로토콜."""

from __future__ import annotations

import asyncio
import contextlib
from collections.abc import Callable

OnLine = Callable[[str], None]


class Link:
    name = "link"

    async def send(self, line: str) -> None:
        raise NotImplementedError

    @property
    def connected(self) -> bool:
        return False


class SerialLink(Link):
    def __init__(self, port: str, baud: int, on_line: OnLine, loop: asyncio.AbstractEventLoop):
        import serial

        self.name = f"serial:{port}"
        self._ser = serial.Serial(port, baud, timeout=0.05)
        self._on_line = on_line
        self._loop = loop
        self._buf = ""
        self._alive = True
        self._task = loop.run_in_executor(None, self._read_loop)

    @property
    def connected(self) -> bool:
        return self._alive and self._ser.is_open

    async def send(self, line: str) -> None:
        payload = (line.strip() + "\n").encode("ascii", errors="ignore")
        await self._loop.run_in_executor(None, self._ser.write, payload)

    def _read_loop(self) -> None:
        while self._alive:
            try:
                chunk = self._ser.read(128)
            except Exception:
                self._alive = False
                return
            if not chunk:
                continue
            self._buf += chunk.decode("ascii", errors="ignore")
            while "\n" in self._buf:
                raw, self._buf = self._buf.split("\n", 1)
                line = raw.strip()
                if line:
                    self._loop.call_soon_threadsafe(self._on_line, line)

    def close(self) -> None:
        self._alive = False
        with contextlib.suppress(Exception):
            self._ser.close()


class TcpClientLink(Link):
    def __init__(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
        on_line: OnLine,
        peer: str,
    ):
        self.name = f"tcp:{peer}"
        self._reader = reader
        self._writer = writer
        self._on_line = on_line
        self._open = True

    @property
    def connected(self) -> bool:
        return self._open

    async def send(self, line: str) -> None:
        self._writer.write((line.strip() + "\n").encode("ascii", errors="ignore"))
        await self._writer.drain()

    async def read_loop(self) -> None:
        try:
            while True:
                raw = await self._reader.readline()
                if not raw:
                    break
                line = raw.decode("ascii", errors="ignore").strip()
                if line:
                    self._on_line(line)
        finally:
            self._open = False
            self._writer.close()
            with contextlib.suppress(Exception):
                await self._writer.wait_closed()

    def close(self) -> None:
        self._open = False
        self._writer.close()


class MockLink(Link):
    """보드 없이 프로토콜·레시피를 돌려 보는 가짜 상자."""

    name = "mock"

    def __init__(self, on_line: OnLine, loop: asyncio.AbstractEventLoop):
        self._on_line = on_line
        self._loop = loop
        self._mm = 320
        self._lift_bottom = 1
        self._lift_top = 0
        self._pusher_back = 1
        self._pusher_front = 0
        self._busy = False
        self._state = "IDLE"
        self._open = True
        self._gen = 0
        self._active_id = ""
        self._active_cmd = ""
        loop.call_soon(on_line, "H box")

    @property
    def connected(self) -> bool:
        return self._open

    async def send(self, line: str) -> None:
        parts = line.split()
        if len(parts) < 3:
            return
        kind, msg_id, cmd = parts[0], parts[1], parts[2]
        if kind == "Q" or cmd == "status":
            self._emit(self._status(msg_id))
            return
        if self._busy and cmd != "halt":
            self._emit(f"F {msg_id} FAIL {cmd} reason=busy")
            return
        self._loop.create_task(self._run(msg_id, cmd))

    def _emit(self, line: str) -> None:
        self._on_line(line)

    def _status(self, msg_id: str) -> str:
        return (
            f"S {msg_id} STATUS state={self._state} mm={self._mm} "
            f"sensor_ok=1 lift_bottom={self._lift_bottom} lift_top={self._lift_top} "
            f"pusher_back={self._pusher_back} pusher_front={self._pusher_front} "
            f"busy={1 if self._busy else 0}"
        )

    async def _run(self, msg_id: str, cmd: str) -> None:
        self._emit(f"A {msg_id} ACK {cmd}")
        if cmd == "halt":
            self._gen += 1
            if self._active_id:
                self._emit(
                    f"F {self._active_id} FAIL {self._active_cmd} reason=halted"
                )
                self._active_id = ""
                self._active_cmd = ""
            self._busy = False
            self._state = "IDLE"
            self._emit(f"D {msg_id} DONE {cmd} reason=stopped mm={self._mm}")
            return

        self._busy = True
        self._active_id = msg_id
        self._active_cmd = cmd
        gen = self._gen
        try:
            if cmd == "lift.up":
                self._state = "LIFT_UP"
                self._lift_bottom = 0
                while self._mm > 70:
                    if gen != self._gen:
                        return
                    self._mm -= 20
                    if self._mm < 70:
                        self._mm = 70
                    self._emit(f"P {msg_id} PROG {cmd} mm={self._mm}")
                    await asyncio.sleep(0.12)
                if gen != self._gen:
                    return
                self._emit(f"D {msg_id} DONE {cmd} reason=tof mm={self._mm}")
            elif cmd == "lift.down":
                self._state = "LIFT_DOWN"
                await asyncio.sleep(0.25)
                if gen != self._gen:
                    return
                self._mm = 320
                self._lift_bottom = 1
                self._lift_top = 0
                self._emit(f"D {msg_id} DONE {cmd} reason=limit_bottom mm={self._mm}")
            elif cmd == "pusher.forward":
                self._state = "PUSH_FWD"
                self._pusher_back = 0
                await asyncio.sleep(0.25)
                if gen != self._gen:
                    return
                self._pusher_front = 1
                self._emit(f"D {msg_id} DONE {cmd} reason=limit_front mm={self._mm}")
            elif cmd == "pusher.back":
                self._state = "PUSH_BACK"
                self._pusher_front = 0
                await asyncio.sleep(0.25)
                if gen != self._gen:
                    return
                self._pusher_back = 1
                self._emit(f"D {msg_id} DONE {cmd} reason=limit_back mm={self._mm}")
            else:
                self._emit(f"F {msg_id} FAIL {cmd} reason=unknown")
        finally:
            if gen == self._gen:
                self._busy = False
                self._state = "IDLE"
                self._active_id = ""
                self._active_cmd = ""

    def close(self) -> None:
        self._open = False
