"""명령 id를 붙이고, 상자의 DONE/FAIL과 짝을 맞춘다."""

from __future__ import annotations

import asyncio
from collections.abc import Callable

from center.links import Link
from center.protocol import Event, format_request, parse_line
from center.recipes import RECIPES, Step

Listener = Callable[[str, Event | None], None]


class Hub:
    def __init__(self) -> None:
        self.links: list[Link] = []
        self._seq = 0
        self._pending: dict[str, asyncio.Future[Event]] = {}
        self._listeners: list[Listener] = []

    def add_listener(self, fn: Listener) -> None:
        self._listeners.append(fn)

    def add_link(self, link: Link) -> None:
        self.links.append(link)
        self._note(f"# 연결 {link.name}")

    def drop_link(self, link: Link) -> None:
        self.links = [item for item in self.links if item is not link]
        self._note(f"# 끊김 {link.name}")

    def active_link(self) -> Link | None:
        for link in self.links:
            if link.connected:
                return link
        return None

    def next_id(self) -> str:
        self._seq = (self._seq + 1) & 0xFFFF
        if self._seq == 0:
            self._seq = 1
        return f"{self._seq:04x}"

    def on_line(self, line: str) -> None:
        event = parse_line(line)
        self._emit(line, event)
        if event is None:
            return
        if event.kind in {"D", "F"} and event.id in self._pending:
            future = self._pending[event.id]
            if not future.done():
                future.set_result(event)
        if event.kind == "S" and event.id in self._pending:
            future = self._pending[event.id]
            if not future.done():
                future.set_result(event)

    async def send_cmd(self, cmd: str, timeout: float = 30.0) -> Event:
        link = self.active_link()
        if link is None:
            raise RuntimeError("상자가 아직 연결되지 않음")

        msg_id = self.next_id()
        line = format_request(msg_id, cmd)
        loop = asyncio.get_running_loop()
        future: asyncio.Future[Event] = loop.create_future()
        self._pending[msg_id] = future
        self._note(f"> {line}")
        try:
            await link.send(line)
            return await asyncio.wait_for(future, timeout)
        except asyncio.TimeoutError as exc:
            raise TimeoutError(f"{cmd} ({msg_id}) 응답 없음") from exc
        finally:
            self._pending.pop(msg_id, None)

    async def run_recipe(
        self,
        steps: tuple[Step, ...],
        step_timeout: float = 30.0,
        on_step: Callable[[int, Step], None] | None = None,
    ) -> Event:
        last: Event | None = None
        for index, step in enumerate(steps, start=1):
            if on_step is not None:
                on_step(index, step)
            last = await self.send_cmd(step.cmd, timeout=step_timeout)
            if last.kind == "F":
                return last
        assert last is not None
        return last

    async def run_named(self, name: str, **kwargs: object) -> Event:
        if name not in RECIPES:
            raise KeyError(name)
        return await self.run_recipe(RECIPES[name], **kwargs)  # type: ignore[arg-type]

    def _emit(self, line: str, event: Event | None) -> None:
        for fn in self._listeners:
            fn(line, event)

    def _note(self, line: str) -> None:
        self._emit(line, parse_line(line) if not line.startswith("#") else None)
