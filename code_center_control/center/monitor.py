"""브라우저로 내보낼 이벤트. MQTT 토픽 이름과 같은 칸에 맞춘다."""

from __future__ import annotations

import asyncio
import json
import time
from collections import deque
from typing import Any

from center.protocol import Event

HISTORY = 250

CMD_TOPIC = "robi/box/cmd"
EVENT_TOPIC = "robi/box/event"
STATUS_TOPIC = "robi/box/status"
HELLO_TOPIC = "robi/box/hello"
ROBOT_CMD_TOPIC = "robi/robot/cmd"
ROBOT_EVENT_TOPIC = "robi/robot/event"
ROBOT_STATUS_TOPIC = "robi/robot/status"
ROBOT_HELLO_TOPIC = "robi/robot/hello"
SYS_TOPIC = "robi/sys"

PHASE_LABEL = {
    "go": "목적지로 이동",
    "back": "원래 자리로",
    "round": "왕복",
    "idle": "대기",
    "arrived": "목적지 도착",
    "home": "원래 자리",
    "NAV": "이동 중",
    "IDLE": "대기",
}


def _is_robot(event: Event | None) -> bool:
    if event is None:
        return False
    if event.kind == "H" and event.cmd == "robot":
        return True
    if (event.cmd or "").startswith("robot."):
        return True
    if "phase" in event.fields and "mm" not in event.fields:
        return True
    return False


def classify(line: str, event: Event | None) -> tuple[str, str]:
    if line.startswith("#"):
        return SYS_TOPIC, "sys"
    if line.startswith(">"):
        if " robot." in line:
            return ROBOT_CMD_TOPIC, "out"
        return CMD_TOPIC, "out"
    if event is None:
        return SYS_TOPIC, "sys"
    robot = _is_robot(event)
    if event.kind == "S":
        return (ROBOT_STATUS_TOPIC if robot else STATUS_TOPIC), "in"
    if event.kind == "H":
        return (ROBOT_HELLO_TOPIC if robot else HELLO_TOPIC), "in"
    if event.kind in {"A", "D", "F", "P"}:
        return (ROBOT_EVENT_TOPIC if robot else EVENT_TOPIC), "in"
    if event.kind in {"C", "Q"}:
        return (ROBOT_CMD_TOPIC if robot else CMD_TOPIC), "in"
    return SYS_TOPIC, "sys"


class Monitor:
    def __init__(self, loop: asyncio.AbstractEventLoop):
        self.loop = loop
        self.history: deque[dict[str, Any]] = deque(maxlen=HISTORY)
        self.clients: set[Any] = set()
        self.mqtt_ok = False
        self.mqtt_host = ""
        self.box_names: list[str] = []
        self.robot_names: list[str] = []
        self.status: dict[str, Any] = {
            "state": "—",
            "mm": None,
            "sensor_ok": None,
            "lift_bottom": None,
            "lift_top": None,
            "pusher_back": None,
            "pusher_front": None,
            "busy": 0,
        }
        self.robot: dict[str, Any] = {
            "phase": "대기",
            "i": 0,
            "n": 0,
            "busy": 0,
        }

    def snapshot(self) -> dict[str, Any]:
        return {
            "type": "snapshot",
            "history": list(self.history),
            "status": self.status,
            "robot": dict(self.robot),
            "box": list(self.box_names),
            "robot_names": list(self.robot_names),
            "mqtt": self.mqtt_ok,
            "mqtt_host": self.mqtt_host,
        }

    def set_box(self, names: list[str]) -> None:
        self.set_devices(names, self.robot_names)

    def set_devices(self, box: list[str], robot: list[str] | None = None) -> None:
        self.box_names = box
        if robot is not None:
            self.robot_names = robot
        self._push(self._meta())

    def _meta(self) -> dict[str, Any]:
        return {
            "type": "meta",
            "box": list(self.box_names),
            "robot_names": list(self.robot_names),
            "mqtt": self.mqtt_ok,
            "mqtt_host": self.mqtt_host,
        }

    def set_mqtt(self, ok: bool, host: str = "") -> None:
        self.mqtt_ok = ok
        if host:
            self.mqtt_host = host
        self._push(self._meta())

    def on_hub(self, line: str, event: Event | None) -> None:
        topic, direction = classify(line, event)
        frame: dict[str, Any] = {
            "type": "msg",
            "via": "hub",
            "ts": time.time(),
            "topic": topic,
            "dir": direction,
            "line": line.lstrip("> ").strip() if line.startswith(">") else line,
            "kind": event.kind if event else "sys",
            "id": event.id if event else "",
            "status": event.status if event else "",
            "cmd": event.cmd if event else "",
            "fields": event.fields if event else {},
        }
        self._apply_status(event)
        self._push(frame)

    def push_mqtt(self, direction: str, topic: str, payload: str) -> None:
        self._push(
            {
                "type": "msg",
                "via": "mqtt",
                "ts": time.time(),
                "topic": topic,
                "dir": direction,
                "line": payload,
                "kind": "MQTT",
                "id": "",
                "status": "",
                "cmd": "",
                "fields": {},
            }
        )

    def push_sys(self, text: str) -> None:
        self._push(
            {
                "type": "msg",
                "via": "hub",
                "ts": time.time(),
                "topic": SYS_TOPIC,
                "dir": "sys",
                "line": text,
                "kind": "sys",
                "id": "",
                "status": "",
                "cmd": "",
                "fields": {},
            }
        )

    def _apply_status(self, event: Event | None) -> None:
        if event is None:
            return
        if _is_robot(event):
            self._apply_robot(event)
            return
        if event.kind == "S":
            for key in (
                "state",
                "mm",
                "sensor_ok",
                "lift_bottom",
                "lift_top",
                "pusher_back",
                "pusher_front",
                "busy",
            ):
                if key in event.fields:
                    value: Any = event.fields[key]
                    if key != "state":
                        try:
                            value = int(value)
                        except ValueError:
                            pass
                    self.status[key] = value
            self._push({"type": "status", "status": dict(self.status)})
        elif event.kind == "P" and "mm" in event.fields:
            try:
                self.status["mm"] = int(event.fields["mm"])
                self.status["busy"] = 1
                if event.cmd:
                    self.status["state"] = event.cmd
                self._push({"type": "status", "status": dict(self.status)})
            except ValueError:
                pass
        elif event.kind in {"D", "F"}:
            self.status["busy"] = 0
            self.status["state"] = "IDLE" if event.kind == "D" else "FAULT"
            if "mm" in event.fields:
                try:
                    self.status["mm"] = int(event.fields["mm"])
                except ValueError:
                    pass
            self._push({"type": "status", "status": dict(self.status)})

    def _apply_robot(self, event: Event) -> None:
        phase_raw = event.fields.get("phase", "")
        if event.kind in {"D", "F"} and not phase_raw:
            phase_raw = "idle"
        if event.kind == "P" and not phase_raw:
            if event.cmd == "robot.back":
                phase_raw = "back"
            elif event.cmd in {"robot.go"}:
                phase_raw = "go"
        if phase_raw:
            self.robot["phase"] = PHASE_LABEL.get(phase_raw, phase_raw)
        if "i" in event.fields:
            try:
                self.robot["i"] = int(event.fields["i"])
            except ValueError:
                pass
        if "n" in event.fields:
            try:
                self.robot["n"] = int(event.fields["n"])
            except ValueError:
                pass
        if event.kind == "P":
            self.robot["busy"] = 1
        elif event.kind in {"D", "F"}:
            self.robot["busy"] = 0
            if event.kind == "D":
                reason = event.fields.get("reason", "")
                if reason == "arrived":
                    self.robot["phase"] = PHASE_LABEL["arrived"]
                elif reason == "home":
                    self.robot["phase"] = PHASE_LABEL["home"]
                elif not phase_raw:
                    self.robot["phase"] = PHASE_LABEL["idle"]
        elif event.kind == "S" and "busy" in event.fields:
            try:
                self.robot["busy"] = int(event.fields["busy"])
            except ValueError:
                pass
        self._push({"type": "robot", "robot": dict(self.robot)})

    def _push(self, frame: dict[str, Any]) -> None:
        if frame.get("type") == "msg":
            self.history.append(frame)
        try:
            self.loop.call_soon_threadsafe(self._schedule, frame)
        except RuntimeError:
            pass

    def _schedule(self, frame: dict[str, Any]) -> None:
        self.loop.create_task(self._broadcast(frame))

    async def _broadcast(self, frame: dict[str, Any]) -> None:
        dead = []
        payload = json.dumps(frame, ensure_ascii=False)
        for ws in list(self.clients):
            try:
                await ws.send_text(payload)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.clients.discard(ws)
