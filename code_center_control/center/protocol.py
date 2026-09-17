"""적재함 UART/TCP 한 줄 프로토콜.

Arduino는 JSON을 안 쓴다. 중앙과 상자는 같은 토큰을 주고받고,
나중에 MQTT JSON은 이 필드를 그대로 감싼다.

  C 00a1 lift.up
  A 00a1 ACK lift.up
  P 00a1 PROG lift.up mm=90
  D 00a1 DONE lift.up reason=tof mm=68
  F 00a1 FAIL lift.up reason=timeout
  Q 00a2 status
  S 00a2 STATUS state=IDLE mm=120 sensor_ok=1 ...
  H box
"""

from __future__ import annotations

from dataclasses import dataclass, field

PRIMITIVES = (
    "lift.up",
    "lift.down",
    "pusher.forward",
    "pusher.back",
    "halt",
    "status",
)

ALIASES = {
    "u": "lift.up",
    "d": "lift.down",
    "f": "pusher.forward",
    "b": "pusher.back",
    "s": "halt",
    "?": "status",
    "상승": "lift.up",
    "하강": "lift.down",
    "전진": "pusher.forward",
    "후진": "pusher.back",
    "정지": "halt",
    "상태": "status",
}

CMD_HELP = {
    "lift.up": "리프트 상승. VL53L0X가 목표 거리 이하가 되면 정지",
    "lift.down": "리프트 하강. 하단 리미트에서 정지",
    "pusher.forward": "푸셔 전진. 앞 리미트에서 정지",
    "pusher.back": "푸셔 후진. 뒤 리미트에서 정지",
    "halt": "지금 동작 즉시 정지",
    "status": "거리·리미트·바쁨 여부",
}


@dataclass
class Event:
    raw: str
    kind: str
    id: str = ""
    status: str = ""
    cmd: str = ""
    fields: dict[str, str] = field(default_factory=dict)

    def get_int(self, key: str, default: int | None = None) -> int | None:
        value = self.fields.get(key)
        if value is None or value == "":
            return default
        try:
            return int(value)
        except ValueError:
            return default


def resolve_cmd(text: str) -> str | None:
    name = text.strip()
    if not name:
        return None
    if name in ALIASES:
        return ALIASES[name]
    if name in PRIMITIVES:
        return name
    return None


def format_request(msg_id: str, cmd: str) -> str:
    if cmd == "status":
        return f"Q {msg_id} status"
    return f"C {msg_id} {cmd}"


def parse_kv(parts: list[str]) -> dict[str, str]:
    out: dict[str, str] = {}
    for part in parts:
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        out[key] = value
    return out


def parse_line(line: str) -> Event | None:
    raw = line.strip()
    if not raw or raw.startswith("#"):
        return None

    parts = raw.split()
    kind = parts[0]

    if kind == "H":
        who = parts[1] if len(parts) > 1 else ""
        return Event(raw=raw, kind="H", status="HELLO", cmd=who)

    if kind not in {"A", "D", "F", "S", "P", "C", "Q"}:
        return Event(raw=raw, kind="?", status="RAW")

    msg_id = parts[1] if len(parts) > 1 else ""
    status = parts[2] if len(parts) > 2 else ""
    if kind == "S":
        fields = parse_kv(parts[3:] if len(parts) > 3 else [])
        return Event(
            raw=raw,
            kind=kind,
            id=msg_id,
            status="STATUS",
            cmd=fields.get("cmd", ""),
            fields=fields,
        )
    cmd = parts[3] if len(parts) > 3 else ""
    fields = parse_kv(parts[4:] if len(parts) > 4 else [])
    return Event(
        raw=raw,
        kind=kind,
        id=msg_id,
        status=status,
        cmd=cmd,
        fields=fields,
    )
