"""선택 MQTT 게이트. 같은 cmd/id를 JSON으로 올린다.

브로커(Mosquitto)가 있을 때만 켠다. 상자와의 실제 전송은 TCP/USB다.
이동로봇이 나중에 같은 토픽에 붙으면 된다.
"""

from __future__ import annotations

import json
import asyncio
from typing import TYPE_CHECKING

import paho.mqtt.client as mqtt

from center.protocol import Event, resolve_cmd

if TYPE_CHECKING:
    from center.hub import Hub
    from center.monitor import Monitor

CMD_TOPIC = "robi/box/cmd"
EVENT_TOPIC = "robi/box/event"
STATUS_TOPIC = "robi/box/status"
ROBOT_CMD_TOPIC = "robi/robot/cmd"
ROBOT_EVENT_TOPIC = "robi/robot/event"
ROBOT_STATUS_TOPIC = "robi/robot/status"


class MqttBus:
    def __init__(
        self,
        hub: Hub,
        host: str,
        port: int,
        loop: asyncio.AbstractEventLoop,
        monitor: Monitor | None = None,
    ):
        self.hub = hub
        self.loop = loop
        self.monitor = monitor
        self.host = f"{host}:{port}"
        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2, client_id="robi-center"
        )
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        self.client.connect_async(host, port, keepalive=30)
        self.client.loop_start()
        hub.add_listener(self._on_hub)

    def _on_connect(self, client: mqtt.Client, *_args: object) -> None:
        client.subscribe(CMD_TOPIC)
        client.subscribe(ROBOT_CMD_TOPIC)
        if self.monitor is not None:
            self.monitor.set_mqtt(True, self.host)

    def _on_disconnect(self, *_args: object) -> None:
        if self.monitor is not None:
            self.monitor.set_mqtt(False, self.host)

    def _on_message(self, _c: mqtt.Client, _u: object, msg: mqtt.MQTTMessage) -> None:
        text = msg.payload.decode("utf-8", errors="ignore")
        if self.monitor is not None:
            self.monitor.push_mqtt("in", msg.topic, text)
        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            cmd = resolve_cmd(text.strip())
            if cmd:
                asyncio.run_coroutine_threadsafe(self._run(cmd), self.loop)
            return
        cmd = resolve_cmd(str(payload.get("cmd", "")))
        if cmd:
            asyncio.run_coroutine_threadsafe(self._run(cmd), self.loop)

    async def _run(self, cmd: str) -> None:
        try:
            await self.hub.send_cmd(cmd)
        except Exception:
            return

    def _on_hub(self, line: str, event: Event | None) -> None:
        if event is None or event.kind not in {"A", "D", "F", "S", "P"}:
            return
        body = {
            "raw": event.raw,
            "kind": event.kind,
            "id": event.id,
            "status": event.status,
            "cmd": event.cmd,
            "fields": event.fields,
        }
        topic = STATUS_TOPIC if event.kind == "S" else EVENT_TOPIC
        if (event.cmd or "").startswith("robot.") or (
            "phase" in event.fields and "mm" not in event.fields
        ):
            topic = ROBOT_STATUS_TOPIC if event.kind == "S" else ROBOT_EVENT_TOPIC
        payload = json.dumps(body, ensure_ascii=False)
        self.client.publish(topic, payload, qos=0)
        if self.monitor is not None:
            self.monitor.push_mqtt("out", topic, payload)

    def close(self) -> None:
        self.client.loop_stop()
        self.client.disconnect()
