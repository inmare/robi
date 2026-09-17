"""브라우저 모니터. 같은 이벤트에 MQTT 토픽 이름을 붙여 보여 준다."""

from __future__ import annotations

import asyncio
import json
from pathlib import Path
from typing import Any

from starlette.applications import Starlette
from starlette.requests import Request
from starlette.responses import FileResponse, JSONResponse
from starlette.routing import Route, WebSocketRoute
from starlette.websockets import WebSocket, WebSocketDisconnect

from center.hub import Hub
from center.monitor import Monitor
from center.protocol import CMD_HELP, PRIMITIVES, resolve_cmd
from center.recipes import RECIPE_HELP, RECIPES

HTML_PATH = Path(__file__).resolve().parent / "static" / "monitor.html"
_bg_tasks: set[asyncio.Task[Any]] = set()


def _spawn(coro: Any) -> None:
    # 참조를 안 남기면 웹에서 만든 task가 바로 GC되어 명령이 안 나간다.
    task = asyncio.create_task(coro)
    _bg_tasks.add(task)
    task.add_done_callback(_bg_tasks.discard)


def create_app(hub: Hub, monitor: Monitor, _loop: asyncio.AbstractEventLoop) -> Starlette:
    async def index(_request: Request) -> FileResponse:
        return FileResponse(
            HTML_PATH,
            media_type="text/html; charset=utf-8",
            headers={"Cache-Control": "no-store"},
        )

    async def meta(_request: Request) -> JSONResponse:
        return JSONResponse(
            {
                "primitives": [
                    {"cmd": name, "help": CMD_HELP[name]} for name in PRIMITIVES
                ],
                "recipes": [
                    {
                        "name": name,
                        "help": RECIPE_HELP[name],
                        "steps": [step.cmd for step in RECIPES[name]],
                    }
                    for name in RECIPES
                    if not any(step.cmd.startswith("robot.") for step in RECIPES[name])
                ],
                "topics": [
                    "robi/box/cmd",
                    "robi/box/event",
                    "robi/box/status",
                    "robi/robot/cmd",
                    "robi/robot/event",
                    "robi/robot/status",
                ],
            }
        )

    async def snapshot(_request: Request) -> JSONResponse:
        data = monitor.snapshot()
        data["box"] = [
            link.name
            for link in hub.links
            if link.connected and getattr(link, "role", "box") != "robot"
        ]
        data["robot_names"] = [
            link.name
            for link in hub.links
            if link.connected and getattr(link, "role", "box") == "robot"
        ]
        return JSONResponse(data)

    async def api_cmd(request: Request) -> JSONResponse:
        try:
            body: dict[str, Any] = await request.json()
        except json.JSONDecodeError:
            return JSONResponse({"ok": False, "error": "json"}, status_code=400)
        recipe = str(body.get("recipe", "")).strip()
        cmd = str(body.get("cmd", "")).strip()
        if recipe:
            _spawn(_run_recipe(hub, monitor, recipe))
            return JSONResponse({"ok": True})
        resolved = resolve_cmd(cmd)
        if resolved is None:
            return JSONResponse({"ok": False, "error": "unknown"}, status_code=400)
        _spawn(_run_cmd(hub, monitor, resolved))
        return JSONResponse({"ok": True})

    async def ws_endpoint(ws: WebSocket) -> None:
        await ws.accept()
        monitor.clients.add(ws)
        hello = monitor.snapshot()
        hello["box"] = [link.name for link in hub.links if getattr(link, "role", "box") != "robot" and link.connected]
        hello["robot_names"] = [link.name for link in hub.links if getattr(link, "role", "box") == "robot" and link.connected]
        await ws.send_text(json.dumps(hello, ensure_ascii=False))
        try:
            while True:
                raw = await ws.receive_text()
                try:
                    body = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                kind = body.get("type")
                if kind == "cmd":
                    resolved = resolve_cmd(str(body.get("cmd", "")))
                    if resolved:
                        _spawn(_run_cmd(hub, monitor, resolved))
                elif kind == "recipe":
                    _spawn(_run_recipe(hub, monitor, str(body.get("name", ""))))
        except WebSocketDisconnect:
            pass
        finally:
            monitor.clients.discard(ws)

    return Starlette(
        routes=[
            Route("/", index),
            Route("/api/meta", meta),
            Route("/api/snapshot", snapshot),
            Route("/api/cmd", api_cmd, methods=["POST"]),
            WebSocketRoute("/ws", ws_endpoint),
        ]
    )


async def _run_cmd(hub: Hub, monitor: Monitor, cmd: str) -> None:
    try:
        await hub.send_cmd(cmd)
    except Exception as exc:
        monitor.push_sys(f"# {cmd} 실패: {exc}")


async def _run_recipe(hub: Hub, monitor: Monitor, name: str) -> None:
    if name not in RECIPES:
        monitor.push_sys(f"# 없는 레시피: {name}")
        return
    monitor.push_sys(f"# 레시피 {name}")
    try:
        await hub.run_named(name)
    except Exception as exc:
        monitor.push_sys(f"# {name} 실패: {exc}")
