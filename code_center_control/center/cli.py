"""중앙 컨트롤 입력창. 한 줄이 원자 명령 하나, 또는 레시피 이름."""

from __future__ import annotations

import argparse
import asyncio
import sys

from center.hub import Hub
from center.links import MockLink, SerialLink, TcpClientLink
from center.protocol import CMD_HELP, Event, resolve_cmd
from center.recipes import RECIPE_HELP, RECIPES


def _print(text: str) -> None:
    print(text, flush=True)


def show_event(line: str, event: Event | None) -> None:
    if event is None:
        if line.startswith("#"):
            _print(line)
        return
    if event.kind == "H":
        _print(f"상자 접속 ({event.cmd or 'box'})")
        return
    if event.kind == "P":
        mm = event.fields.get("mm", "?")
        _print(f"  … {mm} mm")
        return
    if event.kind == "A":
        return
    if event.kind == "?":
        _print(line)
        return
    if line.startswith(">"):
        _print(line)
        return
    _print(f"< {line}")


def help_text() -> str:
    lines = ["원자 명령 (나중에 레시피·MQTT에서 그대로 재사용):"]
    for name, desc in CMD_HELP.items():
        lines.append(f"  {name:18} {desc}")
    lines.append("")
    lines.append("레시피:")
    for name, desc in RECIPE_HELP.items():
        lines.append(f"  {name:18} {desc}")
    lines.append("")
    lines.append("기타: status, halt, help, quit")
    lines.append("한글 별명: 상승 하강 전진 후진 정지 상태")
    return "\n".join(lines)


async def wait_confirm(prompt: str) -> bool:
    _print(prompt + "  [Enter=실행, n=건너뜀, q=중단]")
    text = await asyncio.to_thread(sys.stdin.readline)
    if text is None:
        return False
    key = text.strip().lower()
    if key in {"q", "quit"}:
        raise KeyboardInterrupt
    return key not in {"n", "no", "s"}


async def handle_line(hub: Hub, text: str, stepped: bool) -> None:
    raw = text.strip()
    if not raw:
        return
    if raw in {"help", "h", "?"}:
        _print(help_text())
        return
    if raw in {"quit", "q", "exit"}:
        raise EOFError

    if raw.startswith("recipe "):
        raw = raw.split(None, 1)[1]

    if raw in RECIPES:
        steps = RECIPES[raw]
        _print(f"레시피 {raw}  ({len(steps)}단계)")

        def sync_on_step(index: int, step: object) -> None:
            _print(f"[{index}/{len(steps)}] {step.cmd}  — {step.title}")

        if stepped:
            last = None
            for index, step in enumerate(steps, start=1):
                _print(f"[{index}/{len(steps)}] {step.cmd}  — {step.title}")
                if not await wait_confirm("이 단계를 보낼까?"):
                    continue
                last = await hub.send_cmd(step.cmd)
                if last.kind == "F":
                    _print("레시피 중단 (실패)")
                    return
            return

        await hub.run_recipe(steps, on_step=sync_on_step)
        return

    cmd = resolve_cmd(raw)
    if cmd is None:
        _print(f"모르는 입력: {raw}  (help)")
        return
    await hub.send_cmd(cmd)


async def tcp_server(hub: Hub, host: str, port: int, on_change=None) -> None:
    async def on_client(
        reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        peer = writer.get_extra_info("peername")
        addr = f"{peer[0]}:{peer[1]}" if peer else "?"
        link = TcpClientLink(reader, writer, hub.on_line, addr)
        hub.add_link(link)
        if on_change is not None:
            on_change()
        try:
            await link.read_loop()
        finally:
            hub.drop_link(link)
            if on_change is not None:
                on_change()

    server = await asyncio.start_server(on_client, host, port)
    sockets = server.sockets or []
    where = ", ".join(str(sock.getsockname()) for sock in sockets)
    _print(f"TCP 대기 {where}  (ESP 브리지가 여기로 붙음)")
    async with server:
        await server.serve_forever()


async def async_main(args: argparse.Namespace) -> None:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")

    hub = Hub()
    hub.add_listener(show_event)
    loop = asyncio.get_running_loop()
    tasks: list[asyncio.Task[object]] = []
    mqtt = None
    monitor = None

    if not args.no_web:
        from center.monitor import Monitor

        monitor = Monitor(loop)
        hub.add_listener(monitor.on_hub)

    def refresh_box() -> None:
        if monitor is not None:
            monitor.set_box([link.name for link in hub.links if link.connected])

    if args.mock:
        hub.add_link(MockLink(hub.on_line, loop))
        _print("모의 상자. 보드 없이 명령만 시험")
        refresh_box()

    if args.serial:
        hub.add_link(SerialLink(args.serial, args.baud, hub.on_line, loop))
        _print(f"USB {args.serial} {args.baud}")
        refresh_box()

    if args.mqtt:
        from center.mqtt_bus import MqttBus

        mqtt = MqttBus(hub, args.mqtt, args.mqtt_port, loop, monitor)
        _print(f"MQTT {args.mqtt}:{args.mqtt_port}  구독 robi/box/cmd")
    else:
        mqtt = None

    if not args.no_tcp:
        tasks.append(asyncio.create_task(tcp_server(hub, args.host, args.port, refresh_box)))

    if monitor is not None:
        import uvicorn
        from center.web import create_app

        app = create_app(hub, monitor, loop)
        config = uvicorn.Config(
            app,
            host=args.web_host,
            port=args.web_port,
            log_level="warning",
        )
        server = uvicorn.Server(config)
        server.install_signal_handlers = False
        tasks.append(asyncio.create_task(server.serve()))
        _print(f"브라우저  http://127.0.0.1:{args.web_port}/")


    _print("Robi 중앙 컨트롤. help 로 명령 목록.")
    _print(help_text())

    try:
        if args.web_only:
            await asyncio.Future()
            return
        if args.commands:
            for item in args.commands:
                await handle_line(hub, item, stepped=args.step)
            return
        while True:
            _print("robi> ", end="")
            line = await asyncio.to_thread(sys.stdin.readline)
            if line == "":
                break
            try:
                await handle_line(hub, line, stepped=args.step)
            except EOFError:
                break
            except KeyboardInterrupt:
                _print("중단")
            except TimeoutError as exc:
                _print(str(exc))
            except RuntimeError as exc:
                _print(str(exc))
    finally:
        if mqtt is not None:
            mqtt.close()
        for task in tasks:
            task.cancel()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Robi 중앙 컨트롤")
    parser.add_argument("--serial", help="우노 USB 포트. 예: COM5")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--host", default="0.0.0.0", help="ESP가 붙을 TCP 주소")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--no-tcp", action="store_true", help="TCP 서버를 열지 않음")
    parser.add_argument("--mqtt", help="Mosquitto 주소. 예: 127.0.0.1")
    parser.add_argument("--mqtt-port", type=int, default=1883)
    parser.add_argument("--mock", action="store_true", help="가짜 상자로 명령 시험")
    parser.add_argument("--step", action="store_true", help="레시피를 한 단계씩 확인")
    parser.add_argument("--web-host", default="0.0.0.0", help="모니터 HTTP 주소")
    parser.add_argument("--web-port", type=int, default=8080)
    parser.add_argument("--no-web", action="store_true", help="브라우저 모니터를 열지 않음")
    parser.add_argument("--web-only", action="store_true", help="입력창 없이 브라우저만")
    parser.add_argument(
        "commands",
        nargs="*",
        help="있으면 입력창 없이 이 명령들을 실행하고 끝냄",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.web_only:
        args.no_web = False
    try:
        asyncio.run(async_main(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
