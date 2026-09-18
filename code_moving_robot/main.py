"""이동로봇 메인. 슬롯 TUI에서 경로를 고르고 왕복 주행한다."""

import argparse
from pathlib import Path

from robot.session import run_drive, save_now
from robot.slots import DEFAULT_NAME, SlotStore
from robot.tui import (
    c_dim,
    c_err,
    c_info,
    c_ok,
    c_warn,
    clear_screen,
    draw_box,
    prompt,
    restore_terminal,
)

ROOT = Path(__file__).resolve().parent
STORE = SlotStore(ROOT / "maps" / "slots")
CENTER_HOST = ""
CENTER_PORT = 9000


def main_menu_lines(slots):
    lines = []
    for s in slots:
        if s.empty:
            lines.append(c_dim(f"{s.index})  [비어 있음]"))
        else:
            lines.append(c_ok(f"{s.index})  {s.name}"))
            lines.append(c_dim(f"      {s.saved_at or ''}"))
    lines.append("")
    lines.append(c_info("n)  빈 슬롯에 새 경로"))
    lines.append("q)  종료")
    return lines


def show_main_menu():
    slots = STORE.list()
    clear_screen()
    print(draw_box("ROBI 이동 로봇", main_menu_lines(slots)))
    print()
    print(c_dim("번호 = 재생 · 빈 슬롯/n = 새 기록 (기존 슬롯 덮어쓰기 없음)"))
    return prompt(c_warn("선택: ")).strip().lower()


def choose_empty_slot():
    slots = STORE.list()
    empty = [s for s in slots if s.empty]
    if not empty:
        print(c_err("빈 슬롯이 없습니다. 기존 경로는 재생만 됩니다"))
        return None
    print()
    print(c_info("빈 슬롯"))
    for s in empty:
        print(f"  {s.index})  {c_dim('[비어 있음]')}")
    raw = prompt(c_warn("번호 (Enter 취소): ")).strip()
    if not raw.isdigit():
        return None
    index = int(raw)
    info = STORE.info(index) if 1 <= index <= STORE.n else None
    if info is None or not info.empty:
        print(c_err("빈 슬롯만 고를 수 있습니다"))
        return None
    return index


def ask_name(default=""):
    shown = default or DEFAULT_NAME
    raw = prompt(c_info(f"경로 이름 [{shown}]: ")).strip()
    return raw or shown


def after_drive(result):
    if result.get("grid") is None:
        return
    if not result.get("has_route"):
        print(c_warn("저장할 경로가 없습니다"))
        return
    slot_index = result.get("slot_index")
    name = result.get("name") or DEFAULT_NAME
    dirty = result.get("dirty") or result.get("map_writable")
    if slot_index is None:
        slot_index = choose_empty_slot()
        if slot_index is None:
            print(c_warn("저장 취소"))
            return
        name = ask_name(name)
    elif not dirty:
        return
    save_now(
        STORE,
        slot_index,
        result["grid"],
        result.get("odo"),
        result["start"],
        result["waypoints"],
        result["goal"],
        name,
        replace_map=True,
    )


def drive_new(slot_index=None, slot_name=DEFAULT_NAME):
    result = run_drive(
        store=STORE,
        slot_index=slot_index,
        slot_name=slot_name,
        recording=True,
        map_writable=True,
        grid=None,
        data=None,
        center_host=CENTER_HOST,
        center_port=CENTER_PORT,
    )
    after_drive(result)
    prompt(c_dim("Enter 로 메뉴..."))


def drive_play(info):
    loaded = STORE.load(info.index)
    if loaded is None:
        print(c_err("슬롯을 읽지 못했습니다"))
        prompt(c_dim("Enter..."))
        return
    data, grid, loaded_info = loaded
    result = run_drive(
        store=STORE,
        slot_index=loaded_info.index,
        slot_name=loaded_info.name,
        recording=False,
        map_writable=False,
        grid=grid,
        data=data,
        center_host=CENTER_HOST,
        center_port=CENTER_PORT,
    )
    after_drive(result)
    prompt(c_dim("Enter 로 메뉴..."))


def parse_args():
    parser = argparse.ArgumentParser(description="Robi 이동로봇")
    parser.add_argument(
        "--center",
        default="",
        help="중앙 컨트롤 TCP 주소. 예: 192.168.0.10",
    )
    parser.add_argument("--center-port", type=int, default=9000)
    return parser.parse_args()


def main():
    global CENTER_HOST, CENTER_PORT
    args = parse_args()
    CENTER_HOST = args.center
    CENTER_PORT = args.center_port
    while True:
        try:
            raw = show_main_menu()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if raw in ("q",):
            break
        if raw in ("n",):
            index = choose_empty_slot()
            if index is None:
                prompt(c_dim("Enter..."))
                continue
            name = ask_name(DEFAULT_NAME)
            print(c_ok(f"새 경로는 슬롯 {index} 「{name}」에 저장합니다"))
            drive_new(slot_index=index, slot_name=name)
            continue
        if raw.isdigit():
            index = int(raw)
            if index < 1 or index > STORE.n:
                print(c_err("슬롯은 1~4 입니다"))
                prompt(c_dim("Enter..."))
                continue
            info = STORE.info(index)
            if info.empty:
                name = ask_name(DEFAULT_NAME)
                print(c_ok(f"새 경로는 슬롯 {index} 「{name}」에 저장합니다"))
                drive_new(slot_index=index, slot_name=name)
                continue
            drive_play(info)
            continue
        print(c_warn("1~4, n, q 중에서 고르세요"))
        prompt(c_dim("Enter..."))


if __name__ == "__main__":
    try:
        main()
    finally:
        restore_terminal()
