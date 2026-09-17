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
    lines.append(c_info("n)  새 경로 만들기"))
    lines.append(c_warn("o)  슬롯 덮어쓰기"))
    lines.append("q)  종료")
    return lines


def show_main_menu():
    slots = STORE.list()
    clear_screen()
    print(draw_box("ROBI 이동 로봇", main_menu_lines(slots)))
    print()
    print(c_dim("번호 = 슬롯 선택 · 빈 슬롯은 새 기록 · n 새 경로 · o 덮어쓰기"))
    return prompt(c_warn("선택: ")).strip().lower()


def slot_submenu(info):
    clear_screen()
    lines = [
        c_ok(info.name),
        c_dim(info.saved_at or ""),
        "",
        "1)  재생  — 경로를 갔다가 같은 길로 시작점",
        "2)  덮어쓰기 — 이 슬롯에 새로 기록",
        "b)  뒤로",
    ]
    print(draw_box(f"슬롯 {info.index}", lines))
    raw = prompt(c_warn("선택 [1 재생]: ")).strip().lower()
    if raw in ("2", "o", "overwrite", "덮어쓰기"):
        return "overwrite"
    if raw in ("b", "q", "n"):
        return "back"
    return "play"


def choose_save_slot():
    slots = STORE.list()
    print()
    print(c_info("저장할 슬롯"))
    for s in slots:
        mark = c_dim("[비어 있음]") if s.empty else c_ok(s.label())
        print(f"  {s.index})  {mark}")
    raw = prompt(c_warn("번호 (Enter 취소): ")).strip()
    if not raw.isdigit():
        return None
    index = int(raw)
    if index < 1 or index > STORE.n:
        print(c_err("없는 슬롯입니다"))
        return None
    return index


def confirm_overwrite(info):
    raw = prompt(
        c_warn(
            f"슬롯 {info.index} 「{info.name}」 ({info.saved_at}) 을 덮어쓸까요? [y/N] "
        )
    ).strip().lower()
    return raw in ("y", "yes", "ㅛ")


def ask_name(default=""):
    shown = default or DEFAULT_NAME
    raw = prompt(c_info(f"경로 이름 [{shown}]: ")).strip()
    return raw or shown


def after_drive(result, *, force_overwrite=False):
    if result.get("grid") is None:
        return
    if not result.get("has_route"):
        print(c_warn("저장할 경로가 없습니다"))
        return
    slot_index = result.get("slot_index")
    name = result.get("name") or DEFAULT_NAME
    dirty = result.get("dirty") or result.get("map_writable")
    if slot_index is None:
        slot_index = choose_save_slot()
        if slot_index is None:
            print(c_warn("저장 취소"))
            return
        info = STORE.info(slot_index)
        if not info.empty and not confirm_overwrite(info):
            print(c_warn("저장 취소"))
            return
        if info.empty:
            name = ask_name(name)
        else:
            name = ask_name(info.name)
    elif dirty or force_overwrite:
        info = STORE.info(slot_index)
        if name == DEFAULT_NAME:
            name = ask_name(info.name if not info.empty else name)
    else:
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


def drive_new(slot_index=None, slot_name=DEFAULT_NAME, data=None, grid=None):
    result = run_drive(
        store=STORE,
        slot_index=slot_index,
        slot_name=slot_name,
        recording=True,
        map_writable=True,
        grid=grid,
        data=data,
        center_host=CENTER_HOST,
        center_port=CENTER_PORT,
    )
    after_drive(result, force_overwrite=slot_index is not None)
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
            drive_new()
            continue
        if raw in ("o",):
            index = choose_save_slot()
            if index is None:
                continue
            info = STORE.info(index)
            if not info.empty and not confirm_overwrite(info):
                continue
            data, grid = None, None
            loaded = None if info.empty else STORE.load(index)
            if loaded is not None:
                data, grid, _info = loaded
            drive_new(
                slot_index=index,
                slot_name=info.name if not info.empty else DEFAULT_NAME,
                data=data,
                grid=grid,
            )
            continue
        if raw.isdigit():
            index = int(raw)
            if index < 1 or index > STORE.n:
                print(c_err("슬롯은 1~4 입니다"))
                prompt(c_dim("Enter..."))
                continue
            info = STORE.info(index)
            if info.empty:
                drive_new(slot_index=index)
                continue
            action = slot_submenu(info)
            if action == "back":
                continue
            if action == "overwrite":
                if not confirm_overwrite(info):
                    continue
                data, grid = None, None
                loaded = STORE.load(info.index)
                if loaded is not None:
                    data, grid, _info = loaded
                drive_new(
                    slot_index=info.index,
                    slot_name=info.name,
                    data=data,
                    grid=grid,
                )
                continue
            drive_play(info)
            continue
        print(c_warn("1~4, n, o, q 중에서 고르세요"))
        prompt(c_dim("Enter..."))


if __name__ == "__main__":
    main()
