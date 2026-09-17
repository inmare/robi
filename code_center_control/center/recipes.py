"""원자 명령을 모아 둔 레시피.

상자 펌웨어는 한 번에 한 동작만 한다. 반납 한 권, 원점 복귀 같은
흐름은 여기서 순서를 고정해서 나중에 같은 이름으로 다시 보낸다.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Step:
    cmd: str
    title: str


RECIPES: dict[str, tuple[Step, ...]] = {
    "home": (
        Step("pusher.back", "푸셔를 뒤 리미트(원점)까지"),
        Step("lift.down", "리프트를 하단 리미트까지"),
    ),
    "discharge": (
        Step("lift.up", "맨 위 책이 배출 높이가 될 때까지 상승"),
        Step("pusher.forward", "책을 앞으로 밀기"),
        Step("pusher.back", "푸셔 원점 복귀"),
    ),
    "return_book": (
        Step("pusher.back", "밀기 전에 푸셔 원점"),
        Step("lift.up", "배출 높이까지 상승"),
        Step("pusher.forward", "책 밀기"),
        Step("pusher.back", "푸셔 복귀"),
        Step("lift.down", "리프트 하단 복귀"),
    ),
}

RECIPE_HELP = {
    "home": "안전 원점. 수동 시험 시작 전에 씀",
    "discharge": "지금 맨 위 책 한 권을 밀어 냄. 쌓인 책을 반복할 때 이 레시피",
    "return_book": "원점 → 한 권 배출 → 리프트 내리기. 시연용 한 사이클",
}
