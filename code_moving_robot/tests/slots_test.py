"""슬롯 저장·목록. gpio 없이 PC에서 돈다."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robot.grid import OccupancyGrid
from robot.slots import DEFAULT_NAME, SlotStore


class FakeOdo:
    def __init__(self):
        self.x = 0.1
        self.y = 0.0
        self.yaw = 0.2


def test_empty_then_save_and_list(tmp_path):
    store = SlotStore(tmp_path / "slots", n=4)
    slots = store.list()
    assert len(slots) == 4
    assert all(s.empty for s in slots)
    grid = OccupancyGrid(size_m=8.0, resolution=0.05)
    info = store.save(
        2,
        grid,
        FakeOdo(),
        [0.0, 0.0, 0.0],
        [[0.5, 0.0, 0.0]],
        [1.0, 0.0, 0.0],
        "복도A",
    )
    assert info.empty is False
    assert info.name == "복도A"
    assert info.saved_at is not None
    loaded = store.load(2)
    assert loaded is not None
    data, _grid, meta = loaded
    assert data["name"] == "복도A"
    assert meta.name == "복도A"
    assert store.info(1).empty is True


def test_default_name(tmp_path):
    store = SlotStore(tmp_path / "slots")
    grid = OccupancyGrid(size_m=8.0, resolution=0.05)
    info = store.save(1, grid, FakeOdo(), [0, 0, 0], [], [1, 0, 0], "  ")
    assert info.name == DEFAULT_NAME


if __name__ == "__main__":
    import tempfile

    with tempfile.TemporaryDirectory() as td:
        test_empty_then_save_and_list(Path(td) / "a")
        test_default_name(Path(td) / "b")
    print("slots_test ok")
