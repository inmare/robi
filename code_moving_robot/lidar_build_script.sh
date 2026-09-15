#!/usr/bin/env bash
# code_moving_robot 안에서 실행해도, 다른 경로에서 이 파일을 호출해도
# 스크립트가 있는 폴더(code_moving_robot)로 이동한 뒤 SDK를 .venv에 넣는다.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

SDK_DIR="$ROOT/YDLidar-SDK"
VENV_DIR="$ROOT/.venv"

if [ ! -d "$SDK_DIR/.git" ]; then
  git clone https://github.com/YDLIDAR/YDLidar-SDK.git "$SDK_DIR"
fi

cd "$SDK_DIR"
rm -rf build
mkdir -p build
cd build
cmake ..
make
sudo make install

export PATH="$HOME/.local/bin:$PATH"
if ! command -v uv >/dev/null 2>&1; then
  curl -LsSf https://astral.sh/uv/install.sh | sh
  # shellcheck disable=SC1091
  source "$HOME/.local/bin/env"
fi

cd "$ROOT"
uv venv "$VENV_DIR"
uv pip install --python "$VENV_DIR/bin/python" "$SDK_DIR"

echo "설치 끝. 라이다 시험 (끝나면 모터 OFF):"
echo "  $VENV_DIR/bin/python $ROOT/tests/lidar_test.py"
echo "제자리 지도:"
echo "  $VENV_DIR/bin/python $ROOT/tests/map_scan_test.py"
