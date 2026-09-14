# 여기에서 /dev/ttyUSB0이 있어야 함
lsusb
ls /dev/ttyUSB*

# 권한
sudo usermod -aG dialout $USER
sudo chmod 666 /dev/ttyUSB0

# 설치
sudo apt update
sudo apt install -y cmake make build-essential swig python3-dev python3-pip git
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir -p build && cd build
cmake ..
make
sudo make install
cd ..

# uv 가상환경 설정
curl -LsSf https://astral.sh/uv/install.sh | sh
source $HOME/.local/bin/env

# 빌드
cd  경로/YDLidar-SDK
uv venv ~/ydlidar-venv
source ~/ydlidar-venv/bin/activate
uv pip install .

# 실행
source $HOME/.local/bin/env
~/ydlidar-venv/bin/python 경로/lidar_test.py