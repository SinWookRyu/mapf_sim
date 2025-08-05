#!/bin/bash

# MAPF 시뮬레이터 실행 스크립트

echo "MAPF 시뮬레이터를 시작합니다..."

# 워크스페이스 설정
source install/setup.bash

# 시뮬레이터 실행
echo "시뮬레이터, GUI, RViz를 시작합니다..."
ros2 launch mapf_simulator mapf_simulator.launch.py 