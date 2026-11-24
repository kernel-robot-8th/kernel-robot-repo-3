# 시뮬레이션

이 폴더에는 두 가지가 있습니다.

| 폴더 | 내용 | 쓰는 때 |
|---|---|---|
| [`virtual_pinky_fleet/`](virtual_pinky_fleet/) | 가상 핑키 관제. 실제 핑키 관제와 같은 HTTP 주소로 답하는 핑키 세 대 | 핑키 없이 관제의 주문 흐름을 끝까지 돌려 볼 때 |
| [`ros2_ws/`](ros2_ws/) | 토크탁 창고 Gazebo Harmonic 월드와 bridge | ROS 2 / Nav2를 시뮬레이터에서 시험할 때 |

## 가상 핑키 관제

핑키 실물이 꺼져 있어도 주문 → 적재 자리 → 수령 구역 → 주차장 흐름을 끝까지 돌려 볼 수 있습니다. 모든 응답에 `simulated: true`가 붙습니다. 저장소 루트에서 실행합니다.

```bash
# 터미널 1: 가상 핑키 세 대 (8766번 포트, 실제 핑키 관제는 8765)
python3 simulation/virtual_pinky_fleet/virtual_pinky_fleet.py --time-scale 4

# 터미널 2: 가상 핑키에 붙인 관제 (실제 관제의 8000번 포트와 DB를 섞지 않음)
CONTROL_STATION_DB=/tmp/virtual-orders.sqlite3 CONTROL_STATION_PORT=8001 \
CONTROL_STATION_PINKY_FLEET_URL=http://127.0.0.1:8766 \
PYTHONPATH=src .venv/bin/python -m control_station.app
```

<http://127.0.0.1:8001>에서 주문을 넣으면 가상 핑키가 지도 위를 움직입니다. `CONTROL_STATION_DB`를 주지 않으면 실제 관제와 같은 `data/tasks.sqlite3`를 쓰니 꼭 따로 지정합니다.

가상으로 하지 않는 것: Nav2 경로 계획(점 사이는 직선), 장애물·충돌, 통로·적재 자리 예약, SSH, 배터리 소모. 자세한 동작은 [가상 핑키 관제 문서](../docs/implementation/virtual-pinky-fleet.md)에 있습니다.

## 토크탁 창고 Gazebo 월드

Figma 평면도를 바탕으로 만든 Gazebo Harmonic 창고 월드입니다.

- 외벽은 0.6 m, 내부 칸막이는 0.45 m로 낮춰 안이 잘 보이게 했습니다.
- 바닥은 따뜻한 색 콘크리트, 프레임은 짙은 철제, 화물은 주황·초록입니다.
- 2단 선반은 벽과 통로에서 여유를 두고 배치했습니다.

ROS 2 Jazzy와 Gazebo Harmonic을 호스트에서 직접 실행합니다.

### 실행

호스트에 ROS 2 Jazzy, Gazebo Harmonic, `ros_gz`가 설치되어 있어야 합니다. 저장소 루트에서 실행합니다.

```bash
cd simulation
./run.sh
```

스크립트가 ROS 패키지를 빌드한 뒤 Gazebo 창을 엽니다. 다시 실행하면 바뀐 부분만 다시 빌드합니다.

기본 ROS domain은 `31`입니다. 기존 Humble 컨테이너와 토픽이 섞이지 않게 하려는 값입니다.

- 관제도 시뮬레이터에 붙이려면 같은 터미널에서 `export ROS_DOMAIN_ID=31`을 설정한 뒤 실행합니다.
- 다른 domain을 쓰려면 `ROS_DOMAIN_ID=숫자 ./run.sh`로 지정합니다.

다른 터미널에서 키보드로 로봇을 움직이려면:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=31
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

GUI 없이 ROS/Gazebo 서버만 실행하려면:

```bash
./run.sh headless:=True
```

월드와 지도 원본은 `ros2_ws/src/toktak_warehouse/worlds`와 `ros2_ws/src/toktak_warehouse/maps`에 있습니다. Nav2에서 쓸 지도는 `toktak_warehouse.yaml`입니다.
