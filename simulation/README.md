# 토크탁 창고 시뮬레이션

Figma 평면도를 바탕으로 만든 Gazebo Harmonic 창고 월드입니다. 외벽은 0.6m, 내부 칸막이는 0.45m로 낮춰 내부가 잘 보이게 했습니다. 따뜻한 콘크리트 바닥과 짙은 철제 프레임, 주황·초록 화물로 구성했고, 벽과 통로에서 여유를 둔 2단 선반을 배치했습니다. ROS 2 Jazzy와 Gazebo Harmonic을 호스트에서 직접 실행합니다.

## 실행

호스트에 ROS 2 Jazzy, Gazebo Harmonic, `ros_gz`가 설치된 상태에서 실행합니다.

```bash
cd ~/Desktop/kernel-robot-repo-3/simulation
./run.sh
```

스크립트가 ROS 패키지를 빌드한 뒤 Gazebo 창을 엽니다. 이후 다시 실행하면 변경 사항을 재빌드합니다.

기존 Humble 컨테이너와 토픽이 섞이지 않도록 기본 ROS domain은 `31`입니다. 관제 프로그램도 시뮬레이터에 연결하려면 같은 터미널에서 `export ROS_DOMAIN_ID=31`을 설정해 실행하세요. 다른 domain을 사용하려면 `ROS_DOMAIN_ID=숫자 ./run.sh` 형식으로 지정할 수 있습니다.

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

월드와 지도 원본은 `ros2_ws/src/toktak_warehouse/worlds`와 `ros2_ws/src/toktak_warehouse/maps`에 있습니다. Nav2에서 사용할 지도는 `toktak_warehouse.yaml`입니다.
