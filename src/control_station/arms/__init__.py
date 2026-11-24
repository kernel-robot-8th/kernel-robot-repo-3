"""로봇팔(OMX 두 대, LeRobot ACT 정책) 쪽 관제 기능.

관제는 팔을 직접 움직이지 않는다. 명령을 기록하고 넘기면, 팔 쪽 워커(LeRobot)나 ROS 2 액션 서버가 실행하고
결과를 알려 준다.

    config.py         어느 팔이 어느 선반을 맡는지(config/arms.json), 팔마다 포트·카메라 장치(환경 변수)
    commands.py       팔 명령 큐: 명령 기록, 워커의 long-poll 수령, 상태 전이, UNKNOWN 규칙
    pick.py           주문 흐름의 실제 집기: PICK_WITH_POLICY를 HTTP 큐 또는 ROS 2 액션으로 보냄
    ros_action.py     ROS 2 집기 액션 클라이언트 (/arm_0N/pick_object)
    policies.py       물체 이름 -> 학습 정책 (config/omx-act-policies.json)
    policy_runner.py  관제에서 직접 띄우는 LeRobot 정책 프로세스 1회 실행
    sequences.py      운영자가 고른 정책 여러 개를 차례로 실행하는 연속 실행
"""
