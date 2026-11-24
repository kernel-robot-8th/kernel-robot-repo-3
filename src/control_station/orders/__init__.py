"""주문: 접수(desk) → 배정(assignment, planning) → 단계 실행(flow, task_runner) → 진행 화면(progress_view).

자동 출발(dispatcher), 재고(inventory), 수령 구역(receiving_zones), 주문 문장 해석(command_parser)도 여기 있다.
이 부품들은 control_station/app.py가 만들어 서로 잇는다(OrderFlow → MockTaskOrchestrator → OrderDesk → OrderDispatcher).
"""
