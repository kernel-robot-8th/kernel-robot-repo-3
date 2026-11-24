"""IR 보정 페이지에서 모은 가로 방향 IR 원시 측정(sweep)을 주행 보정값과 따로 저장한다.

실행 위치: PC 관제(app.py가 import)
사용처: app.py의 IR_CALIBRATION (data/ir_calibration/pinky_<번호>.json)

핵심: 선 중심에서 로봇을 옆으로 offset_cm만큼 옮겨 두고 IR 3채널 원시값 20개를 한 기록으로 남긴다.
채널별 평균·표준편차·최소·최대를 같이 저장해 나중에 오프셋별 IR 반응 곡선을 만들 수 있게 한다.
주행에 쓰이는 프로파일(ir_profiles.py)은 이 기록으로 자동으로 바뀌지 않는다.
"""
import json
import math
import os
from pathlib import Path
import statistics
import tempfile
import threading
import uuid


class IRCalibrationStore:
    """로봇별 JSON 파일 하나에 기록 목록을 쌓는다. HTTP 요청 스레드가 동시에 부를 수 있어 RLock으로 묶는다."""
    def __init__(self, directory):
        self.directory = Path(directory)
        self.lock = threading.RLock()

    def records(self, robot):
        with self.lock:
            path = self.directory / f'pinky_{robot}.json'
            return json.loads(path.read_text()) if path.exists() else []

    def append(self, robot, session, offset, samples):
        """샘플 20개(각 raw는 IR 3채널, 12비트 ADC라 0~4095)를 검사해 한 기록으로 덧붙이고 그 기록을 돌려준다."""
        if len(samples) != 20 or any(len(row['raw']) != 3 or any(
                type(v) not in (int, float) or not math.isfinite(v) or not 0 <= v <= 4095
                for v in row['raw']) for row in samples):
            raise ValueError('유효한 새 IR 원시 샘플 20개가 필요합니다.')
        # 행(샘플) 목록을 채널별 열로 뒤집는다: [(ch1 20개), (ch2 20개), (ch3 20개)]
        channels = list(zip(*(row['raw'] for row in samples)))
        record = dict(id=uuid.uuid4().hex, robot=robot, session=session, offset_cm=offset,
                      captured_at=samples[0]['received_at'], samples=samples,
                      mean=[statistics.mean(c) for c in channels],
                      std=[statistics.pstdev(c) for c in channels],
                      minimum=[min(c) for c in channels], maximum=[max(c) for c in channels])
        with self.lock:
            records = self.records(robot)
            # 로봇당 기록 1만 개 상한: 매번 파일 전체를 다시 쓰므로 무한히 커지지 않게 막는다.
            if len(records) >= 10000:
                raise ValueError('수집 기록 한도에 도달했습니다.')
            records.append(record)
            self.directory.mkdir(parents=True, exist_ok=True)
            # 임시 파일에 쓴 뒤 os.replace로 바꿔 끼운다. 실패하면 finally에서 임시 파일을 지운다.
            temporary = None
            try:
                with tempfile.NamedTemporaryFile('w', dir=self.directory, delete=False, encoding='utf-8') as stream:
                    temporary = stream.name
                    json.dump(records, stream, ensure_ascii=False, allow_nan=False)
                os.replace(temporary, self.directory / f'pinky_{robot}.json')
            finally:
                if temporary and os.path.exists(temporary):
                    os.unlink(temporary)
        return record
