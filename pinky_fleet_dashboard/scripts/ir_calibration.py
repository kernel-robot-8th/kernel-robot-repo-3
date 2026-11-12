"""Store raw lateral IR sweeps independently from the driving calibration."""
import json
import math
import os
from pathlib import Path
import statistics
import tempfile
import threading
import uuid


class IRCalibrationStore:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.lock = threading.RLock()

    def records(self, robot):
        with self.lock:
            path = self.directory / f'pinky_{robot}.json'
            return json.loads(path.read_text()) if path.exists() else []

    def append(self, robot, session, offset, samples):
        if len(samples) != 20 or any(len(row['raw']) != 3 or any(
                type(v) not in (int, float) or not math.isfinite(v) or not 0 <= v <= 4095
                for v in row['raw']) for row in samples):
            raise ValueError('유효한 새 IR 원시 샘플 20개가 필요합니다.')
        channels = list(zip(*(row['raw'] for row in samples)))
        record = dict(id=uuid.uuid4().hex, robot=robot, session=session, offset_cm=offset,
                      captured_at=samples[0]['received_at'], samples=samples,
                      mean=[statistics.mean(c) for c in channels],
                      std=[statistics.pstdev(c) for c in channels],
                      minimum=[min(c) for c in channels], maximum=[max(c) for c in channels])
        with self.lock:
            records = self.records(robot)
            if len(records) >= 10000:
                raise ValueError('수집 기록 한도에 도달했습니다.')
            records.append(record)
            self.directory.mkdir(parents=True, exist_ok=True)
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
