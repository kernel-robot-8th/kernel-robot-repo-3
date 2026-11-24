"""크기 한도가 있고 호출한 쪽을 막지 않는 JSONL 기록기. 비밀번호 같은 자격 증명과 HTTP 요청 본문은 절대 기록하지 않는다.

실행 위치: PC·로봇 양쪽 (로봇에는 주행준비 때 runtime_bundle로 올라감)
사용처: app.py(PC `logs/telemetry.jsonl`), 로봇의 ultrasonic_safety.py(`logs/behavior.jsonl`)와
velocity_guard.py(`logs/final-guard.jsonl`).

파일 하나는 5 MB까지, 이전 파일은 4개까지 돌려 쓴다. 디스크 쓰기는 QueueListener 스레드가 따로 하므로 20 Hz 제어
루프가 디스크를 기다리지 않는다. 큐(1000개)가 차면 기다리지 않고 버린다. 같은 사건이 같은 상태로 되풀이되면
0.2초에 한 번(5 Hz)만 남기고, 상태가 바뀌면 바로 남긴다.
"""
import atexit,json,logging,time,math,queue,threading
from logging.handlers import RotatingFileHandler,QueueHandler,QueueListener
from pathlib import Path

def clean(value):
    """JSON에 못 넣는 NaN·inf를 None으로 바꾼다(dict·list 안까지). json.dumps(allow_nan=False)가 실패하지 않게 한다."""
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,dict):return {str(k):clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [clean(v) for v in value]
    return value

class DroppingQueueHandler(QueueHandler):
    """큐가 차면 기록을 조용히 버린다. 기본 QueueHandler는 queue.Full을 오류로 처리해 stderr에 traceback을 남긴다."""
    def enqueue(self,record):
        try:self.queue.put_nowait(record)
        except queue.Full:pass

class ControlJournal:
    """한 파일에 쓰는 JSONL 기록기. record()는 큐에 넣기만 하고 바로 돌아온다."""
    def __init__(self,path):
        path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
        # logging.getLogger가 아니라 직접 만든 Logger라 루트 로거로 전파되지 않고 다른 핸들러와 섞이지 않는다.
        self.logger=logging.Logger(str(path))
        self.handler=RotatingFileHandler(path,maxBytes=5_000_000,backupCount=4)
        self.handler.setFormatter(logging.Formatter('%(message)s'))
        self.queue=queue.Queue(maxsize=1000)
        self.logger.addHandler(DroppingQueueHandler(self.queue))
        self.listener=QueueListener(self.queue,self.handler);self.listener.start()
        # tracks: (event, robot_id)마다 마지막으로 남긴 (state, run_id)와 그 시각. 반복 기록을 줄이는 데 쓴다.
        self.tracks={};self.lock=threading.Lock();self.closed=False
        atexit.register(self.close)
    def record(self,event,**fields):
        """사건 하나를 JSON 한 줄로 남긴다. 같은 (event, robot_id)가 같은 (state, run_id)로 0.2초 안에 다시 오면 버린다."""
        now=time.monotonic();key=(event,fields.get('robot_id'))
        signature=(fields.get('state'),fields.get('run_id'))
        with self.lock:
            previous,at=self.tracks.get(key,(None,0.))
            if signature==previous and now-at<.2:return
            self.tracks[key]=(signature,now)
        self.logger.info(json.dumps(clean(dict(time=time.time(),monotonic=now,event=event,**fields)),allow_nan=False))
    def close(self):
        """atexit에서도 불린다. 큐가 꽉 차 listener.stop()이 종료 표시를 못 넣으면(queue.Full) 핸들러를 닫지 않고 끝낸다."""
        if self.closed:return
        self.closed=True
        try:self.listener.stop()
        except queue.Full:return
        self.handler.close()
