"""Bounded, nonblocking JSONL records; never log credentials/request bodies."""
import atexit,json,logging,time,math,queue,threading
from logging.handlers import RotatingFileHandler,QueueHandler,QueueListener
from pathlib import Path

def clean(value):
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,dict):return {str(k):clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [clean(v) for v in value]
    return value

class DroppingQueueHandler(QueueHandler):
    def enqueue(self,record):
        try:self.queue.put_nowait(record)
        except queue.Full:pass

class ControlJournal:
    def __init__(self,path):
        path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
        self.logger=logging.Logger(str(path))
        self.handler=RotatingFileHandler(path,maxBytes=5_000_000,backupCount=4)
        self.handler.setFormatter(logging.Formatter('%(message)s'))
        self.queue=queue.Queue(maxsize=1000)
        self.logger.addHandler(DroppingQueueHandler(self.queue))
        self.listener=QueueListener(self.queue,self.handler);self.listener.start()
        self.tracks={};self.lock=threading.Lock();self.closed=False
        atexit.register(self.close)
    def record(self,event,**fields):
        now=time.monotonic();key=(event,fields.get('robot_id'))
        signature=(fields.get('state'),fields.get('run_id'))
        with self.lock:
            previous,at=self.tracks.get(key,(None,0.))
            if signature==previous and now-at<.2:return
            self.tracks[key]=(signature,now)
        self.logger.info(json.dumps(clean(dict(time=time.time(),monotonic=now,event=event,**fields)),allow_nan=False))
    def close(self):
        if self.closed:return
        self.closed=True
        try:self.listener.stop()
        except queue.Full:return
        self.handler.close()
