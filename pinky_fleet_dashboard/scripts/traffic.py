"""Persistent reservations. A missing robot never frees an occupied resource."""
import json,math,threading
from pathlib import Path


def required_zones(context,pose,zones):
    needed=[]
    if context.get('transit'):needed.append('parking')
    if context.get('dock_exit'):needed.append('dock5')
    for name,zone in zones.items():
        if name=='parking':continue
        if math.hypot(pose[0]-zone['x_m'],pose[1]-zone['y_m'])<=zone.get('request_radius_m',.40):needed.append(name)
    return sorted(needed)


def lease_allows(lease,token,zones,now):
    try:return (lease.get('token')==token and 0<=lease['expires_monotonic']-now<=5.1 and set(zones)<=set(lease['zones']))
    except (TypeError,KeyError):return False


class TrafficCoordinator:
    def __init__(self,path,zones):
        self.path=Path(path);self.zones=zones;self.lock=threading.RLock();self.owners={};self.waiting={};self.storage_error='';self.queue={};self.counter=0
        try:self.owners=json.loads(self.path.read_text())
        except FileNotFoundError:pass
        except (OSError,ValueError) as exc:self.storage_error=str(exc)
        if not isinstance(self.owners,dict) or any(n not in zones or not isinstance(o,dict) or not all(isinstance(o.get(k),str) for k in ('robot_id','token')) for n,o in self.owners.items()):self.storage_error='Invalid reservation state';self.owners={}
    def save(self):
        self.path.parent.mkdir(parents=True,exist_ok=True)
        tmp=self.path.with_suffix('.tmp');tmp.write_text(json.dumps(self.owners));tmp.replace(self.path)
    def outside(self,name,pose):
        if name=='parking':
            # Route request is held through the whole parking transfer; release
            # needs a fresh explicit outside flag in addition to this envelope.
            x0,x1,y0,y1=self.zones['parking'].get('bounds',[.2,1.2,.2,.6])
            return not (x0<=pose[0]<=x1 and y0<=pose[1]<=y1)
        z=self.zones[name]
        return math.hypot(pose[0]-z['x_m'],pose[1]-z['y_m'])>z.get('request_radius_m',.4)+.05
    def update(self,observations):
        with self.lock:
            if self.storage_error:raise ValueError(self.storage_error)
            before=dict(self.owners)
            for name,owner in list(self.owners.items()):
                o=observations.get(owner['robot_id'])
                if o and o.get('fresh') and ((name not in o.get('zones',[]) and self.outside(name,o['pose'])) or (name=='parking' and o.get('parked'))):
                    del self.owners[name]
            # Fresh physical occupancy protects a stopped robot at a junction.
            for name,z in self.zones.items():
                if name=='parking' or name in self.owners:continue
                for rid,o in observations.items():
                    if o.get('fresh') and math.hypot(o['pose'][0]-z['x_m'],o['pose'][1]-z['y_m'])<.25:
                        self.owners[name]={'robot_id':rid,'token':o.get('token') or 'occupied'}
                        break
            self.waiting={}
            for rid,o in observations.items():
                if o.get('zones') and rid not in self.queue:self.counter+=1;self.queue[rid]=self.counter
                if not o.get('zones'):self.queue.pop(rid,None)
            for rid,o in sorted(observations.items(),key=lambda item:self.queue.get(item[0],float('inf'))):
                if not o.get('fresh') or not o.get('token'):continue
                needed=o.get('zones',[])
                if any(n not in self.zones for n in needed):continue
                conflict=[n for n in needed if n in self.owners and self.owners[n] != {'robot_id':rid,'token':o['token']}]
                if conflict:self.waiting[rid]=conflict;continue
                for name in needed:self.owners[name]={'robot_id':rid,'token':o['token']}
            if self.owners!=before:
                try:self.save()
                except OSError:self.owners=before;raise
            return {rid:[n for n,owner in self.owners.items() if owner=={'robot_id':rid,'token':o.get('token')}]
                    for rid,o in observations.items() if o.get('fresh')}
    def release(self,name,robot_id,token,observation):
        with self.lock:
            if self.owners.get(name)!={'robot_id':robot_id,'token':token}:raise ValueError('예약 회차가 바뀌었습니다.')
            if not observation or not observation.get('fresh'):raise ValueError('최신 위치를 확인할 수 없어 점유를 유지합니다.')
            if not (self.outside(name,observation['pose']) or (name=='parking' and observation.get('at_home_and_stopped'))):
                raise ValueError('로봇이 공유 구역 밖에 있거나 자기 주차장에 정지했는지 확인하세요.')
            previous=dict(self.owners);del self.owners[name]
            try:self.save()
            except OSError:self.owners=previous;raise

    def snapshot(self):
        with self.lock:return {'owners':dict(self.owners),'waiting':dict(self.waiting),'error':self.storage_error}


class LeaseReceiver:
    """Local TTL from a volatile heartbeat; reject duplicates/reordered packets."""
    def __init__(self):self.boot=None;self.sequence=-1;self.retired=set();self.lease={}
    def receive(self,data,now):
        if not isinstance(data,dict) or not isinstance(data.get('boot'),str):return False
        sequence=data.get('sequence')
        if not isinstance(sequence,int) or isinstance(sequence,bool):return False
        if not isinstance(data.get('token'),str) or not isinstance(data.get('zones'),list):return False
        if len(data['zones'])>3 or any(not isinstance(z,str) for z in data['zones']):return False
        if data['boot'] in self.retired:return False
        if data['boot']!=self.boot:
            if self.boot:self.retired.add(self.boot)
            self.boot,self.sequence=data['boot'],-1
        if sequence<=self.sequence:return False
        self.sequence=sequence
        self.lease=dict(token=data['token'],zones=data['zones'],expires_monotonic=now+1.5)
        return True
