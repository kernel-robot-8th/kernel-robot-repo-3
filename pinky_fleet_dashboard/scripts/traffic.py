"""공유 구역(교차로·5번 도킹·주차장) 예약. 연락이 끊긴 로봇의 점유는 저절로 풀리지 않는다.

실행 위치: PC(app.py)·로봇(runtime_bundle). 자세한 설명: pinky_fleet_dashboard/README.md
"""
import json,math,threading
from pathlib import Path


def required_zones(context,pose,zones):
    """이 핑키가 지금 필요한 공유 구역(request_radius_m 거리, request_labels는 그 정류지로 가는 구간 출발부터:
    2026-11-17부터 지금 선 자리에서 기다린다)."""
    needed=[]
    # 주차장은 안이나 경계 ±10 cm일 때만 점유(2026-11-17): 세 대가 동시에 이동할 수 있게.
    if context.get('transit'):
        x0,x1,y0,y1=zones.get('parking',{}).get('bounds',[.2,1.2,.2,.6])
        if x0-.10<=pose[0]<=x1+.10 and y0-.10<=pose[1]<=y1+.10:needed.append('parking')
    if (context.get('dock_exit') or context.get('dock_entry')) and 'dock5' in zones:needed.append('dock5')
    for name,zone in zones.items():
        if name=='parking':continue
        if math.hypot(pose[0]-zone['x_m'],pose[1]-zone['y_m'])<=zone.get('request_radius_m',.40):needed.append(name)
        elif context.get('end_label') in zone.get('request_labels',()):needed.append(name)
    return sorted(set(needed))


# 이 회차(token)의 유효한 허가(남은 수명 0~5.1 s)가 필요한 구역을 모두 담을 때만 True.
def lease_allows(lease,token,zones,now):
    try:return (lease.get('token')==token and 0<=lease['expires_monotonic']-now<=5.1 and set(zones)<=set(lease['zones']))
    except (TypeError,KeyError):return False


class TrafficCoordinator:
    """PC 쪽 예약 판정기. owners를 파일에 남겨 재시작에도 유지하고, 파일이 손상되면 허가를 멈춘다."""
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
            # 주차 이송 중 유지. 해제엔 범위 밖 + 최신 '밖에 있음' 표시가 필요.
            x0,x1,y0,y1=self.zones['parking'].get('bounds',[.2,1.2,.2,.6])
            return not (x0<=pose[0]<=x1 and y0<=pose[1]<=y1)
        z=self.zones[name]
        # 5번을 떠나는 핑키는 충분히 멀어질 때까지 점유를 유지한다(대기 핑키 진입 방지).
        return math.hypot(pose[0]-z['x_m'],pose[1]-z['y_m'])>z.get('release_radius_m',z.get('request_radius_m',.4)+.05)
    def update(self,observations):
        """관측으로 점유를 갱신하고 {robot_id: 허가된 구역}을 돌려준다. 저장에 실패하면 되돌린다."""
        with self.lock:
            if self.storage_error:raise ValueError(self.storage_error)
            before=dict(self.owners)
            for name,owner in list(self.owners.items()):
                o=observations.get(owner['robot_id'])
                if o and o.get('fresh') and ((name not in o.get('zones',[]) and self.outside(name,o['pose'])) or (name=='parking' and o.get('parked'))):
                    del self.owners[name]
            # 경로가 없어도 서 있는 로봇은 교차로를 점유해 보호한다.
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
                # 다른 핑키의 점유만 충돌(2026-11-17). 자기 옛 회차 점유는 새 회차가 넘겨받는다.
                conflict=[n for n in needed if n in self.owners and self.owners[n].get('robot_id') != rid]
                if conflict:self.waiting[rid]=conflict;continue
                for name in needed:self.owners[name]={'robot_id':rid,'token':o['token']}
            if self.owners!=before:
                try:self.save()
                except OSError:self.owners=before;raise
            return {rid:[n for n,owner in self.owners.items() if owner=={'robot_id':rid,'token':o.get('token')}]
                    for rid,o in observations.items() if o.get('fresh')}
    def release(self,name,robot_id,token,observation):
        """운영자 수동 해제. 같은 회차이고 구역 밖이거나 자기 주차 칸에 정지했을 때만 푼다."""
        with self.lock:
            if self.owners.get(name)!={'robot_id':robot_id,'token':token}:raise ValueError('예약 회차가 바뀌었습니다.')
            if not observation or not observation.get('fresh'):raise ValueError('최신 위치를 확인할 수 없어 점유를 유지합니다.')
            if not (self.outside(name,observation['pose']) or (name=='parking' and observation.get('at_home_and_stopped'))):
                raise ValueError('로봇이 공유 구역 밖에 있거나 자기 주차장에 정지했는지 확인하세요.')
            previous=dict(self.owners);del self.owners[name]
            try:self.save()
            except OSError:self.owners=previous;raise

    def drop(self,robot_id):
        """운영자 알림(꺼져서 옮김): 이 핑키의 점유를 모두 풀고 구역 이름을 돌려준다."""
        with self.lock:
            names=[n for n,o in self.owners.items() if o.get('robot_id')==robot_id]
            if not names:return []
            previous=dict(self.owners)
            for n in names:del self.owners[n]
            try:self.save()
            except OSError:self.owners=previous;raise
            return names

    def snapshot(self):
        with self.lock:return {'owners':dict(self.owners),'waiting':dict(self.waiting),'error':self.storage_error}


class LeaseReceiver:
    """로봇 쪽 허가 수신기. 수명은 로봇 수신 시각 + 1.5 s(시계 차이 무관). 중복·역순 패킷과 지난 boot는 버린다."""
    def __init__(self):self.boot=None;self.sequence=-1;self.retired=set();self.lease={}
    def receive(self,data,now):
        if not isinstance(data,dict) or not isinstance(data.get('boot'),str):return False
        sequence=data.get('sequence')
        if not isinstance(sequence,int) or isinstance(sequence,bool):return False
        if not isinstance(data.get('token'),str) or not isinstance(data.get('zones'),list):return False
        if len(data['zones'])>6 or any(not isinstance(z,str) for z in data['zones']):return False
        if data['boot'] in self.retired:return False
        if data['boot']!=self.boot:
            if self.boot:self.retired.add(self.boot)
            self.boot,self.sequence=data['boot'],-1
        if sequence<=self.sequence:return False
        self.sequence=sequence
        self.lease=dict(token=data['token'],zones=data['zones'],expires_monotonic=now+1.5)
        return True
