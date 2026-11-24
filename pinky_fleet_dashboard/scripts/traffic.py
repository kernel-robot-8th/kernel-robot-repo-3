"""공유 구역(교차로·5번 도킹·주차장) 예약. 연락이 끊긴 로봇이 점유한 구역은 저절로 풀리지 않는다.

실행 위치: PC·로봇 양쪽 (app.py가 import, 주행준비 때 runtime_bundle로 로봇에 올라감)
사용처: PC는 TrafficCoordinator(data/traffic_reservations.json에 저장), 로봇은 ultrasonic_safety·velocity_guard가
        나머지 함수로 필요한 구역과 /fleet/lease 허가(0.25초마다, 수명 1.5초)를 검사한다.
"""
import json,math,threading
from pathlib import Path


def required_zones(context,pose,zones):
    """이 핑키가 지금 필요한 공유 구역. 거리(request_radius_m) 기준이고, request_labels가 있는 구역은 끝점이
    그 정류지인 구간의 출발부터 요구한다(2026-11-17: 다음 핑키가 붐비는 곳 40 cm 앞이 아니라 지금 선 자리,
    즉 대기 자리나 주차 칸에서 기다린다)."""
    needed=[]
    # 주차장은 핑키가 그 안이나 바로 옆(경계 ±10 cm)에 있을 때만 점유한다(2026-11-17). 그곳에서 시작·끝나는
    # 구간 전체가 아니다: 이 핑키가 나가자마자 다른 핑키도 출발할 수 있어 세 대가 동시에 이동할 수 있다.
    if context.get('transit'):
        x0,x1,y0,y1=zones.get('parking',{}).get('bounds',[.2,1.2,.2,.6])
        if x0-.10<=pose[0]<=x1+.10 and y0-.10<=pose[1]<=y1+.10:needed.append('parking')
    if (context.get('dock_exit') or context.get('dock_entry')) and 'dock5' in zones:needed.append('dock5')
    for name,zone in zones.items():
        if name=='parking':continue
        if math.hypot(pose[0]-zone['x_m'],pose[1]-zone['y_m'])<=zone.get('request_radius_m',.40):needed.append(name)
        elif context.get('end_label') in zone.get('request_labels',()):needed.append(name)
    return sorted(set(needed))


# 허가가 이 회차(token)의 것이고 유효하며(남은 수명 0~5.1 s, 너무 먼 만료는 거부) 필요한 구역을 모두 담을 때만 True.
def lease_allows(lease,token,zones,now):
    try:return (lease.get('token')==token and 0<=lease['expires_monotonic']-now<=5.1 and set(zones)<=set(lease['zones']))
    except (TypeError,KeyError):return False


class TrafficCoordinator:
    """PC 쪽 예약 판정기. owners {구역: {robot_id, token}}를 파일에 남겨 재시작해도 유지하며, 파일이 손상되면 허가를 멈춘다."""
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
            # 주차장 요청은 주차 이송 내내 유지된다. 해제하려면 이 범위 검사와 함께
            # 최신의 명시적 '밖에 있음' 표시가 필요하다.
            x0,x1,y0,y1=self.zones['parking'].get('bounds',[.2,1.2,.2,.6])
            return not (x0<=pose[0]<=x1 and y0<=pose[1]<=y1)
        z=self.zones[name]
        # release_radius_m: 5번 자리를 떠나는 핑키는 충분히 멀어질 때까지 점유를 유지한다. 그래야 대기 자리의
        # 다음 핑키가 그 안으로 달려 들어가지 않는다. 없으면 request_radius_m + 5 cm.
        return math.hypot(pose[0]-z['x_m'],pose[1]-z['y_m'])>z.get('release_radius_m',z.get('request_radius_m',.4)+.05)
    def update(self,observations):
        """관측(로봇별 fresh·pose·zones·token)으로 점유를 갱신하고 {robot_id: 허가된 구역 목록}을 돌려준다.
        순서: 벗어났거나 주차를 마친 점유 해제 → 구역 중심 25 cm 안에 선 로봇에 점유 부여 → 요청 순서(queue)대로
        다른 핑키와 겹치지 않으면 허가. 저장에 실패하면 되돌린다."""
        with self.lock:
            if self.storage_error:raise ValueError(self.storage_error)
            before=dict(self.owners)
            for name,owner in list(self.owners.items()):
                o=observations.get(owner['robot_id'])
                if o and o.get('fresh') and ((name not in o.get('zones',[]) and self.outside(name,o['pose'])) or (name=='parking' and o.get('parked'))):
                    del self.owners[name]
            # 최신 위치상 실제로 서 있는 로봇은 경로가 없어도 교차로를 점유해 보호한다.
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
                # 다른 핑키의 점유만 충돌이다(2026-11-17). 경로 없이 구역에 서 있던 핑키는 'occupied'(또는 지난
                # 회차)로 점유하고, 자기 새 회차가 그것을 넘겨받는다. 자기 자신을 영원히 기다리지 않게 한다.
                conflict=[n for n in needed if n in self.owners and self.owners[n].get('robot_id') != rid]
                if conflict:self.waiting[rid]=conflict;continue
                for name in needed:self.owners[name]={'robot_id':rid,'token':o['token']}
            if self.owners!=before:
                try:self.save()
                except OSError:self.owners=before;raise
            return {rid:[n for n,owner in self.owners.items() if owner=={'robot_id':rid,'token':o.get('token')}]
                    for rid,o in observations.items() if o.get('fresh')}
    def release(self,name,robot_id,token,observation):
        """운영자의 수동 해제. 같은 회차이고, 최신 위치가 구역 밖이거나 자기 주차 칸에 정지했을 때만 푼다."""
        with self.lock:
            if self.owners.get(name)!={'robot_id':robot_id,'token':token}:raise ValueError('예약 회차가 바뀌었습니다.')
            if not observation or not observation.get('fresh'):raise ValueError('최신 위치를 확인할 수 없어 점유를 유지합니다.')
            if not (self.outside(name,observation['pose']) or (name=='parking' and observation.get('at_home_and_stopped'))):
                raise ValueError('로봇이 공유 구역 밖에 있거나 자기 주차장에 정지했는지 확인하세요.')
            previous=dict(self.owners);del self.owners[name]
            try:self.save()
            except OSError:self.owners=previous;raise

    def drop(self,robot_id):
        """운영자가 이 핑키는 더 이상 점유 구역에 없다고 알린다(꺼져서 사람이 들어 옮긴 경우): 점유를 모두 푼다.
        푼 구역 이름 목록을 돌려준다."""
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
    """로봇 쪽 허가 수신기. 휘발성 heartbeat로 로컬 수명(TTL)을 만들고 중복·역순 패킷은 버린다.
    boot는 관제 실행마다 바뀌는 ID다. 지나간 boot(retired)는 다시 받지 않아 옛 관제의 늦은 패킷을 막는다.
    수명은 로봇 monotonic 수신 시각 + 1.5 s라 PC와 로봇 시계가 달라도 된다."""
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
