"""두 핑키가 정면으로 마주쳤을 때 누가 길을 비킬지 정한다(2026-11-24).

실행 위치: PC 관제(app.py가 YieldArbiter, PROTECTED_PHASES를 import).
사용처: app.py가 주기적으로 update()를 부르고, 결과 지시를 각 로봇 도메인의 /fleet/yield로 발행한다.
로봇에서는 mission_behaviors.GiveWay가 그 지시를 실행한다.

구역(scripts/traffic.py)은 통로와 5번 적재 자리에 한 번에 한 대만 들이지만, 구역 밖에서는 아무도 결정하지
않았다. 코끝을 마주한 두 핑키가 서로를 위해 멈추고, 조금 물러났다가 다시 시도했고, 통로 입구에서는 들어가려고
기다리던 핑키가 나오는 핑키의 길을 막았다. 여기서는 관제가 둘 중 하나를 골라 비키게 한다. 비키는 핑키는 짧게
후진하고(로봇은 라이다로 뒤쪽이 비어 있을 때만 그렇게 한다, mission_behaviors.GiveWay) 다른 핑키가 지나갈
때까지 가만히 서 있는다. 그래야 다른 핑키의 Nav2가 움직이는 장애물이 아니라 서 있는 장애물을 돌아갈 길을 짠다.

계속 가는 쪽은 둘을 가르는 첫 규칙으로 정한다:
  1. 움직이면 안 되는 핑키: 5번에 들어가거나 나오는 중, 거기 서 있는 중, 수령을 위해 서 있는 중;
  2. 통로를 쥔 핑키(좁은 곳 안에 있거나 거기서 나오는 중: 다른 쪽이 입구에서 물러난다);
  3. 5번에 다녀온 핑키(주문 물건을 실었다)가 아직 5번으로 가는 핑키보다 우선;
  4. 구역을 먼저 요청한 핑키(관제의 대기 순번);
  5. 로봇 번호가 낮은 핑키.
규칙 1이나 2에 해당하는 핑키에게는 절대 비키라고 하지 않는다. 계속 가는 쪽이 `swap_after_s` 뒤에도 움직이지
않았으면(다른 쪽이 물러나지 못했거나 그걸로 부족했으면) 역할을 한 번 바꾼다. `hold_limit_s`가 지나면 둘을
놓아주고 `cooldown_s` 동안 건드리지 않는다. 이 모듈의 판단 때문에 무언가가 영원히 서 있는 일은 없다.

순수 함수형: update()는 관제가 아는 핑키별 정보를 받아 발행할 지시를 돌려준다(ROS·입출력 없음, 단위 테스트 가능).
"""
import math

# 규칙 1의 '움직이면 안 되는' 임무 단계들(5번 도킹·진입·수동 도킹·정박·탈출·재도킹, 코스 완료).
PROTECTED_PHASES = frozenset({'docking', 'dock_entry', 'docking_manual', 'docking_anchor', 'docked_manual', 'dock_exit',
                              'redocking', 'course_complete'})


def bearing(a, b):
    """b가 a의 코끝 방향에서 얼마나 벗어나 있는지(rad, 0 = 바로 앞). a, b는 (x, y, yaw)."""
    # atan2(sin, cos)로 각도를 [-pi, pi]로 감싼 뒤 절댓값을 취한다.
    off = math.atan2(b[1]-a[1], b[0]-a[0])-a[2]
    return abs(math.atan2(math.sin(off), math.cos(off)))


def head_on(a, b, meet_m, cone_rad):
    """a와 b가 meet_m(m) 안에 있고 서로가 상대를 앞쪽 원뿔(cone_rad) 안에 두고 있으면 True."""
    return math.hypot(a[0]-b[0], a[1]-b[1]) <= meet_m and bearing(a, b) <= cone_rad and bearing(b, a) <= cone_rad


def rank(rid, robot):
    """값이 큰 쪽이 계속 간다(모듈 설명의 규칙을 순서대로 튜플로 비교).

    순번·번호는 작을수록 우선이므로 음수로 바꾼다. 순번이 없으면 -inf로 가장 뒤가 된다."""
    queue = robot.get('queue')
    return (bool(robot.get('protected')), bool(robot.get('passage')), bool(robot.get('loaded')),
            -(queue if isinstance(queue, (int, float)) else float('inf')), -int(rid) if str(rid).isdigit() else 0)


def may_give_way(robot):
    """규칙 1(protected)이나 2(passage)에 해당하지 않아야 비키라는 지시를 받을 수 있다."""
    return not robot.get('protected') and not robot.get('passage')


class YieldArbiter:
    """마주침 판정과 양보 에피소드 상태를 들고 있는 판정기.

    meet_m: 마주침으로 보는 거리(m), clear_m: 이만큼 벌어지면 끝(m), cone_deg: 앞쪽 원뿔 반각(도),
    back_m: 비키는 핑키가 후진할 거리(m), hold_limit_s: 한 에피소드 최대 길이(s), cooldown_s: 끝난 쌍을
    다시 보지 않는 시간(s), swap_after_s: 역할을 바꾸기까지 기다리는 시간(s), moved_m: '움직였다'로 볼 최소 이동(m).
    """
    def __init__(self, meet_m=.50, clear_m=.75, cone_deg=55., back_m=.25, hold_limit_s=25., cooldown_s=8.,
                 swap_after_s=8., moved_m=.05):
        self.meet_m, self.clear_m, self.cone_rad, self.back_m = meet_m, clear_m, math.radians(cone_deg), back_m
        self.hold_limit_s, self.cooldown_s, self.swap_after_s, self.moved_m = hold_limit_s, cooldown_s, swap_after_s, moved_m
        self.episodes = {}   # 비키는 핑키 -> {to(비켜 주는 상대), id, since(시작 시각), origin(그때 상대 위치), swapped}
        self.cooling = {}    # 두 핑키 쌍(frozenset) -> 이 시각까지 건드리지 않음
        # 에피소드 id를 만들 때 쓰는 일련번호.
        self.count = 0

    def _start(self, yielder, other, robots, now, swapped=False):
        """yielder가 other에게 비키는 에피소드를 시작한다. 상대의 지금 위치를 origin으로 기억한다."""
        self.count += 1
        self.episodes[yielder] = dict(to=other, id=f'{self.count}-{yielder}-{other}', since=now,
                                      origin=tuple(robots[other]['pose'][:2]), swapped=swapped)

    def update(self, robots, now):
        """robots: {id: {pose: (x, y, yaw), fresh, driving, protected, passage, loaded, queue}}. driving은 경로를
        달리는 중이라는 뜻이다(주차 중도, 작업자가 멈춘 것도 아님). 지금 비켜야 하는 핑키마다 {id: {id, to, back_m}}을
        돌려준다. 지시는 계속 돌려받는 동안만 유효하다."""
        # 위치가 새롭고(fresh) 주행 중인 핑키만 판정에 넣는다.
        usable = {rid: r for rid, r in robots.items() if r.get('fresh') and r.get('driving') and r.get('pose')}
        for yielder, episode in list(self.episodes.items()):
            other = episode['to']
            if yielder not in usable or other not in usable:
                del self.episodes[yielder]   # 둘 중 하나가 멈췄거나 경로를 끝냈거나 소식이 끊겼다: 더 붙잡을 이유가 없다
                continue
            a, b = usable[yielder]['pose'], usable[other]['pose']
            apart = math.hypot(a[0]-b[0], a[1]-b[1])
            past = apart > self.meet_m and bearing(b, a) > math.pi/2   # 상대가 비키는 핑키를 이미 등 뒤에 두었다
            # 충분히 벌어졌거나, 지나갔거나, 너무 오래 끌었으면 끝내고 그 쌍을 cooldown_s 동안 쉬게 한다.
            if apart > self.clear_m or past or now-episode['since'] > self.hold_limit_s:
                del self.episodes[yielder]
                self.cooling[frozenset((yielder, other))] = now+self.cooldown_s
                continue
            # 계속 가야 할 상대가 시작 때 자리에서 moved_m도 못 움직였으면 막힌 것이다. 한 번만 역할을 바꾼다.
            stuck = math.hypot(b[0]-episode['origin'][0], b[1]-episode['origin'][1]) < self.moved_m
            if (stuck and not episode['swapped'] and now-episode['since'] > self.swap_after_s
                    and may_give_way(usable[other]) and other not in self.episodes):
                del self.episodes[yielder]
                self._start(other, yielder, usable, now, swapped=True)
        # 새 마주침 찾기: 이미 에피소드에 든 핑키는 빼고, 남은 모든 쌍을 번호 순으로 본다.
        busy = set(self.episodes) | {episode['to'] for episode in self.episodes.values()}
        ids = sorted(usable)
        for i, first in enumerate(ids):
            for second in ids[i+1:]:
                if first in busy or second in busy:
                    continue
                pair = frozenset((first, second))
                if now < self.cooling.get(pair, 0.):
                    continue
                if not head_on(usable[first]['pose'], usable[second]['pose'], self.meet_m, self.cone_rad):
                    continue
                keeps, gives = sorted((first, second), key=lambda rid: rank(rid, usable[rid]), reverse=True)
                if not may_give_way(usable[gives]):
                    continue   # 둘 다 움직이면 안 된다: 그대로 둔다
                self._start(gives, keeps, usable, now)
                busy |= {first, second}
        return {yielder: dict(id=episode['id'], to=episode['to'], back_m=self.back_m)
                for yielder, episode in self.episodes.items()}
