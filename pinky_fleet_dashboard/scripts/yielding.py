"""Who gives way when two Pinkys meet head-on (2026-11-24).

The zones (scripts/traffic.py) keep one Pinky at a time in the passage and at station 5, but outside them nobody
decided anything: two Pinkys that met nose to nose each stopped for the other, backed off a little, tried again, and
at the mouth of the passage the one waiting to go in stood in the way of the one coming out. Here the dashboard
picks one of the two to give way: it reverses a short way (the robot does that only while its LiDAR shows the way
back clear, mission_behaviors.GiveWay) and then stands still until the other is past, so the other's Nav2 has a
standing obstacle to plan round instead of a moving one.

Who keeps going, first rule that tells the two apart:
  1. a Pinky that may not be moved: driving into or out of station 5, standing there, or standing for a pickup;
  2. the one that holds the passage (it is in the narrow part or coming out of it: the other backs away from the mouth);
  3. the one that has been to station 5 (it carries the order) over the one still on its way there;
  4. the one that asked for its zones first (the dashboard's queue number);
  5. the lower robot id.
A Pinky under rule 1 or 2 is never told to give way. When the one that keeps going has not moved after
`swap_after_s` (the other could not back off, or it is not enough) the roles are swapped once; after `hold_limit_s`
the pair is let go and left alone for `cooldown_s`, so nothing stands for ever on this module's word.

Pure: update() takes what the dashboard knows about each Pinky and returns the orders to publish.
"""
import math

PROTECTED_PHASES = frozenset({'docking', 'dock_entry', 'docking_manual', 'docking_anchor', 'docked_manual', 'dock_exit',
                              'redocking', 'course_complete'})


def bearing(a, b):
    """How far off a's nose b lies (rad, 0 = straight ahead); a and b are (x, y, yaw)."""
    off = math.atan2(b[1]-a[1], b[0]-a[0])-a[2]
    return abs(math.atan2(math.sin(off), math.cos(off)))


def head_on(a, b, meet_m, cone_rad):
    """a and b are within meet_m and each has the other in front of it."""
    return math.hypot(a[0]-b[0], a[1]-b[1]) <= meet_m and bearing(a, b) <= cone_rad and bearing(b, a) <= cone_rad


def rank(rid, robot):
    """Greater keeps going (the rules of the module text, in order)."""
    queue = robot.get('queue')
    return (bool(robot.get('protected')), bool(robot.get('passage')), bool(robot.get('loaded')),
            -(queue if isinstance(queue, (int, float)) else float('inf')), -int(rid) if str(rid).isdigit() else 0)


def may_give_way(robot):
    return not robot.get('protected') and not robot.get('passage')


class YieldArbiter:
    def __init__(self, meet_m=.50, clear_m=.75, cone_deg=55., back_m=.25, hold_limit_s=25., cooldown_s=8.,
                 swap_after_s=8., moved_m=.05):
        self.meet_m, self.clear_m, self.cone_rad, self.back_m = meet_m, clear_m, math.radians(cone_deg), back_m
        self.hold_limit_s, self.cooldown_s, self.swap_after_s, self.moved_m = hold_limit_s, cooldown_s, swap_after_s, moved_m
        self.episodes = {}   # the Pinky giving way -> {to, id, since, origin (the other's pose then), swapped}
        self.cooling = {}    # frozenset of the pair -> left alone until
        self.count = 0

    def _start(self, yielder, other, robots, now, swapped=False):
        self.count += 1
        self.episodes[yielder] = dict(to=other, id=f'{self.count}-{yielder}-{other}', since=now,
                                      origin=tuple(robots[other]['pose'][:2]), swapped=swapped)

    def update(self, robots, now):
        """robots: {id: {pose: (x, y, yaw), fresh, driving, protected, passage, loaded, queue}}; driving is a route
        being driven (not parked, not stopped by the operator). Returns {id: {id, to, back_m}} for the Pinkys that
        give way now; an order holds as long as it keeps being returned."""
        usable = {rid: r for rid, r in robots.items() if r.get('fresh') and r.get('driving') and r.get('pose')}
        for yielder, episode in list(self.episodes.items()):
            other = episode['to']
            if yielder not in usable or other not in usable:
                del self.episodes[yielder]   # one of them stopped, ended its route or went quiet: nothing to hold for
                continue
            a, b = usable[yielder]['pose'], usable[other]['pose']
            apart = math.hypot(a[0]-b[0], a[1]-b[1])
            past = apart > self.meet_m and bearing(b, a) > math.pi/2   # the other has the yielder behind it
            if apart > self.clear_m or past or now-episode['since'] > self.hold_limit_s:
                del self.episodes[yielder]
                self.cooling[frozenset((yielder, other))] = now+self.cooldown_s
                continue
            stuck = math.hypot(b[0]-episode['origin'][0], b[1]-episode['origin'][1]) < self.moved_m
            if (stuck and not episode['swapped'] and now-episode['since'] > self.swap_after_s
                    and may_give_way(usable[other]) and other not in self.episodes):
                del self.episodes[yielder]
                self._start(other, yielder, usable, now, swapped=True)
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
                    continue   # neither may be moved: they are left as they are
                self._start(gives, keeps, usable, now)
                busy |= {first, second}
        return {yielder: dict(id=episode['id'], to=episode['to'], back_m=self.back_m)
                for yielder, episode in self.episodes.items()}
