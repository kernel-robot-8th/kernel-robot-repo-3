"""Who gives way when two Pinkys meet head-on (scripts/yielding.py) and how the Pinky does it (GiveWay)."""
import math
import unittest

from scripts.mission_behaviors import GiveWay
from scripts.yielding import YieldArbiter, head_on, rank


def robot(x, y, yaw_deg, **fields):
    return dict(pose=(x, y, math.radians(yaw_deg)), fresh=True, driving=True, **fields)


class HeadOnTests(unittest.TestCase):
    def test_only_two_pinkys_close_and_facing_each_other_count(self):
        a, b = (1.0, 1.6, math.pi), (0.6, 1.6, 0.)                       # nose to nose, 40 cm apart
        self.assertTrue(head_on(a, b, .5, math.radians(55)))
        self.assertFalse(head_on(a, (0.4, 1.6, 0.), .5, math.radians(55)))    # 60 cm: not yet
        self.assertFalse(head_on(a, (0.6, 1.6, math.pi), .5, math.radians(55)))  # one behind the other: following
        self.assertFalse(head_on((1.0, 1.6, math.pi/2), b, .5, math.radians(55)))  # one looks away

    def test_who_keeps_going(self):
        order = lambda **f: rank('11', dict(f))
        self.assertGreater(order(protected=True), order(passage=True, loaded=True, queue=1))
        self.assertGreater(order(passage=True), order(loaded=True, queue=1))
        self.assertGreater(order(loaded=True, queue=9), order(queue=1))
        self.assertGreater(order(queue=1), order(queue=2))
        self.assertGreater(rank('10', {}), rank('11', {}))


class ArbiterTests(unittest.TestCase):
    def test_the_one_on_its_way_in_gives_way_to_the_one_carrying_the_order(self):
        arbiter = YieldArbiter()
        robots = {'10': robot(0.6, 1.6, 0, loaded=True, queue=2), '11': robot(1.0, 1.6, 180, queue=1)}
        orders = arbiter.update(robots, 0.)
        self.assertEqual(list(orders), ['11'])
        self.assertEqual((orders['11']['to'], orders['11']['back_m']), ('10', .25))
        self.assertEqual(arbiter.update(robots, 1.)['11']['id'], orders['11']['id'])   # the same order, held

    def test_the_one_waiting_at_the_mouth_backs_away_from_the_one_holding_the_passage(self):
        arbiter = YieldArbiter()
        robots = {'10': robot(1.57, 1.20, 90, passage=True, queue=5),            # coming out of the top
                  '12': robot(1.57, 1.55, -90, loaded=True, queue=1)}            # waiting to go down, carrying the order
        self.assertEqual(list(arbiter.update(robots, 0.)), ['12'])

    def test_a_pinky_at_the_station_or_in_the_passage_is_never_moved(self):
        arbiter = YieldArbiter()
        robots = {'10': robot(0.6, 1.6, 0, protected=True), '11': robot(1.0, 1.6, 180, passage=True)}
        self.assertEqual(arbiter.update(robots, 0.), {})
        parked = {'10': robot(0.6, 1.6, 0), '11': dict(robot(1.0, 1.6, 180), driving=False)}
        self.assertEqual(arbiter.update(parked, 0.), {})                 # a parked or stopped Pinky is nobody's order
        stale = {'10': robot(0.6, 1.6, 0), '11': dict(robot(1.0, 1.6, 180), fresh=False)}
        self.assertEqual(arbiter.update(stale, 0.), {})

    def test_the_order_ends_when_the_other_is_past_and_the_pair_is_left_alone_for_a_while(self):
        arbiter = YieldArbiter()
        robots = {'10': robot(0.6, 1.6, 0, loaded=True), '11': robot(1.0, 1.6, 180)}
        self.assertIn('11', arbiter.update(robots, 0.))
        robots['11'] = robot(1.25, 1.6, 180)                              # it backed off: still close, still facing
        robots['10'] = robot(0.9, 1.75, 30, loaded=True)                  # the other is going round
        self.assertIn('11', arbiter.update(robots, 3.))
        robots['10'] = robot(1.80, 1.75, 0, loaded=True)                  # past it, with the yielder behind
        self.assertEqual(arbiter.update(robots, 6.), {})
        robots = {'10': robot(0.6, 1.6, 0, loaded=True), '11': robot(1.0, 1.6, 180)}
        self.assertEqual(arbiter.update(robots, 9.), {})                  # cooling: not ordered again at once
        self.assertIn('11', arbiter.update(robots, 6.+8.1))

    def test_when_the_one_that_keeps_going_cannot_move_the_roles_are_swapped_once_then_both_are_let_go(self):
        arbiter = YieldArbiter()
        robots = {'10': robot(0.6, 1.6, 0, loaded=True), '11': robot(1.0, 1.6, 180)}
        self.assertEqual(list(arbiter.update(robots, 0.)), ['11'])
        self.assertEqual(list(arbiter.update(robots, 7.)), ['11'])
        swapped = arbiter.update(robots, 8.1)                             # 10 has not moved 5 cm in 8 s
        self.assertEqual(list(swapped), ['10'])
        self.assertEqual(list(arbiter.update(robots, 20.)), ['10'])       # not swapped back
        self.assertEqual(arbiter.update(robots, 8.1+25.1), {})            # the limit: let go
        moving = {'10': robot(0.6, 1.6, 0, loaded=True), '11': robot(1.0, 1.6, 180)}
        second = YieldArbiter()
        second.update(moving, 0.)
        moving['10'] = robot(0.68, 1.62, 10, loaded=True)                 # it is getting on: no swap
        self.assertEqual(list(second.update(moving, 9.)), ['11'])

    def test_three_pinkys_one_pair_at_a_time_per_pinky(self):
        arbiter = YieldArbiter()
        robots = {'10': robot(0.6, 1.6, 0, loaded=True), '11': robot(1.0, 1.6, 180), '12': robot(1.0, 1.25, 150)}
        orders = arbiter.update(robots, 0.)
        self.assertEqual(list(orders), ['11'])                           # 12 also faces 10, but 10 is already in a pair


class GiveWayTests(unittest.TestCase):
    def test_it_reverses_the_ordered_distance_then_stands_until_the_order_is_gone(self):
        give, order = GiveWay(), {'id': '1-11-10', 'to': '10', 'back_m': .25}
        x, now, states = 0., 0., []
        for _ in range(200):
            motion = give.update(order, (x, 0., 0.), now, rear_clear=True)
            states.append(motion.state)
            x += motion.linear*.05
            now += .05
        self.assertEqual(states[0], 'yield_back')
        self.assertEqual(states[-1], 'yield_wait')
        self.assertAlmostEqual(-x, .25, delta=.01)
        self.assertEqual(give.update(order, (x, 0., 0.), now, True).linear, 0.)   # the same order: it keeps standing
        self.assertIsNone(give.update(None, (x, 0., 0.), now, True))               # order gone: drive as before
        again = give.update({'id': '2-11-12', 'back_m': .25}, (x, 0., 0.), now, True)
        self.assertEqual(again.state, 'yield_back')                                # a new order backs off again

    def test_it_stands_instead_when_the_way_back_is_not_clear_and_never_reverses_past_the_limits(self):
        give = GiveWay()
        blocked = give.update({'id': 'a', 'back_m': .25}, (0., 0., 0.), 0., rear_clear=False)
        self.assertEqual((blocked.linear, blocked.state), (0., 'yield_wait'))
        self.assertEqual(give.update({'id': 'a', 'back_m': .25}, (0., 0., 0.), 1., rear_clear=True).state, 'yield_wait')
        far = GiveWay()
        self.assertEqual(far.update({'id': 'b', 'back_m': 9.}, (0., 0., 0.), 0., True).state, 'yield_back')
        self.assertEqual(far.update({'id': 'b', 'back_m': 9.}, (-.41, 0., 0.), 1., True).state, 'yield_wait')   # 40 cm at most
        slow = GiveWay()
        slow.update({'id': 'c', 'back_m': .25}, (0., 0., 0.), 0., True)
        self.assertEqual(slow.update({'id': 'c', 'back_m': .25}, (-.01, 0., 0.), 10.1, True).state, 'yield_wait')  # 10 s at most
        lost = GiveWay()
        self.assertEqual(lost.update({'id': 'd', 'back_m': .25}, None, 0., True).state, 'yield_wait')            # no odometry
        self.assertIsNone(GiveWay().update({'back_m': .25}, (0., 0., 0.), 0., True))                              # not an order


if __name__ == '__main__':
    unittest.main()
