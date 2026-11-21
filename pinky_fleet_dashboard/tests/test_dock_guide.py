"""No hardware: the drive into station 5 steering on the ceiling camera's live offset (DockEntry with a guide)."""
import math
import random
import tempfile
import unittest
from pathlib import Path

from scripts.mission_behaviors import DockEntry
from scripts.route_geometry import clean_dock_guide


def drive_in(left0, yaw0=0., depth_error=0., delay_s=.2, guided=True, drop_after=None, target=.25):
    """A unicycle driven by DockEntry. Spot frame: the spot at the origin facing +x; the Pinky starts `target +
    depth_error` behind it and `left0` to its left. The camera's view arrives `delay_s` late with a little noise.
    Returns the behaviour's state and where the Pinky stands against the spot (mm, degrees)."""
    rng = random.Random(7)
    entry, persist = DockEntry(), Path(tempfile.mkdtemp())/'latch.json'
    x, y, yaw = -(target+depth_error), left0, yaw0
    ox, oy, oyaw = 5., 3., 1.+yaw0  # the odometry frame is somewhere else entirely
    now, step, history = 0., .05, []
    context = dict(token='t', segment='s', dock_entry_m=target)
    for _ in range(2000):
        history.append((now, x, y, yaw))
        guide = None
        if guided and (drop_after is None or now < drop_after):
            seen = next((h for h in reversed(history) if h[0] <= now-delay_s), None)
            if seen:
                guide = (seen[1]+rng.gauss(0, .001), seen[2]+rng.gauss(0, .001), seen[3]+rng.gauss(0, .004))
        motion = entry.update(context, (ox, oy, oyaw), now, persist, guide)
        if motion.state != 'docking_entry_entering':
            return entry, (x*1000, y*1000, math.degrees(yaw))
        x += motion.linear*math.cos(yaw)*step; y += motion.linear*math.sin(yaw)*step; yaw += motion.angular*step
        ox += motion.linear*math.cos(oyaw)*step; oy += motion.linear*math.sin(oyaw)*step; oyaw += motion.angular*step
        now += step
    raise AssertionError('the entry never ended')


class GuidedDockEntryTests(unittest.TestCase):
    def test_without_a_guide_it_drives_in_as_before_and_keeps_its_sideways_miss(self):
        entry, (forward, left, yaw) = drive_in(.03, guided=False)
        self.assertEqual((entry.state, entry.guided), ('complete', False))
        self.assertAlmostEqual(left, 30., delta=.5)
        self.assertAlmostEqual(forward, 0., delta=4.)

    def test_with_the_cameras_guide_it_comes_in_on_the_spot_from_either_side(self):
        for left0, yaw0 in ((.03, 0.), (-.03, .05), (.02, -.05), (.045, 0.)):
            with self.subTest(left0=left0, yaw0=yaw0):
                entry, (forward, left, yaw) = drive_in(left0, yaw0)
                self.assertEqual((entry.state, entry.guided), ('complete', True))
                self.assertLess(abs(left), 3.)
                self.assertLess(abs(yaw), 3.)
                self.assertLess(abs(forward), 8.)

    def test_the_camera_decides_where_it_stops_a_few_centimetres_either_way(self):
        for depth_error in (.03, -.03):
            with self.subTest(depth_error=depth_error):
                entry, (forward, left, yaw) = drive_in(0., depth_error=depth_error)
                self.assertEqual(entry.state, 'complete')
                self.assertLess(abs(forward), 8.)  # on odometry alone it would stop 30 mm short or past

    def test_a_guide_that_stops_arriving_leaves_it_on_the_heading_it_squared_up_to(self):
        entry, (forward, left, yaw) = drive_in(.03, drop_after=3.)
        self.assertEqual(entry.state, 'complete')
        self.assertLess(abs(yaw), 1.)
        self.assertLess(abs(left), 30.)  # what was corrected until then is kept
        self.assertAlmostEqual(forward, 0., delta=6.)

    def test_a_slow_link_only_costs_a_little_depth(self):
        entry, (forward, left, yaw) = drive_in(.03, delay_s=.5)
        self.assertEqual(entry.state, 'complete')
        self.assertLess(abs(left), 3.)
        self.assertLess(abs(forward), 20.)

    def test_the_guide_cannot_take_it_far_past_the_counted_distance_or_stop_it_far_short(self):
        entry, persist = DockEntry(), Path(tempfile.mkdtemp())/'latch.json'
        context = dict(token='t', segment='s', dock_entry_m=.25)
        entry.update(context, (0., 0., 0.), 0., persist, (-.20, 0., 0.))
        # "arrived" from the camera after only 10 cm of the 25: not believed yet
        self.assertEqual(entry.update(context, (.10, 0., 0.), 3., persist, (0., 0., 0.)).state, 'docking_entry_entering')
        self.assertEqual(entry.update(context, (.18, 0., 0.), 6., persist, (0., 0., 0.)).state, 'docking_entry_complete')
        entry = DockEntry()
        context = dict(token='t', segment='s2', dock_entry_m=.25)
        entry.update(context, (0., 0., 0.), 0., persist, (-.30, 0., 0.))
        # the camera still says 5 cm short, but 31 cm were driven: it stops
        self.assertEqual(entry.update(context, (.31, 0., 0.), 10., persist, (-.05, 0., 0.)).state, 'docking_entry_complete')

    def test_the_relayed_numbers_are_checked(self):
        self.assertEqual(clean_dock_guide(dict(forward_mm=-250., left_mm=31.26, dyaw_deg=2.5)),
                         dict(forward_mm=-250., left_mm=31.3, dyaw_deg=2.5))
        for bad in (None, dict(forward_mm=-600., left_mm=0., dyaw_deg=0.), dict(forward_mm=0., left_mm=300., dyaw_deg=0.),
                    dict(forward_mm=0., left_mm=0., dyaw_deg=200.), dict(forward_mm='a', left_mm=0., dyaw_deg=0.),
                    dict(forward_mm=float('nan'), left_mm=0., dyaw_deg=0.), dict(forward_mm=True, left_mm=0., dyaw_deg=0.)):
            self.assertIsNone(clean_dock_guide(bad))


if __name__ == '__main__':
    unittest.main()


def capture_in(forward0, left0, yaw0, delay_s=.2, drop=None):
    """As drive_in, but handed over to the camera wherever it stands (context dock_capture): `forward0` metres short
    of the spot (negative), `left0` to its left, turned `yaw0`. drop=(from_s, to_s): the camera is silent then."""
    rng = random.Random(11)
    entry, persist = DockEntry(), Path(tempfile.mkdtemp())/'latch.json'
    x, y, yaw = forward0, left0, yaw0
    ox, oy, oyaw = -2., 4., 2.5
    now, step, history, turned_first = 0., .05, [], None
    context = dict(token='t', segment='c', dock_entry_m=.25, dock_capture=True)
    for _ in range(4000):
        history.append((now, x, y, yaw))
        guide = None
        if drop is None or not drop[0] <= now < drop[1]:
            seen = next((h for h in reversed(history) if h[0] <= now-delay_s), None)
            if seen:
                guide = (seen[1]+rng.gauss(0, .001), seen[2]+rng.gauss(0, .001), seen[3]+rng.gauss(0, .004))
        motion = entry.update(context, (ox, oy, oyaw), now, persist, guide)
        if motion.state != 'docking_entry_entering':
            return entry, (x*1000, y*1000, math.degrees(yaw)), turned_first, now
        if turned_first is None and (motion.linear or motion.angular):
            turned_first = motion.linear == 0.
        x += motion.linear*math.cos(yaw)*step; y += motion.linear*math.sin(yaw)*step; yaw += motion.angular*step
        ox += motion.linear*math.cos(oyaw)*step; oy += motion.linear*math.sin(oyaw)*step; oyaw += motion.angular*step
        now += step
    raise AssertionError('the entry never ended')


class CapturedDockEntryTests(unittest.TestCase):
    def test_turned_away_it_first_turns_on_the_spot_then_comes_in_on_the_spot(self):
        # As seen on 2026-11-19: Nav2 left P2 near the square-up point facing the table, 130 degrees off.
        for forward0, left0, yaw0 in ((-.25, .0, math.radians(-132)), (-.27, .03, math.radians(120)),
                                      (-.22, -.04, math.radians(60)), (-.25, .06, math.radians(-175))):
            with self.subTest(forward0=forward0, left0=left0, yaw0=yaw0):
                entry, (forward, left, yaw), turned_first, _ = capture_in(forward0, left0, yaw0)
                self.assertEqual((entry.state, entry.capture, turned_first), ('complete', True, True))
                self.assertLess(abs(forward), 8.)
                self.assertLess(abs(left), 5.)
                self.assertLess(abs(yaw), 4.)

    def test_already_facing_the_spot_it_just_drives_in(self):
        entry, (forward, left, yaw), turned_first, _ = capture_in(-.28, .02, .05)
        self.assertEqual((entry.state, turned_first), ('complete', False))
        self.assertLess(abs(left), 3.)
        self.assertLess(abs(forward), 8.)

    def test_while_the_camera_is_silent_it_stands_and_goes_on_when_it_sees_again(self):
        entry, (forward, left, yaw), _, took = capture_in(-.25, .02, 0., drop=(2., 6.))
        self.assertEqual(entry.state, 'complete')
        self.assertLess(abs(forward), 8.)
        self.assertGreater(took, 4.+.20/.03)  # the four silent seconds were spent standing

    def test_a_camera_that_stays_silent_stops_it_as_a_fault_without_driving_blind(self):
        entry, (forward, left, yaw), _, took = capture_in(-.25, .0, 0., drop=(2., 1e9))
        self.assertEqual(entry.state, 'fault')
        self.assertLess(forward, -150.)  # it did not drive on
        self.assertLess(took, 2.+DockEntry.CAPTURE_BLIND_S+1.)

    def test_the_guide_passes_any_heading_but_not_a_marker_far_from_the_spot(self):
        self.assertIsNotNone(clean_dock_guide(dict(forward_mm=-260., left_mm=20., dyaw_deg=-132.)))
        self.assertIsNone(clean_dock_guide(dict(forward_mm=-260., left_mm=300., dyaw_deg=0.)))
        self.assertIsNone(clean_dock_guide(dict(forward_mm=-600., left_mm=0., dyaw_deg=0.)))


class SelfSquaringDockEntryTests(unittest.TestCase):
    """The Pinky turns to the dock's heading itself (context dock_turn_rad) instead of Nav2 turning it."""

    def run_entry(self, yaw0, guide_after_turn=False, stuck=False, side=0., told=True):
        entry, persist = DockEntry(), Path(tempfile.mkdtemp())/'latch.json'
        x, y, yaw = -.25, side, yaw0
        ox, oy, oyaw = 1., -2., .7+yaw0
        now, step, turned_while_standing = 0., .05, True
        context = dict(token='t', segment='q', dock_entry_m=.25, dock_turn_rad=-yaw0, dock_side_m=side if told else 0.)
        for _ in range(3000):
            guide = (x, y, yaw) if guide_after_turn and not entry.squaring else None
            motion = entry.update(context, (ox, oy, oyaw), now, persist, guide)
            if motion.state != 'docking_entry_entering':
                return entry, (x*1000, y*1000, math.degrees(yaw)), turned_while_standing
            if entry.squaring and motion.linear:
                turned_while_standing = False
            turn = 0. if stuck else motion.angular
            x += motion.linear*math.cos(yaw)*step; y += motion.linear*math.sin(yaw)*step; yaw += turn*step
            ox += motion.linear*math.cos(oyaw)*step; oy += motion.linear*math.sin(oyaw)*step; oyaw += turn*step
            now += step
        raise AssertionError('the entry never ended')

    def test_it_turns_on_the_spot_to_the_docks_heading_then_drives_the_measured_distance_straight(self):
        for yaw0 in (math.radians(-46), math.radians(134), math.radians(8), 0.):
            with self.subTest(yaw0=yaw0):
                entry, (forward, left, yaw), standing = self.run_entry(yaw0)
                self.assertEqual((entry.state, standing), ('complete', True))
                self.assertLess(abs(yaw), 3.5)
                self.assertLess(abs(forward), 8.)
                self.assertLess(abs(left), 12.)

    def test_wheels_that_do_not_turn_it_end_as_a_fault_not_a_blind_drive(self):
        entry, (forward, left, yaw), _ = self.run_entry(math.radians(60), stuck=True)
        self.assertEqual(entry.state, 'fault')
        self.assertAlmostEqual(forward, -250., delta=1.)

    def test_handed_over_beside_the_line_it_steers_onto_it_on_the_way_in(self):
        # "Close enough" at the square-up point is up to 5 cm beside the dock's line; no camera for this Pinky.
        for side, yaw0 in ((.05, 0.), (-.05, math.radians(40)), (.03, math.radians(-90))):
            with self.subTest(side=side, yaw0=yaw0):
                entry, (forward, left, yaw), _ = self.run_entry(yaw0, side=side)
                self.assertEqual(entry.state, 'complete')
                self.assertLess(abs(left), 12.)
                self.assertLess(abs(yaw), 6.)
                self.assertLess(abs(forward), 10.)
        entry, (forward, left, yaw), _ = self.run_entry(0., side=.05, told=False)
        self.assertAlmostEqual(left, 50., delta=1.)  # not told: it keeps the miss, as before


class SightingsTests(unittest.TestCase):
    def test_the_same_guide_read_again_is_one_sighting(self):
        from scripts.navigate_route import Sightings
        seen = Sightings()
        for _ in range(6):
            seen.see([-.25, .01, .0])
        self.assertEqual(seen.count, 1)
        for index in range(3):
            seen.see([-.25+index*.002, .01, .0])
        self.assertEqual(seen.count, 3)
        self.assertEqual(seen.see(None), 0)


class CameraTakeoverTests(unittest.TestCase):
    def test_squared_up_on_a_wrong_heading_the_camera_takes_over_instead_of_a_fault(self):
        # Seen 2026-11-21: after a redock through Nav2 the Pinky stood 93 degrees off; the guided straight run faulted.
        entry, persist = DockEntry(), Path(tempfile.mkdtemp())/'latch.json'
        x, y, yaw = -.22, -.016, math.radians(-93)
        ox, oy, oyaw, now, step = 0., 0., 0., 0., .05
        context = dict(token='t', segment='r', dock_entry_m=.25, dock_turn_rad=0.)
        for _ in range(4000):
            motion = entry.update(context, (ox, oy, oyaw), now, persist, (x, y, yaw))
            if motion.state != 'docking_entry_entering':
                break
            x += motion.linear*math.cos(yaw)*step; y += motion.linear*math.sin(yaw)*step; yaw += motion.angular*step
            ox += motion.linear*math.cos(oyaw)*step; oy += motion.linear*math.sin(oyaw)*step; oyaw += motion.angular*step
            now += step
        self.assertEqual((entry.state, entry.capture), ('complete', True))
        self.assertLess(abs(x*1000), 8.)
        self.assertLess(abs(y*1000), 5.)
        self.assertLess(abs(math.degrees(yaw)), 4.)
