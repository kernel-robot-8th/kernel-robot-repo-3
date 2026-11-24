"""No hardware: the fused low layer in Nav2's height-cell costmap, and IR kept as a reading only.

The fusion itself (scripts/fusion_grid.py) is tested in test_fusion_grid.py."""
import unittest
from unittest.mock import patch

import app
from scripts.install_nav2_runtime import add_ultrasonic_fusion
from scripts.fusion_grid import validated


def pinky_params():
    """The two costmap layers as pinky_navigation's nav2_params.yaml has them (checked 2026-11-14)."""
    scan = {'topic': '/scan', 'data_type': 'LaserScan', 'marking': True, 'clearing': True}
    return {
        'local_costmap': {'local_costmap': {'ros__parameters': {
            'plugins': ['voxel_layer', 'inflation_layer'],
            'voxel_layer': {'plugin': 'nav2_costmap_2d::VoxelLayer', 'origin_z': 0.0, 'z_resolution': 0.05,
                            'z_voxels': 16, 'observation_sources': 'scan', 'scan': dict(scan)}}}},
        'global_costmap': {'global_costmap': {'ros__parameters': {
            'plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'],
            'obstacle_layer': {'plugin': 'nav2_costmap_2d::ObstacleLayer', 'observation_sources': 'scan',
                               'scan': dict(scan)}}}},
    }


class Nav2FusionParamsTests(unittest.TestCase):
    def test_the_ultrasonic_goes_to_the_local_costmap_only_by_default(self):
        parsed = pinky_params()
        add_ultrasonic_fusion(parsed)
        local = parsed['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']
        self.assertEqual(local['observation_sources'].split(), ['scan', 'fusion_low_mark', 'fusion_low_clear'])
        self.assertFalse(local['publish_voxel_map'])
        self.assertEqual(parsed['global_costmap'], pinky_params()['global_costmap'])  # a wrong mark is not kept there

    def test_with_global_both_costmaps_get_height_cells_with_the_scan_and_both_ultrasonic_clouds(self):
        parsed = pinky_params()
        add_ultrasonic_fusion(parsed, include_global=True)
        for name, layer in (('local_costmap', 'voxel_layer'), ('global_costmap', 'obstacle_layer')):
            section = parsed[name][name]['ros__parameters'][layer]
            self.assertEqual(section['plugin'], 'nav2_costmap_2d::VoxelLayer')
            self.assertEqual(section['observation_sources'].split(), ['scan', 'fusion_low_mark', 'fusion_low_clear'])
            self.assertEqual((section['fusion_low_mark']['topic'], section['fusion_low_clear']['topic']),
                             ('/fusion/low_marks', '/fusion/low_clear'))  # scripts/fusion_grid.py publishes these
            self.assertTrue(section['fusion_low_mark']['marking'])
            self.assertFalse(section['fusion_low_mark']['clearing'])
            self.assertTrue(section['fusion_low_clear']['clearing'])
            self.assertFalse(section['fusion_low_clear']['marking'])
            # Both stay in the lowest 5 cm voxel row: the LiDAR plane (about 12.5 cm) is never cleared by them
            self.assertLessEqual(section['fusion_low_clear']['max_obstacle_height'], 0.10)
            self.assertEqual(section['scan']['topic'], '/scan')  # the LiDAR source stays as it was
            self.assertFalse(section['publish_voxel_map'])
        again = pinky_params()
        add_ultrasonic_fusion(again)
        add_ultrasonic_fusion(again)  # applying twice adds nothing twice
        self.assertEqual(again['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']['observation_sources'].split().count('fusion_low_mark'), 1)


class IrReadingOnlyTests(unittest.TestCase):
    def test_ir_does_not_steer_while_navigation_says_so_even_with_a_calibrated_profile(self):
        profiles = type('P', (), {'get': lambda self, rid: {'enabled': True}})()
        with patch.multiple(app, IR_PROFILES=profiles, NAVIGATION={'ir_steering': False}):
            self.assertFalse(app._ir_steers('10'))
        with patch.multiple(app, IR_PROFILES=profiles, NAVIGATION={'ir_steering': True}):
            self.assertTrue(app._ir_steers('10'))

    def test_the_shipped_settings_are_nav2_with_fusion_in_both_costmaps_and_no_detour(self):
        # 2026-11-17: the fused low layer goes to the local and the global costmap, so Nav2 plans round a low obstacle
        # instead of stopping in front of it; the IR does not steer and the old ultrasonic detour stays off.
        self.assertEqual((app.NAVIGATION['ir_steering'], app.NAVIGATION['ultrasonic_fusion'],
                          app.NAVIGATION['ultrasonic_fusion_global'], app.NAVIGATION['ultrasonic_detour']),
                         (False, True, True, False))

    def test_the_shipped_fusion_grid_settings_are_valid(self):
        validated(app.NAVIGATION['fusion_grid'])  # the robot node would refuse to start otherwise


if __name__ == '__main__':
    unittest.main()


class LocalizedBeforeReadyTests(unittest.TestCase):
    def test_a_pose_from_before_localization_does_not_make_the_pinky_ready(self):
        from scripts.fleet_ops import operational_status, REQUIRED
        topics = {key: {'fresh': True, 'value': {}} for key in REQUIRED}
        topics['line_control']['value'] = {'dock_protocol': 'sponge_v1', 'line_algorithm': 'fleet_control_v4'}
        topics['safety_status']['value'] = {'version': 'final_gate_v1'}
        robot = {'stage': 'prepared', 'ssh_reachable': True, 'nav_started_at': 100.0}
        self.assertEqual(operational_status(robot, topics)['state'], 'unlocalized')  # AMCL's default origin pose
        robot['localized_at'] = 90.0  # set before this Nav2 start: stale
        self.assertEqual(operational_status(robot, topics)['state'], 'unlocalized')
        robot['localized_at'] = 101.0
        self.assertEqual(operational_status(robot, topics)['state'], 'ready')


class StatusDisplayTests(unittest.TestCase):
    def test_the_screen_follows_the_lamp_colour_and_says_no_link_when_the_dashboard_is_quiet(self):
        from scripts.status_display import screen_state, render
        self.assertEqual(screen_state({'color': 'blue', 'mode': 'on'}, 10.0, 11.0), ('blue', 'text', None, 'READY', '', None))
        self.assertEqual(screen_state({'color': 'green', 'mode': 'on'}, 10.0, 11.0)[3], 'DRIVING')
        self.assertEqual(screen_state({'color': 'green', 'mode': 'on'}, 10.0, 20.0), ('orange', 'text', None, 'NO LINK', '', None))
        self.assertEqual(screen_state(None, 0.0, 1.0), ('orange', 'text', None, 'NO LINK', '', None))
        self.assertEqual(screen_state(None, 0.0, 1.0, korean=True)[3], '관제 연결 끊김')
        image = render((320, 240), 'P2', '12', 'blue', headline='READY')
        self.assertEqual(image.size, (320, 240))
        self.assertEqual(image.getpixel((5, 5)), (0, 90, 255))

    def test_the_screen_says_what_the_pinky_is_doing_in_korean_only_with_a_korean_font(self):
        from scripts.status_display import screen_state, render
        wanted = {'color': 'green', 'mode': 'on', 'text': '적재 장소로 이동 중', 'word': 'TO LOADING', 'screen': 'face',
                  'face': 'happy', 'detail': '배달: 수령 장소 1', 'detail_en': 'FOR PICKUP 1', 'battery': 87}
        self.assertEqual(screen_state(wanted, 10.0, 11.0, korean=True),
                         ('green', 'face', 'happy', '적재 장소로 이동 중', '배달: 수령 장소 1', 87))
        self.assertEqual(screen_state(wanted, 10.0, 11.0), ('green', 'face', 'happy', 'TO LOADING', 'FOR PICKUP 1', 87))
        self.assertEqual(screen_state(dict(wanted, screen='?'), 10.0, 11.0)[1:3], ('text', None))  # an unknown screen
        # Every screen draws, and the longest headlines are shrunk to the width: the outermost columns stay empty.
        from scripts.fleet_ops import PHASE_ACTIVITY
        from scripts.status_display import korean_font
        for screen, face in (('text', None), ('face', 'happy'), ('face', 'excited'), ('face', 'focus'), ('face', 'surprised'),
                             ('battery', None), ('stop', None)):
            for text, word in PHASE_ACTIVITY.values():
                for headline, font in ((word, None), (text, korean_font())):
                    if headline.isascii() or font:
                        image = render((320, 240), 'P0', '10', 'green', screen, face, headline, 'FOR A', 7, korean_path=font)
                        self.assertTrue(all(image.getpixel((x, y)) == (0, 170, 60) for x in (0, 1, 318, 319)
                                            for y in range(240)), (screen, headline))
                        self.assertIn((255, 255, 255), set(image.getdata()), screen)

    def test_what_a_pinky_is_doing_follows_the_mission_phase(self):
        from scripts.fleet_ops import activity
        ready, idle = {'state': 'ready'}, {'stage': 'prepared'}
        self.assertEqual(activity(idle, None, ready, 86.6), dict(text='대기 중', word='READY', detail='', detail_en='',
                                                                battery=87, screen='text', face=None))
        self.assertEqual(activity(idle, None, {'state': 'unlocalized'})['word'], 'NO POSE')
        driving = {'status': 'running', 'current_label': '5', 'dropoff': '수령 1'}
        self.assertEqual(activity(idle, driving, ready)['text'], '적재 장소로 이동 중')
        self.assertEqual(activity(idle, driving, ready)['detail'], '배달: 수령 장소 1')
        self.assertEqual(activity(idle, dict(driving, phase='docked_manual'), ready)['text'], '물건 싣는 중')
        arrived = activity(idle, dict(driving, phase='course_complete'), ready)
        self.assertEqual((arrived['text'], arrived['detail'], arrived['screen']), ('수령 장소 1 도착', '', 'text'))
        self.assertEqual(activity(idle, dict(driving, current_label='수령 2'), ready)['text'], '수령 장소 2로 이동 중')
        self.assertEqual(activity(idle, dict(driving, current_label='수령 3'), ready)['text'], '수령 장소 3으로 이동 중')
        self.assertEqual(activity(idle, dict(driving, current_label='통로 위'), ready)['text'], '통로로 이동 중')
        self.assertEqual(activity(idle, dict(driving, current_label='4'), ready)['text'], '적재 대기 자리로 이동 중')
        self.assertEqual(activity(idle, {'status': 'running', 'return_home': True, 'current_label': '7'}, ready)['word'],
                         'GOING HOME')
        self.assertEqual(activity(idle, {'status': 'running', 'current_label': '7'}, ready)['text'], '7번 지점으로 이동 중')
        self.assertIsNone(activity(idle, None, ready, None)['battery'])

    def test_a_face_while_driving_the_battery_when_low_and_stopped_after_a_stop(self):
        from scripts.fleet_ops import activity, LOW_BATTERY_PERCENT
        ready, idle = {'state': 'ready'}, {'stage': 'prepared'}
        driving = {'status': 'running', 'current_label': '5'}
        view = lambda route, battery=80: (lambda a: (a['screen'], a['face'], a['word']))(activity(idle, route, ready, battery))
        self.assertEqual(view(driving), ('face', 'happy', 'TO LOADING'))
        self.assertEqual(view(dict(driving, phase='dock_entry')), ('face', 'focus', 'DOCKING'))
        self.assertEqual(view(dict(driving, phase='recovering')), ('face', 'surprised', 'AVOIDING'))
        self.assertEqual(view(dict(driving, phase='returning')), ('face', 'excited', 'GOING HOME'))   # happiest going home
        self.assertEqual(view({'status': 'running', 'current_label': '주차장'}), ('face', 'excited', 'GOING HOME'))
        self.assertEqual(view({'status': 'running', 'return_home': True, 'current_label': '7'}), ('face', 'excited', 'GOING HOME'))
        self.assertEqual(view(dict(driving, phase='docked_manual')), ('text', None, 'LOADING'))  # standing: no face
        self.assertEqual(view(None), ('text', None, 'READY'))
        self.assertEqual(view(dict(driving, stop_pending=True)), ('stop', None, 'STOPPED'))
        self.assertEqual(view({'status': 'cancelled'}), ('stop', None, 'STOPPED'))  # stopped, waiting for a command
        self.assertEqual(view({'status': 'complete'}), ('text', None, 'READY'))
        # Low battery wins over everything; what it was doing moves to the second line.
        low = activity(idle, driving, ready, LOW_BATTERY_PERCENT)
        self.assertEqual((low['screen'], low['face'], low['text'], low['detail']), ('battery', None, '배터리 부족', '적재 장소로 이동 중'))
        self.assertEqual(view(driving, LOW_BATTERY_PERCENT + 1)[0], 'face')
        self.assertEqual(view(None, None)[0], 'text')  # battery unknown: no warning

    def test_the_battery_shown_does_not_jump_with_every_reading_and_the_warning_does_not_flicker(self):
        from scripts.fleet_ops import BatteryView, activity
        view = BatteryView()
        self.assertEqual(view.update(None), (None, False))          # nothing heard yet
        self.assertEqual(view.update(65.4), (65, False))
        for reading in (64.6, 65.9, 66.2, 64.1, 65.0):               # noise of a point: the screen keeps 65
            self.assertEqual(view.update(reading), (65, False))
        self.assertEqual(view.update(None), (65, False))            # a missed reading keeps what was shown
        for _ in range(25):
            shown, low = view.update(19.0)
        self.assertEqual((shown, low), (19, True))
        for _ in range(25):
            shown, low = view.update(23.0)                           # above the threshold, below the release: still low
        self.assertEqual((shown, low), (23, True))
        for _ in range(25):
            shown, low = view.update(26.0)
        self.assertEqual((shown, low), (26, False))
        ready, idle = {'state': 'ready'}, {'stage': 'prepared'}
        self.assertEqual(activity(idle, None, ready, 23, True)['screen'], 'battery')   # the view's word is taken
        self.assertEqual(activity(idle, None, ready, 19, False)['screen'], 'text')


class FakePanel:
    """pinky_lcd's primitives over an array, with writes that time out on demand."""

    def __init__(self, fail=lambda count, size: False):
        import numpy as np
        self.w, self.h, self.spi = 240, 320, object()
        self.ram = np.zeros((self.h, self.w, 2), dtype=np.uint8)
        self.fail, self.writes, self.sizes, self.window, self.at = fail, 0, [], None, 0

    def _write_cmd(self, value):
        pass

    def _write_data(self, value):
        pass

    def _set_windows(self, x0, y0, x1, y1):
        self.window, self.at = (x0, y0, x1, y1), 0

    def _write_data_buffer(self, data):
        import numpy as np
        self.writes += 1
        self.sizes.append(len(data))
        x0, y0, x1, y1 = self.window
        flat = self.ram[y0:y1, x0:x1].reshape(-1)
        if self.fail(self.writes, len(data)):
            flat[self.at:self.at + len(data) // 2] = 0xAA   # half of it went out before the write timed out
            self.ram[y0:y1, x0:x1] = flat.reshape(y1 - y0, x1 - x0, 2)
            raise OSError(110, 'Connection timed out')
        flat[self.at:self.at + len(data)] = np.frombuffer(bytes(data), dtype=np.uint8)
        self.ram[y0:y1, x0:x1] = flat.reshape(y1 - y0, x1 - x0, 2)
        self.at += len(data)


class ScreenBandTests(unittest.TestCase):
    def picture(self, headline='READY', battery=80, color='blue'):
        from scripts.status_display import render
        return render((320, 240), 'P0', '10', color, 'text', None, headline, '', battery)

    def test_the_whole_picture_arrives_as_img_show_would_send_it(self):
        import numpy as np
        from PIL import Image
        from scripts.status_display import Screen, panel_bytes
        panel = FakePanel()
        screen = Screen(lcd=panel, size=(320, 240))
        picture = self.picture()
        self.assertTrue(screen.draw(picture))
        # pinky_lcd.img_show's own arithmetic
        turned = np.asarray(picture.transpose(Image.FLIP_LEFT_RIGHT).transpose(Image.ROTATE_270).convert('RGB'))
        expected = np.zeros((320, 240, 2), dtype=np.uint8)
        expected[..., [0]] = np.add(np.bitwise_and(turned[..., [0]], 0xF8), np.right_shift(turned[..., [1]], 5))
        expected[..., [1]] = np.add(np.bitwise_and(np.left_shift(turned[..., [1]], 3), 0xE0), np.right_shift(turned[..., [2]], 3))
        self.assertTrue((panel_bytes(picture, 240, 320) == expected).all())
        self.assertTrue((panel.ram == expected).all())
        self.assertEqual((panel.writes, max(panel.sizes), screen.failures), (40, 3840, 0))

    def test_only_what_changed_is_sent_again(self):
        from scripts.status_display import Screen, panel_bytes
        panel = FakePanel()
        screen = Screen(lcd=panel, size=(320, 240))
        screen.draw(self.picture(battery=80))
        panel.writes = 0
        self.assertTrue(screen.draw(self.picture(battery=80)))      # the same picture: nothing goes out
        self.assertEqual(panel.writes, 0)
        changed = self.picture(battery=78)
        self.assertTrue(screen.draw(changed))
        self.assertLess(panel.writes, 12)                           # the battery corner, not the 40 bands of a frame
        self.assertTrue((panel.ram == panel_bytes(changed, 240, 320)).all())

    def test_a_band_whose_write_times_out_is_sent_again(self):
        from scripts.status_display import Screen, panel_bytes
        panel = FakePanel(fail=lambda count, size: count % 3 == 0)  # every third write times out
        screen = Screen(lcd=panel, size=(320, 240))
        picture = self.picture('적재 자리로 이동 중' if False else 'TO DOCK', 64, 'green')
        self.assertTrue(screen.draw(picture, budget_s=30.0))
        self.assertTrue((panel.ram == panel_bytes(picture, 240, 320)).all())
        self.assertGreater(screen.failures, 10)
        self.assertFalse(screen.small)                              # failures were never six in a row

    def test_when_big_writes_keep_failing_short_ones_are_tried_and_what_is_left_waits_for_the_next_tick(self):
        from scripts.status_display import Screen, panel_bytes, SMALL_CHUNK
        panel = FakePanel(fail=lambda count, size: size > SMALL_CHUNK)   # only short writes get through
        screen = Screen(lcd=panel, size=(320, 240))
        picture = self.picture()
        done = screen.draw(picture, budget_s=30.0)
        self.assertTrue(screen.small)
        for _ in range(3):                                           # the bands given up on go out on later ticks
            done = done or screen.draw(picture, budget_s=30.0)
        self.assertTrue(done)
        self.assertTrue((panel.ram == panel_bytes(picture, 240, 320)).all())
        dead = Screen(lcd=FakePanel(fail=lambda count, size: True), size=(320, 240))
        self.assertFalse(dead.draw(picture, budget_s=0.2))           # nothing gets through: no exception, not done
        self.assertGreater(dead.failures, 0)
        self.assertIn('timed out', dead.last_error)

    def test_the_screen_opens_without_the_big_clear_and_tries_again_when_a_write_times_out(self):
        from scripts.status_display import Screen

        class StockLCD(FakePanel):
            opened = 0

            def __init__(self):
                super().__init__()
                StockLCD.opened += 1
                if StockLCD.opened == 1:
                    raise OSError(110, 'Connection timed out')  # the first try: a setting byte timed out
                self.clear()

            def clear(self, color=0x0000):
                raise OSError(110, 'Connection timed out')      # the stock clear: big writes that time out under load

        lcd = Screen._open(StockLCD)
        self.assertEqual(StockLCD.opened, 2)
        self.assertIsInstance(lcd, StockLCD)
        screen = Screen(lcd=lcd, size=(320, 240))
        self.assertTrue(screen.banded)
        self.assertTrue(screen.draw(self.picture()))

        class Dead(FakePanel):
            def __init__(self):
                raise OSError(110, 'Connection timed out')

            def clear(self, color=0x0000):
                pass

        with self.assertRaises(OSError):
            Screen._open(Dead)

    def test_standing_for_another_pinky_says_so_with_where_it_was_going(self):
        from scripts.fleet_ops import activity
        ready, idle = {'state': 'ready'}, {'stage': 'prepared'}
        driving = {'status': 'running', 'current_label': '5 정면', 'dropoff': '수령 1'}
        wait = activity(idle, driving, ready, 70, waiting=True)
        self.assertEqual((wait['text'], wait['detail'], wait['screen'], wait['face']), ('교차로 대기', '적재 장소 가는 길', 'text', None))
        self.assertEqual((wait['word'], wait['detail_en']), ('WAITING', 'TO LOADING'))
        give = activity(idle, dict(driving, yield_to='11'), ready, 70)
        self.assertEqual((give['text'], give['word'], give['screen']), ('양보 중', 'GIVING WAY', 'text'))
        # At the station it stands for its own reasons: that is what the screen says.
        self.assertEqual(activity(idle, dict(driving, phase='docked_manual'), ready, 70, waiting=True)['text'], '물건 싣는 중')
        self.assertEqual(activity(idle, None, ready, 70, waiting=True)['text'], '대기 중')
