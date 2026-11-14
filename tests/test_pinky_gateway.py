"""Pinky screen gateway: card shape from the fleet dashboard's record, commands and their log, the HTTP layer."""

from __future__ import annotations

import unittest

from control_station.mock_world import MockWorld
from control_station.pinky_gateway import PinkyGateway, sensor_details, shape_pinky
from test_api_routes import ApiTestCase

FRESH = {"fresh": True, "value": {}}


def robot_record(**route: object) -> dict:
    """The dashboard's record for Pinky C76C as seen on 2026-11-14, with optional route fields."""
    topics = {key: FRESH for key in ("scan", "odom", "ultrasonic", "ir", "map", "tf_pose", "line_control")}
    topics["safety_status"] = {"fresh": True, "value": {"state": "route_or_traffic_wait"}}
    return {"id": "10", "name": "Pinky C76C", "ip": "192.168.0.9", "topics": topics, "route": route or None}


class FakeFleet:
    def __init__(self, answers: dict[str, tuple[int, dict]] | None = None) -> None:
        self.answers = answers or {}
        self.sent: list[tuple[str, object]] = []

    def robots(self) -> dict:
        return {"10": robot_record(status="running", labels=["1", "2", "5"], completed=1, current_label="2"),
                "11": {"id": "11", "name": "Pinky D631", "topics": {}}}

    def fleet_status(self, pinky_id: str) -> dict:
        if pinky_id == "10":
            return {"connected": True, "ready_for_motion": True, "readiness_label": "현재 주행 가능",
                    "battery_percent": 64.0, "battery_volts": 7.37, "phase": "driving", "message": "웨이포인트 2로 이동 중"}
        return {"connected": False, "ready_for_motion": False, "readiness_label": "연결 끊김", "phase": "parked"}

    def stop(self, pinky_id: str) -> tuple[int, dict]:
        self.sent.append(("stop", pinky_id))
        return self.answers.get(pinky_id, (202, {"ok": True, "message": "경로 취소를 요청했습니다."}))

    def send_password(self, password: str) -> tuple[int, dict]:
        self.sent.append(("password", password))
        return 200, {"ok": True, "message": "비밀번호를 실행 중인 프로세스 메모리에만 보관합니다."}


class ShapeTest(unittest.TestCase):
    def test_card_from_a_live_record(self) -> None:
        fleet = FakeFleet()
        card = shape_pinky("10", fleet.robots()["10"], fleet.fleet_status("10"))
        self.assertEqual((card["name"], card["robot_name"], card["ip"]), ("P0", "Pinky C76C", "192.168.0.9"))
        self.assertEqual((card["connected"], card["ready"], card["missing"]), (True, True, []))
        self.assertEqual((card["battery_percent"], card["battery_volts"]), (64.0, 7.37))
        self.assertEqual(card["route"], {"status": "running", "current_label": "2", "completed": 1.0, "total": 3})
        self.assertEqual(card["safety_label"], "경로·허가 없음, 정지")

    def test_a_disconnected_pinky_lists_every_missing_feed(self) -> None:
        fleet = FakeFleet()
        card = shape_pinky("11", fleet.robots()["11"], fleet.fleet_status("11"))
        self.assertEqual(card["phase_label"], "연결 끊김")
        self.assertEqual(len(card["missing"]), 8)
        self.assertIn("안전 감시기", card["missing"])


class SensorDetailTest(unittest.TestCase):
    def test_readings_in_words_from_the_dashboards_topic_values(self) -> None:
        topics = {
            "scan": {"fresh": True, "age_s": 0.1, "value": {"samples": 720, "front_min_m": 0.481}},
            "ultrasonic": {"fresh": True, "value": {"range_m": 0.9697, "min_m": 0.02, "max_m": 3.0}},
            "ir": {"fresh": True, "value": {"channels": [2784, 2355, 3351]}},
            "odom": {"fresh": True, "value": {"linear_mps": 0.0, "angular_rps": 0.0}},
            "tf_pose": {"fresh": True, "value": {"x_m": 0.4206, "y_m": 0.3977, "yaw_rad": 3.1268, "frame_id": "map"}},
        }
        texts = {d["key"]: d["text"] for d in sensor_details(topics)}
        self.assertEqual(texts["scan"], "앞 가장 가까운 물체 0.48 m · 720점")
        self.assertEqual(texts["ultrasonic"], "0.97 m (측정 0.02~3.0 m)")
        self.assertEqual(texts["ir"], "2784 · 2355 · 3351")
        self.assertEqual(texts["tf_pose"], "x 0.42 · y 0.40 m · 179°")
        self.assertIsNone(texts["map"])  # no map topic: nothing to say
        card = shape_pinky("10", {"topics": topics}, {"connected": True})
        self.assertAlmostEqual(card["pose"]["x_m"], 0.4206)


class MapTest(unittest.TestCase):
    def test_live_map_with_fixed_points_and_a_zone_placed_on_its_waypoint(self) -> None:
        fleet = FakeFleet()
        record = robot_record()
        record["topics"]["map"] = {"fresh": True, "value": {"width": 2, "height": 1, "resolution_m": 0.05,
                                                           "origin_x_m": -0.349, "origin_y_m": -0.295, "cells": [0, 100]}}
        record["topics"]["line_control"] = {"fresh": True, "value": {"traffic_config": {
            "dock5": {"x_m": 0.39, "y_m": 1.64}, "junction7": {"x_m": 1.55, "y_m": 1.24}, "parking": {"bounds": [0.2, 1.2, 0.2, 0.6]}}}}
        record["parking"] = {"label": "P0 주차장", "x_m": 0.4, "y_m": 0.4, "yaw_rad": 3.14}
        record["waypoints"] = [{"label": "9", "x_m": 1.6, "y_m": 2.0}]
        fleet.robots = lambda: {"10": record, "11": {}, "12": {}}
        fleet.extras = lambda: {"keepout": None}
        view = PinkyGateway(fleet).map_view([{"zone_id": "zone_a", "label": "수령 구역 A", "route_labels": ["1", "9"]}])
        self.assertEqual((view["available"], view["grid"]["cells"]), (True, [0, 100]))
        self.assertEqual({lm["key"] for lm in view["landmarks"]}, {"dock5", "junction7"})
        self.assertEqual(view["zones"][0], {"zone_id": "zone_a", "label": "수령 구역 A", "waypoint": "9", "x_m": 1.6, "y_m": 2.0})
        self.assertIsNone(view["note"])

    def test_mock_map_places_pinkys_by_phase(self) -> None:
        world = MockWorld()
        view = PinkyGateway(None, world.fleet_status).map_view([{"zone_id": "zone_a", "label": "수령 구역 A"}])
        self.assertTrue(view["simulated"])
        self.assertEqual([(p["name"], p["x_m"], p["y_m"]) for p in view["pinkies"]][0], ("P0", 0.4, 0.4))
        self.assertEqual(len(view["grid"]["cells"]), 44 * 51)


class CommandTest(unittest.TestCase):
    def test_stop_all_sends_each_and_logs_each_answer(self) -> None:
        fleet = FakeFleet({"11": (401, {"error": "SSH 비밀번호를 먼저 입력하세요."})})
        gateway = PinkyGateway(fleet)
        results = gateway.stop_all()
        self.assertEqual([("stop", pid) for pid in ("10", "11", "12")], fleet.sent)
        self.assertEqual([entry["ok"] for entry in results], [True, False, True])
        self.assertEqual(results[1]["message"], "SSH 비밀번호를 먼저 입력하세요.")
        self.assertEqual([entry["pinky_id"] for entry in gateway.view()["log"]], ["12", "11", "10"])  # newest first

    def test_password_is_passed_on_but_never_logged(self) -> None:
        fleet = FakeFleet()
        gateway = PinkyGateway(fleet)
        entry = gateway.send_password("s3cret")
        self.assertTrue(entry["ok"])
        self.assertEqual(fleet.sent, [("password", "s3cret")])
        self.assertNotIn("s3cret", repr(gateway.view()))

    def test_mock_mode_shows_mock_pinkys_and_sends_nothing(self) -> None:
        gateway = PinkyGateway(None, MockWorld().fleet_status)
        view = gateway.view()
        self.assertEqual((view["source"], [p["simulated"] for p in view["pinkies"]]), ("mock_world", [True] * 3))
        entry = gateway.stop("10")
        self.assertEqual((entry["ok"], entry["simulated"]), (False, True))


class PinkyApiTest(ApiTestCase):
    def setUp(self) -> None:
        super().setUp()
        self.fleet = FakeFleet({"12": (409, {"error": "정지 적용 확인 중입니다. 잠시 기다려 주세요."})})
        self.server.pinky_gateway = PinkyGateway(self.fleet)

    def test_screen_and_commands(self) -> None:
        status, view, _ = self.request("GET", "/api/pinky")
        self.assertEqual((status, [p["name"] for p in view["pinkies"]]), (200, ["P0", "P1", "P2"]))
        status, body, _ = self.request("POST", "/api/pinky/10/stop")
        self.assertEqual((status, body["command"]["ok"]), (202, True))
        status, body, _ = self.request("POST", "/api/pinky/12/stop")
        self.assertEqual((status, body["command"]["message"]), (409, "정지 적용 확인 중입니다. 잠시 기다려 주세요."))
        status, body, _ = self.request("POST", "/api/pinky/stop-all")
        self.assertEqual((status, len(body["commands"])), (207, 3))
        status, body, _ = self.request("POST", "/api/pinky/99/stop")
        self.assertEqual((status, body["error_code"]), (404, "PINKY_UNKNOWN"))

    def test_password_route(self) -> None:
        status, body, _ = self.request("POST", "/api/pinky/credentials", {"password": "s3cret"})
        self.assertEqual((status, body["command"]["action"]), (200, "password"))
        self.assertNotIn("s3cret", str(body))
        status, _, _ = self.request("POST", "/api/pinky/credentials", {"password": ""})
        self.assertEqual(status, 400)
        status, _, _ = self.request("POST", "/api/pinky/10/stop", origin="http://evil.example")
        self.assertEqual(status, 403)


if __name__ == "__main__":
    unittest.main()
