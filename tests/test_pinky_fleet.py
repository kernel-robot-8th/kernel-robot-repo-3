"""PinkyFleetClient against a fake of the Pinky team's dashboard (same routes and answers as origin/pinky_fleet)."""

from __future__ import annotations

import json
import os
import tempfile
import threading
import time
import unittest
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest import mock

from control_station.act_policies import load_act_policies
from control_station.arms import load_arms
from control_station.mock_world import MockWorld
from control_station.order_flow import OrderDevices, OrderFlow
from control_station.pinky_fleet import _fresh_battery, PinkyFleetClient, PinkyFleetError
from control_station.store import TaskStore
from control_station.task_orchestrator import MockTaskOrchestrator


class FakeFleet:
    """Mission state like the dashboard keeps it: route status, phase and a fresh token per waiting phase."""

    def __init__(self) -> None:
        self.routes: dict[str, dict | None] = {"10": None, "11": None, "12": None}
        self.signals: list[dict] = []
        self.deliver = True

    def at_station(self, pinky_id: str) -> None:
        self.routes[pinky_id] = {"status": "running", "phase": "docking_manual", "mission_token": uuid.uuid4().hex,
                                 "message": "5번 도착 · 정지 · IR 보정 전 수동 도킹완료 확인 대기", "current_label": "5"}

    def handler(fleet):  # noqa: N805
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args: object) -> None:
                pass

            def _json(self, status: int, body: dict) -> None:
                raw = json.dumps(body).encode()
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)

            def do_GET(self) -> None:  # noqa: N802
                if self.path == "/api/status":
                    self._json(200, {"robots": [{"id": key, "stage": "ready", "message": "준비 완료", "route": route}
                                                for key, route in fleet.routes.items()]})
                else:
                    self._json(404, {"error": "not found"})

            def do_POST(self) -> None:  # noqa: N802
                data = json.loads(self.rfile.read(int(self.headers.get("Content-Length", "0"))) or b"{}")
                robot_id = self.path.split("/")[3]
                route = fleet.routes.get(robot_id) or {}
                phases = {"dock_complete": "docking_manual", "received": "docked_manual"}
                action = data.get("action")
                if (action not in phases or route.get("status") != "running" or route.get("phase") != phases[action]
                        or data.get("token") != route.get("mission_token")):
                    self._json(409, {"error": "현재 임무의 확인 단계와 신호가 일치하지 않습니다."})
                    return
                if not fleet.deliver:
                    self._json(502, {"ok": False, "message": "ssh: connect to host 192.168.0.9 port 22: No route to host"})
                    return
                fleet.signals.append({"robot": robot_id, **data})
                if action == "dock_complete":
                    route.update(phase="docked_manual", mission_token=uuid.uuid4().hex, message="도킹완료 · 정지 유지 · 물건 수령 완료 대기")
                else:
                    route.update(phase="driving", mission_token=None, message="확인 완료 · 다음 단계 진행")
                self._json(202, {"ok": True, "message": "현재 단계 확인 신호를 보냈습니다."})

        return Handler


class PinkyFleetClientTests(unittest.TestCase):
    def setUp(self) -> None:
        self.fleet = FakeFleet()
        server = ThreadingHTTPServer(("127.0.0.1", 0), self.fleet.handler())
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        self.client = PinkyFleetClient(f"http://127.0.0.1:{server.server_address[1]}", status_max_age_s=0)

    def test_phases_map_to_the_order_flow(self) -> None:
        self.assertEqual(self.client.fleet_status("10")["phase"], "parked")
        self.fleet.routes["11"] = {"status": "running", "message": "Nav2 경로 주행 중"}
        self.assertEqual(self.client.fleet_status("11")["phase"], "driving")
        self.fleet.at_station("12")
        status = self.client.fleet_status("12")
        self.assertEqual((status["phase"], status["simulated"]), ("docking_manual", False))
        self.assertEqual(status["mission_token"], self.fleet.routes["12"]["mission_token"])
        with self.assertRaises(PinkyFleetError):
            self.client.fleet_status("13")

    def test_signals_need_the_current_token(self) -> None:
        self.fleet.at_station("10")
        token = self.client.fleet_status("10")["mission_token"]
        with self.assertRaises(PinkyFleetError) as rejected:
            self.client.fleet_confirm("10", "dock_complete", "old-token")
        self.assertEqual(rejected.exception.error_code, "PINKY_SIGNAL_REJECTED")
        self.assertTrue(self.client.fleet_confirm("10", "dock_complete", token)["accepted"])
        self.assertEqual(self.client.fleet_status("10")["phase"], "docked_manual")
        self.fleet.deliver = False
        with self.assertRaises(PinkyFleetError) as undelivered:
            self.client.fleet_confirm("10", "received", self.client.fleet_status("10")["mission_token"])
        self.assertEqual(undelivered.exception.error_code, "PINKY_SIGNAL_NOT_DELIVERED")

    def test_unreachable_dashboard_is_an_os_error(self) -> None:
        with self.assertRaises(OSError):
            PinkyFleetClient("http://127.0.0.1:9", timeout_s=0.5).fleet_status("10")

    def test_an_order_signals_the_real_dashboard_and_the_pinky_drives_on(self) -> None:
        patcher = mock.patch.dict(os.environ, {"CONTROL_STATION_MOCK_STEP_DELAY": "0.05"})
        patcher.start()
        self.addCleanup(patcher.stop)
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        store = TaskStore(str(Path(directory.name) / "t.sqlite3"))
        self.addCleanup(store.close)
        world = MockWorld(drive_s=0.1, pick_s=0.05)
        devices = OrderDevices(world.shelf_stock, world.pinky_load, self.client.fleet_status, self.client.fleet_confirm,
                               world.parking, world.pick, None)
        orchestrator = MockTaskOrchestrator(store, order_flow=OrderFlow(load_arms(), load_act_policies(), devices,
                                                                         dwell_s=0.0, poll_s=0.02))
        self.addCleanup(orchestrator.close)
        task = store.create_task("핑키 적재 위치", "주문", order_items=[{"object_class": "carrot", "quantity": 1}])
        orchestrator.start(task["id"])
        time.sleep(0.3)
        self.assertEqual(store.get_task(task["id"])["status"], "RUNNING")  # the Pinky is not at the station yet
        self.fleet.at_station("10")
        deadline = time.monotonic() + 10
        while store.get_task(task["id"])["status"] == "RUNNING" and time.monotonic() < deadline:
            time.sleep(0.05)
        finished = store.get_task(task["id"])
        self.assertEqual(finished["status"], "SUCCEEDED", finished["error"])
        self.assertEqual([signal["action"] for signal in self.fleet.signals], ["dock_complete", "received"])
        self.assertEqual(self.fleet.routes["10"]["phase"], "driving")



class BatteryReadingTest(unittest.TestCase):
    """Shapes seen from the real dashboard on 2026-11-14 (Pinky C76C, domain 10)."""

    def test_zero_percent_beside_a_live_voltage_is_unknown(self) -> None:
        robot = {"topics": {"battery_percent": {"fresh": True, "value": {"percent": 0.0}},
                            "battery_voltage": {"fresh": True, "value": {"voltage_v": 6.59249210357666}}}}
        self.assertEqual(_fresh_battery(robot), (None, 6.59249210357666))

    def test_a_real_percentage_and_stale_topics(self) -> None:
        robot = {"topics": {"battery_percent": {"fresh": True, "value": {"percent": 64.0}},
                            "battery_voltage": {"fresh": False, "value": {"voltage_v": 7.6}}}}
        self.assertEqual(_fresh_battery(robot), (64.0, None))
        self.assertEqual(_fresh_battery({}), (None, None))


if __name__ == "__main__":
    unittest.main()
