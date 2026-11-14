"""Shelf inventory: debounced camera counts, hold while an arm works, unknown without readings, our picks booked
as a ledger, and what open orders reserve."""

from __future__ import annotations

import os
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

from control_station.inventory import InventoryBook, InventoryMonitor, InventoryOverrides, StockReading
from control_station.mock_world import MockWorld
from control_station.store import TaskStore
from test_api_routes import ApiTestCase
from test_order_desk import make_desk
from test_order_flow import CARROT_APPLE, TERMINAL

APPLE = [{"object_class": "apple", "label": "사과"}]


def read(book: InventoryBook, at: float, **shelves: dict) -> list:
    return book.observe(StockReading(at, shelves))


class InventoryBookTest(unittest.TestCase):
    def test_a_count_is_confirmed_after_three_equal_readings(self) -> None:
        book = InventoryBook()
        for at in (1, 2):
            read(book, at, shelf_b={"apple": 3})
            self.assertEqual(book.state(at)[0], "unknown")
        read(book, 3, shelf_b={"apple": 3})
        self.assertEqual((book.state(3)[0], book.on_shelves()), ("live", {"shelf_b": {"apple": 3}}))

    def test_flicker_is_ignored_and_a_real_drop_is_logged(self) -> None:
        book = InventoryBook()
        for at in (1, 2, 3):
            read(book, at, shelf_b={"apple": 3})
        for at, count in ((4, 2), (5, 3), (6, 2), (7, 3)):  # a missed detection now and then
            self.assertEqual(read(book, at, shelf_b={"apple": count}), [])
        self.assertEqual(book.on_shelves()["shelf_b"]["apple"], 3)
        changes = [read(book, at, shelf_b={"apple": 2}) for at in (8, 9, 10)]
        self.assertEqual(changes[:2], [[], []])
        self.assertEqual(changes[2], [{"shelf_id": "shelf_b", "object_class": "apple", "from": 3, "to": 2,
                                       "kind": "REMOVED_OUTSIDE"}])

    def test_readings_while_an_arm_works_are_not_used(self) -> None:
        book = InventoryBook()
        for at in (1, 2, 3):
            read(book, at, shelf_b={"apple": 3})
        for at in (4, 5, 6):  # the arm hides the shelf
            book.observe(StockReading(at, {"shelf_b": {"apple": 0}}, busy=True))
        self.assertEqual((book.state(6)[0], book.on_shelves()["shelf_b"]["apple"]), ("held", 3))

    def test_no_reading_for_a_while_makes_the_stock_unknown(self) -> None:
        book = InventoryBook(stale_after_s=10)
        for at in (1, 2, 3):
            read(book, at, shelf_b={"apple": 3})
        book.observe(StockReading(9, error="선반 재고를 읽지 못했습니다: 카메라 없음"))
        self.assertEqual(book.state(9)[0], "live")
        self.assertEqual(book.state(14), ("unknown", "선반 재고를 읽지 못했습니다: 카메라 없음"))
        self.assertIsNone(book.snapshot(APPLE, {}, 14)["products"][0]["available"])

    def test_our_picks_are_booked_before_the_camera_sees_them(self) -> None:
        book = InventoryBook()
        for at in (1, 2, 3):
            read(book, at, shelf_b={"apple": 3})
        book.book_moves([("shelf_b", "apple")])
        self.assertEqual(book.on_shelves()["shelf_b"]["apple"], 2)
        self.assertEqual([read(book, at, shelf_b={"apple": 2}) for at in (4, 5, 6)], [[], [], []])  # no outside change

    def test_available_sold_out_and_unknown(self) -> None:
        book = InventoryBook()
        for at in (1, 2, 3):
            read(book, at, shelf_a={"apple": 1}, shelf_b={"apple": 3})

        def row(reserved: int, now: float = 3, overrides=None) -> dict:
            return book.snapshot(APPLE, {"apple": reserved}, now, overrides)["products"][0]

        self.assertEqual((row(3)["available"], row(3)["status"], row(3)["per_shelf"]),
                         (1, "available", {"shelf_a": 1, "shelf_b": 3}))
        self.assertEqual((row(4)["available"], row(4)["status"]), (0, "sold_out"))
        self.assertEqual(row(6)["available"], 0)  # never negative
        self.assertEqual((row(0, now=30)["status"], row(0, now=30)["available"]), ("unknown", None))

    def test_operator_overrides(self) -> None:
        book = InventoryBook()
        unknown_camera = book.snapshot(APPLE, {"apple": 1}, 0, {"apple": {"mode": "manual", "count": 3}})["products"][0]
        self.assertEqual((unknown_camera["source"], unknown_camera["available"], unknown_camera["status"]),
                         ("manual", 2, "available"))  # a hand count keeps selling while the camera is out
        closed = book.snapshot(APPLE, {}, 0, {"apple": {"mode": "closed"}})["products"][0]
        self.assertEqual((closed["status"], closed["available"]), ("closed", 0))


class InventoryOverridesTest(unittest.TestCase):
    def test_settings_survive_a_restart_and_auto_clears_them(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "overrides.json"
        InventoryOverrides(path).set("apple", "manual", 2)
        InventoryOverrides(path).set("carrot", "closed")
        reopened = InventoryOverrides(path)
        self.assertEqual({name: value["mode"] for name, value in reopened.all().items()},
                         {"apple": "manual", "carrot": "closed"})
        reopened.set("apple", "auto")
        self.assertNotIn("apple", InventoryOverrides(path).all())
        with self.assertRaises(ValueError):
            reopened.set("apple", "manual", -1)


class InventoryWithOrdersTest(unittest.TestCase):
    def setUp(self) -> None:
        patcher = mock.patch.dict(os.environ, {"CONTROL_STATION_MOCK_STEP_DELAY": "0.05"})
        patcher.start()
        self.addCleanup(patcher.stop)
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.store = TaskStore(str(Path(directory.name) / "t.sqlite3"))
        self.addCleanup(self.store.close)
        self.desk, self.world, orchestrator = make_desk(self.store)
        self.addCleanup(orchestrator.close)

    def test_open_orders_reserve_until_their_units_are_moved(self) -> None:
        order = self.desk.receive(CARROT_APPLE, "zone_a")
        self.assertEqual(self.store.reserved_units(), {"carrot": 1, "apple": 2})
        clock = iter(range(1, 1000))
        monitor = InventoryMonitor(self.world.shelf_stock, lambda: False, lambda: APPLE,
                                   self.store.reserved_units, self.store.confirmed_transfers,
                                   clock=lambda: next(clock))
        for _ in range(3):
            monitor.read_once()
        self.assertEqual(monitor.snapshot()["products"][0]["available"], 2)  # 4 apples, 2 reserved
        self.desk.start(order["id"])
        deadline = time.monotonic() + 15
        while self.store.get_task(order["id"])["status"] not in TERMINAL and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertEqual(self.store.get_task(order["id"])["status"], "SUCCEEDED")
        self.assertEqual(self.store.reserved_units(), {})
        self.assertEqual(sorted(name for _, _, name in self.store.confirmed_transfers()), ["apple", "apple", "carrot"])
        monitor.read_once()  # books the three moves; the mock shelves already show them
        self.assertEqual(monitor.snapshot()["products"][0]["available"], 2)  # 4 - 2 moved, nothing reserved
        self.assertEqual(monitor.snapshot()["events"], [])  # our own picks are not outside changes



class InventoryApiTest(ApiTestCase):
    def setUp(self) -> None:
        super().setUp()
        world = MockWorld()
        self.server.inventory = InventoryMonitor(
            world.shelf_stock, lambda: False, lambda: APPLE, self.task_store.reserved_units,
            self.task_store.confirmed_transfers, overrides=InventoryOverrides(self.directory / "overrides.json"))

    def test_read_and_override(self) -> None:
        for _ in range(3):
            self.server.inventory.read_once()
        status, view, _ = self.request("GET", "/api/inventory")
        self.assertEqual((status, view["products"][0]["available"]), (200, 4))
        status, view, _ = self.request("POST", "/api/inventory/apple", {"mode": "manual", "count": 1})
        self.assertEqual((status, view["products"][0]["source"], view["products"][0]["available"]), (200, "manual", 1))
        self.assertEqual(view["events"][0]["kind"], "OVERRIDE")
        status, body, _ = self.request("POST", "/api/inventory/banana", {"mode": "closed"})
        self.assertEqual((status, body["error_code"]), (400, "INVENTORY_OVERRIDE_INVALID"))
        status, _, _ = self.request("POST", "/api/inventory/apple", {"mode": "closed"}, origin="http://evil.example")
        self.assertEqual(status, 403)


if __name__ == "__main__":
    unittest.main()
