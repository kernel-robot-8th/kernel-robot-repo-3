from datetime import datetime, timedelta, timezone

import pytest

from control_station import runtime_services as rs

LOCAL = {"CONTROL_STATION_PINKY_FLEET_URL": "http://127.0.0.1:8765"}
RIGHT_ARM = {
    "CONTROL_STATION_RIGHT_FOLLOWER_PORT": "/dev/ttyACM0",
    "CONTROL_STATION_RIGHT_CEILING_CAMERA": "/dev/video2",
    "CONTROL_STATION_RIGHT_JOINT_CAMERA": "/dev/video4",
}


def test_fleet_down_on_this_pc_can_be_started():
    row = rs.fleet_row(LOCAL, robots=None)
    assert (row["state"], row["can_start"]) == ("off", True)


def test_remote_fleet_is_never_started_from_here():
    row = rs.fleet_row({"CONTROL_STATION_PINKY_FLEET_URL": "http://10.0.0.5:8765"}, robots=None)
    assert (row["state"], row["can_start"]) == ("off", False)
    with pytest.raises(rs.RuntimeStartError) as error:
        rs.start("pinky_fleet", {"CONTROL_STATION_PINKY_FLEET_URL": "http://10.0.0.5:8765"})
    assert error.value.code == "PINKY_FLEET_NOT_LOCAL"


def test_fleet_names_the_pinkies_that_need_preparing_again():
    robots = [{"id": "10", "stage": "fault"}, {"id": "11", "stage": "fault"}, {"id": "12", "stage": "prepared"}]
    row = rs.fleet_row(LOCAL, robots=robots)
    assert row["state"] == "warn" and "10, 11번" in row["detail"]


def test_fleet_all_prepared_is_on():
    row = rs.fleet_row(LOCAL, robots=[{"id": "10", "stage": "prepared"}, {"id": "12", "stage": "prepared"}])
    assert (row["state"], row["detail"]) == ("on", "핑키 2대 준비 완료")


def test_mock_fleet_and_unset_parking_are_skipped():
    assert rs.fleet_row({})["state"] == "skip"
    assert rs.parking_row({})["state"] == "skip"


def test_parking_off_can_be_started():
    row = rs.parking_row({"CONTROL_STATION_PARKING_SPOT": "pinky_dock_1"}, running=False)
    assert (row["state"], row["can_start"]) == ("off", True)


def test_arm_without_cameras_is_skipped_and_fresh_worker_is_on():
    now = datetime(2026, 11, 27, 10, 0, tzinfo=timezone.utc)
    workers = [{"robot_id": "arm-02", "worker_id": "arm-right", "waiting_polls": 0,
                "last_poll_at": (now - timedelta(seconds=5)).isoformat()}]
    left, right = rs.arm_rows(RIGHT_ARM, workers, now)
    assert left["state"] == "skip"
    assert (right["state"], right["detail"]) == ("on", "arm-right")


def test_configured_arm_with_stale_worker_is_off():
    now = datetime(2026, 11, 27, 10, 0, tzinfo=timezone.utc)
    workers = [{"robot_id": "arm-02", "worker_id": "arm-right", "waiting_polls": 0,
                "last_poll_at": (now - timedelta(minutes=5)).isoformat()}]
    assert rs.arm_rows(RIGHT_ARM, workers, now)[1]["state"] == "off"


def test_policy_server_only_matters_with_an_arm():
    assert rs.policy_row({})["state"] == "skip"
    assert rs.policy_row(RIGHT_ARM, is_open=False)["state"] == "off"


def test_fleet_start_drops_the_control_station_ros_settings(monkeypatch, tmp_path):
    spawned = {}
    monkeypatch.setattr(rs, "read_fleet_robots", lambda url, timeout_s=0.8: None)
    monkeypatch.setattr(rs, "_port_open", lambda host, port, timeout_s=0.3: False)
    monkeypatch.setattr(rs, "LOG_DIR", tmp_path)
    monkeypatch.setattr(rs, "_spawn", lambda argv, log, env, cwd: spawned.update(argv=argv, env=env))
    env = {**LOCAL, "ROS_DOMAIN_ID": "87", "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST", "PATH": "/usr/bin"}
    rs.start("pinky_fleet", env)
    assert spawned["argv"][-1].endswith("pinky_fleet_dashboard/run.sh")
    assert "ROS_DOMAIN_ID" not in spawned["env"] and "ROS_AUTOMATIC_DISCOVERY_RANGE" not in spawned["env"]
    assert spawned["env"]["PATH"] == "/usr/bin"


def test_running_fleet_is_not_started_twice(monkeypatch):
    monkeypatch.setattr(rs, "read_fleet_robots", lambda url, timeout_s=0.8: [])
    with pytest.raises(rs.RuntimeStartError) as error:
        rs.start("pinky_fleet", LOCAL)
    assert error.value.status == 409


def test_unknown_service_cannot_be_started():
    with pytest.raises(rs.RuntimeStartError) as error:
        rs.start("policy_server", {})
    assert error.value.code == "NOT_STARTABLE"
