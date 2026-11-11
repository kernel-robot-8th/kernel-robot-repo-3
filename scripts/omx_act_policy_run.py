"""Run a LeRobot ACT policy on the real OMX follower for one PICK_WITH_POLICY command, and stop it when the arm
has come back to rest. Used by omx_lerobot_worker.py.

The policy runs through LeRobot async inference: a policy server (`python -m lerobot.async_inference.policy_server`,
started separately on this PC) and LeRobot's RobotClient, which this module runs in the worker process.
An ACT policy has no "done" signal and never stops by itself, so this module decides when to stop:

- RETURNED_TO_REST: the commanded joints first left the recorded home pose (more than `depart_ticks` on some
  joint), then stayed within `rest_ticks` of it for `settle_s`. The training episodes start and end at rest.
- NEVER_DEPARTED: `max_run_s` passed without the arm leaving rest.
- TIMEOUT: `max_run_s` passed while the arm was away from rest.
- CANCELED: the operator canceled the command.

Arriving back at rest says nothing about whether the object was grasped or placed.

Stopping keeps torque on (`disable_torque_on_disconnect=False`): LeRobot's default turns torque off and the arm
falls. LeRobot is imported only when a policy actually runs, so the tests need neither LeRobot nor an arm.
"""

from __future__ import annotations

import hashlib
import json
import os
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_POLICIES = REPOSITORY_ROOT / "config" / "omx-act-policies.json"
ROLES = ("OVERHEAD", "GRIPPER")


def load_policies(path: Path) -> tuple[str, dict[str, dict[str, Any]]]:
    """(SHA-256 of the file, policies by id). The control station freezes the same SHA-256 into each command."""
    raw = path.read_bytes()
    document = json.loads(raw)
    if document.get("schema_version") != 1:
        raise SystemExit(f"{path}: schema_version must be 1")
    return hashlib.sha256(raw).hexdigest(), {policy["policy_id"]: policy for policy in document["policies"]}


def policy_cameras(policy: dict[str, Any], devices: dict[str, str]) -> dict[str, str]:
    """Policy input name (front/wrist) -> device path, following the camera roles the policy was trained with."""
    return {name: devices[role] for name, role in policy["cameras"].items()}


@dataclass
class RestWatcher:
    """Decides when the policy is done from the commanded joint ticks. Pure; fed by the control loop."""

    home: dict[str, int]
    depart_ticks: int = 300
    rest_ticks: int = 150
    settle_s: float = 1.5
    departed: bool = False
    rest_since: float | None = None
    farthest: int = 0

    def distance(self, ticks: dict[str, int]) -> int:
        return max(abs(int(ticks[name]) - int(self.home[name])) for name in ARM_JOINTS)

    def update(self, now: float, ticks: dict[str, int]) -> bool:
        """True once the arm has left rest and has been back within rest_ticks for settle_s."""
        distance = self.distance(ticks)
        self.farthest = max(self.farthest, distance)
        if distance > self.depart_ticks:
            self.departed = True
        if not self.departed or distance > self.rest_ticks:
            self.rest_since = None
            return False
        if self.rest_since is None:
            self.rest_since = now
        return now - self.rest_since >= self.settle_s


@dataclass
class PolicyRunResult:
    stop_reason: str  # RETURNED_TO_REST, NEVER_DEPARTED, TIMEOUT, CANCELED, ERROR
    run_s: float
    farthest_from_home_ticks: int
    last_commanded_ticks: dict[str, int] = field(default_factory=dict)
    actions_sent: int = 0
    message: str = ""


def devices_in_use(paths: list[str]) -> list[str]:
    """Devices among `paths` that another process of this user has open (V4L2 lets only one program stream).
    Processes of other users are not visible without root and are not reported."""
    wanted = {os.path.realpath(path) for path in paths}
    busy: set[str] = set()
    me = str(os.getpid())
    for entry in os.scandir("/proc"):
        if not entry.name.isdigit() or entry.name == me:
            continue
        try:
            for fd in os.scandir(f"/proc/{entry.name}/fd"):
                try:
                    target = os.readlink(fd.path)
                except OSError:
                    continue
                if target in wanted:
                    busy.add(target)
        except OSError:
            continue
    return sorted(busy)


def wait_devices_free(paths: list[str], timeout_s: float = 5.0) -> list[str]:
    """Wait for the control station to close the policy's cameras; returns those still open after `timeout_s`."""
    deadline = time.monotonic() + timeout_s
    while True:
        busy = devices_in_use(paths)
        if not busy or time.monotonic() >= deadline:
            return busy
        time.sleep(0.2)


def check_policy_server(address: str, timeout_s: float = 2.0, ready_timeout_s: float = 5.0) -> str | None:
    """None when the policy server answers its Ready call, else a short error. Nothing is sent to the arm.

    An open port is not enough: a server left with streams of an earlier client accepted connections but never
    answered Ready (2026-11-11), and RobotClient.start() waits for Ready without a time limit."""
    import grpc
    from lerobot.transport import services_pb2, services_pb2_grpc

    channel = grpc.insecure_channel(address)
    try:
        grpc.channel_ready_future(channel).result(timeout=timeout_s)
        services_pb2_grpc.AsyncInferenceStub(channel).Ready(services_pb2.Empty(), timeout=ready_timeout_s)
        return None
    except grpc.FutureTimeoutError:
        return f"no policy server at {address}"
    except grpc.RpcError as error:
        return f"policy server at {address} did not answer Ready: {error.code().name}"
    finally:
        channel.close()


def run_policy(
    policy: dict[str, Any],
    *,
    port: str,
    cameras: dict[str, str],
    server_address: str,
    watcher: RestWatcher,
    max_run_s: float,
    cancel_requested: Callable[[], bool],
    started: Callable[[], None] | None = None,
    max_step_ticks: int | None = None,
    policy_device: str = "cuda",
) -> PolicyRunResult:
    """Run the policy until the watcher, the time limit or a cancel stops it. Torque stays on afterwards.

    `max_step_ticks` limits how far any joint goal may be from the present position at each step (1/30 s); used
    when the policy starts from a pose it was not trained from and its first actions may jump."""
    from lerobot.async_inference.configs import RobotClientConfig
    from lerobot.async_inference.robot_client import RobotClient
    from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
    from lerobot.robots.omx_follower.config_omx_follower import OmxFollowerConfig

    class WatchedClient(RobotClient):
        """RobotClient that remembers the last joint goal it wrote, in raw ticks."""

        last_ticks: dict[str, int] = {}
        actions_sent = 0

        def control_loop_action(self, verbose: bool = False):
            action = super().control_loop_action(verbose)
            calibration = self.robot.bus.calibration
            ticks = {}
            for name in ARM_JOINTS:
                low, high = calibration[name].range_min, calibration[name].range_max
                ticks[name] = round((float(action[f"{name}.pos"]) + 100) / 200 * (high - low) + low)
            self.last_ticks = ticks
            self.actions_sent += 1
            return action

    robot = OmxFollowerConfig(
        port=port,
        id="omx_follower_arm",
        disable_torque_on_disconnect=False,
        # LeRobot clamps each goal to present ± this, in its normalized units (range 0..4095 ticks -> -100..100).
        max_relative_target=None if max_step_ticks is None else max_step_ticks * 200 / 4095,
        cameras={
            name: OpenCVCameraConfig(index_or_path=Path(path), width=640, height=480, fps=30, fourcc="MJPG")
            for name, path in cameras.items()
        },
    )
    config = RobotClientConfig(
        policy_type=policy["policy_type"],
        pretrained_name_or_path=policy["model"],
        robot=robot,
        actions_per_chunk=100,
        task=policy["task"],
        server_address=server_address,
        policy_device=policy_device,
        chunk_size_threshold=0.7,
        fps=30,
        aggregate_fn_name="weighted_average",
        debug_visualize_queue_size=False,
    )
    begin = time.monotonic()
    client = WatchedClient(config)  # connects the arm: LeRobot briefly turns torque off while it configures
    receiver = control = None
    try:
        if not client.start():
            return PolicyRunResult("ERROR", time.monotonic() - begin, 0, message="policy server refused the client")
        receiver = threading.Thread(target=client.receive_actions, daemon=True)
        control = threading.Thread(target=client.control_loop, args=(policy["task"],), daemon=True)
        receiver.start()
        control.start()
        if started:
            started()
        next_cancel_check = 0.0
        reason, message = "", ""
        while not reason:
            time.sleep(0.05)
            now = time.monotonic()
            if not control.is_alive():
                reason, message = "ERROR", "control loop stopped"
            elif client.last_ticks and watcher.update(now, client.last_ticks):
                reason = "RETURNED_TO_REST"
            elif now - begin > max_run_s:
                reason = "TIMEOUT" if watcher.departed else "NEVER_DEPARTED"
            elif now >= next_cancel_check:
                next_cancel_check = now + 1.0
                try:
                    if cancel_requested():
                        reason = "CANCELED"
                except Exception as error:  # cannot read the cancel flag: stop the policy rather than run blind
                    reason, message = "ERROR", f"cancel check failed: {error}"[:300]
        return PolicyRunResult(reason, now - begin, watcher.farthest, dict(client.last_ticks), client.actions_sent, message)
    finally:
        client.shutdown_event.set()
        if control is not None:
            control.join(timeout=5)  # the loop owns the bus; let it finish its step before disconnecting
        client.stop()  # robot.disconnect keeps torque (config above), cameras and the gRPC channel close
        if receiver is not None:
            receiver.join(timeout=3)
