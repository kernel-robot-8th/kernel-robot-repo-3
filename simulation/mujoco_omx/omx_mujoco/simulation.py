"""Small, checked Python control layer around the pinned OMX MJCF model."""

from __future__ import annotations

import math
from pathlib import Path
import time
from typing import Mapping


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_SCENE = (
    REPOSITORY_ROOT
    / "simulation"
    / "mujoco_omx"
    / "robotis_omx"
    / "shelf_photo_match_scene.xml"
)
PICK_PLACE_SCENE = DEFAULT_SCENE.with_name("pick_place_scene.xml")
ARM_JOINTS = ("Joint1", "Joint2", "Joint3", "Joint4", "Joint5")
CONTROL_JOINTS = (*ARM_JOINTS, "Gripper")
GRIPPER_OPEN_RAD = 1.745329  # upper bound declared in the pinned MJCF
GRIPPER_CLOSED_RAD = 0.0


class OMXSimulation:
    """Own one MuJoCo model/data pair and expose only bounded position control."""

    def __init__(
        self,
        scene_path: str | Path = DEFAULT_SCENE,
        *,
        gripper_contact_proxy: bool = False,
        gripper_contact_proxy_radius_m: float = 0.006,
        grasp_weld: bool = False,
    ) -> None:
        try:
            import mujoco
        except ImportError as exc:  # pragma: no cover - depends on optional environment
            raise RuntimeError(
                "MuJoCo is required. Install requirements-mujoco.txt in the simulator environment."
            ) from exc

        self.mujoco = mujoco
        self.scene_path = Path(scene_path).resolve()
        self.spec = mujoco.MjSpec.from_file(str(self.scene_path))
        self._is_photo_match_scene = self.spec.camera("photo_match_front_camera") is not None
        self._is_strawberry_pick_scene = self.scene_path.name == "strawberry_pick_scene.xml"
        if self._is_strawberry_pick_scene:
            strawberry = self.spec.body("photo_match_strawberry")
            strawberry.pos = [0.10, 0.21, 0.220]
            strawberry_joint = strawberry.add_freejoint(name="photo_match_strawberry_free")
            strawberry_joint.damping = [0.5, 0.5, 0.5]
            fruit = self.spec.geom("photo_match_strawberry_body")
            fruit.mass = 0.020
            fruit.friction = [1.0, 0.02, 0.001]
            stem = self.spec.geom("photo_match_strawberry_stem")
            stem.contype = 0
            stem.conaffinity = 0
            stem.density = 0.0
        self._configure_scene_start()
        self._add_wrist_camera_alias()
        self._add_fixed_camera_alias()
        self._grasp_weld = bool(grasp_weld)
        if self._grasp_weld:
            if not self._is_strawberry_pick_scene:
                raise ValueError("grasp weld is restricted to the strawberry experiment scene")
            self.spec.add_equality(
                name="simulation_grasp_weld",
                type=mujoco.mjtEq.mjEQ_WELD,
                objtype=mujoco.mjtObj.mjOBJ_BODY,
                name1="link6",
                name2="photo_match_strawberry",
                active=False,
            )
        self._gripper_contact_proxy = bool(gripper_contact_proxy)
        self._viewer = None
        self._viewer_sync_sim_time = 0.0
        if self._gripper_contact_proxy:
            if not self._is_strawberry_pick_scene:
                raise ValueError("gripper contact proxies are restricted to the strawberry experiment scene")
            if (
                not math.isfinite(gripper_contact_proxy_radius_m)
                or not 0.002 <= gripper_contact_proxy_radius_m <= 0.02
            ):
                raise ValueError("gripper contact proxy radius must be between 0.002 and 0.02 m")
            self._add_gripper_contact_proxies(gripper_contact_proxy_radius_m)
        self.model = self.spec.compile()
        self._grasp_weld_id = -1
        if self._grasp_weld:
            self._grasp_weld_id = int(
                mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_EQUALITY, "simulation_grasp_weld"
                )
            )
            if self._grasp_weld_id < 0:
                raise RuntimeError("Failed to compile the simulation grasp weld")
        if self._gripper_contact_proxy:
            self._disable_original_gripper_collision_meshes()
        self.data = mujoco.MjData(self.model)
        self._joint_ids: dict[str, int] = {}
        self._actuator_ids: dict[str, int] = {}
        for name in CONTROL_JOINTS:
            joint_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            actuator_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
            )
            if joint_id < 0 or actuator_id < 0:
                raise ValueError(f"Required OMX joint or actuator is missing: {name}")
            if self.model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
                raise ValueError(f"OMX control joint is not a hinge: {name}")
            if not self.model.actuator_ctrllimited[actuator_id]:
                raise ValueError(f"OMX actuator is not position bounded: {name}")
            self._joint_ids[name] = int(joint_id)
            self._actuator_ids[name] = int(actuator_id)
        self.reset()

    def _add_gripper_contact_proxies(self, radius_m: float) -> None:
        """Add explicit simulation-only fingertip contacts for a grasp experiment.

        The local positions are estimated from the distal mesh vertices. The
        official visual meshes remain visible; their broad collision meshes are
        disabled after compile when this opt-in experiment mode is selected.
        """
        mujoco = self.mujoco
        for body_name, geom_name, local_pos in (
            ("link6", "link6_fingertip_contact_proxy", [0.06303, -0.00563, 0.00038]),
            ("link7", "link7_fingertip_contact_proxy", [0.06449, 0.00730, 0.00059]),
        ):
            self.spec.body(body_name).add_geom(
                name=geom_name,
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=[radius_m],
                pos=local_pos,
                density=0.0,
                friction=[2.0, 0.02, 0.001],
                rgba=[0.12, 0.12, 0.12, 1.0],
            )

    def _disable_original_gripper_collision_meshes(self) -> None:
        """Keep official gripper meshes visible but use explicit tip contacts."""
        mujoco = self.mujoco
        gripper_bodies = {
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            for name in ("link6", "link7")
        }
        for geom_id in range(self.model.ngeom):
            if (
                int(self.model.geom_bodyid[geom_id]) in gripper_bodies
                and self.model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_MESH
                and self.model.geom_contype[geom_id] != 0
            ):
                self.model.geom_contype[geom_id] = 0
                self.model.geom_conaffinity[geom_id] = 0

    def _configure_scene_start(self) -> None:
        """Keep the established photo-match start pose for the default scene."""
        if not self._is_photo_match_scene:
            return
        yaw = math.pi / 2.0
        base = self.spec.body("link0")
        base.pos = [-0.03, -0.03, 0.0]
        base.quat = [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]

    def _add_wrist_camera_alias(self) -> None:
        """Attach the public camera name without editing the pinned source MJCF."""
        mujoco = self.mujoco
        try:
            if self.spec.camera("wrist_camera") is not None:
                return
        except (KeyError, ValueError):
            pass
        try:
            reference = self.spec.camera("placeholder_wrist_camera")
        except (KeyError, ValueError) as exc:
            raise ValueError("The OMX MJCF must provide its wrist camera pose") from exc
        camera = self.spec.body("link6").add_camera(
            name="wrist_camera",
            pos=reference.pos.tolist(),
            fovy=float(reference.fovy),
        )
        # MuJoCo's parsed MjSpec camera exposes the default quaternion here even
        # though the source XML uses xyaxes. Convert that declared camera basis:
        # local -Z looks along the gripper's +X and camera +Y is link +Z.
        camera.quat = [0.5, 0.5, -0.5, -0.5]

    def _add_fixed_camera_alias(self) -> None:
        if self.spec.camera("fixed_camera") is not None:
            return
        reference_name = (
            "photo_match_front_camera"
            if self._is_photo_match_scene
            else "placeholder_fixed_camera"
        )
        reference = self.spec.camera(reference_name)
        if reference is None:
            raise ValueError("The scene must define a fixed camera")
        camera = self.spec.worldbody.add_camera(
            name="fixed_camera",
            pos=reference.pos.tolist(),
            fovy=float(reference.fovy),
        )
        if self._is_photo_match_scene:
            camera.quat = [0.7933533403, 0.6087614290, 0.0, 0.0]
        else:
            camera.quat = [1.0, 0.0, 0.0, 0.0]

    def reset(self) -> None:
        """Reset robot and free objects to the scene's MJCF initial positions."""
        self.mujoco.mj_resetData(self.model, self.data)
        photo_pose = {
            "Joint1": math.radians(-65.0),
            "Joint2": 1.11827,
            "Joint3": -1.81930,
            "Joint4": -1.745329,
            "Joint5": 2.96672,
            "Gripper": 1.02418,
        }
        for name in CONTROL_JOINTS:
            joint_id = self._joint_ids[name]
            actuator_id = self._actuator_ids[name]
            qpos = (
                photo_pose[name]
                if self._is_photo_match_scene
                else float(self.data.qpos[self.model.jnt_qposadr[joint_id]])
            )
            lower, upper = self.joint_limits(name)
            qpos = min(upper, max(lower, qpos))
            self.data.qpos[self.model.jnt_qposadr[joint_id]] = qpos
            self.data.ctrl[actuator_id] = qpos
        if self._is_photo_match_scene:
            mimic_id = self.mujoco.mj_name2id(
                self.model, self.mujoco.mjtObj.mjOBJ_JOINT, "Gripper_mimic"
            )
            self.data.qpos[self.model.jnt_qposadr[mimic_id]] = -photo_pose["Gripper"]
        self.mujoco.mj_forward(self.model, self.data)

    def joint_positions(self, *, include_mimic: bool = False) -> dict[str, float]:
        """Return current simulated joint angles in radians."""
        names = (*CONTROL_JOINTS, "Gripper_mimic") if include_mimic else CONTROL_JOINTS
        state: dict[str, float] = {}
        for name in names:
            joint_id = self.mujoco.mj_name2id(
                self.model, self.mujoco.mjtObj.mjOBJ_JOINT, name
            )
            if joint_id < 0:
                raise ValueError(f"Simulation joint is missing: {name}")
            state[name] = float(self.data.qpos[self.model.jnt_qposadr[joint_id]])
        return state

    def joint_limits(self, name: str) -> tuple[float, float]:
        """Return the intersection of joint and actuator limits in radians."""
        if name not in self._joint_ids:
            raise ValueError(f"Unsupported control joint: {name}")
        joint_id = self._joint_ids[name]
        actuator_id = self._actuator_ids[name]
        joint_lower, joint_upper = self.model.jnt_range[joint_id]
        ctrl_lower, ctrl_upper = self.model.actuator_ctrlrange[actuator_id]
        lower = max(float(joint_lower), float(ctrl_lower))
        upper = min(float(joint_upper), float(ctrl_upper))
        return lower, upper

    def _validate_targets(self, targets: Mapping[str, float]) -> dict[str, float]:
        if not targets:
            raise ValueError("At least one joint target is required")
        validated: dict[str, float] = {}
        for name, raw_value in targets.items():
            if name not in self._joint_ids:
                raise ValueError(f"Unsupported control joint: {name}")
            if isinstance(raw_value, bool):
                raise ValueError(f"{name} target must be a finite radian value")
            value = float(raw_value)
            if not math.isfinite(value):
                raise ValueError(f"{name} target must be a finite radian value")
            lower, upper = self.joint_limits(name)
            if value < lower or value > upper:
                raise ValueError(
                    f"{name} target {value:.6f} rad is outside model range "
                    f"[{lower:.6f}, {upper:.6f}] rad"
                )
            validated[name] = value
        return validated

    def set_joint_targets(self, targets: Mapping[str, float]) -> None:
        """Set bounded position-actuator targets without advancing simulation time."""
        validated = self._validate_targets(targets)
        for name, value in validated.items():
            self.data.ctrl[self._actuator_ids[name]] = value

    def step(self, steps: int = 1) -> None:
        """Advance simulation by a positive integer number of physics steps."""
        if isinstance(steps, bool) or not isinstance(steps, int) or steps < 1:
            raise ValueError("steps must be a positive integer")
        if self._viewer is None:
            self.mujoco.mj_step(self.model, self.data, nstep=steps)
            return

        remaining = steps
        sync_interval = max(1, round(0.025 / float(self.model.opt.timestep)))
        while remaining:
            chunk = min(remaining, sync_interval)
            with self._viewer.lock():
                self.mujoco.mj_step(self.model, self.data, nstep=chunk)
            remaining -= chunk
            if self.data.time - self._viewer_sync_sim_time >= 0.025:
                self._viewer.sync()
                self._viewer_sync_sim_time = float(self.data.time)
                time.sleep(0.001)

    def attach_viewer(self, viewer: object) -> None:
        """Synchronize this simulation to a passive MuJoCo viewer during stepping."""
        self._viewer = viewer
        self._viewer_sync_sim_time = float(self.data.time)
        self._viewer.sync()

    def move_joint_targets(
        self, targets: Mapping[str, float], duration_s: float = 1.0
    ) -> dict[str, float]:
        """Move selected joints with smoothstep setpoint interpolation."""
        validated = self._validate_targets(targets)
        try:
            duration = float(duration_s)
        except (TypeError, ValueError) as exc:
            raise ValueError("duration_s must be a finite non-negative number") from exc
        if isinstance(duration_s, bool) or not math.isfinite(duration) or duration < 0:
            raise ValueError("duration_s must be a finite non-negative number")
        start = self.joint_positions()
        steps = max(1, math.ceil(duration / float(self.model.opt.timestep)))
        for index in range(1, steps + 1):
            blend = index / steps
            smooth = blend * blend * (3.0 - 2.0 * blend)
            for name, target in validated.items():
                value = start[name] + (target - start[name]) * smooth
                self.data.ctrl[self._actuator_ids[name]] = value
            self.step()
        # A smooth target ramp alone can finish before the position actuator
        # reaches its final setpoint, especially on the heavier gripper link.
        settle_steps = max(1, math.ceil(max(0.3, duration * 0.5) / float(self.model.opt.timestep)))
        for name, target in validated.items():
            self.data.ctrl[self._actuator_ids[name]] = target
        self.step(settle_steps)
        return {name: self.joint_positions()[name] for name in validated}
