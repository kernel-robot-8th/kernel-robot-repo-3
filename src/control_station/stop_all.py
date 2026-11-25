"""One request that asks everything that moves to stop: the arms and the Pinkys (the big red button of the screen).

A software stop request, not an emergency stop. It goes through the same paths as the single stops and takes over
their meaning: the Pinky fleet dashboard's stop (a stop latched on the robot, its Nav2 goal canceled), the arm
command's cancel flag (the worker checks it about once a second while moving), the arm sequence's cancel and the
policy run's interrupt. Every answer says that the request was taken, never that the device stands still.

Every part is asked even when another one fails; what failed is listed in `errors`. Nothing is latched here: an order
that is still running may send its next command, so the operator handles the stopped order afterwards.
"""

from __future__ import annotations

from typing import Any

SEQUENCE_ACTIVE = frozenset({"QUEUED", "DISPATCHED", "RUNNING"})


def stop_everything(app: Any) -> dict[str, Any]:
    """Ask the arms and the Pinkys to stop. `app` is the server object holding arm_sequences, arm_commands,
    policy_run_supervisor and pinky_gateway; a part that is not there is skipped.

    Returns {"ok", "pinkies", "arm_sequence", "arm_commands", "policy_run", "errors"}: ok is True when every Pinky
    request was accepted and no part raised (in mock mode no Pinky request is sent, so ok is False there).
    """
    result: dict[str, Any] = {"ok": False, "pinkies": [], "arm_sequence": None, "arm_commands": [],
                              "policy_run": None, "errors": []}

    def attempt(part: str, call: Any) -> Any:
        try:
            return call()
        except Exception as exc:  # noqa: BLE001 - one part failing must not keep the others from being asked
            result["errors"].append({"part": part, "message": str(exc) or type(exc).__name__})
            return None

    # The arms first: these are local and take milliseconds, so they do not hold back the Pinky requests.
    sequences = getattr(app, "arm_sequences", None)
    if sequences is not None:
        def cancel_sequence() -> dict[str, Any] | None:
            latest = sequences.latest()
            if latest is None or latest.get("state") not in SEQUENCE_ACTIVE:
                return None
            # Before its command, so the sequence does not queue its next step when that command ends.
            return sequences.cancel(latest["sequence_run_id"])

        result["arm_sequence"] = attempt("arm_sequence", cancel_sequence)

    commands = getattr(app, "arm_commands", None)
    if commands is not None:
        result["arm_commands"] = attempt("arm_commands", commands.cancel_active) or []

    policy = getattr(app, "policy_run_supervisor", None)
    if policy is not None:
        def stop_policy() -> dict[str, Any] | None:
            return policy.stop() if policy.snapshot().get("process_alive") else None

        result["policy_run"] = attempt("policy_run", stop_policy)

    gateway = getattr(app, "pinky_gateway", None)
    if gateway is not None:
        result["pinkies"] = attempt("pinkies", gateway.stop_all) or []

    result["ok"] = not result["errors"] and bool(result["pinkies"]) and all(entry["ok"] for entry in result["pinkies"])
    return result
