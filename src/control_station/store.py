"""Small SQLite store for server-created task records."""

from __future__ import annotations

import os
import sqlite3
import uuid
from contextlib import closing
from datetime import datetime
from datetime import timezone
from pathlib import Path

from control_station.models import TaskStatus


class TaskStore:
    def __init__(self, db_path: str | None = None) -> None:
        default_path = Path(__file__).resolve().parents[2] / "data" / "tasks.sqlite3"
        self.path = Path(db_path or os.environ.get("CONTROL_STATION_DB", default_path))
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with closing(self._connect()) as connection:
            with connection:
                connection.execute(
                    """CREATE TABLE IF NOT EXISTS tasks (
                        id TEXT PRIMARY KEY,
                        destination TEXT NOT NULL,
                        arm_job TEXT NOT NULL,
                        status TEXT NOT NULL,
                        created_at TEXT NOT NULL
                    )"""
                )

    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self.path, timeout=5)
        connection.row_factory = sqlite3.Row
        return connection

    @staticmethod
    def _present(row: sqlite3.Row) -> dict[str, str]:
        return {
            "id": row["id"],
            "destination": row["destination"],
            "arm_job": row["arm_job"],
            "status": row["status"],
            "created_at": row["created_at"],
        }

    def list_tasks(self) -> list[dict[str, str]]:
        with closing(self._connect()) as connection:
            rows = connection.execute(
                "SELECT * FROM tasks ORDER BY created_at DESC, id DESC"
            ).fetchall()
        return [self._present(row) for row in rows]

    def create_task(self, destination: str, arm_job: str) -> dict[str, str]:
        task = {
            "id": f"TASK-{uuid.uuid4().hex[:8].upper()}",
            "destination": destination,
            "arm_job": arm_job,
            "status": TaskStatus.PENDING.value,
            "created_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        }
        with closing(self._connect()) as connection:
            with connection:
                connection.execute(
                    "INSERT INTO tasks (id, destination, arm_job, status, created_at) "
                    "VALUES (:id, :destination, :arm_job, :status, :created_at)",
                    task,
                )
        return task
