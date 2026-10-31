"""Hash the checked-in MuJoCo OMX scene, model, and mesh source files."""

from __future__ import annotations

import hashlib
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = REPO_ROOT / "simulation" / "mujoco_omx" / "robotis_omx" / "shelf_pick_scene.xml"


def model_source_artifacts() -> list[dict[str, object]]:
    """Return stable source-relative SHA-256 descriptors for the loaded model inputs."""
    model_directory = SCENE_PATH.parent
    sources = [SCENE_PATH, model_directory / "omx.xml"]
    assets_directory = model_directory / "assets"
    sources.extend(path for path in assets_directory.rglob("*") if path.is_file())
    artifacts: list[dict[str, object]] = []
    for path in sorted(sources, key=lambda item: item.relative_to(REPO_ROOT).as_posix()):
        digest = hashlib.sha256()
        with path.open("rb") as source:
            for block in iter(lambda: source.read(1024 * 1024), b""):
                digest.update(block)
        artifacts.append(
            {
                "path": path.relative_to(REPO_ROOT).as_posix(),
                "path_base": "REPOSITORY_ROOT",
                "hash_algorithm": "sha256",
                "sha256": digest.hexdigest(),
                "size_bytes": path.stat().st_size,
            }
        )
    return artifacts
