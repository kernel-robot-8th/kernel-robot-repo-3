#!/usr/bin/env python3
"""학습한 LeRobot ACT 정책을 녹화 데이터셋 에피소드에 오프라인으로 돌려 보고 결과를 그래프·영상으로 저장한다.

실물 팔과 카메라에는 연결하지 않는다. 녹화된 front/wrist 영상과 관절 상태를 정책에 넣고, 정책이 낸 action과
녹화 때 사람이 조작한 action을 나란히 놓는다. 정책 출력이 다음 관측에 반영되지 않는 열린 루프(open-loop) 비교라서,
여기서 잘 맞아도 실물에서 성공한다는 뜻은 아니다. 실물 시험은 scripts/omx_act_policy_test.sh로 따로 한다.

직접 구현한 이유: lerobot-eval은 gym 시뮬레이션 환경이 필요한데, OMX 실물 데이터셋에 맞는 환경이 없다.

  ~/lerobot-venv/bin/python scripts/omx_act_offline_eval.py \\
      --policy JJinJJin2/omx_act_policy_v3 --episodes 0 25 45 --video

결과: --out 폴더에 에피소드마다 ep{번호}.png(관절별 그래프), ep{번호}.csv, (--video) ep{번호}.mp4, 전체 summary.json.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import torch
from safetensors.torch import load_file

from lerobot.configs.policies import PreTrainedConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors

ACTION = "action"
STATE = "observation.state"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--policy", required=True, help="허브 모델 id 또는 pretrained_model 폴더 경로")
    parser.add_argument("--dataset", default="JJinJJin2/pick_and_place_v2", help="데이터셋 repo id")
    parser.add_argument("--root", default=None, help="데이터셋 로컬 폴더 (기본: ~/.cache/huggingface/lerobot/<repo id>)")
    parser.add_argument("--revision", default="main",
                        help="데이터셋 브랜치·태그. LeRobot 기본값은 v3.0 태그인데, 에피소드를 추가로 올리면 태그가 따라오지 않아 main을 기본으로 둠")
    parser.add_argument("--episodes", type=int, nargs="+", default=[0], help="돌려 볼 에피소드 번호")
    parser.add_argument("--no-sync", action="store_true", help="허브와 캐시를 맞추지 않고 로컬 캐시를 그대로 사용")
    parser.add_argument("--video", action="store_true", help="카메라 영상에 녹화/예측 값을 겹친 mp4도 저장")
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--out", type=Path, default=Path.home() / "lerobot-outputs/eval")
    return parser.parse_args()


def model_dir(policy: str) -> Path:
    path = Path(policy).expanduser()
    if path.is_dir():
        return path
    from huggingface_hub import snapshot_download

    return Path(snapshot_download(policy))


def to_bgr(image: torch.Tensor) -> np.ndarray:
    """데이터셋 이미지(C,H,W, 0–1 float) → OpenCV BGR uint8."""
    rgb = (image.clamp(0, 1).permute(1, 2, 0).numpy() * 255).astype(np.uint8)
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


def draw_overlay(frames: list[np.ndarray], names: list[str], recorded: np.ndarray, predicted: np.ndarray,
                 frame_index: int, new_chunk: bool) -> np.ndarray:
    canvas = np.hstack(frames)
    panel = np.full((28 + 24 * len(names), canvas.shape[1], 3), 32, np.uint8)
    cv2.putText(panel, f"frame {frame_index}" + ("  NEW CHUNK" if new_chunk else ""), (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255) if new_chunk else (220, 220, 220), 1)
    for i, name in enumerate(names):
        error = predicted[i] - recorded[i]
        color = (80, 220, 80) if abs(error) < 5 else (60, 160, 255) if abs(error) < 15 else (80, 80, 255)
        text = f"{name:<18} rec {recorded[i]:7.2f}  pred {predicted[i]:7.2f}  err {error:+7.2f}"
        cv2.putText(panel, text, (10, 44 + 24 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)
    return np.vstack([canvas, panel])


def save_plot(path: Path, episode: int, names: list[str], recorded: np.ndarray, predicted: np.ndarray,
              state: np.ndarray, chunk_starts: list[int], fps: int) -> None:
    t = np.arange(len(recorded)) / fps
    fig, axes = plt.subplots(len(names), 1, figsize=(12, 2.2 * len(names)), sharex=True)
    for i, (ax, name) in enumerate(zip(axes, names)):
        ax.plot(t, recorded[:, i], label="recorded action", color="#1f77b4")
        ax.plot(t, predicted[:, i], label="policy action", color="#d62728", linestyle="--")
        ax.plot(t, state[:, i], label="observation.state", color="#7f7f7f", alpha=0.5, linewidth=0.8)
        for start in chunk_starts:
            ax.axvline(start / fps, color="#bbbbbb", linewidth=0.6)
        mae = np.abs(predicted[:, i] - recorded[:, i]).mean()
        ax.set_ylabel(name, fontsize=8)
        ax.set_title(f"{name}  MAE {mae:.2f}", fontsize=9, loc="left")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("time [s]  (vertical lines: new action chunk)")
    fig.suptitle(f"episode {episode}: recorded vs policy (open-loop)")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main() -> int:
    args = parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    pretrained = model_dir(args.policy)
    config = PreTrainedConfig.from_pretrained(pretrained)
    config.device = args.device
    policy = ACTPolicy.from_pretrained(pretrained, config=config)
    policy.eval()
    preprocessor, postprocessor = make_pre_post_processors(
        config,
        pretrained_path=str(pretrained),
        preprocessor_overrides={"device_processor": {"device": args.device}},
    )
    stats = load_file(str(pretrained / "policy_preprocessor_step_3_normalizer_processor.safetensors"))
    trained_frames = int(stats["action.count"].item()) if "action.count" in stats else None

    dataset = LeRobotDataset(args.dataset, root=args.root, episodes=args.episodes, revision=args.revision,
                             force_cache_sync=not args.no_sync)
    names = dataset.meta.features[ACTION]["names"]
    camera_keys = list(config.image_features)
    fps = dataset.meta.fps
    print(f"정책 {args.policy} (학습 통계 프레임 수 {trained_frames})")
    print(f"데이터셋 {args.dataset}: 전체 {dataset.meta.total_episodes}개 에피소드, "
          f"{dataset.meta.total_frames}프레임 / 돌려 볼 에피소드 {args.episodes}")
    print(f"chunk {config.chunk_size}, n_action_steps {config.n_action_steps}, 카메라 {camera_keys}")

    by_episode: dict[int, list[int]] = {}
    for index, episode in enumerate(dataset.hf_dataset["episode_index"]):
        by_episode.setdefault(int(episode), []).append(index)

    summary = {"policy": args.policy, "trained_stat_frames": trained_frames, "dataset": args.dataset,
               "dataset_total_episodes": dataset.meta.total_episodes, "mode": "open-loop", "episodes": {}}
    for episode in args.episodes:
        indices = by_episode.get(episode)
        if not indices:
            print(f"에피소드 {episode}가 데이터셋에 없습니다", file=sys.stderr)
            continue
        policy.reset()
        writer = None
        recorded, predicted, state, chunk_starts = [], [], [], []
        for step, index in enumerate(indices):
            item = dataset[index]
            new_chunk = len(policy._action_queue) == 0
            if new_chunk:
                chunk_starts.append(step)
            batch = {key: item[key] for key in (STATE, *camera_keys)}
            batch["task"] = item["task"]
            with torch.inference_mode():
                action = postprocessor(policy.select_action(preprocessor(batch)))
            pred = action.squeeze(0).float().cpu().numpy()
            rec = item[ACTION].numpy()
            recorded.append(rec)
            predicted.append(pred)
            state.append(item[STATE].numpy())
            if args.video:
                frame = draw_overlay([to_bgr(item[key]) for key in camera_keys], names, rec, pred, step, new_chunk)
                if writer is None:
                    writer = cv2.VideoWriter(str(args.out / f"ep{episode:03d}.mp4"),
                                             cv2.VideoWriter_fourcc(*"mp4v"), fps, (frame.shape[1], frame.shape[0]))
                writer.write(frame)
            if step % 100 == 0:
                print(f"  ep{episode} {step}/{len(indices)}", flush=True)
        if writer is not None:
            writer.release()

        recorded, predicted, state = np.array(recorded), np.array(predicted), np.array(state)
        save_plot(args.out / f"ep{episode:03d}.png", episode, names, recorded, predicted, state, chunk_starts, fps)
        with open(args.out / f"ep{episode:03d}.csv", "w", newline="") as file:
            out = csv.writer(file)
            out.writerow(["frame", *[f"rec_{n}" for n in names], *[f"pred_{n}" for n in names]])
            for step, (rec, pred) in enumerate(zip(recorded, predicted)):
                out.writerow([step, *np.round(rec, 3), *np.round(pred, 3)])
        mae = np.abs(predicted - recorded).mean(axis=0)
        summary["episodes"][episode] = {"frames": len(indices), "mae": dict(zip(names, np.round(mae, 3).tolist())),
                                        "mae_mean": round(float(mae.mean()), 3)}
        print(f"ep{episode}: 관절별 평균 오차 " + ", ".join(f"{n} {m:.2f}" for n, m in zip(names, mae)))

    (args.out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"결과: {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
