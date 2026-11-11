"""이미 학습한 모델(예: best11)에 현장 사진을 조금 더 넣어 이어서 학습한다.

원래 학습 사진이 없을 때 다른 클래스를 잊지 않도록, 앞쪽 층(특징 추출)을 고정하고 낮은 학습률로 짧게 학습한다.
데이터셋의 클래스 번호와 이름은 시작 모델과 같아야 한다.

    .venv/bin/python yolo/train_finetune.py --weights .control-station/models/best11_2413f4c.pt \
        --data yolo/dataset_carrot_20261107/data.yaml --name carrot_tissue_20261107
"""

import argparse
from pathlib import Path

import yaml
from ultralytics import YOLO


if __name__ == "__main__":
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--weights", type=Path, required=True, help="시작 모델")
    parser.add_argument("--data", type=Path, required=True, help="data.yaml")
    parser.add_argument("--name", required=True, help="결과 폴더 이름 (yolo/runs/<name>)")
    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--freeze", type=int, default=10, help="고정할 앞쪽 층 수 (YOLO 백본)")
    parser.add_argument("--lr0", type=float, default=0.002)
    args = parser.parse_args()

    model = YOLO(str(args.weights))
    names = yaml.safe_load(args.data.read_text(encoding="utf-8"))["names"]
    if {int(k): v for k, v in names.items()} != dict(model.names):
        raise SystemExit(f"클래스가 시작 모델과 다릅니다: {names} != {model.names}")
    model.train(
        data=str(args.data), epochs=args.epochs, batch=args.batch, imgsz=640, device=0, workers=2,
        project=str(root / "runs"), name=args.name, exist_ok=False,
        freeze=args.freeze, lr0=args.lr0, lrf=0.1, warmup_epochs=1, optimizer="AdamW",
        degrees=10, flipud=0.0, fliplr=0.5, mosaic=1.0, close_mosaic=5,
        seed=0, deterministic=True, plots=True,
    )
    print(f"최적 모델: {model.trainer.best}")
