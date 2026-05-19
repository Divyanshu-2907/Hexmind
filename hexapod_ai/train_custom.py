"""Train script for custom YOLOv8n model on local dataset."""

from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Fine-tune YOLOv8n on custom HexMind dataset.")
    parser.add_argument("--data", type=Path, default=Path("dataset/data.yaml"), help="Path to dataset yaml.")
    parser.add_argument("--model", type=Path, default=Path("models/yolov8n.pt"), help="Base YOLOv8n weights path.")
    parser.add_argument("--epochs", type=int, default=50, help="Training epochs.")
    parser.add_argument("--imgsz", type=int, default=320, help="Image size for training.")
    parser.add_argument("--batch", type=int, default=16, help="Batch size.")
    parser.add_argument("--device", type=str, default="cpu", help="Training device: cpu or cuda index.")
    parser.add_argument("--project", type=Path, default=Path("runs/train"), help="Output project directory.")
    parser.add_argument("--name", type=str, default="hexmind_yolov8n", help="Run name.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.data.exists():
        raise FileNotFoundError(f"Dataset yaml not found: {args.data}")
    if not args.model.exists():
        raise FileNotFoundError(f"Base model not found: {args.model}")

    model = YOLO(str(args.model))
    model.train(
        data=str(args.data),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=str(args.project),
        name=args.name,
    )


if __name__ == "__main__":
    main()
