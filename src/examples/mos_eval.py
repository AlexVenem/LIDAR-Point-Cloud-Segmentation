from __future__ import annotations

import argparse
import csv

import numpy as np

from src.core.metrics import compute_binary_metrics
from src.motion_segmentation import (
    MOVING_LABEL,
    STATIC_LABEL,
    MotionSegmenter,
    iter_helimos_labeled,
)


def evaluate_mos(
    data_root: str,
    model_path: str,
    sensor: str,
    split: str,
    output_path: str,
    threshold: float = 0.85,
    max_frames: int | None = None,
) -> None:
    segmenter = MotionSegmenter(threshold=threshold)
    segmenter.load(model_path, override_threshold=True)

    rows = []

    total_tp = 0
    total_fp = 0
    total_fn = 0
    total_tn = 0

    for frame_id, pc, semantic in iter_helimos_labeled(
        data_root=data_root,
        sensor=sensor,
        split=split,
        max_frames=max_frames,
    ):
        valid_mask = (semantic == STATIC_LABEL) | (semantic == MOVING_LABEL)

        if not np.any(valid_mask):
            continue

        y_true = semantic[valid_mask] == MOVING_LABEL

        y_pred_full, _ = segmenter.segment_frame(pc)
        y_pred = y_pred_full[valid_mask]

        metrics = compute_binary_metrics(y_true, y_pred)

        total_tp += metrics.tp
        total_fp += metrics.fp
        total_fn += metrics.fn
        total_tn += metrics.tn

        rows.append({
            "frame_id": frame_id,
            "n_points": metrics.n_points,
            "accuracy": metrics.accuracy,
            "precision": metrics.precision,
            "recall": metrics.recall,
            "f1_score": metrics.f1_score,
            "tp": metrics.tp,
            "fp": metrics.fp,
            "fn": metrics.fn,
            "tn": metrics.tn,
        })

    total_true = np.array(
        [1] * total_tp + [1] * total_fn + [0] * total_fp + [0] * total_tn,
        dtype=bool,
    )
    total_pred = np.array(
        [1] * total_tp + [0] * total_fn + [1] * total_fp + [0] * total_tn,
        dtype=bool,
    )

    total_metrics = compute_binary_metrics(total_true, total_pred)

    rows.append({
        "frame_id": "TOTAL",
        "n_points": total_metrics.n_points,
        "accuracy": total_metrics.accuracy,
        "precision": total_metrics.precision,
        "recall": total_metrics.recall,
        "f1_score": total_metrics.f1_score,
        "tp": total_metrics.tp,
        "fp": total_metrics.fp,
        "fn": total_metrics.fn,
        "tn": total_metrics.tn,
    })

    with open(output_path, "w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "frame_id",
                "n_points",
                "accuracy",
                "precision",
                "recall",
                "f1_score",
                "tp",
                "fp",
                "fn",
                "tn",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)

    print(f"Saved MOS evaluation table to: {output_path}")
    print(
        f"TOTAL: "
        f"accuracy={total_metrics.accuracy:.4f}, "
        f"precision={total_metrics.precision:.4f}, "
        f"recall={total_metrics.recall:.4f}, "
        f"f1={total_metrics.f1_score:.4f}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Evaluate moving/static point classification on HeLiMOS."
    )
    parser.add_argument("--sequence", required=True)
    parser.add_argument("--model", default="models/mos_rf.pkl")
    parser.add_argument("--sensor", default="Velodyne")
    parser.add_argument("--split", default="val", choices=["train", "val", "test"])
    parser.add_argument("--threshold", type=float, default=0.85)
    parser.add_argument("--output", default="output/mos_metrics.csv")
    parser.add_argument("--max-frames", type=int, default=None)

    args = parser.parse_args()

    evaluate_mos(
        data_root=args.sequence,
        model_path=args.model,
        sensor=args.sensor,
        split=args.split,
        output_path=args.output,
        max_frames=args.max_frames,
        threshold=args.threshold,
    )


if __name__ == "__main__":
    main()