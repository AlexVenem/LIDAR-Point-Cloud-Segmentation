from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class ClassificationMetrics:
    accuracy: float
    precision: float
    recall: float
    f1_score: float
    tp: int
    fp: int
    fn: int
    tn: int
    n_points: int


def compute_binary_metrics(
    y_true: np.ndarray,
    y_pred: np.ndarray,
) -> ClassificationMetrics:
    """Compute binary classification metrics for moving/static point labels.

    Positive class: moving.
    Negative class: static.
    """
    y_true = y_true.astype(bool)
    y_pred = y_pred.astype(bool)

    tp = int(np.sum(y_true & y_pred))
    fp = int(np.sum(~y_true & y_pred))
    fn = int(np.sum(y_true & ~y_pred))
    tn = int(np.sum(~y_true & ~y_pred))

    total = tp + fp + fn + tn

    accuracy = (tp + tn) / total if total > 0 else 0.0
    precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
    recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
    f1_score = (
        2 * precision * recall / (precision + recall)
        if (precision + recall) > 0
        else 0.0
    )

    return ClassificationMetrics(
        accuracy=accuracy,
        precision=precision,
        recall=recall,
        f1_score=f1_score,
        tp=tp,
        fp=fp,
        fn=fn,
        tn=tn,
        n_points=total,
    )