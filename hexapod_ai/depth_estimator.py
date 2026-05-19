"""Optional MiDaS depth estimation helper for HexMind AI."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import torch


@dataclass
class DepthEstimate:
    """Container for depth outputs."""

    depth_map: np.ndarray
    normalized_map: np.ndarray


class MiDaSDepthEstimator:
    """Thin wrapper around Intel-ISL MiDaS from torch.hub."""

    def __init__(self, model_type: str = "MiDaS_small", device: Optional[str] = None) -> None:
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.model = torch.hub.load("intel-isl/MiDaS", model_type)
        self.model.to(self.device)
        self.model.eval()

        transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
        if model_type in {"DPT_Large", "DPT_Hybrid"}:
            self.transform = transforms.dpt_transform
        else:
            self.transform = transforms.small_transform

    def infer(self, bgr_frame: np.ndarray) -> DepthEstimate:
        """Predict relative depth from a BGR frame."""
        rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        input_batch = self.transform(rgb).to(self.device)

        with torch.no_grad():
            prediction = self.model(input_batch)
            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=rgb.shape[:2],
                mode="bicubic",
                align_corners=False,
            ).squeeze()

        depth = prediction.cpu().numpy()
        depth_norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        return DepthEstimate(depth_map=depth, normalized_map=depth_norm)

    @staticmethod
    def colorize(normalized_depth: np.ndarray) -> np.ndarray:
        """Convert normalized depth map to a heatmap for visualization."""
        return cv2.applyColorMap(normalized_depth, cv2.COLORMAP_INFERNO)
