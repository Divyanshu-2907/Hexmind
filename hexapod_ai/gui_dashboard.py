"""Simple OpenCV HUD for HexMind AI."""

from __future__ import annotations

from typing import Dict, Optional, Tuple

import cv2
import numpy as np


class HexapodHUD:
    """Draws a lightweight status overlay on top of camera frames."""

    def __init__(self, window_name: str = "HexMind AI") -> None:
        self.window_name = window_name
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    def draw(
        self,
        frame: np.ndarray,
        state: str,
        command: str,
        motion: Tuple[float, float, float],
        target_label: Optional[str],
        target_center_x: Optional[float],
        obstacle_label: Optional[str],
        fps: float,
        serial_ok: bool,
    ) -> np.ndarray:
        """Return frame with HUD annotations."""
        output = frame.copy()
        h, w = output.shape[:2]

        panel_h = 145
        overlay = output.copy()
        cv2.rectangle(overlay, (0, 0), (w, panel_h), (0, 0, 0), -1)
        output = cv2.addWeighted(overlay, 0.45, output, 0.55, 0)

        fwd, strafe, rot = motion
        serial_text = "CONNECTED" if serial_ok else "DISCONNECTED"
        serial_color = (0, 220, 0) if serial_ok else (0, 0, 255)

        rows = [
            f"State: {state}",
            f"Command: {command}",
            f"Motion [fwd, strafe, rot]: [{fwd:.2f}, {strafe:.2f}, {rot:.2f}]",
            f"Target: {target_label or '-'} @ x={target_center_x:.1f}" if target_center_x is not None else "Target: -",
            f"Obstacle: {obstacle_label or '-'}",
            f"FPS: {fps:.1f}",
        ]

        y = 24
        for line in rows:
            cv2.putText(output, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (240, 240, 240), 1, cv2.LINE_AA)
            y += 22

        cv2.putText(output, f"Serial: {serial_text}", (w - 260, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, serial_color, 2, cv2.LINE_AA)
        cv2.line(output, (w // 2, panel_h), (w // 2, h), (40, 180, 255), 1)
        return output

    def show(self, frame: np.ndarray) -> None:
        cv2.imshow(self.window_name, frame)

    def close(self) -> None:
        cv2.destroyWindow(self.window_name)
