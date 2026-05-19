"""Main AI loop for HexMind: YOLO detection -> state machine -> serial command."""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

import cv2
import serial
from ultralytics import YOLO

from gui_dashboard import HexapodHUD


PICK_TARGETS = {"bottle", "cup", "sports ball", "apple", "orange"}
OBSTACLES = {"person", "chair", "dining table", "backpack", "suitcase"}


class BrainState(str, Enum):
    SEARCHING = "SEARCHING"
    APPROACH = "APPROACH"
    PICK = "PICK"
    AVOID = "AVOID"


@dataclass
class DetectionInfo:
    target_label: Optional[str] = None
    target_center_x: Optional[float] = None
    target_area_ratio: float = 0.0
    obstacle_label: Optional[str] = None
    obstacle_center_x: Optional[float] = None
    obstacle_area_ratio: float = 0.0


class HexMindBrain:
    def __init__(
        self,
        stream_source: str | int,
        serial_port: str,
        baud: int = 115200,
        model_paths: list[str] = None,
        conf_threshold: float = 0.35,
        show_hud: bool = True,
        dry_run: bool = False,
    ) -> None:
        self.stream_source = stream_source
        self.serial_port = serial_port
        self.baud = baud
        self.model_paths = model_paths or ["models/yolov8n.pt"]
        self.conf_threshold = conf_threshold
        self.show_hud = show_hud
        self.dry_run = dry_run

        self.models = [YOLO(path) for path in self.model_paths]
        self.ser = None if self.dry_run else serial.Serial(self.serial_port, self.baud, timeout=0.1)
        self.cap = cv2.VideoCapture(self.stream_source)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open stream source: {self.stream_source}")

        self.state: BrainState = BrainState.SEARCHING
        self.avoid_until = 0.0
        self.pick_cooldown_until = 0.0
        self.last_cmd = "STOP"
        self.last_motion: Tuple[float, float, float] = (0.0, 0.0, 0.0)

        self.hud = HexapodHUD() if self.show_hud else None
        self.last_frame_time = time.perf_counter()
        self.last_fps = 0.0

    @staticmethod
    def _clamp(value: float, min_v: float = -1.0, max_v: float = 1.0) -> float:
        return max(min_v, min(max_v, value))

    def _send_serial(self, fwd: float, strafe: float, rot: float, cmd: str) -> None:
        # Protocol required by Mega: "{fwd:.2f},{strafe:.2f},{rot:.2f},{CMD}\n"
        message = f"{fwd:.2f},{strafe:.2f},{rot:.2f},{cmd}\n"
        if self.ser is not None:
            self.ser.write(message.encode("utf-8"))
        self.last_cmd = cmd
        self.last_motion = (fwd, strafe, rot)

    def _pick_best_detections(self, frame_w: int, frame_h: int, results_list: list) -> DetectionInfo:
        info = DetectionInfo()
        best_target_conf = -1.0
        best_obstacle_conf = -1.0
        frame_area = float(frame_w * frame_h)

        for result in results_list:
            boxes = result.boxes
            if boxes is None:
                continue

            names = result.names
            for box in boxes:
                conf = float(box.conf[0].item())
                if conf < self.conf_threshold:
                    continue

                cls_id = int(box.cls[0].item())
                label = names.get(cls_id, str(cls_id)).lower()
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                box_w = max(1.0, x2 - x1)
                box_h = max(1.0, y2 - y1)
                area_ratio = (box_w * box_h) / frame_area
                cx = (x1 + x2) / 2.0

                if label in PICK_TARGETS and conf > best_target_conf:
                    best_target_conf = conf
                    info.target_label = label
                    info.target_center_x = cx
                    info.target_area_ratio = area_ratio

                if label in OBSTACLES and conf > best_obstacle_conf:
                    best_obstacle_conf = conf
                    info.obstacle_label = label
                    info.obstacle_center_x = cx
                    info.obstacle_area_ratio = area_ratio

        return info

    def _state_machine(self, detections: DetectionInfo, frame_w: int) -> Tuple[float, float, float, str]:
        now = time.time()
        fwd, strafe, rot, cmd = 0.0, 0.0, 0.0, "STOP"

        obstacle_too_close = detections.obstacle_area_ratio > 0.16
        target_close_enough_for_pick = detections.target_area_ratio > 0.22

        if self.state != BrainState.AVOID and obstacle_too_close:
            self.state = BrainState.AVOID
            self.avoid_until = now + 1.5

        if self.state == BrainState.AVOID:
            if now < self.avoid_until:
                cmd = "AVOID"
                obstacle_x = detections.obstacle_center_x if detections.obstacle_center_x is not None else frame_w / 2.0
                strafe = -0.45 if obstacle_x > (frame_w / 2.0) else 0.45
                return fwd, strafe, rot, cmd
            self.state = BrainState.SEARCHING

        if self.state == BrainState.PICK:
            if now < self.pick_cooldown_until:
                return 0.0, 0.0, 0.0, "STOP"
            self.state = BrainState.SEARCHING

        if detections.target_center_x is None:
            self.state = BrainState.SEARCHING
        elif target_close_enough_for_pick:
            self.state = BrainState.PICK
        else:
            self.state = BrainState.APPROACH

        if self.state == BrainState.SEARCHING:
            return 0.0, 0.0, 0.30, "WALK"

        if self.state == BrainState.APPROACH and detections.target_center_x is not None:
            err_x = (detections.target_center_x - (frame_w / 2.0)) / (frame_w / 2.0)
            rot = self._clamp(err_x * 0.65)
            fwd = self._clamp(0.40 - abs(err_x) * 0.18, 0.18, 0.45)
            return fwd, 0.0, rot, "WALK"

        if self.state == BrainState.PICK:
            self.pick_cooldown_until = now + 3.0
            return 0.0, 0.0, 0.0, "PICK"

        return fwd, strafe, rot, cmd

    def run(self) -> None:
        try:
            while True:
                ok, frame = self.cap.read()
                if not ok or frame is None:
                    self._send_serial(0.0, 0.0, 0.0, "STOP")
                    time.sleep(0.05)
                    continue

                frame_h, frame_w = frame.shape[:2]
                results_list = []
                annotated = frame.copy()
                for model in self.models:
                    results = model.predict(
                        source=frame,
                        imgsz=320,
                        conf=self.conf_threshold,
                        verbose=False,
                        device="cpu",
                    )
                    res = results[0]
                    results_list.append(res)
                    annotated = res.plot(img=annotated)

                detections = self._pick_best_detections(frame_w, frame_h, results_list)
                fwd, strafe, rot, cmd = self._state_machine(detections, frame_w)
                self._send_serial(fwd, strafe, rot, cmd)

                now = time.perf_counter()
                dt = max(1e-6, now - self.last_frame_time)
                self.last_fps = 1.0 / dt
                self.last_frame_time = now
                if self.hud is not None:
                    annotated = self.hud.draw(
                        frame=annotated,
                        state=self.state.value,
                        command=self.last_cmd,
                        motion=self.last_motion,
                        target_label=detections.target_label,
                        target_center_x=detections.target_center_x,
                        obstacle_label=detections.obstacle_label,
                        fps=self.last_fps,
                        serial_ok=(self.ser.is_open if self.ser is not None else True),
                    )
                    self.hud.show(annotated)
                else:
                    cv2.imshow("HexMind AI", annotated)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        try:
            if self.ser is not None and self.ser.is_open:
                self._send_serial(0.0, 0.0, 0.0, "STOP")
                self.ser.close()
        except Exception:
            pass

        if self.cap.isOpened():
            self.cap.release()
        if self.hud is not None:
            self.hud.close()
        cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="HexMind AI main loop")
    parser.add_argument("--stream", help="ESP32-CAM MJPEG URL, e.g. http://192.168.1.10:81/stream")
    parser.add_argument("--webcam", type=int, help="Laptop webcam index, usually 0")
    parser.add_argument("--port", help="Arduino Mega serial port, e.g. COM4")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--models", nargs="+", default=["models/yolov8n.pt"], help="One or more YOLO model paths")
    parser.add_argument("--conf", type=float, default=0.35, help="Detection confidence threshold")
    parser.add_argument("--no-hud", action="store_true", help="Disable HUD rendering")
    parser.add_argument("--dry-run", action="store_true", help="Run vision/state-machine only, without serial")
    args = parser.parse_args()
    if args.stream is None and args.webcam is None:
        parser.error("Provide one video source: --stream <url> or --webcam <index>.")
    if args.stream is not None and args.webcam is not None:
        parser.error("Use only one video source: --stream OR --webcam.")
    if not args.dry_run and not args.port:
        parser.error("Provide --port unless using --dry-run.")
    return args


def main() -> None:
    args = parse_args()
    stream_source: str | int = args.webcam if args.webcam is not None else args.stream
    brain = HexMindBrain(
        stream_source=stream_source,
        serial_port=(args.port or ""),
        baud=args.baud,
        model_paths=args.models,
        conf_threshold=args.conf,
        show_hud=not args.no_hud,
        dry_run=args.dry_run,
    )
    brain.run()


if __name__ == "__main__":
    main()
