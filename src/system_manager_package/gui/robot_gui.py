#!/usr/bin/env python3
"""Robot State GUI — comprehensive debugging and monitoring dashboard.

A tkinter GUI displaying real-time robot state and telemetry:

    Panel 1 — Vision Stream (top-left):
             Live camera feed with bounding boxes around detected/selected objects
             and highlighted ArUco markers matching the search target. Shows:
             - Detected object bounding boxes (color-coded by type)
             - Target ArUco marker highlighted in yellow
             - Server-tracked bbox overlay (while approaching)

    Panel 2 — Consolidated WSKR Overlay (top-right):
             Floor detection, whisker rays, heading meridians, and telemetry
             from the WSKR autopilot (wskr_overlay/compressed).

    Panel 3 — State & Telemetry (bottom, full width):
             - Current robot state (IDLE, SEARCH, SELECT, APPROACH_OBJ, etc.)
             - Fused heading in degrees
             - Whisker fan diagram (length proportional to range, color-coded)
             - Heading arrow indicating current direction
             - cmd_vel autopilot twist (Vx, Vy, ω)
             - Search phase info (for debugging search behavior)

Refreshes at ~15 Hz. Works standalone (no GUI required for robot_bringup.launch).

Topics subscribed to:
    camera1/image_raw/compressed              — camera feed for vision panel
    wskr_overlay/compressed                   — consolidated WSKR telemetry overlay
    WSKR/tracked_bbox                         — bounding box of tracked object
    WSKR/whisker_lengths                      — proximity sensor array (mm)
    WSKR/target_whisker_lengths               — target intercept whisker lengths
    WSKR/heading_to_target                    — current heading setpoint (degrees)
    WSKR/tracking_mode                        — tracking mode string (visual, etc.)
    WSKR/cmd_vel                              — autopilot velocity command
    robot_state                               — robot FSM state (IDLE, SEARCH, etc.)
    WSKR/search_phase                         — search behavior phase (WANDER, LOOK, etc.)
"""
from __future__ import annotations

import math
import threading
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
import tkinter as tk
from PIL import Image, ImageTk
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Float32MultiArray, String


IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

ARUCO_DICT = cv2.aruco.DICT_4X4_50


def fit_to_label(frame: np.ndarray, lw: int, lh: int) -> np.ndarray:
    """Resize frame to fit label while maintaining aspect ratio."""
    lw = max(lw, 1)
    lh = max(lh, 1)
    sh, sw = frame.shape[:2]
    if sw <= 0 or sh <= 0:
        return np.zeros((lh, lw, 3), dtype=np.uint8)
    scale = min(lw / sw, lh / sh)
    new_w = max(int(sw * scale), 1)
    new_h = max(int(sh * scale), 1)
    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((lh, lw, 3), dtype=np.uint8)
    off_x = (lw - new_w) // 2
    off_y = (lh - new_h) // 2
    canvas[off_y:off_y + new_h, off_x:off_x + new_w] = resized
    return canvas


def ensure_bgr(frame: np.ndarray) -> np.ndarray:
    """Ensure frame is in BGR color format."""
    if frame.ndim == 2:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    if frame.shape[2] == 4:
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    return frame


class RobotGUINode(Node):
    """ROS 2 node for robot state and telemetry GUI."""

    def __init__(self) -> None:
        super().__init__('robot_gui')

        self.bridge = CvBridge()

        # ── shared image frames ──────────────────────────────────────────
        self._cam_lock = threading.Lock()
        self._overlay_lock = threading.Lock()
        self._cam_frame: Optional[np.ndarray] = None
        self._overlay_frame: Optional[np.ndarray] = None

        # ── telemetry state ──────────────────────────────────────────────
        self._tracked_bbox: Optional[tuple[float, float, float, float]] = None
        self._whiskers_mm: Optional[np.ndarray] = None
        self._target_whiskers_mm: Optional[np.ndarray] = None
        self._heading_deg: Optional[float] = None
        self._tracking_mode: str = '—'
        self._cmd_vel: Twist = Twist()
        self._robot_state: str = 'IDLE'
        self._search_phase: str = '—'

        # ── ArUco detection ──────────────────────────────────────────────
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        try:
            self._aruco_detector = cv2.aruco.ArucoDetector(self._aruco_dict)
        except AttributeError:
            self._aruco_detector = None
        self.declare_parameter('target_aruco_id', 1)
        self._target_aruco_id = int(self.get_parameter('target_aruco_id').value)

        # ── subscriptions ────────────────────────────────────────────────
        self.create_subscription(
            CompressedImage, 'camera1/image_raw/compressed', self._on_camera, IMAGE_QOS,
        )
        self.create_subscription(
            CompressedImage, 'wskr_overlay/compressed', self._on_overlay, IMAGE_QOS,
        )
        self.create_subscription(
            Float32MultiArray, 'WSKR/tracked_bbox', self._on_tracked_bbox, 10
        )
        self.create_subscription(
            Float32MultiArray, 'WSKR/whisker_lengths', self._on_whiskers, 10
        )
        self.create_subscription(
            Float32MultiArray, 'WSKR/target_whisker_lengths', self._on_target_whiskers, 10
        )
        self.create_subscription(Float32, 'WSKR/heading_to_target', self._on_heading, 10)
        self.create_subscription(String, 'WSKR/tracking_mode', self._on_tracking_mode, 10)
        self.create_subscription(Twist, 'WSKR/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(String, 'robot_state', self._on_robot_state, 10)
        self.create_subscription(String, 'WSKR/search_phase', self._on_search_phase, 10)

        # ── GUI widget refs ──────────────────────────────────────────────
        self.gui_window: Optional[tk.Tk] = None
        self.cam_label: Optional[tk.Label] = None
        self.overlay_label: Optional[tk.Label] = None
        self.numeric_canvas: Optional[tk.Canvas] = None
        self.state_label: Optional[tk.Label] = None
        self._gui_stop = threading.Event()

        self._start_gui()

    # ── ROS callbacks ────────────────────────────────────────────────────

    def _on_camera(self, msg: CompressedImage) -> None:
        frame = cv2.imdecode(
            np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_REDUCED_COLOR_2,
        )
        if frame is None:
            return
        with self._cam_lock:
            self._cam_frame = frame

    def _on_overlay(self, msg: CompressedImage) -> None:
        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        with self._overlay_lock:
            self._overlay_frame = frame

    def _on_tracked_bbox(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 4:
            self._tracked_bbox = None
            return
        self._tracked_bbox = (
            float(msg.data[0]), float(msg.data[1]),
            float(msg.data[2]), float(msg.data[3]),
        )

    def _on_whiskers(self, msg: Float32MultiArray) -> None:
        self._whiskers_mm = np.asarray(msg.data, dtype=np.float64)

    def _on_target_whiskers(self, msg: Float32MultiArray) -> None:
        self._target_whiskers_mm = np.asarray(msg.data, dtype=np.float64)

    def _on_heading(self, msg: Float32) -> None:
        self._heading_deg = float(msg.data)

    def _on_tracking_mode(self, msg: String) -> None:
        self._tracking_mode = msg.data

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_vel = msg

    def _on_robot_state(self, msg: String) -> None:
        self._robot_state = msg.data

    def _on_search_phase(self, msg: String) -> None:
        self._search_phase = msg.data

    # ── GUI construction ─────────────────────────────────────────────────

    def _start_gui(self) -> None:
        threading.Thread(target=self._gui_run, daemon=True).start()

    def _gui_run(self) -> None:
        root = tk.Tk()
        root.title('Robot State GUI — Debugging Dashboard')
        root.geometry('1400x900')
        root.configure(bg='#101010')
        self.gui_window = root

        grid = tk.Frame(root, bg='#101010')
        grid.pack(fill=tk.BOTH, expand=True)
        grid.rowconfigure(0, weight=1, uniform='row')
        grid.rowconfigure(1, weight=1, uniform='row')
        grid.columnconfigure(0, weight=1, uniform='col')
        grid.columnconfigure(1, weight=1, uniform='col')

        # ── Tile 1: Vision Stream ────────────────────────────────────────
        vision_tile = tk.Frame(grid, bg='#101010', bd=1, relief=tk.FLAT)
        vision_tile.grid(row=0, column=0, sticky='nsew', padx=4, pady=4)

        tk.Label(
            vision_tile, text='Vision Stream (Camera Feed)',
            bg='#202020', fg='#d0d0d0', font=('Arial', 11, 'bold'), anchor='w', padx=6,
        ).pack(side=tk.TOP, fill=tk.X)

        self.cam_label = tk.Label(vision_tile, bg='black')
        self.cam_label.pack(fill=tk.BOTH, expand=True)

        # ── Tile 2: Consolidated WSKR overlay ────────────────────────────
        overlay_tile = tk.Frame(grid, bg='#101010', bd=1, relief=tk.FLAT)
        overlay_tile.grid(row=0, column=1, sticky='nsew', padx=4, pady=4)

        tk.Label(
            overlay_tile, text='WSKR Overlay (Whiskers & Heading)',
            bg='#202020', fg='#d0d0d0', font=('Arial', 11, 'bold'), anchor='w', padx=6,
        ).pack(side=tk.TOP, fill=tk.X)

        self.overlay_label = tk.Label(overlay_tile, bg='black')
        self.overlay_label.pack(fill=tk.BOTH, expand=True)

        # ── Tile 3: State & Telemetry (spans full bottom row) ─────────────
        stats_tile = tk.Frame(grid, bg='#101010')
        stats_tile.grid(row=1, column=0, columnspan=2, sticky='nsew', padx=4, pady=4)

        tk.Label(
            stats_tile, text='Robot State & Telemetry',
            bg='#202020', fg='#d0d0d0', font=('Arial', 11, 'bold'), anchor='w', padx=6,
        ).pack(side=tk.TOP, fill=tk.X)

        # State display bar
        state_bar = tk.Frame(stats_tile, bg='#1e1e1e', pady=4)
        state_bar.pack(side=tk.TOP, fill=tk.X)

        tk.Label(
            state_bar, text='State:', bg='#1e1e1e', fg='white',
            font=('Arial', 11, 'bold'),
        ).pack(side=tk.LEFT, padx=(10, 6))

        self.state_label = tk.Label(
            state_bar, text='IDLE', bg='#1e1e1e', fg='#ffcc00',
            font=('Consolas', 14, 'bold'), anchor='w', width=20,
        )
        self.state_label.pack(side=tk.LEFT, padx=6)

        tk.Label(
            state_bar, text='Search Phase:', bg='#1e1e1e', fg='white',
            font=('Arial', 11, 'bold'),
        ).pack(side=tk.LEFT, padx=(20, 6))

        self.search_phase_label = tk.Label(
            state_bar, text='—', bg='#1e1e1e', fg='#80d0ff',
            font=('Consolas', 12), anchor='w', width=15,
        )
        self.search_phase_label.pack(side=tk.LEFT, padx=6)

        self.numeric_canvas = tk.Canvas(stats_tile, bg='#181818', highlightthickness=0)
        self.numeric_canvas.pack(fill=tk.BOTH, expand=True)

        # ── refresh loop ──────────────────────────────────────────────────
        def refresh() -> None:
            if self._gui_stop.is_set():
                if rclpy.is_initialized():
                    rclpy.shutdown()
                root.destroy()
                return
            self._render_camera_tile()
            self._render_overlay_tile()
            self._render_numeric_tile()
            self._update_state_label()
            root.after(66, refresh)  # ~15 Hz

        root.after(66, refresh)

        def on_close() -> None:
            self._gui_stop.set()

        root.protocol('WM_DELETE_WINDOW', on_close)
        root.mainloop()

    # ── render helpers ────────────────────────────────────────────────────

    def _render_image_label(self, label: tk.Label, frame: Optional[np.ndarray]) -> None:
        lw = label.winfo_width()
        lh = label.winfo_height()
        if frame is None:
            canvas = np.zeros((max(lh, 1), max(lw, 1), 3), dtype=np.uint8)
        else:
            canvas = fit_to_label(frame, lw, lh)
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        photo = ImageTk.PhotoImage(Image.fromarray(rgb))
        label.config(image=photo)
        label.image = photo  # keep reference

    def _render_camera_tile(self) -> None:
        if self.cam_label is None:
            return
        with self._cam_lock:
            frame = None if self._cam_frame is None else self._cam_frame.copy()

        if frame is not None:
            # Detect all ArUco markers
            if self._aruco_detector is not None:
                corners, ids, _ = self._aruco_detector.detectMarkers(frame)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(frame, self._aruco_dict)
            
            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    corner = corners[i][0].astype(int)
                    # Highlight target ArUco in yellow, others in grey
                    if int(marker_id) == self._target_aruco_id:
                        cv2.polylines(frame, [corner], True, (0, 255, 255), 3)
                        cx, cy = corner.mean(axis=0).astype(int)
                        cv2.putText(
                            frame, f'ID:{marker_id}', (cx - 20, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA,
                        )
                    else:
                        cv2.polylines(frame, [corner], True, (128, 128, 128), 2)

            # Draw server-tracked bbox if active
            if self._tracked_bbox is not None:
                h, w = frame.shape[:2]
                xn, yn, wn, hn = self._tracked_bbox
                x1 = int(xn * w)
                y1 = int(yn * h)
                x2 = int((xn + wn) * w)
                y2 = int((yn + hn) * h)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
                cv2.putText(
                    frame, 'TRACKING', (x1, max(0, y1 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA,
                )

        self._render_image_label(self.cam_label, frame)

    def _render_overlay_tile(self) -> None:
        if self.overlay_label is None:
            return
        with self._overlay_lock:
            frame = None if self._overlay_frame is None else self._overlay_frame.copy()
        # Paint the latest tracked bbox here so freshness is bounded
        if frame is not None and self._tracked_bbox is not None:
            scale = float(frame.shape[1])
            xn, yn, wn, hn = self._tracked_bbox
            x1 = int(xn * scale)
            y1 = int(yn * scale)
            x2 = int((xn + wn) * scale)
            y2 = int((yn + hn) * scale)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.putText(
                frame, 'TARGET', (x1, max(0, y1 - 4)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA,
            )
        self._render_image_label(self.overlay_label, frame)

    def _update_state_label(self) -> None:
        if self.state_label is None or self.search_phase_label is None:
            return
        state = self._robot_state
        # Color code state by type
        state_colors = {
            'IDLE': '#80ff80',
            'WANDER': '#80d0ff',
            'SEARCH': '#ffaa00',
            'SELECT': '#ffcc00',
            'APPROACH_OBJ': '#ff8080',
            'APPROACH_TOY': '#ff8080',
            'GRASP': '#ff6060',
            'FIND_BOX': '#ffaa00',
            'APPROACH_BOX': '#ff8080',
            'DROP': '#ff6060',
            'STOPPED': '#ff0000',
            'ERROR': '#ff0000',
        }
        color = state_colors.get(state, '#d0d0d0')
        self.state_label.config(text=state, fg=color)
        self.search_phase_label.config(text=self._search_phase)

    @staticmethod
    def _whisker_color(mm: float) -> str:
        """Color-code whisker distance: green (safe) → yellow (caution) → red (danger)."""
        if mm < 150.0:
            return '#ff3333'  # red: danger
        elif mm < 300.0:
            return '#ffaa00'  # yellow: caution
        else:
            return '#33ff33'  # green: safe

    def _draw_robot_diagram(
        self, c: tk.Canvas, x0: float, y0: float, x1: float, y1: float,
    ) -> None:
        """Draw robot schematic with whisker fan, heading arrow, and cmd_vel."""
        area_w = x1 - x0
        area_h = y1 - y0
        if area_w < 60 or area_h < 60:
            return

        cx = (x0 + x1) / 2.0
        body_h = min(area_w * 0.14, 90.0)
        body_w = body_h * 0.75
        wheel_r = max(body_h * 0.11, 3.0)

        robot_bottom = y1 - 10
        robot_top = robot_bottom - body_h
        body_cx = cx
        body_cy = (robot_top + robot_bottom) / 2.0

        ox, oy = cx, robot_top - 20
        max_ray_px = max(20.0, (oy - y0) - 8)

        whiskers = self._whiskers_mm
        target_whiskers = self._target_whiskers_mm
        scale_mm = 500.0

        # Draw whisker rays
        if whiskers is not None and whiskers.size > 0:
            num_whiskers = len(whiskers)
            for i, mm in enumerate(whiskers):
                angle_frac = i / max(1, num_whiskers - 1)
                angle_deg = -90.0 + angle_frac * 180.0
                angle_rad = math.radians(angle_deg)
                ray_len = min(mm / scale_mm * max_ray_px, max_ray_px)
                ex = ox - ray_len * math.sin(angle_rad)
                ey = oy - ray_len * math.cos(angle_rad)
                color = self._whisker_color(mm)
                c.create_line(ox, oy, ex, ey, fill=color, width=2)

        # Draw target whisker intercepts (magenta diamonds)
        if target_whiskers is not None and target_whiskers.size > 0:
            num_targets = len(target_whiskers)
            for i, mm in enumerate(target_whiskers):
                angle_frac = i / max(1, num_targets - 1)
                angle_deg = -90.0 + angle_frac * 180.0
                angle_rad = math.radians(angle_deg)
                ray_len = min(mm / scale_mm * max_ray_px, max_ray_px)
                mx = ox - ray_len * math.sin(angle_rad)
                my = oy - ray_len * math.cos(angle_rad)
                d = 5
                c.create_polygon(
                    mx - d, my, mx, my - d, mx + d, my, mx, my + d,
                    fill='#ff00ff', outline='#ff00ff', width=1,
                )

        # Robot body
        c.create_rectangle(
            cx - body_w / 2, robot_top, cx + body_w / 2, robot_bottom,
            fill='#2a63d6', outline='#1d4ba3', width=1,
        )
        for wx, wy in (
            (cx - body_w / 2, robot_top), (cx + body_w / 2, robot_top),
            (cx - body_w / 2, robot_bottom), (cx + body_w / 2, robot_bottom),
        ):
            c.create_oval(
                wx - wheel_r, wy - wheel_r, wx + wheel_r, wy + wheel_r,
                fill='#1a1a1a', outline='#404040', width=1,
            )

        # Heading arrow (fused heading_to_target)
        heading = self._heading_deg
        if heading is not None:
            hdg_rad = math.radians(heading)
            arrow_len = body_w * 0.4
            ax = body_cx - arrow_len * math.sin(hdg_rad)
            ay = body_cy - arrow_len * math.cos(hdg_rad)
            c.create_line(
                body_cx, body_cy, ax, ay,
                fill='#ff6060', width=3, arrow=tk.LAST, arrowshape=(10, 12, 4),
            )

    def _render_numeric_tile(self) -> None:
        c = self.numeric_canvas
        if c is None:
            return
        c.delete('all')
        w = max(c.winfo_width(), 1)
        h = max(c.winfo_height(), 1)

        heading = self._heading_deg
        heading_txt = '—' if heading is None else f'{heading:+7.2f}°'
        heading_color = '#80ff80' if self._tracking_mode == 'visual' else '#ffaa40'

        y = 16
        c.create_text(14, y, anchor='nw', text='Heading to target',
                      fill='#a0a0a0', font=('Arial', 11))
        c.create_text(14, y + 18, anchor='nw', text=heading_txt,
                      fill=heading_color, font=('Consolas', 28, 'bold'))
        c.create_text(w - 14, y + 18, anchor='ne', 
                      text=f'mode: {self._tracking_mode}',
                      fill='#ffaa40', font=('Consolas', 14, 'bold'))

        y_diag_top = y + 66
        y_diag_bottom = h - 80
        c.create_text(14, y_diag_top - 18, anchor='nw',
                      text='Whisker fan (mm) + heading arrow',
                      fill='#a0a0a0', font=('Arial', 11))
        self._draw_robot_diagram(c, 14, y_diag_top, w - 14, y_diag_bottom)

        vx = self._cmd_vel.linear.x
        vy = self._cmd_vel.linear.y
        wz_deg = math.degrees(self._cmd_vel.angular.z)
        y_cmd = h - 60
        c.create_text(14, y_cmd, anchor='nw', 
                      text='Autopilot cmd_vel (WSKR/cmd_vel)',
                      fill='#a0a0a0', font=('Arial', 11))
        c.create_text(
            14, y_cmd + 20, anchor='nw',
            text=f'Vx={vx:+6.2f} m/s   Vy={vy:+6.2f} m/s   ω={wz_deg:+6.1f}°/s',
            fill='#e0e0e0', font=('Consolas', 13, 'bold'),
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotGUINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.is_initialized():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
