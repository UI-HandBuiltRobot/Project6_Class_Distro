#!/usr/bin/env python3
"""Search Supervisor — Action Server for Wandering While Searching.

YOUR TASK:
    Implement the search behavior for the WSKR robot. This node is an
    action server that the state manager calls when it needs to find
    something — either a toy (via YOLO detections) or the drop box
    (via ArUco marker detections).

    When a goal arrives, the robot should:
      1. Enable the autopilot (so the robot drives toward headings you set)
      2. Wander by publishing heading angles to WSKR/heading_to_target
      3. Monitor sensor streams for the target
      4. Return success when the target is detected, or abort on timeout

HOW MOVEMENT WORKS:
    You do NOT publish cmd_vel directly. Instead:
      - Publish a heading (degrees) to WSKR/heading_to_target
      - Enable the autopilot via WSKR/autopilot/enable (Bool)
    The autopilot MLP converts the heading + whisker readings into safe
    cmd_vel commands that avoid obstacles.

    Positive heading = target is to the right.
    Negative heading = target is to the left.
    Zero heading     = target is straight ahead.

AVAILABLE SENSOR STREAMS (subscribe to these):
    WSKR/floor_mask       (Image)             — binary mask: white=floor, black=obstacle
    WSKR/aruco_markers    (Float32MultiArray)  — detected ArUco markers [id,x0,y0,...x3,y3]
    WSKR/whisker_lengths  (Float32MultiArray)  — 11 virtual whisker distances (mm)
    vision/yolo/detections (ImgDetectionData)  — YOLO object detections with tracking

CONTROL OUTPUTS (publish to these):
    WSKR/heading_to_target  (Float32)  — desired heading in degrees
    WSKR/autopilot/enable   (Bool)     — enable/disable the autopilot

ACTION INTERFACE:
    Action name: WSKR/search_behavior
    Type: WskrSearch (robot_interfaces/action/WskrSearch)

    Goal fields:
        uint8  target_type   — TARGET_TOY (0) or TARGET_BOX (1)
        uint8  target_id     — ArUco marker ID (only used for TARGET_BOX)
        float32 timeout_sec  — max search duration

    Result fields:
        bool success
        ImgDetectionData detected_object  — the detection that was found

    Feedback fields:
        float32 elapsed_sec
        string  current_phase
        int32   detections_sampled

================================================================================
MINI-TUTORIAL: Writing an action server with sensor subscriptions
================================================================================

1) Cache sensor data in subscription callbacks (thread-safe with a lock):

    def _on_whiskers(self, msg):
        with self.lock:
            self.latest_whiskers = np.asarray(msg.data)

2) In your search loop, read the cached data:

    with self.lock:
        whiskers = self.latest_whiskers

    if whiskers is not None:
        left = float(np.mean(whiskers[:5]))
        right = float(np.mean(whiskers[6:]))
        if left < 200.0:
            heading = -30.0   # obstacle on left, steer right
        elif right < 200.0:
            heading = 30.0    # obstacle on right, steer left

3) Publish the heading to make the robot turn:

    msg = Float32()
    msg.data = heading
    self.heading_pub.publish(msg)

4) Check for YOLO detections (toy search):

    with self.lock:
        detections = self.latest_detections

    if detections is not None:
        for conf in detections.confidence:
            if conf >= self.conf_threshold:
                return detections  # found it!

5) Check for ArUco markers (box search):

    with self.lock:
        markers = self.latest_aruco_markers

    if markers is not None and len(markers.data) >= 9:
        for i in range(0, len(markers.data), 9):
            marker_id = int(markers.data[i])
            if marker_id == target_id:
                return marker_id   # found the box!

6) Use time.sleep() for polling (NOT asyncio.sleep — see threading note):

    while time.time() - start_time < timeout_sec:
        # ... check sensors, update heading ...
        time.sleep(0.05)

================================================================================
"""
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Float32MultiArray

from robot_interfaces.action import WskrSearch
from robot_interfaces.msg import ImgDetectionData


IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class SearchBehavior(Node):
    """Wander + search behavior action server."""

    TARGET_TOY = 0
    TARGET_BOX = 1

    def __init__(self) -> None:
        super().__init__('search_supervisor')

        # Thread-safe sensor cache
        self.lock = threading.Lock()
        self.latest_whiskers: Optional[np.ndarray] = None
        self.latest_detections: Optional[ImgDetectionData] = None
        self.latest_floor_mask: Optional[np.ndarray] = None
        self.latest_aruco_markers: Optional[Float32MultiArray] = None

        cb_group = ReentrantCallbackGroup()

        # ── Sensor subscriptions ────────────────────────────────────
        self.create_subscription(
            Image, 'WSKR/floor_mask', self._on_floor_mask,
            IMAGE_QOS, callback_group=cb_group,
        )
        self.create_subscription(
            Float32MultiArray, 'WSKR/aruco_markers', self._on_aruco_markers,
            10, callback_group=cb_group,
        )
        self.create_subscription(
            Float32MultiArray, 'WSKR/whisker_lengths', self._on_whiskers,
            10, callback_group=cb_group,
        )
        self.create_subscription(
            ImgDetectionData, 'vision/yolo/detections', self._on_detections,
            IMAGE_QOS, callback_group=cb_group,
        )

        # ── Control outputs ─────────────────────────────────────────
        self.heading_pub = self.create_publisher(
            Float32, 'WSKR/heading_to_target', 10,
        )
        self.autopilot_enable_pub = self.create_publisher(
            Bool, 'WSKR/autopilot/enable', 10,
        )

        # ── Action server ───────────────────────────────────────────
        self._action_server = ActionServer(
            self,
            WskrSearch,
            'WSKR/search_behavior',
            execute_callback=self._execute_search,
            cancel_callback=self._handle_cancel,
            callback_group=cb_group,
        )

        self.get_logger().info('Search supervisor action server ready.')

    # ================================================================
    #  Sensor callbacks — cache latest data (thread-safe)
    # ================================================================

    def _on_floor_mask(self, msg: Image) -> None:
        mask = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width,
        )
        with self.lock:
            self.latest_floor_mask = mask

    def _on_aruco_markers(self, msg: Float32MultiArray) -> None:
        with self.lock:
            self.latest_aruco_markers = msg

    def _on_whiskers(self, msg: Float32MultiArray) -> None:
        with self.lock:
            self.latest_whiskers = np.asarray(msg.data, dtype=np.float64)

    def _on_detections(self, msg: ImgDetectionData) -> None:
        with self.lock:
            self.latest_detections = msg

    # ================================================================
    #  Helpers — use these to control the robot
    # ================================================================

    def _handle_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Search goal cancellation received.')
        return CancelResponse.ACCEPT

    def _publish_heading(self, heading_deg: float) -> None:
        """Send a heading angle to the autopilot."""
        msg = Float32()
        msg.data = float(heading_deg)
        self.heading_pub.publish(msg)

    def _enable_autopilot(self, enabled: bool) -> None:
        """Turn the autopilot on or off."""
        msg = Bool()
        msg.data = bool(enabled)
        self.autopilot_enable_pub.publish(msg)

    def _stop_robot(self) -> None:
        """Zero heading and disable autopilot."""
        self._publish_heading(0.0)
        self._enable_autopilot(False)

    # ================================================================
    #  Main action callback — implement your search logic here
    # ================================================================

    async def _execute_search(self, goal_handle) -> WskrSearch.Result:
        """Called when the state manager sends a search goal.

        Args:
            goal_handle: contains .request with fields:
                - target_type: TARGET_TOY (0) or TARGET_BOX (1)
                - target_id:   ArUco ID (for box search)
                - timeout_sec: max search time

        Returns:
            WskrSearch.Result with .success and .detected_object
        """
        goal = goal_handle.request
        target_type = goal.target_type
        target_id = int(goal.target_id)
        timeout_sec = float(goal.timeout_sec) if goal.timeout_sec > 0 else 60.0

        self.get_logger().info(
            f'Search started: target={"TOY" if target_type == self.TARGET_TOY else "BOX"}, '
            f'id={target_id}, timeout={timeout_sec}s'
        )

        # TODO: implement your search logic here.
        #
        # Suggested approach:
        #   1. Enable the autopilot: self._enable_autopilot(True)
        #   2. Loop until timeout:
        #      a. Choose a heading (use whiskers to avoid obstacles)
        #      b. Publish it: self._publish_heading(heading_deg)
        #      c. Check sensor data for your target
        #      d. If found, populate result and call goal_handle.succeed()
        #      e. Sleep briefly: time.sleep(0.05)
        #   3. If timeout, call goal_handle.abort()
        #   4. Always stop the robot in a finally block

        result = WskrSearch.Result()
        result.success = False
        goal_handle.abort()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SearchBehavior()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_robot()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
