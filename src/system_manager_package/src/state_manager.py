#!/usr/bin/env python3
"""State Manager — Finite-State Machine for the Robot Collection Pipeline.

YOUR TASK:
    Implement a finite-state machine (FSM) that orchestrates the robot's
    collection behavior. The robot must: search for toys, select one,
    approach it, grasp it, find the drop box, approach the box, and drop
    the toy. Then repeat.

HOW IT WORKS:
    This node subscribes to /robot_command (std_msgs/String) for external
    commands (e.g. from the Foxglove UI) and publishes the current state
    on /robot_state (std_msgs/String).

    Each state has a handler that calls a ROS action or service, then
    transitions to the next state based on the result. The FSM is
    event-driven: you send a goal, attach a callback, and transition
    inside that callback.

AVAILABLE ROS INTERFACES:
    Actions (long-running, cancellable goals):
        WSKR/search_behavior  (WskrSearch)       — wander + detect
        WSKR/approach_object  (ApproachObject)    — drive toward a target
        xarm_grasp_action     (XArm)              — pick up an object

    Services (quick request/response):
        select_object_service  (SelectObject)     — pick best YOLO detection
        open_gripper_service   (Trigger)           — open the gripper

    Topics:
        /robot_command  (String)  — subscribe: incoming commands
        /robot_state    (String)  — publish: current state name
        WSKR/stop       (Empty)   — publish: emergency stop signal

SUGGESTED STATES (you may rename or add your own):
    IDLE, SEARCH, SELECT, APPROACH_OBJ, GRASP, FIND_BOX, APPROACH_BOX,
    DROP, STOPPED, ERROR

SUGGESTED FLOW:
    IDLE → SEARCH → SELECT → APPROACH_OBJ → GRASP → FIND_BOX →
    APPROACH_BOX → DROP → (back to SEARCH)

================================================================================
MINI-TUTORIAL: FSM patterns in ROS 2
================================================================================

1) Define states with an Enum:

    class RobotState(Enum):
        IDLE = 0
        SEARCH = 1
        MY_CUSTOM_STATE = 2

2) Transition between states:

    def _transition(self, new_state: RobotState):
        self._state = new_state
        self.get_logger().info(f'STATE -> {new_state.name}')
        # Publish state so the UI can display it
        msg = String()
        msg.data = new_state.name
        self._state_pub.publish(msg)
        # Dispatch the handler for this state
        if new_state == RobotState.SEARCH:
            self._do_search()

3) Send an action goal and handle the result:

    def _do_search(self):
        goal = WskrSearch.Goal()
        goal.target_type = WskrSearch.Goal.TARGET_TOY
        goal.timeout_sec = 60.0
        future = self._search_ac.send_goal_async(goal)
        future.add_done_callback(self._on_search_accepted)

    def _on_search_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._transition(RobotState.ERROR)
            return
        goal_handle.get_result_async().add_done_callback(self._on_search_result)

    def _on_search_result(self, future):
        result = future.result().result
        if result.success:
            self._transition(RobotState.SELECT)   # found something!
        else:
            self._transition(RobotState.SEARCH)   # try again

4) Call a service:

    def _do_select(self):
        request = SelectObject.Request()
        future = self._select_cli.call_async(request)
        future.add_done_callback(self._on_select_result)

    def _on_select_result(self, future):
        response = future.result()
        if response.success:
            self.selected_object = response.selected_obj
            self._transition(RobotState.APPROACH_OBJ)
        else:
            self._transition(RobotState.SEARCH)   # nothing to select

5) Handle /robot_command to let the UI drive the FSM:

    def _on_command(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'search':
            self._transition(RobotState.SEARCH)
        elif cmd == 'stop':
            self._transition(RobotState.STOPPED)

================================================================================
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from enum import Enum
import threading
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from robot_interfaces.srv import SelectObject
from robot_interfaces.action import ApproachObject, XArm, WskrSearch


# ---------------------------------------------------------------------------
# Define your states here
# ---------------------------------------------------------------------------
class RobotState(Enum):
    IDLE = 0
    # TODO: add your states


class StateManagerNode(Node):
    def __init__(self):
        super().__init__('state_manager_node')

        self._state = RobotState.IDLE
        self._lock = threading.Lock()
        self.selected_object = None

        # ── ROS interfaces ──────────────────────────────────────────
        # Publisher: broadcast current state name
        self._state_pub = self.create_publisher(String, 'robot_state', 10)
        self._stop_pub = self.create_publisher(Empty, 'WSKR/stop', 1)

        # Subscriber: receive commands from UI
        self.create_subscription(String, 'robot_command', self._on_command, 10)

        # Service clients
        self._select_cli = self.create_client(SelectObject, 'select_object_service')
        self._gripper_cli = self.create_client(Trigger, 'open_gripper_service')

        # Action clients
        self._search_ac = ActionClient(self, WskrSearch, 'WSKR/search_behavior')
        self._approach_ac = ActionClient(self, ApproachObject, 'WSKR/approach_object')
        self._grasp_ac = ActionClient(self, XArm, 'xarm_grasp_action')

        self.get_logger().info(
            'State manager ready in IDLE. Publish to /robot_command to begin.'
        )

    # ================================================================
    #  FSM core — implement your transition logic
    # ================================================================

    def _transition(self, new_state: RobotState):
        """Move to a new state, publish it, and dispatch its handler."""
        # TODO: implement
        pass

    def _on_command(self, msg: String):
        """Handle incoming commands from /robot_command."""
        # TODO: implement
        pass

    # ================================================================
    #  State handlers — implement one per state
    # ================================================================

    # TODO: implement your state handlers and their result callbacks.
    # See the mini-tutorial in the module docstring for examples.


# ────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = StateManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
