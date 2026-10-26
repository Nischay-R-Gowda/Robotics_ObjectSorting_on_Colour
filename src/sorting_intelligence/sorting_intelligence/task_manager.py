import json
from enum import Enum

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class TaskState(Enum):
    IDLE = 0
    DETECT = 1
    PICK = 2
    GRIP = 3
    CLASSIFY = 4
    PLACE = 5
    RELEASE = 6
    RETURN_HOME = 7
    COMPLETE = 8
    FAILED = 9


class TaskManager(Node):
    """One-object red/blue sorting demo with action-result-gated transitions."""

    PICK_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    GRIPPER_JOINTS = [
        'gripper_left_finger_joint', 'gripper_right_finger_joint'
    ]
    # 70 mm cube; initial finger gap is 78 mm. A 6 mm inward move
    # per finger leaves 66 mm and applies a modest pinch instead of
    # driving the fingers deeply through the object.
    HOME_POSITION = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    GRIP_CLOSED_POSITION = 0.006
    # Exact FK for the finger midpoint at the 70 mm cube's pickup centre.
    PICK_POSITION = [0.0, 0.876951, 1.726030, 0.0, -1.032185, 0.0]
    # Carry the cube well above the 0.17 m bin rim before rotating toward the bin.
    # Intermediate high poses keep the arm from sweeping through the near wall.
    HIGH_PICKUP_POSITION = [0.0, 0.281044, 1.669459, 0.0, -0.379707, 0.0]
    # Both bins are reached from the same calibrated pickup point. Joint1 changes
    # sign to choose the bin side; the arm pose keeps the cube above each rim.
    RED_HIGH_BIN_POSITION = [0.499348, 0.473172, 1.413740, 0.0, -0.316116, 0.0]
    BLUE_HIGH_BIN_POSITION = [-0.499348, 0.473172, 1.413740, 0.0, -0.316116, 0.0]
    RED_PLACE_POSITION = [0.499348, 0.711713, 1.529758, 0.0, -0.670674, 0.0]
    BLUE_PLACE_POSITION = [-0.499348, 0.711713, 1.529758, 0.0, -0.670674, 0.0]
    GREEN_HIGH_BIN_POSITION = [-1.030377, 0.333326, 1.602817, 0.0, -0.365347, 0.0]
    GREEN_PLACE_POSITION = [-1.030377, 0.599175, 1.718735, 0.0, -0.747114, 0.0]
    MIN_CONFIDENCE = 0.65

    def __init__(self):
        super().__init__('task_manager')
        self.state = TaskState.IDLE
        self.detected_object = None
        self.action_busy = False
        self.current_action = None
        self.pending_state = None
        self.failure_message = ''
        self.armed = False       # Require an explicit dashboard start command
        self.action_generation = 0
        self.estopped = False
        self.goal_handle = None
        self.status_publisher = self.create_publisher(String, '/sorting/status', 10)

        self.ack_publisher = self.create_publisher(String, '/sorting/command_ack', 10)
        self.create_subscription(String, '/sorting/command', self.command_callback, 10)
        self.create_subscription(String, '/detected_object', self.detection_callback, 10)
        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory'
        )
        self.place_stage = 0
        self.home_stage = 0
        self.high_bin_position = self.RED_HIGH_BIN_POSITION
        self.place_position = self.RED_PLACE_POSITION
        self.bin_label = 'red'
        self.completion_logged = False
        self.timer = self.create_timer(1.0, self.run_state_machine)
        self.get_logger().info('Task manager ready; waiting for a pickup-region detection')
        self.publish_status()

    def publish_status(self):
        status = {
            'state': self.state.name,
            'object_label': (self.detected_object or {}).get('label'),
            'failure': self.failure_message or None,
            'armed': self.armed,
            'estop': self.estopped,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_publisher.publish(msg)

    def detection_callback(self, msg):
        if (self.state != TaskState.DETECT or self.detected_object is not None
                or not self.armed):
            return
        try:
            detection = json.loads(msg.data)
            label = detection['label']
            x = int(detection['x'])
            y = int(detection['y'])
            confidence = float(detection['confidence'])
            if label not in ('red_object', 'green_object', 'blue_object'):
                raise ValueError(f'unsupported label: {label}')
            if not 0.0 <= confidence <= 1.0:
                raise ValueError('confidence must be between 0 and 1')
            if confidence < self.MIN_CONFIDENCE:
                self.get_logger().warning(
                    f'Ignoring low-confidence detection ({confidence:.3f})'
                )
                return
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
            self.get_logger().warning(f'Ignoring invalid detection: {error}')
            return

        self.detected_object = {**detection, 'label': label, 'x': x, 'y': y, 'confidence': confidence}
        self.publish_status()
        self.get_logger().info(
            f"Accepted {label} at image pixels ({x}, {y}), "
            f"confidence={confidence:.3f}"
        )

    def _fail_action(self, message):
        self.get_logger().error(message)
        self.action_generation += 1  # Ignore late callbacks from this failed cycle
        self.goal_handle = None
        self.failure_message = message
        self.action_busy = False
        self.current_action = None
        self.pending_state = None
        self.state = TaskState.FAILED
        self.publish_status()

    def send_trajectory(self, client, joint_names, positions, duration, action_name, next_state):
        if self.action_busy:
            return False
        if len(joint_names) != len(positions):
            self._fail_action(f'{action_name}: joint/position count mismatch')
            return False
        if not client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warning(f'{action_name} action server is not available yet')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = duration
        goal.trajectory.points.append(point)

        self.action_generation += 1
        generation = self.action_generation
        self.action_busy = True
        self.current_action = action_name
        self.pending_state = next_state
        self.get_logger().info(f'Sending {action_name} trajectory: {positions}')
        future = client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed: self.goal_response_callback(completed, generation, action_name)
        )
        return True

    def goal_response_callback(self, future, generation, action_name):
        try:
            handle = future.result()
        except Exception as error:
            if generation == self.action_generation:
                self._fail_action(f'{action_name} goal submission failed: {error}')
            return
        if not handle.accepted:
            if generation == self.action_generation:
                self._fail_action(f'{action_name} trajectory goal was rejected')
            return
        if generation != self.action_generation or self.estopped:
            handle.cancel_goal_async()
            return
        self.goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda completed: self.goal_result_callback(completed, generation, action_name)
        )

    def goal_result_callback(self, future, generation, action_name):
        if generation != self.action_generation or self.state == TaskState.FAILED:
            return  # stale result after a stop, failure, or reset
        try:
            result = future.result().result
        except Exception as error:
            self._fail_action(f'{action_name} result failed: {error}')
            return
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self._fail_action(
                f'{action_name} trajectory failed: {result.error_code} {result.error_string}'
            )
            return

        self.get_logger().info(f'{action_name} trajectory completed successfully')
        next_state = self.pending_state
        self.action_busy = False
        self.current_action = None
        self.goal_handle = None
        self.state = next_state
        self.pending_state = None
        self.publish_status()

    def ack(self, cmd_id, cmd, accepted, note=''):
        msg = String()
        msg.data = json.dumps({'id': cmd_id, 'cmd': cmd, 'accepted': accepted, 'note': note})
        self.ack_publisher.publish(msg)

    def command_callback(self, msg):
        """Dashboard commands: start, stop (finish current cycle), estop, reset."""
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning('Ignoring malformed /sorting/command JSON')
            self.ack(None, None, False, 'Command must be valid JSON')
            return
        if not isinstance(data, dict):
            self.ack(None, None, False, 'Command must be a JSON object')
            return

        cmd = data.get('cmd')
        cmd_id = data.get('id')
        if not isinstance(cmd_id, str) or not cmd_id:
            self.ack(None, cmd if isinstance(cmd, str) else None, False,
                     'A non-empty string id is required')
            return
        if not isinstance(cmd, str):
            self.ack(cmd_id, None, False, 'cmd must be a string')
            return

        accepted, note = True, ''
        if cmd == 'start':
            if self.state == TaskState.FAILED or self.estopped:
                accepted, note = False, 'Reset first: the task is in FAILED'
            else:
                self.armed = True
                self.completion_logged = False
                if self.state == TaskState.COMPLETE:
                    self.detected_object = None
                    self.state = TaskState.DETECT
        elif cmd == 'stop':
            self.armed = False
            if self.state == TaskState.DETECT:
                self.detected_object = None
            note = 'Current cycle finishes; no new pick will start'
        elif cmd == 'estop':
            self.estopped = True
            self.armed = False
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self._fail_action('Emergency stop pressed on the dashboard')
        elif cmd == 'reset':
            if self.state != TaskState.FAILED:
                accepted, note = False, 'Nothing to reset'
            else:
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
                self.action_generation += 1
                self.goal_handle = None
                self.estopped = False
                self.armed = False
                self.failure_message = ''
                self.detected_object = None
                self.place_stage = 0
                self.home_stage = 0
                self.completion_logged = False
                self.action_busy = False
                self.current_action = None
                self.pending_state = None
                self.state = TaskState.IDLE
                note = 'Cleared. Press Start to allow a new pick'
        else:
            accepted, note = False, f'Unknown command: {cmd}'
        self.get_logger().info(f'Command {cmd}: accepted={accepted} {note}')
        self.ack(cmd_id, cmd, accepted, note)
        self.publish_status()

    def send_arm_goal(self, positions, duration, next_state):
        return self.send_trajectory(
            self.arm_client, self.PICK_JOINTS, positions, duration, 'Arm', next_state
        )

    def send_gripper_goal(self, positions, duration, next_state):
        return self.send_trajectory(
            self.gripper_client, self.GRIPPER_JOINTS, positions, duration, 'Gripper', next_state
        )

    def run_state_machine(self):
        self.publish_status()
        if self.action_busy or self.state == TaskState.FAILED:
            return
        if self.state == TaskState.IDLE:
            self.detected_object = None
            self.state = TaskState.DETECT
            self.publish_status()
            self.get_logger().info('State: IDLE -> DETECT')
        elif self.state == TaskState.DETECT:
            if self.detected_object is not None:
                self.state = TaskState.PICK
                self.publish_status()
                self.get_logger().info('Detection accepted; moving to PICK')
        elif self.state == TaskState.PICK:
            self.send_arm_goal(self.PICK_POSITION, 3, TaskState.GRIP)
        elif self.state == TaskState.GRIP:
            self.send_gripper_goal([self.GRIP_CLOSED_POSITION, self.GRIP_CLOSED_POSITION], 2, TaskState.CLASSIFY)
        elif self.state == TaskState.CLASSIFY:
            label = self.detected_object['label']
            self.get_logger().info(f'CLASSIFY: detected {label}')
            if label == 'red_object':
                self.bin_label = 'red'
                self.high_bin_position = self.RED_HIGH_BIN_POSITION
                self.place_position = self.RED_PLACE_POSITION
            elif label == 'blue_object':
                self.bin_label = 'blue'
                self.high_bin_position = self.BLUE_HIGH_BIN_POSITION
                self.place_position = self.BLUE_PLACE_POSITION
            elif label == 'green_object':
                self.bin_label = 'green'
                self.high_bin_position = self.GREEN_HIGH_BIN_POSITION
                self.place_position = self.GREEN_PLACE_POSITION
            else:
                self._fail_action(
                    f'No calibrated sorting pose for {label}; configured routes are red, blue, and green'
                )
                return
            self.place_stage = 0
            self.state = TaskState.PLACE
            self.publish_status()
        elif self.state == TaskState.PLACE:
            if self.place_stage == 0:
                self.get_logger().info('PLACE: raising the gripped cube to a high carry pose')
                if self.send_arm_goal(self.HIGH_PICKUP_POSITION, 3, TaskState.PLACE):
                    self.place_stage = 1
            elif self.place_stage == 1:
                self.get_logger().info(f'PLACE: transferring above the {self.bin_label} bin rim')
                if self.send_arm_goal(self.high_bin_position, 4, TaskState.PLACE):
                    self.place_stage = 2
            else:
                self.get_logger().info(f'PLACE: lowering over the centre of the {self.bin_label} bin opening')
                if self.send_arm_goal(self.place_position, 3, TaskState.RELEASE):
                    self.place_stage = 3
        elif self.state == TaskState.RELEASE:
            self.send_gripper_goal([0.0, 0.0], 2, TaskState.RETURN_HOME)
        elif self.state == TaskState.RETURN_HOME:
            if self.home_stage == 0:
                self.get_logger().info(f'RETURN_HOME: retracting above the {self.bin_label} bin')
                if self.send_arm_goal(self.high_bin_position, 3, TaskState.RETURN_HOME):
                    self.home_stage = 1
            else:
                self.get_logger().info('RETURN_HOME: moving arm to the upright home pose')
                if self.send_arm_goal(self.HOME_POSITION, 5, TaskState.COMPLETE):
                    self.home_stage = 2
        elif self.state == TaskState.COMPLETE:
            if not self.completion_logged:
                self.get_logger().info(f'{self.bin_label.capitalize()} sorting sequence completed; arm is home. Press Start for another cycle.')
                self.completion_logged = True
            self.armed = False
            self.place_stage = 0
            self.home_stage = 0
            # Stay COMPLETE so the operator can verify the dropped object.

def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
