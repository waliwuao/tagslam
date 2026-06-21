#!/usr/bin/env python3
# ---------------------------------------------------------------------------
# PID Control Node (no UI)
# Waits for a service trigger, then runs PID regulation to move the robot
# to a target position relative to AprilTags. Aborts on timeout or if the
# tag is lost for too many consecutive frames.
# ---------------------------------------------------------------------------

import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
import yaml


# ---------------------------------------------------------------------------
# PID Controller
# ---------------------------------------------------------------------------
class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0,
                 integral_max=1.0, output_max=1.0):
        # type: ignore
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_max = integral_max
        self.output_max = output_max
        self.reset()

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._first = True

    def update(self, error, dt):
        # type: (float, float) -> float
        if dt <= 0:
            return 0.0
        self._integral += error * dt
        self._integral = max(-self.integral_max,
                             min(self.integral_max, self._integral))
        derivative = 0.0 if self._first else (error - self._prev_error) / dt
        self._first = False
        self._prev_error = error
        output = (self.kp * error +
                  self.ki * self._integral +
                  self.kd * derivative)
        return max(-self.output_max, min(self.output_max, output))


# ---------------------------------------------------------------------------
# PID Control Node
# ---------------------------------------------------------------------------
class PIDControlNode(Node):
    def __init__(self):
        super().__init__('pid_control_node')

        # Parameters
        self.declare_parameter('config_file', 'config/pid_control.yaml')
        self.declare_parameter('tagslam_config', 'config/tagslam.yaml')

        config_path = self._resolve_path(
            self.get_parameter('config_file').value)
        tagslam_path = self._resolve_path(
            self.get_parameter('tagslam_config').value)

        cfg = self._load_yaml(config_path)

        # Frames
        self._world_frame = cfg.get('world_frame', 'world')
        self._camera_frame = cfg.get('camera_frame', 'cam0')

        # Target
        tgt = cfg.get('target', {})
        self._target_x = float(tgt.get('x', 1.0))
        self._target_y = float(tgt.get('y', 0.0))
        self._target_yaw = float(tgt.get('angular', math.pi))

        # Limits
        self._max_duration = float(cfg.get('max_duration', 3.0))
        self._max_missing = int(cfg.get('max_missing_frames', 10))
        self._done_hold = int(cfg.get('done_hold_frames', 5))

        # PID controllers
        self._pid_x = PID(**cfg['pid_x'])
        self._pid_y = PID(**cfg['pid_y'])
        self._pid_a = PID(**cfg['pid_angular'])

        # Dead zones
        self._dead_linear = float(cfg.get('dead_zone_linear', 0.02))
        self._dead_angular = float(cfg.get('dead_zone_angular', 0.05))

        # TF
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Publisher
        cmd_topic = cfg.get('cmd_vel_topic', '/t0x0101_')
        self._cmd_pub = self.create_publisher(
            Float32MultiArray, cmd_topic, 10)

        # Service
        self._srv = self.create_service(
            Trigger, '~/trigger', self._on_trigger)

        # State
        self._active = False
        self._lock = threading.Lock()
        self._start_time = None
        self._missing_count = 0
        self._done_count = 0
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_yaw = 0.0

        # Control loop timer (20 Hz)
        self._last_time = self.get_clock().now()
        self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f'PID node ready. Target: ({self._target_x:.3f}, '
            f'{self._target_y:.3f}, {math.degrees(self._target_yaw):.1f}°)'
            f'  Topic: {cmd_topic}'
            f'  max_duration={self._max_duration}s'
            f'  max_missing={self._max_missing}')

    # -------------------------------------------------------------------
    # Config helpers
    # -------------------------------------------------------------------
    @staticmethod
    def _load_yaml(path):
        if not os.path.isfile(path):
            raise FileNotFoundError(f'Config not found: {path}')
        with open(path, 'r') as f:
            return yaml.safe_load(f)

    @staticmethod
    def _resolve_path(path):
        if os.path.isabs(path) and os.path.isfile(path):
            return path
        # Try relative to cwd, then workspace root
        candidates = [
            os.path.join(os.getcwd(), path),
        ]
        for p in candidates:
            if os.path.isfile(p):
                return p
        return path  # let it fail with a clear message

    # -------------------------------------------------------------------
    # Service callback
    # -------------------------------------------------------------------
    def _on_trigger(self, request, response):
        with self._lock:
            if self._active:
                response.success = False
                response.message = 'already running'
                return response

            self._start_regulation()
            response.success = True
            response.message = 'regulation started'
            return response

    def _start_regulation(self):
        self._active = True
        self._start_time = self.get_clock().now()
        self._missing_count = 0
        self._done_count = 0
        self._pid_x.reset()
        self._pid_y.reset()
        self._pid_a.reset()
        self.get_logger().info(
            f'Regulation START → target ({self._target_x:.3f}, '
            f'{self._target_y:.3f}, {math.degrees(self._target_yaw):.1f}°)')

    def _stop_regulation(self, reason):
        self._active = False
        self._publish_velocity(0.0, 0.0, 0.0)
        self.get_logger().info(f'Regulation STOP: {reason}')

    # -------------------------------------------------------------------
    # TF lookup
    # -------------------------------------------------------------------
    def _get_pose(self):
        try:
            t = self._tf_buffer.lookup_transform(
                self._world_frame, self._camera_frame,
                rclpy.time.Time())
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return (tx, ty, yaw)
        except Exception:
            return None

    # -------------------------------------------------------------------
    # Publish velocity
    # -------------------------------------------------------------------
    def _publish_velocity(self, vx, vy, va):
        msg = Float32MultiArray()
        msg.data = [float(vx), float(vy), float(va)]
        self._cmd_pub.publish(msg)

    # -------------------------------------------------------------------
    # Control loop (20 Hz)
    # -------------------------------------------------------------------
    def _control_loop(self):
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        if dt <= 0 or dt > 1.0:
            dt = 0.05
        self._last_time = now

        with self._lock:
            if not self._active:
                return

            # Check timeout
            elapsed = (now - self._start_time).nanoseconds * 1e-9
            if elapsed > self._max_duration:
                self._stop_regulation(f'timeout ({elapsed:.2f}s > '
                                      f'{self._max_duration}s)')
                return

            # Get current pose
            pose = self._get_pose()
            if pose is None:
                self._missing_count += 1
                if self._missing_count > self._max_missing:
                    self._stop_regulation(
                        f'tag lost ({self._missing_count} frames)')
                    return
                # Keep publishing zero vel while blind
                self._publish_velocity(0.0, 0.0, 0.0)
                return

            self._missing_count = 0
            self._current_x, self._current_y, self._current_yaw = pose

            # Errors
            ex = self._target_x - self._current_x
            ey = self._target_y - self._current_y
            ea = self._target_yaw - self._current_yaw
            ea = (ea + math.pi) % (2 * math.pi) - math.pi

            # Dead zone
            ex = 0.0 if abs(ex) < self._dead_linear else ex
            ey = 0.0 if abs(ey) < self._dead_linear else ey
            ea = 0.0 if abs(ea) < self._dead_angular else ea

            # Check done
            if ex == 0.0 and ey == 0.0 and ea == 0.0:
                self._done_count += 1
                if self._done_count >= self._done_hold:
                    self._stop_regulation(
                        f'done @ ({self._current_x:.3f}, '
                        f'{self._current_y:.3f}, '
                        f'{math.degrees(self._current_yaw):.1f}°) '
                        f'[{elapsed:.2f}s]')
                    self._publish_velocity(0.0, 0.0, 0.0)
                    return
            else:
                self._done_count = 0

            # PID
            vx = self._pid_x.update(ex, dt)
            vy = self._pid_y.update(ey, dt)
            va = self._pid_a.update(ea, dt)
            self._publish_velocity(vx, vy, va)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PIDControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_regulation('shutdown')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
