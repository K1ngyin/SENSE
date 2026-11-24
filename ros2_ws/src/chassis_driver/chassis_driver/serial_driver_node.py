"""ROS 2 node publishing odometry and sensor messages from STM32 feedback."""

import math
import queue
import threading
import time
from dataclasses import dataclass
from typing import Optional

import serial
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, Imu

from chassis_driver.protocol import FeedbackFrame, FrameParser


@dataclass(frozen=True)
class ReceivedFrame:
    frame: FeedbackFrame
    received_monotonic: float
    ros_stamp: object


class ChassisDriverNode(Node):
    """Read-only serial driver; command-frame transmission is intentionally absent."""

    def __init__(self) -> None:
        super().__init__("chassis_driver")

        self._declare_parameters()
        self._read_parameters()

        # Reliable publishers work with both reliable and best-effort subscribers.
        topic_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, topic_qos)
        self._imu_pub = self.create_publisher(Imu, self._imu_topic, topic_qos)
        self._battery_pub = self.create_publisher(
            BatteryState, self._battery_topic, topic_qos
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, self._diagnostics_topic, 10
        )

        self._frame_queue: queue.Queue[ReceivedFrame] = queue.Queue(maxsize=1000)
        self._stop_event = threading.Event()
        self._state_lock = threading.Lock()
        self._connected = False
        self._last_error = ""
        self._last_frame_monotonic: Optional[float] = None
        self._last_motor_flag: Optional[int] = None
        self._last_battery_v: Optional[float] = None
        self._valid_frame_count = 0
        self._bad_tail_count = 0
        self._bad_checksum_count = 0
        self._bytes_discarded = 0
        self._queue_drop_count = 0
        self._integration_gap_count = 0
        self._last_rate_sample_time = time.monotonic()
        self._last_rate_sample_count = 0
        self._frame_rate_hz = 0.0

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_odom_monotonic: Optional[float] = None

        self._serial_thread = threading.Thread(
            target=self._serial_worker,
            name="chassis-serial-reader",
            daemon=True,
        )
        self._serial_thread.start()

        self._publish_timer = self.create_timer(self._publish_period_sec, self._publish_pending)
        self._diagnostics_timer = self.create_timer(
            self._diagnostics_period_sec, self._publish_diagnostics
        )

        self.get_logger().info(
            f"Feedback-only chassis driver started for {self._port} at {self._baudrate} baud"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "port": "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C98180921-if00",
            "baudrate": 115200,
            "serial_timeout_sec": 0.1,
            "reconnect_interval_sec": 1.0,
            "frame_timeout_sec": 0.5,
            "max_integration_dt_sec": 0.25,
            "publish_period_sec": 0.005,
            "diagnostics_period_sec": 1.0,
            "odom_frame_id": "odom",
            "base_frame_id": "base_link",
            "imu_frame_id": "imu_link",
            "odom_topic": "/odom",
            "imu_topic": "/imu/data_raw",
            "battery_topic": "/battery_state",
            "diagnostics_topic": "/diagnostics",
            "odom_pose_variance_xy_m2": 0.05,
            "odom_pose_variance_yaw_rad2": 0.1,
            "odom_twist_variance_xy_m2ps2": 0.02,
            "odom_twist_variance_wz_rad2ps2": 0.05,
            "imu_acceleration_variance_m2ps4": 0.0,
            "imu_angular_velocity_variance_rad2ps2": 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self) -> None:
        get = lambda name: self.get_parameter(name).value
        self._port = str(get("port"))
        self._baudrate = int(get("baudrate"))
        self._serial_timeout_sec = float(get("serial_timeout_sec"))
        self._reconnect_interval_sec = float(get("reconnect_interval_sec"))
        self._frame_timeout_sec = float(get("frame_timeout_sec"))
        self._max_integration_dt_sec = float(get("max_integration_dt_sec"))
        self._publish_period_sec = float(get("publish_period_sec"))
        self._diagnostics_period_sec = float(get("diagnostics_period_sec"))

        self._odom_frame_id = str(get("odom_frame_id"))
        self._base_frame_id = str(get("base_frame_id"))
        self._imu_frame_id = str(get("imu_frame_id"))
        self._odom_topic = str(get("odom_topic"))
        self._imu_topic = str(get("imu_topic"))
        self._battery_topic = str(get("battery_topic"))
        self._diagnostics_topic = str(get("diagnostics_topic"))

        self._odom_pose_variance_xy = float(get("odom_pose_variance_xy_m2"))
        self._odom_pose_variance_yaw = float(get("odom_pose_variance_yaw_rad2"))
        self._odom_twist_variance_xy = float(get("odom_twist_variance_xy_m2ps2"))
        self._odom_twist_variance_wz = float(get("odom_twist_variance_wz_rad2ps2"))
        self._imu_acceleration_variance = float(get("imu_acceleration_variance_m2ps4"))
        self._imu_angular_velocity_variance = float(
            get("imu_angular_velocity_variance_rad2ps2")
        )

        if not self._port:
            raise ValueError("parameter 'port' must not be empty")
        if self._baudrate <= 0:
            raise ValueError("parameter 'baudrate' must be positive")
        for name, value in (
            ("serial_timeout_sec", self._serial_timeout_sec),
            ("reconnect_interval_sec", self._reconnect_interval_sec),
            ("frame_timeout_sec", self._frame_timeout_sec),
            ("max_integration_dt_sec", self._max_integration_dt_sec),
            ("publish_period_sec", self._publish_period_sec),
            ("diagnostics_period_sec", self._diagnostics_period_sec),
        ):
            if value <= 0.0:
                raise ValueError(f"parameter '{name}' must be positive")
        for name, value in (
            ("odom_pose_variance_xy_m2", self._odom_pose_variance_xy),
            ("odom_pose_variance_yaw_rad2", self._odom_pose_variance_yaw),
            ("odom_twist_variance_xy_m2ps2", self._odom_twist_variance_xy),
            ("odom_twist_variance_wz_rad2ps2", self._odom_twist_variance_wz),
            ("imu_acceleration_variance_m2ps4", self._imu_acceleration_variance),
            ("imu_angular_velocity_variance_rad2ps2", self._imu_angular_velocity_variance),
        ):
            if value < 0.0:
                raise ValueError(f"parameter '{name}' must be zero or positive")

    def _serial_worker(self) -> None:
        last_reported_error = None
        while not self._stop_event.is_set():
            port = None
            parser = FrameParser()
            try:
                port = serial.Serial(
                    port=self._port,
                    baudrate=self._baudrate,
                    timeout=self._serial_timeout_sec,
                )
                with self._state_lock:
                    self._connected = True
                    self._last_error = ""
                last_reported_error = None
                self.get_logger().info(f"Opened chassis serial port {self._port}")

                while not self._stop_event.is_set():
                    data = port.read(128)
                    if not data:
                        continue

                    old_bad_tail = parser.bad_tail_count
                    old_bad_checksum = parser.bad_checksum_count
                    old_discarded = parser.bytes_discarded
                    frames = parser.feed(data)
                    with self._state_lock:
                        self._bad_tail_count += parser.bad_tail_count - old_bad_tail
                        self._bad_checksum_count += (
                            parser.bad_checksum_count - old_bad_checksum
                        )
                        self._bytes_discarded += parser.bytes_discarded - old_discarded

                    for frame in frames:
                        received_monotonic = time.monotonic()
                        with self._state_lock:
                            self._valid_frame_count += 1
                            self._last_frame_monotonic = received_monotonic
                            self._last_motor_flag = frame.motor_flag
                            self._last_battery_v = frame.battery_v
                        ros_stamp = self.get_clock().now().to_msg()
                        self._enqueue_frame(ReceivedFrame(frame, received_monotonic, ros_stamp))

            except (serial.SerialException, OSError) as error:
                message = str(error)
                with self._state_lock:
                    self._connected = False
                    self._last_error = message
                if message != last_reported_error:
                    self.get_logger().warning(
                        f"Chassis serial unavailable ({message}); retrying"
                    )
                    last_reported_error = message
            finally:
                if port is not None:
                    try:
                        port.close()
                    except (serial.SerialException, OSError):
                        pass
                with self._state_lock:
                    self._connected = False

            self._stop_event.wait(self._reconnect_interval_sec)

    def _enqueue_frame(self, received: ReceivedFrame) -> None:
        try:
            self._frame_queue.put_nowait(received)
            return
        except queue.Full:
            pass

        # Keep the most recent state if the ROS executor has fallen behind.
        try:
            self._frame_queue.get_nowait()
        except queue.Empty:
            pass
        with self._state_lock:
            self._queue_drop_count += 1
        try:
            self._frame_queue.put_nowait(received)
        except queue.Full:
            with self._state_lock:
                self._queue_drop_count += 1

    def _publish_pending(self) -> None:
        while True:
            try:
                received = self._frame_queue.get_nowait()
            except queue.Empty:
                break
            self._publish_frame(received)

    def _publish_frame(self, received: ReceivedFrame) -> None:
        frame = received.frame
        stamp = received.ros_stamp

        dt = None
        if self._last_odom_monotonic is not None:
            dt = received.received_monotonic - self._last_odom_monotonic
        self._last_odom_monotonic = received.received_monotonic

        if dt is not None and 0.0 < dt <= self._max_integration_dt_sec:
            delta_yaw = frame.wz_rad_s * dt
            midpoint_yaw = self._yaw + delta_yaw * 0.5
            self._x += (
                frame.vx_m_s * math.cos(midpoint_yaw)
                - frame.vy_m_s * math.sin(midpoint_yaw)
            ) * dt
            self._y += (
                frame.vx_m_s * math.sin(midpoint_yaw)
                + frame.vy_m_s * math.cos(midpoint_yaw)
            ) * dt
            self._yaw = math.atan2(
                math.sin(self._yaw + delta_yaw), math.cos(self._yaw + delta_yaw)
            )
        elif dt is not None and dt > self._max_integration_dt_sec:
            with self._state_lock:
                self._integration_gap_count += 1

        self._publish_odom(frame, stamp)
        self._publish_imu(frame, stamp)
        self._publish_battery(frame, stamp)

    def _publish_odom(self, frame: FeedbackFrame, stamp) -> None:
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self._odom_frame_id
        message.child_frame_id = self._base_frame_id

        message.pose.pose.position.x = self._x
        message.pose.pose.position.y = self._y
        message.pose.pose.position.z = 0.0
        message.pose.pose.orientation.z = math.sin(self._yaw * 0.5)
        message.pose.pose.orientation.w = math.cos(self._yaw * 0.5)
        message.pose.covariance = self._planar_covariance(
            self._odom_pose_variance_xy, self._odom_pose_variance_yaw
        )

        message.twist.twist.linear.x = frame.vx_m_s
        message.twist.twist.linear.y = frame.vy_m_s
        message.twist.twist.angular.z = frame.wz_rad_s
        message.twist.covariance = self._planar_covariance(
            self._odom_twist_variance_xy, self._odom_twist_variance_wz
        )
        self._odom_pub.publish(message)

    def _publish_imu(self, frame: FeedbackFrame, stamp) -> None:
        message = Imu()
        message.header.stamp = stamp
        message.header.frame_id = self._imu_frame_id

        # This frame contains no orientation estimate.
        message.orientation_covariance[0] = -1.0
        message.angular_velocity.x = frame.gyro_rad_s[0]
        message.angular_velocity.y = frame.gyro_rad_s[1]
        message.angular_velocity.z = frame.gyro_rad_s[2]
        message.angular_velocity_covariance = self._imu_covariance(
            self._imu_angular_velocity_variance
        )
        message.linear_acceleration.x = frame.accel_m_s2[0]
        message.linear_acceleration.y = frame.accel_m_s2[1]
        message.linear_acceleration.z = frame.accel_m_s2[2]
        message.linear_acceleration_covariance = self._imu_covariance(
            self._imu_acceleration_variance
        )
        self._imu_pub.publish(message)

    def _publish_battery(self, frame: FeedbackFrame, stamp) -> None:
        message = BatteryState()
        message.header.stamp = stamp
        message.voltage = frame.battery_v
        message.percentage = float("nan")
        message.current = float("nan")
        message.charge = float("nan")
        message.capacity = float("nan")
        message.design_capacity = float("nan")
        message.temperature = float("nan")
        message.present = frame.battery_v > 0.0
        message.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        message.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        message.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        self._battery_pub.publish(message)

    def _publish_diagnostics(self) -> None:
        now_monotonic = time.monotonic()
        with self._state_lock:
            connected = self._connected
            last_error = self._last_error
            last_frame = self._last_frame_monotonic
            motor_flag = self._last_motor_flag
            battery_v = self._last_battery_v
            valid_count = self._valid_frame_count
            bad_tail_count = self._bad_tail_count
            bad_checksum_count = self._bad_checksum_count
            bytes_discarded = self._bytes_discarded
            queue_drops = self._queue_drop_count
            integration_gaps = self._integration_gap_count

            sample_dt = now_monotonic - self._last_rate_sample_time
            if sample_dt > 0.0:
                self._frame_rate_hz = (valid_count - self._last_rate_sample_count) / sample_dt
                self._last_rate_sample_time = now_monotonic
                self._last_rate_sample_count = valid_count
            frame_rate = self._frame_rate_hz

        frame_age = None if last_frame is None else max(0.0, now_monotonic - last_frame)
        motor_state = self._motor_state_name(motor_flag)

        status = DiagnosticStatus()
        status.name = "chassis_driver/serial_feedback"
        status.hardware_id = self._port
        status.level = DiagnosticStatus.OK
        status.message = "serial feedback is current"

        if not connected:
            status.level = DiagnosticStatus.ERROR
            status.message = "serial port is disconnected"
        elif frame_age is None:
            status.level = DiagnosticStatus.WARN
            status.message = "waiting for the first valid feedback frame"
        elif frame_age > self._frame_timeout_sec:
            status.level = DiagnosticStatus.ERROR
            status.message = "feedback frame timeout"
        elif motor_flag not in (0, 1):
            status.level = DiagnosticStatus.WARN
            status.message = "unknown motor status flag"
        elif queue_drops:
            status.level = DiagnosticStatus.WARN
            status.message = "ROS publisher queue has dropped feedback frames"

        values = [
            ("connected", str(connected).lower()),
            ("port", self._port),
            ("baudrate", str(self._baudrate)),
            ("frame_rate_hz", f"{frame_rate:.2f}"),
            ("valid_frame_count", str(valid_count)),
            ("bad_tail_count", str(bad_tail_count)),
            ("bad_checksum_count", str(bad_checksum_count)),
            ("bytes_discarded", str(bytes_discarded)),
            ("queue_drop_count", str(queue_drops)),
            ("integration_gap_count", str(integration_gaps)),
            ("motor_flag", "unknown" if motor_flag is None else str(motor_flag)),
            ("motor_state", motor_state),
            ("battery_voltage_v", "unknown" if battery_v is None else f"{battery_v:.3f}"),
            ("last_frame_age_s", "unknown" if frame_age is None else f"{frame_age:.3f}"),
            ("last_serial_error", last_error or "none"),
        ]
        status.values = [KeyValue(key=key, value=value) for key, value in values]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diagnostics_pub.publish(array)

    @staticmethod
    def _motor_state_name(flag: Optional[int]) -> str:
        if flag == 0:
            return "enabled"
        if flag == 1:
            return "disabled"
        return "unknown"

    @staticmethod
    def _planar_covariance(xy_variance: float, yaw_variance: float) -> list:
        covariance = [0.0] * 36
        covariance[0] = xy_variance
        covariance[7] = xy_variance
        covariance[14] = 1.0e6
        covariance[21] = 1.0e6
        covariance[28] = 1.0e6
        covariance[35] = yaw_variance
        return covariance

    @staticmethod
    def _imu_covariance(variance: float) -> list:
        covariance = [0.0] * 9
        if variance <= 0.0:
            covariance[0] = -1.0
        else:
            covariance[0] = variance
            covariance[4] = variance
            covariance[8] = variance
        return covariance

    def destroy_node(self):
        self._stop_event.set()
        if self._serial_thread.is_alive():
            self._serial_thread.join(timeout=max(1.0, self._serial_timeout_sec + 1.0))
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChassisDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
