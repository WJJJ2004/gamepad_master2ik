#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from humanoid_interfaces.msg import Master2IkMsg

from gamepad_master2ik.gamepad_reader import Gamepad


def clamp(value: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(value, vmax))


def apply_deadband(value: float, deadband: float) -> float:
    if abs(value) < deadband:
        return 0.0
    return value


def move_toward(current: float, target: float, max_delta: float) -> float:
    if current < target:
        return min(current + max_delta, target)
    return max(current - max_delta, target)


class GamepadMaster2IkNode(Node):
    def __init__(self):
        super().__init__("gamepad_master2ik_node")

        # -----------------------------
        # ROS parameters
        # -----------------------------
        self.declare_parameter("publish_rate_hz", 50.0)

        # interface absolute limits
        self.declare_parameter("x_max", 25.0)    # FRONT_MAX
        self.declare_parameter("x_min", -19.0)   # REAR_MAX
        self.declare_parameter("y_max", 17.0)    # RIGHT_MAX
        self.declare_parameter("y_min", -12.0)   # LEFT_MAX
        self.declare_parameter("yaw_max", 5.0)   # R_YAW_MAX
        self.declare_parameter("yaw_min", -5.0)  # L_YAW_MAX

        # per-tick increment gain (full stick = this increment every tick)
        self.declare_parameter("x_step", 0.35)
        self.declare_parameter("y_step", 0.30)
        self.declare_parameter("yaw_step", 0.08)

        # zero return speed (when stick released)
        self.declare_parameter("x_return_step", 0.25)
        self.declare_parameter("y_return_step", 0.20)
        self.declare_parameter("yaw_return_step", 0.06)

        # deadband for analog input
        self.declare_parameter("deadband_x", 0.05)
        self.declare_parameter("deadband_y", 0.05)
        self.declare_parameter("deadband_yaw", 0.05)

        # fixed fields
        self.declare_parameter("flag", 1.0)
        self.declare_parameter("cp_flag", 0.0)

        # gamepad config
        self.declare_parameter("device_path", "")
        self.declare_parameter("vel_scale_x", 1.0)
        self.declare_parameter("vel_scale_y", 1.0)
        self.declare_parameter("vel_scale_rot", 1.0)

        # optional startup state
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)

        # -----------------------------
        # Load parameters
        # -----------------------------
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.x_max = float(self.get_parameter("x_max").value)
        self.x_min = float(self.get_parameter("x_min").value)
        self.y_max = float(self.get_parameter("y_max").value)
        self.y_min = float(self.get_parameter("y_min").value)
        self.yaw_max = float(self.get_parameter("yaw_max").value)
        self.yaw_min = float(self.get_parameter("yaw_min").value)

        self.x_step = float(self.get_parameter("x_step").value)
        self.y_step = float(self.get_parameter("y_step").value)
        self.yaw_step = float(self.get_parameter("yaw_step").value)

        self.x_return_step = float(self.get_parameter("x_return_step").value)
        self.y_return_step = float(self.get_parameter("y_return_step").value)
        self.yaw_return_step = float(self.get_parameter("yaw_return_step").value)

        self.deadband_x = float(self.get_parameter("deadband_x").value)
        self.deadband_y = float(self.get_parameter("deadband_y").value)
        self.deadband_yaw = float(self.get_parameter("deadband_yaw").value)

        self.flag = float(self.get_parameter("flag").value)
        self.cp_flag = float(self.get_parameter("cp_flag").value)

        device_path = str(self.get_parameter("device_path").value).strip()
        vel_scale_x = float(self.get_parameter("vel_scale_x").value)
        vel_scale_y = float(self.get_parameter("vel_scale_y").value)
        vel_scale_rot = float(self.get_parameter("vel_scale_rot").value)

        initial_x = float(self.get_parameter("initial_x").value)
        initial_y = float(self.get_parameter("initial_y").value)
        initial_yaw = float(self.get_parameter("initial_yaw").value)

        # sanity checks
        if self.x_min > self.x_max:
            raise ValueError(f"x_min({self.x_min}) > x_max({self.x_max})")
        if self.y_min > self.y_max:
            raise ValueError(f"y_min({self.y_min}) > y_max({self.y_max})")
        if self.yaw_min > self.yaw_max:
            raise ValueError(f"yaw_min({self.yaw_min}) > yaw_max({self.yaw_max})")
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be > 0")

        # -----------------------------
        # Internal accumulated command state
        # -----------------------------
        self.x_cmd = clamp(initial_x, self.x_min, self.x_max)
        self.y_cmd = clamp(initial_y, self.y_min, self.y_max)
        self.yaw_cmd = clamp(initial_yaw, self.yaw_min, self.yaw_max)

        # -----------------------------
        # Publisher
        # -----------------------------
        self.pub_ = self.create_publisher(Master2IkMsg, "master2ik", 10)

        # -----------------------------
        # Gamepad
        # -----------------------------
        self.gamepad = Gamepad(
            vel_scale_x=vel_scale_x,
            vel_scale_y=vel_scale_y,
            vel_scale_rot=vel_scale_rot,
            device_path=device_path if device_path else None,
        )

        # -----------------------------
        # Timer
        # -----------------------------
        self.timer_ = self.create_timer(1.0 / self.publish_rate_hz, self.on_timer)

        self.get_logger().info("gamepad_master2ik_node started")
        self.get_logger().info(
            f"limits: x[{self.x_min}, {self.x_max}] "
            f"y[{self.y_min}, {self.y_max}] "
            f"yaw[{self.yaw_min}, {self.yaw_max}]"
        )
        self.get_logger().info(
            f"step per tick: x={self.x_step}, y={self.y_step}, yaw={self.yaw_step}"
        )
        self.get_logger().info(
            f"return per tick: x={self.x_return_step}, y={self.y_return_step}, yaw={self.yaw_return_step}"
        )

    def update_axis(self, current: float, u: float, step: float, return_step: float,
                    vmin: float, vmax: float) -> float:
        if abs(u) > 0.0:
            current += u * step
            current = clamp(current, vmin, vmax)
            return current

        # stick released -> smoothly decay to zero
        current = move_toward(current, 0.0, return_step)
        current = clamp(current, vmin, vmax)
        return current

    def on_timer(self):
        if self.gamepad.is_running:
            raw = self.gamepad.get_command()
            # Gamepad API returns [vx, vy, wz]
            u_x = apply_deadband(float(raw[0]), self.deadband_x)
            u_y = apply_deadband(float(raw[1]), self.deadband_y)
            u_yaw = apply_deadband(float(raw[2]), self.deadband_yaw)

            # safety: if weird values appear
            u_x = clamp(u_x, -1.0, 1.0)
            u_y = clamp(u_y, -1.0, 1.0)
            u_yaw = clamp(u_yaw, -1.0, 1.0)
        else:
            # if disconnected, command smoothly returns to zero
            u_x = 0.0
            u_y = 0.0
            u_yaw = 0.0

        self.x_cmd = self.update_axis(
            self.x_cmd, u_x, self.x_step, self.x_return_step, self.x_min, self.x_max
        )
        self.y_cmd = self.update_axis(
            self.y_cmd, u_y, self.y_step, self.y_return_step, self.y_min, self.y_max
        )
        self.yaw_cmd = self.update_axis(
            self.yaw_cmd, u_yaw, self.yaw_step, self.yaw_return_step, self.yaw_min, self.yaw_max
        )

        msg = Master2IkMsg()
        msg.x_length = float(self.x_cmd)
        msg.y_length = float(self.y_cmd)
        msg.yaw = float(self.yaw_cmd)
        msg.flag = float(self.flag)
        msg.cp_flag = float(self.cp_flag)

        msg.one_x_length = 0
        msg.one_y_length = 0
        msg.one_yaw = 0
        msg.one_step_flag = 0

        self.pub_.publish(msg)

    def destroy_node(self):
        try:
            self.gamepad.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GamepadMaster2IkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
