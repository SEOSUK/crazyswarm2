#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32

from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters

from crazyflie_py import Crazyswarm


class SuInterface(Node):
    """
    - o/p : Crazyradio direct (Crazyswarm) arm/disarm 유지
    - i/j/k/l : /crazyflie_server 파라미터 서비스로 세팅 (ros2 param set 과 동일 경로)

    Topics:
      - keyboard_input (std_msgs/String): 'o' arm, 'p' disarm
      - su/use_vel_mode (std_msgs/Float32): toggle 0/1
      - su/cmd_force    (std_msgs/Float32): Fx command

    Target node:
      - /crazyflie_server

    Target params (same as your working CLI):
      - cf2.params.su_cmd.use_vel_mode
      - cf2.params.su_cmd.cmd_fx
    """

    def __init__(self):
        # -----------------------
        # 1) Crazyswarm 먼저 (기존 방식 유지)
        # -----------------------
        # Crazyswarm()가 rclpy.init()을 내부에서 수행하는 환경이 많아서
        # main()에서는 rclpy.init()을 호출하지 않는 구조로 갑니다.
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.cf = self.swarm.allcfs.crazyflies[0]

        super().__init__("su_interface")

        # -----------------------
        # 2) Config
        # -----------------------
        self.server_node_name = "/crazyflie_server"
        self.cf_prefix = "cf2.params"  # 필요시 바꾸기

        self.param_use_vel_mode = f"{self.cf_prefix}.su_cmd.use_vel_mode"
        self.param_cmd_fx       = f"{self.cf_prefix}.su_cmd.cmd_fx"

        # -----------------------
        # 3) Subscriptions
        # -----------------------
        self.create_subscription(String,  "keyboard_input", self.keyboard_callback, 10)
        self.create_subscription(Float32, "su/use_vel_mode", self.vel_mode_callback, 10)
        self.create_subscription(Float32, "su/cmd_force",    self.force_callback,    10)

        # -----------------------
        # 4) Param service client
        # -----------------------
        self.param_client = self.create_client(SetParameters, f"{self.server_node_name}/set_parameters")
        self.get_logger().info(f"Waiting for {self.server_node_name}/set_parameters ...")
        if not self.param_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(
                f"Parameter service not available: {self.server_node_name}/set_parameters\n"
                f"-> Is crazyflie_server running?"
            )

        # -----------------------
        # 5) Rate limit / deadband
        # -----------------------
        self.fx_deadband = 0.02     # [N]
        self.fx_rate_hz  = 20.0
        self._last_fx = None
        self._last_fx_time = self.get_clock().now()

        self._last_toggle = None

        self.get_logger().info("su_interface ready (arm via Crazyswarm, params via ROS param service).")

    # ======================
    # Helpers
    # ======================
    def _set_param_double(self, name: str, value: float) -> bool:
        """Set a double param on /crazyflie_server via SetParameters."""
        if not self.param_client.service_is_ready():
            self.get_logger().warn("param service not ready")
            return False

        p = ParamMsg()
        p.name = name
        p.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE,
            double_value=float(value),
        )

        req = SetParameters.Request()
        req.parameters = [p]

        future = self.param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        if future.result() is None:
            self.get_logger().error(f"SetParameters failed (no response) for {name}")
            return False

        result = future.result().results[0]
        if not result.successful:
            self.get_logger().error(f"Failed to set {name}: {result.reason}")
            return False

        return True

    # ======================
    # Callbacks
    # ======================
    def keyboard_callback(self, msg: String):
        if not msg.data:
            return
        c = msg.data[0]

        # ✅ 기존 요구: o/p는 ARM/DISARM 토글 유지
        try:
            if c == "o":
                self.cf.arm(True)
                self.get_logger().info("ARM command sent (Crazyswarm).")
            elif c == "p":
                self.cf.arm(False)
                self.get_logger().info("DISARM command sent (Crazyswarm).")
        except Exception as e:
            self.get_logger().error(f"Failed to send arm/disarm: {e}")

    def vel_mode_callback(self, msg: Float32):
        raw = float(msg.data)
        toggle = 1.0 if raw >= 0.5 else 0.0

        # only send if changed
        if self._last_toggle is not None and toggle == self._last_toggle:
            return
        self._last_toggle = toggle

        ok = self._set_param_double(self.param_use_vel_mode, toggle)
        if ok:
            self.get_logger().info(f"Set {self.param_use_vel_mode} = {toggle:.0f}")

    def force_callback(self, msg: Float32):
        fx = float(msg.data)
        now = self.get_clock().now()

        # deadband
        if self._last_fx is not None and abs(fx - self._last_fx) < self.fx_deadband:
            return

        # rate limit
        if (now - self._last_fx_time).nanoseconds < int(1e9 / self.fx_rate_hz):
            return

        self._last_fx = fx
        self._last_fx_time = now

        ok = self._set_param_double(self.param_cmd_fx, fx)
        if ok:
            self.get_logger().info(f"Set {self.param_cmd_fx} = {fx:.3f}")

    def shutdown(self):
        # Best-effort safe
        try:
            self.get_logger().info("Landing before shutdown...")
            self.cf.land(targetHeight=0.04, duration=2.5)
            self.timeHelper.sleep(3.0)
        except Exception as e:
            self.get_logger().error(f"Error during landing: {e}")

        try:
            self.cf.arm(False)
        except Exception:
            pass


def main(args=None):
    # ✅ Crazyswarm()가 rclpy.init을 내부에서 하는 경우가 많아서 여기서는 호출하지 않음
    node = SuInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
