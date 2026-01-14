import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from crazyflie_py import Crazyswarm


class SuInterface(Node):
    def __init__(self):
        # 먼저 Crazyswarm 초기화 → 내부적으로 rclpy.init() 실행됨
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.cf = self.swarm.allcfs.crazyflies[0]

        # 그 다음 Node 생성자 호출 (충돌 없음)
        super().__init__('su_interface')

        # 키보드 입력 (o/p)
        self.subscription = self.create_subscription(
            String,
            'keyboard_input',
            self.keyboard_callback,
            10
        )

        # ★ vel_mode 토픽
        self.vel_mode_sub = self.create_subscription(
            Float32,
            'su/use_vel_mode',
            self.vel_mode_callback,
            10
        )

        # ★ force 토픽 (j/k/l 에서 오는 힘 명령)
        self.force_sub = self.create_subscription(
            Float32,
            'su/cmd_force',
            self.force_callback,
            10
        )

        self.get_logger().info("su_interface node ready.")

    # ======================
    # 콜백들
    # ======================

    def keyboard_callback(self, msg: String):
        if not msg.data:
            return
        input_char = msg.data[0]
        if input_char == 'o':
            self.cf.arm(True)
            self.get_logger().info("ARM command sent.")
        elif input_char == 'p':
            self.cf.arm(False)
            self.get_logger().info("DISARM command sent.")

    # ★ vel_mode 콜백: 펌웨어 파라미터 su_cmd.use_vel_mode 설정
    def vel_mode_callback(self, msg: Float32):
        val = float(msg.data)
        self.get_logger().info(f"[VEL_CB] recv {val}")
        try:
            # Crazyswarm / crazyflie_py 에서 사용하는 방식
            self.cf.setParam("su_cmd.use_vel_mode", val)
            self.get_logger().info(f"Set su_cmd.use_vel_mode = {val}")
        except Exception as e:
            self.get_logger().error(f"Failed to set su_cmd.use_vel_mode: {e}")

    # ★ force 콜백: 펌웨어 파라미터 su_cmd.cmd_fx 설정
    def force_callback(self, msg: Float32):
        val = float(msg.data)
        self.get_logger().info(f"[FORCE_CB] recv {val}")
        try:
            # su_cmd_integrator.c 에서 선언한 파라미터:
            #   PARAM_ADD(PARAM_FLOAT, cmd_fx, &su_cmd_fx)
            # 여기에 직접 써줌 → su_cmd_dbg.cmd_fx 로 로깅됨
            self.cf.setParam("su_cmd.cmd_fx", val)
            self.get_logger().info(f"Set su_cmd.cmd_fx = {val}")
        except Exception as e:
            self.get_logger().error(f"Failed to set su_cmd.cmd_fx: {e}")

    # 착륙 및 종료
    def shutdown(self):
        try:
            self.get_logger().info("Landing before shutdown...")
            self.cf.land(targetHeight=0.04, duration=2.5)
            self.timeHelper.sleep(3.0)
        except Exception as e:
            self.get_logger().error(f"Error during landing: {e}")


def main(args=None):
    node = SuInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

