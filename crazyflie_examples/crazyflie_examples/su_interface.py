import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from crazyflie_py import Crazyswarm


class SuInterface(Node):
    def __init__(self):
        # 먼저 Crazyswarm 초기화 → 내부적으로 rclpy.init() 실행됨
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.cf = self.swarm.allcfs.crazyflies[0]

        # 그 다음 Node 생성자 호출 (충돌 없음)
        super().__init__('su_interface')

        self.subscription = self.create_subscription(
            String,
            'keyboard_input',
            self.keyboard_callback,
            10
        )
        self.get_logger().info("su_interface node ready.")

    def keyboard_callback(self, msg):
        if not msg.data:
            return
        input_char = msg.data[0]
        if input_char == 'o':
            self.cf.arm(True)
            self.get_logger().info("ARM command sent.")
        elif input_char == 'p':
            self.cf.arm(False)
            self.get_logger().info("DISARM command sent.")

    def shutdown(self):
        self.cf.land(targetHeight=0.04, duration=2.5)
        self.timeHelper.sleep(3.0)


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
