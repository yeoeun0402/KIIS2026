"""[더미] 실시간 모니터 자리 — 이 실험 버전에서는 기본으로 꺼져 있다 (./run.sh --monitor 일 때만 실행).

나중에 붙일 때: kiis-platform의 web_monitor_node(브라우저 http://127.0.0.1:8765)를 가져오면 된다.
지금은 주행 노드 토픽이 들어오는지만 2초마다 한 줄로 찍는다(더미 화면).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Int16, Int32


class MonitorStub(Node):
    def __init__(self):
        super().__init__("monitor_stub")
        self.last = {"pwm": None, "pot": None, "state": None}
        self.create_subscription(Int16, "/car/pwm", lambda m: self.last.__setitem__("pwm", m.data), 10)
        self.create_subscription(Int32, "/car/steering_pot", lambda m: self.last.__setitem__("pot", m.data), 10)
        self.create_subscription(Int8, "/car/state", lambda m: self.last.__setitem__("state", m.data), 10)
        self.create_timer(2.0, self.tick)
        self.get_logger().warn("모니터는 더미 — 터미널 한 줄 요약만 (웹 모니터 없음)")

    def tick(self):
        self.get_logger().info(f"pwm={self.last['pwm']} pot={self.last['pot']} state={self.last['state']}")


def main(args=None):
    rclpy.init(args=args)
    node = MonitorStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
