"""[더미] Unity 디지털 트윈 연결 자리 — 이 실험 버전에서는 기본으로 꺼져 있다 (./run.sh --twin 일 때만 실행).

나중에 붙일 때: kiis-platform의 twin_bridge(arduino_node를 socket://<윈도우>:45002로 연 것)처럼
/car/pwm·/car/steer_cmd를 받아 트윈 펌웨어(TwinFirmware.cs, car_controller.ino 규약)로 같은 바이트를 보내면 된다.
지금은 더미 데이터만 /twin/car/steering_pot에 낸다(배선 확인용).
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class TwinStub(Node):
    def __init__(self):
        super().__init__("twin_stub")
        self.pub = self.create_publisher(Int32, "/twin/car/steering_pot", 10)
        self.t = 0.0
        self.create_timer(0.05, self.tick)
        self.get_logger().warn("트윈은 더미 — 가짜 /twin/car/steering_pot만 발행 (Unity 연결 없음)")

    def tick(self):
        self.t += 0.05
        self.pub.publish(Int32(data=int(500 + 200 * math.sin(self.t))))


def main(args=None):
    rclpy.init(args=args)
    node = TwinStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
