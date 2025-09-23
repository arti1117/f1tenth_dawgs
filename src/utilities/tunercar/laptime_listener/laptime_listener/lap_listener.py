import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import argparse

class LapListener(Node):
    def __init__(self, topic):
        super().__init__('lap_listener')
        self.sub = self.create_subscription(Float64, topic, self.cb, 10)
        self.received = False

    def cb(self, msg: Float64):
        if not self.received:
            self.received = True
            # print "lap_time" and terminate
            print(f"{msg.data}", flush=True)
            rclpy.shutdown()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/lap_time', help='lap time topic (Float64)')
    args = parser.parse_args()

    rclpy.init()
    node = LapListener(args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
