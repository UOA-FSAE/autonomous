#!/usr/bin/python3
import numpy as np
import rclpy
from rclpy.node import Node
from moa_msgs.msg import BoundaryStamped

class test_planning_path(Node):
    def __init__(self):
        super().__init__("test_planning_path")

        left = self.create_publisher(BoundaryStamped, "track/bound_l", 10)
        right = self.create_publisher(BoundaryStamped, "track/bound_r", 10)

        x = np.arange(-10,10)
        y = np.arange(0,20)
        msgl = np.array([],np.float32)
        msgr = np.array([],np.float32)
        while True:
            for i in range(len(x)):
                cx = float(x[i])
                cy = float(y[i])
                msgl = np.append(msgl,[cx,cy])
                msgr = np.append(msgr,[cx,cy-1])
            # print(msgl)
            left.publish(BoundaryStamped(coords=msgl))
            right.publish(BoundaryStamped(coords=msgr))


def main():
    rclpy.init()

    NDE = test_planning_path()

    try:
        rclpy.spin_once(NDE)
    except Exception as e:
        print(f"node spin error: {e}")

    NDE.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
