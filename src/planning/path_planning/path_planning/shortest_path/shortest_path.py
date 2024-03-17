import rclpy
from rclpy.node import Node

class test(Node):
    def __init__(self):
        super().__init__("test")
        print("it works lets go")

def main():
    rclpy.init()
    nde = test()
    rclpy.spin_once(nde)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
