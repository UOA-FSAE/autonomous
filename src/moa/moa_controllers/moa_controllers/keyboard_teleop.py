#!/usr/bin/python3
from timeit import default_timer
from sshkeyboard import listen_keyboard
import rclpy
from rclpy.node import Node 
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from std_msgs.msg import Header

class keyboard_teleop(Node):
    def __init__(self, update):
        super().__init__("keyboard_teleop")
        self.steering_angle = 0
        self.speed = 0
        self.top_speed = 10  # m/s
        self.update = update
        self.up_key_pressed = False
        self.duration = 1000

        # log control msg
        txt = """
                    CONTROL THE GOKART USING ARROWS KEYS!
                    RIGHT ARROW = RIGHT STEER
                    LEFT ARROW = LEFT STEER
                    UP ARROW = INCREASE SPEED
                    DOWN ARROW = DECREASE SPEED
                    
                    nb: keys have to be pressed one by one two different keys pressed simulatenously doesn't work
        """
        self.get_logger().info(txt)

        # publisher
        self.keyboard_teleop_pub = self.create_publisher(AckermannDriveStamped, "cmd_vel", 10)
        listen_keyboard(on_press=self.key_pressed, on_release=self.key_released,delay_second_char=0.01,delay_other_chars=0.01)

    def key_pressed(self,key):
        if key == "left":
            self.steering_angle = max(self.steering_angle-self.update, -25)
            print(self.steering_angle)
        if key == "right":
            self.steering_angle = min(self.steering_angle+self.update, 25)
            print(self.steering_angle)
        if key == "up":
            self.up_key_pressed = True
            self.speed = min(self.speed+self.update, self.top_speed)
            print(self.speed)
        if key == "down":
            self.speed = max(self.speed-2*self.update, 0)
            print(self.speed)
        if key == "z":
            self.speed = 0
            print(self.speed)
        
        # publish msg
        msg = self.get_ackerman_stamped(self.get_ackerman())
        self.keyboard_teleop_pub.publish(msg)

    def key_released(self,key):
        if key == "up":
            self.up_key_pressed = False
            self.released_up_time = default_timer()

    # def slowcardown(self):
    #     print('hello')
    #     duration = default_timer() - self.released_up_time
    #     if duration > 1 and self.speed != 0 and not self.up_key_pressed:
    #         self.speed = max(self.speed-1, 0)
    #         print(self.speed)

    def get_ackerman(self):
        args = {
            'steering_angle': float(self.steering_angle),
            'steering_angle_velocity': 0.0,
            'speed': float(self.speed),
            'acceleration': 0.0,
            'jerk': 0.0,
        }

        return AckermannDrive(**args)

    def get_ackerman_stamped(self, ackerman_msg):
        args = {"header": Header(stamp=self.get_clock().now().to_msg(),
                                  frame_id="keyboard_teleop"),
                "drive": ackerman_msg}
        
        return AckermannDriveStamped(**args)

def main(args=None):
    rclpy.init(args=args)

    node = keyboard_teleop(update=1.0)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()