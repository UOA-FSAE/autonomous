from pyPS4Controller.controller import Controller
import rclpy
from rclpy.node import node
from ackerman_msgs.msg import AckermannDrive, AckermannDriveStamped
from std_msgs.msg import Header

# R3 right and left buttons max is 32767 when pressed and -32767 when fully released
# L3/left joystick goes up,down when fully pushed up is -32767 and positive when down
# L3/left joystick goes right and down when fully pushed is 32767 when pushed fully right and negative when fully left
class joystick_teleop(Node):
    def __init__(self, max_speed):
        # publisher
        super().__init__("joystick_teleop")
        self.logger().info("joystick teleoperation node started")
        # now make sure the controller is paired over the Bluetooth and turn on the listener        
        joystick = MyController(max_speed = max_speed,
                                min_speed = 0,
                                max_angle = 18,
                                min = -32767,
                                max = 32767,
                                speed = 0,
                                previous_value = -32767,
                                previous_value2 = -32767,
                                publisher=self.create_publisher(AckermannDriveStamped, "cmd_vel", 10),
                                interface="/dev/input/js0"
                                )
        joystick.listen()

class MyController(Controller):  # create a custom class for your controller and subclass Controller
    """
    If we want to bind an action to the X button on the controller, we need to override its respective methods.

    Some of the buttons have a binary On/Off state. For example the X, Circle, Square, and Triangle buttons.
    When overriding their respective methods there are no args in the function signature.

    Some controls like the L2, L3, R2 and R3 have a variable On state.
    When overriding their respective method, there is a value argument in the function signature 
    which indicates the degree of the input.

    You can put any custom code inside the functions bellow. I have put print statements in there just so you
    can copy/paste the code, connect controller, play with the inputs and see the result.

    All of  the functions that you can override are listed in this script.
    """
    def __init__(self, max_speed, min_speed, max_angle, min, max, speed, previous_value, previous_value2, publisher, **kwargs):
        Controller.__init__(self, **kwargs)
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.max_angle = max_angle
        self.min = min
        self.max = max 
        self.speed = speed 
        self.previous_value = previous_value
        self.previous_value2 = previous_value2
        self.angle = 0
        self.joystick_teleop_pub = publisher
    
    def get_ackerman(self):
        args = {
            'steering_angle': float(self.angle),
            'steering_angle_velocity': 0.0,
            'speed': float(self.speed),
            'acceleration': 0.0,
            'jerk': 0.0,
        }

        return AckermannDrive(**args)

    def get_ackerman_stamped(self, ackerman_msg):
        args = {"header": Header(stamp=self.get_clock().now().to_msg(),
                                  frame_id="joystick_teleop"),
                "drive": ackerman_msg}
        
        return AckermannDriveStamped(**args)

    def on_L3_left(self, value):
        self.angle = (value/self.max) * self.max_angle
        print(f"angle = {self.angle}")
        # publish msg
        msg = self.get_ackerman_stamped(self.get_ackerman())
        self.joystick_teleop_pub.publish(msg)

    def on_L3_right(self, value):
        self.angle = (value/self.max) * self.max_angle
        print(f"angle = {self.angle}")
        # publish msg
        msg = self.get_ackerman_stamped(self.get_ackerman())
        self.joystick_teleop_pub.publish(msg)

    def on_R3_up(self, value):
        if value > self.previous_value:
            self.speed = (value-self.min)/(self.max-self.min) * (self.max_speed-self.min_speed)
            self.previous_value = value
            self.previous_value2 = self.min
        print(f"speed = {self.speed}")
        # publish msg
        msg = self.get_ackerman_stamped(self.get_ackerman())
        self.joystick_teleop_pub.publish(msg)

    def on_R3_down(self, value):
        if value > self.previous_value:
            self.speed = (value-self.min)/(self.max-self.min) * (self.max_speed-self.min_speed)
            self.previous_value = value
            self.previous_value2 = self.min
        print(f"speed = {self.speed}")
        # publish msg
        msg = self.get_ackerman_stamped(self.get_ackerman())
        self.joystick_teleop_pub.publish(msg)

    def on_R3_left(self, value):
        if value > self.previous_value2:
            self.speed = (1-((value+self.max)/(self.max*2)))*self.speed
            self.previous_value = (self.speed*(self.max-self.min))/(self.max_speed-self.min_speed) + self.min
            self.previous_value2 = value
        print(f"speed = {self.speed}")
        # publish msg
        msg = self.get_ackerman_stamped(self.get_ackerman())
        self.joystick_teleop_pub.publish(msg)

    def on_R3_right(self, value):
        if value > self.previous_value2:
            self.speed = (1-((value+self.max)/(self.max*2)))*self.speed
            self.previous_value = (self.speed*(self.max-self.min))/(self.max_speed-self.min_speed) + self.min
            self.previous_value2 = value
        print(f"speed = {self.speed}")
        # publish msg
        msg = self.get_ackerman_stamped(self.get_ackerman())
        self.joystick_teleop_pub.publish(msg)

    # def on_L3_up(self, value):
    #     print(f"on_L3_up {value}")

    # def on_L3_down(self, value):
    #     print(f"on_L3_down {value}")

    # def on_L3_at_rest(self):
    #     """R3 joystick is at rest after the joystick was moved and let go off"""
    #     print("on_L3_at_rest")

    # def on_L3_press(self):
    #     """R3 joystick is clicked"""
    #     print("on_L3_press")

    # def on_L3_release(self):
    #     """R3 joystick is released after the click"""
    #     print("on_L3_release")

    # def on_x_press(self):
    #     print("on_x_press")

    # def on_x_release(self):
    #     print("on_x_release")

    # def on_triangle_press(self):
    #     print("on_triangle_press")

    # def on_triangle_release(self):
    #     print("on_triangle_release")

    # def on_circle_press(self):
    #     print("on_circle_press")

    # def on_circle_release(self):
    #     print("on_circle_release")

    # def on_square_press(self):
    #     print("on_square_press")

    # def on_square_release(self):
    #     print("on_square_release")

    # def on_L1_press(self):
    #     print("on_L1_press")

    # def on_L1_release(self):
    #     print("on_L1_release")

    # def on_L2_press(self, value):
    #     print("on_L2_press")

    # def on_L2_release(self):
    #     print("on_L2_release")

    # def on_R1_press(self):
    #     print("on_R1_press")

    # def on_R1_release(self):
    #     print("on_R1_release")

    # def on_R2_press(self, value):
    #     print("on_R2_press")

    # def on_R2_release(self):
    #     print("on_R2_release")

    # def on_up_arrow_press(self):
    #     print("on_up_arrow_press")

    # def on_up_down_arrow_release(self):
    #     print("on_up_down_arrow_release")

    # def on_down_arrow_press(self):
    #     print("on_down_arrow_press")

    # def on_left_arrow_press(self):
    #     print("on_left_arrow_press")

    # def on_left_right_arrow_release(self):
    #     print("on_left_right_arrow_release")

    # def on_right_arrow_press(self):
    #     print("on_right_arrow_press")

    # def on_R3_at_rest(self):
    #     """R3 joystick is at rest after the joystick was moved and let go off"""
    #     print("on_R3_at_rest")

    # def on_R3_press(self):
    #     """R3 joystick is clicked. This event is only detected when connecting without ds4drv"""
    #     print("on_R3_press")

    # def on_R3_release(self):
    #     """R3 joystick is released after the click. This event is only detected when connecting without ds4drv"""
    #     print("on_R3_release")

    # def on_options_press(self):
    #     print("on_options_press")

    # def on_options_release(self):
    #     print("on_options_release")

    # def on_share_press(self):
    #     """this event is only detected when connecting without ds4drv"""
    #     print("on_share_press")

    # def on_share_release(self):
    #     """this event is only detected when connecting without ds4drv"""
    #     print("on_share_release")

    # def on_playstation_button_press(self):
    #     """this event is only detected when connecting without ds4drv"""
    #     print("on_playstation_button_press")

    # def on_playstation_button_release(self):
    #     """this event is only detected when connecting without ds4drv"""
    #     print("on_playstation_button_release")

def main(args=None):
    max_speed = 10
    joystick_teleop(max_speed)

if __name__ == "__main__":
    main()