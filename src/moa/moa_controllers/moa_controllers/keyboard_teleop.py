#!/usr/bin/python3
import keyboard
from timeit import default_timer

class keyboard_teleop():
    def __init__(self, steering_angle, speed, top_speed,update):
        self.steering_angle = steering_angle
        self.speed = speed
        self.top_speed = top_speed  # m/s
        self.update = update
        self.up_key_pressed = False

        keyboard.on_press_key("left", self.onleftkeypress)
        keyboard.on_press_key("right", self.onrightkeypress)
        keyboard.on_press_key("up", self.onupkeypress)
        keyboard.on_press_key("down", self.ondownkeypress)
        keyboard.on_release_key("up", self.onreleaseupkeypress)

        # RCLPY timer here which calls a method to slow car down: 

        start = default_timer()
        while True:
            duration = default_timer()-start
            if duration > 1:
                self.slowcardown()
                start=default_timer()

    def onleftkeypress(self,event):
        self.steering_angle = max(self.steering_angle-self.update, -25)
        print(self.steering_angle)
    def onrightkeypress(self,event):
        self.steering_angle = min(self.steering_angle+self.update, 25)
        print(self.steering_angle)
    def onupkeypress(self,event):
        self.up_key_pressed = True
        self.speed = min(self.speed+self.update, self.top_speed)
        print(self.speed)
    def ondownkeypress(self,event):
        self.speed = max(self.speed-self.update, 0)
        print(self.speed)
    def onreleaseupkeypress(self,event):
        self.up_key_pressed = False
    def slowcardown(self):
        if self.speed != 0 and not self.up_key_pressed:
            self.speed = max(self.speed-1, 0)
            print(self.speed)

if __name__ == "__main__":
    inst = keyboard_teleop(0,0,10,0.5)