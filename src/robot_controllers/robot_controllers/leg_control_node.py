#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import json
from adafruit_servokit import ServoKit
import time
import math


class Head:
    def __init__ (self, channel_y, channel_x, invert_x=False, invert_y=False):
        self.pca = ServoKit(channels=16)
        self.channel_x = channel_x
        self.channel_y = channel_y
        self.invert_x = invert_x
        self.invert_y = invert_y

    def map(self, value, inMin, inMax, outMin, outMax):
        return outMin + (float(value - inMin) / float(inMax - inMin) * (outMax - outMin))
    

    def move_xy(self, angolo_x, angolo_y):

        if self.invert_x:
            angolo_x = self.map(angolo_x, 0, 180, 180, 0)
        if self.invert_y:
            angolo_y = self.map(angolo_y, 0, 180, 180, 0)

        self.pca.servo[self.channel_x].angle = angolo_x
        self.pca.servo[self.channel_y].angle = angolo_y


class Leg:
    def __init__(self, channel_y, channel_x, invert_x=False, invert_y=False):
        self.pca = ServoKit(channels=16)
        self.channel_x = channel_x
        self.channel_y = channel_y
        self.invert_x = invert_x
        self.invert_y = invert_y
        self.lunghezza_gamba = 14.0  # in cm


    def map(self, value, inMin, inMax, outMin, outMax):
        return outMin + (float(value - inMin) / float(inMax - inMin) * (outMax - outMin))

    def move_xy(self, angolo_x, angolo_y):

        if self.invert_x:
            angolo_x = self.map(angolo_x, 0, 180, 180, 0)
        if self.invert_y:
            angolo_y = self.map(angolo_y, 0, 180, 180, 0)

        self.pca.servo[self.channel_x].angle = angolo_x
        self.pca.servo[self.channel_y].angle = angolo_y

    def backward(self, x0, y0, x1, y1, num_steps):

        self.delta_x = (x1 - x0) / num_steps
        self.delta_y = (y1 - y0) / num_steps
        self.y = y0
        self.x = x0

        for step in range(num_steps):
            self.x = x0 + step * self.delta_x
            self.pca.servo[self.channel_x].angle = self.x
            self.pca.servo[self.channel_y].angle = self.y

        for step in range(num_steps):
            self.y = y0 + step * self.delta_y
            self.pca.servo[self.channel_x].angle = self.x
            self.pca.servo[self.channel_y].angle = self.y

        for step in range(num_steps):
            self.x = x1 - step * self.delta_x
            self.pca.servo[self.channel_x].angle = self.x
            self.pca.servo[self.channel_y].angle = self.y

        for step in range(num_steps):
            self.y = y1 - step * self.delta_y
            self.pca.servo[self.channel_x].angle = self.x
            self.pca.servo[self.channel_y].angle = self.y





class JoystickSubscriber(Node):
    def __init__(self):
        super().__init__("leg_contoller_subscriber")
        self.subscription = self.create_subscription(Point, "right_joystick_data", self.callback, 10)
        self.pca = ServoKit(channels=16)
        self.get_logger().info("Leg cntroller subscriber has been started")
        self.head = Head(12, 13)
        self.leg_A = Leg(4, 5, invert_x=True, invert_y=False)
        self.leg_B = Leg(0, 1, invert_x=True, invert_y=False)
        self.leg_C = Leg(2, 3, invert_x=True, invert_y=True)
        self.leg_D = Leg(11, 10)
        self.leg_E = Leg(9, 8, invert_x=False, invert_y=True)
        self.leg_F = Leg(6, 7, invert_x=False, invert_y=True)




    
    def callback(self, msg):
        x = msg.x
        y = msg.y
        z = msg.z

        self.R_legs = [self.leg_A, self.leg_E, self.leg_D]
        self.L_legs = [self.leg_F, self.leg_B, self.leg_C]

        angle_x = 155
        offset = 13
        angle_x = int((x + 1.0) * 90)
        angle_y = int((y + 1.0) * 90)

        self.head.move_xy(angle_y + offset, angle_x)

            # for leg in self.R_legs:
            #     leg.move_xy(angle_x + offset, 95)   
            # for leg in self.L_legs:
            #     leg.move_xy(angle_x - offset, 60)


            # time.sleep(self.delay)

            # for leg in self.R_legs:
            #     leg.move_xy(angle_x + offset, 60)   
            # for leg in self.L_legs:
            #     leg.move_xy(angle_x - offset, 95)

            # time.sleep(self.delay)  

            # for leg in self.R_legs:
            #     leg.move_xy(angle_x - offset, 60)   
            # for leg in self.L_legs:
            #     leg.move_xy(angle_x + offset, 95)

            # time.sleep(self.delay)  


            # for leg in self.R_legs:
            #     leg.move_xy(angle_x - offset, 95)   
            # for leg in self.L_legs:
            #     leg.move_xy(angle_x + offset, 60)

            # time.sleep(self.delay)  









        # for leg in self.R_legs:
        #     leg.move_xy(angle_x + offset, 55)   
        # for leg in self.L_legs:
        #     leg.move_xy(angle_x - offset, 100)


        # time.sleep(self.delay)

        # for leg in self.R_legs:
        #     leg.move_xy(angle_x + offset, 100)   
        # for leg in self.L_legs:
        #     leg.move_xy(angle_x - offset, 55)

        # time.sleep(self.delay)  

        # for leg in self.R_legs:
        #     leg.move_xy(angle_x - offset, 100)   
        # for leg in self.L_legs:
        #     leg.move_xy(angle_x + offset, 55)

        # time.sleep(self.delay)  


        # for leg in self.R_legs:
        #     leg.move_xy(angle_x - offset, 55)   
        # for leg in self.L_legs:
        #     leg.move_xy(angle_x + offset, 100)

        # time.sleep(self.delay)  






        # for leg in self.L_legs:
        #     leg.move_xy(angle_x, 45)     
        # for leg in self.R_legs:
        #     leg.move_xy(angle_x, 110)

        # time.sleep(self.delay)  


        self.get_logger().info("command data - X: {}, Y: {}, Z: {}".format(x, y, z))






        # self.leg_F.backward(20, 100, 65, 100, 300)
        # self.leg_A.forward(60, 20, 70, 100, 500)   # ( x0, x1, y0, y1, num_steps):






def main(args=None):
    rclpy.init(args=args)
    node = JoystickSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
