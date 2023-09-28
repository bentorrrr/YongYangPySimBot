#!/usr/bin/python3

import os, platform
if platform.system() == "Linux" or platform.system() == "Darwin":
    os.environ["KIVY_VIDEO"] = "ffpyplayer"
    
from pysimbotlib.core import PySimbotApp, Robot
from kivy.config import Config
import logging



import random

# Force the program to show user's log only for "debug" level or more. The info log will be disabled.
Config.set('kivy', 'log_level', 'debug')

class MyRoboot(Robot):
    sensors = []
    foodangle = 0
    cnt = 0   

    def update(self):        
        self.foodangle = self.smell_nearest()
        self.sensors = self.distance()
        # logging.critical(f"Foodangle = {self.foodangle}")
        #self.turn(5)
        self.Rotating()

    def Rotating(self):
        # if self.sensors[0] > 5 and self.sensors[1] > 10 and self.sensors[7] > 10 and self.foodangle > 20 and self.foodangle < -20:
        # if self.sensors[0] < 10:
        #     self.move(-13)
        if self.cnt >= 15:
            self.move(-20)
            self.cnt = 0
        elif self.sensors[0] < 15:
            self.turn(14)
            self.cnt += 1
        elif self.sensors[1] < 15:
            self.turn(-14)
            self.cnt += 1
        elif self.sensors[7] < 15:
            self.turn(14)
            self.cnt += 1
        else:
            self.turn(0)
            self.walking()

    def Rotate2Food(self):
        if self.foodangle > 20:
            self.turn(15)
        elif self.foodangle < -20:
            self.turn(-15)

    def walking(self):
        self.cnt = 0
        if self.stuck:
            self.move(-18)
        else:
            self.move(12)
            self.Rotate2Food()

        


if __name__ == '__main__':
    app = PySimbotApp(robot_cls=MyRoboot, enable_wasd_control=False, interval=.1)
    app.run()