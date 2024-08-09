#!/usr/bin/python3

import os, sys
import random

from pysimbotlib.App import PySimbotApp
from pysimbotlib.Robot import Robot
from kivy.core.window import Window
from kivy.logger import Logger

# Number of robot that will be run
ROBOT_NUM = 1

# Delay between update (default: 1/60 (or 60 frame per sec))
TIME_INTERVAL = 1.0/60 #10frame per second 

# Max tick
MAX_TICK = 5000

# START POINT
START_POINT = (20, 560)

# Map file
MAP_FILE = 'maps/default_map.kv'

class FuzzyRobot(Robot):

    def __init__(self):
        super(FuzzyRobot, self).__init__()
        self.pos = START_POINT

    def update(self):
        ''' Update method which will be called each frame
        '''        
        self.ir_values = self.distance()
        self.target = self.smell()

        # initial list of rules
        rules = list()
        turns = list()
        moves = list()

        turndefault = 37
        turnfood = 43
        movedefault = 10

        # rule 0
        rules.append(self.front_far() * self.left_far() * self.right_far())
        turns.append(0)
        moves.append(movedefault+2)

        # rule 1 smell left
        rules.append(self.smell_left() * self.front_far() * self.left_far() * self.right_far())
        turns.append(-turnfood)
        moves.append(movedefault)

        # rule 2 smell right
        rules.append(self.smell_right() * self.front_far() * self.left_far() * self.right_far())
        turns.append(turnfood)
        moves.append(movedefault)

        # rule 3 front near
        rules.append(self.front_near())
        turns.append(-turndefault)
        moves.append(-movedefault- 1)

        # rule 4 right near
        rules.append(self.right_near())
        turns.append(-turndefault)
        moves.append(movedefault)
        
        # rule 5 left near
        rules.append(self.left_near())
        turns.append(turndefault)
        moves.append(movedefault)

        # rule 6 front far left right near
        rules.append(self.front_far() * self.left_near() * self.right_near())
        turns.append(0)
        moves.append(movedefault)

        # rule 7 front left near right far
        rules.append(self.front_near() * self.left_near() * self.right_far())
        turns.append(turndefault)
        moves.append(movedefault)

        # rule 8 front right near left far
        rules.append(self.front_near() * self.left_far() * self.right_near())
        turns.append(-turndefault)
        moves.append(movedefault)
        
        rules.append(self.front_right_near())
        turns.append(-turndefault)
        moves.append(-movedefault)

        rules.append(self.front_left_near())
        turns.append(turndefault)
        moves.append(-movedefault)

        ans_turn = 0.0
        ans_move = 0.0
        for r, t, m in zip(rules, turns, moves):
            ans_turn += t * r
            ans_move += m * r

        if(ans_turn == 0 and ans_move == 0):
            ans_move = -movedefault
        
        self.turn(ans_turn)
        self.move(ans_move)
        
    def front_far(self):
        irfront = self.ir_values[0]
        if irfront <= 10:
            return 0.0
        elif irfront >= 40:
            return 1.0
        else:
            return (irfront-10.0) / 30.0
    
    def front_near(self):
        return 1 - self.front_far()

    def left_far(self):
        irleft = self.ir_values[6]
        if irleft <= 10:
            return 0.0
        elif irleft >= 30: 
            return 1.0
        else:
            return (irleft-10.0) / 20.0

    def left_near(self):
        return 1 - self.left_far()

    def right_far(self):
        irright = self.ir_values[2]
        if irright <= 10:
            return 0.0
        elif irright >= 30:
            return 1.0
        else:
            return (irright-10.0) / 20.0

    def right_near(self):
        return 1.0 - self.right_far()
    
    def front_left_far(self):
        irfrontleft = self.ir_values[7]
        if irfrontleft <= 5:
            return 0.0
        elif irfrontleft >= 35:
            return 1.0
        else:
            return (irfrontleft-5) / 30.0
        
    def front_left_near(self):
        return 1.0 - self.front_left_far()

    def front_right_far(self):
        irfrontright = self.ir_values[1]
        if irfrontright <= 5:
            return 0.0
        elif irfrontright >= 35:
            return 1.0
        else:
            return (irfrontright-5) / 30.0
    
    def front_right_near(self):
        return 1 - self.front_right_far()
    
    def smell_right(self):
        target = self.smell()
        if target >= 90:
            return 1.0
        elif target <= 0:
            return 0.0
        else:
            return target / 90.0

    def smell_center(self):
        target = abs(self.smell())
        if target >= 45:
            return 1.0
        elif target <= 0:
            return 0.0
        else:
            return target / 45.0

    def smell_left(self):
        target = self.smell()
        if target <= -90:
            return 1.0
        elif target >= 0:
            return 0.0
        else:
            return -target / 90.0

if __name__ == '__main__':
    app = PySimbotApp(FuzzyRobot, ROBOT_NUM, mapPath=MAP_FILE, interval=TIME_INTERVAL, maxtick=MAX_TICK)
    app.run()