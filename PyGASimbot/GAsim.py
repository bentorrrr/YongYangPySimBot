import os, sys
import random
import csv
import math

from pysimbotlib.core import Robot, Simbot, Util, PySimbotApp
from kivy.core.window import Window
from kivy.logger import Logger

next_gen_robots = []

def before_simulation(simbot: Simbot):
    Logger.info(len(simbot.robots))
    for robot in simbot.robots:
    # random RULES value for the first generation
        if simbot.simulation_count == 0:
            Logger.info("GA: initial population")
            for i, RULE in enumerate(robot.RULES):
                for k in range(len(RULE)):
                    robot.RULES[i][k] = random.randrange(256)
    # used the calculated RULES value from the previous generation
        else:
            Logger.info("GA: copy the rules from previous generation" + str(simbot.simulation_count - 1) + " to current generation")
            # rule = read_rule("./best_gen{0}.csv".format(simbot.simulation_count - 1))
            # Logger.info(rule)
            # # for simbot_robot in simbot.robots:
            # #     simbot_robot.RULES = rule
            for simbot_robot, robot_from_last_gen in zip(simbot.robots, next_gen_robots):
                simbot_robot.RULES = robot_from_last_gen.RULES
            break



def after_simulation(simbot: Simbot):
    Logger.info("GA: Start GA Process ...")
    # There are some simbot and robot calcalated statistics and property during simulation
    # - simbot.simulation_count
    # - simbot.eat_count
    # - simbot.food_move_count
    # - simbot.score
    # - simbot.scoreStr
    # - simbot.robot[0].eat_count
    # - simbot.robot[0].collision_count
    # - simbot.robot[0].color
    # evaluation – compute fitness values here
    for robot in simbot.robots:
        food_pos = simbot.objectives[0].pos
        robot_pos = robot.pos
        distance = Util.distance(food_pos, robot_pos)
        robot.fitness = 1000 - int(distance)
        robot.fitness -= robot.collision_count
    # descending sort and rank: the best 10 will be on the list at index 0 to 9
    simbot.robots.sort(key=lambda robot: robot.fitness, reverse=True)    
    # fitnesslog(simbot.robots[0].fitness)
    # empty the list
    next_gen_robots.clear()
    # adding the best to the next generation.
    next_gen_robots.append(simbot.robots[0])
    write_rule(simbot.robots[0], "./best_gen{0}.csv".format(simbot.simulation_count-1))

    num_robots = len(simbot.robots)
    Logger.info("GA: num_robots = " + str(num_robots))
    def select():
        index = random.randrange(0 ,11)
        return simbot.robots[index]
    # doing genetic operations
    for _ in range(num_robots - 1):
        select1 = select()
        select2 = select() # design the way for selection by yourself
        while select1 == select2:
            select2 = select()
        # crossover
        for i, RULE in enumerate(select1.RULES):
            for k in range(len(RULE)):
                if k > 8 and k < 4:
                    select1.RULES[i][k], select2.RULES[i][k] = select2.RULES[i][k], select1.RULES[i][k]
                ranval1 = random.random()
                ranval2 = random.random()
                ranrul1 = random.randrange(0,256)
                ranrul2 = random.randrange(0,256)
                if ranval1 < 0.3:
                    select1.RULES[i][k] = ranrul1
                if ranval2 < 0.3:
                    select2.RULES[i][k] = ranrul2
            # if ranval1 < 0.4:
            #     select1.RULES[i]= ranrul1
            # if ranval2 < 0.4:
            #     select2.RULES[i]= ranrul2
        
        next_gen_robots.append(select2)
        # write_rule(select, "./best_gen{0}.csv".format(simbot.simulation_count-1))

def fitnesslog(fitness: float):
    with open("./fitnesslog.csv", "a") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerows(fitness)

def write_rule(robot, filename):
    with open(filename, "a") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerows(robot.RULES)

def read_rule(filename):
    with open(filename, "r") as f:
        reader = csv.reader(f)
        out = [b for b in list(reader)]
        for i in range(len(out)):
            out[i] = [ int(j) for j in out[i] ]
        return out
class StupidRobot(Robot):
    RULE_LENGTH = 11
    NUM_RULES = 10
    def __init__(self, **kwarg):
        super(StupidRobot, self).__init__(**kwarg)
        self.RULES = [[0] * self.RULE_LENGTH for _ in range(self.NUM_RULES)]
        # initial list of rules
        self.rules = [0.] * self.NUM_RULES
        self.turns = [0.] * self.NUM_RULES
        self.moves = [0.] * self.NUM_RULES
        self.fitness = 0

    def update(self):
        self.ir_values = self.distance()
        self.S0, self.S1, self.S2, self.S3, self.S4, self.S5, self.S6, self.S7 = self.ir_values
        self.target = self.smell()
        for i, RULE in enumerate(self.RULES):
            self.rules[i] = 1.0
            for k, RULE_VALUE in enumerate(RULE):
                if k < 8:
                    if RULE_VALUE % 5 == 1:
                        if k == 0: self.rules[i] *= self.S0_near()
                        elif k == 1: self.rules[i] *= self.S1_near()
                        elif k == 2: self.rules[i] *= self.S2_near()
                        elif k == 3: self.rules[i] *= self.S3_near()
                        elif k == 4: self.rules[i] *= self.S4_near()
                        elif k == 5: self.rules[i] *= self.S5_near()
                        elif k == 6: self.rules[i] *= self.S6_near()
                        elif k == 7: self.rules[i] *= self.S7_near()
                    elif RULE_VALUE % 5 == 2:
                        if k == 0: self.rules[i] *= self.S0_far()
                        elif k == 1: self.rules[i] *= self.S1_far()
                        elif k == 2: self.rules[i] *= self.S2_far()
                        elif k == 3: self.rules[i] *= self.S3_far()
                        elif k == 4: self.rules[i] *= self.S4_far()
                        elif k == 5: self.rules[i] *= self.S5_far()
                        elif k == 6: self.rules[i] *= self.S6_far()
                        elif k == 7: self.rules[i] *= self.S7_far()
                elif k == 8:
                    temp_val = RULE_VALUE % 6
                    if temp_val == 1: self.rules[i] *= self.smell_left()
                    elif temp_val == 2: self.rules[i] *= self.smell_center()
                    elif temp_val == 3: self.rules[i] *= self.smell_right()
                elif k==9: self.turns[i] = (RULE_VALUE % 181) - 90
                elif k==10: self.moves[i] = (RULE_VALUE % 21) - 10            
        answerTurn = 0.0
        answerMove = 0.0
        for turn, move, rule in zip(self.turns, self.moves, self.rules):
            answerTurn += turn * rule
            answerMove += move * rule
        
        self.turn(answerTurn)
        self.move(answerMove)


    def S0_near(self):
        if self.S0 <= 0: return 1.0
        elif self.S0 >= 100: return 0.0
        else: return 1 - (self.S0 / 100.0)
    def S0_far(self):
        if self.S0 <= 0: return 0.0
        elif self.S0 >= 100: return 1.0
        else: return self.S0 / 100.0
 
    def S1_near(self):
        if self.S1 <= 0: return 1.0
        elif self.S1 >= 100: return 0.0
        else: return 1 - (self.S1 / 100.0)
    
    def S1_far(self):
        if self.S1 <= 0: return 0.0
        elif self.S1 >= 100: return 1.0
        else: return self.S1 / 100.0

    def S2_near(self):
        if self.S2 <= 0: return 1.0
        elif self.S2 >= 100: return 0.0
        else: return 1 - (self.S2 / 100.0)
    
    def S2_far(self):
        if self.S2 <= 0: return 0.0
        elif self.S2 >= 100: return 1.0
        else: return self.S2 / 100.0

    def S3_near(self):
        if self.S3 <= 0: return 1.0
        elif self.S3 >= 100: return 0.0
        else: return 1 - (self.S3 / 100.0)

    def S3_far(self):
        if self.S3 <= 0: return 0.0
        elif self.S3 >= 100: return 1.0
        else: return self.S3 / 100.0

    def S4_near(self):
        if self.S4 <= 0: return 1.0
        elif self.S4 >= 100: return 0.0
        else: return 1 - (self.S4 / 100.0)
    
    def S4_far(self):
        if self.S4 <= 0: return 0.0
        elif self.S4 >= 100: return 1.0
        else: return self.S4 / 100.0
    
    def S5_near(self):
        if self.S5 <= 0: return 1.0
        elif self.S5 >= 100: return 0.0
        else: return 1 - (self.S5 / 100.0)
    
    def S5_far(self):
        if self.S5 <= 0: return 0.0
        elif self.S5 >= 100: return 1.0
        else: return self.S5 / 100.0
    
    def S6_near(self):
        if self.S6 <= 0: return 1.0
        elif self.S6 >= 100: return 0.0
        else: return 1 - (self.S6 / 100.0)
    
    def S6_far(self):
        if self.S6 <= 0: return 0.0
        elif self.S6 >= 100: return 1.0
        else: return self.S6 / 100.0
    
    def S7_near(self):
        if self.S7 <= 0: return 1.0
        elif self.S7 >= 100: return 0.0
        else: return 1 - (self.S7 / 100.0)
    
    def S7_far(self):
        if self.S7 <= 0: return 0.0
        elif self.S7 >= 100: return 1.0
        else: return self.S7 / 100.0
    
    def smell_right(self):
        if self.target >= 45: return 1.0
        elif self.target <= 0: return 0.0
        else: return self.target / 45.0
    
    def smell_left(self):
        if self.target <= -45: return 1.0
        elif self.target >= 0: return 0.0
        else: return 1-(-1*self.target)/45.0
        
    def smell_center(self):
        if self.target <= 45 and self.target >= 0: return self.target / 45.0
        if self.target <= -45 and self.target <= 0: return 1-(-1*self.target)/45.0
        else: return 0.0



if __name__ == '__main__':
    try:
        app = PySimbotApp(robot_cls = StupidRobot,
        num_robots=100,
        food_move_after_eat=False,
        customfn_before_simulation=before_simulation,
        customfn_after_simulation=after_simulation,
        interval = 1.0/60.0,
        max_tick = 250,
        simulation_forever=True,)
        app.run()
    except Exception as e:
        Logger.exception(e)
    
    