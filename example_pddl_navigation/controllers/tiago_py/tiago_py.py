import os
import numpy as np
import math
import random
from controller import Robot
import pyperplan
from pyperplan.planner import find_domain, HEURISTICS, search_plan, SEARCHES
robot = Robot()

timestep = int(robot.getBasicTimeStep())


l_motor = robot.getDevice("wheel_left_joint")
r_motor = robot.getDevice("wheel_right_joint")
lidar = robot.getDevice("lidar")
lidar.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)
gps = robot.getDevice('gps')
gps.enable(timestep)
l_motor.setPosition(math.inf)
r_motor.setPosition(math.inf)


RESOLUTION=198


def call_planner(domain, problem):
    def get_callable_names(callables, omit_string):
        names = [c.__name__ for c in callables]
        names = [n.replace(omit_string, "").replace("_", " ") for n in names]
        return ", ".join(names)

    search_names = get_callable_names(SEARCHES.values(), "_search")
    heuristic = "hff"
    search="bfs"
    problem = os.path.abspath(problem)
    domain = os.path.abspath(domain)
    search = SEARCHES[search]
    heuristic = None
    use_preferred_ops = heuristic == "hffpo"
    solution = search_plan(
        domain,
        problem,
        search,
        heuristic,
        use_preferred_ops=use_preferred_ops,
    )
    return solution

def write_task(where, to):
    output = "task.pddl"
    fd = open(output, 'w')
    fd.write(f"""(define (problem go-to) (:domain example)
	(:objects
		bot - robot
		catroom sheeproom deerroom emptyroom - room
	)
	(:init
		(in bot {where}room)
		(cangowest sheeproom catroom)
		(cangowest deerroom emptyroom)

		(cangoeast catroom sheeproom)
		(cangoeast emptyroom deerroom)

		(cangonorth emptyroom catroom)
		(cangonorth deerroom sheeproom)

		(cangosouth sheeproom deerroom)
		(cangosouth catroom emptyroom)

	)
	(:goal
		(in bot {to}room)
	)
    )""")
    fd.close()


class ChangeRoom(object):
    # Intentionally poor way of working with state machines!
    # We expect better!
    def __init__(self, angle):
        self.target_angle = angle
        self.state = "orientate"

    def detect_doors(self, l, th=1):
        # detect opening
        open = None
        back_d = l[len(l)//2]
        for i in range(len(l)-1, 0, -1):
            d = l[i]
            if d-back_d < -th:
                open = i
            back_d = d
        # detect closing
        close = None
        back_d = l[len(l)//2]
        for i in range(0, len(l)):
            d = l[i]
            if d-back_d < -th:
                close = i
            back_d = d
        return ((open+close)/2-RESOLUTION//2)*math.pi/RESOLUTION, open, close

    def step(self, lidar_values, compass_values):
        angle = math.atan2(compass_values[0], -compass_values[2])
        if self.state == "orientate":
            print('ORIENTATE', angle, self.target_angle)
            error = self.target_angle-angle
            print(error)
            if math.fabs(error)>0.15: # This is a constant control.
                if error < 0:        # You can try with a Propotional (or even PID)!
                    l_motor.setVelocity(1)
                    r_motor.setVelocity(-1)
                else:
                    l_motor.setVelocity(-1)
                    r_motor.setVelocity(+1)
            else:
                self.state = "approach"
                l_motor.setVelocity(0)
                r_motor.setVelocity(0)
        elif self.state == "approach":
            print('APPROACH')
            target_angle, o, c = self.detect_doors(lidar_values)
            print('APPROACH', o, c)
            l_motor.setVelocity(1+target_angle*2)
            r_motor.setVelocity(1-target_angle*2)
           
            next = False
            if o is not None:
                if o < RESOLUTION//6:
                    next = True
            if c is not None:
                if c>RESOLUTION-RESOLUTION//6:
                    next = True
            if next:
                self.state = "cross" 
                self.cross_steps = 1200
        elif self.state == "cross":
            print("CROSS")
            self.cross_steps -=1 
            l_motor.setVelocity(0.8)
            r_motor.setVelocity(0.8)
            H = RESOLUTION//2
            if np.mean(np.array(lidar_values[H-10:H+10]))< 1.4 or self.cross_steps == 0:
                self.state = "done"
                print("DONE")
        elif self.state == "done":
            l_motor.setVelocity(0.3)
            r_motor.setVelocity(-0.3)
            

class Chill(object):
    # Intentionally poor way of working with state machines!
    # We expect better!
    def __init__(self):
        self.tick = 0
        self.state = "done"

    def step(self, lidar_values, compass_values):
        l_motor.setVelocity(math.cos(0.01*self.tick)*1.5)
        r_motor.setVelocity(-math.cos(0.01*self.tick)*1.5)
        self.tick += 1 
            
def get_room(values):
    if values[0]>=0:
        if values[1]>=0:
            return "cat"
        else:
            return "sheep"
    else:
        if values[1]>=0:
            return "empty"
        else:
            return "deer"
        


behaviour = None
current_action = None

while (robot.step(timestep) != -1):
    room = get_room(gps.getValues())
    write_task(room, "empty")
    print(room)

    solution = call_planner("domain.pddl", "task.pddl")
    print(solution)
    if len(solution)>0:
        first_action = solution[0].name[1:].split()[0]
    else:
        first_action = "chill"
    print(f'{first_action=}')

    allow_action_change = True
    if behaviour is not None:
        if behaviour.state == "cross":
            allow_action_change = False

    if allow_action_change and first_action != current_action:
        current_action = first_action
        if first_action == "movewest":
            behaviour = ChangeRoom(math.pi/2)
        elif first_action == "movenorth":
            behaviour = ChangeRoom(0)
        elif first_action == "moveeast":
            behaviour = ChangeRoom(-math.pi/2)
        elif first_action == "movesouth":
            behaviour = ChangeRoom(math.pi)
        elif first_action == "chill":
            behaviour = Chill()
        else:
            print("to do")
         
    lidar_values = lidar.getRangeImage()[1:-1]
    compass_values = compass.getValues()
    behaviour.step(lidar_values, compass_values)

