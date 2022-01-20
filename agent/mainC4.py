
import sys
import time

from croblink import *
from math import *
import xml.etree.ElementTree as ET
from Map import MyMap, manhattan

CELLROWS=7
CELLCOLS=14

center_id = 0
left_id = 1
right_id = 2
back_id = 3
directions = [0, -90, 180, 90]



class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.state = "stop"
        self.target = []
        self.target_position = []
        self.offset = []
        self.global_target = [27, 13]
        self.direction = 0
        self.position = [27, 13]
        self.map = MyMap()
        self.path = []
        self.prev_path = [[-1,-1]]
        self.visited_targets = {}
        self.goal = False

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()
        while True:
            self.readSensors()
            self.move()
            

    def move(self):
        if not self.offset:
            self.offset = [self.measures.x - self.position[0],self.measures.y - self.position[1]]
            self.target = [self.measures.x, self.measures.y]
            self.map.update(self.position, 9)
        rSpeed =0
        lSpeed = 0
        k = 0.7
        if self.state == "go":
            if self.direction in [0,2]:
                y = self.measures.y
                x = self.measures.x
                target = self.target
            else:
                x = self.measures.y
                y = self.measures.x
                target = [self.target[1], self.target[0]]
            inv = 1
            if self.direction in [3,2]:
                inv = -1
            inv2 = 1
            if self.direction in [1, 2]:
                inv2 = -1

            rSpeed = k * (y - target[1]) * inv
            if abs(x - target[0]) <= 0.1:
                self.position = self.target_position
                if self.measures.irSensor[center_id] > 1:
                    self.driveMotors(0,0)
                self.nextState()
            else:
                if (x - target[0])*inv2 < 0:
                    lSpeed = 0.8 * (1/(self.measures.irSensor[center_id]+0.001))
                else:
                    lSpeed = -0.2
                    rSpeed = 0
                    #print("back")
            # print("POS: " + str([self.measures.x, self.measures.y]) + " TARGET: " + str(self.target) + " DIR: " + str(self.direction))
        elif self.state == "turn":
            if abs(self.measures.compass - directions[self.direction]) <= 2:
                #print("Finished turn")
                #self.nextState()
                self.state = 'go'
                return
            #print(abs(self.measures.compass - directions[self.direction]))
            lSpeed = 0
            sign = lambda a: (a > 0) - (a < 0)
            rSpeed = 0.03 * (self.measures.compass - directions[self.direction])
        elif self.state == "stop":
            self.nextState()
        else:
            print("WHAT?")

        lSpeed *= 0.15
        rSpeed *= 0.15
        left_speed = lSpeed+rSpeed/2
        right_speed = lSpeed-rSpeed/2
        # print(lSpeed)
        # print(rSpeed)
        self.driveMotors(left_speed, right_speed)

    def nextState(self):
        self.updateMap()
        if self.position == self.global_target and not self.goal:
            #print("At target, getting new one")
            self.nextTarget()
            self.nextState()

        elif self.target_position:
            if self.position[1] > self.target_position[1] and self.direction != 1:
                if self.map.get([self.position[0], self.position[1]-1]) == 1:
                    self.recalculate_path()
                    self.nextState()
                    return
                self.direction = 1
                self.state = 'turn'
                #print("turning")
            elif self.position[1] < self.target_position[1] and self.direction != 3:
                if self.map.get([self.position[0], self.position[1]+1]) == 1:
                    self.recalculate_path()
                    self.nextState()
                    return
                self.direction = 3
                self.state = 'turn'
                #print("turning to: " + str(self.target_position))
            elif self.position[0] < self.target_position[0] and self.direction != 0:
                if self.map.get([self.position[0]+1, self.position[1]]) == 1:
                    self.recalculate_path()
                    self.nextState()
                    return
                self.direction = 0
                self.state = 'turn'
                #print("turning")
            elif self.position[0] > self.target_position[0] and self.direction != 2:
                if self.map.get([self.position[0]-1, self.position[1]]) == 1:
                    self.recalculate_path()
                    self.nextState()
                    return
                self.direction = 2
                self.state = 'turn'
                #print("turning")
            elif self.position == self.target_position:
                #print("Go forward through path")
                self.set_target(self.path[0])
                self.target_position = self.path[0]
                #print("New target: " + str(self.target))
                #print("New target: " + str(self.path[0]))
                self.path.pop(0)
                self.nextState()
            elif self.measures.irSensor[center_id] > 1:
                self.recalculate_path()
                self.nextState()
            else:
                self.state = 'go'
                #print("Going forward")

        else:
            #print("Searching for path from: " + str(self.position) + " to: " + str(self.global_target))
            self.path = self.map.search_path_a_star(self.position, self.global_target)
            if not self.path:
                self.nextTarget()
                self.nextState()
            self.prev_path = [x for x in self.path]
            self.set_target(self.path[0])
            self.target_position = self.path[0]
            #print("Path found: " + str(self.path))
            self.nextState()

    def updateMap(self):
        front_pos, back_pos, left_pos, right_pos = self.getNeighbors(1)
        if self.measures.irSensor[center_id] > 1:
            self.map.update(front_pos, 1)
        else:
            self.map.update(front_pos, 2)
        if self.measures.irSensor[back_id] > 1:
            self.map.update(back_pos, 1)
        else:
            self.map.update(back_pos, 2)
        if self.measures.irSensor[left_id] > 1:
            self.map.update(left_pos, 1)
        else:
            self.map.update(left_pos, 2)
        if self.measures.irSensor[right_id] > 1:
            self.map.update(right_pos, 1)
        else:
            self.map.update(right_pos, 2)
        if self.map.get(self.position) != 9:
            self.map.update(self.position, 2)
        if self.measures.ground != -1:
            self.visited_targets[self.measures.ground] = self.position
            print("Target: " + str(self.measures.ground) + " found at: " + str(self.position))

    def set_target(self, target):
        self.target = [target[0]+self.offset[0], target[1]+self.offset[1]]

    def recalculate_path(self):
        #print("can't go forward, recalculate path")
        #print("Searching for new path from: " + str(self.position) + " to: " + str(self.global_target))
        self.nextTarget()
        self.path = self.map.search_path_a_star(self.position, self.global_target)
        if not self.path:
            self.nextTarget()
        if self.path == self.prev_path:
            print("Stuck")
            self.map.print()
            self.state = 'stop'
            return
        self.prev_path = [x for x in self.path]
        #print("Path found: " + str(self.path))
        if not self.path:
            print("No path found")
            self.state = 'stop'
            return
        self.target_position = self.path[0]
        self.path.pop(0)

    def getNeighbors(self, step):
        if self.direction==0:
            transformations = [[step, 0], [-step, 0], [0, step], [0, -step]]
        elif self.direction==1:
            transformations = [[0, -step], [0, step], [step, 0], [-step, 0]]
        elif self.direction==2:
            transformations = [[-step, 0], [step, 0], [0, -step], [0, step]]
        else:
            transformations = [[0, step], [0, -step], [-step, 0], [step, 0]]
        front_pos = [self.position[0] + transformations[0][0], self.position[1] + transformations[0][1]]
        back_pos = [self.position[0] + transformations[1][0], self.position[1] + transformations[1][1]]
        left_pos = [self.position[0] + transformations[2][0], self.position[1] + transformations[2][1]]
        right_pos = [self.position[0] + transformations[3][0], self.position[1] + transformations[3][1]]
        return front_pos, back_pos, left_pos, right_pos

    def sort_pos(self, pos):
        return manhattan(self.position, pos)

    def nextTarget(self):
        if len(self.visited_targets.keys()) == int(self.nBeacons):
            if self.position == self.visited_targets[0]:
                if self.goal:
                    return
                self.compute_best_path()
                #self.map.print_to_file("mapC3.out")
                self.driveMotors(0,0)
                sys.exit(0)
            self.global_target = self.visited_targets[0]
            self.path = []
            self.target_position = []
            print("Returning home")
            return
        f_pos, b_pos, l_pos, r_pos = self.getNeighbors(2)
        #f_pos2, b_pos2, l_pos2, r_pos2 = self.getNeighbors(1)
        unvisited = []
        next_target = []
        for i in range(len(self.map.map)):
            for j in range(len(self.map.map[i])):
                if self.map.map[i][j] == 0:
                    unvisited.append([i,j])
        if f_pos in unvisited and len(self.map.search_path_a_star(self.position, f_pos)) == 2:
            #print("Front")
            next_target = f_pos
        elif l_pos in unvisited and len(self.map.search_path_a_star(self.position, l_pos)) == 2:
            #print("Back")
            next_target = l_pos
        elif r_pos in unvisited and len(self.map.search_path_a_star(self.position, r_pos)) == 2:
            #print("Left")
            next_target = r_pos
        elif b_pos in unvisited and len(self.map.search_path_a_star(self.position, b_pos)) == 2:
            #print("Right")
            next_target = b_pos
        else:
            shortest_len = 99
            for pos in sorted(unvisited, key=self.sort_pos):
                path = self.map.search_path_a_star(self.position, pos)
                if not path:
                    continue
                if len(path) < shortest_len:
                    next_target = pos
                    shortest_len = len(path)
                #print(str(pos) + "  " + str(len(path)))
                if shortest_len < 30:
                    break

        if not next_target:
            print("No more targets")
            print(self.visited_targets)
            self.compute_best_path()
            #self.map.print_to_file("mapC3.out")
            self.driveMotors(0, 0)
            sys.exit(0)

        self.global_target = next_target
        self.path = []
        self.target_position = []
        print("New global target: " + str(self.global_target))

    def compute_best_path(self):

        total_path = []
        for i in range(int(self.nBeacons)-1):
            total_path += self.map.search_path_a_star(self.visited_targets[i],
                                                      self.visited_targets[i+1])[:-1]
        total_path += self.map.search_path_a_star(self.visited_targets[int(self.nBeacons)-1],
                                                  self.visited_targets[0])
        adjusted_path = [[pos[0]-27, pos[1]-13] for pos in total_path]
        adjusted_targets = []
        for i in range(int(self.nBeacons)):
            pos = self.visited_targets[i]
            adjusted_targets.append([pos[0]-27,pos[1]-13])
        print(adjusted_targets)
        file = open(outfile, 'w')
        for pos in adjusted_path:
            s = str(pos[0]) + " " + str(pos[1])
            if pos in adjusted_targets and pos != [0,0]:
                target = adjusted_targets.index(pos)
                s += " #" + str(target)
            s += '\n'
            file.write(s)

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()

        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None

           i=i+1



rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None
outfile = ""

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "--outfile" or sys.argv[i] == "-f") and i != len(sys.argv) - 1:
        outfile = sys.argv[i + 1]
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
