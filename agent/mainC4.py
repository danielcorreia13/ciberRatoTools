import sys
import time

from croblink import *
from math import *
import xml.etree.ElementTree as ET
from Map import MyMap, manhattan
import itertools

CELLROWS = 7
CELLCOLS = 14

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
        self.absolutePos = [27, 13]
        self.map = MyMap()
        self.path = []
        self.prev_path = [[-1, -1]]
        self.visited_targets = {}
        self.goal = False
        self.lastTargets = []
        self.hard_targets = []

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
            if self.measures.start:
                self.move()

    def x(self):
        return self.absolutePos[0]

    def y(self):
        return self.absolutePos[1]

    def move(self):
        if not self.offset:
            # self.offset = [self.x() - self.position[0],self.y() - self.position[1]]
            # self.target = [self.x(), self.y()]
            self.offset = [0, 0]
            self.target = self.position
            self.map.update(self.position, 9)
            self.updateMap()
        rSpeed = 0
        lSpeed = 0
        k = 0.2
        if self.state == "go":
            if self.direction in [0, 2]:
                x = self.x()
                target = self.target
            else:
                x = self.y()
                target = [self.target[1], self.target[0]]
            if self.measures.irSensor[left_id] > 2.5 or self.measures.irSensor[right_id] > 2.5:
                rSpeed = k * (self.measures.irSensor[left_id] - self.measures.irSensor[right_id])
                rSpeed = min(rSpeed, 0.3)
                # print("ADJUSTTTTTTTTT align")
            else:
                rSpeed = 0
            # print(abs(x - target[0]))
            # print(self.measures.irSensor[center_id] )
            # print("Y: " + str(self.y()))
            if ((abs(x - target[0]) <= 0.15) and self.measures.irSensor[center_id] < 1) or\
                    ((abs(x - target[0]) <= 0.9) and self.measures.irSensor[center_id] >= 2.2):

                self.position = self.target_position
                self.absolutePos = self.target
                print("Absolute Pos: " + str(self.absolutePos))
                print("Diff: " + str(abs(x - target[0])))
                print("Sensor: " + str(self.measures.irSensor[center_id] ))
                self.driveMotors(0, 0)
                if abs(self.measures.compass - directions[self.direction]) > 4:# and not self.measures.irSensor[center_id] >= 2.2:
                    self.state = "turn"
                    # print("TURNNNNNN")
                else:
                    self.updateMap()
                    self.nextState()
                return
            else:
                # if (x - target[0]) * inv2 <= 0:
                lSpeed = 0.8 * (1 / (self.measures.irSensor[center_id] + 0.001))
                # else:
                #     lSpeed = -0.2
                #     rSpeed = 0
                #     # print("back")
            # print("POS: " + str([self.x(), self.y()]) + " TARGET: " + str(self.target) + " DIR: " + str(self.direction))
        elif self.state == "turn":
            if abs(self.measures.compass - directions[self.direction]) <= 4:
                self.driveMotors(0, 0)
                print("Finished turn")
                # self.state = 'go'
                self.updateMap()
                self.nextState()
                return
            # print(abs(self.measures.compass - directions[self.direction]))
            lSpeed = 0
            if self.direction in [0,2]:
                sign = -1
            else:
                sign = 1
            # print(self.direction)
            rSpeed = 0.03 * (self.measures.compass - directions[self.direction])
        elif self.state == "stop":
            print("STOP")
            self.driveMotors(0, 0)
            self.nextState()
            return
        else:
            print("WHAT?")

        self.updatePos(lSpeed)
        # print(self.measures.irSensor[left_id] )
        if lSpeed != 0:
            rSpeed *= lSpeed
        lSpeed *= 0.15
        rSpeed *= 0.15
        left_speed = lSpeed + rSpeed / 2
        right_speed = lSpeed - rSpeed / 2

        self.driveMotors(left_speed, right_speed)


    def updatePos(self, lSpeed):
        if lSpeed < 0.6 or self.state != 'go':
            return
        # print("Old: " + str(self.absolutePos))
        moveFactor = 0.133
        if self.direction == 0:
            self.absolutePos[0] += moveFactor
        elif self.direction == 1:
            self.absolutePos[1] -= moveFactor
        elif self.direction == 2:
            self.absolutePos[0] -= moveFactor
        elif self.direction == 3:
            self.absolutePos[1] += moveFactor
        # print("New: " + str(self.absolutePos))


    def nextState(self):
        # self.updateMap()
        # self.map.print()
        # if self.position == [33, 13]:
        #     print(self.position)
        #     self.map.print()
        #     self.driveMotors(0,0)
        #     sys.exit()
        if self.position == self.global_target and not self.goal:
            # print("At target, getting new one")
            print("nextTarget()")
            self.nextTarget()
            self.state = 'stop'

        elif self.target_position:
            if self.position[1] > self.target_position[1] and self.direction != 1:
                if self.map.get([self.position[0], self.position[1] - 1]) == 1:
                    self.recalculate_path()
                    self.state = 'stop'
                    return
                self.direction = 1
                self.state = 'turn'
                # print("turning")
            elif self.position[1] < self.target_position[1] and self.direction != 3:
                if self.map.get([self.position[0], self.position[1] + 1]) == 1:
                    self.recalculate_path()
                    self.state = 'stop'
                    return
                self.direction = 3
                self.state = 'turn'
                # print("turning to: " + str(self.target_position))
            elif self.position[0] < self.target_position[0] and self.direction != 0:
                if self.map.get([self.position[0] + 1, self.position[1]]) == 1:
                    self.recalculate_path()
                    self.state = 'stop'
                    return
                self.direction = 0
                self.state = 'turn'
                # print("turning")
            elif self.position[0] > self.target_position[0] and self.direction != 2:
                if self.map.get([self.position[0] - 1, self.position[1]]) == 1:
                    self.recalculate_path()
                    self.state = 'stop'
                    return
                self.direction = 2
                self.state = 'turn'
                # print("turning")
            elif self.position == self.target_position:
                # print("Go forward through path")
                old_target = self.global_target
                if self.global_target not in self.lastTargets:
                    print("tring new target")
                    self.nextTarget()
                if self.global_target != old_target:
                    print("found new target")
                    self.target_position = None
                    self.lastTargets.append(self.global_target)
                    if len(self.lastTargets) > 3:
                        self.lastTargets.remove(self.lastTargets[0])
                    while not self.map.search_path_a_star(self.position, self.global_target):
                        print("no path to new target")
                        self.nextTarget()

                    self.get_path()


                self.set_target(self.path[0])
                self.target_position = self.path[0]
                print("pos: " + str(self.position))
                print("Target: " + str(self.target_position))
                self.path.pop(0)
                self.state = 'stop'
                return
            elif self.measures.irSensor[center_id] > 1:
                self.recalculate_path()
                self.state = 'stop'
            else:
                self.state = 'go'
                print("Going forward")

        else:
            self.get_path()
            self.state = 'stop'
            # print("Path found: " + str(self.path))

    def get_path(self):
        print("Searching for path from: " + str(self.position) + " to: " + str(self.global_target))
        self.path = self.map.search_path_a_star(self.position, self.global_target)
        while not self.path:
            print("no path to target")
            self.nextTarget()
            print(self.position)
            print(self.global_target)
            self.path = self.map.search_path_a_star(self.position, self.global_target)

        self.prev_path = [x for x in self.path]
        self.set_target(self.path[0])
        self.target_position = self.path[0]
        self.path.pop(0)

    def updateMap(self):
        print("Updating map at: " + str(self.position))
        front_pos, back_pos, left_pos, right_pos = self.getNeighbors(1)
        if self.position == [43, 5] or self.position == [43, 3]:
            self.map.print()
            print(self.getNeighbors(1))
            print(self.measures.irSensor[center_id])
            print(self.measures.irSensor[right_id])
            print(self.measures.irSensor[left_id])
            print(self.measures.irSensor[back_id])

        if self.measures.irSensor[center_id] > 1.2:
            self.map.update(front_pos, 1)
        else:
            self.map.update(front_pos, 2)
        if self.measures.irSensor[back_id] > 1.2:
            self.map.update(back_pos, 1)
        else:
            self.map.update(back_pos, 2)
        if self.measures.irSensor[left_id] > 1.2:
            self.map.update(left_pos, 1)
        else:
            self.map.update(left_pos, 2)
        if self.measures.irSensor[right_id] > 1.2:
            self.map.update(right_pos, 1)
        else:
            self.map.update(right_pos, 2)
        if self.map.get(self.position) != 9:
            self.map.update(self.position, 2)
        if self.measures.ground != -1:
            self.visited_targets[self.measures.ground] = self.position
            self.map.update(self.position, 3+self.measures.ground)
            print("Target: " + str(self.measures.ground) + " found at: " + str(self.position))
        # self.map.print()

    def set_target(self, target):
        # self.target = [target[0]+self.offset[0], target[1]+self.offset[1]]
        self.target = [target[0], target[1]]

    def recalculate_path(self):
        print("can't go forward, recalculate path")
        # print("Searching for new path from: " + str(self.position) + " to: " + str(self.global_target))
        self.nextTarget()
        self.path = self.map.search_path_a_star(self.position, self.global_target)
        if not self.path:
            print("no path next target")
            self.nextTarget()
            return
        elif self.path == self.prev_path:
            print("Stuck")
            self.map.print()
            self.state = 'stop'
            return
        else:
            print("New path found")
            self.prev_path = [x for x in self.path]
            self.target_position = self.path[0]
            self.path.pop(0)
        # print("Path found: " + str(self.path))
        #     if not self.path:
        #         print("No path found")
        #         self.state = 'stop'
        #         return



    def getNeighbors(self, step):
        if self.direction == 0:
            transformations = [[step, 0], [-step, 0], [0, step], [0, -step]]
        elif self.direction == 1:
            transformations = [[0, -step], [0, step], [step, 0], [-step, 0]]
        elif self.direction == 2:
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

    def nextTarget(self, close=False):
        old_target = self.global_target
        # self.map.print()
        f_pos, b_pos, l_pos, r_pos = self.getNeighbors(2)
        # f_pos2, b_pos2, l_pos2, r_pos2 = self.getNeighbors(1)
        adjacent = []
        others = []
        next_target = []
        for i in range(len(self.map.map)):
            if i % 2 == 0:
                continue
            for j in range(len(self.map.map[i])):
                if j % 2 == 0:
                    continue
                if self.map.map[i][j] == 0 and [i, j] not in self.hard_targets:
                    if self.is_adjacent_to_map([i,j]):
                        adjacent.append([i, j])
                    else:
                        others.append([i, j])
        # print(adjacent)
        if adjacent:
            unvisited = sorted(adjacent, key=self.sort_pos)
        else:
            unvisited = sorted(others, key=self.sort_pos)

        # if f_pos in adjacent+others and len(self.map.search_path_a_star(self.position, f_pos)) == 2:
        #     next_target = f_pos
        #     print("Front: " + str(f_pos))
        if r_pos in adjacent+others and len(self.map.search_path_a_star(self.position, r_pos)) == 2:
            next_target = r_pos
            print("Right")
        elif l_pos in adjacent+others and len(self.map.search_path_a_star(self.position, l_pos)) == 2:
            next_target = l_pos
            print("Left")
        elif f_pos in adjacent+others and len(self.map.search_path_a_star(self.position, f_pos)) == 2:
            next_target = f_pos
            print("Front: " + str(f_pos))
        elif b_pos in adjacent+others and len(self.map.search_path_a_star(self.position, b_pos)) == 2:
            next_target = b_pos
        elif not close:
            shortest_len = 99

            for pos in unvisited:
                path = self.map.search_path_a_star(self.position, pos)
                if not path:
                    self.hard_targets.append(pos)
                    continue
                if len(path) < shortest_len:
                    next_target = pos
                    shortest_len = len(path)
                # print(str(pos) + "  " + str(len(path)))
                if shortest_len < 10:
                    break
        else:
            return
        if not next_target:
            print("No more easy targets")
            possible_targets = [pos for pos in unvisited if pos in self.hard_targets]
            for pos in sorted(possible_targets, key=self.sort_pos):
                path = self.map.search_path_a_star(self.position, pos)
                if not path:
                    continue
                if len(path) < shortest_len:
                    next_target = pos
                    shortest_len = len(path)
                if shortest_len < 30:
                    break

        if not next_target:
            print("No more targets")
            self.map.print()
            if len(self.visited_targets.keys()) != int(self.nBeacons):
                print("NOT ALL TARGETS FOUND")

            if self.position == self.visited_targets[0]:
                if self.goal:
                    return
                self.compute_best_path()
                self.map.print_to_file("mapC4.out")
                self.map.print()
                self.driveMotors(0, 0)
                sys.exit(0)
            self.global_target = [27, 13]#self.visited_targets[0]
            print("Initial pos: " + str(self.visited_targets[0]))
            if not self.map.search_path_a_star(self.position, self.global_target):
                print("Can't go home :(")
                sys.exit()
            # self.path = []
            self.target_position = []
            print("Returning home")
            return
        if next_target == old_target:
            return
        self.global_target = next_target

        # self.path = []
        self.target_position = []
        print("New global target: " + str(self.global_target))
        self.map.print()

    def is_adjacent_to_map(self, pos):
        neig = getNeighbors(2, pos)
        visited = []
        for i in range(len(self.map.map)):
            for j in range(len(self.map.map[i])):
                if self.map.map[i][j] not in [0, 1]:
                    visited.append([i, j])
        return any([nei in visited for nei in neig])

    def compute_best_path(self):
        possible_paths = []
        # for i in range(int(self.nBeacons) - 1):
        #     total_path += self.map.search_path_a_star(self.visited_targets[i],
        #                                               self.visited_targets[i + 1])[:-1]
        # total_path += self.map.search_path_a_star(self.visited_targets[int(self.nBeacons) - 1],
        #                                           self.visited_targets[0])
        permutations = list(itertools.permutations(self.visited_targets.keys()))
        print("perm: " + str(permutations))
        for perm in permutations:
            print(perm)
            path =[]
            for i in range(len(perm)-1):
                path += self.map.search_path_a_star(self.visited_targets[perm[i]],
                                                          self.visited_targets[perm[i+1]])[:-1]
            path += self.map.search_path_a_star(self.visited_targets[perm[-1]],
                                                      self.visited_targets[perm[0]])
            print(path)
            possible_paths.append(path)
        total_path = possible_paths[0]
        for path in possible_paths:
            if len(path) < len(total_path):
                total_path = path
        adjusted_path = [[pos[0] - 27, pos[1] - 13] for pos in total_path]
        adjusted_targets = []
        for i in range(int(self.nBeacons)):
            pos = self.visited_targets[i]
            adjusted_targets.append([pos[0] - 27, pos[1] - 13])
        print(adjusted_targets)
        global outfile
        if not outfile:
            outfile = "C4.out"
        file = open(outfile, 'w')
        for pos in adjusted_path:
            s = str(pos[0]) + " " + str(pos[1])
            if pos in adjusted_targets and pos != [0, 0]:
                target = adjusted_targets.index(pos)
                s += " #" + str(target)
            s += '\n'
            file.write(s)


def getNeighbors(step, pos):
    transformations = [[0, step], [0, -step], [-step, 0], [step, 0]]
    front_pos = [pos[0] + transformations[0][0], pos[1] + transformations[0][1]]
    back_pos = [pos[0] + transformations[1][0], pos[1] + transformations[1][1]]
    left_pos = [pos[0] + transformations[2][0], pos[1] + transformations[2][1]]
    right_pos = [pos[0] + transformations[3][0], pos[1] + transformations[3][1]]
    return [front_pos, back_pos, left_pos, right_pos]

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()

        self.labMap = [[' '] * (CELLCOLS * 2 - 1) for i in range(CELLROWS * 2 - 1)]
        i = 1
        for child in root.iter('Row'):
            line = child.attrib['Pattern']
            row = int(child.attrib['Pos'])
            if row % 2 == 0:  # this line defines vertical lines
                for c in range(len(line)):
                    if (c + 1) % 3 == 0:
                        if line[c] == '|':
                            self.labMap[row][(c + 1) // 3 * 2 - 1] = '|'
                        else:
                            None
            else:  # this line defines horizontal lines
                for c in range(len(line)):
                    if c % 3 == 0:
                        if line[c] == '-':
                            self.labMap[row][c // 3 * 2] = '-'
                        else:
                            None

            i = i + 1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None
outfile = ""

for i in range(1, len(sys.argv), 2):
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
    rob = MyRob(rob_name, pos, [0.0, 90.0, -90.0, 180.0], host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()

    rob.run()
