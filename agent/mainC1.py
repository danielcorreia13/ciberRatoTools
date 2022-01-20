
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

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

        state = 'stop'
        stopped_state = 'run'
        self.ground = 0
        self.laps = 0
        while True:
            self.readSensors()
            self.move()

    def move(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        #print(self.measures.irSensor[right_id])
        #print(self.measures.irSensor[left_id])

        rSpeed = 0
        if self.measures.irSensor[center_id] > 2:
            if self.measures.irSensor[center_id] > 3.5:
                lSpeed = 0
            else:
                lSpeed = 0.05
            if self.measures.irSensor[left_id] > self.measures.irSensor[right_id]:
                print('Rotate right')
                rSpeed = 0.15
            else:
                print('Rotate left')
                rSpeed = -0.15
            print(rSpeed)
        elif abs(self.measures.irSensor[left_id] - self.measures.irSensor[right_id]) > 1.8\
            and (self.measures.irSensor[left_id] > 3\
            or self.measures.irSensor[right_id] > 3):
            print("Go and adjust")
            # if self.measures.irSensor[left_id] > self.measures.irSensor[right_id]:
            #     print('Center to right')
            #     rSpeed = 0.05
            # else:
            #     print('Center to left')
            #     rSpeed = -0.05
            rSpeed = 0.05 * (self.measures.irSensor[left_id] - self.measures.irSensor[right_id])
            #lSpeed = 1 - abs(self.measures.irSensor[left_id] - self.measures.irSensor[right_id])/1.5
            lSpeed =  0.2 * 1/(self.measures.irSensor[left_id] - self.measures.irSensor[right_id])
        elif self.measures.irSensor[left_id] < 1:
            print("Lean to left")
            lSpeed = 0.15
            rSpeed = -0.1
        elif self.measures.irSensor[right_id] < 1:
            print("Lean to right")
            lSpeed = 0.15
            rSpeed = 0.1
        else:
            print("Go")
            lSpeed = 0.15
            rSpeed = 0.03 * (self.measures.irSensor[left_id] - self.measures.irSensor[right_id])

        if lSpeed < 0:
            lSpeed = 0
        left_speed = lSpeed+rSpeed
        right_speed = lSpeed-rSpeed
        if left_speed > 0.15:
            left_speed = 0.15
        if right_speed > 0.15:
            right_speed = 0.15

        if self.measures.ground != -1 and self.measures.ground != self.ground:
            if self.measures.ground == 0:
                if self.ground != 2:
                    print("going back")
                else:
                    self.laps+=1
                    print("NEW LAP!!!")
                    print(str(self.laps)+" laps")
            elif self.measures.ground < self.ground:
                print("going back")
            self.ground = self.measures.ground

        # print("l: " + str(left_speed))
        # print("r: " + str(right_speed))
        self.driveMotors(left_speed, right_speed)


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

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
