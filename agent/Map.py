import sys

def manhattan(a, b):
    return sum(abs(val1-val2) for val1, val2 in zip(a,b))

class Node():
    def __init__(self, parent, pos):
        self.parent = parent
        self.pos = pos
        self.g = 0  # distance to start
        self.h = 0  # heuristic
        self.f = 0  # cost

    def __eq__(self, other):
        #return self.pos[0] == other.pos[0] and self.pos[1] == other.pos[1]
        return self.pos == other.pos

    def __str__(self):
        return str(self.pos)


class MyMap():
    def __init__(self, n_lines=27, n_columns=56):
        self.n_lines = n_lines
        self.n_columns = n_columns
        self.map = [[0 for x in range(n_lines)] for x in range(n_columns)]

    def update(self, pos, value):
        self.map[pos[0]][pos[1]] = value

    def get(self, pos):
        return self.map[pos[0]][pos[1]]

    def search_path_a_star(self, start, target):
        start_node = Node(None, start)
        end_node = Node(None, target)

        open_nodes = [start_node]
        closed_nodes = []

        while len(open_nodes) > 0:
            if len(open_nodes) > 1000:
                print("Path to long, skiping  " + str(target))
                return []
            node = open_nodes[0]
            current_index = 0

            #get node with least cost
            for index, item in enumerate(open_nodes):
                if item.f < node.f:
                    node = item
                    current_index = index

            open_nodes.pop(current_index)
            closed_nodes.append(node)
            #if found target
            if node == end_node:
                path = []
                current = node
                # get path
                while current is not None:
                    path.append(current.pos)
                    current = current.parent
                return path[::-1]  # return reversed path

            # compute all possible children
            children = []
            for new_position in [(0, -2), (0, 2), (-2, 0), (2, 0)]:#, (-1, -1), (-1, 1), (1, -1), (1, 1)]:  # Adjacent squares

                new_node_position = [node.pos[0] + new_position[0], node.pos[1] + new_position[1]]
                new_node_middle_pos = [node.pos[0] + int(new_position[0]/2), node.pos[1] + int(new_position[1]/2)]
                # make sure in map
                if new_node_position[0] > self.n_columns-1 or new_node_position[0] < 0 or\
                        new_node_position[1] > self.n_lines-1 or new_node_position[1] < 0:
                    continue

                #make sure not too far
                # if manhattan(new_node_position, start) > 30:
                #     continue

                # Make sure walkable terrain
                if self.map[new_node_position[0]][new_node_position[1]] in [1] or\
                   self.map[new_node_middle_pos[0]][new_node_middle_pos[1]] in [1]:
                    continue
                # Create new node
                new_node = Node(node, new_node_position)
                children.append(new_node)

            for child in children:
                flag = False
                for closed in closed_nodes:
                    if child == closed:
                        flag = True
                        break
                if flag:
                    continue
                child.g = node.g + 2  # add 1 to new node distance to start
                # calculate child heuristic
                #child.h = ((child.pos[0] - end_node.pos[0]) ** 2) + ((child.pos[1] - end_node.pos[1]) ** 2)
                child.h = manhattan(child.pos, end_node.pos)
                # cost = g+h
                child.f = child.g + child.h

                flag = False
                for open_node in open_nodes:
                    if child == open_node and child.g > open_node.g:
                        flag = True
                        break
                if flag:
                    continue
                open_nodes.append(child)
        # print("No path found: " + str(start) + " -> " + str(target))
        # self.print()
        return []

    def print(self):
        transposed = list(map(list, zip(*self.map)))
        for i in range(len(transposed)):
            s = ""
            for item in transposed[-(i+1)]:
                s+=str(item)
            print(s)

    def print_to_file(self, filename):
        transposed = list(map(list, zip(*self.map)))
        outfile = open(filename, 'w')
        for i in range(len(transposed)):
            s = ""
            for item in transposed[-(i + 1)]:
                if item == 1:
                    if i % 2 == 0:
                        s += symbols[item][0]
                    else:
                        s += symbols[item][1]
                elif item >= 3:
                    s += item-3
                else:
                    s += symbols[item]
            outfile.write(s[:-1]+'\n')


symbols = {0: ' ',
           1: ['-', '|'],
           2: 'X',
           9: 'I'}






