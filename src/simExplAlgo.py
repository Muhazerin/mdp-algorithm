from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
import time
from enum import IntEnum
from collections import Counter, deque
from constants import Bearing

# TODO: Robot needs to know that it reaches GOAL
# TODO: Robot needs to know when to stop
#  (i.e. after reaching GOAL at least once, if it reaches START, it will stop)
#  (idea: if coverage < user specified coverage, change exploration algo from left wall hugging to seek unexplored)
TIME = 0.01

INFINITE_COST = 9999
MOVE_COST = 10
TURN_COST = 20


class AlgoStatus(IntEnum):
    SEEK_GOAL = 0
    SEEK_HOME = 1
    FP_UNEXPLORED = 2
    FP_HOME = 3


class Node:
    def __init__(self, robot_center, robot_bearing, parent=None, g=INFINITE_COST, h=INFINITE_COST, move_cmd=None):
        self.__robot_center = robot_center
        self.__robot_bearing = robot_bearing
        self.__parent = parent
        self.__g = g
        self.__h = h
        self.__move_cmd = move_cmd

    @property
    def robot_center(self):
        return self.__robot_center

    @property
    def robot_bearing(self):
        return self.__robot_bearing

    @property
    def parent(self):
        return self.__parent

    @property
    def g(self):
        return self.__g

    @property
    def h(self):
        return self.__h

    @property
    def move_cmd(self):
        return self.__move_cmd

    def get_f(self):
        return self.__g + self.__h

    def __eq__(self, other):
        if other is None:
            return False
        else:
            return self.robot_center[0] == other.robot_center[0] and \
                   self.robot_center[1] == other.robot_center[1]


# A* utility function: Returns a char or None: 'B', 'L', 'R', 'T', None
def is_unexplored_cell_valid(row, col, explored_map, obstacle_map):
    # Check bottom
    valid = True
    for r in range(row - 1, row - 4, -1):
        if valid:
            if r < 0:
                valid = False
                break
            else:
                for c in range(col - 1, col + 2):
                    if 14 < c < 0 or explored_map[r][c] == 0 or obstacle_map[r][c] == 1:
                        valid = False
                        break
    if valid:
        return 'B'
    # Check left
    valid = True
    for c in range(col - 1, col - 4, -1):
        if valid:
            if c < 0:
                valid = False
                break
            else:
                for r in range(row - 1, row + 2):
                    if 19 < r < 0 or explored_map[r][c] == 0 or obstacle_map[r][c] == 1:
                        valid = False
                        break
    if valid:
        return 'L'
    # Check right
    valid = True
    for c in range(col + 1, col + 4):
        if valid:
            if c > 14:
                valid = False
                break
            else:
                for r in range(row - 1, row + 2):
                    if 19 < r < 0 or explored_map[r][c] == 0 or obstacle_map[r][c] == 1:
                        valid = False
                        break
    if valid:
        return 'R'
    # Check top
    valid = True
    for r in range(row + 1, row + 4):
        if valid:
            if r > 19:
                valid = False
                break
            else:
                for c in range(col - 1, col + 2):
                    if 14 < c < 0 or explored_map[r][c] == 0 or obstacle_map[r][c] == 1:
                        valid = False
                        break
    if valid:
        return 'R'
    # if code reach here, it means the unexplored cell is unreachable
    return None


# A* utility function: find an unexplored part of the map that is explorable
def find_valid_unexplored(explored_map, obstacle_map):
    goal = None
    facing = None
    for row in range(0, 19):
        for col in range(0, 15):
            if explored_map[row][col] == 0:
                valid = is_unexplored_cell_valid(row, col, explored_map, obstacle_map)
                if valid is not None:
                    if valid == 'B':
                        goal = [(col + 1), (row + 1) - 2]
                        facing = 0
                    elif valid == 'L':
                        goal = [(col + 1) - 2, (row + 1)]
                        facing = 2
                    elif valid == 'R':
                        goal = [(col + 1) + 2, (row + 1)]
                        facing = 6
                    else:
                        goal = [(col + 1), (row + 1) + 2]
                        facing = 4
                    return goal, facing
    # if the code reach here, it means there's an unreachable unexplored cell
    print('Error! Unreachable unexplored cell')
    return goal, facing


# A* utility function: get the index of the node with the smallest f in open_list
def get_smallest_f_node_index(open_list):
    index = 0
    smallest = open_list[index].get_f()
    for i in range(1, len(open_list)):
        if open_list[i].get_f() < smallest:
            smallest = open_list[i].get_f()
            index = i
    return index


# A* utility function: returns a boolean whether the robot center is bounded in the map
def is_robot_center_valid(x, y):
    if 1 < x < 15 and 1 < y < 20:
        return True
    else:
        return False


# A* utility function: returns a boolean whether on the current robot position,
#   the robot clash if any obstacle or unexplored map
def is_robot_clash_obstacle(x, y, direction, explored_map, obstacle_map):
    if direction == 'top':
        y = y + 1
        for x2 in range(x - 1, x + 2):
            if explored_map[y - 1][x2 - 1] == 0 or obstacle_map[y - 1][x2 - 1] == 1:
                return True
        return False
    elif direction == 'left':
        x = x - 1
        for y2 in range(y - 1, y + 2):
            if explored_map[y2 - 1][x - 1] == 0 or obstacle_map[y2 - 1][x - 1] == 1:
                return True
        return False
    elif direction == 'right':
        x = x + 1
        for y2 in range(y - 1, y + 2):
            if explored_map[y2 - 1][x - 1] == 0 or obstacle_map[y2 - 1][x - 1] == 1:
                return True
        return False
    else:
        y = y - 1
        for x2 in range(x - 1, x + 2):
            if explored_map[y - 1][x2 - 1] == 0 or obstacle_map[y - 1][x2 - 1] == 1:
                return True
        return False


# A* utility function: return the Manhattan distance
def get_manhattan_distance(robot_center, goal):
    return (abs(robot_center[0] - goal[0]) * 10) + (abs(robot_center[1] - goal[1]) * 10)


# A* utility function: generate the move cmd after the search
def gen_move_cmd(dest_node):
    node = dest_node
    path = deque([])
    while True:
        if node.parent is None:
            break
        node.move_cmd.reverse()
        for move in node.move_cmd:
            path.appendleft(move)
        node = node.parent
    return path


# A* search to the goal
def a_star_search(starting_robot_center, goal, facing, starting_robot_bearing, explored_map, obstacle_map):
    closed_list = deque([])
    open_list = deque([])
    start_node = Node(starting_robot_center, starting_robot_bearing, parent=None, g=0, h=0)
    open_list.append(start_node)

    while open_list:  # while the open_list is not empty
        # find the node with the smallest f and pop it from open_list
        q = open_list[get_smallest_f_node_index(open_list)]
        open_list.remove(q)
        closed_list.append(q)

        # generate smallest_node 4 successors (top, left, right, bottom)
        # top successor
        top_robot_center = [q.robot_center[0], q.robot_center[1] + 1]
        if is_robot_center_valid(top_robot_center[0], top_robot_center[1]) and not \
                is_robot_clash_obstacle(top_robot_center[0], top_robot_center[1], 'top', explored_map,
                                        obstacle_map):
            if q.robot_bearing == Bearing.NORTH:
                move_cmd = ['F']
                g = MOVE_COST
            elif q.robot_bearing == Bearing.EAST:
                move_cmd = ['RL', 'F']
                g = TURN_COST + MOVE_COST
            elif q.robot_bearing == Bearing.SOUTH:
                move_cmd = ['RL', 'RL', 'F']
                g = TURN_COST + TURN_COST + MOVE_COST
            else:
                move_cmd = ['RR', 'F']
                g = TURN_COST + MOVE_COST

            current_bearing = Bearing.NORTH
            if top_robot_center == goal and facing == Bearing.EAST:
                move_cmd.append('RR')
                current_bearing = Bearing.EAST
            elif top_robot_center == goal and facing == Bearing.WEST:
                move_cmd.append('RL')
                current_bearing = Bearing.WEST

            top_node = Node(top_robot_center, current_bearing, parent=q, g=g,
                            h=get_manhattan_distance(top_robot_center, goal), move_cmd=move_cmd)
            if top_robot_center == goal:
                return top_node
            else:
                # If the successor node is in the closed list, ignore it
                found_in_closed = False
                for node in closed_list:
                    if node == top_node:
                        found_in_closed = True
                        break
                # Else, do the following
                if not found_in_closed:
                    # Check if the successor node is inside open_list
                    found_in_open = False
                    index = 0
                    for node in open_list:
                        if node == top_node:
                            found_in_open = True
                            break
                        index = index + 1
                    # If the successor node is not in the open list, add it
                    if not found_in_open:
                        open_list.append(top_node)
                    else:  # Check the f
                        existing_node = open_list[index]
                        # if successor f < existing f, update the node to successor node
                        if existing_node.get_f() > top_node.get_f():
                            open_list[index] = top_node

        # bottom successor
        bottom_robot_center = [q.robot_center[0], q.robot_center[1] - 1]
        if is_robot_center_valid(bottom_robot_center[0], bottom_robot_center[1]) and not \
                is_robot_clash_obstacle(bottom_robot_center[0], bottom_robot_center[1], 'bottom', explored_map,
                                        obstacle_map):
            if q.robot_bearing == Bearing.NORTH:
                move_cmd = ['RL', 'RL', 'F']
                g = TURN_COST + TURN_COST + MOVE_COST
            elif q.robot_bearing == Bearing.EAST:
                move_cmd = ['RR', 'F']
                g = TURN_COST + MOVE_COST
            elif q.robot_bearing == Bearing.SOUTH:
                move_cmd = ['F']
                g = MOVE_COST
            else:
                move_cmd = ['RL', 'F']
                g = TURN_COST + MOVE_COST

            current_bearing = Bearing.SOUTH
            if bottom_robot_center == goal and facing == Bearing.EAST:
                move_cmd.append('RL')
                current_bearing = Bearing.EAST
            elif bottom_robot_center == goal and facing == Bearing.WEST:
                move_cmd.append('RR')
                current_bearing = Bearing.WEST

            bottom_node = Node(bottom_robot_center, current_bearing, parent=q, g=g,
                               h=get_manhattan_distance(bottom_robot_center, goal), move_cmd=move_cmd)
            if bottom_robot_center == goal:
                return bottom_node
            else:
                # If the successor node is in the closed list, ignore it
                found_in_closed = False
                for node in closed_list:
                    if node == bottom_node:
                        found_in_closed = True
                        break
                # Else, do the following
                if not found_in_closed:
                    # Check if the successor node is inside open_list
                    found_in_open = False
                    index = 0
                    for node in open_list:
                        if node == bottom_node:
                            found_in_open = True
                            break
                        index = index + 1
                    # If the successor node is not in the open list, add it
                    if not found_in_open:
                        open_list.append(bottom_node)
                    else:  # Else , check the f
                        existing_node = open_list[index]
                        # if successor f < existing f, update the node to successor node
                        if existing_node.get_f() > bottom_node.get_f():
                            open_list[index] = bottom_node

        # left successor
        left_robot_center = [q.robot_center[0] - 1, q.robot_center[1]]
        if is_robot_center_valid(left_robot_center[0], left_robot_center[1]) and not \
                is_robot_clash_obstacle(left_robot_center[0], left_robot_center[1], 'left', explored_map,
                                        obstacle_map):
            if q.robot_bearing == Bearing.NORTH:
                move_cmd = ['RL', 'F']
                g = TURN_COST + MOVE_COST
            elif q.robot_bearing == Bearing.EAST:
                move_cmd = ['RL', 'RL', 'F']
                g = TURN_COST + TURN_COST + MOVE_COST
            elif q.robot_bearing == Bearing.SOUTH:
                move_cmd = ['RR', 'F']
                g = TURN_COST + MOVE_COST
            else:
                move_cmd = ['F']
                g = MOVE_COST

            current_bearing = Bearing.WEST
            if left_robot_center == goal and facing == Bearing.NORTH:
                move_cmd.append('RR')
                current_bearing = Bearing.NORTH
            elif left_robot_center == goal and facing == Bearing.SOUTH:
                move_cmd.append('RL')
                current_bearing = Bearing.SOUTH

            left_node = Node(left_robot_center, current_bearing, parent=q, g=g,
                             h=get_manhattan_distance(left_robot_center, goal), move_cmd=move_cmd)
            if left_robot_center == goal:
                return left_node
            else:
                # If the successor node is in the closed list, ignore it
                found_in_closed = False
                for node in closed_list:
                    if node == left_node:
                        found_in_closed = True
                        break
                # Else, do the following
                if not found_in_closed:
                    # Check if the successor node is inside open_list
                    found_in_open = False
                    index = 0
                    for node in open_list:
                        if node == left_node:
                            found_in_open = True
                            break
                        index = index + 1
                    # If the successor node is not in the open list, add it
                    if not found_in_open:
                        open_list.append(left_node)
                    else:  # Else , check the f
                        existing_node = open_list[index]
                        # if successor f < existing f, update the node to successor node
                        if existing_node.get_f() > left_node.get_f():
                            open_list[index] = left_node

        # right successor
        right_robot_center = [q.robot_center[0] + 1, q.robot_center[1]]
        if is_robot_center_valid(right_robot_center[0], right_robot_center[1]) and not \
                is_robot_clash_obstacle(right_robot_center[0], right_robot_center[1], 'right', explored_map,
                                        obstacle_map):
            if q.robot_bearing == Bearing.NORTH:
                move_cmd = ['RR', 'F']
                g = TURN_COST + MOVE_COST
            elif q.robot_bearing == Bearing.EAST:
                move_cmd = ['F']
                g = MOVE_COST
            elif q.robot_bearing == Bearing.SOUTH:
                move_cmd = ['RL', 'F']
                g = TURN_COST + MOVE_COST
            else:
                move_cmd = ['RL', 'RL', 'F']
                g = TURN_COST + TURN_COST + MOVE_COST

            current_bearing = Bearing.EAST
            if right_robot_center == goal and facing == Bearing.NORTH:
                move_cmd.append('RL')
                current_bearing = Bearing.NORTH
            elif right_robot_center == goal and facing == Bearing.SOUTH:
                move_cmd.append('RR')
                current_bearing = Bearing.SOUTH

            right_node = Node(right_robot_center, current_bearing, parent=q, g=g,
                              h=get_manhattan_distance(right_robot_center, goal), move_cmd=move_cmd)
            if right_robot_center == goal:
                return right_node
            else:
                # If the successor node is in the closed list, ignore it
                found_in_closed = False
                for node in closed_list:
                    if node == right_node:
                        found_in_closed = True
                        break
                # Else, do the following
                if not found_in_closed:
                    # Check if the successor node is inside open_list
                    found_in_open = False
                    index = 0
                    for node in open_list:
                        if node == right_node:
                            found_in_open = True
                            break
                        index = index + 1
                    # If the successor node is not in the open list, add it
                    if not found_in_open:
                        open_list.append(right_node)
                    else:  # Else , check the f
                        existing_node = open_list[index]
                        # if successor f < existing f, update the node to successor node
                        if existing_node.get_f() > right_node.get_f():
                            open_list[index] = right_node
    return None


class SimExplAlgo(QObject):
    finished = pyqtSignal()
    signalSense = pyqtSignal()
    signalMoveRobotForward = pyqtSignal()
    signalMoveRobotBackward = pyqtSignal()
    signalRotateRobotRight = pyqtSignal()
    signalRotateRobotLeft = pyqtSignal()

    def __init__(self):
        super(SimExplAlgo, self).__init__()
        self.__algoStatus = AlgoStatus.SEEK_GOAL
        self.__robotJustTurnedLeft = False
        self.__stop = False
        self.signalSense.emit()

    @pyqtSlot(dict, list, list, list, int)
    def determineMove(self, frontLeftDict, allCorners, exploredMap, obstacleMap, robotBearing):
        print()
        if self.__algoStatus == AlgoStatus.SEEK_GOAL and [15, 19] in allCorners:
            self.__algoStatus = AlgoStatus.SEEK_HOME
        elif self.__algoStatus == AlgoStatus.SEEK_HOME and [-1, 0] in allCorners:
            # Count the explored map coverage
            coun = Counter()
            for row in exploredMap:
                coun.update(row)

            if coun[1] < 300:  # TODO: Will change to estimated coverage
                self.__algoStatus = AlgoStatus.FP_UNEXPLORED
            else:
                self.__stop = True

        if not self.__stop:
            if self.__algoStatus != AlgoStatus.FP_UNEXPLORED:
                time.sleep(TIME)
                if not self.__robotJustTurnedLeft:
                    if frontLeftDict['L'] == 1 and frontLeftDict['F'] == 0:  # if left is not free and front is free
                        self.signalMoveRobotForward.emit()
                        self.signalSense.emit()
                    elif frontLeftDict['L'] == 1 and frontLeftDict[
                        'F'] == 1:  # if left is not free and front is not free
                        self.signalRotateRobotRight.emit()
                        self.signalSense.emit()
                    elif frontLeftDict['L'] == 0:  # if left is free, turn left to hug the left wall
                        self.__robotJustTurnedLeft = True
                        self.signalRotateRobotLeft.emit()
                        self.signalSense.emit()
                else:
                    self.__robotJustTurnedLeft = False
                    self.signalMoveRobotForward.emit()
                    self.signalSense.emit()
            else:
                goal, facing = find_valid_unexplored(exploredMap, obstacleMap)
                if goal is None or facing is None:
                    self.__stop = True
                    self.finished.emit()
                else:
                    dest_node = a_star_search([allCorners[0][0] + 2, allCorners[0][1] - 1], goal, facing, robotBearing,
                                              exploredMap, obstacleMap)
                    if dest_node is None:
                        print("WTF!! HOW CAN A STAR SEARCH RETURNS NONE!!")
                    else:
                        move_cmd = gen_move_cmd(dest_node)
                        for cmd in move_cmd:
                            time.sleep(TIME * 2)
                            if cmd == 'RR':
                                self.signalRotateRobotRight.emit()
                            elif cmd == 'RL':
                                self.signalRotateRobotLeft.emit()
                            else:
                                self.signalMoveRobotForward.emit()
                self.__stop = True
                self.finished.emit()
        else:
            self.finished.emit()

    def run(self):
        self.__stop = False
        self.__algoStatus = AlgoStatus.SEEK_GOAL
        self.signalSense.emit()
