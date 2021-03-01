from constants import Bearing
from collections import deque


INFINITE_COST = 9999
MOVE_COST = 10
TURN_COST = 20


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
            elif top_robot_center == goal and facing == Bearing.SOUTH:
                move_cmd.append('RL')
                move_cmd.append('RL')
                current_bearing = Bearing.SOUTH

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
            elif bottom_robot_center == goal and facing == Bearing.NORTH:
                move_cmd.append('RL')
                move_cmd.append('RL')
                current_bearing = Bearing.NORTH

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
            elif left_robot_center == goal and facing == Bearing.EAST:
                move_cmd.append('RL')
                move_cmd.append('RL')
                current_bearing = Bearing.EAST

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
            elif right_robot_center == goal and facing == Bearing.WEST:
                move_cmd.append('RL')
                move_cmd.append('RL')
                current_bearing = Bearing.WEST

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


# A* utility function: get all the unexplored grid
def get_all_unexplored_grid(explored_map):
    list_of_coordinates = list()
    for row in range(0, len(explored_map)):
        for col in range(0, len(explored_map[row])):
            if explored_map[row][col] == 0:
                list_of_coordinates.append([row, col])
    return list_of_coordinates


# A* utility function: get the nearest unexplored grid to the robot
# current_robot_center is in xy, list_of_coordinates is in row, col
def find_nearest_unexplored_grid(list_of_coordinates, current_robot_center):
    min_index = 0
    min_distance = (abs(current_robot_center[0] - 1 - list_of_coordinates[min_index][1]) * 10) + \
                   (abs(current_robot_center[1] - 1 - list_of_coordinates[min_index][0]) * 10)
    for i in range(1, len(list_of_coordinates)):
        distance = (abs(current_robot_center[0] - 1 - list_of_coordinates[i][1]) * 10) + \
                   (abs(current_robot_center[1] - 1 - list_of_coordinates[i][0]) * 10)
        if distance < min_distance:
            min_distance = distance
            min_index = i
    return list_of_coordinates[min_index]


def find_nearest_obstacle(unexplored_grid_coordinate, obstacle_map):
    found = False
    lower_bound = 0
    upper_bound = 1
    distance = 0
    while not found:
        lower_bound = lower_bound - 1
        upper_bound = upper_bound + 1
        distance = distance + 1
        if distance > 20:   # it should not reach here
            return None
        for row in range(unexplored_grid_coordinate[0] + lower_bound, unexplored_grid_coordinate[0] + upper_bound):
            if 0 <= row <= 19:
                for col in range(unexplored_grid_coordinate[1] + lower_bound, unexplored_grid_coordinate[1] + upper_bound):
                    if 0 <= col <= 14:
                        if obstacle_map[row][col] == 1:
                            return [row, col]


def get_surrounding_obstacle(obstacle_coordinate, explored_map, obstacle_map):
    list_of_coordinates = list()
    coordinates = [
        [obstacle_coordinate[0] + 1, obstacle_coordinate[1]],   # top
        [obstacle_coordinate[0] - 1, obstacle_coordinate[1]],   # bottom
        [obstacle_coordinate[0], obstacle_coordinate[1] + 1],   # right
        [obstacle_coordinate[0], obstacle_coordinate[1] - 1],   # left
    ]
    for coordinate in coordinates:
        row = coordinate[0]
        col = coordinate[1]
        if 0 <= row <= 19 and 0 <= col <= 14:
            if explored_map[row][col] == 1 and obstacle_map[row][col] == 0:
                if coordinate == coordinates[0]:
                    list_of_coordinates.append({
                        'robot_pos': [col + 1, row + 2],
                        'bearing': Bearing.WEST
                    })
                elif coordinate == coordinates[1]:
                    list_of_coordinates.append({
                        'robot_pos': [col + 1, row],
                        'bearing': Bearing.EAST
                    })
                elif coordinate == coordinates[2]:
                    list_of_coordinates.append({
                        'robot_pos': [col, row + 1],
                        'bearing': Bearing.SOUTH
                    })
                else:
                    list_of_coordinates.append({
                        'robot_pos': [col + 2, row + 1],
                        'bearing': Bearing.NORTH
                    })
    return list_of_coordinates


# list_of_coordinates and current_robot_position is in xy
def find_nearest_goal(list_of_coordinates, current_robot_center):
    min_index = 0
    min_distance = (abs(current_robot_center[0] - list_of_coordinates[0]['robot_pos'][0]) * 10) + \
                   (abs(current_robot_center[1] - list_of_coordinates[0]['robot_pos'][1]) * 10)
    for i in range(1, len(list_of_coordinates)):
        distance_to_obstacle = (abs(current_robot_center[0] - list_of_coordinates[i]['robot_pos'][0]) * 10) + \
                               (abs(current_robot_center[1] - list_of_coordinates[i]['robot_pos'][1]) * 10)
        if distance_to_obstacle < min_distance:
            min_index = i
            min_distance = distance_to_obstacle
    return list_of_coordinates[min_index]


def get_nearest_goal(explored_map, obstacle_map, current_robot_center):
    list_of_unexplored_grid = get_all_unexplored_grid(explored_map)
    nearest_unexplored_grid = find_nearest_unexplored_grid(list_of_unexplored_grid, current_robot_center)
    nearest_obstacle = find_nearest_obstacle(nearest_unexplored_grid, obstacle_map)
    surrounding_obstacle_grid = get_surrounding_obstacle(nearest_obstacle, explored_map, obstacle_map)
    nearest_goal = find_nearest_goal(surrounding_obstacle_grid, current_robot_center)
    return nearest_goal
