from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
from collections import deque
import heapq
import time

from PyQt5.QtWidgets import QGraphicsGridLayout

from src.constants import Bearing

TIME = 0.1
CARDINAL = 10
DIAGONAL = 14


class ActlFastPathAlgo(QObject):
    signalSendCmd = pyqtSignal(str)
    finished = pyqtSignal()

    def __init__(self):
        super(ActlFastPathAlgo, self).__init__()
        self.__stop = False
        self._fastestPath = []

    # read in p2 string and return 15x20 grid
    def process_string(p2):
        grid = [[0 for _ in range(15)] for _ in range(20)]
        p2_bin = bin(int(p2, 16))[2:]
        while len(p2_bin) < 300:
            p2_bin = '0' + p2_bin
        for i in range(20):
            for j in range(15):
                grid[i][j] = int(p2_bin[i * 15 + j])
        return grid

    # expand obstacles in grid
    def preprocess(grid):
        q = deque([])
        for i in range(len(grid)):
            for j in range(len(grid[0])):
                if grid[i][j] == 1:
                    q.append((i, j))
        for cell in q:
            for i in range(-1, 2):
                for j in range(-1, 2):
                    if 0 <= cell[0] + i < len(grid) and 0 <= cell[1] + j < len(grid[0]):
                        grid[cell[0] + i][cell[1] + j] = 1
        for i in range(20):
            grid[i][0] = 1
            grid[i][-1] = 1
        for j in range(15):
            grid[0][j] = 1
            grid[-1][j] = 1

    # IF 8-DIRECTIONAL MOVEMENT (HEURISTIC)
    # def compute_octile_h(node, goal=(18,13)):
    #     i, j = node
    #     if grid[i][j] == 1:
    #         return float('inf')
    #     x_dist = abs(node[0]-goal[0])
    #     y_dist = abs(node[1]-goal[1])
    #     return round(CARDINAL*max(x_dist, y_dist) + (DIAGONAL-CARDINAL)*min(x_dist, y_dist), 1)

    # heuristic for 4-directional movement
    def compute_manhattan_h(grid, node, goal=(18, 13)):
        i, j = node
        if grid[i][j] == 1:
            return float('inf')
        x_dist = abs(node[0] - goal[0])
        y_dist = abs(node[1] - goal[1])
        return CARDINAL * (x_dist + y_dist)

    # bearing calculation in 360 degrees
    def get_bearing(src, dst):
        if src[0] == dst[0]:
            if src[1] == dst[1] + 1:
                return 270
            return 90
        elif src[1] == dst[1]:
            if src[0] == dst[0] + 1:
                return 180
            return 0
        # IF 8-DIRECTIONAL MOVEMENT
        # elif src[0] == dst[0]+1:
        #     if src[1] == dst[1]+1:
        #         return 225
        #     return 315
        # elif src[0] == dst[0]-1:
        #     if src[0] == dst[0]+1:
        #         return 45
        #     return 135

    def turn_cost(prev, new):
        diff = abs(new - prev)
        diff = min(diff, 360 - diff)
        return diff / 4.5

    # implementation of A* algorithm
    def a_star(grid, start=(1, 1), goal=(18, 13)):
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {}
        cost_so_far = {}
        bearing = {}
        came_from[start] = None
        cost_so_far[start] = 0
        bearing[start] = 0
        while frontier:
            cur = heapq.heappop(frontier)[1]
            if cur == goal:
                break
            for i in range(-1, 2):
                for j in range(-1, 2):
                    x = cur[0] + i
                    y = cur[1] + j
                    if not 0 <= x < 20 or not 0 <= y < 15:
                        continue
                    neighbor = (x, y)
                    if i == 0 or j == 0:
                        move_cost = CARDINAL
                    # to activate if using 8-directional movement
                    else:
                        continue
                    new_bearing = ActlFastPathAlgo.get_bearing(cur, neighbor)
                    new_cost = cost_so_far[cur] + move_cost + ActlFastPathAlgo.turn_cost(bearing[cur], new_bearing)
                    if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                        cost_so_far[neighbor] = new_cost
                        # change heuristic if using 8-directional movement
                        priority = new_cost + ActlFastPathAlgo.compute_manhattan_h(grid, neighbor)
                        heapq.heappush(frontier, (priority, neighbor))
                        came_from[neighbor] = cur
                        bearing[neighbor] = new_bearing
        return came_from, cost_so_far, bearing

    # generate path from start to waypoint or waypoint to goal
    def gen_half_path(self, grid, start, goal):
        came_from, cost_so_far, bearing = ActlFastPathAlgo.a_star(grid, start, goal)
        to = goal
        path = [to]
        cost = [cost_so_far[to]]
        direction = [bearing[to]]
        while came_from[to]:
            to = came_from[to]
            path.append(to)
            cost.append(cost_so_far[to])
            direction.append(bearing[to])
        # path = [pos for pos in reversed(path)] # for debug
        cost = [pos for pos in reversed(cost)]  # for debug
        direction = [pos for pos in reversed(direction)]
        return direction, cost[-1]

    # in case waypoint is in cell next to obstacle (i.e. inside expanded obstacle wall)
    def check_waypoint(grid, waypoint):
        waypoint = (waypoint[1], waypoint[0])
        x, y = waypoint
        # uncomment depending on algorithm
        # if grid[x][y] == 0:
        #     return [waypoint]
        res = []
        # get all possible new waypoint locations
        for i in range(-1, 2):
            for j in range(-1, 2):
                if grid[x + i][y + j] == 0:
                    res.append((x + i, y + j))
        return res

    # code for calibration
    # def add_calibration(grid, pos, bearing, commands):
    #     try:
    #         if bearing == Bearing.NORTH:
    #             if (grid[pos[1] + 1][pos[0] - 1] == 1 and grid[pos[1] + 1][pos[0]] == 1 and grid[pos[1] + 1][
    #                 pos[0] + 1] == 1) or pos[1] + 1 >= len(grid):
    #                 commands.append("f")
    #             elif (grid[pos[1] - 1][pos[0] - 1] == 1 and grid[pos[1]][pos[0] - 1] == 1 and grid[pos[1] + 1][
    #                 pos[0] - 1] == 1) or pos[0] - 1 < 0:
    #                 commands.append("s")
    #             elif (grid[pos[1] - 1][pos[0] + 1] == 1 and grid[pos[1]][pos[0] + 1] == 1 and grid[pos[1] + 1][
    #                 pos[0] + 1] == 1) or pos[0] + 1 >= len(grid[0]):
    #                 commands.append("r")
    #                 commands.append("f")
    #                 commands.append("l")
    #         elif bearing == Bearing.SOUTH:
    #             if (grid[pos[1] - 1][pos[0] - 1] and grid[pos[1] - 1][pos[0]] and grid[pos[1] - 1][pos[0] + 1] == 1) or pos[
    #                 1] - 1 < 0:
    #                 commands.append("f")
    #             elif (grid[pos[1] - 1][pos[0] + 1] == 1 and grid[pos[1]][pos[0] + 1] == 1 and grid[pos[1] + 1][
    #                 pos[0] + 1] == 1) or pos[0] + 1 >= len(grid[0]):
    #                 commands.append("s")
    #             elif (grid[pos[1] - 1][pos[0] - 1] == 1 and grid[pos[1]][pos[0] - 1] == 1 and grid[pos[1] + 1][
    #                 pos[0] - 1] == 1) or pos[0] + 1 < 0:
    #                 commands.append("r")
    #                 commands.append("f")
    #                 commands.append("l")
    #         elif bearing == Bearing.EAST:
    #             if (grid[pos[1] - 1][pos[0] + 1] == 1 and grid[pos[1]][pos[0] + 1] == 1 and grid[pos[1] + 1][
    #                 pos[0] + 1] == 1) or pos[0] + 1 >= len(grid[0]):
    #                 commands.append("f")
    #             elif (grid[pos[1] + 1][pos[0] - 1] == 1 and grid[pos[1] + 1][pos[0]] == 1 and grid[pos[1] + 1][
    #                 pos[0] + 1] == 1) or pos[1] + 1 >= len(grid):
    #                 commands.append("s")
    #             elif (grid[pos[1] - 1][pos[0] - 1] == 1 and grid[pos[1] - 1][pos[0]] == 1 and grid[pos[1] - 1][
    #                 pos[0] + 1]) == 1 or pos[1] - 1 < 0:
    #                 commands.append("r")
    #                 commands.append("f")
    #                 commands.append("l")
    #         elif bearing == Bearing.WEST:
    #             if (grid[pos[1] - 1][pos[0] - 1] and grid[pos[1]][pos[0] - 1] and grid[pos[1] + 1][pos[0] - 1]) == 1 or pos[
    #                 0] - 1 < 0:
    #                 commands.append("f")
    #             elif (grid[pos[1] - 1][pos[0] - 1] == 1 and grid[pos[1] - 1][pos[0]] == 1 and grid[pos[1] - 1][
    #                 pos[0] + 1] == 1) or pos[1] - 1 < 0:
    #                 commands.append("s")
    #             elif (grid[pos[1] + 1][pos[0] - 1] == 1 and grid[pos[1] + 1][pos[0]] == 1 and grid[pos[1] + 1][
    #                 pos[0] + 1] == 1) or pos[1] + 1 >= len(grid):
    #                 commands.append("r")
    #                 commands.append("f")
    #                 commands.append("l")
    #     except Exception as err:
    #         print(f'add_calibration error: {err}')

    def add_calibration_2(self, obstacle_map, pos, bearing, fPath):
        if bearing == Bearing.NORTH:
            col = pos[0] - 2
            row = pos[1] + 2
            if row > 19 and col < 0:
                fPath.append('sf')
            else:
                if 0 <= col <= 14:
                    if 0 <= row <= 19:
                        col = pos[0] - 2
                        row = pos[1] - 1
                        if obstacle_map[row][col] == 1 and obstacle_map[row + 1][col] == 1 and obstacle_map[row + 2][col] == 1:
                            fPath.append('s')

                        col = pos[0] - 1
                        row = pos[1] + 2
                        if obstacle_map[row][col] == 1 and obstacle_map[row][col + 1] == 1 and obstacle_map[row][col + 2] == 1:
                            fPath.append('f')
                    else:
                        fPath.append('f')
                else:
                    fPath.append('s')
        elif bearing == Bearing.SOUTH:
            col = pos[0] + 2
            row = pos[1] - 2
            if row < 0 and col > 14:
                fPath.append('sf')
            else:
                if 0 <= col <= 14:
                    if 0 <= row <= 19:
                        col = pos[0] + 2
                        row = pos[1] - 1
                        if obstacle_map[row][col] == 1 and obstacle_map[row + 1][col] == 1 and obstacle_map[row + 2][col] == 1:
                            fPath.append('s')

                        col = pos[0] - 1
                        row = pos[1] - 2
                        if obstacle_map[row][col] == 1 and obstacle_map[row][col + 1] == 1 and obstacle_map[row][col + 2] == 1:
                            fPath.append('f')
                    else:
                        fPath.append('f')
                else:
                    fPath.append('s')
        elif bearing == Bearing.EAST:
            col = pos[0] + 2
            row = pos[1] + 2
            if row > 19 and col > 14:
                fPath.append('sf')
            else:
                if 0 <= row <= 19:
                    if 0 <= col <= 14:
                        col = pos[0] - 1
                        row = pos[1] + 2
                        if obstacle_map[row][col] == 1 and obstacle_map[row][col + 1] == 1 and obstacle_map[row][col + 2] == 1:
                            fPath.append('s')

                        col = pos[0] + 2
                        row = pos[1] - 1
                        if obstacle_map[row][col] == 1 and obstacle_map[row + 1][col] == 1 and obstacle_map[row + 2][col] == 1:
                            fPath.append('f')
                    else:
                        fPath.append('f')
                else:
                    fPath.append('s')
        elif bearing == Bearing.WEST:
            col = pos[0] - 2
            row = pos[1] - 2
            if row < 0 and col < 0:
                fPath.append('sf')
            else:
                if 0 <= row <= 19:
                    if 0 <= col <= 14:
                        col = pos[0] - 1
                        row = pos[1] - 2
                        if obstacle_map[row][col] == 1 and obstacle_map[row][col + 1] == 1 and obstacle_map[row][col + 2] == 1:
                            fPath.append('s')

                        col = pos[0] - 2
                        row = pos[1] - 1
                        if obstacle_map[row][col] == 1 and obstacle_map[row + 1][col] == 1 and obstacle_map[row + 2][col] == 1:
                            fPath.append('f')
                    else:
                        fPath.append('f')
                else:
                    fPath.append('s')

    # combine 2 half paths generated from gen_half_path
    def gen_full_path(self, grid, waypoint):
        obstacle_map = []
        for row in grid:
            obstacle_map.append(row[:])
        ActlFastPathAlgo.preprocess(grid)
        waypoint = ActlFastPathAlgo.check_waypoint(grid, waypoint)
        # reverse coordinates to fit grid system used in fastest path algorithm
        cost = float('inf')
        for wp in waypoint:
            route, cost1 = self.gen_half_path(grid, (1, 1), wp)
            route2, cost2 = self.gen_half_path(grid, wp, (18, 13))
            for d in route2[1:]:
                route.append(d)
            # convert bearing to 2-4-6-8 system used in main program
            for i in range(len(route)):
                route[i] //= 45
            if cost1 + cost2 < cost:
                cost = cost1 + cost2
                self._fastestPath = route
        commands = ActlFastPathAlgo.convert_commands(route)
        # uncomment if they dont want calibration
        self._fastestPath = commands

        # if they want calibration
        # pos = [1, 1]
        # bearing = Bearing.NORTH
        # fPath = []
        # print(len(commands))
        # for i in range(len(commands) - 1):
        #     print(i)
        #     print(commands[i])
        #     print()
        #     if commands[i] == 'l':
        #         bearing = Bearing.rotateLeft(bearing)
        #         fPath.append(commands[i])
        #         self.add_calibration_2(obstacle_map, pos, bearing, fPath)
        #     elif commands[i] == 'r':
        #         bearing = Bearing.rotateRight(bearing)
        #         fPath.append(commands[i])
        #         self.add_calibration_2(obstacle_map, pos, bearing, fPath)
        #     else:
        #         n = int(commands[i])
        #         if bearing == Bearing.NORTH:
        #             pos[1] += n
        #         elif bearing == Bearing.SOUTH:
        #             pos[1] -= n
        #         elif bearing == Bearing.EAST:
        #             pos[0] += n
        #         else:
        #             pos[0] -= n
        #         fPath.append(commands[i])
        #         self.add_calibration_2(obstacle_map, pos, bearing, fPath)
        # fPath.append(commands[len(commands) - 1])
        # self._fastestPath = fPath

        # for i in range(1, len(commands)):
        #     print(bearing)
        #     if commands[i] == 'l':
        #         ActlFastPathAlgo.add_calibration(grid, pos, bearing, fPath)
        #         bearing = Bearing.rotateLeft(bearing)
        #     elif commands[i] == 'r':
        #         print('1')
        #         ActlFastPathAlgo.add_calibration(grid, pos, bearing, fPath)
        #         print('2')
        #         bearing = Bearing.rotateRight(bearing)
        #         print('3')
        #     elif commands[i] == '9':
        #         ActlFastPathAlgo.add_calibration(grid, pos, bearing, fPath)
        #         if bearing == Bearing.NORTH:
        #             pos[1] += int(commands[i])
        #         elif bearing == Bearing.SOUTH:
        #             pos[1] -= int(commands[i])
        #         elif bearing == Bearing.EAST:
        #             pos[0] += int(commands[i])
        #         else:
        #             pos[0] -= int(commands[i])
        #     else:
        #         if bearing == Bearing.NORTH:
        #             pos[1] += int(commands[i])
        #         elif bearing == Bearing.SOUTH:
        #             pos[1] -= int(commands[i])
        #         elif bearing == Bearing.EAST:
        #             pos[0] += int(commands[i])
        #         else:
        #             pos[0] -= int(commands[i])
        #     fPath.append(commands[i])
        # self._fastestPath = fPath
        return self._fastestPath

    def convert_commands(num_route):
        commands = []
        count = 0
        for i in range(1, len(num_route)):
            if num_route[i] == num_route[i - 1] - 2 or (num_route[i] == 6 and num_route[i - 1] == 0):
                if count > 0:
                    if count > 9:
                        commands.append(str(9))
                        commands.append(str(count - 9))
                    else:
                        commands.append(str(count))
                    count = 0
                commands.append("l")
            elif num_route[i] == num_route[i - 1] + 2 or (num_route[i] == 0 and num_route[i - 1] == 6):
                if count > 0:
                    if count > 9:
                        commands.append(str(9))
                        commands.append(str(count - 9))
                    else:
                        commands.append(str(count))
                    count = 0
                commands.append("r")
            elif abs(num_route[i] - num_route[i - 1]) == 4:
                if count > 0:
                    if count > 9:
                        commands.append(str(9))
                        commands.append(str(count - 9))
                    else:
                        commands.append(str(count))
                    count = 0
                commands.append("l")
                commands.append("l")
            count = count + 1
        if count > 0:
            if count > 9:
                commands.append(str(9))
                commands.append(str(count - 9))
            else:
                commands.append(str(count))
        return commands

    def run(self):
        print(len(self._fastestPath))
        cmd = "".join(self._fastestPath)
        msg = "FP|" + cmd
        self.signalSendCmd.emit(msg)
        # self.finished.emit()
