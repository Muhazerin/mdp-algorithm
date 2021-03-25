import time

from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
from constants import ImgRecogAlgoStatus, Bearing
from simExplFastPath import a_star_search, gen_move_cmd
import cv2
import os
import show_image


def find_nearest_obstacle(list_of_coordinates, current_robot_center):
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


def can_calibrate(robot_bearing, obstacle_map, all_corners):
    if robot_bearing == Bearing.NORTH:
        coordinate = all_corners[2]
        if coordinate[0] < 0:  # if the col is out of bound
            return True
        elif obstacle_map[coordinate[1]][coordinate[0]] == 1 and \
                obstacle_map[coordinate[1] + 2][coordinate[0]] == 1:
            # if there are obstacle on top and bottom left
            return True
        else:
            return False
    elif robot_bearing == Bearing.EAST:
        coordinate = all_corners[0]
        if coordinate[1] > 19:  # if the row is out of bound
            return True
        elif obstacle_map[coordinate[1]][coordinate[0]] == 1 and \
                obstacle_map[coordinate[1]][coordinate[0] + 2] == 1:
            # if there are obstacle on top and bottom left
            return True
        else:
            return False
    elif robot_bearing == Bearing.SOUTH:
        coordinate = all_corners[1]
        if coordinate[0] > 14:  # if the col is out of bound
            return True
        elif obstacle_map[coordinate[1]][coordinate[0]] == 1 and \
                obstacle_map[coordinate[1] - 2][coordinate[0]] == 1:
            # if there are obstacle on top and bottom left
            return True
        else:
            return False
    else:
        coordinate = all_corners[3]
        if coordinate[1] < 0:  # if the row is out of bound
            return True
        elif obstacle_map[coordinate[1]][coordinate[0]] == 1 and \
                obstacle_map[coordinate[1]][coordinate[0] - 2] == 1:
            # if there are obstacle on top and bottom left
            return True
        else:
            return False


class ActlImgRecogAlgo(QObject):
    finished = pyqtSignal()
    signalSendMsg = pyqtSignal(str)
    signalAfterPhoto = pyqtSignal()

    def __init__(self):
        super(ActlImgRecogAlgo, self).__init__()
        self.__stop = False
        self.__algo_status = ImgRecogAlgoStatus.SEEK_GOAL
        self.__robot_just_turned_left = False
        self.__robot_just_turned_left_and_move_forward = False
        self.__phantom_block_loop = False
        self.__front_left_dict = dict()
        self.__move_cmd = None
        self.__move_cmd_index = -1
        self.__initial_pos = None
        self.__no_of_left_rotation = 0
        self.__img_id = set()
        self.__pic_taken = False
        self.__complete_ir = False
        self.__timeout = False
        self.__obstacle_left_hug_map = [
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        ]

        # [imgId]
        self.__imgRecMap = [
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
        ]

    @pyqtSlot()
    def timer_timeout(self):
        print('Actual Img Rec 5m40s passed')
        self.__timeout = True
        self.process_image()

    def send_msg(self, msg):
        cmd = "EC|" + msg
        print(f'send msg: {cmd}')
        self.signalSendMsg.emit(cmd)

    def reset_map(self):
        self.__obstacle_left_hug_map = [
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        ]

        # Testing against arena 4
        self.__imgRecMap = [
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
        ]

    # check the top, bottom, left, right of the r,c if the robot has left hug the obstacle
    # robot pos is in x,y coordinate
    def check_surrounding_obstacle(self, r, c, explored_map, obstacle_map):
        list_of_coordinates = list()
        coordinates = [[r + 1, c],
                       [r - 1, c],
                       [r, c - 1],
                       [r, c + 1]]
        for coordinate in coordinates:
            row = coordinate[0]
            col = coordinate[1]
            if 0 <= row <= 19 and 0 <= col <= 14:
                if explored_map[row][col] == 1 and obstacle_map[row][col] == 0 and \
                        self.__obstacle_left_hug_map[row][col] == 0:
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

    def obstacle_without_left_hug(self, explored_map, obstacle_map):
        list_of_coordinates = list()
        for row in range(0, len(explored_map)):
            for col in range(0, len(explored_map[row])):
                if explored_map[row][col] == 1 and obstacle_map[row][col] == 1:
                    temp = self.check_surrounding_obstacle(row, col, explored_map, obstacle_map)
                    if temp:
                        list_of_coordinates = list_of_coordinates + temp
        return list_of_coordinates

    # Check if there is obstacle on left of robot
    def is_left_obstacle(self, robot_bearing, all_corners, obstacle_map):
        left_obstacle = False
        if robot_bearing == Bearing.NORTH:
            coordinate = all_corners[2]
            if coordinate[0] >= 0:
                for row in range(coordinate[1], coordinate[1] + 3):
                    if obstacle_map[row][coordinate[0]] == 1:
                        left_obstacle = True
                        self.__obstacle_left_hug_map[row][coordinate[0] + 1] = 1
        elif robot_bearing == Bearing.EAST:
            coordinate = all_corners[0]
            if coordinate[1] <= 19:
                for col in range(coordinate[0], coordinate[0] + 3):
                    if obstacle_map[coordinate[1]][col] == 1:
                        left_obstacle = True
                        self.__obstacle_left_hug_map[coordinate[1] - 1][col] = 1
        elif robot_bearing == Bearing.SOUTH:
            coordinate = all_corners[1]
            if coordinate[0] <= 14:
                for row in range(coordinate[1], coordinate[1] - 3, -1):
                    if obstacle_map[row][coordinate[0]] == 1:
                        left_obstacle = True
                        self.__obstacle_left_hug_map[row][coordinate[0] - 1] = 1
        else:
            coordinate = all_corners[3]
            if coordinate[1] >= 0:
                for col in range(coordinate[0], coordinate[0] - 3, -1):
                    if obstacle_map[coordinate[1]][col] == 1:
                        left_obstacle = True
                        self.__obstacle_left_hug_map[coordinate[1] + 1][col] = 1
        return left_obstacle

    def mark_img(self, robot_bearing, coordinate, result):
        try:
            if robot_bearing == Bearing.NORTH:
                if result[1] == -1:
                    self.__imgRecMap[coordinate[1]][coordinate[0]] = result[0]
                    return [coordinate[1], coordinate[0]]
                elif result[1] == 0:
                    self.__imgRecMap[coordinate[1] + 1][coordinate[0]] = result[0]
                    return [coordinate[1] + 1, coordinate[0]]
                else:
                    self.__imgRecMap[coordinate[1] + 2][coordinate[0]] = result[0]
                    return [coordinate[1] + 2, coordinate[0]]
            elif robot_bearing == Bearing.EAST:
                if result[1] == -1:
                    self.__imgRecMap[coordinate[1]][coordinate[0]] = result[0]
                    return [coordinate[1], coordinate[0]]
                elif result[1] == 0:
                    self.__imgRecMap[coordinate[1]][coordinate[0] + 1] = result[0]
                    return [coordinate[1], coordinate[0] + 1]
                else:
                    self.__imgRecMap[coordinate[1]][coordinate[0] + 1] = result[0]
                    return [coordinate[1], coordinate[0] + 2]
            elif robot_bearing == Bearing.SOUTH:
                if result[1] == -1:
                    self.__imgRecMap[coordinate[1]][coordinate[0]] = result[0]
                    return [coordinate[1], coordinate[0]]
                elif result[1] == 0:
                    self.__imgRecMap[coordinate[1] - 1][coordinate[0]] = result[0]
                    return [coordinate[1] - 1, coordinate[0]]
                else:
                    self.__imgRecMap[coordinate[1] - 2][coordinate[0]] = result[0]
                    return [coordinate[1] - 2, coordinate[0]]
            else:
                if result[1] == -1:
                    self.__imgRecMap[coordinate[1]][coordinate[0]] = result[0]
                    return [coordinate[1], coordinate[0]]
                elif result[1] == 0:
                    self.__imgRecMap[coordinate[1]][coordinate[0] - 1] = result[0]
                    return [coordinate[1], coordinate[0] - 1]
                else:
                    self.__imgRecMap[coordinate[1]][coordinate[0] - 2] = result[0]
                    return [coordinate[1], coordinate[0] - 2]
        except Exception as err:
            print(f'actlImgRecogAlgo::mark_img() Error msg: {err}')
            self.finished.emit()

    def process_image(self):
        folders = os.listdir('images')
        img_folders = []
        for folder in folders:
            img_folders.append(cv2.imread(f'images/{folder}/image0.jpg'))

        if img_folders:
            w_min = min(img.shape[1] for img in img_folders)
            w_min /= 2
            im_list_resize = [cv2.resize(img,
                                         (int(w_min), int(img.shape[0] * w_min / img.shape[1])),
                                         interpolation=cv2.INTER_CUBIC)
                              for img in img_folders]
            if len(im_list_resize) == 5:
                all_img = show_image.concat_tile_resize([[im_list_resize[0], im_list_resize[1], im_list_resize[2]],
                                              [im_list_resize[3], im_list_resize[4]]
                                              ])
            elif len(im_list_resize) == 4:
                all_img = show_image.concat_tile_resize([[im_list_resize[0], im_list_resize[1]],
                                              [im_list_resize[2], im_list_resize[3]]
                                              ])
            elif len(im_list_resize) == 3:
                all_img = show_image.concat_tile_resize([[im_list_resize[0], im_list_resize[1], im_list_resize[2]],
                                              ])
            elif len(im_list_resize) == 2:
                all_img = show_image.concat_tile_resize([[im_list_resize[0], im_list_resize[1]],
                                              ])
            elif len(im_list_resize) == 1:
                all_img = im_list_resize[0]

            # cv2.imshow('all_img.jpg', all_img)
            cv2.imwrite('all_img.jpg', all_img)
            # cv2.waitKey()
            self.send_ip_again()
        else:
            print('No images found in disk')

        print('emitting finished')
        self.finished.emit()

    def send_ip_again(self):
        try:
            for row in range(len(self.__imgRecMap)):
                for col in range(len(self.__imgRecMap[row])):
                    if self.__imgRecMap[row][col] != -1:
                        self.signalSendMsg.emit(f'IP|{self.__imgRecMap[row][col]},{col},{row}')
        except Exception as err:
            print('Unable to send ip again')
            print(f'send_ip_again() Error: {err}')

    def normal_robot_movement(self):
        # normal robot movement algorithm
        print(self.__front_left_dict)
        if self.__robot_just_turned_left_and_move_forward:
            print('turn left, move forward')
            self.__robot_just_turned_left_and_move_forward = False
            if self.__front_left_dict['L'] == 0:  # Phantom Block Loop Detected
                self.__phantom_block_loop = True
                self.__no_of_left_rotation = self.__no_of_left_rotation + 1
                self.send_msg('l')
            else:
                if self.__front_left_dict['F'] == 1:
                    self.__no_of_left_rotation = self.__no_of_left_rotation - 1
                    self.send_msg('r')
                else:
                    self.send_msg('1')
        elif self.__phantom_block_loop:
            print('phantom loop')
            if self.__front_left_dict['F'] == 0:  # if front is free in phantom_block_loop
                self.__no_of_left_rotation = self.__no_of_left_rotation - 1
                self.send_msg('1')
            else:  # if front is not empty
                self.send_msg('r')
                self.__phantom_block_loop = False
        elif not self.__robot_just_turned_left:     # normal movement
            print('normal movement')
            if self.__front_left_dict['L'] == 1 and \
                    self.__front_left_dict['F'] == 0:  # if left is not free and front is free
                self.send_msg('1')
            elif self.__front_left_dict['L'] == 1 and \
                    self.__front_left_dict['F'] == 1:  # if left and front is not free
                self.__no_of_left_rotation = self.__no_of_left_rotation - 1
                self.send_msg('r')
            elif self.__front_left_dict['L'] == 0:  # if left is free, turn left to hug the left wall
                self.__robot_just_turned_left = True
                self.__no_of_left_rotation = self.__no_of_left_rotation + 1
                self.send_msg('l')
        else:   # just turn left
            print('just turn left')
            self.__robot_just_turned_left = False
            self.__robot_just_turned_left_and_move_forward = True
            self.send_msg('1')

    @pyqtSlot(dict, list, list, list, int)
    def determine_move(self, front_left_dict, all_corners, explored_map, obstacle_map, robot_bearing):
        print('in determine_move')
        self.__front_left_dict = front_left_dict
        robot_center = all_corners[0][:]
        robot_center[0] = robot_center[0] + 1
        robot_center[1] = robot_center[1] - 2
        # if there are 5 images, stop the algorithm
        if not self.__timeout:
            # initialisation
            if len(self.__img_id) == 5:
                self.__stop = True
            elif self.__algo_status == ImgRecogAlgoStatus.SEEK_GOAL and [15, 19] in all_corners:
                # if algo is seeking goal and robot is at goal, change algo status to seek home
                self.__algo_status = ImgRecogAlgoStatus.SEEK_HOME
            elif self.__algo_status == ImgRecogAlgoStatus.SEEK_HOME and [-1, 0] in all_corners:
                # if algo is seeking home and robot is at home, search for obstacle
                self.__algo_status = ImgRecogAlgoStatus.SEARCH_OBSTACLE
            elif self.__algo_status == ImgRecogAlgoStatus.LEFT_WALL_HUG:
                # if algo is at obstacle, it is perfoming left_wall_hug
                # check if initial_pos is set and if robot_center == initial pos and no_of_left_rotation == 4
                if self.__initial_pos is None:
                    # Just arrive at obstacle
                    self.__initial_pos = robot_center
                    self.__no_of_left_rotation = 0
                elif self.__initial_pos == robot_center and self.__no_of_left_rotation == 4:
                    # robot has done 1 loop
                    self.__initial_pos = None
                    self.__no_of_left_rotation = 0
                    list_of_coordinates = self.obstacle_without_left_hug(explored_map, obstacle_map)
                    # if there are more obstacles to hug, hug
                    # if not, stop
                    if list_of_coordinates:
                        self.__algo_status = ImgRecogAlgoStatus.SEARCH_OBSTACLE
                    else:
                        self.__stop = True
            print(f'stop: {self.__stop}')
            if not self.__stop:
                if self.__algo_status == ImgRecogAlgoStatus.SEARCH_OBSTACLE:
                    list_of_coordinates = self.obstacle_without_left_hug(explored_map, obstacle_map)
                    # if there is obstacle without left hug, fp to nearest obstacle
                    if list_of_coordinates:
                        # robot_pos in obstacle_coordinate is in x,y coordinate
                        obstacle_coordinate = find_nearest_obstacle(list_of_coordinates, robot_center)
                        robot_center[0] += 1
                        robot_center[1] += 1
                        dest_node = a_star_search(robot_center, obstacle_coordinate['robot_pos'],
                                                  obstacle_coordinate['bearing'], robot_bearing, explored_map, obstacle_map)
                        print(f'\n{obstacle_coordinate}\n')
                        if dest_node is None:
                            print("A STAR SEARCH is unable to search the fastest path")
                            self.__stop = True
                        else:
                            self.__algo_status = ImgRecogAlgoStatus.FP_TO_OBSTACLE
                            self.__move_cmd = gen_move_cmd(dest_node)
                            self.__move_cmd_index = -1
                            self.send_a_star_move_cmd()
                    else:
                        self.__stop = True
                elif self.__algo_status == ImgRecogAlgoStatus.LEFT_WALL_HUG:
                    # Left_wall_hug obstacle after 1 whole round
                    left_obstacle = self.is_left_obstacle(robot_bearing, all_corners, obstacle_map)
                    # if got obstacle on left and havent take picture
                    if left_obstacle and not self.__pic_taken:
                        self.signalSendMsg.emit('TP|')
                    else:
                        self.__pic_taken = False
                        self.normal_robot_movement()
                elif self.__algo_status == ImgRecogAlgoStatus.FP_TO_OBSTACLE:
                    self.send_a_star_move_cmd()
                else:
                    # algo_status = seek_home or seek_goal
                    # at every move, check if there is an obstacle on the left.
                    # if yes, take pic
                    print('determine_move::normal algo_status')
                    left_obstacle = self.is_left_obstacle(robot_bearing, all_corners, obstacle_map)
                    if left_obstacle and not self.__pic_taken:
                        self.signalSendMsg.emit('TP|')
                    else:
                        print('in determine_move::doing normal_robot_movement')
                        self.__pic_taken = False
                        self.normal_robot_movement()
            else:
                self.process_image()

    def send_a_star_move_cmd(self):
        if self.__move_cmd is not None:
            self.__move_cmd_index = self.__move_cmd_index + 1
            if self.__move_cmd_index < len(self.__move_cmd):
                if self.__move_cmd[self.__move_cmd_index] == 'RR':
                    self.send_msg('r')
                elif self.__move_cmd[self.__move_cmd_index] == 'RL':
                    self.send_msg('l')
                else:
                    self.send_msg('1')
            else:
                print('FP to obstacle has ended')
                self.__move_cmd_index = -1
                self.__move_cmd = None
                self.__algo_status = ImgRecogAlgoStatus.LEFT_WALL_HUG
                self.signalAfterPhoto.emit()    # reusing code. need to go back to determine move
        else:
            self.__move_cmd_index = -1

    @pyqtSlot(list, int, list)
    def save_prediction(self, result, bearing, all_corners):
        try:
            self.__pic_taken = True
            if bearing == Bearing.NORTH:
                coordinate = all_corners[2]
            elif bearing == Bearing.EAST:
                coordinate = all_corners[0]
            elif bearing == Bearing.SOUTH:
                coordinate = all_corners[1]
            else:
                coordinate = all_corners[3]
            if result:
                if result[0] not in self.__img_id:
                    print("[Algo] Saving Prediction...")
                    self.__img_id.add(result[0])
                    marked_coordinate = self.mark_img(bearing, coordinate, result)
                    self.signalSendMsg.emit(f'IP|{result[0]},{marked_coordinate[1]},{marked_coordinate[0]}')
                    time.sleep(0.5)
            self.signalAfterPhoto.emit()
            # self.signalSendMsg.emit('EC|s')
        except Exception as err:
            print(f'save_prediction:: Error: {err}')

    @pyqtSlot()
    def run(self):
        print('Actual Image Recognition Started')
        self.__stop = False
        self.__algo_status = ImgRecogAlgoStatus.SEEK_GOAL
        self.__robot_just_turned_left = False
        self.__robot_just_turned_left_and_move_forward = False
        self.__phantom_block_loop = False
        self.__move_cmd = None
        self.__move_cmd_index = -1
        self.__initial_pos = None
        self.__no_of_left_rotation = 0
        self.__img_id = set()
        self.__pic_taken = False
        self.__complete_ir = False
        self.reset_map()
        self.signalSendMsg.emit('EC|s')
