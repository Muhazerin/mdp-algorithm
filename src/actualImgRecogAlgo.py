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


# img was taken and result is positive
def get_obstacle_coordinate(robot_bearing, all_corners, result):
    if robot_bearing == Bearing.NORTH:
        coordinate = all_corners[2]
        if result[1] == -1:
            return coordinate
        elif result[1] == 0:
            return [coordinate[0], coordinate[1] + 1]
        else:
            return [coordinate[0], coordinate[1] + 2]
    elif robot_bearing == Bearing.EAST:
        coordinate = all_corners[0]
        if result[1] == -1:
            return coordinate
        elif result[1] == 0:
            return [coordinate[0] + 1, coordinate[1]]
        else:
            return [coordinate[0] + 2, coordinate[1]]
    elif robot_bearing == Bearing.SOUTH:
        coordinate = all_corners[1]
        if result[1] == -1:
            return coordinate
        elif result[1] == 0:
            return [coordinate[0], coordinate[1] - 1]
        else:
            return [coordinate[0], coordinate[1] - 1]
    else:
        coordinate = all_corners[3]
        if result[1] == -1:
            return coordinate
        elif result[1] == 0:
            return [coordinate[0] - 1, coordinate[1]]
        else:
            return [coordinate[0] - 2, coordinate[1]]


class ActlImgRecogAlgo(QObject):
    finished = pyqtSignal()
    signalSendMsg = pyqtSignal(str)

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

        self.__pic_taken_location = [
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

    @pyqtSlot()
    def timer_timeout(self):
        print('Actual Img Rec 5m40s passed')
        self.__timeout = True

    def send_msg(self, msg):
        cmd = "EC|" + msg
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

        self.__pic_taken_location = [
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

    def update_hug_left_obstacle(self, robot_bearing, all_corners, obstacle_map):
        if robot_bearing == Bearing.NORTH:
            coordinate = all_corners[2]
            if coordinate[0] >= 0:  # within arena
                for row in range(coordinate[1], coordinate[1] + 3):
                    if obstacle_map[row][coordinate[0]] == 1:
                        self.__obstacle_left_hug_map[row][coordinate[0] + 1] = 1
        elif robot_bearing == Bearing.EAST:
            coordinate = all_corners[0]
            if coordinate[1] <= 19:  # within arena
                for col in range(coordinate[0], coordinate[0] + 3):
                    if obstacle_map[coordinate[1]][col] == 1:
                        self.__obstacle_left_hug_map[coordinate[1] - 1][col] = 1
        elif robot_bearing == Bearing.SOUTH:
            coordinate = all_corners[1]
            if coordinate[0] <= 14:  # within arena
                for row in range(coordinate[1], coordinate[1] - 3, -1):
                    if obstacle_map[row][coordinate[0]] == 1:
                        self.__obstacle_left_hug_map[row][coordinate[0] - 1] = 1
        else:
            coordinate = all_corners[3]
            if coordinate[1] >= 0:  # within arena
                for col in range(coordinate[0], coordinate[0] - 3, -1):
                    if obstacle_map[coordinate[1]][col] == 1:
                        self.__obstacle_left_hug_map[coordinate[1] + 1][col] = 1

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

            cv2.imshow('all.jpg', all_img)
            # cv2.waitKey()
        else:
            print('No images found in disk')

        print('emitting finished')
        self.finished.emit()

    def pic_taken_on_top_left(self, robot_bearing, all_corners):
        if robot_bearing == Bearing.NORTH:
            coordinate = all_corners[2]
            if coordinate[0] >= 0:  # within arena
                if self.__pic_taken_location[coordinate[1] + 2][coordinate[0]] == 1:
                    return True
                else:
                    return False
            else:
                return True
        elif robot_bearing == Bearing.EAST:
            coordinate = all_corners[0]
            if coordinate[1] <= 19:  # within arena
                if self.__pic_taken_location[coordinate[1]][coordinate[0] + 2] == 1:
                    return True
                else:
                    return False
            else:
                return True
        elif robot_bearing == Bearing.SOUTH:
            coordinate = all_corners[1]
            if coordinate[0] <= 14:  # within arena
                if self.__pic_taken_location[coordinate[1] - 2][coordinate[0]] == 1:
                    return True
                else:
                    return False
            else:
                return True
        else:
            coordinate = all_corners[3]
            if coordinate[1] >= 0:  # within arena
                if self.__pic_taken_location[coordinate[1]][coordinate[0] - 2] == 1:
                    return True
                else:
                    return False
            else:
                return True

    def pic_taken_on_bottom_left(self, robot_bearing, all_corners):
        if robot_bearing == Bearing.NORTH:
            coordinate = all_corners[2]
            if coordinate[0] >= 0:  # within arena
                if self.__pic_taken_location[coordinate[1]][coordinate[0]] == 1:
                    return True
                else:
                    return False
            else:
                return True
        elif robot_bearing == Bearing.EAST:
            coordinate = all_corners[0]
            if coordinate[1] <= 19:  # within arena
                if self.__pic_taken_location[coordinate[1]][coordinate[0]] == 1:
                    return True
                else:
                    return False
            else:
                return True
        elif robot_bearing == Bearing.SOUTH:
            coordinate = all_corners[1]
            if coordinate[0] <= 14:  # within arena
                if self.__pic_taken_location[coordinate[1]][coordinate[0]] == 1:
                    return True
                else:
                    return False
            else:
                return True
        else:
            coordinate = all_corners[3]
            if coordinate[1] >= 0:  # within arena
                if self.__pic_taken_location[coordinate[1]][coordinate[0]] == 1:
                    return True
                else:
                    return False
            else:
                return True

    def update_pic_location(self, robot_bearing, all_corners, obstacle_map):
        if robot_bearing == Bearing.NORTH:
            coordinate = all_corners[2]
            if coordinate[0] >= 0:  # within arena
                for row in range(coordinate[1], coordinate[1] + 3):
                    if obstacle_map[row][coordinate[0]] == 1:
                        self.__pic_taken_location[row][coordinate[0]] = 1
        elif robot_bearing == Bearing.EAST:
            coordinate = all_corners[0]
            if coordinate[1] <= 19:  # within arena
                for col in range(coordinate[0], coordinate[0] + 3):
                    if obstacle_map[coordinate[1]][col] == 1:
                        self.__pic_taken_location[coordinate[1]][col] = 1
        elif robot_bearing == Bearing.SOUTH:
            coordinate = all_corners[1]
            if coordinate[0] <= 14:  # within arena
                for row in range(coordinate[1], coordinate[1] - 3, -1):
                    if obstacle_map[row][coordinate[0]] == 1:
                        self.__pic_taken_location[row][coordinate[0]] = 1
        else:
            coordinate = all_corners[3]
            if coordinate[1] >= 0:  # within arena
                for col in range(coordinate[0], coordinate[0] - 3, -1):
                    if obstacle_map[coordinate[1]][col] == 1:
                        self.__pic_taken_location[coordinate[1]][col] = 1

    def normal_robot_movement(self, robot_bearing, all_corners, obstacle_map):
        # normal robot movement algorithm
        if not self.__phantom_block_loop:
            if self.__front_left_dict['L'] == 1:    # left is not free
                self.update_hug_left_obstacle(robot_bearing, all_corners, obstacle_map)
                if self.__robot_just_turned_left_and_move_forward:
                    self.__robot_just_turned_left_and_move_forward = False
                if self.__front_left_dict['F'] == 1:    # front is not free
                    if not self.pic_taken_on_top_left(robot_bearing, all_corners):
                        self.update_pic_location(robot_bearing, all_corners, obstacle_map)
                        self.send_msg('TP|')
                    else:
                        self.__no_of_left_rotation -= 1
                        self.send_msg('r')
                else:   # front is free
                    if not self.pic_taken_on_bottom_left(robot_bearing, all_corners):
                        self.update_pic_location(robot_bearing, all_corners, obstacle_map)
                        self.send_msg('TP|')
                    else:
                        self.send_msg('1')
            else:  # left is free
                if self.__robot_just_turned_left_and_move_forward:
                    self.__robot_just_turned_left_and_move_forward = False
                    self.__phantom_block_loop = True
                    self.__no_of_left_rotation += 1
                    self.send_msg('l')
                elif self.__robot_just_turned_left:
                    self.__robot_just_turned_left = False
                    self.__robot_just_turned_left_and_move_forward = True
                    self.send_msg('1')
                else:
                    self.__robot_just_turned_left = True
                    self.__no_of_left_rotation += 1
                    self.send_msg('l')
        else:
            if self.__front_left_dict['F'] == 0:
                self.send_msg('1')
            else:
                self.__phantom_block_loop = False
                self.__no_of_left_rotation -= 1
                self.send_msg('r')

    @pyqtSlot(dict, list, list, list, int)
    def determine_move(self, front_left_dict, all_corners, explored_map, obstacle_map, robot_bearing):
        print()
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

            if not self.__stop:
                if self.__algo_status == ImgRecogAlgoStatus.SEARCH_OBSTACLE:
                    list_of_coordinates = self.obstacle_without_left_hug(explored_map, obstacle_map)
                    # if there is obstacle without left hug, fp to nearest obstacle
                    if list_of_coordinates:
                        # robot_pos in obstacle_coordinate is in x,y coordinate
                        obstacle_coordinate = find_nearest_obstacle(list_of_coordinates, robot_center)
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
                    self.normal_robot_movement(robot_bearing, all_corners, obstacle_map)
                elif self.__algo_status == ImgRecogAlgoStatus.FP_TO_OBSTACLE:
                    self.send_a_star_move_cmd()
                else:
                    # algo_status = seek_home or seek_goal
                    self.normal_robot_movement(robot_bearing, all_corners, obstacle_map)
            else:
                self.process_image()
        else:
            # Timer time out
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
                self.__move_cmd_index = -1
                self.__move_cmd = None
                self.__algo_status = ImgRecogAlgoStatus.LEFT_WALL_HUG
        else:
            self.__move_cmd_index = -1

    @pyqtSlot(list, int, list, list)
    def save_prediction(self, result, bearing, all_corners, obstacle_map):
        if result:
            if result[0] not in self.__img_id:
                print("[Algo] Saving Prediction...")
                self.__img_id.add(result[0])
                marked_coordinate = get_obstacle_coordinate(bearing, all_corners, obstacle_map, result)
                self.signalSendMsg.emit(f'IP|{result[0]},{marked_coordinate[0]},{marked_coordinate[1]}')
                time.sleep(0.5)
        self.signalSendMsg.emit('EC|s')

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
        self.reset_map()
        self.signalSendMsg.emit('EC|s')
