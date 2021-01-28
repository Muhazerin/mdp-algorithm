from constants import AlgoStatus


# Define the constant (x,y) of the START_POS and GOAL_POS
START_POS = [0, -120]
GOAL_POS = [480, -800]

class SimExplorationAlgo:
    def __init__(self, robot):
        self.__robot = robot
        self.__status = AlgoStatus.IDLE

    def start(self):
        pass