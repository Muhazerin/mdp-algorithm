from constants import SensorLocation, Bearing
# TODO: Add sensor boundary like cannot sense beyond the obstacle

class RobotSensorConfiguration:
    def __init__(self, map):
        self.__sensorList = [
            SimIrSensor(SensorLocation.FRONT, map)
            # SimIrSensor(SensorLocation.LEFT, map)
        ]

    def senseAll(self, robotBearing, robotX, robotY):
        for sensor in self.__sensorList:
            sensor.sense(robotBearing, robotX, robotY)


class SimIrSensor:
    def __init__(self, sensorLocation, map):
        self.__sensorLocation = sensorLocation
        self.__map = map

    @property
    def sensorLocation(self):
        return self.__sensorLocation

    def senseFront(self, robotX, robotY):
        x = int(robotX / 40)
        y = int(abs(robotY) / 40)
        obstacle = False

        for j in range(y, y + 2):
            for i in range(x, x + 3):
                self.__map.updateExploredMap(j, i, 1)
                if self.__map.simExplObstacleMap[j][i] == 0:
                    self.__map.updateObstacleMap(j, i, 0)
                else:
                    obstacle = True
                    self.__map.updateObstacleMap(j, i, 1)
            if obstacle:
                break

    def sense(self, robotBearing, robotX, robotY):
        if robotBearing == Bearing.NORTH:
            if self.__sensorLocation == SensorLocation.FRONT:
                self.senseFront(robotX, robotY)
