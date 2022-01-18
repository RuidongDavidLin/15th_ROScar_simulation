import map
import math
#import matplotlib.pyplot as plt

class pathPlanning:
    def __init__(self, distanceK, angleK, angleGate, angleGateK):
        self.__mapCreater(distanceK, angleK, angleGate, angleGateK)
        self.__target = [[], [1, 2], [3, 4, 5], [32, 31], [9, 10], [11, 12, 13], [17, 18], [19, 20, 21],
                         [22, 23, 24], [25, 26],
                         [27, 28, 29, 30], [14, 15, 16], [6, 7, 8]]
        self.target=self.__target
        self.__pointXY = [[0,0],[-0.201,0.0284],[4.93,-0.127],[9.35,-1.49],[15,-0.474],[14.7,-5.68],[14.5,-9.5],[7.51,-9.26],[2.94,-9.01],[-0.528,-9],[6.34,-4.75],[8.46,-2.6],[10,-0.249]]
        #self.__pointXY=[[0,0],[-12.8,-1.95],[-7.81,-1.3],[-3.27,-1.82],[-2.42,-0.287],[2.75,-0.287],[2.75,-5.09],[2.53,-8.53],[-8.13,-9],[-11.7,-9.41],[-5.65,-5.34],[-4.04,-3.1],[-3.02,-0.545]]
        self.__pointKey = [[0, 0], [1, 2], [1, 9], [2,12], [2, 10], [2, 1], [12, 4], [12, 3], [12, 2], [4, 12], [4, 5],
                           [5, 4], [5, 6], [5, 11], [11, 3], [11, 5], [11, 10], [6, 5], [6, 7], [7, 6], [7, 10],
                           [7, 8], [8, 7], [8, 10], [8, 9], [9, 8], [9, 1], [10, 2], [10, 11], [10, 7], [10, 8],
                           [3, 12], [3, 11]]

    def findBestPathToCoordinateOnlyTarget(self, startDirection, mastPass, end):
        path = self.findBestPathToDirection(startDirection, mastPass, end)
        print(path)
        pathEnd = path[-1]
        path = path[:-1]
        pathCoordinate = []
        for i in path:
            point = self.__pointXY[self.__pointKey[i][1]]
            pathCoordinate.append(point)
        endNextCoordinate = self.__pointXY[self.__pointKey[pathEnd][1]]
        endEndPoint = self.__pathInterpolationlittle(pathCoordinate[-1], endNextCoordinate, 0.7)
        pathCoordinate.append(endEndPoint[0])


        return pathCoordinate

    def findBestPathToDirection(self, startDirection, mastPass, end):
        self.__bestPath = []
        self.__bestPathCost = float('inf')
        endDirction = self.__target[end]
        mastPassDirection = []
        for i in mastPass:
            mastPassDirection.append(self.__target[i])
        lastDirectionList = [0] * 100
        lastDirectionList[0] = startDirection
        self.__findBestPathFunc(lastDirectionList, 0, 0, mastPassDirection, endDirction)
        afBestPath = [self.__bestPath[0]]
        for i in range(len(self.__bestPath)-1):
            if self.__bestPath[i]!=self.__bestPath[i+1]:
                afBestPath.append(self.__bestPath[i+1])
        print(self.__bestPath)
        return self.__pathTranslater(afBestPath)

    def __pathInterpolationlittle(self, start, end, len):
        if start[0] == end[0]:
            if end[1] > start[1]:
                afPath = []
                y = start[1]
                y = y + len
                x = (y - start[1]) / (end[1] - start[1]) * (end[0] - start[0]) + start[0]
                afPath.append([x, y])
                return afPath
            else:
                afPath = []
                y = start[1]
                y = y - len
                x = (y - start[1]) / (end[1] - start[1]) * (end[0] - start[0]) + start[0]
                afPath.append([x, y])

                return afPath
        elif end[0] > start[0]:
            len = len * math.cos(math.atan((start[1] - end[1]) / (start[0] - end[0])))
            afPath = []
            x = start[0]
            x = x + len
            y = (x - start[0]) / (end[0] - start[0]) * (end[1] - start[1]) + start[1]
            afPath.append([x, y])

            return afPath
        else:
            len = len * math.cos(math.atan((start[1] - end[1]) / (start[0] - end[0])))
            afPath = []
            x = start[0]
            x = x - len
            y = (x - start[0]) / (end[0] - start[0]) * (end[1] - start[1]) + start[1]
            afPath.append([x, y])
            return afPath

    def __pathTranslater(self, path):
        afTransPath = []
        for i in range(len(path) - 1):
            newPath = [path[i + 1]]
            while self.__path[path[i]][newPath[-1]] != path[i]:
                newPath.append(self.__path[path[i]][newPath[-1]])
            newPath.append(path[i])
            newPath.reverse()
            afTransPath = afTransPath + newPath[:-1]
        afTransPath = afTransPath + path[-1:]
        return afTransPath

    def __findBestPathFunc(self, lastDirection, lastDirectionLen, lastDirectionCost, mastPass, end):
        if len(mastPass) == 0:
            for i in end:
                lastDirectionCostEnd = lastDirectionCost + self.__pathK[lastDirection[lastDirectionLen]][i]
                if lastDirectionCostEnd < self.__bestPathCost:
                    lastDirection[lastDirectionLen + 1] = i
                    self.__bestPath = lastDirection[0:lastDirectionLen + 2]
                    self.__bestPathCost = lastDirectionCostEnd
        else:
            for i in range(len(mastPass)):
                for j in mastPass[i]:
                    if lastDirectionCost + self.__pathK[lastDirection[lastDirectionLen]][j] > self.__bestPathCost:
                        continue
                    lastDirection[lastDirectionLen + 1] = j
                    self.__findBestPathFunc(lastDirection, lastDirectionLen + 1,
                                            lastDirectionCost + self.__pathK[lastDirection[lastDirectionLen]][j],
                                            mastPass[0:i] + mastPass[i + 1:], end)

 
    def __mapCreater(self, distanceK, angleK, angleGate, angleGateK):
        dMap = map.dMap
        aMap = map.aMap

        hybridMap = [[0] * 33 for _ in range(33)]
        for i in range(33):
            for j in range(33):
                if aMap[i][j] > angleGate:
                    hybridMap[i][j] = dMap[i][j] * distanceK + aMap[i][j] * angleK * angleGateK
                else:
                    hybridMap[i][j] = dMap[i][j] * distanceK + aMap[i][j] * angleK

        self.__path = [[0] * 33 for _ in range(33)]
        self.__pathK = [[0] * 33 for _ in range(33)]
        for i in range(32):
            self.__pathK[i + 1], self.__path[i + 1] = self.__dijkstra(hybridMap, i + 1)

    def __dijkstra(self, map, start):
        T = [0] * 33
        dis = [float("inf")] * 33
        disDel = [1] * 33
        dis[start] = 0
        path = [-1] * 33
        key = 32
        while key != 0:
            num = dis.index(min(dis))
            T[num] = dis[num]
            for i in range(33):
                if map[num][i] != 0:
                    if dis[i] > map[num][i] + dis[num] and disDel[i]:
                        dis[i] = map[num][i] + dis[num]
                        path[i] = num
            dis[num] = float("inf")
            disDel[num] = 0
            key = key - 1
        return T, path

