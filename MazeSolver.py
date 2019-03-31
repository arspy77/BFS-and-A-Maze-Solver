import matplotlib.pyplot as plt
import numpy as np
import queue
import os

class Labyrinth:
    # Attribute

    # Method
    # Initializer with filename as argument
    def __init__(self, filename):
        try:
            f = open(filename, "r")
            lines = f.readlines()
            self.map = np.array([[int(num) for num in line.strip('\n')] for line in lines])
        except FileNotFoundError:
            print("Error, File Not Found")
    
    def setStartAndGoal(self, _start, _goal):
        self.start = _start
        self.goal = _goal

    def validateNode(self, node):
        try:
            return self.map[node[0]][node[1]] == 0
        except IndexError:
            return False

    def AStar(self):
        q = queue.PriorityQueue()
        costSoFar = 0
        distanceToGoal = abs(self.goal[0] - self.start[0]) + abs(self.goal[1] - self.start[1])
        q.put((costSoFar + distanceToGoal, [self.start]))
        goalReached = False
        while not goalReached:
            path = q.get()[1]
            node = path[-1]

            if node == self.goal:
                goalReached = True
                self.path = path
            else:
                costSoFar = abs(node[0] - self.start[0]) + abs(node[1] - self.start[1])
                if (node[0] > 0):
                    if (self.map[node[0]-1][node[1]] == 0) and not [node[0]-1, node[1]] in path:
                        optionalNode = [node[0]-1,node[1]]
                        newPath = list(path)
                        newPath.append(optionalNode)
                        distanceToGoal = abs(self.goal[0] - optionalNode[0]) + abs(self.goal[1] - optionalNode[1])
                        q.put((costSoFar + distanceToGoal, newPath))

                if (node[0] < len(self.map) - 1):
                    if (self.map[node[0]+1][node[1]] == 0) and not [node[0]+1, node[1]] in path:
                        optionalNode = [node[0]+1, node[1]]
                        newPath = list(path)
                        newPath.append(optionalNode)
                        distanceToGoal = abs(self.goal[0] - optionalNode[0]) + abs(self.goal[1] - optionalNode[1])
                        q.put((costSoFar + distanceToGoal, newPath))
                
                if (node[1] > 0):
                    if (self.map[node[0]][node[1]-1] == 0) and not [node[0], node[1]-1] in path:
                        optionalNode = [node[0], node[1] - 1]
                        newPath = list(path)
                        newPath.append(optionalNode)
                        distanceToGoal = abs(self.goal[0] - optionalNode[0]) + abs(self.goal[1] - optionalNode[1])
                        q.put((costSoFar + distanceToGoal, newPath))

                if (node[1] < len(self.map[0]) - 1):
                    if (self.map[node[0]][node[1]+1] == 0) and not [node[0], node[1]+1] in path:
                        optionalNode = [node[0], node[1] + 1]
                        newPath = list(path)
                        newPath.append(optionalNode)
                        distanceToGoal = abs(self.goal[0] - optionalNode[0]) + abs(self.goal[1] - optionalNode[1])
                        q.put((costSoFar + distanceToGoal, newPath))

    def BFS(self):
        q = queue.Queue()
        q.put([self.start])
        goalReached = False
        while not goalReached:
            path = q.get()
            node = path[-1]

            if node == self.goal:
                goalReached = True
                self.path = path
            else:
                if (node[0] > 0):
                    if (self.map[node[0]-1][node[1]] == 0) and not [node[0]-1, node[1]] in path:
                        newPath = list(path)
                        newPath.append([node[0]-1, node[1]])
                        q.put(newPath)

                if (node[0] < len(self.map) - 1):
                    if (self.map[node[0]+1][node[1]] == 0) and not [node[0]+1, node[1]] in path:
                        newPath = list(path)
                        newPath.append([node[0]+1, node[1]])
                        q.put(newPath)
                
                if (node[1] > 0):
                    if (self.map[node[0]][node[1]-1] == 0) and not [node[0], node[1]-1] in path:
                        newPath = list(path)
                        newPath.append([node[0], node[1] - 1])
                        q.put(newPath)
                if (node[1] < len(self.map[0]) - 1):
                    if (self.map[node[0]][node[1]+1] == 0) and not [node[0], node[1]+1] in path:
                        newPath = list(path)
                        newPath.append([node[0], node[1] + 1])
                        q.put(newPath)

    def printMap(self):
        tempMap = self.map
        try:
            for tile in self.path:
                tempMap[tile[0]][tile[1]] = 2
        except AttributeError:
            pass

        tempMap[self.start[0]][self.start[1]] = 3
        tempMap[self.goal[0]][self.goal[1]] = 4
        palette = np.array([ [255, 255, 255],
                    [0, 0, 0],
                    [0, 0, 255],
                    [255,0,0],
                    [0,255,0]])
        plt.imshow(palette[tempMap])
        plt.axis("off")
        plt.show()

if __name__ == "__main__":
    print("Welcome to Maze Solver")
    print("Input Maze File")
    filename = input()
    while not os.path.isfile(filename):
        print("Error, not found!")
        filename = input()

    L = Labyrinth(filename)
    start = list(map(int, input("Input Start = ").split(" ")))
    print(start)
    while not L.validateNode(start):
        print("Error, your input cannot choose as start node")
        start = list(map(int, input("Input Start = ").split(" ")))
    goal = list(map(int, input("Input Goal = ").split(" ")))
    while not L.validateNode(goal):
        print("Error, your input cannot choose as start node")
        goal = list(map(int, input("Input Goal = ").split(" ")))
    L.setStartAndGoal(start, goal)
    # L.printMap()

    isExit = False
    while not isExit:
        print("Choose Method")
        print("1. AStar")
        print("2. BFS")
        print("3. Exit")
        method = input(">>")
        if (method.lower() == "astar" or method == '1'):
            L.AStar()
            L.printMap()
        elif (method.lower() == "bfs" or method == '2'):
            L.BFS()
            L.printMap()
        elif (method.lower() == "exit" or method == '3'):
            isExit = True
        else:
            print("Command not Found")
    