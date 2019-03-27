import matplotlib.pyplot as plt
import numpy as np
import queue

class Labyrinth:
    # Attribute

    # Method
    # Initializer with filename as argument
    def __init__(self, filename):
        f = open(filename, "r")
        lines = f.readlines()
        self.map = np.array([[int(num) for num in line.strip('\n')] for line in lines])
        self.start = [11,0]
        self.goal = [39,40]
    
    def AStar(self):
        q = queue.PriorityQueue()

    def BFS(self):
        q = queue.Queue()
        q.put([self.start])
        goalReached = False
        while not goalReached:

            path = q.get()

            node = path[-1]
            # print(node)

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
        #[[1,0],[1,1],[1,2],[1,3],[2,3],[3,3],[3,2],[3,1],[4,1],[5,1],[6,1],[7,1],[8,1],[9,1],[9,2],[9,3],[8,3],[7,3],[6,3],[5,3],[5,4],[5,5],[4,5],[3,5],[2,5],[1,5],[1,6],[1,7],[1,8],[1,9],[2,9],[3,9],[3,8],[3,7],[4,7],[5,7],[5,8],[5,9],[6,9],[7,9],[8,9],[9,9],[9,10]]
        self.BFS()
        tempMap = self.map
        for tile in self.path:
            tempMap[tile[0]][tile[1]] = 2
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
    L = Labyrinth("input.txt")
    L.printMap()