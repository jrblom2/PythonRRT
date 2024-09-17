import numpy as np

class Tree:
    
    edgeList = []

    nodeList = []

    coordList = []

    incrimentalDistance = 2

    def __init__(self, rootX, rootY):
        self.rootNode = Node(np.array([rootX, rootY]))
        self.nodeList.append(self.rootNode)
        self.coordList.append(self.rootNode.coords)
        self.goalCoords = (0,0)
    
    def smallestDistanceSearch(self, compareNode, currentNode=None):
        if currentNode == None:
            currentNode = self.rootNode
        
        bestFoundNode = currentNode
        for node in currentNode.children:
            bestofChildren = self.smallestDistanceSearch(compareNode, node)
            if bestofChildren.calcDistance(compareNode) < bestFoundNode.calcDistance(compareNode):
                bestFoundNode = bestofChildren
        
        return bestFoundNode

class Node:

    #coords is a list of two floats indicating x-y coords
    coords = np.array([])

    children = []
    linesToChildren = []


    def __init__(self, coords, parentNode=None, parentLine=None):
        self.coords = coords
        self.parentNode = parentNode
        self.parentLine = parentLine
        self.children = []

    def calcDistance(self, anotherNode):
        return np.linalg.norm(self.coords - anotherNode.coords)
    
    def printNode(self):
        print("x: ", self.coords[0], " y: ", self.coords[1])