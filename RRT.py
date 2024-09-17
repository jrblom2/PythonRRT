import TreeClass as Ts
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import LineCollection

###[2]###
def p(p1, p2, p3):
    (x1, y1), (x2, y2), (x3, y3) = p1, p2, p3
    dx, dy = x2-x1, y2-y1
    det = dx*dx + dy*dy
    a = (dy*(y3-y1)+dx*(x3-x1))/det
    return x1+a*dx, y1+a*dy

graphIterations = 100
rootX = 50
rootY = 50

tree = Ts.Tree(rootX, rootY)

#Circle setup
numCircles = 5
circleMaxRadius = 10
circleList = []
for i in range(numCircles):
    circleX = random.uniform(0, tree.configX)
    circleY = random.uniform(0, tree.configY)
    circleRadius = random.uniform(0, circleMaxRadius)
    newCircle = patches.Circle((circleX, circleY), circleRadius)
    circleList.append(newCircle)

for x in range(graphIterations):
    randomX = random.uniform(0, tree.configX)
    randomY = random.uniform(0, tree.configY)

    randomNode = Ts.Node(np.array([randomX, randomY]))

    #find closest node in tree to random node
    closestNode = tree.smallestDistanceSearch(randomNode)

    distance = closestNode.calcDistance(randomNode)

    closestNodeX = closestNode.coords[0]
    closestNodeY = closestNode.coords[1]

    #normalize distance and add to existing coords
    newX = ((randomNode.coords[0] - closestNodeX) / distance) * tree.incrimentalDistance
    newY = ((randomNode.coords[1] - closestNodeY) / distance) * tree.incrimentalDistance

    newNode = Ts.Node(np.array([newX + closestNodeX, newY + closestNodeY]))

    #Housekeeping
    newNode.parentNode = closestNode
    tree.nodeList.append(newNode)
    tree.coordList.append(newNode.coords)
    closestNode.children.append(newNode)

    newLine = [closestNode.coords, newNode.coords]
    newNode.parentLine = newLine
    closestNode.linesToChildren.append(newLine)
    tree.edgeList.append([closestNode.coords, newNode.coords])

#Matplotlib graph plotting
fig, ax = plt.subplots()
print(tree.edgeList)
lines = LineCollection(tree.edgeList)
ax.add_collection(lines)
###[1]###
plt.scatter(*zip(*tree.coordList))
plt.show()

