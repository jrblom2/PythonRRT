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

def circleListContainsPoint(circleList, point):
    containsPoint = False
    for circle in circleList:
        if circle.contains_point(point):
            containsPoint = True
    return containsPoint

graphIterations = 500
rootX = 50
rootY = 50
configX = 100
configY = 100

#Circle setup
numCircles = 10
circleMaxRadius = 10
circleList = []
for i in range(numCircles):
    circleX = random.uniform(0, configX)
    circleY = random.uniform(0, configY)
    circleRadius = random.uniform(3, circleMaxRadius)
    newCircle = patches.Circle((circleX, circleY), circleRadius, color='black')
    circleList.append(newCircle)

#Initial point setup
while True:
    rootX = random.uniform(0, configX)
    rootY = random.uniform(0, configY)
    containsRoot = circleListContainsPoint(circleList, (rootX, rootY))
    if not containsRoot:
        tree = Ts.Tree(rootX, rootY)
        break

#Goal point setup
while True:
    goalX = random.uniform(0, configX)
    goalY = random.uniform(0, configY)
    containsGoal = circleListContainsPoint(circleList, (goalX, goalY))
    if not containsGoal:
        tree.goalCoords = (goalX, goalY)
        break

steps = 0
while steps < graphIterations:
    randomX = random.uniform(0, configX)
    randomY = random.uniform(0, configY)

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
    newLine = [closestNode.coords, newNode.coords]
    newNode.parentNode = closestNode
    newNode.parentLine = newLine

    #Check collision
    newPointCollides = False
    for circle in circleList:
        #get closest point on new line to circle
        closestPoint = p(newLine[0], newLine[1], circle.center)
        if circle.contains_point(closestPoint):
            newPointCollides = True

    if not newPointCollides:
        tree.nodeList.append(newNode)
        tree.coordList.append(newNode.coords)
        closestNode.children.append(newNode)
        closestNode.linesToChildren.append(newLine)
        tree.edgeList.append([closestNode.coords, newNode.coords])
        steps = steps + 1

#Matplotlib graph plotting
fig, ax = plt.subplots()
for circle in circleList:
    ax.add_patch(circle)
lines = LineCollection(tree.edgeList)
ax.add_collection(lines)
###[1]###
plt.scatter(*zip(*tree.coordList), marker='.')
plt.show()

