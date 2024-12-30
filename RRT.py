import TreeClass as Ts
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import LineCollection

# import imageio.v3 as iio


# [2]#
def p(p1, p2, p3):
    (x1, y1), (x2, y2), (x3, y3) = p1, p2, p3
    dx, dy = x2 - x1, y2 - y1
    det = dx * dx + dy * dy
    a = (dy * (y3 - y1) + dx * (x3 - x1)) / det
    a = min(1, max(0, a))
    return x1 + a * dx, y1 + a * dy


# [2]#


def circleListContainsPoint(circleList, point):
    containsPoint = False
    for circle in circleList:
        if circle.contains_point(point):
            containsPoint = True
    return containsPoint


def circleListCollidesWithLine(circleList, line):
    newPointCollides = False
    for circle in circleList:
        # get closest point on new line to circle
        closestPoint = p(line[0], line[1], circle.center)
        if circle.contains_point(closestPoint):
            newPointCollides = True
    return newPointCollides


def collidesWith2DArray(image, line):
    point1 = line[0]
    point2 = line[1]
    distance = np.linalg.norm(point1 - point2)
    distanceReducer = 10

    stepSize = distance / distanceReducer

    newX = ((point2[0] - point1[0]) / distance) * stepSize
    newY = ((point2[1] - point1[1]) / distance) * stepSize

    collides = False
    newPoint = point1
    for i in range(distanceReducer):
        newPoint = np.array([newX + newPoint[0], newY + newPoint[1]])
        roundedX = round(newPoint[0])
        roundedY = round(newPoint[1])
        print(roundedX)
        print(roundedY)
        if image[roundedX][roundedY][0] != 0:
            collides = True
    print(collides)
    return collides


# Image setup
# im = iio.imread('N_map.png')

graphIterations = 500
maxChecks = 1000
rootX = 50
rootY = 50
configX = 100
configY = 100

# Circle setup
numCircles = 30
circleMaxRadius = 10
circleList = []
doCircles = True
if doCircles:
    for i in range(numCircles):
        circleX = random.uniform(0, configX)
        circleY = random.uniform(0, configY)
        circleRadius = random.uniform(3, circleMaxRadius)
        newCircle = patches.Circle(
            (circleX, circleY), circleRadius, color='black'
        )
        circleList.append(newCircle)

# Initial point setup
while True:
    rootX = random.uniform(0, configX)
    rootY = random.uniform(0, configY)
    containsRoot = circleListContainsPoint(circleList, (rootX, rootY))
    if not containsRoot:
        tree = Ts.Tree(rootX, rootY)
        break

# Goal point setup
while True:
    goalX = random.uniform(0, configX)
    goalY = random.uniform(0, configY)
    containsGoal = circleListContainsPoint(circleList, (goalX, goalY))
    if not containsGoal:
        tree.goalNode = Ts.Node(np.array([goalX, goalY]))
        break

steps = 0
totalChecks = 0
lastNodeCreated = tree.rootNode
while steps < graphIterations and totalChecks < maxChecks:
    # Check for line of sight on goal to last node created, root by default
    potentialGoalLine = [lastNodeCreated.coords, tree.goalNode.coords]
    goalLineCollides = circleListCollidesWithLine(
        circleList, potentialGoalLine
    )
    # goalLineCollidesImage = collidesWith2DArray(im, potentialGoalLine)

    if not goalLineCollides:
        # line of sight found
        tree.goalNode.parentNode = lastNodeCreated
        tree.goalNode.parentLine = potentialGoalLine

        tree.nodeList.append(tree.goalNode)
        tree.coordList.append(tree.goalNode.coords)
        lastNodeCreated.children.append(tree.goalNode)
        lastNodeCreated.linesToChildren.append(potentialGoalLine)
        tree.edgeList.append(potentialGoalLine)
        tree.goalFound = True
        break

    # If no line of sight, try to create a new random node
    randomX = random.uniform(0, configX)
    randomY = random.uniform(0, configY)

    randomNode = Ts.Node(np.array([randomX, randomY]))

    # find closest node in tree to random node
    closestNode = tree.smallestDistanceSearch(randomNode)

    distance = closestNode.calcDistance(randomNode)

    closestNodeX = closestNode.coords[0]
    closestNodeY = closestNode.coords[1]

    # normalize distance and add to existing coords
    newX = (
        (randomNode.coords[0] - closestNodeX) / distance
    ) * tree.incrimentalDistance
    newY = (
        (randomNode.coords[1] - closestNodeY) / distance
    ) * tree.incrimentalDistance

    newNode = Ts.Node(np.array([newX + closestNodeX, newY + closestNodeY]))

    # Housekeeping
    newLine = [closestNode.coords, newNode.coords]
    newNode.parentNode = closestNode
    newNode.parentLine = newLine

    # Check collision
    newPointCollidesCircles = circleListCollidesWithLine(circleList, newLine)
    # newPointCollidesImage = collidesWith2DArray(im, newLine)

    if not newPointCollidesCircles:
        tree.nodeList.append(newNode)
        tree.coordList.append(newNode.coords)
        closestNode.children.append(newNode)
        closestNode.linesToChildren.append(newLine)
        tree.edgeList.append(newLine)
        lastNodeCreated = newNode
        steps = steps + 1
    totalChecks = totalChecks + 1

# Matplotlib graph plotting
goalEdges, goalPoints = tree.buildGoalPathEdgeandPointList()

fig, ax = plt.subplots()

# Plot circles if they exist
for circle in circleList:
    ax.add_patch(circle)

# Plot image, if it exists
# plt.imshow(im, cmap='gray', origin='lower')

lines = LineCollection(tree.edgeList)
ax.add_collection(lines)
# [1]#
plt.scatter(*zip(*tree.coordList), marker='.')
# [1]#

# goal route
goalLines = LineCollection(goalEdges)
goalLines.set_color('red')
ax.add_collection(goalLines)
plt.scatter(*zip(*goalPoints), marker='.', color='red')
plt.scatter(*goalPoints[0], marker='.', color='green', zorder=10)
plt.scatter(*goalPoints[-1], marker='.', color='yellow', zorder=10)
plt.show()
