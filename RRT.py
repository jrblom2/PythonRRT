import TreeClass as Ts
from Sphere import Sphere
import random
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from mpl_toolkits.mplot3d import Axes3D


# [2]#
def p(p1, p2, p3):
    (x1, y1, z1), (x2, y2, z2), (x3, y3, z3) = p1, p2, p3
    dx, dy, dz = x2 - x1, y2 - y1, z2 - z1
    det = dx * dx + dy * dy + dz * dz
    a = (dy * (y3 - y1) + dx * (x3 - x1) + dz * (z3 - z1)) / det
    a = min(1, max(0, a))
    return x1 + a * dx, y1 + a * dy, z1 + a * dz


# [2]#


def sphereListContainsPoint(sphereList, point):
    containsPoint = False
    for sphere in sphereList:
        if sphere.containsPoint(point):
            containsPoint = True
    return containsPoint


def sphereListCollidesWithLine(sphereList, line):
    newPointCollides = False
    for sphere in sphereList:
        # get closest point on new line to sphere
        closestPoint = p(line[0], line[1], sphere.center)
        if sphere.containsPoint(closestPoint):
            newPointCollides = True
    return newPointCollides


graphIterations = 500
maxChecks = 1000
rootX = 50
rootY = 50
configX = 100
configY = 100
configZ = 100

# Sphere setup
numSpheres = 30
sphereMaxRadius = 10
sphereList = []
doSpheres = True
if doSpheres:
    for i in range(numSpheres):
        sphereX = random.uniform(0, configX)
        sphereY = random.uniform(0, configY)
        sphereZ = random.uniform(0, configZ)
        sphereRadius = random.uniform(3, sphereMaxRadius)
        newSphere = Sphere((sphereX, sphereY, sphereZ), sphereRadius)
        sphereList.append(newSphere)

# Initial point setup
while True:
    rootX = random.uniform(0, configX)
    rootY = random.uniform(0, configY)
    rootZ = random.uniform(0, configZ)
    containsRoot = sphereListContainsPoint(sphereList, (rootX, rootY, rootZ))
    if not containsRoot:
        tree = Ts.Tree(rootX, rootY, rootZ)
        break

# Goal point setup
while True:
    goalX = random.uniform(0, configX)
    goalY = random.uniform(0, configY)
    goalZ = random.uniform(0, configZ)
    containsGoal = sphereListContainsPoint(sphereList, (goalX, goalY, goalZ))
    if not containsGoal:
        tree.goalNode = Ts.Node(np.array([goalX, goalY, goalZ]))
        break

steps = 0
totalChecks = 0
lastNodeCreated = tree.rootNode
while steps < graphIterations and totalChecks < maxChecks and False:
    # Check for line of sight on goal to last node created, root by default
    potentialGoalLine = [lastNodeCreated.coords, tree.goalNode.coords]
    goalLineCollides = sphereListCollidesWithLine(
        sphereList, potentialGoalLine
    )

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
    newPointCollidesspheres = sphereListCollidesWithLine(sphereList, newLine)

    if not newPointCollidesspheres:
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

fig = plt.figure()
fig.patch.set_facecolor('#222222')  # Set background color to dark gray
ax = fig.add_subplot(111, projection='3d')
ax.set_facecolor('#222222')  # Set plot background color to dark gray

# Plot spheres if they exist
for sphere in sphereList:
    sphere.plotMesh(ax)

ax.set_xlabel('X-axis', color='white')
ax.set_ylabel('Y-axis', color='white')
ax.set_zlabel('Z-axis', color='white')
ax.tick_params(axis='x', colors='white')
ax.tick_params(axis='y', colors='white')
ax.tick_params(axis='z', colors='white')

lines = Line3DCollection(tree.edgeList)
ax.add_collection3d(lines)
# [1]#
ax.scatter(*zip(*tree.coordList), marker='.')
# [1]#

# goal route
goalLines = Line3DCollection(goalEdges)
goalLines.set_color('red')
ax.add_collection3d(goalLines)

plt.scatter(*zip(*goalPoints), marker='.', color='red')
plt.scatter(*goalPoints[0], marker='.', color='green', zorder=10)
plt.scatter(*goalPoints[-1], marker='.', color='yellow', zorder=10)
plt.show()
