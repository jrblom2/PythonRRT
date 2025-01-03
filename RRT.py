import TreeClass as Ts
from Sphere import Sphere
import random
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from mpl_toolkits.mplot3d import Axes3D

import plotly
import plotly.graph_objects as go


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


def doRRT(seed=None):
    graphIterations = 500
    maxChecks = 10000
    rootX = 50
    rootY = 50
    configX = 100
    configY = 100
    configZ = 100

    # Sphere setup
    numSpheres = 10
    sphereMaxRadius = 50
    sphereList = []
    doSpheres = True
    if seed is None:
        seed = random.randint(0, 1000)
        print('Seed: ', seed)
    random.seed(seed)
    if doSpheres:
        for i in range(numSpheres):
            sphereX = random.uniform(0, configX)
            sphereY = random.uniform(0, configY)
            sphereZ = random.uniform(0, configZ)
            sphereRadius = random.uniform(20, sphereMaxRadius)
            newSphere = Sphere((sphereX, sphereY, sphereZ), sphereRadius)
            sphereList.append(newSphere)

    # Initial point setup
    while True:
        rootX = random.uniform(0, configX)
        rootY = random.uniform(0, configY)
        rootZ = random.uniform(0, configZ)
        containsRoot = sphereListContainsPoint(
            sphereList, (rootX, rootY, rootZ)
        )
        if not containsRoot:
            tree = Ts.Tree(rootX, rootY, rootZ)
            break

    # Goal point setup
    while True:
        goalX = random.uniform(0, configX)
        goalY = random.uniform(0, configY)
        goalZ = random.uniform(0, configZ)
        containsGoal = sphereListContainsPoint(
            sphereList, (goalX, goalY, goalZ)
        )
        if not containsGoal:
            tree.goalNode = Ts.Node(np.array([goalX, goalY, goalZ]))
            break

    steps = 0
    totalChecks = 0
    lastNodeCreated = tree.rootNode
    while steps < graphIterations and totalChecks < maxChecks:
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
        randomZ = random.uniform(0, configZ)

        randomNode = Ts.Node(np.array([randomX, randomY, randomZ]))

        # find closest node in tree to random node
        closestNode = tree.smallestDistanceSearch(randomNode)

        distance = closestNode.calcDistance(randomNode)

        closestNodeX = closestNode.coords[0]
        closestNodeY = closestNode.coords[1]
        closestNodeZ = closestNode.coords[2]

        # normalize distance and add to existing coords
        newX = (
            (randomNode.coords[0] - closestNodeX) / distance
        ) * tree.incrimentalDistance
        newY = (
            (randomNode.coords[1] - closestNodeY) / distance
        ) * tree.incrimentalDistance
        newZ = (
            (randomNode.coords[2] - closestNodeZ) / distance
        ) * tree.incrimentalDistance

        newNode = Ts.Node(
            np.array(
                [newX + closestNodeX, newY + closestNodeY, newZ + closestNodeZ]
            )
        )

        # Housekeeping
        newLine = [closestNode.coords, newNode.coords]
        newNode.parentNode = closestNode
        newNode.parentLine = newLine

        # Check collision
        newPointCollidesspheres = sphereListCollidesWithLine(
            sphereList, newLine
        )

        if not newPointCollidesspheres:
            tree.nodeList.append(newNode)
            tree.coordList.append(newNode.coords)
            closestNode.children.append(newNode)
            closestNode.linesToChildren.append(newLine)
            tree.edgeList.append(newLine)
            lastNodeCreated = newNode
            steps = steps + 1
        totalChecks = totalChecks + 1

    return tree, sphereList


# Matplotlib graph plotting
def plotWithMatPlot(tree, sphereList):
    goalEdges, goalPoints = tree.buildGoalPathEdgeandPointList()
    fig = plt.figure()
    fig.patch.set_facecolor('#222222')  # Set background color to dark gray
    ax = fig.add_subplot(111, projection='3d')
    ax.set_facecolor('#222222')  # Set plot background color to dark gray

    # Plot spheres if they exist
    for sphere in sphereList:
        sphere.plotMeshMatPlot(ax)

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

    plt.show()


def plotWithPyPlot(tree, sphereList, inputColor):
    goalEdges, goalPoints = tree.buildGoalPathEdgeandPointList()

    data = []
    contours = go.surface.Contours(
        x=go.surface.contours.X(highlight=False),
        y=go.surface.contours.Y(highlight=False),
        z=go.surface.contours.Z(highlight=False),
    )
    for sphere in sphereList:
        X, Y, Z = sphere.plotMeshPyPlot()
        data.append(
            go.Surface(
                x=X,
                y=Y,
                z=Z,
                colorscale=[[0, inputColor], [1, inputColor]],
                showscale=False,
                hoverinfo='skip',
                contours=contours,
            )
        )

    for edge in tree.edgeList:
        point1 = edge[0]
        point2 = edge[1]
        x = [point1[0], point2[0]]
        y = [point1[1], point2[1]]
        z = [point1[2], point2[2]]
        data.append(
            go.Scatter3d(
                x=x,
                y=y,
                z=z,
                mode='lines',
                line=dict(color='blue'),
                hoverinfo='skip',
            )
        )

    for edge in goalEdges:
        point1 = edge[0]
        point2 = edge[1]
        x = [point1[0], point2[0]]
        y = [point1[1], point2[1]]
        z = [point1[2], point2[2]]
        data.append(
            go.Scatter3d(
                x=x,
                y=y,
                z=z,
                mode='lines',
                line=dict(color='red'),
                hoverinfo='skip',
            )
        )

    x = tree.goalNode.coords[0]
    y = tree.goalNode.coords[1]
    z = tree.goalNode.coords[2]
    data.append(
        go.Scatter3d(
            x=[x],
            y=[y],
            z=[z],
            mode='markers',
            marker=dict(
                size=4,
                color='green',
            ),  # Set the desired color
            hoverinfo='skip',
        )
    )

    x = tree.rootNode.coords[0]
    y = tree.rootNode.coords[1]
    z = tree.rootNode.coords[2]
    data.append(
        go.Scatter3d(
            x=[x],
            y=[y],
            z=[z],
            mode='markers',
            marker=dict(size=4, color='yellow'),  # Set the desired color
            hoverinfo='skip',
        )
    )

    fig = go.Figure(data=data)

    camera = dict(eye=dict(x=1.0, y=1.0, z=0.8))

    fig.update_layout(
        showlegend=False,
        scene=dict(
            xaxis=dict(
                showgrid=False,
                showticklabels=False,
                visible=False,
                showspikes=False,
            ),
            yaxis=dict(
                showgrid=False,
                showticklabels=False,
                visible=False,
                showspikes=False,
            ),
            zaxis=dict(
                showgrid=False,
                showticklabels=False,
                visible=False,
                showspikes=False,
            ),
        ),
        autosize=False,
        width=600,
        height=600,
        paper_bgcolor='white',
        plot_bgcolor='white',
        margin=dict(l=0, r=0, t=0, b=0),
        scene_camera=camera,
        hovermode=False,
    )

    plotly.io.renderers.default = "plotly_mimetype+notebook"
    fig.show()
