import numpy as np


class Sphere:

    def __init__(self, center, radius):
        self.center = center
        self.radius = radius

        self.phi = np.linspace(0, np.pi, 100)
        self.theta = np.linspace(0, 2 * np.pi, 100)

    def containsPoint(self, point):
        x = (point[0] - self.center[0]) ** 2
        y = (point[1] - self.center[1]) ** 2
        z = (point[2] - self.center[2]) ** 2
        return (x + y + z) < self.radius**2

    def plotMesh(self, axis):
        phi, theta = np.meshgrid(self.phi, self.theta)
        x = self.center[0] + self.radius * np.sin(phi) * np.cos(theta)
        y = self.center[1] + self.radius * np.sin(phi) * np.sin(theta)
        z = self.center[2] + self.radius * np.cos(phi)
        axis.plot_surface(x, y, z, color='#FF5733', alpha=0.8)  # Orange color
