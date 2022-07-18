import csv
import math
import numpy as np

from orientationSensor import OrientationSensor

# Fit a sphere to a point cloud
def sphereFit(spX, spY, spZ):
    # Assemble the A matrix
    A = np.ones((len(spX), 4))
    A[:, 0] = 2 * spX
    A[:, 1] = 2 * spY
    A[:, 2] = 2 * spZ

    # Assemble the f matrix
    f = np.zeros((len(spX), 1))
    f[:, 0] = (spX * spX) + (spY * spY) + (spZ * spZ)
    x, _, _, _ = np.linalg.lstsq(A, f, rcond=None)

    # Solve for the radius
    r = math.sqrt((x[0] * x[0]) + (x[1] * x[1]) + (x[2] * x[2]) + x[3])

    return r, x[0], x[1], x[2]

