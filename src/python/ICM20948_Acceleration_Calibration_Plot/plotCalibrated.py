############################
#          BABANZ          #
############################
# PLOT UNCALIBRTED AND CALIBRATED ACCELEROMETER MEASUREMENTS VIA MAGNETO
# Credit: Michael Wrona


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from skspatial.objects import Sphere


# Define calibration parameters
A = np.array([[0.998849, -0.000332, -0.001715],  # 'A^-1' matrix from Magneto
              [-0.000332, 0.998278, 0.000661],
              [-0.001715, 0.000661, 0.992787]])
# 'Combined bias (b)' vector from Magneto
b = np.array([-0.000849, -0.202153, 0.041852])

# Read raw data and apply calibration
rawData = np.genfromtxt('acceldata.txt', delimiter='\t')  # raw measurement file
units = 'm/s^2\'s'  # units of accelerometer measurements (used for axis labels)

N = len(rawData)
calibData = np.zeros((N, 3), dtype='float')
for i in range(N):
    currMeas = np.array([rawData[i, 0], rawData[i, 1], rawData[i, 2]])
    calibData[i, :] = A @ (currMeas - b)

# Plot XY data
plt.figure()
plt.plot(rawData[:, 0], rawData[:, 1], 'bx', label='Raw Meas.')
plt.plot(calibData[:, 0], calibData[:, 1], 'rx', label='Calibrated Meas.')
plt.title('XY Accelerometer Data')
plt.xlabel('X [{}]'.format(units))
plt.ylabel('Y [{}]'.format(units))
plt.legend()
plt.grid()
plt.axis('equal')

# Plot YZ data
plt.figure()
plt.plot(rawData[:, 1], rawData[:, 2], 'bx', label='Raw Meas.')
plt.plot(calibData[:, 1], calibData[:, 2], 'rx', label='Calibrated Meas.')
plt.title('YZ Accelerometer Data')
plt.xlabel('Y [{}]'.format(units))
plt.ylabel('Z [{}]'.format(units))
plt.legend()
plt.grid()
plt.axis('equal')

# Plot XZ data
plt.figure()
plt.plot(rawData[:, 0], rawData[:, 2], 'bx', label='Raw Meas.')
plt.plot(calibData[:, 0], calibData[:, 2], 'rx', label='Calibrated Meas.')
plt.title('XZ Accelerometer Data')
plt.xlabel('X [{}]'.format(units))
plt.ylabel('Z [{}]'.format(units))
plt.legend()
plt.grid()
plt.axis('equal')

# Plot 3D scatter
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
for i in range(N):
    xraw = rawData[i, 0]
    yraw = rawData[i, 1]
    zraw = rawData[i, 2]

    xcalib = calibData[i, 0]
    ycalib = calibData[i, 1]
    zcalib = calibData[i, 2]

    ax.scatter(xraw, yraw, zraw, color='r', marker='x', label='Raw')
    ax.scatter(xcalib, ycalib, zcalib, color='b', marker='x', label='Calibrated')

ax.set_title('3D Scatter Plot of Accelerometer Data')
ax.set_xlabel('X [{}]'.format(units))
ax.set_ylabel('Y [{}]'.format(units))
ax.set_zlabel('Z [{}]'.format(units))
ax.legend(['Raw', 'Calibrated'])

sphere = Sphere([0, 0, 0], 9.8066)
sphere.plot_3d(ax, alpha=0.2)

plt.show()