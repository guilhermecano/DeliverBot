import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors as mcolors
import matplotlib.patches as mpatches
from sklearn import neighbors
import matplotlib.dates as mdates
import matplotlib.animation as animation
from datetime import datetime
import math
from scipy.interpolate import splprep, splev

robotPath = np.genfromtxt('gt.txt')
laserPath = np.genfromtxt('points_laser.txt')
# nodes = np.genfromtxt('nodes.txt')
robot_patch = mpatches.Patch(color='yellow', label='Robot')
nodes_patch = mpatches.Patch(color='red', label='Nodes')
lasers_patch = mpatches.Patch(color='black', label='Laser')

plt.style.use('plotConfig.mplstyle')
plt.title('RAW DATA')
plt.legend(handles=[
robot_patch,
lasers_patch])

plt.plot(robotPath[:, 0],robotPath[:, 1], 'ro', color = 'y', linewidth = 1)
plt.plot(laserPath[:, 0],laserPath[:, 1], 'ro', color = 'k', linewidth = 1)

plt.show()
