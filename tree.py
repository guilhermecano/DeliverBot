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

laserPath = np.genfromtxt('points_laser.txt')
tree = np.genfromtxt('tree.txt')
tree_patch = mpatches.Patch(color='red', label='Nodes')
routes_patch = mpatches.Patch(color='yellow', label='Routes')
lasers_patch = mpatches.Patch(color='black', label='Lasers')

plt.style.use('plotConfig.mplstyle')
# plt.title('RAW DATA')
plt.legend(handles=[
tree_patch,
lasers_patch,routes_patch])

plt.plot(laserPath[:, 0],laserPath[:, 1], 'ro', color = 'k', linewidth = 1)
plt.plot(tree[:,::2].T, tree[:, 1::2].T, 'y', linewidth = 1)
plt.plot(tree[:, 0],tree[:, 1], 'ro', color = 'r', linewidth = 3)
plt.plot(tree[:, 2],tree[:, 3], 'ro', color = 'r', linewidth = 3)

plt.show()
