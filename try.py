import argparse
import time
import msgpack
from enum import Enum, auto
import csv
import random

import numpy as np

from planning_utils import a_star, heuristic, create_grid, get_lonlat_limits
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local, local_to_global


TARGET_ALTITUDE = 5
SAFETY_DISTANCE = 5

data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

# minimum and maximum east coordinates
east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

# given the minimum and maximum coordinates we can
# calculate the size of the grid.
north_size = int(np.ceil(north_max - north_min))
east_size = int(np.ceil(east_max - east_min))

grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

# lon_min, lon_max, lat_min, lat_max = get_lonlat_limits(north_min, north_max, east_min, east_max, self.global_home)

print(north_size,east_size)

