# FCND - 3D Motion Planning
![Quad Image](./misc/enroute.png)



This project is a continuation of the Backyard Flyer project where you executed a simple square shaped flight path. In this project you will integrate the techniques that you have learned throughout the last several lessons to plan a path through an urban environment. Check out the [project rubric](https://review.udacity.com/#!/rubrics/1534/view) for more detail on what constitutes a passing submission.

### Here I will describe how I address each point in the rubric

### README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf.

That's exactly what you are reading! :)

### Explain the Starter Code
These scripts contain a basic planning implementation that includes:
1. `planning_utils.py`:
    1. `create_grid(data, drone_altitude, safety_distance)`:
        Returns a 2D grid representation of the configuration space based on obstacles defined in the csv file provided, *drone_altitude* and *safety_distance* arguments.
    2. `Action`:
        Defines possible navigation actions. Actions are defined as tupples where the first 2 values are the delta of the action relative to the current position, and the third value is the cost to perform the action.
        
        `Action` class methods:
        
            cost: returns the cost to perform the action.
            delta: returns the delta of the action relative to the current position.
        
    3. `valid_actions(grid, current_node)`:
        Returns a list of possible valid actions given a *grid* and *current_node* arguments by checking obstacles on the *grid*.
    4. `a_star(grid, h, start, goal)`:
        Returns a path between the *start* and *goal* by implementing the A\* algorithm over the given *gird* and based on the heuristic function, *h*.
    5. `heuristic(position, goal_position)`:
        Returns the heuristic cost of the path from *position* to *goal_position*.

2. `motion_planning.py`:
    - `motion_planning.py` is an extension to `backyard_flyer_solution.py`, in which a new `plan_path` method and `PLANNING` state are added to incorporate path planning methods (here A\*) to find a path from start to goal positions.
    `plan_path` method simply utilizes functions in `planning_utils.py` to do a path planning on a grid created based on the csv file. It calculates a path and converts it into multiple way points so that the drone will fly through those waypoints.
    The `PLANNING` state occurs after `ARMING` and before `TAKEOFF`.

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
The `lat0` and `lon0` values are extracted from colliders csv file:
```python
# read lat0, lon0 from colliders into floating point values
with open('colliders.csv', newline='') as f:
    reader = csv.reader(f)
    row1 = next(reader)

lat = row1[0].split()
lat0 = float(lat[1])
lon = row1[1].split()
lon0 = float(lon[1])
```
Use the `self.set_home_position()` method to set global home based on `lat0` and `lon0`:
```python
# set home position to (lon0, lat0, 0)
self.set_home_position(lon0, lat0, 0)
```

#### 2. Set the current local position
Firstly, retrieve the current position in geodetic coordinates from `self._latitude`, `self._longitude` and `self._altitude`. Then use the utility function global_to_local() to convert to local position:
```python
# retrieve current global position
current_global_position = [self._longitude, self._latitude,  self._altitude]
# convert to current local position using global_to_local()
current_local_position = global_to_local(current_global_position, self.global_home)
```

#### 3. Set grid start position to be the current local position
This is done by appling the north and east offsets to the local position to get the grid start location:
```python
# convert start position to current position rather than map center
grid_start = (int(current_local_position[0]) - north_offset, int(current_local_position[1]) - east_offset)
```

#### 4. Set grid goal position from geodetic coordinates
Create a `get_lonlat_limits` function in `planning_tils.py` which returns the minimum and maximum of both latitude and longtitude based on the csv file:
```python
def get_lonlat_limits(north_min, north_max, east_min, east_max, global_home):
    southwest = (north_min, east_min, 0)
    southeast = (north_min, east_max, 0)
    northeast = (north_max, east_max, 0)
    northwest = (north_max, east_min, 0)
    global_limits = [local_to_global(c, global_home) for c in [southwest, southeast, northeast, northwest]]
    dummy = np.vstack((np.min(global_limits, 0), np.max(global_limits, 0))).T
    lon_min, lon_max = dummy[0]
    lat_min, lat_max = dummy[1]
    return lon_min, lon_max, lat_min, lat_max
```
In `motion_planning.py`, generate a random goal coordinate within the map using `get_lonlat_limits`, then use `global_to_local` method to convert this random goal coordinate to a local goal position in the map, finally the grid goal position is set by applying the offsets:
```python
# adapt to set goal as latitude / longitude position and convert
# minimum and maximum north coordinates
north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

# minimum and maximum east coordinates
east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))
# limits of lon and lat
lon_min, lon_max, lat_min, lat_max = get_lonlat_limits(north_min, north_max, east_min, east_max, self.global_home)

# determine whether the goal is feasible
feasibility = False
while not feasibility:
    goal_lon = np.random.uniform(lon_min, lon_max)
    goal_lat = np.random.uniform(lat_min, lat_max)
    goal_position = global_to_local((goal_lon, goal_lat, 0), self.global_home)
    grid_goal = (int(goal_position[0]) - north_offset, int(goal_position[1]) - east_offset)
    # If the computing time is too long, there would be a connection error
    if np.linalg.norm(np.array(grid_start) - np.array(grid_goal)) <= 300:
        feasibility = grid[grid_goal[0]][grid_goal[1]] != 1
    else:
        feasibility = False
```

#### 5. Update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2)
Define a set of diagonal actions in `Action` class in `planning_utils.py`:
```python
class Action(Enum):

    WEST = (0, -1, 1)
    SOUTHWEST = (1, -1, np.sqrt(2))
    SOUTH = (1, 0, 1)
    SOUTHEAST = (1, 1, np.sqrt(2))
    EAST = (0, 1, 1)
    NORTHEAST = (-1, 1, np.sqrt(2))
    NORTH = (-1, 0, 1)
    NORTHWEST = (-1, -1, np.sqrt(2))
```
Modify `valid_actions` method according to the added diagonal actions:
```python
if x - 1 < 0 or grid[x - 1, y] == 1:
    valid_actions.remove(Action.NORTH)
if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
    valid_actions.remove(Action.NORTHWEST)
if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
    valid_actions.remove(Action.NORTHEAST)
if x + 1 > n or grid[x + 1, y] == 1:
    valid_actions.remove(Action.SOUTH)
if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
    valid_actions.remove(Action.SOUTHWEST)
if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
    valid_actions.remove(Action.SOUTHEAST)
if y - 1 < 0 or grid[x, y - 1] == 1:
    valid_actions.remove(Action.WEST)
if y + 1 > m or grid[x, y + 1] == 1:
    valid_actions.remove(Action.EAST)
```

#### 6. Cull waypoints
Use a collinearity test to prune the path of unnecessary waypoints:
```python
def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)
# use determinant to check collinearity
def collinearity_check(p1, p2, p3, epsilon=1e-6):   
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

if path is not None:
    pruned_path = [p for p in path]
# prune the path!
    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i + 1])
        p3 = point(pruned_path[i + 2])
        if collinearity_check(p1, p2, p3, epsilon=1e-6):
            pruned_path.remove(pruned_path[i + 1])
        else:
            i += 1        
else:
    pruned_path = path
```

### Execute the flight
#### It works!!!

![Motion Planning Demo](./result1.gif)

# Extra Challenges
The submission requirements for this project are laid out in the rubric, but if you feel inspired to take your project above and beyond, or maybe even keep working on it after you submit, then here are some suggestions for interesting things to try.

<!-- ## Option to do this project in a GPU backed virtual machine in the Udacity classroom!
Rather than downloading the simulator and starter files you can simply complete this project in a virual workspace in the Udacity classroom! Follow [these instructions](https://classroom.udacity.com/nanodegrees/nd787/parts/5aa0a956-4418-4a41-846f-cb7ea63349b3/modules/0c12632a-b59a-41c1-9694-2b3508f47ce7/lessons/5f628104-5857-4a3f-93f0-d8a53fe6a8fd/concepts/ab09b378-f85f-49f4-8845-d59025dd8a8e?contentVersion=1.0.0&contentLocale=en-us) to proceed with the VM. 

## To complete this project on your local machine, follow these instructions:
### Step 1: Download the Simulator
This is a new simulator environment!  

Download the Motion-Planning simulator for this project that's appropriate for your operating system from the [simulator releases respository](https://github.com/udacity/FCND-Simulator-Releases/releases).

### Step 2: Set up your Python Environment
If you haven't already, set up your Python environment and get all the relevant packages installed using Anaconda following instructions in [this repository](https://github.com/udacity/FCND-Term1-Starter-Kit)

### Step 3: Clone this Repository
```sh
git clone https://github.com/udacity/FCND-Motion-Planning
```
### Step 4: Test setup
The first task in this project is to test the [solution code](https://github.com/udacity/FCND-Motion-Planning/blob/master/backyard_flyer_solution.py) for the Backyard Flyer project in this new simulator. Verify that your Backyard Flyer solution code works as expected and your drone can perform the square flight path in the new simulator. To do this, start the simulator and run the [`backyard_flyer_solution.py`](https://github.com/udacity/FCND-Motion-Planning/blob/master/backyard_flyer_solution.py) script.

```sh
source activate fcnd # if you haven't already sourced your Python environment, do so now.
python backyard_flyer_solution.py
```
The quad should take off, fly a square pattern and land, just as in the previous project. If everything functions as expected then you are ready to start work on this project. 

### Step 5: Inspect the relevant files
For this project, you are provided with two scripts, `motion_planning.py` and `planning_utils.py`. Here you'll also find a file called `colliders.csv`, which contains the 2.5D map of the simulator environment. 

### Step 6: Explain what's going on in  `motion_planning.py` and `planning_utils.py`

`motion_planning.py` is basically a modified version of `backyard_flyer.py` that leverages some extra functions in `planning_utils.py`. It should work right out of the box.  Try running `motion_planning.py` to see what it does. To do this, first start up the simulator, then at the command line:
 
```sh
source activate fcnd # if you haven't already sourced your Python environment, do so now.
python motion_planning.py
```

You should see the quad fly a jerky path of waypoints to the northeast for about 10 m then land.  What's going on here? Your first task in this project is to explain what's different about `motion_planning.py` from the `backyard_flyer_solution.py` script, and how the functions provided in `planning_utils.py` work. 

### Step 7: Write your planner

Your planning algorithm is going to look something like the following:

- Load the 2.5D map in the `colliders.csv` file describing the environment.
- Discretize the environment into a grid or graph representation.
- Define the start and goal locations. You can determine your home location from `self._latitude` and `self._longitude`. 
- Perform a search using A* or other search algorithm. 
- Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
- Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0]). 

Some of these steps are already implemented for you and some you need to modify or implement yourself.  See the [rubric](https://review.udacity.com/#!/rubrics/1534/view) for specifics on what you need to modify or implement.

### Step 8: Write it up!
When you're finished, complete a detailed writeup of your solution and discuss how you addressed each step. You can use the [`writeup_template.md`](./writeup_template.md) provided here or choose a different format, just be sure to describe clearly the steps you took and code you used to address each point in the [rubric](https://review.udacity.com/#!/rubrics/1534/view). And have fun!

## Extra Challenges
The submission requirements for this project are laid out in the rubric, but if you feel inspired to take your project above and beyond, or maybe even keep working on it after you submit, then here are some suggestions for interesting things to try.

### Try flying more complex trajectories
In this project, things are set up nicely to fly right-angled trajectories, where you ascend to a particular altitude, fly a path at that fixed altitude, then land vertically. However, you have the capability to send 3D waypoints and in principle you could fly any trajectory you like. Rather than simply setting a target altitude, try sending altitude with each waypoint and set your goal location on top of a building!

### Adjust your deadbands
Adjust the size of the deadbands around your waypoints, and even try making deadbands a function of velocity. To do this, you can simply modify the logic in the `local_position_callback()` function.

### Add heading commands to your waypoints
This is a recent update! Make sure you have the [latest version of the simulator](https://github.com/udacity/FCND-Simulator-Releases/releases). In the default setup, you're sending waypoints made up of NED position and heading with heading set to 0 in the default setup. Try passing a unique heading with each waypoint. If, for example, you want to send a heading to point to the next waypoint, it might look like this:

```python
# Define two waypoints with heading = 0 for both
wp1 = [n1, e1, a1, 0]
wp2 = [n2, e2, a2, 0]
# Set heading of wp2 based on relative position to wp1
wp2[3] = np.arctan2((wp2[1]-wp1[1]), (wp2[0]-wp1[0]))
```

This may not be completely intuitive, but this will yield a yaw angle that is positive counterclockwise about a z-axis (down) axis that points downward.

Put all of these together and make up your own crazy paths to fly! Can you fly a double helix?? 
![Double Helix](./misc/double_helix.gif)

Ok flying a double helix might seem like a silly idea, but imagine you are an autonomous first responder vehicle. You need to first fly to a particular building or location, then fly a reconnaissance pattern to survey the scene! Give it a try! -->