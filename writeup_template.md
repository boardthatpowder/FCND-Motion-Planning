## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

The `motion_planning.py` includes the main `MotionPlanning` class which is accountable for guiding the drone along a path. Upon start, it registers 3 callbacks listening to the `LOCAL_POSITION`, `LOCAL_VELOCITY` and `STATE` events. The various states of the drone form a state machine comprised of `MANUAL` > `ARMING` > `PLANNING` > `TAKEOFF` > `WAYPOINT` > `LANDING` > `DISARMING` > `MANUAL`.

Upon a `LOCAL_POSITION` event received, the flight state is checked. If `TAKEOFF` and the local position is within 5% of the target the state is updated to `WAYPOINT` along with the drone commanded to fly to the first waypoint.

Upon a `LOCAL_VELOCITY` event received, the flight state is checked to see if the state is `LANDING`. If so, and the position is within 0.1 of a grid cell, and the altitude 0.01 of the ground, the drone is disarmed.

Upon a `STATE` event received, if the drone is in mission and the state is one of `MANUAL`, `ARMING` (and armed), `PLANNING`, or `DISARMING` (and not armed or guided), the state machine as described previously is transitioned.

The functionality that occurs on each state change event is as follows:

| State From | Guard | State To |
| --- | --- | --- |
| `MANUAL` | in mission. | `ARMING` | 
| `ARMING` | in mission, and armed. | `PLANNING` |
| `PLANNING` | in mission. | `TAKEOFF` |
| `DISARMING` | in mission, not armed, and not guided. | `MANUAL` |

The last piece of this class that needs explanation is the `plan_path()` method. At this moment it is a skeleton outline of how to plan the path with implementation outstanding which will be completed as part of this project.

The `planning_utils.py` script contains a number of functions/enum related to areas of flight that we have previosuly implemented something similar as part of past lessons:
- `create_grid()` - Returns a grid representation of a 2D configuration space based on given obstacle data, drone altitude and safety distance arguments.
- `Action` enum - encapsulates the delta of the action and the associated cost of each possible action of navigating a grid.
- `valid_actions()` - Returns a list of valid actions given a grid and current node.
- `a_star()` - An implementation of the _a*_ algorithm to find a path from a start to a goal while avoiding obstacles within a grid.
- `heurestic()` - A hueristic cost function based on Manhatton distance.



### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
*Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.*

The 1st line of the `colliders.csv` file contains a latitude and longitude which are the coordinates of the center of the map. This is in the format of `lat0 <latitude>, lon0 <longitude>`. I programatically opened the file, read the first line, then split the string appropriately to retrieve the latitude and longitude values. These were then passed as `self.set_home_position(lon0, lat0, alt0)` to initialize the home position.

#### 2. Set your current local position
*Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.*

Following on from the previous step I first test that my globsl position is as expected:

```python
        # DONE: retrieve current global position
        print('global_position: {}'.format(self.global_position))
```

From here I use the provided `global_to_local()` function to convert to local (NED) position:

```python
        # DONE: convert to current local position using global_to_local()
        n_loc, e_loc, d_loc = global_to_local(self.global_position, self.global_home)
        print('n_loc: {}, e_loc: {}, d_loc: {}'.format(n_loc, e_loc, d_loc))

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
```

#### 3. Set grid start position from local position
*This is another step in adding flexibility to the start location. As long as it works you're good to go!*

As part of defining my grid, I return the north and east offsets:

```python
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, edges, north_offset, east_offset = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        
```

Using these offsets I calculate the start on the grid:

```python
        grid_start = (int(np.ceil(n_loc - north_offset)), int(np.ceil(e_loc - east_offset)))
```

As I'm using a graph to represent my configuration space, I find the closet point within my graph to use as a start location:

```python
        graph_start = closest_point(G, grid_start)
```

#### 4. Set grid goal position from geodetic coords
*This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.*

Using Google Maps I manually chose a goal:

```python
        super_duper_burgers = (-122.39400878361174, 37.792827005959616, 0.)
        grid_goal_global = super_duper_burgers 
```

I converted the geodetic coords to local coords:

```python
        grid_goal_local = global_to_local(grid_goal_global , self.global_home)
```

Then finally found the closets point to the goal within my graph:

```python
        graph_goal = closest_point(G, (grid_goal_local[0], grid_goal_local[1]))
```


#### 5. Modify A* to include diagonal motion (or replace A* altogether)
*Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.*

Instead of modifying A* to include diagonal motions, I refactored it to support a graph.  The differences are as follows:

Modified the function parameters to pass in a graph instead of grid.

```python
    def a_star(graph, h, start, goal):
```

Modified the section of code that used to loop through valid actions to instead loop through all connected nodes of the current node in context, and obtain the cost stored as *weight*

```python

            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                branch_cost = current_cost + cost
                queue_cost = branch_cost + h(next_node, goal)
```

#### 6. Cull waypoints 
*For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.*

I chose to perform a collinearity check in determining which waypoints to prune. As I'm using a graph, I used a larger epsilon of `1e-3`:

```python
def collinearity_check(p1, p2, p3, epsilon=1e-3):   
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

def prune_path(path):
    pruned_path = [p for p in path]
    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])
        if collinearity_check(p1, p2, p3):
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path

def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)
```

### Execute the flight
#### 1. Does it work?
It works!

The following is a representation of the path identified:

![map](pruned_path.png)

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


