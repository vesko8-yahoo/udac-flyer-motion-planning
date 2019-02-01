## Project: 3D Motion Planning

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
These scripts contain a basic planning implementation that includes creating a grid map and using this grid map to plan.
As part of this they read collider.csv which contains the obstacles and that is included in their grid map.
They start at the middle of the map and the destination is 10 diagonal grid cells away. Since planning_utils.py
only allows for non-diagonal movementsit the path is not a straight line.

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position

I am using python to read the first line of the file and then I parse out the lat0 and lon0 values, and pass
them to self.set_home_position().

#### 2. Set your current local position
I use a function: global_to_local() in which I use the utm library to convert from global to local frame.

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.
You can do this by using the function `def local_to_global(self, local_position, global_home)` which I added.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
I have modified the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2). I modified the Action class to add additional allowable actions. Also had to modfify the `valid_actions` method to account for the new actions.

#### 6. Cull waypoints 
I use a collinearity test to prune waypoints. Implemented the following function: `prune_path(self, path)`. It looks at every 3 consecutive points on the path, and if they are colinear removes the middle point from the path. For the collinearity see the `collinearity_check` function. It uses  a trick from linear algebra, where it arranges the 3 points as rows of a 3 by 3 matrix with the last column of the matrix set to all 1s. If the determinant of that matrix is 0 then the 3 points are colinear.


### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


