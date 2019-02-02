import argparse
import time
import msgpack
from enum import Enum, auto

from skimage.morphology import medial_axis
from skimage.util import invert
import utm
import numpy as np

from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    #This is from Less02
    def global_to_local(self, global_position, global_home):
        (east_home, north_home, _, _) = utm.from_latlon(global_home[1], global_home[0])
        (east, north, _, _) = utm.from_latlon(global_position[1], global_position[0])                 
        local_position = np.array([north - north_home, east - east_home, -(global_position[2] - global_home[2])])
    
        return local_position

    def local_to_global(self, local_position, global_home):
        (east_home, north_home, zone_number, zone_letter) = utm.from_latlon(
                                                            global_home[1], global_home[0])
        
        (lat, lon) = utm.to_latlon(east_home + local_position[1],
                                north_home + local_position[0], zone_number,
                                zone_letter)
                                
        global_position = np.array([lon, lat, -(local_position[2]-global_home[2])])
        
        return global_position

    #V
    def point(self, p):
        #This reshape() just adds an extra dimention at the end with the number 1 eg:
        #[[316. 445.]] becomes [[316. 445.   1.]]
        return np.array([p[0], p[1], 1.]).reshape(1, -1)

    #V
    def collinearity_check(self, p1, p2, p3, epsilon=1e-6):
        #using the point method above we have 3 points and we can jst take the determinant
        # np.concatenate defaults to axis=0 so each point becomes a row in the m matrix:
        #[[316. 445.   1.]
        #[317. 446.   1.]
        #[318. 447.   1.]]
        m = np.concatenate((p1, p2, p3), 0)
        det = np.linalg.det(m)
        return abs(det) < epsilon

    #V I am using collinearity here instead of Bresenham method.
    def prune_path(self, path):
        pruned_path = [p for p in path]
        
        i = 0
        while i < len(pruned_path) - 2:
            p1 = self.point(pruned_path[i])
            p2 = self.point(pruned_path[i+1])
            p3 = self.point(pruned_path[i+2])
            
            # If the 3 points are in a line remove
            # the 2nd point.
            # The 3rd point now becomes and 2nd point
            # and the check is redone with a new third point
            # on the next iteration.
            if self.collinearity_check(p1, p2, p3):
                pruned_path.remove(pruned_path[i+1])
            else:
                i += 1
        return pruned_path

    def find_start_goal(self, skel, start, goal):
        skel_cells = np.transpose(skel.nonzero())
        start_min_dist = np.linalg.norm(np.array(start) - np.array(skel_cells), axis=1).argmin()
        near_start = skel_cells[start_min_dist]
        goal_min_dist = np.linalg.norm(np.array(goal) - np.array(skel_cells), axis=1).argmin()
        near_goal = skel_cells[goal_min_dist]
        
        return near_start, near_goal

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 6

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: DONE read lat0, lon0 from colliders into floating point values
        # TODO: DONE set home position to (lon0, lat0, 0)
        global_pos = (self.global_position[0], self.global_position[1], self.global_position[2])
        print("GLOBAL_POSITION1: ", global_pos)
        #GLOBAL_POSITION1:  (-122.3974519, 37.7924801, 0.108)
        first_line = "" 
        with open('colliders.csv') as f:
            first_line = f.readline()
        print(first_line)
        print(self._latitude, self._longitude)
        lines = first_line.split(",")
        #print(lines)
        #['lat0 37.792480', ' lon0 -122.397450\n']
        lat = lines[0].strip(' \t\n\rlat0')
        longit = lines[1].strip(' \t\n\rlon0')
        self.set_home_position(float(longit), float(lat), 0)
        #V: In real life we will read our current position from the GPS using self._longitude and self._latitude
        #which is the same as the values in self.global_position (note that longitude is first before latitude)
        #self.set_home_position(self._longitude, self._latitude, 0)
        # TODO: DONE retrieve current global position
        global_pos = (self.global_position[0], self.global_position[1], self.global_position[2])
        print("GLOBAL_POSITION2: ", global_pos)
        # TODO: DONE convert to current local position using global_to_local()
        local_pos = self.global_to_local(global_pos, (self._longitude, self._latitude, 0))
        print(local_pos) # [ 0.     0.    -0.108]

        #V - The below print outputs this if you run it fresh. Note 3.7e+01=37; 1.22e+02=122
        #so self.global_home = self.global_position. But why are they the same. What is the difference.
        #global home [-122.39745   37.79248    0.     ], 
        #   position [-1.22397450e+02  3.77924806e+01 -8.20000000e-02], local position [ 0.07728764 -0.01342452  0.08251671]
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        #V If you run it again after reaching destinations then self.global_home = self.global_position
        #global home [-122.3974533   37.7924804    0.       ], 
        #   position [-122.3973445   37.7925691    0.26     ], local position [ 9.85105324  9.66836548 -0.26058951]


        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        grid_start = (-north_offset, -east_offset)
        #V the reason this works is because your local NED position is (0,0,0) but your grid starts from 0.
        #V So to convert from NED to Grid you just need to offset by north_min and east_min (which are negative).
        print("GRID_START", grid_start)
        #V Less03 - 04 Uses medial axis transfrom (still a grid) which is much safer than 03
        # TODO: DONE convert start position to current position rather than map center

        # Set goal as some arbitrary position on the grid
        #Vgrid_goal = (-north_offset + 10, -east_offset + 10)
        # TODO: DONE adapt to set goal as latitude / longitude position and convert

        
        goal_llh = (-122.39733567,   37.79256955, 0)
        #goal_llh = self.local_to_global((10,10,0), self.global_home)
        print("goal_llh: ", goal_llh)
        goal_in_local = self.global_to_local(goal_llh, self.global_home)
        grid_goal = (int(goal_in_local[0] - north_offset), int(goal_in_local[1] - east_offset))
        #V Instead of using grid we can use medial axis  - lets call it skeleton
        skeleton = medial_axis(invert(grid))
        skel_start, skel_goal = self.find_start_goal(skeleton, 
                        (self.local_position[0], self.local_position[1]), #current position in local (NED)
                        (goal_in_local[0], goal_in_local[1]))
        print('Skel Start and Goal: ', skel_start, skel_goal)

        # Run A* to find a path from start to goal
        # TODO: DONE add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        #V Get an error here because skel_start, skel_goal are the same - somethings is wrong
        #path, _ = a_star(invert(skeleton).astype(np.int), heuristic, tuple(skel_start), tuple(skel_goal))
        # TODO: DONE prune path to minimize number of waypoints
        path = self.prune_path(path)
        print(path)
        # TODO (if you're feeling ambitious): Try a different approach altogether!

        # Convert path to waypoints
        #V: Note we add the offsets (north_min and east_min which are negative) to our path points
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
