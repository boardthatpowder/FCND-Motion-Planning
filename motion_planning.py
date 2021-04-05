import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import random
import networkx as nx
import matplotlib.pyplot as plt


from planning_utils import a_star, heuristic, create_grid_and_edges, closest_point, prune_path
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local, local_to_global


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
            print('local_position: {}, target_position: {}'.format(self.local_position, self.target_position))
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
        print('state_callback: flight_state: {}'.format(self.flight_state))
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

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 3
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # DONE: read lat0, lon0 from colliders into floating point values
        with open('colliders.csv') as f:
            first_line = f.readline().split(',')
        lat0 = float(first_line[0].strip().split(' ')[1])
        lon0 = float(first_line[1].strip().split(' ')[1])
        alt0 = 0.
        print('start pos: ({}, {}, {})'.format(lon0, lat0, alt0))
        
        # DONE: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, alt0)

        # DONE: retrieve current global position
        print('global_position: {}'.format(self.global_position))

        # DONE: convert to current local position using global_to_local()
        n_loc, e_loc, d_loc = global_to_local(self.global_position, self.global_home)
        print('n_loc: {}, e_loc: {}, d_loc: {}'.format(n_loc, e_loc, d_loc))

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, edges, north_offset, east_offset = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print('grid = {}'.format(grid))
        # print("edges = {}".format(edges))
        print('north_offset: {}, east_offset: {}'.format(north_offset, east_offset))

        # create the graph with the weight of the edges set to the Euclidean distance between the points
        G = nx.Graph()
        for e in edges:
            p1 = e[0]
            p2 = e[1]
            dist = np.linalg.norm(np.array(p2) - np.array(p1))
            G.add_edge(p1, p2, weight=dist)

        # Define starting point on the grid (this is just grid center)
        # DONE: convert start position to current position rather than map center
        #grid_start = [local_pos[0], local_pos[1]]
        grid_start = (int(np.ceil(n_loc - north_offset)), int(np.ceil(e_loc - east_offset)))
        graph_start = closest_point(G, grid_start)
        print('grid_start: {}, graph_start: {}'.format(grid_start, graph_start))
        
        # Set goal as some arbitrary position on the grid
        # DONE: adapt to set goal as latitude / longitude position and convert
        super_duper_burgers = (-122.39400878361174, 37.792827005959616, 0.)
        grid_goal_global = super_duper_burgers 
        print('grid_goal_global: {}'.format(grid_goal_global))
        grid_goal_local = global_to_local(grid_goal_global , self.global_home)
        print('grid_goal_local: {}'.format(grid_goal_local))
        graph_goal = closest_point(G, (grid_goal_local[0], grid_goal_local[1]))
        print('graph_goal: {}'.format(graph_goal))

        # Run A* to find a path from start to goal
        # DONE: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        path, cost = a_star(G, heuristic, graph_start, graph_goal)
        self.plot(grid, edges, path, grid_start, graph_start, grid_goal_local, graph_goal, 'full_path.png')

        # DONE: prune path to minimize number of waypoints
        pruned_path = prune_path(path)
        self.plot(grid, edges, pruned_path, grid_start, graph_start, grid_goal_local, graph_goal, 'pruned_path.png')

        # TODO (if you're feeling ambitious): Try a different approach altogether!

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]
        print('waypoints: {}'.format(waypoints))
        
        # Set self.waypoints
        self.waypoints = waypoints
        # DONE: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()

    def plot(self, grid, edges, path, grid_start, graph_start, grid_goal, graph_goal, filename):
        plt.imshow(grid, origin='lower', cmap='Greys') 

        for e in edges:
            p1 = e[0]
            p2 = e[1]
            plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')

        plt.plot([grid_start[1], grid_start[1]], [grid_start[0], grid_start[0]], 'r-')
        if path is not None:
            pp = np.array(path)
            plt.plot(pp[:, 1], pp[:, 0], 'g')
            plt.scatter(pp[:, 1], pp[:, 0])
            
        # plt.plot([grid_start[1], grid_start[1]], [grid_start[0], grid_start[0]], 'r-')
        # for i in range(len(path)-1):
        #     p1 = path[i]
        #     p2 = path[i+1]
        #     plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'r-')
        # plt.plot([grid_goal[1], graph_goal[1]], [grid_goal[0], graph_goal[0]], 'r-')
            
        plt.plot(grid_start[1], grid_start[0], 'gx')
        plt.plot(grid_goal[1], grid_goal[0], 'gx')

        plt.xlabel('EAST', fontsize=20)
        plt.ylabel('NORTH', fontsize=20)
        plt.savefig(filename)
        plt.clf()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=120)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
