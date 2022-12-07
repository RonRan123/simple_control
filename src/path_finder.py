#!/usr/bin/env python
from Queue import PriorityQueue
import rospy
import time
import numpy as np
import itertools

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose, Vector3
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import math
from geometry_msgs.msg import Point


class PathFinder():
    def __init__(self):
        # Set the rate
        self.rate = 5.0
        self.dt = 1.0 / self.rate

        # Environment details
        self.obstacle_list = []
        self.door_list = []
        self.lidar = []
        self.map_size = (11, 11)
        self.path = np.array([[0, 0]])
        self.map = []

        # Create the position
        self.position = np.zeros(3, dtype=np.float64)
        self.quaternion = np.zeros(4)
        self.yaw = 0

        self.gps_sub = rospy.Subscriber("uav/sensors/gps", PoseStamped, self.get_gps)
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.get_map)

        self.frontier = PriorityQueue()
        self.came_from = {}
        self.cost_so_far = {}

        self.count = 0

        # Need to input from the arg file
        self.map_width = rospy.get_param('/update_map_node/map_width', 11)
        self.map_height = rospy.get_param('/update_map_node/map_height', 11)

        self.drone_pub = rospy.Publisher("/uav/input/position", Vector3, queue_size=1)


        self.UpdateLoop()

    # Call back to get the gps data
    def get_gps(self, msg):
        self.position[0] = msg.pose.position.x
        self.position[1] = msg.pose.position.y
        self.position[2] = msg.pose.position.z

        self.quaternion = (msg.pose.orientation.x,
                           msg.pose.orientation.y,
                           msg.pose.orientation.z,
                           msg.pose.orientation.w)
        _, _, self.yaw = euler_from_quaternion(self.quaternion)
    
    def get_map(self, msg):
        width = msg.info.width
        height = msg.info.height
        res = msg.info.resolution
    
        self.start_x = msg.info.origin.position.x
        self.start_y = msg.info.origin.position.y

        self.end_x = self.start_x + (width * res)
        self.end_y = self.start_y + (height * res)

        map_data = np.reshape(msg.data, (width, height))
        self.map_size = (width / 2.0, height / 2.0)
        self.obstacle_list[:] = []  # clear the list
        self.door_list[:] = []  # clear the list
        self.goal = None
        for xi in range(0, width):
            for yi in range(0, height):
                if map_data[xi, yi] == -1:
                    self.door_list.append((xi + self.start_x, yi + self.start_y, 'closed'))
                elif map_data[xi, yi] == -2:
                    self.door_list.append((xi + self.start_x, yi + self.start_y, 'open'))
                elif map_data[xi, yi] == -3:
                    self.goal = (xi + self.start_x, yi + self.start_y)
                else:
                    self.obstacle_list.append((xi + self.start_x, yi + self.start_y, map_data[xi, yi]))
        self.map = map_data
    
    def drone_to_grid(self, x_d, y_d):
        if self.map == []:
            return
        width, height = self.map.shape
        world_frame_x = width / 2.0
        world_frame_y = height / 2.0
        x_d, y_d = x_d + world_frame_x, y_d + world_frame_y
        return int(x_d), int(y_d)

    def get_neighbors(self):
        # Means empty map and hence no neighbors
        if self.map == []:
            return []
        neighbors = []
        # width, height = self.map.shape
        # Convert GPS position
        drone_x, drone_y = self.drone_to_grid(self.position[0], self.position[1])

        # for xi in range(drone_x-1, drone_x+2):
        #     for yi in range(drone_y-1, drone_y+2):
        #         if (xi, yi) == (drone_x, drone_y):
        #             continue
        #         if 0 <= xi < width and 0 <= yi < height and (self.map[xi][yi] == 0 or self.map[xi][yi] == -2):
        #             neighbors.append((xi, yi))
        # return neighbors
        if(self.map[drone_x + 1][drone_y] == 0 or self.map[drone_x + 1][drone_y] == -2 or self.map[drone_x + 1][drone_y] == -3):
            if self.map[drone_x + 1][drone_y] == -2:
                neighbors.append(([drone_x + 1, drone_y], True))
            else:
                neighbors.append(([drone_x + 1, drone_y], False))
            
        if(self.map[drone_x - 1][drone_y] == 0 or self.map[drone_x - 1][drone_y] == -2 or self.map[drone_x - 1][drone_y] == -3):
            if self.map[drone_x - 1][drone_y] == -2:
                neighbors.append(([drone_x - 1, drone_y], True))
            else:
                neighbors.append(([drone_x - 1, drone_y], False))
            
        if(self.map[drone_x][drone_y + 1] == 0 or self.map[drone_x][drone_y + 1] == -2 or self.map[drone_x][drone_y + 1] == -3):
            if self.map[drone_x][drone_y + 1] == -2:
                neighbors.append(([drone_x, drone_y + 1], True))
            else:
                neighbors.append(([drone_x, drone_y + 1], False))
            
        if(self.map[drone_x][drone_y - 1] == 0 or self.map[drone_x][drone_y - 1] == -2 or self.map[drone_x][drone_y - 1] == -3):
            if self.map[drone_x][drone_y - 1] == -2:
                neighbors.append(([drone_x, drone_y - 1], True))
            else:
                neighbors.append(([drone_x, drone_y - 1], False))

        return neighbors

    def grid_to_drone(self, x_g, y_g):
        world_frame_x = int(self.map_width / 2.0)
        world_frame_y = int(self.map_height / 2.0)
        return x_g - world_frame_x, y_g - world_frame_y

    def move_drone(self, x, y):
        vector = Vector3()
        vector.z = 3
        vector.x, vector.y = self.grid_to_drone(x, y)
        self.drone_pub.publish(vector)
    
    def plan(self, goal_position):


        final_path = []
        # while not frontier.empty():
        if self.get_neighbors() == []:
            return False
        current = self.frontier.get()
        current = list(current[1])

        print("Moving to current: ", current)
        self.move_drone(current[0], current[1])
        time.sleep(10)

        if current[0] == goal_position[0] and current[1] == goal_position[1]:
            return True

        for next in self.get_neighbors():
            next_pos = next[0]
            door_test = next[1]
            if door_test:
                edge_cost = 0
            else:
                edge_cost = 1
            goal_cost = math.sqrt(((goal_position[0] - next_pos[0])**2) + ((goal_position[1] - next_pos[1]) ** 2))
            new_cost = self.cost_so_far[str(current)] + edge_cost
            if str(next_pos) not in self.cost_so_far or next_pos < self.cost_so_far[str(next_pos)]:
                self.cost_so_far[str(next_pos)] = new_cost
                priority = new_cost + goal_cost
                self.frontier.put((priority, next_pos))
                self.came_from[str(next_pos)] = current

        print("Error: No Path Found")
        print("done with this part of the loop")
        # return [drone_position]

    def UpdateLoop(self):
        # Set the rate
        rate = rospy.Rate(self.rate)

        drone_start = (int(self.map_width/2.0),int(self.map_height/2.0))

        self.frontier.put((0, drone_start))
        self.came_from[str(list(drone_start))] = None
        self.cost_so_far[str(list(drone_start))] = 0


        # While running
        while not rospy.is_shutdown():

            if self.count % 15 == 0:
                print("trying to move to a new position")
                x = self.plan((9,9))
                if x == True:
                    self.move_drone(9,9)
                    break
                

                
            
            # self.map_pub.publish(self.map)
            self.count+= 1

            # Sleep any excess time
            rate.sleep()

def main():
  rospy.init_node('path_finder_node')
  try:
    angcon = PathFinder()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()