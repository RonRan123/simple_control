#!/usr/bin/env python
import rospy
import time
import numpy as np
import itertools

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose, Vector3
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import math
from door_opener import DoorOpener
from tower_to_map import TowerToMap
from geometry_msgs.msg import Point
# from door_opener import Do


class UpdateMap():
    def __init__(self):
        # Set the rate
        self.rate = 5.0
        self.dt = 1.0 / self.rate
        self.threshold = 0.08

        # Aim towards center of block
        self.offset = 0.1
        # Want to strength our map over time
        self.adjust = 0.1

        self.count = 0
        self.position_counter = 0

        # Need to input from the arg file
        self.map_width = rospy.get_param('/update_map_node/map_width', 11)
        self.map_height = rospy.get_param('/update_map_node/map_height', 11)
        
        self.map_resolution = 1

        # Environment
        self.position = np.zeros(3, dtype=np.float64)
        self.quaternion = np.zeros(4)
        self.yaw = 0

        self.doors = {}
        
        self.map = OccupancyGrid()
        self.map.info.width = self.map_width
        self.map.info.height = self.map_height
        self.map.info.resolution = self.map_resolution
        self.map.data = [50] * self.map_width * self.map_height * self.map_resolution

        self.map_final = set()

        self.map.info.origin.position.x = -1 * self.map_width / 2.0
        self.map.info.origin.position.y = -1 * self.map_height / 2.0

        # Subscribers
        self.gps_sub = rospy.Subscriber("uav/sensors/gps", PoseStamped, self.get_gps)
        self.laser_sub = rospy.Subscriber("/uav/sensors/lidar", LaserScan, self.get_laser)
        # Publishers
        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1)
        self.drone_pub = rospy.Publisher("/uav/input/position", Vector3, queue_size=1)

        self.is_door_open = False
        self.lidar = []
        self.prev_lidar = []
        
        self.door_opener = None
        self.door_opener = DoorOpener()
        
        self.tower_to_map = TowerToMap()
        

        self.target = None
        self.dog = None
        # self.target = (21, 21)

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

    # Vectors stemming out from the drone
    def get_laser(self, msg):
        new_lidar_data = []
        # print('Drone position', self.position[0], self.position[1])
        for index, drone_range in enumerate(msg.ranges):
            if drone_range < msg.range_max:
                angle = msg.angle_min + index * msg.angle_increment  # - self.yaw
                # TODO: Decide on one of these for the moving drone
                position = (drone_range * math.cos(angle), drone_range * math.sin(angle), True, angle)
                # position = (self.position[0] + range * math.cos(angle),
                #             self.position[1] + range * math.sin(angle))
                # print('angle', angle)
                # print('range', range)
                # print('position', position)
                new_lidar_data.append(position)
            # No obstacle found in that direction
            else:
                length = msg.range_max
                angle = msg.angle_min + index * msg.angle_increment  # - self.yaw
                position = (length * math.cos(angle), length * math.sin(angle), False, angle)
                new_lidar_data.append(position)
        if self.count > 1:
            self.prev_lidar = self.lidar
        self.lidar = new_lidar_data

    def get_doors(self):
        ans = []
        if self.count > 2:
            for i in range(len(self.lidar)):
                # print(self.lidar[x])
                total_diff = abs(self.lidar[i][0] - self.prev_lidar[i][0]) + abs(self.lidar[i][1] - self.prev_lidar[i][1])
                if total_diff > self.threshold:
                    ans.append(self.lidar[i])
        possible_doors = {}
        for d in ans:
            lidar_c_x, lidar_c_y = self.lidar_to_drone(d[0], d[1], d[3])
            # The contionus values converted to OccupancyGrid
            door_x, door_y = self.drone_to_grid(lidar_c_x, lidar_c_y)
            if (door_x, door_y) not in possible_doors:
                possible_doors[(door_x, door_y)] = 1
            else:
                possible_doors[(door_x, door_y)] += 1
        for key, count in possible_doors.items():
            # might want to increase count to 3
            if count >= 2:
                if not key in self.doors:
                    self.doors[key] = -1  
        return ans
        # pass


        # self.lidar = np.array(self.lidar)
    def lidar_to_drone(self, x_l, y_l, angle= None):
        drone_start_x = self.position[0] 
        drone_start_y = self.position[1]
        # Old erroneous code, want to get rid of
        if not angle:
            adjusted_x = x_l + self.sign(x_l) * self.offset
            adjusted_y = y_l + self.sign(y_l) * self.offset
        else:
            adjusted_x = x_l + math.cos(angle) * self.offset
            adjusted_y = y_l + math.sin(angle) * self.offset

        return drone_start_x + adjusted_x, drone_start_y + adjusted_y

    def drone_to_grid(self, x_d, y_d):
        world_frame_x = self.map_width / 2.0
        world_frame_y = self.map_height / 2.0
        x_d, y_d = x_d + world_frame_x, y_d + world_frame_y
        return (int(x_d), int(y_d))
    
    def grid_to_drone(self, x_g, y_g):
        world_frame_x = int(self.map_width / 2.0)
        world_frame_y = int(self.map_height / 2.0)
        return x_g - world_frame_x, y_g - world_frame_y

    def sign(self, x):
        return int(math.copysign(1, x))

    # https://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm#Python
    def line(self, x0, y0, x1, y1):
        res = []
        "Bresenham's line algorithm"
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                res.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                res.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy        
        res.append((x, y))
        return res

    def build_map(self):
        # Protected values that represent closed door, open door, and target
        special = {-1, -2, -3}

        world_frame_x_int = int(self.map_width / 2)
        world_frame_y_int = int(self.map_height / 2)

        map = self.map

        width = self.map_width
        height = self.map_height
        res = self.map_resolution
    
        drone_start_x, drone_start_y = self.drone_to_grid(self.position[0], self.position[1])

        end_x = drone_start_x + (width * res)
        end_y = drone_start_y + (height * res)

        map_data = np.reshape(map.data, (width, height))
        if not self.dog:
            self.dog = self.tower_to_map.get_dog_position()
        if not self.target and self.dog:
            xt, yt = int(round(self.dog.x + (width / 2.0))), int(round(self.dog.y + (height/2.0)))
            self.target = (xt, yt)
            # self.target = self.drone_to_grid(dog.x, dog.y)
            # xt, yt = self.target
            map_data[xt][yt] = -3
            # Python is broken
            # print(dog, xt, yt, width/2.0, height/2.0, dog.x + width/2.0, dog.y + height/2.0)
            
        # print('target', self.target)

        lidar_list = self.lidar
        # Go over the recent LIDAR data to update map
        for xl, yl, isObstacleThere, angle in lidar_list:
            # The continuous lidar values, with the offset
            lidar_c_x, lidar_c_y = self.lidar_to_drone(xl, yl, angle)
            # The contionus values converted to OccupancyGrid
            lidar_x, lidar_y = self.drone_to_grid(lidar_c_x, lidar_c_y)

            # More evidence that a particular position is clear
            points = self.line(drone_start_x, drone_start_y, lidar_x, lidar_y )
            if points[-1] == (8,7):
                print('Breseham', points, drone_start_x, drone_start_y, lidar_x, lidar_y, isObstacleThere)
            for xi, yi in points[:-1]:
                if (xi, yi) not in self.map_final and map_data[xi][yi] not in special:
                    map_data[xi][yi] = max(0, map_data[xi][yi] - self.adjust * 100)
            

            # The last point will either be an obstacle or free
            if len(points) > 1 and map_data[lidar_x][lidar_y] not in special and (lidar_x, lidar_y) not in self.map_final:
                if isObstacleThere:
                    map_data[lidar_x][lidar_y] = min(100, map_data[lidar_x][lidar_y] + self.adjust * 100)
                else:
                    map_data[lidar_x][lidar_y] = max(0, map_data[lidar_x][lidar_y] - self.adjust * 100)
        
        # Put the doors on the map
        for key, state in self.doors.items():
            xd, yd = key
            map_data[xd][yd] = state
        
        for xi in range(-1, +2):
            for yi in range(-1, 2):
                adjacent_x, adjacent_y = drone_start_x + xi, drone_start_y + yi
                magnitude = abs(xi) + abs(yi)
                if not 0 <= adjacent_x < width and 0 <= adjacent_y < height:
                    continue
                # If cells to my left right top and down are 0 or 100, put as finalized
                if magnitude == 1 and (adjacent_x, adjacent_y) not in self.map_final:
                    if (map_data[adjacent_x][adjacent_y] == 0 or map_data[adjacent_x][adjacent_y] == 100) and (adjacent_x, adjacent_y) not in self.map_final:
                        self.map_final.add((adjacent_x, adjacent_y))
                        # print('map_final', (adjacent_x, adjacent_y))
                # I'm next to unopened door, open it
                if map_data[adjacent_x][adjacent_y] == -1:
                    door_point = Point()
                    # Need to convert back to drone perspective
                    door_point.x = adjacent_x-world_frame_x_int
                    door_point.y = adjacent_y-world_frame_y_int
                    door_point.z = 3
                    # print(door_point)
                    if(self.door_opener.use_key_client(door_point)):
                        self.doors[(adjacent_x, adjacent_y)] = -2
        
        # Seems inefficient, should look into an alternative
        map_data = map_data.tolist()
        data = list(itertools.chain.from_iterable(map_data))
        map.data = data

    def move_drone(self, x, y):
        vector = Vector3()
        vector.z = 3
        vector.x, vector.y = self.grid_to_drone(x, y)
        self.drone_pub.publish(vector)

    def UpdateLoop(self):
        # Set the rate
        rate = rospy.Rate(self.rate)

        # While running
        while not rospy.is_shutdown():
            self.build_map()
            self.map_pub.publish(self.map)
            self.get_doors()
            self.count+= 1
            # Sleep any excess time
            rate.sleep()
            time.sleep(1)
            # print()


def main():
  rospy.init_node('update_map_node')
  try:
    angcon = UpdateMap()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()