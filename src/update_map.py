#!/usr/bin/env python
import rospy
import time
import numpy as np
import itertools

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import math
from door_opener import DoorOpener
from geometry_msgs.msg import Point


class UpdateMap():
    def __init__(self):
        # Set the rate
        self.rate = 5.0
        self.dt = 1.0 / self.rate

        # Aim towards center of block
        self.offset = 0.3
        # Want to strength our map over time
        self.adjust = 0.6
        self.range = 5

        self.count = 0

        # Need to input from the arg file
        self.map_width = 11
        self.map_height = 11
        
        self.map_resolution = 1

        # Environment
        self.position = np.zeros(3, dtype=np.float64)
        self.quaternion = np.zeros(4)
        self.yaw = 0
        self.lidar = []
        
        self.map = OccupancyGrid()
        self.map.info.width = self.map_width
        self.map.info.height = self.map_height
        self.map.info.resolution = self.map_resolution
        self.map.data = [50] * self.map_width * self.map_height * self.map_resolution

        self.map.info.origin.position.x = -1 * self.map_width / 2.0
        self.map.info.origin.position.y = -1 * self.map_height / 2.0

        # Subscribers
        self.gps_sub = rospy.Subscriber("uav/sensors/gps", PoseStamped, self.get_gps)
        self.laser_sub = rospy.Subscriber("/uav/sensors/lidar", LaserScan, self.get_laser)
        # Publishers
        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1)

        self.is_door_open = False

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
        self.lidar = []
        # print('Drone position', self.position[0], self.position[1])
        for index, range in enumerate(msg.ranges):
            if range < msg.range_max:
                angle = msg.angle_min + index * msg.angle_increment  # - self.yaw
                # TODO: Decide on one of these for the moving drone
                position = (range * math.cos(angle), range * math.sin(angle), True)
                # position = (self.position[0] + range * math.cos(angle),
                #             self.position[1] + range * math.sin(angle))
                # print('angle', angle)
                # print('range', range)
                # print('position', position)
                self.lidar.append(position)
            # No obstacle found in that direction
            else:
                length = msg.range_max
                angle = msg.angle_min + index * msg.angle_increment  # - self.yaw
                position = (length * math.cos(angle), length * math.sin(angle), False)
                self.lidar.append(position)

        # self.lidar = np.array(self.lidar)
    def lidar_to_drone(self, x_l, y_l):
        world_frame_x = (self.map_width-1) / 2
        world_frame_y = (self.map_height-1) / 2
        drone_start_x = self.position[0] + world_frame_x
        drone_start_y = self.position[1] + world_frame_y

        adjusted_x = x_l + self.sign(x_l) * self.offset
        adjusted_y = y_l + self.sign(y_l) * self.offset
        return drone_start_x + adjusted_x, drone_start_y + adjusted_y

    def drone_to_grid(self, x_d, y_d):
        return int(round(x_d)), int(round(y_d))

    def sign(self, x):
        return int(math.copysign(1, x))

    def build_map(self):
        world_frame_x = self.map_width / 2.0
        world_frame_y = self.map_height / 2.0

        map = self.map

        width = self.map_width
        height = self.map_height
        res = self.map_resolution
        
    
        drone_start_x = int(self.position[0] + world_frame_x)
        drone_start_y = int(self.position[1] + world_frame_y)

        end_x = drone_start_x + (width * res)
        end_y = drone_start_y + (height * res)

        map_data = np.reshape(map.data, (width, height))

        # Go over the recent LIDAR data to update map
        lidar_list = self.lidar
        # if self.count <= 5:
        #     print(lidar_list)
        # self.count += 1

        for xl, yl, isObstacleThere in lidar_list:
            # The continuous lidar values, with the offset
            lidar_c_x, lidar_c_y = self.lidar_to_drone(xl, yl)
            # The contionus values converted to OccupancyGrid
            lidar_x, lidar_y = self.drone_to_grid(lidar_c_x, lidar_c_y)

            # More evidence it clear
            if self.count <=5:
                print('vals', xl, yl, drone_start_x, lidar_x, drone_start_y, lidar_y, isObstacleThere)
            # Need to fix this, will always produce a squre/rectangle when I want a single "line" of boxes to adjust
            for xi in range(drone_start_x, lidar_x, self.sign(lidar_x-drone_start_x)):
                for yi in range(drone_start_y, lidar_y, self.sign(lidar_y-drone_start_y)):
                    map_data[xi][yi] = max(0, map_data[xi][yi] - self.adjust * 100)

            # More evidence its an obstacle
            if isObstacleThere:
                map_data[lidar_x][lidar_y] = min(100, map_data[lidar_x][lidar_y] + self.adjust * 100)
        
        # Hardcoded door
        if(not self.is_door_open):
            map_data[int(round(world_frame_x))][int(round(-0.5+ world_frame_y))] = -1 
        else:
            map_data[int(round(world_frame_x))][int(round(-0.5+ world_frame_y))] = -2
        time.sleep(1)
        if(not self.is_door_open and self.count >= 2):
            door_opener_test = DoorOpener()
            hard_coded_point = Point()
            hard_coded_point.x = 1
            hard_coded_point.y = 0
            hard_coded_point.z = 3
            print()
            print()
            if(door_opener_test.use_key_client(hard_coded_point)):
                map_data[int(round(world_frame_x))][int(round(-0.5+ world_frame_y))] = -2
                print("Able to open the door")

            # print("did it open a door: ", door_opener_test.use_key_client(hard_coded_point))
            self.is_door_open = True
            # print("should have opened the door")
            print()


        # Seems inefficient, should look into an alternative
        map_data = map_data.tolist()
        data = list(itertools.chain.from_iterable(map_data))
        map.data = data

    def UpdateLoop(self):
        # Set the rate
        rate = rospy.Rate(self.rate)

        # While running
        while not rospy.is_shutdown():
            self.build_map()
            self.map_pub.publish(self.map)
            self.count+= 1

            # Sleep any excess time
            rate.sleep()


def main():
  rospy.init_node('Update_Map_Node')
  try:
    angcon = UpdateMap()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()