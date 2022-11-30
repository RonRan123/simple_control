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

class UpdateMap():
    def __init__(self):
        # Set the rate
        self.rate = 5.0
        self.dt = 1.0 / self.rate

        # Aim towards center of block
        self.offset = 0.5
        # Want to strength our map over time
        self.adjust = 0.1
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
            # if msg.range_min <range < msg.range_max
            angle = msg.angle_min + index * msg.angle_increment  # - self.yaw
            # TODO: Decide on one of these for the moving drone
            position = (range * math.cos(angle), range * math.sin(angle))
            # position = (self.position[0] + range * math.cos(angle),
            #             self.position[1] + range * math.sin(angle))
            # print('angle', angle)
            # print('range', range)
            # print('position', position)
            self.lidar.append(position)
        # self.lidar = np.array(self.lidar)



    def build_map(self):
        sign = lambda x: math.copysign(1, x)
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
        for xl, yl in lidar_list:
            # Discount the "inf" values because means did not run into obstacle
            if not math.isinf(xl) and not math.isinf(yl):
                adjusted_x = xl + sign(xl) * self.offset
                adjusted_y = yl + sign(yl) * self.offset
                length = round(math.sqrt(adjusted_x**2 + adjusted_y**2))

                current_x = drone_start_x + adjusted_x
                current_y = drone_start_y + adjusted_y
                obstacle_x = int(round(current_x))
                obstacle_y = int(round(current_y))

                # More evidence it clear
                for xi in range(drone_start_x, obstacle_x, 1):
                    for yi in range(drone_start_y, obstacle_y, 1):
                        map_data[xi][yi] = max(0, map_data[xi][xi] - self.adjust * 100)

                
                # More evidence its an obstacle
                map_data[obstacle_x][obstacle_y] = min(100, map_data[obstacle_x][obstacle_y] + self.adjust * 100)
            
            # If the lidar returns infinity, it is clear al lthe way in that direction
            else:
                if xl == float("inf") and yl == float("inf"):
                    for xi in range(drone_start_x, drone_start_x + self.range + 1, 1):
                        for yi in range(drone_start_y, drone_start_y + self.range + 1, 1):
                            map_data[xi][yi] = max(0, map_data[xi][yi] - self.adjust * 100)
                elif xl == float("inf"):
                    for xi in range(drone_start_x, drone_start_x + self.range + 1, 1):
                        map_data[xi][drone_start_y] = max(0, map_data[xi][drone_start_y] - self.adjust * 100)
                else:
                    for yi in range(drone_start_y, drone_start_y + self.range + 1, 1):
                        map_data[drone_start_x][yi] = max(0, map_data[drone_start_x][yi] - self.adjust * 100)

        # map_data[int(offset_x + start_x)][int(offset_y + start_y)] = -2
        
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