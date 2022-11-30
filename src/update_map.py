#!/usr/bin/env python
import rospy
import time
import numpy as np

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


    def get_laser(self, msg):
        self.lidar = []
        # print('Drone position', self.position[0], self.position[1])
        for index, range in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment  # - self.yaw
            position = (self.position[0] + range * math.cos(angle),
                        self.position[1] + range * math.sin(angle))
            # print('angle', angle)
            # print('range', range)
            # print('position', position)
            self.lidar.append(position)
        self.lidar = np.array(self.lidar)

    # def build_map(self):
    #     map = self.map

    #     width = self.map_width
    #     height = self.map_height
    #     res = self.map_resolution
        
    #     self.start_x = self.position[0]
    #     self.start_y = self.position[1]

    #     self.end_x = self.start_x + (width * res)
    #     self.end_y = self.start_y + (height * res)

    #     map_data = np.reshape(map.data, (width, height))

    #     for xi in range(0, width):
    #         for yi in range(0, height):
    #             map_data[xi][yi] = 50
    #     map.data = map_data.tolist()

    def UpdateLoop(self):
        # Set the rate
        rate = rospy.Rate(self.rate)

        # While running
        while not rospy.is_shutdown():
            # self.build_map()
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