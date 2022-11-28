#!/usr/bin/env python
import rospy
import time
import numpy as np

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import math

class LidarReader():
    def __init__(self):
        # Set the rate
        self.rate = 5.0
        self.dt = 1.0 / self.rate

        # Environment
        self.position = np.zeros(3, dtype=np.float64)
        self.quaternion = np.zeros(4)
        self.yaw = 0
        self.lidar = []

        # Subscribers
        self.gps_sub = rospy.Subscriber("uav/sensors/gps", PoseStamped, self.get_gps)
        self.laser_sub = rospy.Subscriber("/uav/sensors/lidar", LaserScan, self.get_laser)
        # Publishers
        self.map_pub = rospy.Pubscriber("/map", OccupancyGrid)

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

        # This is the main loop of this class
    def UpdateLoop(self):
        # Set the rate
        rate = rospy.Rate(self.rate)

        # While running
        while not rospy.is_shutdown():

            # Display the position
            self.gui_object.world["path"] = self.path
            self.gui_object.world["obstacles"] = self.obstacle_list
            self.gui_object.world["doors"] = self.door_list
            self.gui_object.quads['quad1']['position'] = list(self.position)
            self.gui_object.quads['quad1']['orientation'] = list(euler_from_quaternion(self.quaternion))
            self.gui_object.world['lidar'] = self.lidar
            self.gui_object.crashed = self.crashed
            if self.goal is not None:
                self.gui_object.world['goal'] = self.goal
            else:
                if 'goal' in self.gui_object.world:
                    del self.gui_object.world['goal']

            if not self.gui_object.crashed:
                self.gui_object.update()

            # Sleep any excess time
            rate.sleep()


def main():
  rospy.init_node('update_map')
  try:
    angcon = LidarReader()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()