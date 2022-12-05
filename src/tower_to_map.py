#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
from environment_controller.srv import *
import tf2_ros

from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point


# Create a class which we will use to take keyboard commands and convert them to a position
class TowerToMap():
  # On node initialization
  def __init__(self):
    # print("initializing ")
    time.sleep(10)

    # Create the publisher and subscriber
    self.tower_finding_listener = rospy.Subscriber("/cell_tower/position", Point, self.get_target)

    # print("made the buffers")
    # TODO: Instantiate the Buffer and TransformListener
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)

    self.target = None
    self.final_dog_position = None

  def get_target(self, msg):
    self.target = msg
    pass
  
  def get_dog_position(self):
    if self.final_dog_position is not None:
      return self.final_dog_position
    
    while not rospy.is_shutdown() and self.final_dog_position == None:
      try: 
        transform = self.tfBuffer.lookup_transform('cell_tower', 'world', rospy.Time())

        temp_point = PointStamped()
        temp_point.point.x = self.target.x
        temp_point.point.y = self.target.y
        temp_point.point.z = self.target.z

        new_point = do_transform_point(temp_point, transform)

        final_point = Point()
        final_point.x = new_point.point.x
        final_point.y = new_point.point.y
        final_point.z = new_point.point.z
        
        # print()
        # print("The dog target position is: ", final_point)
        # print()

        self.final_dog_position = final_point
        return self.final_dog_position

      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print('tf2 exception, continuing')

  def mainloop(self):
    print("shouldn't be here")
    pass

if __name__ == '__main__':
  rospy.init_node('tower_to_map_node')
  try:
    pp = DoorOpener()
  except rospy.ROSInterruptException:
    pass