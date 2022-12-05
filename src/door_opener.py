#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
from environment_controller.srv import *
import tf2_ros

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point


# Create a class which we will use to take keyboard commands and convert them to a position
class DoorOpener():
  # On node initialization
  def __init__(self):
    time.sleep(10)

    # Create the publisher and subscriber
    self.keys_left_sub = rospy.Subscriber("/keys_remaining", Int32, self.get_keys)
    self.tower_finding_listener = rospy.Subscriber("/cell_tower/position", Point, self.get_target)

    print("made the buffers")
    # TODO: Instantiate the Buffer and TransformListener
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)

    self.target = None

    
    self.mainloop()

  def get_keys(self, msg):
    # print(msg)
    pass

  def get_target(self, msg):
    self.target = msg
    pass

  def use_key_client(self, loc):
    rospy.wait_for_service('use_key')
    try:
        # print("trying the service proxy")
        use_key_response = rospy.ServiceProxy('use_key', use_key)
        resp = use_key_response(loc)
        # print("the respose is: ")
        # print(resp)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



  def mainloop(self):
    while not rospy.is_shutdown():
      time.sleep(2)

      try: 
        # print("in the try")
        #TODO: Lookup the tower to world transform
        # print("trying to connect to the buffer")
        transform = self.tfBuffer.lookup_transform('cell_tower', 'world', rospy.Time())

        # rospy.loginfo("got the transform")

        #TODO: Convert the goal to a PointStamped
        temp_point = PointStamped()
        temp_point.point.x = self.target.x
        temp_point.point.y = self.target.y
        temp_point.point.z = self.target.z
        # print("got the point to be ", temp_point)

        #TODO: Use the do_transform_point function to convert the point using the transform
        new_point = do_transform_point(temp_point, transform)

        # print("the new point is: ", new_point)

        #TODO: Convert the point back into a vector message containing integers
        final_point = Point()
        final_point.x = new_point.point.x
        final_point.y = new_point.point.y
        final_point.z = new_point.point.z
        
        print()
        print("The dog target position is: ", final_point)
        print()


        #TODO: Publish the vector
        # self.goal_pub.publish(final_point)

        # rospy.loginfo(str(rospy.get_name()) + ": Publishing Transformed Goal {}".format([final_point.x, final_point.y]))

        # The tower will automatically send you a new goal once the drone reaches the requested position.
        #TODO: Reset the goal
        # self.goal = None
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print('tf2 exception, continuing')
      # continue

    # # Create the publisher and subscriber
    # time.sleep(5)
    # print("in the main method")
    # hard_coded_point = Point()
    # hard_coded_point.x = 1
    # hard_coded_point.y = 1
    # hard_coded_point.z = 3
    # print("the hardcoded point is: ")
    # print(hard_coded_point)
    # print("trying door location")



if __name__ == '__main__':
  rospy.init_node('door_opener_node')
  try:
    pp = DoorOpener()
  except rospy.ROSInterruptException:
    pass