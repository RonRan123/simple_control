#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
from environment_controller.srv import *



# Create a class which we will use to take keyboard commands and convert them to a position
class DoorOpener():
  # On node initialization
  def __init__(self):
    # Create the publisher and subscriber
    self.keys_left_sub = rospy.Subscriber("/keys_remaining", Int32, self.get_keys)
    self.mainloop()

  def get_keys(self, msg):
    # print(msg)
    pass

  def use_key_client(self, loc):
    rospy.wait_for_service('use_key')
    try:
        print("trying the service proxy")
        use_key_response = rospy.ServiceProxy('use_key', use_key)
        resp = use_key_response(loc)
        print("the respose is: ")
        print(resp)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



  def mainloop(self):
    # # Create the publisher and subscriber
    # pass
    time.sleep(5)
    print("in the main method")
    hard_coded_point = Point()
    hard_coded_point.x = 1
    hard_coded_point.y = 1
    hard_coded_point.z = 3
    print("the hardcoded point is: ")
    print(hard_coded_point)
    print("trying door location")
    print(self.use_key_client(hard_coded_point))






if __name__ == '__main__':
  rospy.init_node('door_opener_node')
  try:
    pp = DoorOpener()
  except rospy.ROSInterruptException:
    pass