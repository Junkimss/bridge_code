#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool,SetBoolResponse
import random


class Test_node:
  def __init__(self):
    # 노드이름 test_node1
    rospy.init_node("test_node1")
    # publisher 이름 API/cmd_vel 등등 /.
    self.pub = rospy.Publisher("API/cmd_vel",Twist,queue_size=10)
    self.sub = rospy.Subscriber("feedback_vel",Twist,self.sub_msg)
    self.service_client = rospy.ServiceProxy("API/follow_me_mode",SetBool)
    # 1hz
    self.timer = rospy.Timer(rospy.Duration(0.1), self.pub_msg)


  def pub_msg(self,event):

    twist_msg = Twist()
    twist_msg.linear.x = random.uniform(-10,10)
    twist_msg.linear.x = round(twist_msg.linear.x,1)
    twist_msg.linear.y = random.uniform(-10,10)
    twist_msg.linear.y = round(twist_msg.linear.y,1)
    twist_msg.linear.z = random.uniform(-10,10)
    twist_msg.linear.z = round(twist_msg.linear.z,1)

    twist_msg.angular.x = random.uniform(0,360)
    twist_msg.angular.x = round(twist_msg.angular.x,1)
    twist_msg.angular.y = random.uniform(0,360)
    twist_msg.angular.y = round(twist_msg.angular.y,1)
    twist_msg.angular.z = random.uniform(0,360)
    twist_msg.angular.z = round(twist_msg.angular.z,1)

    self.pub.publish(twist_msg)
    rospy.loginfo('ROS1 API/cmd_vel: linear.x={0}, linear.y={1}, linear.z={2}, angular.z={3},angular.z={4}, angular.z={5}'.format(twist_msg.linear.x,twist_msg.linear.y,twist_msg.linear.z,twist_msg.angular.x,twist_msg.angular.y,twist_msg.angular.z))

  def sub_msg(self,msg):
    rospy.loginfo('Received Twist: linear.x={0}, linear.y={1}, linear.z={2}, angular.z={3},angular.z={4}, angular.z={5}'.format(msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z))


def main(args=None):
  node = Test_node()
  try :
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo('Keyboard Interrupt (SIGINT)')

if __name__ == '__main__':
    main()
