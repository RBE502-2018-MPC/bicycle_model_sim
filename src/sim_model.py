#!/usr/bin/env python

''' Run-time simulator of the RC car for testing MPC without
    having to have access to the car, or the motion capture setup.
    Uses the bicycle kinematic model

    Author: Jordan Burklund
'''

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped

def bicycle_sim():
  # Setup the node
  rospy.init_node('bicycle_sim')

  # Rate to run physics updates
  rate = rospy.Rate(300)

  # Publishers for simulated pose and velocity
  pub_pose = rospy.Publisher('/vrpn_client_node/rc_car/pose', PoseStamped, queue_size=10)
  pub_twist  = rospy.Publisher('/vrpn_client_node/rc_car/twist', TwistStamped, queue_size=10)

  # Setup the basic info
  my_pose = PoseStamped()
  my_pose.header.frame_id = "world"
  my_twist = TwistStamped()
  my_twist.header.frame_id = "world"


  seq_id = 0
  while not rospy.is_shutdown():
    # Update the headers
    time = rospy.get_rostime()
    my_pose.header.seq = seq_id
    my_pose.header.stamp = time
    my_twist.header.seq = seq_id
    my_twist.header.stamp = time

    # Update the state in the relevant messages
    my_pose.pose.position.x = 0
    my_pose.pose.position.y = my_pose.pose.position.y + 0.01
    my_pose.pose.position.z = 0

    # Publish the data
    pub_pose.publish(my_pose)
    pub_twist.publish(my_twist)

    seq_id = seq_id+1
    rate.sleep()

if __name__ == '__main__':
  try:
    bicycle_sim()
  except rospy.ROSInterruptException:
    pass
