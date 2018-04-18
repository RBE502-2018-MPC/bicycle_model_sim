#!/usr/bin/env python

''' Run-time simulator of the RC car for testing MPC without
    having to have access to the car, or the motion capture setup.
    Uses the bicycle kinematic model

    Author: Jordan Burklund
'''

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion
from rc_node.msg import car_input
from tf.transformations import quaternion_from_euler
import tf
from math import tan, atan, cos, sin, pi
import numpy as np

my_car_input = car_input()

class BicycleModel():

  def __init__(self):
    ''' State vector: |    x   |  Inputs: |   V    | motor voltage
                      |    y   |          | deltaf | steering angle
                      |   psi  |
                      |    v   |
        From: http://www.me.berkeley.edu/~frborrel/pdfpub/IV_KinematicMPC_jason.pdf '''
    self.state = np.zeros(4)
    self.statedot = np.zeros(4)
    self.lr = 1.0
    self.lf = 1.0

  def update(self, V, deltaf, dT):
    v = self.state[3];
    psi = self.state[2];
    beta = atan(self.lr/(self.lr+self.lf)*tan(deltaf*pi/180.0))
    self.statedot = np.array([v*cos(psi+beta),
                              v*sin(psi+beta),
                              v/self.lr*sin(beta),
                              0])
    self.state = np.add(self.state, self.statedot*dT)
    self.state[3] = V  # TODO just assume voltage and velocity are proportional for now (no dynamics)


def input_callback(data):
  my_car_input.power = data.power
  my_car_input.steer_angle = data.steer_angle

def bicycle_sim():
  # Setup the node
  rospy.init_node('bicycle_sim')

  # Rate to run physics updates
  rate = rospy.Rate(300)

  # Publishers for simulated pose and velocity
  pub_pose = rospy.Publisher('/vrpn_client_node/rc_car/pose', PoseStamped, queue_size=10)
  pub_twist  = rospy.Publisher('/vrpn_client_node/rc_car/twist', TwistStamped, queue_size=10)

  # Subscriber for car inputs
  rospy.Subscriber('/car_input', car_input, input_callback)

  # Setup the basic info
  my_pose = PoseStamped()
  my_pose.header.frame_id = "world"
  my_twist = TwistStamped()
  my_twist.header.frame_id = "world"
  pose_broadcaster = tf.TransformBroadcaster()

  # Initialize the model
  model = BicycleModel()

  seq_id = 0
  while not rospy.is_shutdown():
    # Update the headers
    time = rospy.get_rostime()
    my_pose.header.seq = seq_id
    my_pose.header.stamp = time
    my_twist.header.seq = seq_id
    my_twist.header.stamp = time

    # Update the model state
    model.update(my_car_input.power, my_car_input.steer_angle, rate.sleep_dur.to_sec())

    # Update the state in the relevant messages
    my_pose.pose.position.x = model.state[0]
    my_pose.pose.position.y = model.state[1]
    my_pose.pose.orientation = Quaternion(*quaternion_from_euler(0,0,model.state[2]))
    my_twist.twist.linear.x = model.statedot[0]
    my_twist.twist.linear.y = model.statedot[1]
    my_twist.twist.angular.z = model.statedot[3]
    pose_broadcaster.sendTransform(
      (model.state[0], model.state[1], 0),
      quaternion_from_euler(0, 0, model.state[2]),
      time,
      "rc_car",
      "world"
    )

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
