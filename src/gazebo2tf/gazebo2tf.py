#!/usr/bin/env python

import rospy
import tf
from gazebo_msgs.msg import LinkStates

tfBroadcaster = None

def on_link_state_msg(linkStatesMsg):
  for (index, name) in enumerate(linkStatesMsg.name):
    pose = linkStatesMsg.pose[index]
    print('%d: %s: pose:\n%s\n---' % (index, name, pose))
    #tfBroadcaster.sendTransform(pose, name, 'gazebo_world')


def main():
  rospy.init_node('gazebo2tf')

  linkStateSub = rospy.Subscriber('gazebo/link_states', LinkStates, on_link_state_msg)

  global tfBroadcaster
  tfBroadcaster = tf.TransformBroadcaster()

  rospy.loginfo('Spinning')
  rospy.spin()

if __name__ == '__main__':
  main()
