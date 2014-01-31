#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import LinkStates

from gazebo2tf.model_names import *

tfBroadcaster = None
submodelsToBeIgnored = []

def point2Tuple(point):
  return (point.x, point.y, point.z)

def quaternion2Tuple(quaternion):
  return (quaternion.x, quaternion.y, quaternion.z, quaternion.w)

def on_link_state_msg(linkStatesMsg):
  """
  Publish tf for each model according to the following criteria:
  - If the model foo is not a composite (contains a single link), publish gazebo_world->foo
  - If the model bar is a composite (bar::{pa,pb::pba}), publish gazebo_world->bar, bar->pa, bar->pb, pb->pba

  All models must follow the rules that
  - All links must be named 'link' or have the suffix '_link'
  - Each (sub)model foo contains a link with exactly one of the names: link, foo_link, base, *_base, world_link, *_world_link
  """
  for (index, name) in enumerate(linkStatesMsg.name):
    pose = linkStatesMsg.pose[index]
    print('%d: name=%s: pose=\n%s' % (index, name, pose))
    (parentName, modelName, linkName) = splitName(name)
    print('parentName=%s modelName=%s linkName=%s' % (parentName, modelName, linkName))
    if isBaseLinkName(linkName, modelName):
      tfFromName=parentName
      tfToName=modelName
    else:
      tfFromName=modelName
      tfToName=linkName.replace('_link', '')
    print('tfFromName=%s tfToName=%s' % (tfFromName, tfToName))

    if tfFromName in submodelsToBeIgnored:
      print('Ignored submodel')
      continue;

    if tfFromName == worldLinkName:
      relativePose = pose
    else:
      # TODO
      relativePose = Pose()
      relativePose.orientation.w = 1
    tfBroadcaster.sendTransform(point2Tuple(relativePose.position), quaternion2Tuple(relativePose.orientation), rospy.get_rostime(), tfToName, tfFromName)
    print('---')


def main():
  rospy.init_node('gazebo2tf')

  global submodelsToBeIgnored
  submodelsToBeIgnored = rospy.get_param('~ignore_submodels_of', '').split(';')
  rospy.loginfo('Ignoring submodels of: ' + str(submodelsToBeIgnored))

  linkStateSub = rospy.Subscriber('gazebo/link_states', LinkStates, on_link_state_msg)

  global tfBroadcaster
  tfBroadcaster = tf.TransformBroadcaster()

  rospy.loginfo('Spinning')
  rospy.spin()

if __name__ == '__main__':
  main()
