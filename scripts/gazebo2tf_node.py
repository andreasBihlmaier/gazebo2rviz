#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import LinkStates
import tf_conversions.posemath as pm

from gazebo2rviz.model_names import *

tfBroadcaster = None
submodelsToBeIgnored = []
lastUpdateTime = None
updatePeriod = 0.02
maxResolveTrials = 2

def point2Tuple(point):
  return (point.x, point.y, point.z)

def quaternion2Tuple(quaternion):
  return (quaternion.x, quaternion.y, quaternion.z, quaternion.w)

def on_link_states_msg(linkStatesMsg):
  """
  Publish tf for each model according to the following criteria:
  - If the model foo is not a composite (contains a single link), publish gazebo_world->foo
  - If the model bar is a composite (bar::{pa,pb::pba}), publish gazebo_world->bar, bar->pa, bar->pb, pb->pba

  All models must follow the rules that
  - All links must be named 'link' or have the suffix '_link'
  - Each (sub)model FOO contains a link with exactly one of the names: link, FOO_link, base, *_base, world_link, *_world_link
  """
  global lastUpdateTime
  sinceLastUpdateDuration = rospy.get_rostime() - lastUpdateTime
  if sinceLastUpdateDuration.to_sec() < updatePeriod:
    return
  lastUpdateTime = rospy.get_rostime()

  poseDict = {}
  poseResolveList = []
  for (index, name) in enumerate(linkStatesMsg.name):
    pose = linkStatesMsg.pose[index]
    #print('%d: name=%s: pose=\n%s' % (index, name, pose))
    (parentName, modelName, linkName) = splitName(name)
    #print('parentName=%s modelName=%s linkName=%s' % (parentName, modelName, linkName))
    if isBaseLinkName(modelName, linkName):
      tfFromName=prefixName(name, parentName)
      tfToName=prefixName(name, modelName)
    else:
      tfFromName=prefixName(name, modelName)
      tfToName=prefixName(name, linkName)
    #print('tfFromName=%s tfToName=%s' % (tfFromName, tfToName))

    if tfFromName in submodelsToBeIgnored:
      print('Ignored submodel')
      continue;

    tfNameTuple = (tfFromName, tfToName)
    if tfFromName == worldLinkName:
      relativePose = pose
    else:
      poseDict[(worldLinkName, tfToName)] = (pose, False)
      relativePose = None
      poseResolveList.append((tfNameTuple, 0))
    poseDict[tfNameTuple] = (relativePose, True)
    #print('---')

  while poseResolveList:
    #print('len(poseResolveList)=%d' % (len(poseResolveList)))
    resolveTuple, poseResolveList = poseResolveList[0], poseResolveList[1:]
    tfNameTuple, resolveTrials = resolveTuple
    if resolveTrials > maxResolveTrials:
      print('Giving up to trying to resolve %s -> %s' % (tfFromName, tfToName))
      poseDict.pop(tfNameTuple)
      continue
    tfFromName, tfToName = tfNameTuple
    #print('tfFromName=%s tfToName=%s' % (tfFromName, tfToName))
    wMVfromPose = poseDict.get((worldLinkName, tfFromName), None)
    wMVtoPose = poseDict.get((worldLinkName, tfToName), None)
    if not wMVfromPose or not wMVtoPose:
      #print('Could not resolve %s -> %s trial=%d: %s' % (tfFromName, tfToName, resolveTrials, 'FROM' if not wMVfromPose else 'TO'))
      poseResolveList.append((tfNameTuple, resolveTrials + 1))
    else:
      wMVfrom = pm.fromMsg(wMVfromPose[0])
      wMVto = pm.fromMsg(wMVtoPose[0])
      fromMVto = wMVfrom.Inverse() * wMVto
      relativePose = pm.toMsg(fromMVto)
      poseDict[tfNameTuple] = (relativePose, True)

  for tfNameTuple in poseDict:
    (tfFromName, tfToName) = tfNameTuple
    if not poseDict[tfNameTuple][1]:
      continue
    relativePose = poseDict[tfNameTuple][0]
    #print('Publishing %s -> %s' % (tfFromName, tfToName))
    tfBroadcaster.sendTransform(point2Tuple(relativePose.position), quaternion2Tuple(relativePose.orientation), rospy.get_rostime(), tfToName, tfFromName)

def main():
  rospy.init_node('gazebo2tf')

  global submodelsToBeIgnored
  submodelsToBeIgnored = rospy.get_param('~ignore_submodels_of', '').split(';')
  rospy.loginfo('Ignoring submodels of: ' + str(submodelsToBeIgnored))

  global lastUpdateTime
  lastUpdateTime = rospy.get_rostime()
  linkStatesSub = rospy.Subscriber('gazebo/link_states', LinkStates, on_link_states_msg)

  global tfBroadcaster
  tfBroadcaster = tf.TransformBroadcaster()

  rospy.loginfo('Spinning')
  rospy.spin()

if __name__ == '__main__':
  main()
