#!/usr/bin/env python

import rospy
#import tf
#from geometry_msgs.msg import Pose
#import tf_conversions.posemath as pm
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker

from gazebo2rviz.model_names import *

lastUpdateTime = None
updatePeriod = 0.1
markerPub = None
modelDict = {} # modelName -> [(tfName1, meshPath1), ...]

def loadModelFromSDF(modelName):
  rospy.loginfo('Loading model: %s' % (modelName))
  return []

def on_model_states_msg(modelStatesMsg):
  global lastUpdateTime
  sinceLastUpdateDuration = rospy.get_rostime() - lastUpdateTime
  if sinceLastUpdateDuration.to_sec() < updatePeriod:
    return
  lastUpdateTime = rospy.get_rostime()

  protoMarkerMsg = Marker()
  protoMarkerMsg.header.stamp = rospy.get_rostime()
  protoMarkerMsg.frame_locked = True
  protoMarkerMsg.id = 0
  protoMarkerMsg.type = Marker.MESH_RESOURCE
  protoMarkerMsg.action = Marker.ADD
  protoMarkerMsg.pose.position.x = 0.0
  protoMarkerMsg.pose.position.y = 0.0
  protoMarkerMsg.pose.position.z = 0.0
  protoMarkerMsg.pose.orientation.x = 0.0
  protoMarkerMsg.pose.orientation.y = 0.0
  protoMarkerMsg.pose.orientation.z = 0.0
  protoMarkerMsg.pose.orientation.w = 1.0
  protoMarkerMsg.scale.x = 1.0
  protoMarkerMsg.scale.y = 1.0
  protoMarkerMsg.scale.z = 1.0
  protoMarkerMsg.mesh_use_embedded_materials = True
  protoMarkerMsg.color.a = 0.0
  protoMarkerMsg.color.r = 0.0
  protoMarkerMsg.color.g = 0.0
  protoMarkerMsg.color.b = 0.0

  for (index, name) in enumerate(modelStatesMsg.name):
    print('%d: name=%s\n' % (index, name))
    if not name in modelDict:
      modelDict[name] = loadModelFromSDF(name)

    for modelPart in modelDict[name]:
      tfName, meshPath = modelPart
      markerMsg = protoMarkerMsg
      markerMsg.header.frame_id = tfName
      markerMsg.ns = tfName
      markerMsg.mesh_resource = meshPath
      markerPub.pub(markerMsg)


def main():
  rospy.init_node('gazebo2marker')

  global submodelsToBeIgnored
  submodelsToBeIgnored = rospy.get_param('~ignore_submodels_of', '').split(';')
  rospy.loginfo('Ignoring submodels of: ' + str(submodelsToBeIgnored))

  global lastUpdateTime
  lastUpdateTime = rospy.get_rostime()
  modelStatesSub = rospy.Subscriber('gazebo/model_states', ModelStates, on_model_states_msg)

  global markerPub
  markerPub = rospy.Publisher('/visualization_marker', Marker)

  rospy.loginfo('Spinning')
  rospy.spin()

if __name__ == '__main__':
  main()
