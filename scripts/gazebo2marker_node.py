#!/usr/bin/env python


import rospy
import tf
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker

from gazebo2rviz.model_names import *
from gazebo2rviz.load_sdf import *


lastUpdateTime = None
updatePeriod = 0.5
markerPub = None
previousModels = []
modelDict = {} # modelName -> [(tfName1, meshPose1, meshPath1), ...]



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
  protoMarkerMsg.lifetime = rospy.Duration(2 * updatePeriod)
  protoMarkerMsg.mesh_use_embedded_materials = True
  protoMarkerMsg.color.a = 0.0
  protoMarkerMsg.color.r = 0.0
  protoMarkerMsg.color.g = 0.0
  protoMarkerMsg.color.b = 0.0
  protoMarkerMsg.scale.x = 1.0
  protoMarkerMsg.scale.y = 1.0
  protoMarkerMsg.scale.z = 1.0

  currentModelSet = set(modelStatesMsg.name)
  global previousModels
  previousModelSet = set(previousModels)
  #addedModelSet = currentModelSet.difference(previousModelSet)
  addedModelSet = currentModelSet
  #print('addedModelSet=%s' % str(addedModelSet))
  for (index, name) in enumerate(addedModelSet):
    #print('%d: name=%s\n' % (index, name))
    modelName = name2modelName(name)
    if not name in modelDict:
      modelDict[name] = loadModelFromSDF(modelName, '', name)

    for modelPart in modelDict[name]:
      tfName = modelPart['tf_name']
      if not 'mesh_path' in modelPart:
        continue
      meshPose = modelPart['mesh_pose']
      meshPath = modelPart['mesh_path']
      markerMsg = protoMarkerMsg
      markerMsg.header.frame_id = tfName
      markerMsg.ns = tfName
      markerMsg.mesh_resource = meshPath
      markerMsg.pose = meshPose
      #print('Publishing ADD:\n' + str(markerMsg))
      markerPub.publish(markerMsg)

  #deletedModelSet = previousModelSet.difference(currentModelSet)
  #protoRmMarkerMsg = Marker()
  #protoRmMarkerMsg.header.stamp = rospy.get_rostime()
  #protoRmMarkerMsg.id = 0
  #protoRmMarkerMsg.action = Marker.DELETE
  #for deletedName in deletedModelSet:
  #  deletedModelName = name2modelName(deletedName)
  #  for modelPart in modelDict[deletedName]:
  #    tfName, meshPose, meshPath = modelPart
  #    rmMarkerMsg = protoRmMarkerMsg
  #    rmMarkerMsg.header.frame_id = tfName
  #    rmMarkerMsg.ns = tfName
  #    print('Publishing DELETE:\n' + str(rmMarkerMsg))
  #    markerPub.publish(rmMarkerMsg)

  previousModels = modelStatesMsg.name


def main():
  rospy.init_node('gazebo2marker')

  global submodelsToBeIgnored
  submodelsToBeIgnored = rospy.get_param('~ignore_submodels_of', '').split(';')
  rospy.loginfo('Ignoring submodels of: ' + str(submodelsToBeIgnored))

  global markerPub
  markerPub = rospy.Publisher('/visualization_marker', Marker)
  rospy.sleep(rospy.Duration(0, 100 * 1000))

  global lastUpdateTime
  lastUpdateTime = rospy.get_rostime()
  modelStatesSub = rospy.Subscriber('gazebo/model_states', ModelStates, on_model_states_msg)

  rospy.loginfo('Spinning')
  rospy.spin()

if __name__ == '__main__':
  main()
