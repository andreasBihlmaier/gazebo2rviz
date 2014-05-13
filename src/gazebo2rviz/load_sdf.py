import os
import xml.etree.ElementTree as ET

import rospy
from PyKDL import Frame, Rotation, Vector
import tf_conversions.posemath as pm
from geometry_msgs.msg import Pose

from gazebo2rviz.model_names import *



modelsPath = os.path.expanduser('~/.gazebo/models/')



def poseRPYList2Pose(poseRPYList):
  poseValues = [float(val) for val in poseRPYList]
  pose = Frame(Rotation.RPY(poseValues[3], poseValues[4], poseValues[5]),
               Vector(poseValues[0], poseValues[1], poseValues[2]))
  poseMsg = pm.toMsg(pose)
  #print(poseMsg)
  return poseMsg


def loadModelFromSDF(modelName, modelNamePrefix = '', modelTfName = '', absPose = poseRPYList2Pose('0 0 0  0 0 0'.split())):
  rospy.loginfo('Loading model: modelName=%s modelNamePrefix=%s modelTfName=%s' % (modelName, modelNamePrefix, modelTfName))
  if not modelTfName:
    modelTfName = modelName
  model = []
  try:
    modelTree = ET.parse(modelsPath + os.sep + modelName + os.sep + 'model.sdf')
  except IOError as e:
    print e
    return []

  absPoseTf = pm.fromMsg(absPose)

  for linkTag in modelTree.iter('link'):
    rawLinkName = linkTag.attrib['name']
    modelPart = {}
    if isBaseLinkName(modelName, rawLinkName):
      linkName = modelTfName
    else:
      linkName = modelTfName + joinString + rawLinkName
    modelPart['tf_name'] = modelNamePrefix + linkName
    modelPart['link_pose'] = poseRPYList2Pose('0 0 0 0 0 0'.split())
    for linkPoseTag in linkTag.findall('pose'):
      linkPoseElements = linkPoseTag.text.split()
      modelPart['link_pose'] = poseRPYList2Pose(linkPoseElements)
    modelPart['link_pose'] = pm.toMsg(absPoseTf * pm.fromMsg(modelPart['link_pose']))
    for visualTag in linkTag.iter('visual'):
      modelPart['mesh_pose'] = Pose()
      for poseTag in visualTag.iter('pose'):
        meshPoseElements = poseTag.text.split()
        modelPart['mesh_pose'] = poseRPYList2Pose(meshPoseElements)
      for meshTag in visualTag.iter('mesh'):
        for uriTag in meshTag.findall('uri'):
          modelPart['mesh_path'] = uriTag.text.replace('model://', 'file://' + modelsPath)
          rospy.loginfo('Found: tf_name=%s mesh_path=%s' % (modelPart['tf_name'], modelPart['mesh_path']))
    model.append(modelPart)

  for includeTag in modelTree.iter('include'):
    uriTag = includeTag.findall('uri')[0]
    includedModelName = uriTag.text.replace('model://', '')
    sdfPath = uriTag.text.replace('model://', 'file://' + modelsPath)
    nameTagList = includeTag.findall('name')
    if nameTagList:
      includedTfName = nameTagList[0].text
    else:
      includedTfName = includedModelName
    include_pose = absPose
    for includePoseTag in includeTag.findall('pose'):
      include_pose = pm.toMsg(absPoseTf * pm.fromMsg(poseRPYList2Pose(includePoseTag.text.split())))
    prefix = modelNamePrefix + modelTfName + joinString
    model.extend(loadModelFromSDF(includedModelName, prefix, includedTfName, include_pose))

  return model


def point2Tuple(point):
  return (point.x, point.y, point.z)

def quaternion2Tuple(quaternion):
  return (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
