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


def loadModelFromSDF(modelName, modelNamePrefix = '', modelTfName = ''):
  rospy.loginfo('Loading model: modelName=%s modelNamePrefix=%s modelTfName=%s' % (modelName, modelNamePrefix, modelTfName))
  if not modelTfName:
    modelTfName = modelName
  model = []
  modelTree = ET.parse(modelsPath + os.sep + modelName + os.sep + 'model.sdf')
  for linkTag in modelTree.iter('link'):
    rawLinkName = linkTag.attrib['name']
    if isBaseLinkName(modelName, rawLinkName):
      linkName = modelTfName
    else:
      linkName = modelTfName + joinString + rawLinkName
    for visualTag in linkTag.iter('visual'):
      meshPose = Pose()
      for poseTag in visualTag.iter('pose'):
        meshPoseElements = poseTag.text.split()
        meshPose = poseRPYList2Pose(meshPoseElements)
      for meshTag in visualTag.iter('mesh'):
        for uriTag in meshTag.findall('uri'):
          meshPath = uriTag.text.replace('model://', 'file://' + modelsPath)
          tfName = modelNamePrefix + linkName
          modelPart = (tfName, meshPose, meshPath)
          rospy.loginfo('Found: tfName=%s meshPath=%s' % (tfName, meshPath))
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
    prefix = modelNamePrefix + modelTfName + joinString
    model.extend(loadModelFromSDF(includedModelName, prefix, includedTfName))

  return model
