#!/usr/bin/env python
"""
Publish all visuals within a SDF file as rviz Markers
"""

import argparse

import rospy
from visualization_msgs.msg import Marker

from gazebo2rviz.model_names import *
from gazebo2rviz.load_sdf import *


markerPub = None
protoMarkerMsg = Marker()
protoMarkerMsg.frame_locked = True
protoMarkerMsg.id = 0
protoMarkerMsg.type = Marker.MESH_RESOURCE
protoMarkerMsg.action = Marker.ADD
protoMarkerMsg.mesh_use_embedded_materials = True
protoMarkerMsg.color.a = 0.0
protoMarkerMsg.color.r = 0.0
protoMarkerMsg.color.g = 0.0
protoMarkerMsg.color.b = 0.0
protoMarkerMsg.scale.x = 1.0
protoMarkerMsg.scale.y = 1.0
protoMarkerMsg.scale.z = 1.0
markers = []



def publishMarkers():
  for marker in markers:
    tfName, meshPose, meshPath = marker
    markerMsg = protoMarkerMsg
    markerMsg.header.frame_id = tfName
    markerMsg.ns = tfName
    markerMsg.mesh_resource = meshPath
    markerMsg.pose = meshPose
    #print('Publishing:\n' + str(markerMsg))
    markerPub.publish(markerMsg)



def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('-f', '--freq', type=float, default=2, help='Frequency Markers are published (default: 2 Hz)')
  parser.add_argument('-n', '--name', help='Publish Marker under this name (default: SDF model name)')
  parser.add_argument('sdf', help='SDF model to publish (e.g. coke_can)')
  args = parser.parse_args()

  rospy.init_node('sdf2marker')

  global protoMarkerMsg
  updatePeriod = 1. / args.freq
  protoMarkerMsg.lifetime = rospy.Duration(2 * updatePeriod)
  protoMarkerMsg.header.stamp = rospy.get_rostime()

  global markerPub
  markerPub = rospy.Publisher('/visualization_marker', Marker)

  global markers
  markers = loadModelFromSDF(args.sdf, '', args.name)
  #print(markers)

  rospy.loginfo('Spinning')
  r = rospy.Rate(args.freq)
  while not rospy.is_shutdown():
    publishMarkers();
    r.sleep()

if __name__ == '__main__':
  main()
