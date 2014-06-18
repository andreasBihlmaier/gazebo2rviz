#!/usr/bin/env python
"""
Publish all visuals within a SDF file as rviz Markers
"""

import argparse
import copy

import rospy
from visualization_msgs.msg import Marker
from tf.transformations import *

import pysdf


submodelsToBeIgnored = []
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
world = None
markers = []



def prepare_link_marker(link, full_linkname):
  if link.visual.geometry_type == 'mesh':
    marker_msg = copy.deepcopy(protoMarkerMsg)
    marker_msg.header.frame_id = full_linkname
    marker_msg.ns = full_linkname
    marker_msg.mesh_resource = link.visual.geometry_data['uri'].replace('model://', 'file://' + pysdf.models_path)
    scale = (float(val) for val in link.visual.geometry_data['scale'].split())
    marker_msg.scale.x, marker_msg.scale.y, marker_msg.scale.z = scale
    marker_msg.pose = pysdf.homogeneous2pose_msg(link.visual.pose)
    markers.append(marker_msg)
  # TODO other geometry_types


def prepare_markers(prefix):
  world.for_all_links(prepare_link_marker)
  for marker in markers:
    marker.header.frame_id = prefix + pysdf.sdf2tfname(marker.header.frame_id)
    marker.ns = prefix + pysdf.sdf2tfname(marker.ns)


def publishMarkers():
  for marker in markers:
    #print(marker)
    markerPub.publish(marker)



def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('-f', '--freq', type=float, default=2, help='Frequency Markers are published (default: 2 Hz)')
  parser.add_argument('-p', '--prefix', default='', help='Publish with prefix')
  parser.add_argument('sdf', help='SDF model to publish (e.g. coke_can)')
  args = parser.parse_args(rospy.myargv()[1:])

  rospy.init_node('sdf2marker')

  global submodelsToBeIgnored
  submodelsToBeIgnored = rospy.get_param('~ignore_submodels_of', '').split(';')
  rospy.loginfo('Ignoring submodels of: ' + str(submodelsToBeIgnored))

  global protoMarkerMsg
  updatePeriod = 1. / args.freq
  protoMarkerMsg.lifetime = rospy.Duration(2 * updatePeriod)
  protoMarkerMsg.header.stamp = rospy.get_rostime()

  global markerPub
  markerPub = rospy.Publisher('/visualization_marker', Marker)

  global world
  sdf = pysdf.SDF(model=args.sdf)
  world = sdf.world

  prepare_markers(args.prefix)

  rospy.loginfo('Spinning')
  r = rospy.Rate(args.freq)
  while not rospy.is_shutdown():
    publishMarkers();
    r.sleep()

if __name__ == '__main__':
  main()
