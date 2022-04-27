#!/usr/bin/env python3
"""
Publish all visuals within a SDF file as rviz Markers
"""

import argparse

import rospy
from visualization_msgs.msg import Marker
from tf.transformations import *

import pysdf
from gazebo2rviz import *


updatePeriod = 0.5
use_collision = False
submodelsToBeIgnored = []
markerPub = None
world = None
markers = []



def prepare_link_marker(link, full_linkname):
  marker_msg = link2marker_msg(link, full_linkname, use_collision, rospy.Duration(2 * updatePeriod))
  if marker_msg:
    markers.append(marker_msg)


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
  parser.add_argument('-c', '--collision', action='store_true', help='Publish collision instead of visual elements')
  parser.add_argument('sdf', help='SDF model to publish (e.g. coke_can)')
  args = parser.parse_args(rospy.myargv()[1:])

  rospy.init_node('sdf2marker')

  global submodelsToBeIgnored
  submodelsToBeIgnored = rospy.get_param('~ignore_submodels_of', '').split(';')
  rospy.loginfo('Ignoring submodels of: ' + str(submodelsToBeIgnored))

  global updatePeriod
  updatePeriod = 1. / args.freq

  global use_collision
  use_collision = args.collision

  global markerPub
  markerPub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

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
