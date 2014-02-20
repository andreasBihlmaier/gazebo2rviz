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



def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('-f', '--freq', type=float, default=2, help='Frequency Markers are published (default: 2 Hz)')
  parser.add_argument('-n', '--name', help='Publish Marker under this name (default: SDF model name)')
  parser.add_argument('sdf', help='SDF file to publish')
  args = parser.parse_args()

  rospy.init_node('sdf2marker')

  global markerPub
  markerPub = rospy.Publisher('/visualization_marker', Marker)

  rospy.loginfo('Spinning')
  rospy.spin()

if __name__ == '__main__':
  main()
