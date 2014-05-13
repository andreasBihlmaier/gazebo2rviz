#!/usr/bin/env python
"""
Publish all links within a SDF file as 'static' tf transformation
"""

import argparse

import rospy
import tf
import tf_conversions.posemath as pm

from gazebo2rviz.model_names import *
from gazebo2rviz.load_sdf import *

submodelsToBeIgnored = []
tfBroadcaster = None
markers = None


def publishTF(prefix = ''):
  for marker in markers:
    tfFromName = prefix
    tfToName = marker['tf_name']
    if tfFromName == tfToName:
      continue
    pose = marker['link_pose']
    translation = point2Tuple(pose.position)
    rotation = quaternion2Tuple(pose.orientation)
    #print('Publishing %s -> %s (translation: %s; rotation: %s)' % (tfFromName, tfToName, translation, rotation))
    tfBroadcaster.sendTransform(translation, rotation, rospy.get_rostime(), tfToName, tfFromName)


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('-f', '--freq', type=float, default=10, help='Frequency TFs are published (default: 10 Hz)')
  parser.add_argument('-p', '--prefix', help='Publish under this prefix name (default: SDF model name)')
  parser.add_argument('sdf', help='SDF model to publish (e.g. coke_can)')
  args = parser.parse_args(rospy.myargv()[1:])

  rospy.init_node('sdf2tfstatic')

  global submodelsToBeIgnored
  submodelsToBeIgnored = rospy.get_param('~ignore_submodels_of', '').split(';')
  rospy.loginfo('Ignoring submodels of: ' + str(submodelsToBeIgnored))

  global tfBroadcaster
  tfBroadcaster = tf.TransformBroadcaster()

  global markers
  markers = loadModelFromSDF(args.sdf, '', args.prefix)
  #print(markers)

  prefix = args.prefix if args.prefix else args.sdf

  rospy.loginfo('Spinning')
  r = rospy.Rate(args.freq)
  while not rospy.is_shutdown():
    publishTF(prefix);
    r.sleep()


if __name__ == '__main__':
  main()
