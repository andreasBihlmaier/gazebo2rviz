#!/usr/bin/env python
"""
Publish specific transform between two links within a SDF file as 'static' tf transformation (with different name)
"""

import argparse

import rospy
import tf
import tf_conversions.posemath as pm

from gazebo2rviz.model_names import *
from gazebo2rviz.load_sdf import *


def get_pose(markers, tf_name):
  for marker in markers:
    if marker['tf_name'] == tf_name:
      return marker['link_pose']
  return None


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('-f', '--freq', type=float, default=10, help='Frequency TFs are published (default: 10 Hz)')
  parser.add_argument('sdf', help='SDF model to publish (e.g. coke_can)')
  parser.add_argument('source_from_tf', help='Base Frame from SDF')
  parser.add_argument('source_to_tf', help='Target Frame from SDF')
  parser.add_argument('target_from_tf', nargs='?', help='Published Base Frame')
  parser.add_argument('target_to_tf', nargs='?', help='Published Target Frame')
  args = parser.parse_args(rospy.myargv()[1:])
  source_from_tf, source_to_tf = args.source_from_tf, args.source_to_tf
  target_from_tf = args.target_from_tf if args.target_from_tf else source_from_tf
  target_to_tf = args.target_to_tf if args.target_to_tf else source_to_tf
  print('Publishing TF %s -> %s from SDF %s as TF %s -> %s' % (source_from_tf, source_to_tf, args.sdf, target_from_tf, target_to_tf))

  rospy.init_node('sdf2extract_tfstatic')

  tfBroadcaster = tf.TransformBroadcaster()

  markers = loadModelFromSDF(args.sdf)
  from_tf_pose = get_pose(markers, source_from_tf)
  if not from_tf_pose:
    print('SDF %s does not contain source_from_tf %s' % (args.sdf, source_from_tf))
    return 1
  to_tf_pose = get_pose(markers, source_to_tf)
  if not to_tf_pose:
    print('SDF %s does not contain source_to_tf %s' % (args.sdf, source_to_tf))
    return 1

  from_to_tf = pm.toMsg(pm.fromMsg(from_tf_pose).Inverse() * pm.fromMsg(to_tf_pose))
  translation = point2Tuple(from_to_tf.position)
  rotation = quaternion2Tuple(from_to_tf.orientation)


  rospy.loginfo('Spinning')
  r = rospy.Rate(args.freq)
  while not rospy.is_shutdown():
    tfBroadcaster.sendTransform(translation, rotation, rospy.get_rostime(), target_to_tf, target_from_tf)
    r.sleep()


if __name__ == '__main__':
  main()
