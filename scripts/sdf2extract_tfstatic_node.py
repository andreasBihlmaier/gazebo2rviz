#!/usr/bin/env python
"""
Publish specific transform between two links within a SDF file as 'static' tf transformation (with different name)
"""

import argparse

import rospy
import tf
import tf_conversions.posemath as pm
from tf.transformations import *

import pysdf



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

  rospy.init_node('sdf2extract_tfstatic')

  sdf = pysdf.SDF(model=args.sdf)
  world = sdf.world
  from_link = world.get_link(source_from_tf)
  if not from_link:
    rospy.logerr('SDF %s does not contain source_from_tf %s' % (args.sdf, source_from_tf))
    return 1
  to_link = world.get_link(source_to_tf)
  if not to_link:
    rospy.logerr('SDF %s does not contain source_to_tf %s' % (args.sdf, source_to_tf))
    return 2

  from_to_tf = concatenate_matrices(inverse_matrix(from_link.pose_world), to_link.pose_world)
  translation, quaternion = pysdf.homogeneous2translation_quaternion(from_to_tf)
  
  tfBroadcaster = tf.TransformBroadcaster()

  rospy.loginfo('Publishing TF %s -> %s from SDF %s as TF %s -> %s: t=%s q=%s' % (source_from_tf, source_to_tf, args.sdf, target_from_tf, target_to_tf, translation, quaternion))
  rospy.loginfo('Spinning')
  r = rospy.Rate(args.freq)
  while not rospy.is_shutdown():
    tfBroadcaster.sendTransform(translation, quaternion, rospy.get_rostime(), target_to_tf, target_from_tf)
    r.sleep()


if __name__ == '__main__':
  main()
