#!/usr/bin/env python3
"""
Publish all links within a SDF file as 'static' tf transformation
"""

import argparse

import rospy
import tf
import tf_conversions.posemath as pm
from tf.transformations import *

import pysdf

ignored_submodels = []
tfBroadcaster = None
world = None
tfs = []



def is_ignored(model):
  model_full_name = model.get_full_name()
  for ignored_submodel in ignored_submodels:
    if model_full_name == ignored_submodel or model_full_name.endswith('::' + ignored_submodel):
      return True
  return False


def calculate_tfs(prefix):
  world.for_all_joints(calculate_joint_tf)
  for tf in tfs:
    tf[0] = prefix + pysdf.sdf2tfname(tf[0])
    tf[1] = prefix + pysdf.sdf2tfname(tf[1])


def calculate_joint_tf(joint, full_jointname):
  full_prefix = full_jointname.replace(joint.name, '')
  if is_ignored(joint.parent_model):
    print("Ignoring TF %s -> %s" % (full_prefix + joint.parent, full_prefix + joint.child))
    return
  rel_tf = concatenate_matrices(inverse_matrix(joint.tree_parent_link.pose_world), joint.tree_child_link.pose_world)
  translation, quaternion = pysdf.homogeneous2translation_quaternion(rel_tf)
  tfs.append([full_prefix + joint.parent, full_prefix + joint.child, translation, quaternion])


def publish_tf():
  for tf in tfs:
    #print(tf)
    tfBroadcaster.sendTransform(tf[2], tf[3], rospy.get_rostime(), tf[1], tf[0])


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('-f', '--freq', type=float, default=10, help='Frequency TFs are published (default: 10 Hz)')
  parser.add_argument('-p', '--prefix', default='', help='Publish with prefix')
  parser.add_argument('sdf', help='SDF model to publish (e.g. coke_can)')
  args = parser.parse_args(rospy.myargv()[1:])

  rospy.init_node('sdf2tfstatic')

  global ignored_submodels
  ignored_submodels = rospy.get_param('~ignore_submodels', '').split(';')
  rospy.loginfo('Ignoring submodels of: %s' % ignored_submodels)

  global tfBroadcaster
  tfBroadcaster = tf.TransformBroadcaster()

  global world
  sdf = pysdf.SDF(model=args.sdf)
  world = sdf.world

  calculate_tfs(args.prefix)

  rospy.loginfo('Spinning')
  r = rospy.Rate(args.freq)
  while not rospy.is_shutdown():
    publish_tf();
    r.sleep()


if __name__ == '__main__':
  main()
