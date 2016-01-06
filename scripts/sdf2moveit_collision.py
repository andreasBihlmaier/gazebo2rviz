#!/usr/bin/env python
"""
Publish all collision elements within a SDF file as MoveIt CollisionObjects
"""

import argparse

import rospy
#from visualization_msgs.msg import Marker
from moveit_msgs.msg import CollisionObject
from tf.transformations import *

import pysdf
from gazebo2rviz import *


ignored_submodels = []
collision_pub = None
world = None
collision_objects = {}


def append_to_collision_object(collision_object_target, collision_object_source):
  print("Appending\n%s\n to\n%s" % (collision_object_source, collision_object_target))


def get_root_link(link):
  # Travere model tree upward until either at the root or an ignored submodel is found which is origin of collision object.
  model = link.parent_model
  while True:
    if not model.parent_model:
      break
    elif model.name in ignored_submodels:
      print('TODO')
      break
    model = model.parent_model
  return model.root_link.name


def link_to_collision_object(link, full_linkname):
  supported_geometry_types = ['mesh', 'cylinder', 'sphere', 'box']
  linkpart = getattr(link, 'collision')

  if linkpart.geometry_type not in supported_geometry_types:
    print("Element %s with geometry type %s not supported. Ignored." % (full_linkname, linkpart.geometry_type))
    return

  collision_object = CollisionObject()
  collision_object.header.frame_id = pysdf.sdf2tfname(full_linkname)

  # TODO
  #if linkpart.geometry_type == 'mesh':
  #  marker_msg.type = Marker.MESH_RESOURCE
  #  marker_msg.mesh_resource = linkpart.geometry_data['uri'].replace('model://', 'file://' + pysdf.models_path)
  #  scale = (float(val) for val in linkpart.geometry_data['scale'].split())
  #  marker_msg.scale.x, marker_msg.scale.y, marker_msg.scale.z = scale
  #else:
  #  marker_msg.color.a = 1
  #  marker_msg.color.r = marker_msg.color.g = marker_msg.color.b = 0.5

  #if linkpart.geometry_type == 'box':
  #  marker_msg.type = Marker.CUBE
  #  scale = (float(val) for val in linkpart.geometry_data['size'].split())
  #  marker_msg.scale.x, marker_msg.scale.y, marker_msg.scale.z = scale
  #elif linkpart.geometry_type == 'sphere':
  #  marker_msg.type = Marker.SPHERE
  #  marker_msg.scale.x = marker_msg.scale.y = marker_msg.scale.z = 2.0 * float(linkpart.geometry_data['radius'])
  #elif linkpart.geometry_type == 'cylinder':
  #  marker_msg.type = Marker.CYLINDER
  #  marker_msg.scale.x = marker_msg.scale.y = 2.0 * float(linkpart.geometry_data['radius'])
  #  marker_msg.scale.z = float(linkpart.geometry_data['length'])

  #print(marker_msg)
  return collision_object


def convert_to_collision_object(link, full_linkname):
  collision_object = link_to_collision_object(link, full_linkname)
  if not collision_object:
    return

  link_root = get_root_link(link)
  print("link_root=%s" % link_root)
  if link_root not in collision_objects:
    collision_objects[link_root] = CollisionObject()
  append_to_collision_object(collision_objects[link_root], collision_object)



def main():
  parser = argparse.ArgumentParser()
  #parser.add_argument('-f', '--freq', type=float, default=2, help='Frequency Markers are published (default: 2 Hz)')
  #parser.add_argument('-p', '--prefix', default='', help='Publish with prefix')
  parser.add_argument('sdf', help='SDF model to publish (e.g. coke_can)')
  args = parser.parse_args(rospy.myargv()[1:])

  rospy.init_node('sdf2moveit_collision')

  global ignored_submodels
  ignored_submodels = rospy.get_param('~ignore_submodels_of', '').split(';')
  rospy.loginfo('Ignoring submodels of: %s' % ignored_submodels)

  global collision_pub
  collision_pub = rospy.Publisher('/visualization_marker', CollisionObject)

  global world
  sdf = pysdf.SDF(model=args.sdf)
  world = sdf.world

  world.for_all_links(convert_to_collision_object)

  #rospy.loginfo('Spinning')
  #r = rospy.Rate(args.freq)
  #while not rospy.is_shutdown():
  #  publishMarkers();
  #  r.sleep()


if __name__ == '__main__':
  main()
