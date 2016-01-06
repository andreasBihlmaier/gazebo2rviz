#!/usr/bin/env python
"""
Publish all collision elements within a SDF file as MoveIt CollisionObjects
"""

import argparse
from pyassimp import pyassimp

import rospy
#from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, PoseStamped, Point
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
from tf.transformations import *

import pysdf
from gazebo2rviz import *


ignored_submodels = []
collision_pub = None
world = None
collision_objects = {}



# Slightly modified PlanningSceneInterface.__make_mesh from moveit_commander/src/moveit_commander/planning_scene_interface.py
def make_mesh(co, name, pose, filename, scale = (1, 1, 1)):
    print("name=%s filename=%s" % (name, filename))
    scene = pyassimp.load(filename)
    if not scene.meshes:
        raise MoveItCommanderException("There are no meshes in the file")
    co.operation = CollisionObject.ADD
    co.id = name
    co.header = pose.header
    
    mesh = Mesh()
    for face in scene.meshes[0].faces:
        triangle = MeshTriangle()
        if len(face.indices) == 3:
            triangle.vertex_indices = [face.indices[0], face.indices[1], face.indices[2]]
        mesh.triangles.append(triangle)
    for vertex in scene.meshes[0].vertices:
        point = Point()
        point.x = vertex[0]*scale[0]
        point.y = vertex[1]*scale[1]
        point.z = vertex[2]*scale[2]
        mesh.vertices.append(point)
    co.meshes = [mesh]
    co.mesh_poses = [pose.pose]
    pyassimp.release(scene)
    return co


def append_to_collision_object(collision_object_target, collision_object_source):
  print("Appending\n%s\n to\n%s" % (collision_object_source, collision_object_target))


def get_root_collision_model(link):
  # Travere model tree upward until either at the root or an ignored submodel is found which is origin of collision object.
  model = link.parent_model
  while True:
    if not model.parent_model:
      break
    elif model.name in ignored_submodels:
      print('TODO')
      break
    model = model.parent_model
  return model


def link_to_collision_object(link, full_linkname):
  supported_geometry_types = ['mesh', 'cylinder', 'sphere', 'box']
  linkpart = getattr(link, 'collision')

  if linkpart.geometry_type not in supported_geometry_types:
    print("Element %s with geometry type %s not supported. Ignored." % (full_linkname, linkpart.geometry_type))
    return

  collision_object = CollisionObject()
  collision_object.header.frame_id = pysdf.sdf2tfname(full_linkname)

  if linkpart.geometry_type == 'mesh':
    scale = tuple(float(val) for val in linkpart.geometry_data['scale'].split())
    mesh_path = linkpart.geometry_data['uri'].replace('model://', pysdf.models_path)
    root_collision_model = get_root_collision_model(link)
    link_pose_in_parent_frame = pysdf.homogeneous2pose_msg(link.pose_world * inverse_matrix(root_collision_model.pose_world))
    link_pose_stamped = PoseStamped()
    link_pose_stamped.pose = link_pose_in_parent_frame
    make_mesh(collision_object, full_linkname, link_pose_stamped, mesh_path, scale)
  elif linkpart.geometry_type == 'box':
    print('TODO')
    #marker_msg.type = Marker.CUBE
    #scale = (float(val) for val in linkpart.geometry_data['size'].split())
    #marker_msg.scale.x, marker_msg.scale.y, marker_msg.scale.z = scale
  elif linkpart.geometry_type == 'sphere':
    print('TODO')
    #marker_msg.type = Marker.SPHERE
    #marker_msg.scale.x = marker_msg.scale.y = marker_msg.scale.z = 2.0 * float(linkpart.geometry_data['radius'])
  elif linkpart.geometry_type == 'cylinder':
    print('TODO')
    #marker_msg.type = Marker.CYLINDER
    #marker_msg.scale.x = marker_msg.scale.y = 2.0 * float(linkpart.geometry_data['radius'])
    #marker_msg.scale.z = float(linkpart.geometry_data['length'])

  print('CollisionObject for %s:\n%s' % (full_linkname, collision_object))
  return collision_object


def convert_to_collision_object(link, full_linkname):
  collision_object = link_to_collision_object(link, full_linkname)
  if not collision_object:
    return

  link_root = get_root_collision_model(link).root_link.name
  print("link_root=%s" % link_root)
  if link_root not in collision_objects:
    collision_objects[link_root] = CollisionObject()
    collision_objects[link_root].id = link_root
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
  collision_pub = rospy.Publisher('/visualization_marker', CollisionObject, queue_size=10)

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
