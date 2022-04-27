#!/usr/bin/env python3
"""
Publish all collision elements within a SDF file as MoveIt CollisionObjects
"""

import argparse
import os
try:
  from pyassimp import pyassimp
except:
  # support pyassimp > 3.0
  import pyassimp

import rospy
#from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, PoseStamped, Point
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
from tf.transformations import *

import pysdf
from gazebo2rviz import *


ignored_submodels = []
collision_objects = {}



# Slightly modified PlanningSceneInterface.__make_mesh from moveit_commander/src/moveit_commander/planning_scene_interface.py
def make_mesh(co, name, pose, filename, scale = (1, 1, 1)):
    #print("make_mesh(name=%s filename=%s)" % (name, filename))
    scene = pyassimp.load(filename)
    if not scene.meshes or len(scene.meshes) == 0:
        raise MoveItCommanderException("There are no meshes in the file")
    if len(scene.meshes[0].faces) == 0:
        raise MoveItCommanderException("There are no faces in the mesh")
    co.operation = CollisionObject.ADD
    co.id = name
    co.header = pose.header

    mesh = Mesh()
    first_face = scene.meshes[0].faces[0]
    if hasattr(first_face, '__len__'):
        for face in scene.meshes[0].faces:
            if len(face) == 3:
                triangle = MeshTriangle()
                triangle.vertex_indices = [face[0], face[1], face[2]]
                mesh.triangles.append(triangle)
    elif hasattr(first_face, 'indices'):
        for face in scene.meshes[0].faces:
            if len(face.indices) == 3:
                triangle = MeshTriangle()
                triangle.vertex_indices = [face.indices[0],
                                           face.indices[1],
                                           face.indices[2]]
                mesh.triangles.append(triangle)
    else:
        raise MoveItCommanderException("Unable to build triangles from mesh due to mesh object structure")
    for vertex in scene.meshes[0].vertices:
        point = Point()
        point.x = vertex[0]*scale[0]
        point.y = vertex[1]*scale[1]
        point.z = vertex[2]*scale[2]
        mesh.vertices.append(point)
    if not isinstance(co.meshes, list):
        co.meshes = []
    co.meshes += [mesh]

    if not isinstance(co.mesh_poses, list):
        co.mesh_poses = []
    co.mesh_poses += [pose.pose]

    pyassimp.release(scene)
    return co


def append_to_collision_object(sink_collision_object, source_collision_object):
  sink_collision_object.primitives.extend(source_collision_object.primitives)
  sink_collision_object.primitive_poses.extend(source_collision_object.primitive_poses)
  sink_collision_object.meshes.extend(source_collision_object.meshes)
  sink_collision_object.mesh_poses.extend(source_collision_object.mesh_poses)
  sink_collision_object.planes.extend(source_collision_object.planes)
  sink_collision_object.plane_poses.extend(source_collision_object.plane_poses)


def is_ignored(model):
  model_full_name = model.get_full_name()
  for ignored_submodel in ignored_submodels:
    if model_full_name == ignored_submodel or model_full_name.endswith('::' + ignored_submodel):
      return True
  return False


def get_root_collision_model(link):
  # Travere model tree upward until either at the root or an ignored submodel is found which is origin of collision object.
  model = link.parent_model
  while True:
    if not model.parent_model:
      break
    elif is_ignored(model):
      break
    model = model.parent_model
  return model


def link_to_collision_object(link, full_linkname):
  supported_geometry_types = ['mesh', 'cylinder', 'sphere', 'box']
  linkparts = getattr(link, 'collisions')

  if is_ignored(link.parent_model):
    print("Ignoring link %s." % full_linkname)
    return

  collision_object = CollisionObject()
  collision_object.header.frame_id = pysdf.sdf2tfname(full_linkname)
  root_collision_model = get_root_collision_model(link)
  link_pose_in_root_frame = pysdf.homogeneous2pose_msg(concatenate_matrices(link.pose_world, inverse_matrix(root_collision_model.pose_world)))

  for linkpart in linkparts:
      if linkpart.geometry_type not in supported_geometry_types:
          print("Element %s with geometry type %s not supported. Ignored." % (full_linkname, linkpart.geometry_type))
          continue

      if linkpart.geometry_type == 'mesh':
        scale = tuple(float(val) for val in linkpart.geometry_data['scale'].split())
        for models_path in pysdf.models_paths:
          test_mesh_path = linkpart.geometry_data['uri'].replace('model://', models_path)
          if os.path.isfile(test_mesh_path):
            mesh_path = test_mesh_path
            break
        if mesh_path:
          link_pose_stamped = PoseStamped()
          link_pose_stamped.pose = link_pose_in_root_frame
          make_mesh(collision_object, full_linkname, link_pose_stamped, mesh_path, scale)
        else:
          print("ERROR: No mesh found for '%s' in element '%s'." % (linkpart.geometry_data['uri'], full_linkname))
      elif linkpart.geometry_type == 'box':
        scale = tuple(float(val) for val in linkpart.geometry_data['size'].split())
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = scale
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(link_pose_in_root_frame)
      elif linkpart.geometry_type == 'sphere':
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = 2.0 * float(linkpart.geometry_data['radius'])
        collision_object.primitives.append(sphere)
        collision_object.primitive_poses.append(link_pose_in_root_frame)
      elif linkpart.geometry_type == 'cylinder':
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = tuple((2.0 * float(linkpart.geometry_data['radius']), float(linkpart.geometry_data['length'])))
        collision_object.primitives.append(cylinder)
        collision_object.primitive_poses.append(link_pose_in_root_frame)

  #print('CollisionObject for %s:\n%s' % (full_linkname, collision_object))
  return collision_object


def convert_to_collision_object(link, full_linkname):
  collision_object = link_to_collision_object(link, full_linkname)
  if not collision_object:
    return

  link_root = get_root_collision_model(link).root_link.name
  if link_root not in collision_objects:
    collision_objects[link_root] = CollisionObject()
    collision_objects[link_root].id = link_root
    collision_objects[link_root].operation = CollisionObject.ADD
  append_to_collision_object(collision_objects[link_root], collision_object)



def main():
  parser = argparse.ArgumentParser()
  #parser.add_argument('-f', '--freq', type=float, default=2, help='Frequency Markers are published (default: 2 Hz)')
  #parser.add_argument('-p', '--prefix', default='', help='Publish with prefix')
  parser.add_argument('sdf', help='SDF model to publish (e.g. coke_can)')
  args = parser.parse_args(rospy.myargv()[1:])

  rospy.init_node('sdf2moveit_collision')

  global ignored_submodels
  ignored_submodels = rospy.get_param('~ignore_submodels', '').split(';')
  rospy.loginfo('Ignoring submodels of: %s' % ignored_submodels)

  planning_scene_pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10)

  sdf = pysdf.SDF(model=args.sdf)
  world = sdf.world

  world.for_all_links(convert_to_collision_object)
  planning_scene_msg = PlanningScene()
  planning_scene_msg.is_diff = True
  for (collision_object_root, collision_object) in collision_objects.iteritems():
    if collision_object_root in ignored_submodels:
      print('TODO2')  # attached object instead of collision object
    else:
      planning_scene_msg.world.collision_objects.append(collision_object)
      planning_scene_msg.world.collision_objects[-1].header.frame_id = 'world'

  while planning_scene_pub.get_num_connections() < 1:
    rospy.sleep(0.1)
  planning_scene_pub.publish(planning_scene_msg)


  #rospy.loginfo('Spinning')
  #r = rospy.Rate(args.freq)
  #while not rospy.is_shutdown():
  #  publishMarkers();
  #  r.sleep()


if __name__ == '__main__':
  main()
