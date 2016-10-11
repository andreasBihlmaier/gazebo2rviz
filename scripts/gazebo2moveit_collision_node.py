#!/usr/bin/env python
"""
Publish all collision elements in gazebo as MoveIt CollisionObjects
"""

from __future__ import print_function

import argparse

import rospy
from gazebo_msgs.msg import ModelStates
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
import os.path
from geometry_msgs.msg import PoseStamped, Point
from pyassimp import pyassimp

import pysdf
from tf.transformations import *

updatePeriod = 0.5
submodelsToBeIgnored = []
planning_scene_pub = None
world = None
lastUpdateTime = None

model_cache = {}
ignored_submodels = []
collision_objects = {}
collision_objects_updated = {}


# Slightly modified PlanningSceneInterface.__make_mesh from moveit_commander/src/moveit_commander/planning_scene_interface.py
def make_mesh(co, name, pose, filename, scale = (1, 1, 1)):
    print("make_mesh(name=%s filename=%s)" % (name, filename))
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
    if not isinstance(co.meshes, list):
        co.meshes = []
    co.meshes += [mesh]

    if not isinstance(co.mesh_poses, list):
        co.mesh_poses = []
    co.mesh_poses += [pose.pose]

    pyassimp.release(scene)
    return co

def is_ignored(model):
  model_full_name = model.get_full_name()
  for ignored_submodel in ignored_submodels:
    if model_full_name == ignored_submodel or model_full_name.endswith('::' + ignored_submodel):
      return True
  return False


def link_to_collision_object(link, full_linkname):
  supported_geometry_types = ['mesh', 'cylinder', 'sphere', 'box']
  linkparts = getattr(link, 'collisions')

  if is_ignored(link.parent_model):
    print("Ignoring link %s." % full_linkname)
    return

  collision_object = CollisionObject()
  collision_object.header.frame_id = pysdf.sdf2tfname(full_linkname)

  for linkpart in linkparts:
      if linkpart.geometry_type not in supported_geometry_types:
          print("Element %s with geometry type %s not supported. Ignored." % (full_linkname, linkpart.geometry_type))
          continue

      if linkpart.geometry_type == 'mesh':
        scale = tuple(float(val) for val in linkpart.geometry_data['scale'].split())
        for models_path in pysdf.models_paths:
          resource = linkpart.geometry_data['uri'].replace('model://', models_path)
          if os.path.isfile(resource):
            mesh_path = resource
            break
        link_pose_stamped = PoseStamped()
        link_pose_stamped.pose = linkpart.pose
        make_mesh(collision_object, full_linkname, link_pose_stamped, mesh_path, scale)
      elif linkpart.geometry_type == 'box':
        scale = tuple(float(val) for val in linkpart.geometry_data['size'].split())
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = scale
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(pysdf.homogeneous2pose_msg(linkpart.pose))
      elif linkpart.geometry_type == 'sphere':
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = float(linkpart.geometry_data['radius'])
        collision_object.primitives.append(sphere)
        collision_object.primitive_poses.append(pysdf.homogeneous2pose_msg(linkpart.pose))
      elif linkpart.geometry_type == 'cylinder':
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = tuple((float(linkpart.geometry_data['length']), float(linkpart.geometry_data['radius'])))
        collision_object.primitives.append(cylinder)
        collision_object.primitive_poses.append(pysdf.homogeneous2pose_msg(linkpart.pose))

  # print('CollisionObject for %s:\n%s' % (full_linkname, collision_object))
  return collision_object


def convert_to_collision_object(link, full_linkname):
  collision_object = link_to_collision_object(link, full_linkname)
  if not collision_object:
    return

  link_root = collision_object.header.frame_id
  if link_root not in collision_objects:
    collision_objects[link_root] = CollisionObject()
    collision_objects[link_root].id = link_root
    collision_objects[link_root].operation = CollisionObject.ADD
  append_to_collision_object(collision_objects[link_root], collision_object)


def append_to_collision_object(sink_collision_object, source_collision_object):
  sink_collision_object.primitives.extend(source_collision_object.primitives)
  sink_collision_object.primitive_poses.extend(source_collision_object.primitive_poses)
  sink_collision_object.meshes.extend(source_collision_object.meshes)
  sink_collision_object.mesh_poses.extend(source_collision_object.mesh_poses)
  sink_collision_object.planes.extend(source_collision_object.planes)
  sink_collision_object.plane_poses.extend(source_collision_object.plane_poses)


def update_collision_object(link, full_linkname, **kwargs):
  link_root = pysdf.sdf2tfname(full_linkname) 
  collision_objects_updated[link_root] = CollisionObject()
  collision_objects_updated[link_root].id = link_root
  collision_objects_updated[link_root].operation = CollisionObject.MOVE
  if 'pose' in kwargs:
    updated_pose = kwargs['pose']
    move_collision_object(collision_objects_updated[link_root], collision_objects[link_root], updated_pose)


def move_collision_object(sink_collision_object, source_collision_object, updated_pose):
  link_world = pysdf.pose_msg2homogeneous(updated_pose)
  for pose in source_collision_object.primitive_poses:
    primitive_pose_in_link = pysdf.pose_msg2homogeneous(pose)
    primitive_pose_in_world = pysdf.homogeneous2pose_msg(concatenate_matrices(link_world, primitive_pose_in_link))
    sink_collision_object.primitive_poses.extend([primitive_pose_in_world])
  for pose in source_collision_object.mesh_poses:
    mesh_pose_in_link = pysdf.pose_msg2homogeneous(pose)
    mesh_pose_in_world = pysdf.homogeneous2pose_msg(concatenate_matrices(link_world, mesh_pose_in_link))
    sink_collision_object.primitive_poses.extend([mesh_pose_in_world])
  for pose in source_collision_object.plane_poses:
    plane_pose_in_link = pysdf.pose_msg2homogeneous(pose)
    plane_pose_in_world = pysdf.homogeneous2pose_msg(concatenate_matrices(link_world, plane_pose_in_link))
    sink_collision_object.primitive_poses.extend([plane_pose_in_world])


def on_model_states_msg(model_states_msg):
  global lastUpdateTime
  global planning_scene_pub
  sinceLastUpdateDuration = rospy.get_rostime() - lastUpdateTime
  if sinceLastUpdateDuration.to_sec() < updatePeriod:
    return

  lastUpdateTime = rospy.get_rostime()

  for (model_idx, modelinstance_name) in enumerate(model_states_msg.name):
    model_name = pysdf.name2modelname(modelinstance_name)
    #print('model_name:', model_name)
    
    if not model_name in model_cache:
      # Add new collision object
      sdf = pysdf.SDF(model=model_name)
      model_cache[model_name] = sdf.world.models[0] if len(sdf.world.models) >= 1 else None
      if model_cache[model_name]:
        print('Loaded model: %s' % model_cache[model_name].name)
        model = model_cache[model_name]
        model.for_all_links(convert_to_collision_object)   
        planning_scene_msg = PlanningScene()
        planning_scene_msg.is_diff = True
        for (collision_object_root, collision_object) in collision_objects.iteritems():
          if collision_object_root in ignored_submodels:
            pass
          else:
            planning_scene_msg.world.collision_objects.append(collision_object)
            planning_scene_msg.world.collision_objects[-1].header.frame_id = 'world'
        planning_scene_pub.publish(planning_scene_msg)
      else:
        print('Unable to load model: %s' % model_name) 

    # Move existing object  
    model = model_cache[model_name]
    if model:
        model.for_all_links(update_collision_object, pose=model_states_msg.pose[model_idx])
    else: # Not an SDF model
      continue

    planning_scene_msg = PlanningScene()
    planning_scene_msg.is_diff = True
    for (collision_object_root, collision_object) in collision_objects_updated.iteritems():
      if collision_object_root in ignored_submodels:
        pass
      else:
        planning_scene_msg.world.collision_objects.append(collision_object)
        planning_scene_msg.world.collision_objects[-1].header.frame_id = 'world'
    planning_scene_pub.publish(planning_scene_msg)


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('-f', '--freq', type=float, default=2, help='Frequency Markers are published (default: 2 Hz)')
  args = parser.parse_args(rospy.myargv()[1:])

  rospy.init_node('gazebo2moveit')

  global submodelsToBeIgnored
  submodelsToBeIgnored = rospy.get_param('~ignore_submodels_of', '').split(';')
  rospy.loginfo('Ignoring submodels of: ' + str(submodelsToBeIgnored))

  global updatePeriod
  updatePeriod = 1. / args.freq
  print ("updatePeriod", updatePeriod)
 
  global planning_scene_pub
  planning_scene_pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10)

  global lastUpdateTime
  lastUpdateTime = rospy.get_rostime()
  modelStatesSub = rospy.Subscriber('gazebo/model_states', ModelStates, on_model_states_msg)

  rospy.loginfo('Spinning')
  rospy.spin()

if __name__ == '__main__':
  main()