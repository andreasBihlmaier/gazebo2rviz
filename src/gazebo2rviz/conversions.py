from __future__ import print_function

import copy

import rospy
from visualization_msgs.msg import Marker

import pysdf



protoMarkerMsg = Marker()
protoMarkerMsg.frame_locked = True
protoMarkerMsg.id = 0
protoMarkerMsg.action = Marker.ADD
protoMarkerMsg.mesh_use_embedded_materials = True
protoMarkerMsg.color.a = 0.0
protoMarkerMsg.color.r = 0.0
protoMarkerMsg.color.g = 0.0
protoMarkerMsg.color.b = 0.0
supported_geometry_types = ['mesh', 'cylinder', 'sphere', 'box']


def link2marker_msg(link, full_linkname, use_collision = False, lifetime = rospy.Duration(0)):
  marker_msg = None
  linkpart = None
  if use_collision:
    linkpart = getattr(link, 'collision')
  else: # visual
    linkpart = getattr(link, 'visual')

  if not linkpart.geometry_type in supported_geometry_types:
    return

  marker_msg = copy.deepcopy(protoMarkerMsg)
  marker_msg.header.frame_id = pysdf.sdf2tfname(full_linkname)
  marker_msg.header.stamp = rospy.get_rostime()
  marker_msg.lifetime = lifetime
  marker_msg.ns = pysdf.sdf2tfname(full_linkname)
  marker_msg.pose = pysdf.homogeneous2pose_msg(linkpart.pose)

  if linkpart.geometry_type == 'mesh':
    marker_msg.type = Marker.MESH_RESOURCE
    marker_msg.mesh_resource = linkpart.geometry_data['uri'].replace('model://', 'file://' + pysdf.models_path)
    scale = (float(val) for val in linkpart.geometry_data['scale'].split())
    marker_msg.scale.x, marker_msg.scale.y, marker_msg.scale.z = scale
  else:
    marker_msg.color.a = 1
    marker_msg.color.r = marker_msg.color.g = marker_msg.color.b = 0.5

  if linkpart.geometry_type == 'box':
    marker_msg.type = Marker.CUBE
    scale = (float(val) for val in linkpart.geometry_data['size'].split())
    marker_msg.scale.x, marker_msg.scale.y, marker_msg.scale.z = scale
  elif linkpart.geometry_type == 'sphere':
    marker_msg.type = Marker.SPHERE
    marker_msg.scale.x = marker_msg.scale.y = marker_msg.scale.z = 2.0 * float(linkpart.geometry_data['radius'])
  elif linkpart.geometry_type == 'cylinder':
    marker_msg.type = Marker.CYLINDER
    marker_msg.scale.x = marker_msg.scale.y = 2.0 * float(linkpart.geometry_data['radius'])
    marker_msg.scale.z = float(linkpart.geometry_data['length'])

  #print(marker_msg)
  return marker_msg
