from __future__ import print_function

import copy

import rospy
from visualization_msgs.msg import Marker

import pysdf



protoMarkerMsg = Marker()
protoMarkerMsg.frame_locked = True
protoMarkerMsg.id = 0
protoMarkerMsg.type = Marker.MESH_RESOURCE
protoMarkerMsg.action = Marker.ADD
protoMarkerMsg.mesh_use_embedded_materials = True
protoMarkerMsg.color.a = 0.0
protoMarkerMsg.color.r = 0.0
protoMarkerMsg.color.g = 0.0
protoMarkerMsg.color.b = 0.0


def link2marker_msg(link, full_linkname, lifetime = rospy.Duration(0)):
  marker_msg = None
  if link.visual.geometry_type == 'mesh':
    marker_msg = copy.deepcopy(protoMarkerMsg)
    marker_msg.header.frame_id = full_linkname
    marker_msg.header.stamp = rospy.get_rostime()
    marker_msg.lifetime = lifetime
    marker_msg.ns = full_linkname
    marker_msg.mesh_resource = link.visual.geometry_data['uri'].replace('model://', 'file://' + pysdf.models_path)
    scale = (float(val) for val in link.visual.geometry_data['scale'].split())
    marker_msg.scale.x, marker_msg.scale.y, marker_msg.scale.z = scale
    marker_msg.pose = pysdf.homogeneous2pose_msg(link.visual.pose)
  # TODO other geometry_types

  return marker_msg
