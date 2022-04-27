#!/usr/bin/env python3

import argparse

import rospy
import tf
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker

import pysdf
from gazebo2rviz import *


updatePeriod = 0.5
use_collision = False
submodelsToBeIgnored = []
markerPub = None
world = None
model_cache = {}
lastUpdateTime = None
worldsdf = None



def publish_link_marker(link, full_linkname, **kwargs):
  full_linkinstancename = full_linkname
  if 'model_name' in kwargs and 'instance_name' in kwargs:
    full_linkinstancename = full_linkinstancename.replace(kwargs['model_name'], kwargs['instance_name'], 1)
  marker_msgs = link2marker_msg(link, full_linkinstancename, use_collision, rospy.Duration(2 * updatePeriod))
  if len(marker_msgs) > 0:
    for marker_msg in marker_msgs:
      markerPub.publish(marker_msg)


def on_model_states_msg(model_states_msg):
  global lastUpdateTime
  sinceLastUpdateDuration = rospy.get_rostime() - lastUpdateTime
  if sinceLastUpdateDuration.to_sec() < updatePeriod:
    return
  lastUpdateTime = rospy.get_rostime()

  for (model_idx, modelinstance_name) in enumerate(model_states_msg.name):
    #print(model_idx, modelinstance_name)
    model_name = pysdf.name2modelname(modelinstance_name)
    #print('model_name:', model_name)
    if not model_name in model_cache:
      model_cache[model_name] = None
      if worldsdf:
        for model in worldsdf.world.models:
          if model.name == model_name:
            model_cache[model_name] = model
            break
      else:
        sdf = pysdf.SDF(model=model_name)
        if len(sdf.world.models) >= 1:
          model_cache[model_name] = sdf.world.models[0]
      if model_cache[model_name]:
        rospy.loginfo('Loaded model: %s' % model_cache[model_name].name)
      else:
        rospy.loginfo('Unable to load model: %s' % model_name)
    model = model_cache[model_name]
    if not model: # Not an SDF model
      continue
    #print('model:', model)
    model.for_all_links(publish_link_marker, model_name=model_name, instance_name=modelinstance_name)


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('-f', '--freq', type=float, default=2, help='Frequency Markers are published (default: 2 Hz)')
  parser.add_argument('-c', '--collision', action='store_true', help='Publish collision instead of visual elements')
  parser.add_argument('-w', '--worldfile', type=str, help='Read models from this world file')
  args = parser.parse_args(rospy.myargv()[1:])

  rospy.init_node('gazebo2marker')

  global submodelsToBeIgnored
  submodelsToBeIgnored = rospy.get_param('~ignore_submodels_of', '').split(';')
  rospy.loginfo('Ignoring submodels of: ' + str(submodelsToBeIgnored))
  if submodelsToBeIgnored:
    rospy.logerr('ignore_submodels_of is currently not supported and will thus have no effect')


  global updatePeriod
  updatePeriod = 1. / args.freq

  global use_collision
  use_collision = args.collision

  if args.worldfile:
    global worldsdf
    worldsdf = pysdf.SDF(file=args.worldfile)

  global markerPub
  markerPub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
  rospy.sleep(rospy.Duration(0, 100 * 1000))

  global lastUpdateTime
  lastUpdateTime = rospy.get_rostime()
  modelStatesSub = rospy.Subscriber('gazebo/model_states', ModelStates, on_model_states_msg)

  rospy.loginfo('Spinning')
  rospy.spin()

if __name__ == '__main__':
  main()
