worldLinkName = 'gazebo_world'
baseLinkNames = ['link', 'base', 'world_link']
baseLinkNameEndings = ['_base', '_world_link']

def isBaseLinkName(linkName, modelName):
  return (linkName in baseLinkNames) \
         or (linkName == modelName + '_link') \
         or any(linkName.endswith(suffix) for suffix in baseLinkNameEndings)

def splitName(name):
  nameSplitters = name.split('::')
  (modelName, linkName) = nameSplitters[-2:]
  if len(nameSplitters) > 2:
    [parentName] = nameSplitters[-3:-2]
  else:
    parentName = worldLinkName
  return (parentName, modelName, linkName)
