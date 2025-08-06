from lidarLib.LidarConfigs import lidarConfigs
from lidarLib import LidarConfigs


config:lidarConfigs = lidarConfigs("/dev/lidar0")
config.printConfigs()
config.writeToJson("lidar0.json")

config = lidarConfigs.configsFromJson("lidar0.json")
config.printConfigs()