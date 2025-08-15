from lidarLib.Lidar import Lidar
import sys

from lidarLib.LidarConfigs import lidarConfigs

def stop(configPath:str):
    """
        <h2>Utility function to stop a lidar.</h2>
        configPath should be the path to a lidar config file. \n
        This function boots the lidar in a specific stop mode and so is much more robust than simply starting a lidar and then stopping it. 
    """
    config = lidarConfigs.configsFromJson(configPath) # type: ignore
    config.isStop=True
    lidar = Lidar(lidarConfigs.configsFromJson(configPath)) # type: ignore



if __name__ == '__main__':
    if len(sys.argv)>1:
        stop(sys.argv[1])
    else:
        stop("lidar0.json")