import json
import threading
import time
from lidarHitboxingMap import lidarHitboxMap
from lidarLib import lidarManager
from lidarLib.FRCLidarPublisher import publisher
from lidarLib.Lidar import Lidar
import sys
import warnings
from lidarLib.LidarConfigs import lidarConfigs
from lidarLib.lidarPipeline import lidarPipeline
class FRCQuickstartLidarProject:

    def __init__(self, configs:list[lidarConfigs], teamNumber:int, autoStart=True):
        if len(configs) == 0:
            raise ValueError("a quickstart lidar project must have at least one lidar")
        
        self.configs=configs
        self.teamNumber=teamNumber
        self.publisher = publisher(self.teamNumber)
        print(self.configs)
        if autoStart:
            self.main()


    @classmethod
    def fromConfigs(cls:"lidarConfigs", filepath:str)->"FRCQuickstartLidarProject":
        configList=[]
        teamNumber=0

        with open(filepath, 'r') as file:
            data:dict = json.load(file)
            configPathList = data.get("lidarConfigs")

            if data.get("type", "") != "projectConfig":
                raise Warning(
                    "the file given does not include the correct type tag.",
                    "Please make sure to include \"type\" : \"projectConfig\" in all project config files so that the library can easily differentiate them."
                )

            
            teamNumber=data.get("teamNumber", 0)

            if teamNumber==0:
                
                raise Warning("Could not find a team number in config file. The process will continue but will only be able to connect to a simulated robot (port 127.0.0.1)")

            for configPath in configPathList:
                try:
                    configList.append(lidarConfigs.configsFromJson(configPath))
                except ValueError:
                    raise Warning("WARNING: Could not create a lidar config from path", configPath)
                except OSError:
                    raise Warning("WARNING: Could not find json file on path", configPath)

        if len(configList) == 0:
            raise ValueError("no lidar configs were able to resolved on path given", filepath)
        
        return cls(configList, teamNumber)

   
        

    def main(self):
        print(self.publisher)
        thread = None#threading.Thread(target=self.session, daemon=True, kwargs={"ntPublisher":self.publisher, "shouldLiveSupplier":self.publisher.isConnected, "configList":self.configs})
        
        


        while True:
            if (self.publisher.isConnected()) and (not thread or not thread.is_alive()):
                # print(self.publisher.isConnected(), thread.is_alive())
                thread = threading.Thread(target=FRCQuickstartLidarProject.session, daemon=True, kwargs={"ntPublisher":self.publisher, "configList":self.configs})
                thread.start()
                

            if (not self.publisher.isConnected()) and thread!=None and thread.is_alive():
                thread.join(5)
                if thread.is_alive:
                    raise Warning("Lidar thread did not properly terminate")
                
            
            time.sleep(5)
            
    @classmethod
    def session(cls, ntPublisher:publisher, configList:list[lidarConfigs]):
        
        
        lidars = []
        for config in configList:
            lidars.append(lidarManager.makePipedLidar(config))
        


        
        while ntPublisher.isConnected():

            hitboxMap:lidarHitboxMap = lidarHitboxMap()
            pointMap=[]
            lidarTranslations = []
            for lidar in lidars:
                if lidar.isConnected() and lidar.getLastMap():
                    lidar.setCurrentLocalTranslation(ntPublisher.getPoseAsTran())
                    hitboxMap.addMap(lidar.getLastMap())
                    pointMap = pointMap+lidar.getLastMap().getPoints()
                    lidarTranslations.append(lidar.getCombinedTranslation())
                    
            
            ntPublisher.publishHitboxesFromHitboxMap(hitboxMap)
            ntPublisher.publishPointsFromLidarMeasurements(pointMap)
            ntPublisher.publishLidarPosesFromTrans(lidarTranslations)


        for lidar in lidars:
            lidar.sendQuitRequest()


    

if __name__ == '__main__':
    if len(sys.argv)>1:
        FRCQuickstartLidarProject.fromConfigs(sys.argv[1])
    else:
        raise ValueError("FRC quickstart projects must be run with a command line argument detailing a json config file.")