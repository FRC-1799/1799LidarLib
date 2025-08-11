import json
import threading
import time
from typing import Any
import typing
from lidarLib import lidarManager
from lidarLib.translation import translation
from lidarLib.lidarPipeline import lidarPipeline
from lidarLib.FRCLidarPublisher import publisher
from lidarLib.lidarMeasurement import lidarMeasurement
import sys
from lidarLib.LidarConfigs import lidarConfigs
class FRCQuickstartLidarProject:

    def __init__(self, configs:list[lidarConfigs], teamNumber:int=0, autoStart:bool=True):
        if len(configs) == 0:
            raise ValueError("a quickstart lidar project must have at least one lidar")
        
        self.configs=configs
        self.teamNumber=teamNumber
        self.publisher = publisher(self.teamNumber)
        if autoStart:
            self.main()


    @classmethod
    @typing.no_type_check

    def fromConfigs(cls:"FRCQuickstartLidarProject", filepath:str)->"FRCQuickstartLidarProject": # type: ignore
        configList:list[lidarConfigs]=[]
        teamNumber=0

        with open(filepath, 'r') as file:
            data:dict[str, Any] = json.load(file)
            configPathList:list[str] = data.get("lidarConfigs") # type: ignore

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
                    configList.append(lidarConfigs.configsFromJson(configPath)) # type: ignore
                except ValueError:
                    raise Warning("WARNING: Could not create a lidar config from path", configPath)
                except OSError:
                    raise Warning("WARNING: Could not find json file on path", configPath)

        if len(configList) == 0:
            raise ValueError("no lidar configs were able to resolved on path given", filepath)
        
        return cls(configList, teamNumber) # type: ignore

   
        

    def main(self):
        thread = None#threading.Thread(target=self.session, daemon=True, kwargs={"ntPublisher":self.publisher, "shouldLiveSupplier":self.publisher.isConnected, "configList":self.configs})
        
        


        while True:
            if (self.publisher.isConnected()) and ((not thread) or (not thread.is_alive())):
                # print(self.publisher.isConnected(), thread.is_alive())
                thread = threading.Thread(target=FRCQuickstartLidarProject.session, daemon=True, kwargs={"ntPublisher":self.publisher, "configList":self.configs})
                thread.start()
                

            if (not self.publisher.isConnected()) and thread!=None and thread.is_alive():
                thread.join(5)
                if thread.is_alive():
                    raise Warning("Lidar thread did not properly terminate")
                if input("should continue") == 'n':
                    return
                
            print("update", thread )
            time.sleep(5)
            
    @classmethod
    def session(cls, ntPublisher:publisher, configList:list[lidarConfigs]):
        
        
        lidars:list[lidarPipeline] = []
        for config in configList:
            lidars.append(lidarManager.makePipedLidar(config))
        
        time.sleep(3)

        
        while ntPublisher.isConnected():

            pointMap:list[lidarMeasurement]=[]
            lidarTranslations:list[translation] = []
            for lidar in lidars:
                if lidar.isConnected() and lidar.getLastMap():
                    lidar.setCurrentLocalTranslation(ntPublisher.getRobotPoseAsTrans())
                    pointMap = pointMap+lidar.getLastMap().getPoints()
                    lidarTranslations.append(lidar.getCombinedTranslation())
                    
            
            ntPublisher.publishPointsFromLidarMeasurements(pointMap)
            ntPublisher.publishLidarPosesFromTrans(lidarTranslations)

        for lidar in lidars:
            lidar.sendQuitRequest()
        time.sleep(5)


    

if __name__ == '__main__':
    if len(sys.argv)>1:
        FRCQuickstartLidarProject.fromConfigs(sys.argv[1]) # type: ignore
    else:
        raise ValueError("FRC quickstart projects must be run with a command line argument detailing a json config file.")