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
    """<h2>Class to manage a fully manage a Lidar system for FRC robots and publish data to network tables.</h2>"""

    def __init__(self, configs:list[lidarConfigs], teamNumber:int=0, autoStart:bool=True):
        """
            <h2> Creates a Lidar system for FRC robots</h2>
            WARNING. This function will start an infinite loop unless it auto start is set to false (in which case no lidar data will be read). 
            If other code needs to run at the same time as this system please use a thread. 

            Configs should be a list of lidar config options to be managed. If you would like to use a lidar project .json use the fromConfigs method instead.\n
            Team number should be an FRC team number. 
            If set to 0 or left on default the code will still work but will be unable to connect to real robots, only simulated ones (on port 127.0.0.1).\n

            AutoStart determines wether the system should start running. Otherwise it will need to be started by calling the main function. \n

        """
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
        """
            <h2>Creates a lidar FRC lidar system from a .json file.
            WARNING. This function will start an infinite loop unless it auto start is set to false (in which case no lidar data will be read). 
            If other code needs to run at the same time as this system please use a thread. 
        """
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
        """
            <h2> Main loop of a quickstart project.</h2>
            WARNING. This function will start an infinite loop. 
            If other code needs to run at the same time as this system please use a thread. 
        """
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

                
            time.sleep(5)
            
    @classmethod
    def session(cls, ntPublisher:publisher, configList:list[lidarConfigs]):
        """
            <h2> A single session of a lidar project.</h2>
            WARNING. This function will loop as long as the given publisher is connected so if any other code needs to run while the session is active it should run in a thread.

            ntPublisher should be a lidarLib publisher object or something polymorphic. \n
            config list should be a list of lidar configs with at least one config.\n

            This session contains all the lidars within it so when the session ends so will all the lidars. 
            The session will only last as long as the given publisher is connected. Once killed a new session will need to be started.
        """
        
        lidars:list[lidarPipeline] = []
        for config in configList:
            lidars.append(lidarManager.makePipedLidar(config))
        
        time.sleep(10)
        
        while ntPublisher.isConnected():

            pointMap:list[lidarMeasurement]=[]
            lidarTranslations:list[translation] = []
            for lidar in lidars:
                if not lidar.isConnected():
                    
                    index = lidars.index(lidar)
                    print("lidar", configList[index].name, "Restarted")

                    lidars[index]=lidarManager.makePipedLidar(configList[index])   

                elif lidar.getLastMap():
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