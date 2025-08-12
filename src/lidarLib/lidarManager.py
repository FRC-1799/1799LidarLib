
from lidarLib.LidarConfigs import lidarConfigs
from multiprocessing import Pipe, Process
from lidarLib import *
from lidarLib.Lidar import Lidar
import setproctitle

from lidarLib.lidarPipeline import dataPacket, dataPacketType, lidarPipeline
import time



def lidarManager(pipeline:"lidarPipeline", lidarConfig:lidarConfigs):
    print("Manager start")

    setproctitle.setproctitle(lidarConfig.name + "manager")

    lidar:Lidar = Lidar(lidarConfig)

    if (not lidarConfig.autoConnect):
        raise ValueError("piped lidars must be created with auto connect on but lidar", lidarConfig.port, "was created as piped with it off")


    pipeline._sendScanTypes(lidar.getScanModes()) # type: ignore
    pipeline._sendSampleRate(lidar.getSampleRate()) # type: ignore
    pipeline._sendLidarInfo(lidar.getInfo()) # type: ignore
    pipeline._sendScanModeTypical(lidar.getScanModeTypical()) # type: ignore
    pipeline._sendScanModeCount(lidar.getScanModeCount()) # type: ignore


    quitCount=0
    timesReset=0
    
    start =time.perf_counter()

    while pipeline.shouldLive:
        
        

        if not pipeline.isConnected():
            time.sleep(0.1)
            break


        if not lidar.isRunning() and lidar.isConnected():
            lidar.startScan()
            time.sleep(5)
            quitCount+=100
            if quitCount>10000:
                lidar.disconnect()

                timesReset+= 1
                pipeline._sendData(dataPacket(dataPacketType.quitWarning, timesReset)) # type: ignore
                time.sleep(0.001)

                lidar:Lidar = Lidar(lidarConfig)
                lidar.connect()
                lidar.setMotorPwm(500)
                lidar.startScan()

        else:
            quitCount-=1
        

        if pipeline.getDataPacket(dataPacketType.translation):
            lidar.setCurrentGlobalTranslation(pipeline.getDataPacket(dataPacketType.translation).data)
        


        for action in pipeline._getActionQue(): # type: ignore
            if action.function==Lidar.startScan: # type: ignore
                try:
                    action.function(lidar, *action.args) # type: ignore
                except:
                    lidar.stop()
                    
                    time.sleep(1)
                    lidar.lidarSerial.flush()
                    # lidar:Lidar = Lidar(*lidarArgs)
                    # lidar.setCurrentLocalTranslation(localTranslation)
                    # lidar.connect(connectionArgs)
                    
                    lidar.startScan()
 

            
            elif action.returnType==-1:
                action.function(lidar, *action.args) # type: ignore
            else:
                pipeline._sendData(dataPacket(action.returnType, action.function(lidar, *action.args))) # type: ignore

        if (lidar.getLastMap()):
            pipeline._sendMap(lidar.getLastMap()) # type: ignore
        
        pipeline._sendTrans(lidar.getCombinedTrans()) # type: ignore

        
        sleepClock:float=start+0.02-time.perf_counter()
        if sleepClock>0:
            time.sleep(sleepClock)

        
        start+=0.02

    print("lidar shut down")
    lidar.disconnect()
    exit()




def makePipedLidar(lidarConfig:lidarConfigs)-> "lidarPipeline":
    """
        Creates a separate posses that handles all rendering and can be updated via a pipe(connection)
        returns a tuple with the first argument being the process, this can be use cancel the process but the primary use is to be saved so the renderer doesn't get collected
        the second argument is one end of a pipe that is used to update the render engine. this pipe should be passed new lidar maps periodically so they can be rendered. 
    """


    print("manager")
    returnPipe, lidarPipe = Pipe(duplex=True)
    returnPipe=lidarPipeline(returnPipe)
    lidarPipe=lidarPipeline(lidarPipe)
    process= Process(target=lidarManager, args=(lidarPipe, lidarConfig), daemon=True)
    returnPipe.host=process

    process.start()


    return returnPipe

    