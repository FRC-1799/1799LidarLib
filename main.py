
'''Animates distances and measurement quality'''

from multiprocessing import Process
import threading
from constants import constants
from lidarLib.lidarMeasurement import lidarMeasurement
from lidarLib.Lidar import Lidar
import matplotlib.pyplot as plot
import numpy as np
import matplotlib.animation as animation
from functools import partial
import pickle
import time
from lidarLib.translation import translation
from renderLib.renderMachine import initMachine
from lidarLib import lidarManager
from lidarLib.lidarPipeline import lidarPipeline
from lidarHitboxingMap import lidarHitboxMap
import multiexit
from lidarLib.LidarConfigs import lidarConfigs
from lidarLib.FRCLidarPublisher import publisher





def session(ntPublisher:publisher, shouldLiveSupplier:callable):
    print("session started\n\n\n\n")
    lidar0:lidarPipeline = lidarManager.makePipedLidar(lidarConfigs.configsFromJson("lidar0.json"))
    # lidar0.connectSmart()
    
    lidars = [lidar0]
    time.sleep(5)
    


    # cartRenderer, cartPipe = initMachine(1)
    # polarRenderer, polarPipe = initMachine(0)
    
    while shouldLiveSupplier():
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
    time.sleep(1)
    print("session terminated")

    

def main():
    ntPublisher = publisher(teamNumber=1799)
    thread = threading.Thread(target=session, daemon=True, kwargs={"ntPublisher":ntPublisher, "shouldLiveSupplier":ntPublisher.isConnected})
    
    


    while True:
        print("while cycle")
        if (ntPublisher.isConnected()) and not thread.is_alive():
            thread = threading.Thread(target=session, daemon=True, kwargs={"ntPublisher":ntPublisher, "shouldLiveSupplier":ntPublisher.isConnected})
            print(ntPublisher.isConnected(), thread.is_alive())
            thread.start()
            

        if (not ntPublisher.isConnected()) and thread.is_alive():
            thread.join(5)
            if thread.is_alive:
                print("Lidar thread did not properly terminate")
            
        
        time.sleep(5)
            

            


if __name__ == '__main__':
    main()