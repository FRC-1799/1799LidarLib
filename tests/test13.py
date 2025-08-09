
'''Animates distances and measurment quality'''
from lidarLib.LidarConfigs import lidarConfigs
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
from lidarLib import lidarManager, lidarPipeline


PORT_NAME = '/dev/ttyUSB0'
DMAX = 1600
IMIN = 20
IMAX = 20



def run():
    lidar = lidarManager.makePipedLidar(lidarConfigs("/dev/lidar0", defaultSpeed=500))
    lidar.connectSmart()
    
    

    # axis = subplot.scatter([0, 1], [100, 2000], s=1, c=[IMIN, IMAX],
    #                        cmap=plot.cm.Greys_r, lw=0)
    
    #lidar.setCurrentLocalTranslation(translation(0,0, 180))
    time.sleep(1)
    #lidar.currentMap.printMap()
    #print(lidar.currentMap.points)
    #lidar.currentMap.thisFuncDoesNothing()
    renderer, pipe = initMachine()
    
    
    try:
        while pipe.isConnected():
            
            pipe.send(lidar.getLastMap())
            time.sleep(0.1)
            #print("data sent", lidar.lastMap.mapID)
    except Exception as e:
        print("Exception: ", e)
            #print("data sent", lidar.lastMap.mapID)

    # ani = animation.FuncAnimation(
    # fig, partial(update_line, lidar=lidar, line=line),
    #frames=np.linspace(0, 2*np.pi, 128), blit=True)
    # for i in range(10):
    #     with open('data'+str(i)+'.pkl', 'wb') as file:
    #         pickle.dump(lidar.lastMap, file)
    #         time.sleep(2)
    
    #lidar.sendQuitReqeust()
 
    
    print("the run is done")

if __name__ == '__main__':
    run()