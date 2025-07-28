
'''Animates distances and measurment quality'''

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
import lidarHitboxingMap


PORT_NAME = '/dev/ttyUSB0'
DMAX = 1600
IMIN = 20
IMAX = 20



def run():
    l0Process, lidar = lidarManager.makePipedLidar(False, None, translation.fromCart(5000,5000,0))
    lidar.connectSmart(port="/dev/lidar0", baudrate=256000, timeout=3, pwm=500)
    
    

    # axis = subplot.scatter([0, 1], [100, 2000], s=1, c=[IMIN, IMAX],
    #                        cmap=plot.cm.Greys_r, lw=0)
    
    #lidar.setCurrentLocalTranslation(translation(0,0, 180))
    time.sleep(1)
    #lidar.currentMap.printMap()
    #print(lidar.currentMap.points)
    #lidar.currentMap.thisFuncDoesNothing()
    cartRenderer, cartPipe = initMachine(1)
    polarRenderer, polarPipe = initMachine(0)
    
    
    
    while cartPipe.isConnected() and polarPipe.isConnected():
        newMap=lidarHitboxingMap.lidarHitboxMap()
        newMap.addMap(lidar.getMap())
        cartPipe.send(newMap)
        polarPipe.send(lidar.getMap())
        time.sleep(0.1)
            #print("data sent", lidar.lastMap.mapID)

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