import math
from multiprocessing.connection import Connection
import matplotlib.pyplot as plot
from matplotlib.axis import Axis
import matplotlib.animation as animation
import numpy as np
from multiprocessing import Process, Pipe
import lidarHitboxNode
from lidarHitboxingMap import lidarHitboxMap
from renderLib.renderPipeCap import renderPipeCap
from constants import constants

DMAX=4000

def updateLinePolar(num, pipe:renderPipeCap, subplot:plot.Figure)->Axis:
    """
        updates the subplot using data gained from the pipe
        pipe should be the read end of a renderPipe cap thats partner is consistently supplied with up to date lidar maps
        subplot should be a axis value that is tied to a plot being updated by an animation(with this or a extension of this as its active function)
        num is an placeholder argument that is automatically supplied by animation but never used
    """

    subplot.clear()
    scan = pipe._get()
    #print(scan.mapID)
    if scan == None:
        return
    scan=scan.getPoints()
    angles=np.array([point.angle for point in scan])
    distances=np.array([point.distance for point in scan])
    #offsets = np.array([[point.angle, point.distance] for point in scan])
    #offsets=[scan[0].angle, scan[0].distance]
    #subplot.set_offsets(offsets)pass

    intens = np.array([1 for point in scan])
    #subplot.set_array(intens)
    #print("render cycle", len(intens))
    return subplot.scatter(angles*3.14/180, distances, s=10, c=intens, cmap=plot.cm.Greys_r, lw=0),


def polarRenderMachine(pipeCap:renderPipeCap)->None:
    """
        Initializes a render and animation and displays it. 
        For performance reasons this function should be set up on its own process(automatically done in the initMachine function) so its slowness can not effect the data gatherers
        Pipe should be the read end of a renderPipe cap thats partner is consistently supplied with up to date lidar maps
    """
    fig = plot.figure()
    subplot = plot.subplot(111, projection='polar')
    subplot.set_rmax(DMAX)
    subplot.grid(True)
    
    anim=animation.FuncAnimation(fig, updateLinePolar, blit=False,
    fargs=(pipeCap, subplot), interval=50, save_count=50)
    
    plot.show()


def cartRenderMachine(pipeCap:renderPipeCap)->None:
    global updateLineCartHeartBeat
    updateLineCartHeartBeat=0
    fig = plot.figure()
    subplot = plot.subplot(
                            # math.ceil(constants.mapWidthMeters/constants.mapNodeSizeMeters/100),
                            # math.ceil(constants.mapHeightMeters/constants.mapNodeSizeMeters/100),
                            # 1,
                            projection='rectilinear',
        )
    subplot.grid(True)
    

    anim=animation.FuncAnimation(fig, updateLineCart, blit=False,
    fargs=(pipeCap, subplot), interval=50, save_count=50)
    
    plot.show()



def updateLineCart(num, pipeCap:renderPipeCap, subplot:plot.Figure):
    global updateLineCartHeartBeat
    subplot.clear()
    scan:lidarHitboxMap = pipeCap._get()
    #print(scan.mapID)
    if scan == None:
        return
    scan=scan.getAs1DList()
    xVals:list[lidarHitboxNode]=[]
    yVals:list[lidarHitboxNode] = []
    intens:list[float] = []
    
    updateLineCartHeartBeat+=1
    #print( updateLineCartHeartBeat)
    for point in scan:
        if (not point.isOpen):
            xVals.append(point.x/constants.mapNodeSizeMeters)
            yVals.append(point.y/constants.mapNodeSizeMeters)
            intens.append(1)
    #offsets = np.array([[point.angle, point.distance] for point in scan])
    #offsets=[scan[0].angle, scan[0].distance]
    #subplot.set_offsets(offsets)
    
    #subplot.set_array(intens)
    #print("render cycle", len(intens))
    return subplot.scatter(np.array(xVals), np.array(yVals), s=10, c=np.array(intens), cmap=plot.cm.Greys_r, lw=0),
   
    
    


def initMachine(type:int = 0)->tuple[Process, Connection]:
    """
        Creates a separate proses that handles all rendering and can be updated via a pipe(connection)
        returns a tuple with the first argument being the process, this can be use cancel the process but the primary use is to be saved so the renderer doesn't get collected
        the second argument is one end of a pipe that is used to update the render engine. this pipe should be passed new lidar maps periodically so they can be rendered. 
        WARNING all code that deals with the pipe should be surrounded by a try except block as the pipe will start to throw errors whenever the user closes the render machine.
    """
    returnPipe, machinePipe = Pipe(duplex=True)
    returnPipe=renderPipeCap(returnPipe)
    machinePipe=renderPipeCap(machinePipe)
    if type==0:
        process= Process(target=polarRenderMachine, args=(machinePipe,))
    elif type==1:
        process=Process(target=cartRenderMachine, args=(machinePipe,))
    else:
        raise ValueError("tried to create a render machine with type value ", type, ". This type does not exist")
    process.start()


    return process, returnPipe

    