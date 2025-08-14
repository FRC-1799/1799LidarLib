from typing import Any, Tuple
import matplotlib.pyplot as plot
from matplotlib.axes import Axes

import matplotlib.animation as animation
from matplotlib.figure import Figure
import numpy as np
from multiprocessing import Process, Pipe

from lidarLib.renderLib.renderPipeCap import renderPipeCap

DMAX=4000

def updateLinePolar(num:int, pipe:renderPipeCap, subplot:Figure)->Tuple[Any]:
    """
        <h2>Updates the subplot using data gained from the pipe.</h2>
        pipe should be the read end of a renderPipe cap thats partner is consistently supplied with up to date lidar maps.
        subplot should be a axis value that is tied to a plot being updated by an animation(with this or a extension of this as its active function).
        num is an placeholder argument that is automatically supplied by animation but never used
    """

    subplot.clear()
    scan = pipe._get() # type: ignore
    #print(scan.mapID)
    if scan == None: # type: ignore
        return # type: ignore
    scan=scan.getPoints()
    angles=np.array([point.angle for point in scan])
    distances=np.array([point.distance for point in scan])
    #offsets = np.array([[point.angle, point.distance] for point in scan])
    #offsets=[scan[0].angle, scan[0].distance]
    #subplot.set_offsets(offsets)pass

    intens = np.array([1 for point in scan]) # type: ignore
    #subplot.set_array(intens)
    #print("render cycle", len(intens))
    return subplot.scatter(angles*3.14/180, distances, s=10, c=intens, cmap=plot.cm.Greys_r, lw=0), # type: ignore


def polarRenderMachine(pipeCap:renderPipeCap)->None:
    """
        <h2>Initializes a render and animation and displays it. </h2> 
        For performance reasons this function should be set up on its own process(automatically done in the initMachine function) so its slowness can not effect the data gatherers.
        Pipe should be the read end of a renderPipe cap thats partner is consistently supplied with up to date lidar maps.
    """
    fig = plot.figure() # type: ignore
    subplot:Axes = plot.subplot(111, projection='polar') # type: ignore
    subplot.set_rmax(DMAX) # type: ignore
    subplot.grid(True) # type: ignore
    
    anim=animation.FuncAnimation(fig, updateLinePolar, blit=False, # type: ignore
    fargs=(pipeCap, subplot), interval=50, save_count=50)
    
    plot.show() # type: ignore


    


def initMachine(type:int = 0)->tuple[Process, renderPipeCap]:
    """
        <h2>Creates a separate proses that handles all rendering and can be updated via a pipe(connection).</h2>
        Returns a tuple with the first argument being the process, this can be use cancel the process but the primary use is to be saved so the renderer doesn't get collected.
        The second argument is one end of a pipe that is used to update the render engine. this pipe should be passed new lidar maps periodically so they can be rendered. 
        WARNING all code that deals with the pipe should be surrounded by a try except block as the pipe will start to throw errors whenever the user closes the render machine.
    """
    returnPipe, machinePipe = Pipe(duplex=True)
    returnPipe=renderPipeCap(returnPipe)
    machinePipe=renderPipeCap(machinePipe)

    process= Process(target=polarRenderMachine, args=(machinePipe,))

    process.start()


    return process, returnPipe

    