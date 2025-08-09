from lidarLib.Lidar import Lidar
import matplotlib.pyplot as plot
import numpy as np
import matplotlib.animation as animation
import time

arr=[[1,2],[3,4],[5,6]]

def testFunc(num, line):
    global newarr
    line.set_offsets(newarr)
    return line



fig = plot.figure()
axis = plot.subplot(111, projection='polar')
line = axis.scatter([0, 0], [0, 0], s=5, c=[0, 20],
                        cmap=plot.cm.Greys_r, lw=0)
axis.set_rmax(4000)
axis.grid(True)

newarr=([(subArr[0],subArr[1]) for subArr in arr])
print(newarr)

anim=animation.FuncAnimation(fig, testFunc,
        fargs=(line), interval=50, save_count=50)
plot.show()

print("runs over")