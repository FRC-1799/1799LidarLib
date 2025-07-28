#!/usr/bin/env python3
'''Animates distances and measurement quality'''
import math
from lidarLib.lidarMeasurement import lidarMeasurement
from lidarLib.Lidar import Lidar
import time
from lidarLib.translation import translation

# Update the port name to match macOS's serial device naming
PORT_NAME = "/dev/LidarFL"  # Replace with the actual device name
DMAX = 1600
IMIN = 20
IMAX = 20
SIZEOFMAPITEM = 0.3
MAP_SIZE = 500  # Size of the map for both x and y directions


def run():
    lidar = Lidar(debugMode=True, deadband=None)
    # Update the port name for macOS
    lidar.connect(port=PORT_NAME, baudrate=256000, timeout=3)
    lidar.setMotorPwm(500)
    
    lidar.getScanModes()
    print(lidar.getSampleRate())
    print(lidar.getScanModeTypical())
    lidar.startScan()
    time.sleep(2)

    try:
        while True:
            printPointLocations(lidar)
            time.sleep(0.1)
    except Exception as e:
        print(e)
    finally:
        lidar.stop()
        lidar.disconnect()
        print("The run is done")


def printPointLocations(lidar: Lidar):
    # Iterate through the points from the lidar map
    for point in lidar.__lastMap.getPoints():
        cart_vals = point.getCart()  # Get Cartesian coordinates (x, y)
        x, y = cart_vals

        # Print the (x, y) locations of points
        print(f"x: {x:.2f}, y: {y:.2f}")


if __name__ == '__main__':
    run()
