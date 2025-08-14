from multiprocessing.connection import Connection

from lidarLib.lidarMap import lidarMap

class renderPipeCap:
    """<h2>Class that encapsulates pipe connections between a user and the render engine.</h2>"""
    def __init__(self, pipe:Connection):
        """Creates a render pipe cap surrounding the pipe input"""
        self.pipe=pipe
        self.mostRecentVal:lidarMap=None # type: ignore


    def _get(self)->lidarMap:
        """
            <h2>Returns the most recent data sent over the pipe, since the render machine never sends data this function should never be used by the user.</h2>
        """
        while self.pipe.poll():
        
            #print("data received")
            temp = self.pipe.recv()
                
            if temp.__class__ !=ping:    
                self.mostRecentVal = temp
        #print("data updated", self.mostRecentVal.mapID)
        return self.mostRecentVal

        
    def send(self, sendable:lidarMap)->None:
        """<h2>Sends the imputed lidar map to the other side of the pipe(aka the render machine).</h2>"""
        
        self.pipe.send(sendable)

    def isConnected(self)->bool:
        """<h2>Returns wether or not the pipe has a valid connection.</h2>"""
        try:
            self.pipe.send(ping())
            return True
        except EOFError:
            return False
    
    def close(self):
        self.pipe.close()


class ping:
    """<h2>Class that can be sent back and forth to make sure the pipeline is alive but does not cause any actions to be preformed.</h2>"""
    pass