from multiprocessing.connection import Connection

from lidarLib import lidarMap

class renderPipeCap:
    """class that encapsulates pipe connections between a user and the render engine"""
    def __init__(self, pipe:Connection):
        """Creates a render pipe cap surrounding the pipe input"""
        self.pipe=pipe
        self.mostRecentVal=None


    def _get(self)->lidarMap:
        """
            INTERNAL FUNCTION, NOT FOR OUTSIDE USE
            returns the most recent data sent over the pipe, since the render machine never sends data this function should never be used by the user
        """
        while self.pipe.poll():
        
            #print("data received")
            temp = self.pipe.recv()
                
            if temp.__class__ !=ping:    
                self.mostRecentVal = temp
        #print("data updated", self.mostRecentVal.mapID)
        return self.mostRecentVal

        
    def send(self, sendable:lidarMap)->None:
        """Sends the imputed lidar map to the other side of the pipe(aka the render machine)"""
        
        self.pipe.send(sendable)

    def isConnected(self)->bool:
        try:
            self.pipe.send(ping())
            return True
        except EOFError:
            return False
    
    def close(self):
        self.pipe.close()


class ping:
    pass