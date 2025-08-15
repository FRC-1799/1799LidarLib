from multiprocessing import Process
from multiprocessing.connection import Connection
from time import time
from typing import Any, Callable

from lidarLib.lidarMap import lidarMap
from lidarLib.Lidar import Lidar

from lidarLib.lidarProtocol import LidarDeviceInfo, LidarHealth, LidarSampleRate, LidarScanMode
from lidarLib.translation import translation


class lidarPipeline:
    """
        <h2>Class that encapsulates pipe connections between a user and a lidar manager.</h2>
        This class has all of the user side functions present in a standard lidar object and can therefor be used mostly interchangeably.
        However be aware that due to the very different internals of the two classes they may behave differently in certain circumstances. 
    """
    def __init__(self, pipe:Connection, host:Process=None): # type: ignore
        """<h2>Creates a pipeline to connect to a lidar.</h2>"""
        self.__pipe=pipe
        self.__dataPackets:list[dataPacket] = []
        self.host=host


        for dataType in dataPacketType.options: # type: ignore
            self.__dataPackets.append(None) # type: ignore

        self.shouldLive=True

        self.__commandQue:list[commandPacket] = []

    def __get(self)->None:
        """
            <h2>Reads all the data from the pipeline into the various data buffers.</h2>
        """
            
        if self.isConnected():    
            while self.__pipe.poll():
                try:
                    mostRecentVal = self.__pipe.recv()
                    if (mostRecentVal.__class__ == dataPacket):
                        self.__dataPackets[mostRecentVal.type] = mostRecentVal

                    elif (mostRecentVal.__class__ == commandPacket):
                        self.__commandQue.append(mostRecentVal)

                    elif(mostRecentVal.__class__ == quitPacket):
                        self.shouldLive=False
                    
                    elif(mostRecentVal.__class__ == ping):
                        pass

                    else:
                        raise ValueError("attempted to send a value over the lidar pipeline that was not of type commandPacket or dataPacket")
                
                except EOFError:
                    print("lidar pipeline missing connection")
                    return 
        else:
            print("lidar pipeline missing connection")


    def close(self)->None:
        """
            <h2>Closes the connecting pipe.</h2>
            WARNING. This function does not properly shut down anything. Please use sendQuitRequest instead.
        """
        self.__pipe.close()


    def _getNextAction(self)->"commandPacket":
        """
            <h2>Returns the next action packet in the que sent by the other lidar pipe and removes said action from the que or none if no action is qued.</h2>
            This function should only be called on the lidar side of the pipe as the lidar will never send actions through the que.   
            If you would like to get the next action without removing it from the que use peakNextAction Instead.
        """
        self.__get()
        if (len(self.__commandQue)>0):
            return self.__commandQue.pop(0)
        
        return None # type: ignore
    
    def _peakNextAction(self)->"commandPacket":
        """
            <h2>Returns the next action packet in the que sent by the other lidar pipe but does not remove it from the que or none if no action is qued.</h2>
            This function should only be called on the lidar side of the pipe as the lidar will never send actions through the que.   
        """

        self.__get()
        if (len(self.__commandQue)>0):
            return self.__commandQue[0]
        
        return None# type:ignore
    
    def _getActionQue(self)->list["commandPacket"]:
        """
            <h2>Returns the entire action que sent from the other end of the lidar pipeline.</h2>
            WARNING This function resets the action que and so if actions are not handled properly this may cause bugs.
            If you would like to get the que without resetting it use peakActionQue instead.
            This function should only be called on the lidar side of the pipe as the lidar will never send actions through the que.
        """
        self.__get()
        temp = self.__commandQue
        self.__commandQue=[]
        return temp
    
    def _peakActionQue(self)->list["commandPacket"]:
        """
            <h2>Returns the entire action que sent from the other end of the lidar pipeline.</h2>
            This function should only be called on the lidar side of the pipe as the lidar will never send actions through the que.
        """
        self.__get()
        return self.__commandQue
    
    def getAllPackets(self)->list["dataPacket"]:
        """<h2>Gets the list of the most recently received data packets received by the pipe.</h2>"""
        self.__get()
        return self.__dataPackets
    
    def getDataPacket(self, type:int)->"dataPacket":
        """<h2>Returns the most recently received data packet from by the pipe using the index given.</h2>"""
        self.__get()
        return self.__dataPackets[type]
    
    def getData(self, type:int)->Any:
        data:dataPacket = self.getDataPacket(type)
        if data:
            return data.data
        return None

    def _sendMap(self, map:lidarMap)->None:
        """
            <h2>Sends the given map to the other side of the pipe.</h2>
            This function should only be called on the lidar side of the pipe as the lidar will not read anything sent to it through this path.
        """

        self._sendData(dataPacket(dataPacketType.lidarMap, map))

    def _sendTrans(self, translation:translation)->None:
        """
            <h2>Sends the given Translation to the other side of the pipe.</h2>
            WARNING. This function is used internally by the lidar to return the combined translation information.
            To send a Translation to the lidar use the setCurrentLocalTranslation or setCurrentGlobalTranslation functions.
        """

        self._sendData(dataPacket(dataPacketType.translation, translation))

    def _sendSampleRate(self, sampleRate:LidarSampleRate)->None:
        """
            <h2>Sends te given Sample rate to the other side of the pipe.</h2>
            This function should only be called on the lidar side of the pipe as the lidar will not read anything sent to it through this path.
        """

        self._sendData(dataPacket(dataPacketType.sampleRate, sampleRate))

    def _sendScanTypes(self, scanTypes:list[LidarScanMode])->None:
        """
            <h2>Sends the given scan types to the other side of the pipe.</h2>
            This function should only be called on the lidar side of the pipe as the lidar will not read anything sent to it through this path.
        """

        self._sendData(dataPacket(dataPacketType.scanModes, scanTypes))

    def _sendLidarInfo(self, lidarInfo:LidarDeviceInfo)->None:
        """
            <h2>Sends the given lidar info to the other side of the pipe.</h2>
            This function should only be called on the lidar side of the pipe as the lidar will not read anything sent to it through this path.
        """
                
        self._sendData(dataPacket(dataPacketType.lidarInfo, lidarInfo))

    def _sendLidarHealth(self, lidarHealth:LidarHealth)->None:
        """
           <h2> Sends the given lidar health to the other side of the pipe.</h2>
            This function should only be called on the lidar side of the pipe as the lidar will not read anything sent to it through this path.
        """

        self._sendData(dataPacket(dataPacketType.lidarHealth, lidarHealth))

    def _sendScanModeTypical(self, scanMode:int)->None:
        """
            <h2>Sends the given scan mode to the other side of the pipe.</h2>
            This function should only be called on the lidar side of the pipe as the lidar will not read anything sent to it through this path.
        """
                
        self._sendData(dataPacket(dataPacketType.scanModeTypical, scanMode))

    def _sendScanModeCount(self, scanModeCount:int)->None:
        """
            <h2>Sends the given scan mode count to the other side of the pipe.</h2>
            This function should only be called on the lidar side of the pipe as the lidar will not read anything sent to it through this path.
        """

        self._sendData(dataPacket(dataPacketType.scanModeCount, scanModeCount))


    def _sendData(self, data:"dataPacket")->None:
        """<h2>Sends the given data packet to the other side of the pipe.</h2>"""
        if (data.__class__!= dataPacket or data.type not in dataPacketType.options):
            raise ValueError("Attempted to send data over a lidar pipeline with an invalid data type packet")
        self.__pipe.send(data)

    def _sendAction(self, action:"commandPacket")->None:
        """<h2>Sends the given action packet to the other side of the pipe</h2>"""
        if action.__class__ != commandPacket:
            raise ValueError("Attempted to send action through the lidar pipeline that wasn't an action")
        self.__pipe.send(action)
    


    def sendQuitRequest(self)->None:
        """
            <h2>Sends a package that will stop the lidar and shut down the pipeline.</h2>
            This function should be the last one called on the pipeline as the pipeline will shut down shortly after this call.
        """
        self.__pipe.send(quitPacket())

    def isConnected(self)->bool:
        """
            <h2>Returns wether or not the other side of the pipe is still open. </h2>
            This function does not check if code is still working on the other side of the pipe. Just that the other side of the pipe is open.
        """
        try:
            self.__pipe.send(ping())
        except: 
            # print("pipe closure detected\n\n\n\n\n\n\n\n\n")
            return False

        return True


    def isRunning(self)->bool:
        """
            <h2>Returns wether or not the lidar is currently scanning.</h2>
            Do to the nature of piped lidar this function will guess wether or not the lidar is currently scanning based on received data and therefor may not be perfectly accurate and should not be relied on without some cleaning. 
            Due to the nature of piped lidar this information may be slightly out of date as data is only refreshed so often.
            However it should normally be accurate to 20ms or less.
        """

        return self.getLastMap().endTime + 10/self.getLastMap().getHz() > time()

    def disconnect(self, leaveRunning:bool=False)->None:
        """
            <h2>Disconnects the lidar from a connected port.</h2>
            Before disconnecting the function will stop the lidar motor and scan(if applicable) unless leaveRunning is set to true.\n
            This function does not shut down the pipeline, only the lidar on the other side of the pipeline. If you would like to fully stop the pipeline use sendQuitRequest.
        """

        self._sendAction(commandPacket(Lidar.disconnect,[leaveRunning]))

    def stop(self)->None:
        """
            <h2>Stops the current scan cycle on the lidar but does not stop the motor.</h2>
            Due to the nature of a piped lidar this action may take a small amount of time to execute as it is sent and processed however it should normally only take 20 ms or less. 
        """

        self._sendAction(commandPacket(Lidar.stop,[]))
    
    def reset(self)->None:
        """
            <h2>Restarts the lidar as if it was just powered on but does not effect the client side lidar lib at all.</h2>
            Due to the nature of a piped lidar this action may take a small amount of time to execute as it is sent and processed however it should normally only take 20 ms or less. 
        """

        self._sendAction(commandPacket(Lidar.reset,[]))
    
    def setMotorPwm(self, pwm:int, overrideInternalValue:bool=True)->None:
        """
            <h2>Sets the lidar's motor to the specified pwm value. </h2>
            The speed must be a positive number or 0 and lower or equal to the specified max value(currently 1023). \n
            Due to the nature of a piped lidar this action may take a small amount of time to execute as it is sent and processed however it should normally only take 20 ms or less. 
            
        """

        self._sendAction(commandPacket(Lidar.setMotorPwm,[pwm, overrideInternalValue]))

    def connect(self)->None:
        """
            <h2>Connects to a lidar object with the information specified in the config file.</h2>
            Due to the nature of a piped lidar this action may take a small amount of time to execute as it is sent and processed however it should normally only take 20 ms or less. 
        """

        self._sendAction(commandPacket(Lidar.connect,[]))

    def getLastMap(self)->lidarMap:
        """
            <h2>Returns the last full map measured by the lidar.</h2>
            Due to the nature of piped lidar this information may be slightly out of date as data is only refreshed so often. However it should normally be accurate to 20ms or less.
        """
    
        return self.getData(dataPacketType.lidarMap)


    def startScan(self)->None:
        """
            <h2>Starts a standard scan on the lidar and starts the update cycle.</h2>
            Due to the nature of a piped lidar this action may take a small amount of time to execute as it is sent and processed however it should normally only take 20 ms or less. 
        """

        self._sendAction(commandPacket(Lidar.startScan, []))

    def startScanExpress(self, mode:int="auto")->None: # type: ignore
        """
            <h2>Starts a scan in express mode (using a compression format so that more samples may be handled per second).</h2>
            If a mode is specified then the lidar will attempt to start express in given mode. 
            If mode is set to \"auto\" or not set at all the lidar will instead start and express scan in the recommended mode for the given model.
            WARNING. The lidar lib does not check that a specified express mode is supported by the connected lidar. This must be done by the user.\n

            Due to the nature of a piped lidar this action may take a small amount of time to execute as it is sent and processed however it should normally only take 20 ms or less. 
        """
        self._sendAction(commandPacket(Lidar.startScanExpress, [mode]))

    def startForceScan(self)->None:
        """
            <h2>Initializes a force scan. This scan will always be run by the lidar no matter its current state.</h2>
            Since force scans use the same return packets as normal scans it may appear that the lidarlib initialized a normal scan and not a force scan but all data will be handled properly.\n
        
            Due to the nature of a piped lidar this action may take a small amount of time to execute as it is sent and processed however it should normally only take 20 ms or less. 

        """
        self._sendAction(commandPacket(Lidar.startForceScan, []))
        
    def setCurrentLocalTranslation(self, translation:translation)->None:
        """
            <h2>Sets the lidar's objects local translation.</h2>
            This translation should be used for the translation from the lidar to the center of the robot. 
            The translation will be added to all future points read by the lidar(until changed) but will not be added to old retroactively.\n
            Due to the nature of a piped lidar this action may take a small amount of time to execute as it is sent and processed however it should normally only take 20 ms or less.             
        """
        self._sendAction(commandPacket(Lidar.setCurrentLocalTranslation, [translation]))

    def setCurrentGlobalTranslation(self, translation:translation):
        """
            <h2>Sets the lidar's objects global translation. </h2>
            This translation should be used for the translation between the robot and the 0,0 of the field 
            The translation will be added to all future points read by the lidar(until changed) but will not be added to old retroactively.\n
            Due to the nature of a piped lidar this action may take a small amount of time to execute as it is sent and processed however it should normally only take 20 ms or less. 
        
        """
        self._sendAction(commandPacket(Lidar.setCurrentGlobalTranslation, [translation]))

    def setDeadband(self, deadband:list[int])->None:
        """
            <h2>Sets a deadband of angles that will be dropped by the lidar. </h2>
            The dropped angle is calculated after the local translation but before the global translation. 
            The imputed argument should be a list of ints in which the first argument is the start of the deadband and the second is the end. 
            If the second argument is larger than the first the deadband will be assumed to wrap past 360 degrees.\n 
            Due to the nature of a piped lidar this action may take a small amount of time to execute as it is sent and processed however it should normally only take 20 ms or less. 
        """
        self._sendAction(commandPacket(Lidar.setDeadband, [deadband]))

    def getCombinedTranslation(self)->translation:
        """
            <h2>Returns the current combined translation of the lidar. AKA a translation that Incorporates both the local and global translations.</h2>
            Due to the nature of piped lidar this information may be slightly out of date as data is only refreshed so often. However it should normally be accurate to 20ms or less.
        """
        return self.getData(dataPacketType.translation)           

    def getInfo(self)->LidarDeviceInfo:
        """
            <h2>Returns the connected lidar's info in the form of a RPlidarDeviceInfo object.</h2>
            Due to technical limitations this function returns data cached when the lidar was most recently connected.
            While this will not normally cause issues it is something to be aware of.
        """

        return self.getData(dataPacketType.lidarInfo)

    def getHealth(self)->LidarHealth:
        """
            <h2>Returns the connected lidar's health in the form of a RPlidarDeviceHealth object.</h2>
            Due to technical limitations this function returns data cached when the lidar was most recently connected.
            While this will not normally cause issues it is something to be aware of.
        """
        
        return self.getData(dataPacketType.lidarHealth)
    
    def getSampleRate(self)->LidarSampleRate:
        """
            <h2>Fetches and returns the connected lidar's sample rates for both standard and express modes. </h2>
            The measurements are in microseconds per reading. the data is returned in the form of a RPlidarSampleRateObject.
            Due to technical limitations this function returns data cached when the lidar was most recently connected.
            While this will not normally cause issues it is something to be aware of.
        """

        return self.getData(dataPacketType.sampleRate)

    def getScanModeTypical(self)->int:
        """
            <h2>Returns the best scan mode for the connected lidar.</h2>
            Due to technical limitations this function returns data cached when the lidar was most recently connected.
            While this will not normally cause issues it is something to be aware of.
        """
        
        return self.getData(dataPacketType.scanModeTypical)

    def getScanModeCount(self)->int:
        """
            <h2>Returns the number of scan modes supported by the connected lidar.</h2>
            Due to technical limitations this function returns data cached when the lidar was most recently connected.
            While this will not normally cause issues it is something to be aware of.
        """

        return self.getData(dataPacketType.scanModeCount)
    
    def getScanModes(self)->list[LidarScanMode]:
        """
            <h2>Returns a list of RPlidarScanMode objects representing each scan mode supported by the current connected lidar. </h2>
            Due to technical limitations this function returns data cached when the lidar was most recently connected.
            While this will not normally cause issues it is something to be aware of.\n
            WARNING: some of the modes returned may be supported by the lidar but not supported by the client side lib.
        """

        return self.getData(dataPacketType.scanModes)
        


class commandPacket:
    """<h2>A packet to send functions over a lidar pipeline.</h2>"""
    def __init__(self, function:Callable, args:list[Any], returnType:int=-1): # type: ignore
        """
            <h2>Creates a command packet.</h2>
            Function should be function from the lidar class that will be called on the lidar on the other side pipe. \n 
            Args should be a list of positional arguments to be used in the function call. They must be in order. \n
            Return type may be used to define a return type for the function. 
            If set the lidar pipeline will attempt to pass back the result of the function under the return type specified. 
            returnType should be an integer code corresponding to one of the values in the dataPacketType class.
            If set to -1 or left to default the functions return value will not be passed back through the pipe. 
        """
        self.function:Callable = function # type: ignore
        self.args = args
        if returnType not in dataPacketType.options and returnType!=-1:
            raise ValueError("attempted to make a command packet with a return type that does not exist")

        self.returnType=returnType

class dataPacketType:
    """
        <h2> An enum like class to contain all of the data types used by the dataPacket Class.</h2>
        These integer codes are used by the dataPacket class to determine what kind of data is contained and to sort it.
    """
    lidarMap = 0
    translation = 1
    quitWarning = 2
    sampleRate = 3
    scanModes = 4
    lidarInfo = 5
    lidarHealth = 6
    scanModeTypical=7
    scanModeCount=8
    options:list[int] = [
        lidarMap, translation, quitWarning,
        sampleRate, scanModes, lidarInfo,
        lidarHealth, scanModeTypical, scanModeCount
    ]
    

class dataPacket():
    """<h2> A class to send data across a lidar pipeline </h2>"""
    def __init__(self, type:int, data:Any):
        """
            <h2>Creates a data packet</h2>
            Type should be an integer code from the dataPacketType class. This will be used to keep track of the kind of data contained within. \n
            Data should be a value of the type specified by the above integer code.
        """
        if (type not in dataPacketType.options):
            raise ValueError("Tried to create a data packet with an invalid data type")
         
        self.type = type
        self.data:Any=data

class quitPacket:
    """<h2>A class used by the lidar pipeline to symbolize a quit request </h2>"""
    pass

class ping:
    """
        <h2>A class that can be sent through the lidar pipeline but will be discarded. </h2>
        This is used so that the system can tell if the pipeline is still open, even when no data is being currently sent.
    """
    pass
