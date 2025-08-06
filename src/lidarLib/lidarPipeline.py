from multiprocessing import Pipe, Process
from multiprocessing.connection import Connection
from time import time

from lidarLib.lidarMap import lidarMap
from lidarLib.Lidar import Lidar
from enum import Enum

from lidarLib.lidarProtocol import RPlidarDeviceInfo, RPlidarHealth, RPlidarSampleRate, RPlidarScanMode
from lidarLib.translation import translation


class lidarPipeline:
    """
        Class that encapsulates pipe connections between a user and a lidar manager
        This class has all of the user side functions present in a standard lidar object and can therefor be used mostly interchangeably.
        However be aware that due to the very different internals of the two classes they may behave differently in certain circumstances. 
    """
    def __init__(self, pipe:Connection, host:Process=None):
        """Creates a render pipe cap surrounding the pipe input"""
        self.__pipe=pipe
        self.__dataPackets = []
        self.host=host


        for type in dataPacketType.options:
            print(type)
            self.__dataPackets.append(None)

        self.shouldLive=True

        self.__commandQue:list[commandPacket] = []

    def __get(self)->lidarMap:
        """
            INTERNAL FUNCTION, NOT FOR OUTSIDE USE
            returns the most recent data sent over the pipe, since the render machine never sends data this function should never be used by the user
        """
            
        if self.isConnected():    
            while self.__pipe.poll():
                try:
                    mostRecentVal = self.__pipe.recv()
                    if (mostRecentVal.__class__ == dataPacket):
                        self.__dataPackets[mostRecentVal.type] = mostRecentVal.data

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
            Closes the connecting pipe.
            WARNING. This function does not properly shut down anything. Please use sendQuitRequest instead.
        """
        self.__pipe.close()


    def _getNextAction(self)->"commandPacket":
        """
            Returns the next action packet in the que sent by the other lidar pipe and removes said action from the que.
            This function should only be called on the lidar side of the pipe as the lidar will never send actions through the que.   
    
        """
        self.__get()
        if (self.__commandQue.len>0):
            return self.__commandQue.pop(0)
        
        return None
    
    def _peakNextAction(self)->"commandPacket":
        """
            Returns the next action packet in the que sent by the other lidar pipe but does not remove it from the que.
            This function should only be called on the lidar side of the pipe as the lidar will never send actions through the que.   
        """

        self.__get()
        if (self.__commandQue.len>0):
            return self.__commandQue[0]
        
        return None
    
    def _getActionQue(self)->list["commandPacket"]:
        """
            Returns the entire action que sent from the other end of the lidar pipeline.
            WARNING This function resets the action que and so if actions are not handled properly this may cause bugs
            This function should only be called on the lidar side of the pipe as the lidar will never send actions through the que.
        """
        self.__get()
        temp = self.__commandQue
        self.__commandQue=[]
        return temp
    
    def _peakActionQue(self)->list["commandPacket"]:
        """
            Returns the entire action que sent from the other end of the lidar pipeline.
            This function should only be called on the lidar side of the pipe as the lidar will never send actions through the que.
        """
        self.__get()
        return self.__commandQue
    
    def getAllPackets(self)->list["dataPacket"]:
        """Gets the list of the most recently received data packets received by the pipe."""
        self.__get()
        return self.__dataPackets
    
    def getDataPacket(self, type:int)->"dataPacket":
        """Returns the most recently received data packet from by the pipe using the index given. """
        self.__get()
        return self.__dataPackets[type]
    

    def _sendMap(self, map:lidarMap)->None:
        """
            Sends the given map to the other side of the pipe.
            This function should only be called on the lidar side of the pipe as the lidar will not read anything sent to it through this path.
        """

        self._sendData(dataPacket(dataPacketType.lidarMap, map))

    def _sendTrans(self, translation:translation)->None:
        """
            Sends the given Translation to the other side of the pipe.
            WARNING. This function is used internally by the lidar to return the combined translation information.
            To send a Translation to the lidar use the setCurrentLocalTranslation or setCurrentGlobalTranslation functions.
        """

        self._sendData(dataPacket(dataPacketType.translation, translation))

    def _sendSampleRate(self, sampleRate:RPlidarSampleRate)->None:
        """
            Sends te given Sample rate to the other side of the pipe.
            This function should only be called on the lidar side of the pipe as the lidar will not read anything sent to it through this path.
        """

        self._sendData(dataPacket(dataPacketType.sampleRate, sampleRate))

    def _sendScanTypes(self, scanTypes:list[RPlidarScanMode])->None:
        """
            Sends the given scan types to the other side of the pipe.
            This function should only be called on the lidar side of the pipe as the lidar will not read anything sent to it through this path.
        """

        self._sendData(dataPacket(dataPacketType.scanModes, scanTypes))

    def _sendLidarInfo(self, lidarInfo:RPlidarDeviceInfo)->None:
        """
            Sends the given lidar info to the other side of the pipe.
            This function should only be called on the lidar side of the pipe as the lidar will not read anything sent to it through this path.
        """
                
        self._sendData(dataPacket(dataPacketType.lidarInfo, lidarInfo))

    def _sendLidarHealth(self, lidarHealth:RPlidarHealth)->None:
        """
            Sends the given lidar health to the other side of the pipe.
            This function should only be called on the lidar side of the pipe as the lidar will not read anything sent to it through this path.
        """

        self._sendData(dataPacket(dataPacketType.lidarHealth, lidarHealth))

    def _sendScanModeTypical(self, scanMode:RPlidarScanMode)->None:
        """
            Sends the given scan mode to the other side of the pipe.
            This function should only be called on the lidar side of the pipe as the lidar will not read anything sent to it through this path.
        """
                
        self._sendData(dataPacket(dataPacketType.scanModeTypical, scanMode))

    def _sendScanModeCount(self, scanModeCount:int)->None:
        """
            Sends the given scan mode count to the other side of the pipe.
            This function should only be called on the lidar side of the pipe as the lidar will not read anything sent to it through this path.
        """

        self._sendData(dataPacket(dataPacketType.scanModeCount, scanModeCount))


    def _sendData(self, data:"dataPacket")->None:
        """
            Sends the given data packet to the other side of the lidar.
        """
        if (data.__class__!= dataPacket or data.type not in dataPacketType.options):
            raise ValueError("Attempted to send data over a lidar pipeline with an invalid data type packet")
        self.__pipe.send(data)

    def _sendAction(self, action:"commandPacket")->None:
        if action.__class__ != commandPacket:
            raise ValueError("Attempted to send action through the lidar pipeline that wasn't an action")
        self.__pipe.send(action)
    


    def sendQuitRequest(self)->None:
        """
            Sends a package that will kill the lidar remotely.
            If this package is sent to the side of the pipeline not managing any lidar it will not do anything.
        """
        self.__pipe.send(quitPacket())

    def isConnected(self)->bool:
        """Returns wether or not the other side of the pipe is still open. 
            This function does not check if code is still working on the other side of the pipe. Just that the other side of the pipe is open.
        """
        try:
            self.__pipe.send(ping())
        except EOFError as e:
            print("pipe closure detected")
            return False

        return True


    def isRunning(self)->bool:
        """
            Returns wether or not the lidar is currently scanning.
            Do to the nature of piped lidar this function will guess wether or not the lidar is currently scanning based on received data and therefor may have a small delay. 
            This function should only be called on the management side of the pipeline. 
            Because it tries to interact with a lidar object on the other side of the pipeline it will do nothing if called on the side with the lidar. 
            
            Due to the nature of piped lidar this information may be slightly out of date as data is only refreshed so often.
            However it should normally be accurate to 20ms or less.
        """

        return self.getLastMap().endTime + 10/self.getLastMap().getHz > time()

    def disconnect(self, leaveRunning=False)->None:
        """
            Disconnects the lidar from a connected port
            Before disconnecting the function will stop the lidar motor and scan(if applicable) unless leaveRunning is set to true
        """

        self._sendAction(commandPacket(Lidar.disconnect,[leaveRunning]))

    def stop(self)->None:
        """
            Stops the current scan cycle on the lidar but does not stop the motor.
            Due to the nature of a piped lidar this action may take a small amount of time to execute as it is sent and processed however it should normally only take 20 ms or less. 
        """

        self._sendAction(commandPacket(Lidar.stop,[]))
    
    def reset(self)->None:
        """
            Restarts the lidar as if it was just powered on but does not effect the client side lidar lib at all. 
            Due to the nature of a piped lidar this action may take a small amount of time to execute as it is sent and processed however it should normally only take 20 ms or less. 
        """

        self._sendAction(commandPacket(Lidar.reset,[]))
    
    def setMotorPwm(self, pwm:int, overrideInternalValue=True)->None:
        """
            Sets the lidar's motor to the specified pwm value. the speed must be a positive number or 0 and lower or equal to the specified max value(currently 1023).
            Due to the nature of a piped lidar this action may take a small amount of time to execute as it is sent and processed however it should normally only take 20 ms or less. 
            
        """

        self._sendAction(commandPacket(Lidar.setMotorPwm,[pwm, overrideInternalValue]))

    def connect(self)->None:
        """
            Connects to a lidar object with the information specified in the config file.
            Due to the nature of a piped lidar this action may take a small amount of time to execute as it is sent and processed however it should normally only take 20 ms or less. 
        """

        self._sendAction(commandPacket(Lidar.connect,[]))

    def getLastMap(self)->lidarMap:
        """
            Returns the last full map measured by the lidar.
            Due to the nature of piped lidar this information may be slightly out of date as data is only refreshed so often.
            However it should normally be accurate to 20ms or less.
        """

        return self.getDataPacket(dataPacketType.lidarMap)


    def startScan(self)->None:
        """
            Starts a standard scan on the lidar and starts the update cycle.
            Due to the nature of a piped lidar this action may take a small amount of time to execute as it is sent and processed however it should normally only take 20 ms or less. 
        """

        self._sendAction(commandPacket(Lidar.startScan, []))

    def startScanExpress(self, mode:int="auto")->None:
        """
            Starts a scan in express mode (using a compression format so that more samples may be handled per second).
            If a mode is specified then the lidar will attempt to start express in given mode. 
            If mode is set to \"auto\" or not set at all the lidar will instead start and express scan in the recommended mode for the given model.
            WARNING. The lidar lib does not check that a specified express mode is supported by the connected lidar. This must be done by the user.
        
            Due to the nature of a piped lidar this action may take a small amount of time to execute as it is sent and processed however it should normally only take 20 ms or less. 
        """
        self._sendAction(commandPacket(Lidar.startScanExpress, [mode]))

    def startForceScan(self)->None:
        """
            Initializes a force scan. This scan will always be run by the lidar no matter its current state. 
            Since force scans use the same return packets as normal scans it may appear that the lidarlib initialized a normal scan and not a force scan but all data will be handled properly.
        
            Due to the nature of a piped lidar this action may take a small amount of time to execute as it is sent and processed however it should normally only take 20 ms or less. 

        """
        self._sendAction(commandPacket(Lidar.startForceScan, []))
        
    def setCurrentLocalTranslation(self, translation:translation)->None:
        """
            Sets the lidar's objects local translation. This translation should be used for the translation from the lidar to the center of the robot. 
            The translation will be added to all future points read by the lidar(until changed) but will not be added to old retroactively.
            Due to the nature of a piped lidar this action may take a small amount of time to execute as it is sent and processed however it should normally only take 20 ms or less.             
        """
        self._sendAction(commandPacket(Lidar.setCurrentLocalTranslation, [translation]))

    def setCurrentGlobalTranslation(self, translation:translation):
        """
            Sets the lidar's objects global translation. This translation should be used for the translation between the robot and the 0,0 of the field 
            The translation will be added to all future points read by the lidar(until changed) but will not be added to old retroactively.
            Due to the nature of a piped lidar this action may take a small amount of time to execute as it is sent and processed however it should normally only take 20 ms or less. 
        
        """
        self._sendAction(commandPacket(Lidar.setCurrentGlobalTranslation, [translation]))

    def setDeadband(self, deadband:list[int])->None:
        """
            Sets a deadband of angles that will be dropped by the lidar. The dropped angle is calculated after the local translation but before the global translation. 
            The imputed argument should be a list of ints in which the first argument is the start of the deadband and the second is the end. 
            If the second argument is larger than the first the deadband will be assumed to wrap past 360 degrees. 
            Due to the nature of a piped lidar this action may take a small amount of time to execute as it is sent and processed however it should normally only take 20 ms or less. 
        """
        self._sendAction(commandPacket(Lidar.setDeadband, [deadband]))

    def getCombinedTranslation(self)->translation:
        """
            Returns the current combined translation of the lidar. AKA a translation that Incorporates both the local and global translations. 
            Due to the nature of piped lidar this information may be slightly out of date as data is only refreshed so often.
            However it should normally be accurate to 20ms or less.
        """
        return self.__dataPackets[dataPacketType.translation]           

    def getInfo(self)->RPlidarDeviceInfo:
        """
            Returns the connected lidar's info in the form of a RPlidarDeviceInfo object.
            Due to technical limitations this function returns data cached when the lidar was most recently connected.
            While this will not normally cause issues it is something to be aware of.
        """

        return self.getDataPacket(dataPacketType.lidarInfo)

    def getHealth(self)->RPlidarHealth:
        """
            Returns the connected lidar's health in the form of a RPlidarDeviceHealth object.
            Due to technical limitations this function returns data cached when the lidar was most recently connected.
            While this will not normally cause issues it is something to be aware of.
        """
        
        return self.getDataPacket(dataPacketType.lidarHealth)
    
    def getSampleRate(self)->RPlidarSampleRate:
        """
            Fetches and returns the connected lidar's sample rates for both standard and express modes. 
            The measurements are in microseconds per reading. the data is returned in the form of a RPlidarSampleRateObject.
            Due to technical limitations this function returns data cached when the lidar was most recently connected.
            While this will not normally cause issues it is something to be aware of.
        """

        return self.getDataPacket(dataPacketType.sampleRate)

    def getScanModeTypical(self)->int:
        """
            Returns the best scan mode for the connected lidar.
            Due to technical limitations this function returns data cached when the lidar was most recently connected.
            While this will not normally cause issues it is something to be aware of.
        """
        return self.getDataPacket(dataPacketType.scanModeTypical)

    def getScanModeCount(self)->int:
        """
            Returns the number of scan modes supported by the connected lidar.
            Due to technical limitations this function returns data cached when the lidar was most recently connected.
            While this will not normally cause issues it is something to be aware of.
        """

        return self.getDataPacket(dataPacketType.scanModeCount)
    
    def getScanModes(self)->list[RPlidarScanMode]:
        """
            Returns a list of RPlidarScanMode objects for each scan mode supported by the current connected lidar.
            Due to technical limitations this function returns data cached when the lidar was most recently connected.
            While this will not normally cause issues it is something to be aware of.
            WARNING: some of the modes returned may be supported by the lidar but not supported by the client side lib.
        """

        return self.getDataPacket(dataPacketType.scanModes)
        


class commandPacket:
    def __init__(self, function:callable, args:list, returnType:int=-1):
        self.function = function
        self.args = args
        if returnType not in dataPacketType.options and returnType!=-1:
            raise ValueError("attempted to make a command packet with a return type that does not exist")

        self.returnType=returnType

class dataPacketType:

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
    def __init__(self, type:int, data):
        if (type not in dataPacketType.options):
            raise ValueError("Tried to create a data packet with an invalid data type")
         
        self.type = type
        self.data=data

class quitPacket:
    pass

class ping:
    pass
