
from time import sleep
import time
from lidarLib.LidarConfigs import lidarConfigs
from lidarLib.rplidarSerial import RPlidarSerial
from lidarLib.lidarProtocol import *
from lidarLib.lidarMap import lidarMap
from lidarLib.lidarMeasurement import lidarMeasurement
import threading
from lidarLib.translation import translation
from typing import Callable, Type




class Lidar:
    """class to handle, read, and translate data from a RPlidar (only A2M12 has been tested but should work for all)"""
    def __init__(self, config:lidarConfigs):
        """initializes lidar object but does not attempt to connect or start any scans"""

        if config.isStop:
            self.isStopFunction()
            return

        self.lidarSerial:RPlidarSerial = None # type: ignore
        self.measurements = None
        self.currentMap=lidarMap(self)
        self.__lastMap=lidarMap(self)
        #self.eventLoop()
        #self.deadband=deadband
        self.config = config
        self.capsuleType:Type[LidarScanCapsule] =None # type: ignore
        self.loop:threading.Thread = None # type: ignore
        self.dataDescriptor=None
        self.isDone:bool=False
        self.currentMotorPWM=0
        #self.debugMode=debugMode

        self.capsulePrev:LidarScanCapsule
        
        self.scanModes:list[LidarScanMode]=[]
        self.typicalScanMode:int=None # type: ignore
        self.lidarInfo:LidarDeviceInfo=None # type: ignore
        self.lidarHealth:LidarHealth=None # type: ignore
        self.sampleRate:LidarSampleRate=None # type: ignore

        self.localTranslation=self.config.localTrans
        self.globalTranslation=translation.default()
        self.combinedTranslation=translation.default()

        if (config.autoConnect):
            self.connect()


        
        

        
        

    
    def __del__(self):
        
        self.disconnect()

    def __enter__(self):
        return self

    def __exit__(self, exceptionType:None, exceptionValue:None, exceptionTraceback:None):
        self.disconnect()


    def isStopFunction(self):
        self.lidarSerial = RPlidarSerial()
        self.lidarSerial.open(self.config.port, self.config.vendorID, self.config.productID, self.config.serialNumber, self.config.baudrate,timeout=self.config.timeout)
        if self.isConnected():
            self.disconnect()


    def connect(self)->None:
        """Connects to a lidar object with the information specified in the config file."""
        self.lidarSerial = RPlidarSerial()
        self.lidarSerial.open(self.config.port, self.config.vendorID, self.config.productID, self.config.serialNumber, self.config.baudrate,timeout=self.config.timeout)
        print(self.config.port)
        print(self.config.baudrate)
        
        if self.lidarSerial.isOpen():
            print("PyRPlidar Info : device is connected")
        else:
            self.readToCapsule=None
            raise ConnectionError("could not find lidar unit")
        
        self.stop()
        time.sleep(1.002)
        self.lidarSerial.flush()
        time.sleep(0.002)

        self.__getHealth()
        self.__getInfo()
        self.__getSampleRate()
        self.__getScanModeCount()
        self.__getScanModes()
        self.__getScanModeTypical()
        
        if self.config.autoStart:
            if self.config.mode=="normal":
                print("starting in normal")
                self.startScan()
            if self.config.mode == "express":
                print("starting in express")
                self.startScanExpress()
        

    def isRunning(self):
        """Returns wether or not the lidar is currently scanning."""
        return self.loop and self.loop.is_alive()

    def __establishLoop(self, updateFunc:Callable[[], None], resetLoop:bool=True)->None:
        """
            INTERNAL FUNCTION, NOT FOR OUTSIDE USE
            Establishes the update loop thread as well as fetching a scan descriptor
            if reset loop is set to false the function will only fetch the descriptor and change the update function but it will not tamper with the update thread
        """
        

        self.__update=updateFunc
        if resetLoop:
            self.loop = threading.Thread(target=self.__updateLoop, daemon=True)
            self.loop.start()

    def disconnect(self, leaveRunning:bool=False)->None:
        """
            Disconnects the lidar from a connected port
            Before disconnecting the function will stop the lidar motor and scan(if applicable) unless leaveRunning is set to true
        """

        if self.lidarSerial is not None: # type: ignore
            self.isDone=True
            if not leaveRunning:
                self.stop()
                self.setMotorPwm(0)
            self.lidarSerial.close()
            
            self.lidarSerial = None # type: ignore
            print("PyRPlidar Info : device is disconnected")

    def __updateLoop(self)->None:
        """
            INTERNAL FUNCTION, NOT FOR OUTSIDE USE
            function called by the update loop so that the actual update function can be changed depending on the scan type run
        """
        while not self.isDone:
            self.__update()
            sleep(0.001)
            
    def __restartScan(self)->None:
        """Restarts the running scan, this function only works for simple scans rn and calling it during any other scan type may cause issues"""
        self.stop()
        #self.setMotorPwm(0)
        
        #sleep(0.001)
        self.lidarSerial.flush()
        
        
        self.__startRawScan()
        


    def __update(self)->None:
        """
            INTERNAL FUNCTION, NOT FOR OUTSIDE USE
            placeholder function, update is revalued whenever a scan is started so this code should never run. for more information see __standardUpdate or __expressUpdate
        """
        raise LidarProtocolError("Update was called without a valid connection established, this may because a user tried to call it")


    def __standardUpdate(self)->None:
        """
            INTERNAL FUNCTION, NOT FOR OUTSIDE USE
            actual update function for standard and force scans. Will read the buffer into new measurements which are passed into the current map
        """
        
        while not self.isDone:
            #print(self.lidarSerial.bufferSize())
            if self.dataDescriptor and (self.lidarSerial.bufferSize()>=self.dataDescriptor.dataLength):
                #print("update working")
                newData=self.__receiveData(self.dataDescriptor)
                if not self.__validatePackage(newData, printErrors=self.config.debugMode):
                    self.__restartScan()
                    return
                self.currentMap.addVal(lidarMeasurement(newData), self.combinedTranslation)
            else:
                #print("break hit")
                break
        
        #print("thingy")
    #@DeprecationWarning
    def __capsuleUpdate(self)->None:
        """
            INTERNAL FUNCTION, NOT FOR OUTSIDE USE
            DEPRECATED
            actual update function for express scans. Will read the buffer into new measurements which are passed into the current map
            since the express scan is currently deprecated in this library this function should never be called
        """

        while not self.capsulePrev:  # type: ignore
            if self.dataDescriptor and (self.lidarSerial.bufferSize()>=self.dataDescriptor.dataLength):

                self.capsulePrev = self.capsuleType(self.__receiveData(self.dataDescriptor))
        
        while not self.isDone:
            #print("update")
            #print(self.dataDescriptor, self.lidarSerial.bufferSize())
            if self.dataDescriptor and (self.lidarSerial.bufferSize()>=self.dataDescriptor.dataLength):
                #print("data read")
                data = self.__receiveData(self.dataDescriptor)
                capsule_current = self.capsuleType(data)
                
                nodes = self.capsuleType._parseCapsule(self.capsulePrev, capsule_current) # type: ignore
                for index, node in enumerate(nodes):
                        self.currentMap.addVal(lidarMeasurement(raw_bytes=None, measurement_hq=node), self.combinedTranslation) # type: ignore

                self.capsulePrev = capsule_current
            else:
                break
            


                


    def __validatePackage(self, pack:bytes, printErrors:bool=False)->bool:
        """Takes a 5 length byte pack response to a standard or forced scan and returns wether or not that scan has all of the correct checksums(and some other checks for legitimacy)"""
        startFlag = bool(pack[0] & 0x1)
        quality = pack[0] >> 2 # type: ignore
        angle = ((pack[1] >> 1) + (pack[2] << 7)) / 64.0
        distance = (pack[3] + (pack[4] << 8)) / 4.0


        #checks first checksum
        if startFlag == bool(pack[0]>>1 & 0x1):
            if printErrors:
                print("Start flag checksum was invalid")
            return False
        
        #checks second checksum
        if not bool(pack[1]&0x1):
            if printErrors:
                print("second byte checksum was invalid")
            return False

        #checks if angle is over 360
        if angle>360:
            if printErrors:
                print("angle was invalid at value ", angle)
            return False
        
        #checks if distance is over 25 meters(max for lidar should be 16)
        if distance>25000:
            if printErrors:
                print("distance was too far away to be legit at value ", distance)
            return False

        return True
        


    def __sendCommand(self, cmd:bytes, payload:bytes=None)->None: # type: ignore
        """
            INTERNAL FUNCTION, NOT FOR OUTSIDE USE.
            sends the specified command over the serial bus. throws an RPlidarConnectionError if no serial bus is connected.
        """
        if self.lidarSerial == None: # type: ignore
            raise LidarConnectionError("PyRPlidar Error : device is not connected")

        self.lidarSerial.sendData(LidarCommand(cmd, payload).rawBytes)

    def __receiveDescriptor(self)->LidarResponse:
        """
            INTERNAL FUNCTION, NOT FOR OUTSIDE USE.
            Receives but does not save a rplidar scan descriptor. Assumes that the descriptor is the first thing in the bus and will otherwise throw an error.
        """
        if self.lidarSerial == None: # type: ignore
            raise LidarConnectionError("PyRPlidar Error : device is not connected")
        
        count=0
        while self.lidarSerial.bufferSize()<7:
            sleep(0.001)
            count+=0.001
            if count>10:
                raise LidarConnectionError("did not receive connection response from RPlidar:", self.config.name)


        descriptor = LidarResponse(self.lidarSerial.receiveData(RPLIDAR_DESCRIPTOR_LEN))
        
        if descriptor.syncByte1 != RPLIDAR_SYNC_BYTE1[0] or descriptor.syncByte2 != RPLIDAR_SYNC_BYTE2[0]:
            raise LidarProtocolError("PyRPlidar Error : sync bytes are mismatched", hex(descriptor.syncByte1), hex(descriptor.syncByte2))

        return descriptor

    def __receiveData(self, descriptor:LidarResponse, waitTime:int=0)->bytes:
        """
            INTERNAL FUNCTION, NOT FOR OUTSIDE USE.
            Fetches a package from the lidar using the entered descriptor as a guide. Will return none if the buffer doesn't contain enough data.
        """
        if self.lidarSerial == None: # type: ignore
            raise LidarConnectionError("PyRPlidar Error : device is not connected")
        count=0
        
        while self.lidarSerial.bufferSize()<descriptor.dataLength:
            sleep(0.001)
            count+=0.001
            if count>waitTime:
                return None # type: ignore
            
        return self.lidarSerial.receiveData(descriptor.dataLength)
        



    def stop(self)->None:
        """Stops the current scan cycle on the lidar but does not stop the motor"""
        self.__sendCommand(RPLIDAR_CMD_STOP)

    def reset(self)->None:
        """Restarts the lidar as if it was just powered on but does not effect the client side lidar lib at all"""
        self.__sendCommand(RPLIDAR_CMD_RESET)


    def setMotorPwm(self, pwm:int, overrideInternalValue:bool=True)->None:
        """Sets the lidar's motor to the specified pwm value. the speed must be a positive number or 0 and lower or equal to the specified max value(currently 1023)"""
        if pwm<0 or pwm>RPLIDAR_MAX_MOTOR_PWM:
            raise ValueError("lidar pwm was set to a value not within the range: ",pwm)
        self.lidarSerial.setDtr(False)

        if self.config.defaultSpeed<0 or self.config.defaultSpeed>RPLIDAR_MAX_MOTOR_PWM:
            raise ValueError("lidar config pwm was set to a value not within the range: ",pwm)


        if overrideInternalValue:
            self.config.defaultSpeed=pwm

        self.__sendCommand(RPLIDAR_CMD_SET_MOTOR_PWM, struct.pack("<H", self.config.defaultSpeed))
        
    
    

    def __getInfo(self)->None:
        """
            Fetches and saves the connected lidar's info in the form of a RPlidarDeviceInfo object.
            This function should be called once when the lidar is first connected and never again.
        """
        self.__sendCommand(RPLIDAR_CMD_GET_INFO)
        descriptor = self.__receiveDescriptor()
        data = self.__receiveData(descriptor, 1)
        if data==None: # type: ignore
            raise RuntimeError("Could not fetch info from the lidar")
        self.lidarInfo = LidarDeviceInfo(data)


    def getInfo(self)->LidarDeviceInfo:
        """
            Returns the connected lidar's info in the form of a RPlidarDeviceInfo object.
            Due to technical limitations this function returns data cached when the lidar was most recently connected.
            While this will not normally cause issues it is something to be aware of.
        """
        if not self.lidarInfo:
            raise ValueError("Lidar info can not be fetched before the lidar has been connected")
        return self.lidarInfo



    def __getHealth(self)->None:
        """
            Fetches and saves the connected lidar's health in the form of a RPlidarDeviceHealth object. 
            This function should be called once when the lidar is first connected and never again.
        """

        self.__sendCommand(RPLIDAR_CMD_GET_HEALTH)
        descriptor = self.__receiveDescriptor()
        data = self.__receiveData(descriptor, 1)
        if data==None: # type: ignore
            raise RuntimeError("Could not get health data from the lidar")
        self.lidarHealth = LidarHealth(data)

    def getHealth(self)->LidarHealth:
        """
            Returns the connected lidar's health in the form of a RPlidarDeviceHealth object.
            Due to technical limitations this function returns data cached when the lidar was most recently connected.
            While this will not normally cause issues it is something to be aware of.
        """
        if not self.lidarHealth:
            raise ValueError("Lidar health can not be fetched before the lidar has been connected")
        return self.lidarHealth

    def __getSampleRate(self)->None:
        """
            Fetches and saves the connected lidar's sample rates for both standard and express modes. This function should be called once when the lidar is first connected and never again.  
            The measurements are in microseconds per reading. the data is returned in the form of a RPlidarSampleRateObject.
        """
        self.__sendCommand(RPLIDAR_CMD_GET_SAMPLERATE)
        descriptor = self.__receiveDescriptor()
        data = self.__receiveData(descriptor,1)
        if data==None: # type: ignore
            raise RuntimeError("Could not get sample rate from the lidar")
        self.sampleRate = LidarSampleRate(data)

    def getSampleRate(self)->LidarSampleRate:
        """
            Fetches and returns the connected lidar's sample rates for both standard and express modes. 
            The measurements are in microseconds per reading. the data is returned in the form of a RPlidarSampleRateObject.
            Due to technical limitations this function returns data cached when the lidar was most recently connected.
            While this will not normally cause issues it is something to be aware of.
        """
        if not self.sampleRate:
            raise ValueError("Lidar sample rate can not be fetched before the lidar has been connected")
        return self.sampleRate

    def __getLidarConf(self, payload:bytes)->bytes:
        """
            INTERNAL FUNCTION, NOT FOR OUTSIDE USE
            Fetches and returns the connected lidar's current configuration and returns it as a pack of bytes
        """
        
        self.__sendCommand(RPLIDAR_CMD_GET_LIDAR_CONF, payload)
        descriptor = self.__receiveDescriptor()
        data = self.__receiveData(descriptor, 1)
        if data==None: # type: ignore
            raise RuntimeError("Could not get config data from the lidar")
        return data

    def __getScanModeCount(self)->None:
        """
            Fetches and saves the number of scan modes supported by the connected lidar.
            This function should be called once when the lidar is first connected and never again.
        """

        data = self.__getLidarConf(struct.pack("<I", RPLIDAR_CONF_SCAN_MODE_COUNT))
        self.scanModeCount = struct.unpack("<H", data[4:6])[0]
        
    
    def getScanModeCount(self)->int:
        """
            Returns the number of scan modes supported by the connected lidar.
            Due to technical limitations this function returns data cached when the lidar was most recently connected.
            While this will not normally cause issues it is something to be aware of.
        """

        if not self.scanModeCount:
            raise ValueError("Lidar scan mode count can not be fetched before the lidar has been connected")
        return self.scanModeCount

    def __getScanModeTypical(self)->None:
        """
            Fetches saves the best scan mode for the connected lidar.
            This function should be called once when the lidar is first connected and never again.
        """

        data = self.__getLidarConf(struct.pack("<I", RPLIDAR_CONF_SCAN_MODE_TYPICAL))
        self.typicalScanMode = struct.unpack("<H", data[4:6])[0]
        
    

    def getScanModeTypical(self)->int:
        """
            Returns the best scan mode for the connected lidar.
            Due to technical limitations this function returns data cached when the lidar was most recently connected.
            While this will not normally cause issues it is something to be aware of.
        """

        if not self.typicalScanMode:
            raise ValueError("Lidar typical scan mode can not be fetched before the lidar has been connected")
        
        return self.typicalScanMode

    def __getScanModes(self)->None:
        """
            Fetches and saves the scan modes of the lidar for later use. This function should be called once when the lidar is first connected and never again.
            WARNING: some of the modes returned may be supported by the lidar but not supported by the client side lib.
        """
        
        for mode in range(self.getScanModeCount()):
            scan_mode = LidarScanMode(
                            self.__getLidarConf(struct.pack("<IH", RPLIDAR_CONF_SCAN_MODE_NAME, mode)),
                            self.__getLidarConf(struct.pack("<IH", RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE, mode)),
                            self.__getLidarConf(struct.pack("<IH", RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE, mode)),
                            self.__getLidarConf(struct.pack("<IH", RPLIDAR_CONF_SCAN_MODE_ANS_TYPE, mode)))
            #print(scan_mode)
            self.scanModes.append(scan_mode)
        
        
    
    def getScanModes(self)->list[LidarScanMode]:
        """
            Returns a list of RPlidarScanMode objects for each scan mode supported by the current connected lidar.
            Due to technical limitations this function returns data cached when the lidar was most recently connected.
            While this will not normally cause issues it is something to be aware of.
            WARNING: some of the modes returned may be supported by the lidar but not supported by the client side lib.
        """
        if not self.scanModes:
            raise ValueError("Lidar scan modes can not be fetched before the lidar has been connected")
        
        return self.scanModes
    
    


    def startScan(self)->None:
        """
            Starts a standard scan on the lidar and starts the update cycle.
        """
        
        if self.isRunning():
            raise RuntimeError("Attempted to start a scan on lidar", self.config.name,
                                "while a scan was already running. Please stop a scan before starting another one as running 2 at once can cause issues.")

        self.__sendCommand(RPLIDAR_CMD_SCAN)
        self.dataDescriptor = self.__receiveDescriptor()

        self.setMotorPwm(self.config.defaultSpeed, overrideInternalValue=False)
        
        self.__establishLoop(self.__standardUpdate)



    def __startRawScan(self)->None:
        """
            INTERNAL FUNCTION, NOT FOR OUTSIDE USE
            Initializes a scan without starting a even loop or reSaving descriptor information. If both of those things have not already been started/saved calling this function will lead to issues
            Used to restart scans when errors have occurred. 
        """
        self.__sendCommand(RPLIDAR_CMD_SCAN)
        self.lidarSerial.receiveData(RPLIDAR_DESCRIPTOR_LEN)
    

    def startScanExpress(self, mode:int = "auto"): # type: ignore
        """
            Starts a scan in express mode (using a compression format so that more samples may be handled per second).
            If a mode is specified then the lidar will attempt to start express in given mode. 
            If mode is set to \"auto\" or not set at all the lidar will instead start and express scan in the recommended mode for the given model.
            WARNING. The lidar lib does not check that a specified express mode is supported by the connected lidar. This must be done by the user.
        """

        if self.isRunning():
            raise RuntimeError("Attempted to start a scan on lidar", self.config.name,
                                "while a scan was already running. Please stop a scan before starting another one as running 2 at once can cause issues.")

        if mode == "auto": # type: ignore
            mode = self.getScanModeTypical()

        if mode not in range(0,5):
            raise ValueError("mode value must be \"auto\" or an integer in range 0-4 instead of the given", mode)

        self.setMotorPwm(self.config.defaultSpeed, overrideInternalValue=False)
        self.__sendCommand(RPLIDAR_CMD_EXPRESS_SCAN, struct.pack("<BI", mode, 0x00000000))
        self.dataDescriptor = self.__receiveDescriptor()
        print(self.dataDescriptor)

        self.__establishLoop(self.__capsuleUpdate)

        if self.dataDescriptor.dataType == 0x82:
            self.capsuleType = LidarScanCapsule
        elif self.dataDescriptor.dataType == 0x84:
            self.capsuleType = LidarScanUltraCapsule # type: ignore
        elif self.dataDescriptor.dataType == 0x85:
            self.capsuleType = LidarScanDenseCapsule # type: ignore
        else:
            raise LidarProtocolError("RPlidar Error : scan data type is not supported")
        

    def isConnected(self)->bool:
        return self.lidarSerial and self.lidarSerial.isOpen()
        
    def getCombinedTrans(self)->translation:
        """
            Returns the current combined translation of the lidar. AKA a translation that Incorporates both the local and global translations. 
        """
        return self.combinedTranslation
    
    def startForceScan(self)->None:
        """
            Initializes a force scan. This scan will always be run by the lidar no matter its current state. 
            Since force scans use the same return packets as normal scans it may appear that the lidarlib initialized a normal scan and not a force scan but all data will be handled properly
        """
        self.__sendCommand(RPLIDAR_CMD_FORCE_SCAN)
        self.__establishLoop(self.__standardUpdate)

    
    def _mapIsDone(self)->None:
        """Handles all the cleanup that is needed when a scan map is done and a new one needs to be initialized"""
        self.__lastMap=self.currentMap
        self.currentMap=lidarMap(self, mapID=self.__lastMap.mapID+1, deadband=self.config.deadband, sensorThetaOffset=self.localTranslation.theta)
        if self.config.debugMode:
            print("map swap attempted")
            print(len(self.__lastMap.getPoints()),self.__lastMap.len, self.__lastMap.mapID, self.__lastMap.getRange(), self.__lastMap.getHz(), self.__lastMap.getPeriod())
            print(len(self.currentMap.getPoints()),self.currentMap.len ,self.currentMap.mapID)
        

    def setCurrentLocalTranslation(self, translation:translation)->None:
        """
            Sets the lidar's objects local translation. This translation should be used for the translation from the lidar to the center of the robot. 
            The translation will be added to all future points read by the lidar(until changed) but will not be added to old retroactively.
        """
        self.localTranslation=translation
        self.currentMap.setOffset(self.localTranslation.theta)
        self.combinedTranslation=self.localTranslation.combineTranslation(self.globalTranslation)

    def setCurrentGlobalTranslation(self, translation:translation)->None:
        """
            Sets the lidar's objects global translation. This translation should be used for the translation between the robot and the 0,0 of the field 
            The translation will be added to all future points read by the lidar(until changed) but will not be added to old retroactively.
        """
        self.globalTranslation=translation
        self.combinedTranslation=self.globalTranslation.combineTranslation(self.localTranslation)


    def setDeadband(self, deadband:list[float])->None:
        """
            Sets a deadband of angles that will be dropped by the lidar. The dropped angle is calculated after the local translation but before the global translation. 
            The imputed argument should be a list of ints in which the first argument is the start of the deadband and the second is the end. 
            If the second argument is larger than the first the deadband will be assumed to wrap past 360 degrees. 
        """
        self.config.deadband=deadband


    def getLastMap(self)->lidarMap:
        """Returns the last full map measured by the lidar."""
        return self.__lastMap
        self.currentMap.setDeadband(deadband)
