import struct
import codecs




RPLIDAR_SYNC_BYTE1 = b'\xA5'
RPLIDAR_SYNC_BYTE2 = b'\x5A'

RPLIDAR_DESCRIPTOR_LEN = 7

RPLIDAR_SEND_MODE_SINGLE_RES     = b'\x00'
RPLIDAR_SEND_MODE_MULTIPLE_RES   = b'\x01'

RPLIDAR_CMD_STOP            = b'\x25'
RPLIDAR_CMD_RESET           = b'\x40'
RPLIDAR_CMD_SCAN            = b'\x20'
RPLIDAR_CMD_EXPRESS_SCAN    = b'\x82'
RPLIDAR_CMD_FORCE_SCAN      = b'\x21'
RPLIDAR_CMD_GET_INFO        = b'\x50'
RPLIDAR_CMD_GET_HEALTH      = b'\x52'
RPLIDAR_CMD_GET_SAMPLERATE  = b'\x59'
RPLIDAR_CMD_GET_LIDAR_CONF  = b'\x84'

RPLIDAR_CMD_HQ_MOTOR_SPEED_CTRL = b'\xA8'
RPLIDAR_CMD_SET_MOTOR_PWM       = b'\xF0'
RPLIDAR_CMD_GET_ACC_BOARD_FLAG  = b'\xFF'


RPLIDAR_MAX_MOTOR_PWM       = 1023
RPLIDAR_DEFAULT_MOTOR_PWM   = 660


RPLIDAR_CONF_SCAN_MODE_COUNT            = 0x00000070
RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE    = 0x00000071
RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE     = 0x00000074
RPLIDAR_CONF_SCAN_MODE_ANS_TYPE         = 0x00000075
RPLIDAR_CONF_SCAN_MODE_TYPICAL          = 0x0000007C
RPLIDAR_CONF_SCAN_MODE_NAME             = 0x0000007F


RPLIDAR_STATUS = {
    0: "GOOD",
    1: "WARNING",
    2: "ERROR"
}

RPLIDAR_ANS_TYPE = {
    0x81: "NORMAL",
    0x82: "CAPSULED",
    0x83: "HQ",
    0x84: "ULTRA_CAPSULED",
    0x85: "DENSE_CAPSULED"
}


RPLIDAR_VARBITSCALE_X2_SRC_BIT = 9
RPLIDAR_VARBITSCALE_X4_SRC_BIT = 11
RPLIDAR_VARBITSCALE_X8_SRC_BIT = 12
RPLIDAR_VARBITSCALE_X16_SRC_BIT = 14

RPLIDAR_VARBITSCALE_X2_DEST_VAL = 512
RPLIDAR_VARBITSCALE_X4_DEST_VAL = 1280
RPLIDAR_VARBITSCALE_X8_DEST_VAL = 1792
RPLIDAR_VARBITSCALE_X16_DEST_VAL = 3328

VBS_SCALED_BASE = [RPLIDAR_VARBITSCALE_X16_DEST_VAL,
                   RPLIDAR_VARBITSCALE_X8_DEST_VAL,
                   RPLIDAR_VARBITSCALE_X4_DEST_VAL,
                   RPLIDAR_VARBITSCALE_X2_DEST_VAL,
                   0]

VBS_SCALED_LVL = [4, 3, 2, 1, 0]

VBS_TARGET_BASE = [(0x1 << RPLIDAR_VARBITSCALE_X16_SRC_BIT),
                   (0x1 << RPLIDAR_VARBITSCALE_X8_SRC_BIT),
                   (0x1 << RPLIDAR_VARBITSCALE_X4_SRC_BIT),
                   (0x1 << RPLIDAR_VARBITSCALE_X2_SRC_BIT),
                   0]



class LidarConnectionError(Exception):
    pass

class LidarProtocolError(Exception):
    pass



class LidarCommand:
    """class to encapsulate a command sent to the lidar"""
    def __init__(self, cmd:bytes, payload:bytes=None): # type: ignore
        """Creates a rplidar command of type cmd. payload will be sent if set for commands that require extra information """
        self.cmd = cmd
        self.payload = payload
        self.rawBytes = RPLIDAR_SYNC_BYTE1 + cmd
        
        if payload is not None: # type: ignore
            size = struct.pack('B', len(payload))
            self.rawBytes += size + payload
            self.rawBytes += struct.pack('B', self.getChecksum(self.rawBytes))

    def getChecksum(self, data:bytes):
        """returns the checksum of the packet entered"""
        checkSum = 0
        for value in data: checkSum ^= value
        return checkSum


class LidarResponse:
    """Class to define scan metadata information returned by the lidar"""
    
    def __init__(self, rawBytes:bytes):
        """creates a rplidarResponse from the raw bytes returned by the lidar"""
        self.syncByte1 = rawBytes[0]
        self.syncByte2 = rawBytes[1]
        sizeQ30LengthType = struct.unpack("<L", rawBytes[2:6])[0]
        self.dataLength = sizeQ30LengthType & 0x3FFFFFFF
        self.sendMode = sizeQ30LengthType >> 30
        self.dataType = rawBytes[6]

    def __str__(self):
        data = { # type: ignore
            "syncByte1" : hex(self.syncByte1),
            "syncByte2" : hex(self.syncByte2),
            "dataLength" : self.dataLength,
            "sendMode" : self.sendMode,
            "dataType" : hex(self.dataType),
        }
        return str(data) # type: ignore






class LidarDeviceInfo:
    """Class to handle and store device information given by the lidar"""
    def __init__(self, rawBytes:bytes):
        """creates device info object from the raw bytes returned by the lidar"""
        self.model = rawBytes[0]
        self.firmwareMinor = rawBytes[1]
        self.firmwareMajor = rawBytes[2]
        self.hardware = rawBytes[3]
        self.serialNumber = codecs.encode(rawBytes[4:], 'hex').upper()
        self.serialNumber = codecs.decode(self.serialNumber, 'ascii')

    def __str__(self):
        data = { # type: ignore
            "model" : self.model,
            "firmwareMinor" : self.firmwareMinor,
            "firmwareMajor" : self.firmwareMajor,
            "hardware" : self.hardware,
            "serialNumber" : self.serialNumber
        }
        return str(data) # type: ignore


class LidarHealth:
    """class to handle and store health information given by the lidar"""
    def __init__(self, rawBytes:bytes):
        """creates a lidar health object from the raw byte package returned by the lidar"""
        self.status = rawBytes[0]
        self.error_code = (rawBytes[1] << 8) + rawBytes[2]

    def __str__(self):
        data = {
            "status" : self.status,
            "error_code" : self.error_code
        }
        return str(data)


class LidarSampleRate:
    """Class to handle and store sampleRate information given by the lidar"""
    def __init__(self, rawBytes:bytes):
        """creates a lidar sampleRate object from the raw byte package returned by the lidar"""
        self.typeStandard = rawBytes[0] + (rawBytes[1] << 8)
        self.typeExpress = rawBytes[2] + (rawBytes[3] << 8)
    
    def __str__(self):
        data = {
            "t_standard" : self.typeStandard,
            "t_express" : self.typeExpress
        }
        return str(data)


class LidarScanMode:
    """Class to handle and store a scan mode given by the lidar"""
    def __init__(self, dataName:bytes, dataMaxDistance:bytes, dataUsPerSample:bytes, dataAnsType:bytes):
        """Creates a scan mode using the byte packs returned by the lidar"""
        self.usPerSample = struct.unpack("<I", dataUsPerSample[4:8])[0]
        self.maxDistance = struct.unpack("<I", dataMaxDistance[4:8])[0]
        self.ansType = struct.unpack("<B", dataAnsType[4:5])[0]
        self.name = codecs.decode(dataName[4:-1], 'ascii')
    
    def __str__(self):
        data = { # type: ignore
            "name" : self.name,
            "max_distance" : self.maxDistance,
            "us_per_sample" : self.usPerSample,
            "ans_type" : RPLIDAR_ANS_TYPE[self.ansType]
        }
        return str(data) # type: ignore





# @DeprecationWarning
class LidarMeasurementHQ:
    
    def __init__(self, syncBit:int, angleQ6:int, distQ2:int):
        self.start_flag = syncBit | ((not syncBit) << 1)
        self.quality = (0x2f << 2) if distQ2 else 0
        self.angle_z_q14:float = (angleQ6 << 8) // 90
        self.dist_mm_q2:float = distQ2

    def getAngle(self):
        return self.angle_z_q14 * 90.0 / 16384.0
    
    def getDistance(self):
        return self.dist_mm_q2 / 4.0



# @DeprecationWarning
class LidarCabin:
    
    def __init__(self, rawBytes:bytes):
        self.distance1 = (rawBytes[0] >> 2) + (rawBytes[1] << 6)
        self.distance2 = (rawBytes[2] >> 2) + (rawBytes[3] << 6)
        self.d_theta1 = (rawBytes[4] & 0x0F) + ((rawBytes[0] & 0x03) << 4)
        self.d_theta2 = (rawBytes[4] >> 4) + ((rawBytes[2] & 0x03) << 4)

# @DeprecationWarning
class LidarScanCapsule:
    
    def __init__(self, rawBytes:bytes):
        self.syncByte1 = (rawBytes[0] >> 4) & 0xF
        self.syncByte2 = (rawBytes[1] >> 4) & 0xF
        self.checksum = (rawBytes[0] & 0xF) + ((rawBytes[1] & 0xF) << 4)
        self.startAngleQ6 = rawBytes[2] + ((rawBytes[3] & 0x7F) << 8)
        self.startFlag = bool((rawBytes[3] >> 7) & 0x1)
        self.cabins = list(map(
                               LidarCabin,
                               [rawBytes[i:i+5] for i in range(4, len(rawBytes), 5)]
                               ))

    @classmethod
    def _parseCapsule(cls, capsulePrev:"LidarScanCapsule", capsuleCurrent:"LidarScanCapsule")->list[LidarMeasurementHQ]:
        
        nodes:list[LidarMeasurementHQ] = []
        
        currentStartAngleQ8 = capsuleCurrent.startAngleQ6 << 2
        prevStartAngleQ8 = capsulePrev.startAngleQ6 << 2
        
        diffAngleQ8 = (currentStartAngleQ8)-(prevStartAngleQ8)
        if prevStartAngleQ8 > currentStartAngleQ8:
            diffAngleQ8 += (360 << 8)
        
        angleIncQ16 = (diffAngleQ8 << 3)
        currentAngleRawQ16 = (prevStartAngleQ8 << 8)

        for pos in range(len(capsulePrev.cabins)):
            
            distQ2 = [0] * 2
            angleQ6 = [0] * 2
            syncBit = [0] * 2
            
            distQ2[0] = capsulePrev.cabins[pos].distance1 << 2
            distQ2[1] = capsulePrev.cabins[pos].distance2 << 2
            
            angleOffset1Q3 = capsulePrev.cabins[pos].d_theta1
            angleOffset2Q3 = capsulePrev.cabins[pos].d_theta2
            
            angleQ6[0] = ((currentAngleRawQ16 - (angleOffset1Q3<<13))>>10)
            syncBit[0] = 1 if ((currentAngleRawQ16 + angleIncQ16) % (360<<16)) < angleIncQ16 else 0
            currentAngleRawQ16 += angleIncQ16
            
            
            angleQ6[1] = ((currentAngleRawQ16 - (angleOffset2Q3<<13))>>10)
            syncBit[1] = 1 if ((currentAngleRawQ16 + angleIncQ16) % (360<<16)) < angleIncQ16 else 0
            currentAngleRawQ16 += angleIncQ16

            
            for cpos in range(2):

                if angleQ6[cpos] < 0: angleQ6[cpos] += (360 << 6)
                if angleQ6[cpos] >= (360 << 6): angleQ6[cpos] -= (360 << 6)
                
                node = LidarMeasurementHQ(syncBit[cpos], angleQ6[cpos], distQ2[cpos])
                nodes.append(node)

        return nodes





# @DeprecationWarning
class LidarDenseCabin:
    
    def __init__(self, rawBytes:bytes):
        self.distance = (rawBytes[0] << 8) + rawBytes[1]

# @DeprecationWarning
class LidarScanDenseCapsule:

    def __init__(self, rawBytes:bytes):
        self.syncByte1 = (rawBytes[0] >> 4) & 0xF
        self.syncByte2 = (rawBytes[1] >> 4) & 0xF
        self.checksum = (rawBytes[0] & 0xF) + ((rawBytes[1] & 0xF) << 4)
        self.startAngleQ6 = rawBytes[2] + ((rawBytes[3] & 0x7F) << 8)
        self.startFlag = bool((rawBytes[3] >> 7) & 0x1)
        self.cabins = list(map(
                               LidarDenseCabin,
                               [rawBytes[i:i+2] for i in range(4, len(rawBytes), 2)]
                               ))

    @classmethod
    def _parseCapsule(cls, capsulePrev:"LidarScanDenseCapsule", capsuleCurrent:"LidarScanDenseCapsule"):
        
        nodes:list[LidarMeasurementHQ] = []
        
        currentStartAngleQ8 = capsuleCurrent.startAngleQ6 << 2
        prevStartAngleQ8 = capsulePrev.startAngleQ6 << 2
        
        diffAngleQ8 = (currentStartAngleQ8)-(prevStartAngleQ8)
        if prevStartAngleQ8 > currentStartAngleQ8:
            diffAngleQ8 += (360 << 8)
        
        angleIncQ16 = (diffAngleQ8 << 8) // 40
        currentAngleRawQ16 = (prevStartAngleQ8 << 8)

        for pos in range(len(capsulePrev.cabins)):
            
            distQ2 = 0
            angleQ6 = 0
            syncBit = 0

            syncBit = 1 if (((currentAngleRawQ16 + angleIncQ16) % (360 << 16)) < angleIncQ16) else 0
            
            angleQ6 = (currentAngleRawQ16 >> 10)
            if angleQ6 < 0: angleQ6 += (360 << 6)
            if angleQ6 >= (360 << 6): angleQ6 -= (360 << 6)
            currentAngleRawQ16 += angleIncQ16
            
            distQ2 = capsulePrev.cabins[pos].distance << 2

            node = LidarMeasurementHQ(syncBit, angleQ6, distQ2)
            nodes.append(node)

        return nodes





# @DeprecationWarning
class LidarUltraCabin:
    
    def __init__(self, rawBytes:bytes):
        self.major = ((int(rawBytes[1]) & 0xF) << 8) + int(rawBytes[0])
        self.predict1 = ((int(rawBytes[2]) & 0x3F) << 4) + ((int(rawBytes[1]) >> 4) & 0xF)
        self.predict2 = ((int(rawBytes[3]) & 0xFF) << 2) + ((int(rawBytes[2]) >> 6) & 0x3)
    
        if self.predict1 & 0x200: self.predict1 |= 0xFFFFFC00
        if self.predict2 & 0x200: self.predict2 |= 0xFFFFFC00
    
    def __str__(self):
        data = {
            "major" : hex(self.major),
            "predict1" : hex(self.predict1),
            "predict2" : hex(self.predict2),
        }
        return str(data)
# @DeprecationWarning
class LidarScanUltraCapsule:

    def __init__(self, rawBytes:bytes):
        self.syncByte1 = (rawBytes[0] >> 4) & 0xF
        self.syncByte2 = (rawBytes[1] >> 4) & 0xF
        self.checksum = (rawBytes[0] & 0xF) + ((rawBytes[1] & 0xF) << 4)
        self.startAngleQ6 = rawBytes[2] + ((rawBytes[3] & 0x7F) << 8)
        self.startFlag = bool((rawBytes[3] >> 7) & 0x1)
        self.ultraCabins = list(map(
                                     LidarUltraCabin,
                                     [rawBytes[i:i+4] for i in range(4, len(rawBytes), 4)]
                                     ))
    

    def __str__(self):
        data = { # type: ignore
            "syncByte1" : hex(self.syncByte1),
            "syncByte2" : hex(self.syncByte2),
            "checksum" : hex(self.checksum),
            "startAngleQ6" : hex(self.startAngleQ6),
            "startFlag" : self.startFlag,
            "ultraCabins" : [str(ultra_cabin) for ultra_cabin in self.ultraCabins]
        }
        return str(data) # type: ignore

    @classmethod
    def _varbitscaleDecode(cls, scaled:int):
        
        scaleLevel = 0
        
        for i in range(len(VBS_SCALED_BASE)):
            
            remain = scaled - VBS_SCALED_BASE[i]
            if remain >= 0:
                scaleLevel = VBS_SCALED_LVL[i]
                return (VBS_TARGET_BASE[i] + (remain << scaleLevel), scaleLevel)
        
        return (0, scaleLevel)

    @classmethod
    def _parseCapsule(cls, capsulePrev:"LidarScanUltraCapsule", capsuleCurrent:"LidarScanUltraCapsule"):
        
        nodes:list[LidarMeasurementHQ] = []
        
        currentStartAngleQ8 = capsuleCurrent.startAngleQ6 << 2
        prevStartAngleQ8 = capsulePrev.startAngleQ6 << 2
        
        diffAngleQ8 = (currentStartAngleQ8)-(prevStartAngleQ8)
        if prevStartAngleQ8 > currentStartAngleQ8:
            diffAngleQ8 += (360 << 8)

        angleIncQ16 = (diffAngleQ8 << 3) // 3
        currentAngleRawQ16 = (prevStartAngleQ8 << 8)
        
        for pos in range(len(capsulePrev.ultraCabins)):
            
            distQ2 = [0] * 3
            angleQ6 = [0] * 3
            syncBit = [0] * 3
            
            distMajor = capsulePrev.ultraCabins[pos].major
            
            # signed partial integer, using the magic shift here
            # DO NOT TOUCH
            
            distPredict1 = capsulePrev.ultraCabins[pos].predict1
            distPredict2 = capsulePrev.ultraCabins[pos].predict2
            
            distMajor2 = 0
            
            # prefetch next ...
            if pos == len(capsulePrev.ultraCabins) - 1:
                distMajor2 = capsuleCurrent.ultraCabins[0].major
            else:
                distMajor2 = capsulePrev.ultraCabins[pos + 1].major
            
            
            # decode with the var bit scale ...
            distMajor, scalelvl1 = LidarScanUltraCapsule._varbitscaleDecode(distMajor)
            distMajor2, scalelvl2 = LidarScanUltraCapsule._varbitscaleDecode(distMajor2)
            
            
            distBase1 = distMajor
            distBase2 = distMajor2
            
            if not(distMajor) and distMajor2:
                distBase1 = distMajor2
                scalelvl1 = scalelvl2
            
            distQ2[0] = (distMajor << 2)
            if (distPredict1 == 0xFFFFFE00) or (distPredict1 == 0x1FF):
                distQ2[1] = 0
            else:
                distPredict1 = (distPredict1 << scalelvl1)
                distQ2[1] = ((distPredict1 + distBase1) << 2) & 0xFFFFFFFF
            
            if (distPredict2 == 0xFFFFFE00) or (distPredict2 == 0x1FF):
                distQ2[2] = 0
            else:
                distPredict2 = (distPredict2 << scalelvl2)
                distQ2[2] = ((distPredict2 + distBase2) << 2) & 0xFFFFFFFF
        
        
            for cpos in range(3):
                
                syncBit[cpos] = 1 if (((currentAngleRawQ16 + angleIncQ16) % (360 << 16)) < angleIncQ16) else 0
                
                offsetAngleMeanQ16 = int(7.5 * 3.1415926535 * (1 << 16) / 180.0)
                
                if distQ2[cpos] >= (50 * 4):
                    
                    k1 = 98361
                    k2 = int(k1 / distQ2[cpos])
                    
                    offsetAngleMeanQ16 = int(8 * 3.1415926535 * (1 << 16) / 180) - int(k2 << 6) - int((k2 * k2 * k2) / 98304)
                
                angleQ6[cpos] = (currentAngleRawQ16 - int(offsetAngleMeanQ16 * 180 / 3.14159265)) >> 10
                currentAngleRawQ16 += angleIncQ16
                
                if angleQ6[cpos] < 0: angleQ6[cpos] += (360 << 6)
                if angleQ6[cpos] >= (360 << 6): angleQ6[cpos] -= (360 << 6)
                
                node:LidarMeasurementHQ = LidarMeasurementHQ(syncBit[cpos], angleQ6[cpos], distQ2[cpos])
                nodes.append(node)

        return nodes
