import json
from typing import Any
import typing


from lidarLib import lidarProtocol
from lidarLib.translation import translation




class lidarConfigs:
    """<h2>Class to store and handle lidar configs.</h2>"""
    
    defaultConfigs:dict[str, Any] = {
                
        "port" : None,
        "localTrans": translation.default(),
        "baudrate" : 256000,
        "deadband" : None,
        "timeout" : 3,
        "mode"  : "normal",
        "debugMode" : False,
        "isStop" : False,
        "autoStart" : False,
        "autoConnect" : True,
        "defaultSpeed" : lidarProtocol.RPLIDAR_DEFAULT_MOTOR_PWM,
        "vendorID" : 0x10c4,
        "productID" : 0xea60,
        "serialNumber" : None, 
        "name" : None,
        "type": "ValueThatWillNeverBeUsedButNeedsToExistForReasons"

    }




    def __init__(
                    self, 
                    port:str = defaultConfigs["port"],
                    vendorID:int = defaultConfigs["vendorID"],
                    productID:int = defaultConfigs["productID"],
                    serialNumber:str = defaultConfigs["serialNumber"],
                    localTrans:translation = defaultConfigs['localTrans'], 
                    baudrate:int = defaultConfigs["baudrate"], 
                    timeout:int=defaultConfigs["timeout"], 
                    mode:str=defaultConfigs["mode"], 
                    deadband:list[float]=defaultConfigs["deadband"], 
                    debugMode:bool=defaultConfigs["debugMode"],
                    isStop:bool=defaultConfigs["isStop"], 
                    autoStart:bool=defaultConfigs["autoStart"], 
                    autoConnect:bool=defaultConfigs["autoConnect"], 
                    defaultSpeed:int=defaultConfigs["defaultSpeed"],
                    name:str = defaultConfigs["name"]
                    
            ):
        """
            <h2>Creates a lidar config from specified configs.</h2>
            While this function is usable we recommend storing configs in a json and using the configFromJson static method instead. 
        """

        self.port=port
        self.localTrans:translation = localTrans
        self.baudrate:int = baudrate
        self.timeout:int=timeout
        self.deadband:list[float]=deadband
        self.debugMode:bool=debugMode
        self.isStop:bool=isStop
        self.autoStart:bool=autoStart
        self.defaultSpeed:int = defaultSpeed

        self.mode:str=mode
        self.autoConnect:bool=autoConnect
        self.vendorID:int=vendorID
        self.productID:int = productID
        self.serialNumber:str = serialNumber
        self.name:str = name

        if not self.port and not self.serialNumber:
            raise ValueError("Ether a serial number or a port must be specified in a lidar configs object")

        if debugMode:
            self.printConfigs()
            
    def printConfigs(self):
        """<h2>Prints configs </h2>"""
        print("lidarConfig args", 
            "\nport:", self.port,
            "\nlocalTrans:", self.localTrans,
            "\nbaudrate:", self.baudrate,
            "\ndeadband:", self.deadband,
            "\ndebugMode:", self.debugMode,
            "\nisStop:", self.isStop,
            "\ntimeout: ", self.timeout,
            "\nautoStart:", self.autoStart,
            "\nautoConnect:", self.autoConnect,
            "\ndefualtSpeed:", self.defaultSpeed,
            "\nmode:", self.mode,         
            "\nvendorID: ", self.vendorID,
            "\nproductID: ", self.productID,
            "\nserialNumber: ", self.serialNumber,
            "\nname:", self.name
        )

    @classmethod # type: ignore
    @typing.no_type_check
    def configsFromJson(cls:"lidarConfigs", path:str)->"lidarConfigs": # type: ignore
        """<h2>Creates a config object from a given json.<h2>"""
        try:
            with open(path, 'r') as file:
                data:dict[str, any] = json.load(file) # type: ignore

                if data.get("type", "") != "lidarConfig": # type: ignore
                    raise Warning(
                        "the file given does not include the correct type tag.",
                        "Please make sure to include \"type\" : \"lidarConfig\" in all lidar config files so that the library can easily differentiate them."
                    )


                return cls(
                    port = data.get("port"),  # type: ignore
                    vendorID = data.get("vendorID", lidarConfigs.defaultConfigs["vendorID"]),
                    productID = data.get("productID", lidarConfigs.defaultConfigs["productID"]),
                    serialNumber = data.get("serialNumber", lidarConfigs.defaultConfigs["serialNumber"]),

                    localTrans = translation.fromCart(
                        data.get("localTrans").get("x", lidarConfigs.defaultConfigs["localTrans"].x),
                        data.get("localTrans").get("y", lidarConfigs.defaultConfigs["localTrans"].y),
                        data.get("localTrans").get("rotation", lidarConfigs.defaultConfigs["localTrans"].rotation)
                    ),
                    baudrate = data.get("baudrate", lidarConfigs.defaultConfigs["baudrate"]),
                    deadband = data.get("deadband", lidarConfigs.defaultConfigs["deadband"]),
                    timeout = data.get("timeout", lidarConfigs.defaultConfigs["timeout"]),
                    mode  = data.get("mode", lidarConfigs.defaultConfigs["mode"]),
                    debugMode = data.get("debugMode", lidarConfigs.defaultConfigs["debugMode"]),
                    isStop = data.get("isStop", lidarConfigs.defaultConfigs["isStop"]),
                    autoStart = data.get("autoStart", lidarConfigs.defaultConfigs["autoStart"]),
                    autoConnect = data.get("autoConnect", lidarConfigs.defaultConfigs["autoConnect"]),
                    defaultSpeed = data.get("defaultSpeed", lidarConfigs.defaultConfigs["defaultSpeed"]),
                    name = data.get("name", lidarConfigs.defaultConfigs["name"])

                )
            


        except FileNotFoundError:
            raise FileNotFoundError(f"Error: File not found at path:", path)
        except json.JSONDecodeError:
            raise SystemError(f"Error: Invalid JSON format in file:", path)
        except Exception as e:
            raise RuntimeError("Something went wrong trying to read the json" + e)

    def writeToJson(self, path:str):
        """<h2>Writes this config file to a .json at the specified location </h2>"""
        try:
            data = { # type: ignore
                
                "port" : self.port,
                "localTrans": {
                    "x":self.localTrans.x,
                    "y":self.localTrans.y,
                    "rotation":self.localTrans.rotation
                },
                "baudrate" : self.baudrate,
                "deadband" : self.deadband,
                "timeout" : self.timeout,
                "mode"  : self.mode,
                "debugMode" : self.debugMode,
                "isStop" : self.isStop,
                "autoStart" : self.autoStart,
                "autoConnect" : self.autoConnect,
                "defaultSpeed" : self.defaultSpeed,

                "serialNumber" : self.serialNumber,
                "productID" : self.productID,
                "vendorID" : self.vendorID,
                "name" : self.name,
                "type" : "lidarConfig"
            }

            keysToRemove = []
            for key in data.keys():
                if data[key] == lidarConfigs.defaultConfigs[key]:
                    keysToRemove.append(key) # type: ignore

            for key in keysToRemove: # type: ignore
                data.pop(key) # type: ignore
            

            with open(path, 'w') as file:
                json.dump(data, file, indent=4)                


        except FileNotFoundError:
            print(f"Error: File not found at path: [path]")
            return None




