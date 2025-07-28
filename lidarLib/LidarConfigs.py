import json


from lidarLib import lidarProtocol
from lidarLib.translation import translation




class lidarConfigs:


    defaultConfigs = {
                
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
        "name" : None
    }




    def __init__(
                    self, 
                    port:str = defaultConfigs["port"],
                    vendorID = defaultConfigs["vendorID"],
                    productID = defaultConfigs["productID"],
                    serialNumber = defaultConfigs["serialNumber"],
                    localTrans = defaultConfigs['localTrans'], 
                    baudrate = defaultConfigs["baudrate"], 
                    timeout=defaultConfigs["timeout"], 
                    mode=defaultConfigs["mode"], 
                    deadband=defaultConfigs["deadband"], 
                    debugMode=defaultConfigs["debugMode"],
                    isStop=defaultConfigs["isStop"], 
                    autoStart=defaultConfigs["autoStart"], 
                    autoConnect=defaultConfigs["autoConnect"], 
                    defaultSpeed=defaultConfigs["defaultSpeed"],
                    name = defaultConfigs["name"]
            ):


        self.port=port
        self.localTrans = localTrans
        self.baudrate = baudrate
        self.timeout=timeout
        self.deadband=deadband
        self.debugMode=debugMode
        self.isStop=isStop
        self.autoStart=autoStart
        self.defaultSpeed = defaultSpeed

        self.mode=mode
        self.autoConnect=autoConnect
        self.deadband = deadband
        self.vendorID=vendorID
        self.productID = productID
        self.serialNumber = serialNumber
        self.name = name

        if not self.port and not self.serialNumber:
            raise ValueError("Ether a serial number or a port must be specified in a lidar configs object")

        if debugMode:
            self.printConfigs()
            
    def printConfigs(self):
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

    @classmethod
    def configsFromJson(cls:"lidarConfigs", path:str)->"lidarConfigs":
        try:
            with open(path, 'r') as file:
                data:dict = json.load(file)

                return cls(
                    port = data.get("port"), 
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
            print(f"Error: File not found at path: [path]")
            return None
        except json.JSONDecodeError:
            print(f"Error: Invalid JSON format in file: [path]")
            return None

    def writeToJson(self, path:str):
        try:
            data = {
                
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
            }

            keysToRemove = []
            for key in data.keys():
                if data[key] == lidarConfigs.defaultConfigs[key]:
                    keysToRemove.append(key)

            for key in keysToRemove:
                data.pop(key)
            

            with open(path, 'w') as file:
                json.dump(data, file, indent=4)                


        except FileNotFoundError:
            print(f"Error: File not found at path: [path]")
            return None




