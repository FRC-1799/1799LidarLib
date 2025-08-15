import serial
from serial.tools import list_ports



class RPlidarSerial:
    """Class to handle the serial bus used by a lidar"""
    def __init__(self):
        """
            <h2>Creates a lidar serial object but does not connect to any ports</h2>
            Actual connection must be done through the open function
        """
        self.serial:serial.Serial = None # type: ignore

    def open(self, port:str, vendorID:int, productID:int, serialNumber:str, baudrate:int, timeout:int=3)->None:
        """
            <h2>Opens a serial port. </h2>
            This function can connect in two ways, ether directly to a serial port or through usb id information.
            We recommend using the USB information as it is much more consistent across power cycles. \n
            Using USB information:
                This method needs a product ID, vendor ID, and serial number. All of these are standard to the USB spec and can be found through various command line tools or using this libraries lidar config tool.
            
            Using port:
                This will use some serial port ex /dev/ttyUSB0. We do not recommend this method as serial port names can change during power cycles.     

            Baudrate should be an integer corresponding too the baudrate. 
            WARNING If baudrate is incorrectly set the serial bus will open but will not be able to correctly send or receive data.

            Will print a warning but not throw an error if the port can not be opened. This error checking should be managed somewhere else
        """

        if (not (vendorID and productID and serialNumber)) and not port:
            raise ValueError("lidarSerial can not connect without ether a (vendor id, product id and serial number) or a port")


        if (vendorID and productID and serialNumber):
            for bus in list_ports.comports():
                if bus.serial_number == serialNumber and bus.vid == vendorID and bus.pid == productID:
                    port = bus.device
                    print(port)

        if not port:
            raise ValueError("Could not find device with product id:", productID, ", Vendor id:", vendorID, ", and serialNumber:", serialNumber, ".",
            "Please make sure the lidar is plugged in and check that these three values are correct")

        if self.serial is not None: # type: ignore
            self.close()


        try:
            self.serial = serial.Serial(port, baudrate, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=timeout, dsrdtr=True)
        except serial.SerialException as err:
            print(err)
            print("Failed to connect to the rplidar")
    
    def close(self)->None:
        """<h2>Closes and deletes the internal serial port</h2>"""
        if self.serial is None: # type: ignore
            return
        self.serial.close()
        self.serial=None # type: ignore
    
   
    
    def sendData(self, data:bytes)->None:
        """<h2>sends the specified data through the serial port</h2>"""
        self.serial.write(data)
    
    def receiveData(self, size:int)->bytes:
        """
            <h2>Receives and returns the specified number of bytes from the serial port.</h2>
            If not enough bytes are available all that are will be returned
        """
        return self.serial.read(size)

    def setDtr(self, value:int)->None:
        """
            <h2>Sets the dtr of the internal serial port. </h2>
            Whenever the port is closed this value will be lost and need to be reset.
        """
        self.serial.dtr = value # type: ignore 

    def isOpen(self)->bool:
        """<h2>Returns wether or not the internal port is open</h2>"""
        return self.serial!=None  # type: ignore
    

    def bufferSize(self)->int:
        """
            <h2>Returns the number of Bytes currently in the serial buffer. </h2>
            Will return 0 if the serial bus is not open.
        """
        if self.isOpen():
            return self.serial.in_waiting
        return 0


    def flush(self)->None:
        """<h2>Flushes the serial buffer. </h2>"""
        self.serial.reset_input_buffer()
        