import serial
from serial.tools import list_ports


"""
    Opens a serial port on the specified port and with the specified baud rate. 
    Will print a warning but not throw an error if the port can not be opened. This error checking should be managed somewhere else
"""
for bus in list_ports.comports(include_links=True):
    if bus.serial_number == "0d36450424516c46aed8d13c7c377c9a":
        port = bus.device
        print(port)



try:
    serial = serial.Serial(port, 256000, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=3, dsrdtr=True)
except serial.SerialException as err:
    print(err)
    print("Failed to connect to the rplidar")