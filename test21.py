from serial.tools import list_ports

for bus in list_ports.comports():
    print(bus)
    print(bus.pid)