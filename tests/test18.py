import usb.core

lidar  = usb.core.find(idVendor=0x10c4, idProduct=0xea60)
print(lidar.port_number)
print(lidar.serial_number)