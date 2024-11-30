import serial 

serial_port = serial.Serial('COM3', 115200, timeout=1)

while True:    
    print(serial_port.readlines())