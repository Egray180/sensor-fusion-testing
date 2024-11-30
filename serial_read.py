import serial

class Read: 

    def __init__(self, file=None):
        try:
            self.serialPort = serial.Serial("COM4", 9600)
            self.serialInput = True 
        except serial.serialutil.SerialException:
            print("serial not connected")
            self.serialInput = False
            self.dataFile = open(file, 'r')

    def getData(self):
        if self.serialInput: 
            if self.serialPort.in_waiting:
                line = self.serialPort.readline()
            else: 
                while True:
                    if self.serialPort.in_waiting: 
                        line = self.serialPort.readline().strip().split(',') # strip() to remove \n
                        break
            return line
        else:
            line = self.dataFile.readline().strip().split(',')
            return line

class Indices: 
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

if __name__ == "__main__":
    ports = serial.tools.list_ports.comports()
    for port in ports: 
        print(str(port))