from serial.tools import list_ports
import serial
import numpy as np
from ctypes import c_uint16

start = b"\x10"

# Get nanovna device automatically
def getport() -> str:
    VID = 0x0483 #1155
    PID = 0x5740 #22336
    device_list = list_ports.comports()
    for device in device_list:
        if device.vid == VID and device.pid == PID:
            return device.device
    raise OSError("device not found")


def fnc(in_buff, idx):
    return in_buff[idx], in_buff[idx + 1]


def main():

    stm = serial.Serial(getport(), 115200)  

    ip = 0
    while ip < 1:

        stm.write(start)
        data = stm.read(200)

        var3 = np.array([])

        i = 0
        while i < 200:

            lb, hb = fnc(data, i)

            hb = hb << 8
            i += 2
            buf = float((hb | lb) * 3.3 / 4096)
            var3 = np.append(var3, buf)
        
        ip += 1

    print(var3)
    stm.close()










'''
        i = 0
        buf = 0.0
        while i < 200: #for i in range(100):
            for x in range(2):
                lb = data[i + x]
                print(f"{i}  {lb} one")
                i = i + 1
                hb = data[i] << 8
                print(f"{i}  {hb} two")
                i = i + 1
            buf = 

            buf = float((hb | lb) * 3.3 / 4096)
            print(buf)
            var3 = np.append(var3, buf)
          
 '''   
        #print(var3)
        #ip += 1
        
    

    
if __name__ == '__main__':
    main()
