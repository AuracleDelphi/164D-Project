import serial

ser = serial.Serial("COM10",9600, timeout = 1)

def retreiveData():
    ser.write(b'1')
    data = ser.readline().decode('ascii')
    return data

while(True):
    print(retreiveData())
    
    
