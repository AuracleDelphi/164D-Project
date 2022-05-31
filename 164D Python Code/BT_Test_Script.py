import serial

ser = serial.Serial("COM5",9600, timeout = 1)

def retreiveData():
    ser.write(b'1')
    data = ser.readline().decode('ascii')
    return data

while(True):
    uInput = input("Retreive data? ")
    if uInput == '1':
        print(retreiveData())
    else:
        ser.write(b'0')

#sArr = ['']*40
#for i in range(0,40):
#    s = retreiveData()
#    print(s)
#    sArr[i] = s
#ser.write(b'0')
#print(sArr)


    
