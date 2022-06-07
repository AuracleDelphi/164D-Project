import serial
import sys
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import time

def retrieveData(ser):
    ser.write(b'1')
    data = ser.readline().decode('ascii')
    ser.write(b'0')
    return data

def returnTemps(data):
    ambient = data[0:4]
    obj = data[6:len(data)]
    if((ambient != '' and obj != '') and len(data) < 17):
        ambient = float(ambient) * 9/5 + 32
        obj = float(obj) * 9/5 + 32
    else:
        ambient = 0.00
        obj = 0.00
    return(ambient, obj)

def collectTemps(num, ser):
    t0 = time.time()
    ambient_arr = np.zeros(num)
    object_arr = np.zeros(num)
    for i in range(0, num):
        ambient, obj = returnTemps(retrieveData(ser))
        ambient_arr[i] = ambient
        object_arr[i] = obj
    t1 = time.time()
    meas_time = t1-t0
    t = np.linspace(0, meas_time, len(ambient_arr))
    return(ambient_arr, object_arr, t)

def plotTemps(ambient_arr, object_arr, t):
    plt.scatter(t, ambient_arr)
    plt.scatter(t, object_arr)
    plt.title("Temperature Measurements")
    plt.xlabel("Time (s)")
    plt.ylabel("Temperature (F)")
    plt.legend(["Ambient", "Object"])
    plt.grid()
    plt.show()



def main():
    print("Please enter a COM port (COM4, COM5, ...): ")
    port = input()
    ser = serial.Serial(port,9600, timeout = 1)
    print("Connection successful.")
    while(True):
        print("Press 'Enter' to obtain a data point.")
        print("Or, enter a number of datapoints to graph over time.") 
        print("Or, enter 'exit' to close.")
        command = input()
        if(command == 'exit'):
            sys.exit()

        elif(command == ""):
            ambient, obj = returnTemps(retrieveData(ser))
            print("Ambient temp (F): " + str(ambient))
            print("Object temp (F): " + str(obj))

        elif(command == "."):
            data = retrieveData(ser)
            ambient = data[0:4]
            obj = data[6:len(data)]
            print("Ambient temp voltage (mV): " + str(ambient))
            print("Object temp voltage (mV): " + str(obj))

        elif(int(command) > 0 and int(command) < 1000):
            ambient_arr, object_arr, t = collectTemps(int(command), ser)
            plotTemps(ambient_arr, object_arr, t)
            np.savetxt("tempData.csv", (ambient_arr, object_arr, t), delimiter=",")

        else:
            print("Invalid entry.")
        


if __name__ == "__main__":
    main()
