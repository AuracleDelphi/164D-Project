import serial
import sys
import matplotlib.pyplot as plt
import numpy as np
import time

def retreiveData(ser):
    ser.write(b'1')
    data = ser.readline().decode('ascii')
    ser.write(b'0')
    return data

def returnTemps(data):
    ambient = data[0:4]
    object = data[6:len(data)]
    ambient = float(ambient) * 9/5 + 32
    object = float(object) * 9/5 + 32
    return(ambient, object)

def collectTemps(num, ser):
    t0 = time.time()
    ambient_arr = np.zeros(num)
    object_arr = np.zeros(num)
    for i in range(0, num):
        ambient, object = returnTemps(retreiveData(ser))
        ambient_arr[i] = ambient
        object_arr[i] = object
    t1 = time.time()
    meas_time = t0-t1
    return(ambient_arr, object_arr, meas_time)

def plotTemps(ambient_arr, object_arr, meas_time):
    t = np.linspace(0, meas_time, len(ambient_arr))
    plt.scatter(t, ambient_arr)
    plt.scatter(t, object_arr)
    plt.title("Temperature Measurements")
    plt.xlabel("Time (s)")
    plt.ylabel("Temperature (F)")
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
            ambient, object = returnTemps(retreiveData(ser))
            print("Ambient temp (F): " + str(ambient))
            print("Object temp (F): " + str(object))
        elif(int(command) > 0 and int(command) < 1000):
            ambient_arr, object_arr, meas_time = collectTemps(int(command), ser)
            plotTemps(ambient_arr, object_arr, meas_time)
        else:
            print("Invalid entry.")
        


if __name__ == "__main__":
    main()
