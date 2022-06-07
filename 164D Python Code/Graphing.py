# -*- coding: utf-8 -*-
"""
Created on Tue Jun  7 01:16:23 2022

@author: yogur
"""
import numpy as np
import matplotlib.pyplot as plt
from io import StringIO
import csv

def plotTemps(ambient_arr, object_arr, t):
    plt.scatter(t, ambient_arr)
    plt.scatter(t, object_arr)
    plt.title("Temperature Measurements")
    plt.xlabel("Time (s)")
    plt.ylabel("Temperature (F)")
    plt.legend(["Ambient", "Object"])
    plt.ylim([45,70])
    plt.grid()
    plt.show()
    


path = 'C:/Users/yogur/Documents/GitHub/164D-Project/164D Python Code/minifridgetest.csv'
with open(path, 'r') as f:
    reader = csv.reader(f, delimiter=',')
    data = np.array(list(reader)).astype(float)
ambient_arr = data[0,:]
object_arr = data[1,:]
t = data[2,:]    
plotTemps(ambient_arr, object_arr, t)

