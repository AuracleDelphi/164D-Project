import tkinter as tk
import random
import serial

ser = serial.Serial("COM4",9600, timeout = 1)

def retreiveData():
    ser.write(b'1')
    data = ser.readline().decode('ascii')
    ser.write(b'0')
    return data


class Example(tk.Frame):
    def __init__(self, *args, **kwargs):
        tk.Frame.__init__(self, *args, **kwargs)
        self.canvas = tk.Canvas(self, background="black")
        self.canvas.pack(side="top", fill="both", expand=True)

        # create lines for velocity and torque
        self.velocity_line = self.canvas.create_line(0,0,0,0, fill="red")
        self.torque_line = self.canvas.create_line(0,0,0,0, fill="blue")

        # start the update process
        self.update_plot()

    def update_plot(self):
        s = retreiveData()
        v = s[0:4]
        t = s[6:len(s)]
        self.add_point(self.velocity_line, v)
        self.add_point(self.torque_line, t)
        self.canvas.xview_moveto(1.0)
        self.after(10, self.update_plot)

    def add_point(self, line, y):
        coords = self.canvas.coords(line)
        x = coords[-2] + 1
        coords.append(x)
        coords.append(y)
        coords = coords[-200:] # keep # of points to a manageable size
        self.canvas.coords(line, *coords)
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

if __name__ == "__main__":
    root = tk.Tk()
    Example(root).pack(side="top", fill="both", expand=True)
    root.mainloop()
