import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import platform
import datetime as dt
import os
fig, ax = plt.subplots()
line, = ax.plot(np.random.rand(10))
ax.set_ylim(0, 1030)
xdata, ydata = [0]*100, [0]*100
SerialIn = serial.Serial('/dev/ttyUSB0',9600)
# save_name = "/home/pi/Documents"

def update(data):
    line.set_ydata(data)
    return line,

def run(data):
    global xdata, ydata
    x,y = data
    if (x == 0):
        xdata = [0]*100
        ydata = [0]*100
    del xdata[0]
    del ydata[0]
    xdata.append(x)
    ydata.append(y)
    line.set_data(xdata, ydata)
    return line,

def data_gen():
    x = 9
    while True:
        if (x >= 9):
            x = 0
        else:
            x += 0.1

        try:
            inRaw = SerialIn.readline()
            inInt = int(inRaw)
        except:
            inInt = 0

        yield x, inInt

        #saving figure as time
        # filename = dt.datetime.now().strftime("%Y-%m-%d_%H.%M.%S")
        # completed_video_name = os.path.join(save_name, filename + ".png")
        # plt.savefig(completed_video_name)
ani = animation.FuncAnimation(fig, run, data_gen, interval=0, blit=True)
plt.show()
