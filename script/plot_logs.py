#! /usr/bin/env python3
import matplotlib.pyplot as plt
import os
dt = 3 * pow(10, -3)
def plot_log_file(filename:str):
    file = open(filename)
    title = None
    t = 0
    time = []
    j1 = []
    j2 = [] 
    j3 = [] 
    j4 = []
    ref1 = []
    ref2 = []
    for line in file:
        #Filter out the logs section
        log_data = line.split(" ")
        if title is None:
            title = log_data[2][1:-1]
        data = log_data[-1]
        data = data.split(",")
        time.append(t)
        j1.append(float(data[0]))
        j2.append(float(data[1]))
        j3.append(float(data[2]))
        j4.append(float(data[3]))
        try:
            ref1.append(float(data[4]))
            ref2.append(float(data[5]))
        except:
            pass
        t+=dt
    plt.figure()
    plt.grid(True)    
    plt.title(str(title))
    plt.ylabel("y-Axis")
    plt.xlabel("Time (s)")
    plt.plot(j1)
    plt.plot(j2)
    plt.plot(j3)
    plt.plot(j4)
    # plt.plot(ref1)
    # plt.plot(ref2)
    plt.legend(["Left Hip", "Left Knee", "Right Hip", "Right Knee"])



if __name__ == "__main__":
    home_path = (os.path.expanduser("~"))
    plot_log_file(home_path  + "/exo_ws/logs/d_logs.log")
    plot_log_file(home_path  + "/exo_ws/logs/dref_logs.log")
    plot_log_file(home_path  + "/exo_ws/logs/effort_logs.log")
    plot_log_file(home_path  + "/exo_ws/logs/err_logs.log")
    plot_log_file(home_path  + "/exo_ws/logs/p_logs.log")
    plot_log_file(home_path  + "/exo_ws/logs/pdout_logs.log")
    plot_log_file(home_path  + "/exo_ws/logs/pos_logs.log")
    plot_log_file(home_path  + "/exo_ws/logs/posd_logs.log")
    plot_log_file(home_path  + "/exo_ws/logs/ref_logs.log")
    plt.show()
