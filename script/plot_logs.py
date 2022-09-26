#! /usr/bin/env python3
import sre_parse
from turtle import home
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.backends.backend_tkagg import NavigationToolbar2Tk

# import rclpy
# from rclpy.node import Node
import os
import tkinter as tk
from tkinter import filedialog as fd
from tkinter import Menu
dt = 3 * pow(10, -3)

#TODO:
#   Create a way to close existing plots
#       - FIx Master closing as well bug
#   Create an auto-backup thing
#   Plot individual joints ect

def plot_log_file(filename:str) -> int:
    """Plots data from the hip joint log files, only works for those
    :returns: Figure Number"""
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
    figure = plt.figure()
    plt.grid(True)    
    plt.title(str(title))
    plt.ylabel("y-Axis")
    plt.xlabel("Sample (#)")
    plt.plot(j1)
    plt.plot(j2)
    plt.plot(j3)
    plt.plot(j4)
    plt.plot(ref1)
    plt.legend(["Left Hip", "Left Knee", "Right Hip", "Right Knee"])
    return figure

def plot_dt_log_file(filename:str):
    """Plots the dt (Calculated change in time) over each sample"""
    file = open(filename)
    title = None
    dt_data = []

    for line in file:
        log_data = line.split(" ")
        if title is None:
            title = log_data[2][1:-1]
        data = log_data[-1]
        data = data.split(",")
        dt_data.append(float(data[0])) 
    plt.figure()
    plt.grid(True)
    plt.title(str(title))
    plt.ylabel("Sample Time <s>")
    plt.xlabel("Sample(#)")
    plt.plot(dt_data)


# def init_plotter_node(args = None):
#     rclpy.init(args=args)
#     node = PlotterNode()
#     rclpy.spin(node)

class Plotter():
    def __init__(self) -> None:
        self.homeDir =  (os.path.expanduser("~"))
        self.plotArray = [] #Used to track which plots are open
        self.figureNum = 1 #Number used to track which plots 
    
    def construct_GUI(self):
        self.root = tk.Tk()
        self.root.title("VSAL Plot")
        menubar = Menu(self.root)
        self.root.config(menu=menubar)
        file_menu = Menu(menubar)
        preset_menu = Menu(menubar)
        file_menu.add_command(
            label="Exit",
            command = self.close_gui
        )
        file_menu.add_command(
            label="Open File",
            command=self.open_file
        )
        menubar.add_cascade(
            label="File", 
            menu=file_menu
        )
        self.root.protocol("WM_DELETE_WINDOW", self.close_gui)
        #Set up parent frame
        self.master = tk.Frame(self.root, width=500, height=500, background="white")
        self.master.pack(fill=tk.BOTH, expand=True)
        self.root.mainloop()
    def open_file(self):
        """Create a new file, to be used with AI_EXO custom log files only"""
        #Create a new frame
        newFigure = FigureFrame(self.master, self.figureNum)
        self.figureNum +=1

    
    def close_gui(self):
        plt.close('all')
        self.root.destroy()


class FigureFrame():
    def __init__(self, master, figureNum):
        self.master = master
        self.figureNum = figureNum
        self.construct_frame()

    def construct_frame(self):
        self.frame = tk.Frame(self.master)
        figLabel = tk.Label(self.frame, text="Figure %d"%self.figureNum)
        figLabel.pack(side=tk.TOP,anchor=tk.CENTER)

        #Create a close button
        filename = fd.askopenfilename(
            title="Select debug log",
            initialdir="/home/"
        )
        self.fig = plot_log_file(filename)
        graph = FigureCanvasTkAgg(self.fig, self.frame)
        NavigationToolbar2Tk(graph, self.frame)
        graph.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH)
        #Create a close button
        closeButton = tk.Button(self.frame, text="Close Figure", command = self.close_figure)
        closeButton.pack(side=tk.TOP, anchor=tk.W)

        
        #Pack the frame
        if (self.figureNum%2 == 0):
            self.frame.pack(side=tk.BOTTOM, fill =tk.BOTH)
        else:
            self.frame.pack(side=tk.LEFT, fill=tk.BOTH)
    def close_figure(self):
        """Used to close the figure and destroy the associated frame"""
        plt.close(self.fig)
        self.frame.destroy()



if __name__ == "__main__":
    plter = Plotter()
    plter.construct_GUI()
    # home_path = (os.path.expanduser("~"))
    # plot_log_file(home_path  + "/exo_ws/logs/d_logs.log")
    # plot_log_file(home_path  + "/exo_ws/logs/dref_logs.log")
    # plot_log_file(home_path  + "/exo_ws/logs/effort_logs.log")
    # plot_log_file(home_path  + "/exo_ws/logs/err_logs.log")
    # plot_log_file(home_path  + "/exo_ws/logs/p_logs.log")
    # plot_log_file(home_path  + "/exo_ws/logs/pdout_logs.log")
    # plot_log_file(home_path  + "/exo_ws/logs/pos_logs.log")
    # plot_log_file(home_path  + "/exo_ws/logs/posd_logs.log")
    # plot_log_file(home_path  + "/exo_ws/logs/ref_logs.log")
    # plot_dt_log_file(home_path + "/exo_ws/logs/dt_logs.log")
    # plt.show()
