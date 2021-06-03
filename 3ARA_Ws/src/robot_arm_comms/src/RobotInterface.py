#!/usr/bin/env python

import Tkinter as tk
import sys
import os
import rospy 
from std_msgs.msg import Float32
import threading

# Global variables
angle_1 = 0

# Private methods
# Publish data into 'setpoint_angles' ROS topic
def publishData():
    rospy.init_node('RobotInterface', anonymous=True)
    pub = rospy.Publisher('setpoint_angles', Float32, queue_size=10)
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(angle_1)
        rate.sleep()

# Gets the value of the scale widget when it changes
def updateValue1(val):
    global angle_1
    angle_1 = val
    print(angle_1)

def windowGUI():
    # Creation of the window
    mainWindow = tk.Tk()
    mainWindow.title("Robot Arm Controls")
    mainWindow.geometry('900x500')

    # Slide to control first joint
    slider_1 = tk.Scale(mainWindow, from_=0, to=100, orient="horizontal", length=300, width=20, command=updateValue1)
    label_1 = tk.Label(mainWindow, text="Joint 1")
    label_1.grid(row=0, column=2, pady = 10)
    slider_1.grid(row = 1, column = 2)

    # Slide to control second joint
    slider_2 = tk.Scale(mainWindow, from_=0, to=100, orient="horizontal", length=300, width=20)
    label_2 = tk.Label(mainWindow, text="Joint 2")
    label_2.grid(row=8, column=2, pady = 10)
    slider_2.grid(row = 9, column = 2)
    
    # Slide to control third joint
    slider_3 = tk.Scale(mainWindow, from_=0, to=100, orient="horizontal", length=300, width=20)
    label_3 = tk.Label(mainWindow, text="Joint 3")
    label_3.grid(row=16, column=2, pady = 10)
    slider_3.grid(row = 17, column = 2) 

    init_button = tk.Button(mainWindow, text="Iniciar", relief="groove", borderwidth=5)
    init_button.grid(row=30, column=5, pady=40)

    mainWindow.mainloop()


publishThread = threading.Thread(target=publishData())
publishThread.start()

windowThread = threading.Thread(target=windowGUI())
windowThread.start()




