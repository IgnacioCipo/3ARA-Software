#!/usr/bin/env python

import sys 
import os
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic
import rospy
import std_msgs
from sensor_msgs.msg import JointState
import time

class Window(QMainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        ui_path = os.path.dirname(os.path.abspath(__file__))
        # Loads the window made in QtDesigner
        #uic.loadUi("MainWindow.ui", self)
        uic.loadUi(os.path.join(ui_path, "MainWindow.ui"), self)
        self.pub = rospy.Publisher('setpoint_angles', JointState, queue_size = 10)
        self.joints = JointState()
        self.joints.name = ['angle_1', 'angle_2', 'angle_3']

        # Enables go to home position
        self.goToHomeButton.setEnabled(True)

        # Enables routine 1 button
        self.routineButton.setEnabled(True)

        # goToHomeButton callback function
        self.goToHomeButton.clicked.connect(self.goToHomeMsg)

        # Set routineButton callback function
        self.routineButton.clicked.connect(self.executeRoutine1)

        # Callbacks function when sliders changes it's values
        self.jointOneSlider.valueChanged[int].connect(self.slideValueChanged)
        self.jointTwoSlider.valueChanged[int].connect(self.slideValueChanged)
        self.jointThreeSlider.valueChanged[int].connect(self.slideValueChanged)

        # Style for LCD numbers
        self.showLCD_1.setDigitCount(3)
        self.showLCD_2.setDigitCount(3)
        self.showLCD_3.setDigitCount(3)

    def goToHomeMsg(self):
        self.jointOneSlider.setValue(0)
        self.jointTwoSlider.setValue(0)
        self.jointThreeSlider.setValue(0)

    def executeRoutine1(self):
        execution_time = 8000                   # Time to reach the next position
        for i in range(2):                      # Number of times to execute the routine   
            self.jointOneSlider.setValue(45)
            self.jointTwoSlider.setValue(45)
            self.jointThreeSlider.setValue(45)
            delayLoop = QEventLoop()
            QTimer.singleShot(execution_time, delayLoop.quit)
            delayLoop.exec_()
            self.jointOneSlider.setValue(25)
            self.jointTwoSlider.setValue(25)
            self.jointThreeSlider.setValue(25)
            delayLoop = QEventLoop()
            QTimer.singleShot(execution_time, delayLoop.quit)
            delayLoop.exec_()
            print(i)
            
    def slideValueChanged(self):
        #print(self.jointOneSlider.value())
        #print(self.jointTwoSlider.value())
        #print(self.jointThreeSlider.value())
        self.showLCD_1.display(self.jointOneSlider.value())
        self.showLCD_2.display(self.jointTwoSlider.value())
        self.showLCD_3.display(self.jointThreeSlider.value())
        self.joints.position = [self.jointOneSlider.value(), self.jointTwoSlider.value(), self.jointThreeSlider.value()]
        self.pub.publish(self.joints)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    rospy.init_node('robot_gui', anonymous=True)
    controlWindow = Window()
    controlWindow.show()
    app.exec_()