import sys 
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import uic
import rospy
import std_msgs
from sensor_msgs.msg import JointState

class Window(QMainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        # Loads the window made in QtDesigner
        uic.loadUi("MainWindow.ui", self)

        self.pub = rospy.Publisher('setpoint_angles', JointState, queue_size = 10)
        self.joints = JointState()
        self.joints.name = ['angle_1', 'angle_2', 'angle_3']

        # Enables go to home position
        self.goToHomeButton.setEnabled(True)

        # goToHomeButton callback function
        self.goToHomeButton.clicked.connect(self.goToHomeMsg)

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