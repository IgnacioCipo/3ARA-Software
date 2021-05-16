#!/usr/bin/env python

import serial
import threading
import time
import rospy
import sys
from std_msgs.msg import Float32
from std_msgs.msg import String
import struct

class SerialComms:
    def __init__(self):
        self.device_port = rospy.get_param('~port', '/dev/ttyS5')           #COM PORT - Default -> 5
        self.baudrate = rospy.get_param('~baudrate', '57600')               #Baudrate 
        self.timeout = float( rospy.get_param('~timeout', '10'))            # 10 Hz 
        self.angles_topic = rospy.get_param('~angles_topic', 'angles')
        self.comm_freq = float(rospy.get_param('~comm_freq', '15'))         # 15 Hz for communication

        rospy.set_param("~port", self.device_port)
        rospy.set_param("~baudrate", self.baudrate)
        rospy.set_param("~angles_topic", self.angles_topic)

        self.data_ready = False
        self.data_ok = False
        self.serial = serial.Serial()
        self.thread_stop = threading.Event()

        try:
            rospy.loginfo("Starting communication on serial port: "+self.device_port+" at "+self.baudrate+" bauds")
            self.serial = serial.Serial(self.device_port, self.baudrate, timeout=self.timeout)
        except:
            rospy.logerr("Error ocurred starting serial comunnication")
            self.serial.close()
            sys.exit(0)

        self.publisher = rospy.Publisher(self.angles_topic, String, queue_size=10)
        #self.timer_angles = rospy.Timer(rospy.Duration(1.0/self.comm_freq), self.anglesPub) 
        self.serialWriter = rospy.Timer(rospy.Duration(2), self.sendToSTM)       # Each 2 seconds

    # serial reading loop
    def serialHandle(self):
        while(not self.thread_stop.is_set()):
            if self.serial.inWaiting() > 0:
                # Reads 12 bytes
                data = self.serial.read(12)
                self.data_ok = True
                print('Angle 1: '+str(struct.unpack('f', data[0:4])[0])+' || Angle 2: '+str(struct.unpack('f', data[4:8])[0])+
                ' || Angle 3: '+str(struct.unpack('f', data[8:12])[0]))
                angles_buffer = [str(struct.unpack('f', data[0:4])[0]), str(struct.unpack('f', data[4:8])[0]), str(struct.unpack('f', data[8:12])[0])]
                self.anglesPub(angles_buffer)
                # Clean input buffer
                self.serial.reset_input_buffer()
                '''
                rx_buffer = self.serial.readlines(self.serial.inWaiting())
                print(rx_buffer)
                self.serial.reset_input_buffer()
                '''
    # Publish angles in ROS topic called 'angles'
    def anglesPub(self, buffer):
        if self.data_ok:
            self.data_ok = False
            self.publisher.publish("Angle 1: "+buffer[0]+" || Angle 2: "+buffer[1]+ "|| Angle 3: "+buffer[2])
            '''
            for angles in buffer:
                self.publisher.publish(angles)
            '''
    # Send info to STM32f407
    def sendToSTM(self, Event):
        angle_1 = 45.5
        angle_2 = 50.0
        angle_3 = 55.0
        angle_1_bytes = struct.pack('f', angle_1)
        #tx_buffer = [0xA1, angle_1_bytes[0], angle_1_bytes[1], angle_1_bytes[2], angle_1_bytes[3], 0xB1]
        angle_2_bytes = struct.pack('f', angle_2)
        angle_3_bytes = struct.pack('f', angle_3)
        tx_buffer = [0xA1, angle_1_bytes[0], angle_1_bytes[1], angle_1_bytes[2], angle_1_bytes[3], 
            angle_2_bytes[0], angle_2_bytes[1], angle_2_bytes[2], angle_2_bytes[3],
            angle_3_bytes[0], angle_3_bytes[1], angle_3_bytes[2], angle_3_bytes[3], 0xB1]

        self.serial.write(tx_buffer)
        print("Package sent")
        
    def stopThread(self):
        self.thread_stop.set()

if __name__ == "__main__":

    # ROS init node
    rospy.init_node('CommsScript', disable_signals = True)

    # Create communication object 
    Comms = SerialComms()

    # Create a thread for communication with STM32 uC
    CommsThread = threading.Thread(target=Comms.serialHandle())
    CommsThread.start()

    rospy.spin()
    # If this point is reached, will stop the execution of the thread and close the serial port
    print("Shutting down")
    Comms.stopThread()
    CommsThread.join()
    Comms.serial.close()
    rospy.signal_shutdown("End")

