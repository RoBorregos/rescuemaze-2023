#!/usr/bin/env python3
#encoding=utf-8

import rospy
from geometry_msgs.msg import Twist
import os, time
import _thread

import math
from math import pi as PI, degrees, radians, sin, cos
import os
import time
import sys, traceback
from serial.serialutil import SerialException
from serial import Serial

import roslib

# from geometry_msgs.msg import Quaternion, Twist, Pose
# from nav_msgs.msg import Odometry
from std_msgs.msg import Int8, Float32MultiArray
# from tf.broadcaster import TransformBroadcaster
# from sensor_msgs.msg import Range, Imu

from std_srvs.srv import Trigger, TriggerResponse
from exploration.srv import GoalStatus, GoalStatusResponse, GoalStatusRequest
from exploration.srv import VLXDist, VLXDistResponse, VLXDistRequest

import struct
import binascii


class Microcontroller:
    ''' Configuration Parameters
    '''
    
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=0.5):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.encoder_count = 0
        self.writeTimeout = timeout
        self.interCharTimeout = timeout / 30.

        self.WAITING_FF = 0
        self.WAITING_AA = 1
        self.RECEIVE_LEN = 2
        self.RECEIVE_PACKAGE = 3
        self.RECEIVE_CHECK = 4
        self.HEADER0 = 0xff
        self.HEADER1 = 0xaa
        
        self.SUCCESS = 0
        self.FAIL = -1

        self.receive_state_ = self.WAITING_FF
        self.receive_check_sum_ = 0
        self.payload_command = b''
        self.payload_ack = b''
        self.payload_args = b''
        self.payload_len = 0
        self.byte_count_ = 0
        self.receive_message_length_ = 0
    
        # Keep things thread safe
        self.mutex = _thread.allocate_lock()

    def connect(self):
        try:
            print("Connecting to Microcontroller on port", self.port, "...")
            self.port = Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout, writeTimeout=self.writeTimeout)
            # The next line is necessary to give the firmware time to wake up.
            time.sleep(1)
            state_, val = self.get_baud()
            if val != self.baudrate:
                time.sleep(1)
                state_, val  = self.get_baud()
                if val != self.baudrate:
                    raise SerialException
            print("Connected at", self.baudrate)
            print("Microcontroller is ready.")

        except SerialException:
            print("Serial Exception:")
            print(sys.exc_info())
            print("Traceback follows:")
            traceback.print_exc(file=sys.stdout)
            print("Cannot connect to Microcontroller!")
            os._exit(1)

    def open(self): 
        ''' Open the serial port.
        '''
        self.port.open()

    def close(self): 
        ''' Close the serial port.
        '''
        self.port.close() 
    
    def send(self, cmd):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        self.port.write(cmd)


    def receiveFiniteStates(self, rx_data):
        if self.receive_state_ == self.WAITING_FF:
            #print str(binascii.b2a_hex(rx_data))
            if rx_data == b'\xff':
                self.receive_state_ = self.WAITING_AA
                self.receive_check_sum_ =0
                self.receive_message_length_ = 0
                self.byte_count_=0
                self.payload_ack = b''
                self.payload_args = b''
                self.payload_len = 0


        elif self.receive_state_ == self.WAITING_AA :
             if rx_data == b'\xaa':
                 self.receive_state_ = self.RECEIVE_LEN
                 self.receive_check_sum_ = 0
             else:
                 self.receive_state_ = self.WAITING_FF

        elif self.receive_state_ == self.RECEIVE_LEN:
             self.receive_message_length_, = struct.unpack("B",rx_data)
             self.receive_state_ = self.RECEIVE_PACKAGE
             self.receive_check_sum_ = self.receive_message_length_
        elif self.receive_state_ == self.RECEIVE_PACKAGE:
             if self.byte_count_==0:
                 self.payload_ack = rx_data
             else:
                 self.payload_args += rx_data
             uc_tmp_, = struct.unpack("B",rx_data)
             self.receive_check_sum_ = self.receive_check_sum_ + uc_tmp_
             self.byte_count_ +=1
             if self.byte_count_ >= self.receive_message_length_:
                 self.receive_state_ = self.RECEIVE_CHECK

        elif self.receive_state_ == self.RECEIVE_CHECK:
            self.receive_state_ = self.WAITING_FF
            return 1 
        else:
            self.receive_state_ = self.WAITING_FF
        return 0

    def recv(self, timeout=0.5):
        timeout = min(timeout, self.timeout)
        ''' This command should not be used on its own: it is called by the execute commands   
            below in a thread safe manner.  Note: we use read() instead of readline() since
            readline() tends to return garbage characters from the Microcontroller
        '''
        c = ''
        value = ''
        attempts = 0
        c = self.port.read(1)
        while self.receiveFiniteStates(c) != 1:
            c = self.port.read(1)
            attempts += 1
            if attempts * self.interCharTimeout > timeout:
                return 0
        return 1
            
    def recv_ack(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        ack = self.recv(self.timeout)
        return ack == 'OK'

    def execute(self, cmd):
        ''' Thread safe execution of "cmd" on the Microcontroller returning a single integer value.
        '''
        self.mutex.acquire()
        
        try:
            self.port.flushInput()
        except:
            pass
        
        ntries = 1
        attempts = 0
        
        try:
            self.port.write(cmd)
            res = self.recv(self.timeout)
            while attempts < ntries and res !=1 :
                try:
                    self.port.flushInput()
                    self.port.write(cmd)
                    res = self.recv(self.timeout)
                    #print "response : " + str(binascii.b2a_hex(res))
                except:
                    print("Exception executing command: " + str(binascii.b2a_hex(cmd)))
                attempts += 1
        except:
            self.mutex.release()
            print("Exception executing command: " + str(binascii.b2a_hex(cmd)))
            return 0
        
        self.mutex.release()
        return 1
                                 

    def get_baud(self):
        ''' Get the current baud rate on the serial port.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x00) + struct.pack("B", 0x01)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           val, = struct.unpack('I', self.payload_args)
           return  self.SUCCESS, val 
        else:
           # print("ACK", self.payload_ack, self.payload_ack == b'\x00', self.execute(cmd_str)==1)
           return self.FAIL, 0

    def get_VLX(self):
        ''' Get the current VLX on the serial port.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x01) + struct.pack("B", 0x02)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           valFront, valRight, valLeft = struct.unpack('3f', self.payload_args)
           return  self.SUCCESS, valFront, valRight, valLeft 
        else:
           # print("ACK", self.payload_ack, self.payload_ack == b'\x00', self.execute(cmd_str)==1)
           return self.FAIL, 0, 0, 0
        
    def get_goal_state(self):
        ''' Get the current goal state on the serial port.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x02) + struct.pack("B", 0x03)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           val, = struct.unpack('f', self.payload_args)
           return  self.SUCCESS, val 
        else:
           # print("ACK", self.payload_ack, self.payload_ack == b'\x00', self.execute(cmd_str)==1)
           return self.FAIL, False
        
    def set_goal(self, goal):
        ''' Set the current goal state on the serial port.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x05, 0x03) + struct.pack("i", goal) + struct.pack("B", 0x04) 
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           return  self.SUCCESS
        else:
           # print("ACK", self.payload_ack, self.payload_ack == b'\x00', self.execute(cmd_str)==1)
           return self.FAIL
        
    def send_lidar(self, df, db, dr, dl):
        ''' Send the current lidar state on the serial port.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x11, 0x04) + struct.pack("4f", df, db, dr, dl) + struct.pack("B", 0x05)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           return  self.SUCCESS
        else:
            # print("ACK", self.payload_ack, self.payload_ack == b'\x00', self.execute(cmd_str)==1)
            return self.FAIL
        
    def get_lidar(self):
        ''' Get the lidar distances detected in arduino. Used for debugging
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x07) + struct.pack("B", 0x08)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
            valFront, valBack, valRight, valLeft = struct.unpack('4f', self.payload_args)
            return  self.SUCCESS , valFront, valBack, valRight, valLeft
        else:
            # print("ACK", self.payload_ack, self.payload_ack == b'\x00', self.execute(cmd_str)==1)
            return self.FAIL
        
    def send_victims(self, victims):
        ''' Send the current victims state on the serial port.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x05, 0x05) + struct.pack("i", victims) + struct.pack("B", 0x06)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           return  self.SUCCESS
        else:
            # print("ACK", self.payload_ack, self.payload_ack == b'\x00', self.execute(cmd_str)==1)
            return self.FAIL
        
    def get_start_state(self):
        ''' Get the current start state on the serial port.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x06) + struct.pack("B", 0x07)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           val, = struct.unpack('?', self.payload_args)
           return  self.SUCCESS, val 
        else:
           # print("ACK", self.payload_ack, self.payload_ack == b'\x00', self.execute(cmd_str)==1)
           return self.FAIL, False
           

def goal_callback(data):
    global controller
    controller.set_goal(data.data)    

def lidar_callback(data):
    global controller
    controller.send_lidar(data.data[0], data.data[1], data.data[2], data.data[3])

def victims_callback(data):
    global controller
    controller.send_victims(data.data)

def goal_status(req):
    global controller
    return GoalStatusResponse(controller.get_goal_state()[1])

def get_vlx(req):
    global controller

    dist = controller.get_VLX()
    return VLXDistResponse(dist[1], dist[2], dist[3])

def start_status(req):
    global controller
    return TriggerResponse(controller.get_start_state()[1], "Start status")

if __name__ == '__main__':
    rospy.init_node('serial_node')

    name = rospy.get_name() + '/'
    port = ""
    baud = ""
    timeout = ""

    if rospy.has_param(name + 'port'):
        port = rospy.get_param(name + 'port')

    if rospy.has_param(name + 'baud'):
        baud = rospy.get_param(name + 'baud')

    if rospy.has_param(name + 'timeout'):
        timeout = rospy.get_param(name + 'timeout')

    rospy.loginfo("Connecting to serial port %s at %d baud" % (port, baud))

    rospy.Subscriber("/unit_movement", Int8, goal_callback)
    rospy.Subscriber("/lidar_serial", Float32MultiArray, lidar_callback)
    rospy.Subscriber("/dispenser", Int8, victims_callback)

    # Goal status service
    s = rospy.Service('/get_goal_status', GoalStatus, goal_status)
    # VLX service
    s2 = rospy.Service('/get_vlx', VLXDist, get_vlx)
    # Robot start service
    s3 = rospy.Service('/get_start_status', Trigger, start_status)

    global controller

    controller = Microcontroller(port=port, baudrate=baud, timeout=timeout)
    controller.connect()

    # Tests
    while (1):
        print("Loop")
        # print(controller.get_baud())
        #print(controller.get_VLX())
        #print(controller.get_baud())
        #print(controller.get_goal_state())
        #print(controller.get_start_state())
        #controller.set_goal(5)
        #print(controller.get_goal_state())
        controller.send_lidar(1.5, 4.9, -3.0, 4.3)
        print(controller.get_lidar())
        rospy.sleep(1)



        