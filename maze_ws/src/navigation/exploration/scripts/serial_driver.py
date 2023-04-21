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

from geometry_msgs.msg import Quaternion #, Twist, Pose
# from nav_msgs.msg import Odometry
from std_msgs.msg import Int8, Float32, Float32MultiArray
# from tf.broadcaster import TransformBroadcaster
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler

from std_srvs.srv import Trigger, TriggerResponse
from exploration.srv import GoalStatus, GoalStatusResponse, GoalStatusRequest
from exploration.srv import VLXDist, VLXDistResponse, VLXDistRequest
from nav_main.srv import GetWallsDist, GetWallsDistResponse, GetWallsDistRequest

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
        
    def get_goal(self):
        ''' Get the current goal state on the serial port.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x08) + struct.pack("B", 0x09)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           val, = struct.unpack('i', self.payload_args)
           return  self.SUCCESS, val 
        else:
           # print("ACK", self.payload_ack, self.payload_ack == b'\x00', self.execute(cmd_str)==1)
           return self.FAIL, False
        
    def get_imu_val(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x09) + struct.pack("B", 0x0A)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           yaw, yaw_vel, x_acc, y_acc, z_acc, = struct.unpack('5f', self.payload_args)
           return  self.SUCCESS, yaw, yaw_vel, x_acc, y_acc, z_acc
        else:
           return self.FAIL, 0, 0, 0, 0, 0
    
    def get_lidar_use(self):
        ''' Get if lidar is being used.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x0A) + struct.pack("B", 0x0B)
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

def arduino_lidar_callback(data):
    global controller
    print(controller.get_lidar())

def get_cur_goal(req):
    global controller
    return GoalStatusResponse(controller.get_goal()[1])

def goal_status(req):
    global controller
    pub_imu()
    return GoalStatusResponse(controller.get_goal_state()[1])

def get_vlx(req):
    global controller

    dist = controller.get_VLX()
    return VLXDistResponse(dist[1], dist[2], dist[3])

def start_status(req):
    global controller
    return TriggerResponse(controller.get_start_state()[1], "Start status")

def lidar_status(req):
    global controller
    return TriggerResponse(controller.get_lidar_use()[1], "Lidar status")

def pub_imu(data):
    global controller, imuPub, imuAnglePub, imu_frame_id
    controller.get_imu_val()
    
    try:
        stat_, yaw, yaw_vel, acc_x, acc_y, acc_z = controller.get_imu_val()
        # Degree to radians
        yaw = yaw * 3.1415926 / 180.0
        isValid = True
        if yaw == 0.0:
            isValid = False

        # print("yaw:  " + str(yaw)+" yaw_vel: " + str(yaw_vel)+" acc_x: " + str(acc_x)+" acc_y: " + str(acc_y)+" acc_z: " + str(acc_z))
        if isValid:
            imu_data = Imu()  
            imu_data.header.stamp = rospy.Time.now()
            imu_data.header.frame_id = imu_frame_id 
            imu_data.orientation_covariance[0] = 1000000
            imu_data.orientation_covariance[1] = 0
            imu_data.orientation_covariance[2] = 0
            imu_data.orientation_covariance[3] = 0
            imu_data.orientation_covariance[4] = 1000000
            imu_data.orientation_covariance[5] = 0
            imu_data.orientation_covariance[6] = 0
            imu_data.orientation_covariance[7] = 0
            imu_data.orientation_covariance[8] = 0.000001

            newquat = quaternion_from_euler(0, 0, yaw)
            imu_data.orientation = Quaternion(newquat[0], newquat[1], newquat[2], newquat[3])
            imu_data.linear_acceleration_covariance[0] = -1
            imu_data.angular_velocity_covariance[0] = -1

            imu_data.linear_acceleration.x = acc_x
            imu_data.linear_acceleration.y = acc_y
            imu_data.linear_acceleration.z = acc_z

            imu_data.angular_velocity.x = 0.0
            imu_data.angular_velocity.y = 0.0
            imu_data.angular_velocity.z = yaw_vel
            imuPub.publish(imu_data)
            imuAnglePub.publish(yaw)
    except:
        rospy.logerr("IMU exception triggered")
        return


if __name__ == '__main__':

    only_test_lidar = False

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

    
    if only_test_lidar:
        rospy.wait_for_service('get_walls_dist')
        get_walls = rospy.ServiceProxy('get_walls_dist', GetWallsDist)
    else:
        rospy.Subscriber("/lidar_serial", Float32MultiArray, lidar_callback)
    
    rospy.Subscriber("/unit_movement", Int8, goal_callback)
    rospy.Subscriber("/dispenser", Int8, victims_callback)
    rospy.Subscriber("/lidar_data", Int8, arduino_lidar_callback)

    # Goal status service
    s = rospy.Service('/get_goal_status', GoalStatus, goal_status)
    # VLX service
    s2 = rospy.Service('/get_vlx', VLXDist, get_vlx)
    # Robot start service
    s3 = rospy.Service('/get_start_status', Trigger, start_status)

    s4 = rospy.Service('/get_cur_goal', GoalStatus, get_cur_goal)

    s5 = rospy.Service('/get_lidar_status', Trigger, lidar_status)

    global controller, imuPub, imuAnglePub, imu_frame_id

    imuPub = rospy.Publisher('imu', Imu, queue_size=5)
    imuAnglePub = rospy.Publisher('imu_angle', Float32, queue_size=5)
    imu_frame_id = rospy.get_param('imu_frame_id', 'imu_base')

    controller = Microcontroller(port=port, baudrate=baud, timeout=timeout)
    controller.connect()
    rospy.spin()

    if only_test_lidar:
        # get lidar 
        distances = get_walls()
        controller.send_lidar(distances.front, distances.back, distances.right, distances.left)
        print(controller.get_lidar())
        rospy.sleep(1)
        
        while not rospy.is_shutdown():
            # get lidar 
            distances = get_walls()
            controller.send_lidar(distances.front, distances.back, distances.right, distances.left)
            print(controller.get_lidar())
            rospy.sleep(1)

        