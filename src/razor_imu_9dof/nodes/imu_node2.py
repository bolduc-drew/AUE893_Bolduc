#!/usr/bin/env python
#
# Code created/modified by Andrew Bolduc for the use with Sparkfun IMU SEN-14001 
# Originally adapted from imu_node.py by Tang Tiong Yew 
# Copyright (c) 2012, Tang Tiong Yew All rights reserved.
#
import string
import math
import sys
import rospy
import serial

#from time import time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
# from dynamic_reconfigure.server import Server
degrees2rad = math.pi/180.0
rospy.init_node("razor_node")
pub = rospy.Publisher('imu', Imu, queue_size=1)
pub2 = rospy.Publisher('traj', Point, queue_size=1)
pub3 = rospy.Publisher('vel', Point, queue_size=1)

velMsg = Point()
trajMsg = Point()
imuMsg = Imu()

accel_factor = 9.806
seq = 0.0
samplePeriod = 0
velx = 0.0
vely = 0.0
velz = 0.0
last_velx = 0.0
last_vely = 0.0
last_velz = 0.0
posx = 0.0
posy = 0.0
posz = 0.0
last_posx = 0.0
last_posy = 0.0
last_posz = 0.0
last_time = 0.0
last_a_x=0
last_a_y=0
last_a_z = 0

default_port='/dev/ttyACM0'
port = rospy.get_param('~port', default_port)

# Check COM port and baud rate
rospy.loginfo("Opening %s...", port)
try:
    ser = serial.Serial(port=port, baudrate=115200, timeout=1)
except serial.serialutil.SerialException:
    rospy.logerr("IMU not found at port "+port + ". Did you specify the correct port?")
    #exit
    sys.exit(0)

rospy.loginfo("Giving the razor IMU board 5 seconds to boot...")
rospy.sleep(5)

#discard old input
#flush manually
# discard = ser.readlines() 
# rospy.loginfo("Flushing first 200 IMU entries...")

rospy.loginfo("Calibrating")
for x in range(0, 100):
    line = ser.readline()
    line = line.replace("#YPRAG=","")   # Delete "#YPRAG="
    words = string.split(line,",")    # Fields split
    if len(words) > 2:
        acc_x = math.fabs(float(words[1])* accel_factor)
        acc_y = math.fabs(float(words[2]) * accel_factor) 
        acc_z = math.fabs(float(words[3])* accel_factor  + accel_factor) 
        if x > 1:
            if math.fabs(acc_x) < 0.8*accel_factor: 
                if acc_x> last_a_x:
                    last_a_x = acc_x
                    # last_a_x = (last_a_x*(x-1)+acc_x)/x
            if math.fabs(acc_y) < 0.8*accel_factor:
                if acc_y> last_a_y:
                    last_a_y = acc_y 
                    # last_a_y = (last_a_y*(x-1)+acc_y)/x
            if math.fabs(acc_z) < 0.8*accel_factor: 
                if acc_z> last_a_z:
                    last_a_z = acc_z 
                    # last_a_z = (last_a_z*(x-1)+acc_z)/x
           
        
rospy.loginfo(last_a_x)
rospy.loginfo(last_a_y)
rospy.loginfo(last_a_z)

rospy.loginfo("Publishing IMU data... /imu /vel /traj")

while not rospy.is_shutdown():
    line = ser.readline()
    line = line.replace("#YPRAG=","")   # Delete "#YPRAG="
    words = string.split(line,",")    # Fields split
    if len(words) > 2:
        
        # linear acceleration in m/s2
        if math.fabs(float(words[1])* accel_factor)>last_a_x and math.fabs(float(words[1])* accel_factor) <0.8*accel_factor:
            acc_x = float(words[1])* accel_factor
        else: 
            acc_x = 0
        if math.fabs(float(words[2])* accel_factor)>last_a_y and math.fabs(float(words[2])* accel_factor) <0.8*accel_factor:
            acc_y = float(words[2]) * accel_factor 
        else: 
            acc_y = 0
        if math.fabs(float(words[3])* accel_factor  + accel_factor)>last_a_z and math.fabs(float(words[3])* accel_factor  + accel_factor) < 0.8*accel_factor:
            acc_z = float(words[3])* accel_factor  + accel_factor 
        else:
            acc_z = 0
        
        imuMsg.linear_acceleration.x = acc_x
        imuMsg.linear_acceleration.y = acc_y
        imuMsg.linear_acceleration.z = acc_z
        # angular_velocity in rad/sec
        imuMsg.angular_velocity.x = float(words[4])
        imuMsg.angular_velocity.y = float(words[5]) 
        imuMsg.angular_velocity.z = float(words[6]) 
        # orientation in quaternions
        imuMsg.orientation.x = float(words[7])
        imuMsg.orientation.y = float(words[8]) 
        imuMsg.orientation.z = float(words[9]) 
        
        if seq < 1:
            time_offset = float(words[0])
            rospy.loginfo(time_offset)
        elif seq > 0:
            samplePeriod = (float(words[0])-time_offset)-last_time
            last_time = float(words[0]) - time_offset
            velx = last_velx + float(words[1])*samplePeriod
            vely = last_vely + float(words[2])*samplePeriod
            velz = last_velz + float(words[3])*samplePeriod
            last_velx = velx
            last_vely = vely
            last_velz = velz
            velMsg.x = last_posx
            velMsg.y = last_posy
            velMsg.z = last_posz
        elif seq > 1:
            posx = last_posx + velx*samplePeriod
            posy = last_posy + vely*samplePeriod
            posz = last_posz + velz*samplePeriod
            last_posx = posx
            last_posy = posy
            last_posz = posz
            trajMsg.x = last_posx
            trajMsg.y = last_posy
            trajMsg.z = last_posz
        # Publish message


    imuMsg.header.stamp= rospy.Time.now()
    imuMsg.header.frame_id = 'base_imu_link'
    imuMsg.header.seq = seq
    seq = seq + 1
    pub.publish(imuMsg)
    pub2.publish(trajMsg)
    pub3.publish(velMsg)

ser.close