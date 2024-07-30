#!/usr/bin/env python3


import rospy
import serial
from imu_driver.msg import imu_msg
import sys
import numpy as np
from rospy import Time
port =  sys.argv[1]

# Reading the port form terminal command
# port =  sys.argv[1]

port =  sys.argv[1]

if __name__ == '__main__':
    #SENSOR_NAME = "gps"
    rospy.init_node('imu_node')
    serial_port = rospy.get_param(port, '/dev/ttyUSB0')
    serial_baud = rospy.get_param('~baudrate',115200)
    sampling_rate = rospy.get_param('~sampling_rate',40.0)
    
    port = serial.Serial(port, serial_baud, timeout=3.)
    rospy.logdebug("Using imu sensor on port "+serial_port+" at "+str(serial_baud))
    
    imu_pub = rospy.Publisher('/imu', imu_msg, queue_size=10)
    
    rospy.logdebug("Initialization complete")
    rospy.loginfo("Publishing IMU data")
    imu_msg = imu_msg()
    sequence= 0
    imu_msg.Header.frame_id = "IMU1_FRAME" 


# if __name__ == '__main__':
#     rospy.init_node('gps_node')
#     serial_port = rospy.get_param('~port', '/dev/ttyUSB0')
#     serial_baud = rospy.get_param('~baudrate',4800)
#     sampling_rate = rospy.get_param('~sampling_rate',5.0)
  
#     port = serial.Serial(serial_port, serial_baud, timeout=3.)    
#     gps_pub = rospy.Publisher('/gps', imu_msg, queue_size=10)
#     rospy.logdebug("Initialization complete")
#     rospy.loginfo("Publishing gps data")
#     imu_msg = imu_msg()
#     imu_msg.header.seq = 0
#     imu_msg.header.frame_id = "GPS1_Frame" 
   
    try:
        while not rospy.is_shutdown():
            current_time = rospy.get_rostime()
            line = port.readline()
            stringformat = str(line)
            data = list(map(str,stringformat.split(',')))
            #print(current_time.secs,current_time.nsecs)
            if (data[0] == "b'\\r$VNYMR" or data[0]=="b'$VNYMR")and data[2] == '':
                rospy.logwarn("Not Recieveing IMU data ")
                print("NO DATA")
            elif (data[0] == "b'\\r$VNYMR" or data[0]=="b'$VNYMR")and data[2] !='':
                #print("Values:",data)
                #imu_msg.header.stamp = Time(current_time.secs, current_time.nsecs)
                
                yaw = float(data[1])
                pitch = float(data[2])
                roll = float(data[3])
                magX = float(data[4])
                magY = float(data[5])
                magZ = float(data[6])
                accX = float(data[7])
                accY = float(data[8])
                accZ = float(data[9])
                gyroX = float(data[10])
                gyroY = float(data[11])
                gyroZ = float(data[12].split('*')[0])
                qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
                qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
                qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
                qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
                magnitude = (qw**2 + qx**2 + qy**2 + qz**2)**0.5
                imu_msg.Header.stamp.secs = int(current_time.secs)
                imu_msg.Header.stamp.nsecs = int(current_time.nsecs)
                imu_msg.Header.frame_id = 'IMU1_Frame'
                imu_msg.Header.seq=sequence
                imu_msg.IMU.header.seq=sequence
                imu_msg.IMU.header.frame_id='IMU1_Frame'
                imu_msg.IMU.header.stamp.secs = int(current_time.secs)
                imu_msg.IMU.header.stamp.nsecs = int(current_time.nsecs)
                imu_msg.IMU.orientation.x = qx/magnitude
                imu_msg.IMU.orientation.y = qy/magnitude
                imu_msg.IMU.orientation.z = qz/magnitude
                imu_msg.IMU.orientation.w = qw/magnitude
                imu_msg.IMU.linear_acceleration.x = accX
                imu_msg.IMU.linear_acceleration.y = accY
                imu_msg.IMU.linear_acceleration.z = accZ
                imu_msg.IMU.angular_velocity.x = gyroX
                imu_msg.IMU.angular_velocity.y = gyroY
                imu_msg.IMU.angular_velocity.z = gyroZ
                imu_msg.MagField.header.seq=sequence
                imu_msg.MagField.header.frame_id='IMU1_Frame'
                imu_msg.MagField.header.stamp.secs = int(current_time.secs)
                imu_msg.MagField.header.stamp.nsecs = int(current_time.nsecs)
                imu_msg.MagField.magnetic_field.x = magX
                imu_msg.MagField.magnetic_field.y = magY
                imu_msg.MagField.magnetic_field.z = magZ
                imu_pub.publish(imu_msg)
                sequence+=1
                
                
                
                
                
                # latitude = data[2]
                # lat_dir = data[3]
                # longitude = data[4]
                # lon_dir = data[5]

                # deg_lat = float(latitude[:2])
                # dec_min_lat = float(latitude[2:])
                # deg_deg_lat = deg_lat + dec_min_lat / 60.0
                # deg_long = float(longitude[:3])
                # dec_min_long = float(longitude[3:])
                # deg_deg_long = deg_long + dec_min_long / 60.0
                # if lat_dir== 'S':
                #     deg_deg_lat=deg_deg_lat*(-1)
                # if lon_dir=='W':
                #     deg_deg_long=deg_deg_long*(-1)
                # imu_msg.latitude = deg_deg_lat
                # imu_msg.longitude = deg_deg_long
                # imu_msg.header.seq+=1
                # imu_pub.publish(imu_msg)
                
                
            
    except rospy.ROSInterruptException:
        port.close()
    
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down IMU node...")
