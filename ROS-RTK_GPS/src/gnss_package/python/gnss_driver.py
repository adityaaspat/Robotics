#!/usr/bin/env python3


import rospy
import serial
from gnss_package.msg import gnss_msg
import utm
import sys
from rospy import Time

# Reading the port form terminal command
port =  sys.argv[1]
if __name__ == '__main__':
    rospy.init_node('gnss_node')
    serial_port = rospy.get_param(port, '/dev/ttyACM0')
    serial_baud = rospy.get_param('~baudrate',57600)
    sampling_rate = rospy.get_param('~sampling_rate',5.0)
  
    port = serial.Serial(port, serial_baud, timeout=3.)    
    gnss_pub = rospy.Publisher('/gnss', gnss_msg, queue_size=10)
    gnss_msg = gnss_msg()
    gnss_msg.header.seq = 0
    gnss_msg.header.frame_id = "gnss1_Frame" 
   
    try:
        while not rospy.is_shutdown():
            line = port.readline()
            stringformat = str(line)
            data = list(map(str,stringformat.split(',')))
            #print(data)
            if (data[0] == "b'\\r$GNGGA" or data[0]=="b'$GNGGA")and data[2] == '':
                rospy.logwarn("Not Recieveing GPS data ")
                print("NO DATA")
            elif (data[0] == "b'\\r$GNGGA" or data[0]=="b'$GNGGA")and data[2] !='':
                print("Values:",data)
                quality=data[6]
                utc_time = float(data[1])
                secs = int(utc_time)
                nsecs = int((utc_time - secs) * 1e9)

                gnss_msg.header.stamp = Time(secs, nsecs)
                gnss_msg.altitude = float(data[9])
                latitude = data[2]
                lat_dir = data[3]
                longitude = data[4]
                lon_dir = data[5]

                deg_lat = float(latitude[:2])
                dec_min_lat = float(latitude[2:])
                deg_deg_lat = deg_lat + dec_min_lat / 60.0
                deg_long = float(longitude[:3])
                dec_min_long = float(longitude[3:])
                deg_deg_long = deg_long + dec_min_long / 60.0
                if lat_dir== 'S':
                    deg_deg_lat=deg_deg_lat*(-1)
                if lon_dir=='W':
                    deg_deg_long=deg_deg_long*(-1)
                gnss_msg.latitude = deg_deg_lat
                gnss_msg.longitude = deg_deg_long
                gnss_msg.rtk_quality=int(quality)
                gnss_msg.utm_easting, gnss_msg.utm_northing, gnss_msg.zone, gnss_msg.letter = utm.from_latlon(deg_deg_lat,deg_deg_long)
                gnss_msg.header.seq+=1
                gnss_pub.publish(gnss_msg)
                #rospy.sleep(sleep_time)
            
    except rospy.ROSInterruptException:
        port.close()
    
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down gnss node...")