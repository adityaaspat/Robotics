#!/usr/bin/env python3


import rospy
import serial
from gps_driver.msg import gps_msg
import utm
import sys
from rospy import Time
port =  sys.argv[1]

# Reading the port form terminal command
# port =  sys.argv[1]

port =  sys.argv[1]

if __name__ == '__main__':
    #SENSOR_NAME = "gps"
    rospy.init_node('gps_node')
    serial_port = rospy.get_param(port, '/dev/ttyUSB0')
    serial_baud = rospy.get_param('~baudrate',4800)
    sampling_rate = rospy.get_param('~sampling_rate',5.0)
  
    port = serial.Serial(port, serial_baud, timeout=3.)
    rospy.logdebug("Using GPS sensor on port "+serial_port+" at "+str(serial_baud))
    
    gps_pub = rospy.Publisher('/gps', gps_msg, queue_size=10)
    
    rospy.logdebug("Initialization complete")
    rospy.loginfo("Publishing gps data")
    gps_msg = gps_msg()
    gps_msg.header.seq = 0
    gps_msg.header.frame_id = "GPS1_Frame" 


# if __name__ == '__main__':
#     rospy.init_node('gps_node')
#     serial_port = rospy.get_param('~port', '/dev/ttyUSB0')
#     serial_baud = rospy.get_param('~baudrate',4800)
#     sampling_rate = rospy.get_param('~sampling_rate',5.0)
  
#     port = serial.Serial(serial_port, serial_baud, timeout=3.)    
#     gps_pub = rospy.Publisher('/gps', gps_msg, queue_size=10)
#     rospy.logdebug("Initialization complete")
#     rospy.loginfo("Publishing gps data")
#     gps_msg = gps_msg()
#     gps_msg.header.seq = 0
#     gps_msg.header.frame_id = "GPS1_Frame" 
   
    try:
        while not rospy.is_shutdown():
            line = port.readline()
            stringformat = str(line)
            data = list(map(str,stringformat.split(',')))
            #print(data)
            if (data[0] == "b'\\r$GPGGA" or data[0]=="b'$GPGGA")and data[2] == '':
                rospy.logwarn("Not Recieveing GPS data ")
                print("NO DATA")
            elif (data[0] == "b'\\r$GPGGA" or data[0]=="b'$GPGGA")and data[2] !='':
                print("Values:",data)
                utc_time = float(data[1])
                secs = int(utc_time)
                nsecs = int((utc_time - secs) * 1e9)

                gps_msg.header.stamp = Time(secs, nsecs)
                gps_msg.altitude = float(data[9])
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
                gps_msg.latitude = deg_deg_lat
                gps_msg.longitude = deg_deg_long
                gps_msg.utm_easting, gps_msg.utm_northing, gps_msg.zone, gps_msg.letter = utm.from_latlon(deg_deg_lat,deg_deg_long)
                gps_msg.header.seq+=1
                gps_pub.publish(gps_msg)
                
                
            
    except rospy.ROSInterruptException:
        port.close()
    
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down GPS node...")
