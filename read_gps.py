#!/usr/bin/env python2.7

import serial 
import time
import pynmea2

from read_sensors.srv import getGPS, getGPSResponse

import rospy
from std_msgs.msg import String

def getGps():
	port="/dev/ttyUSB0"
	
	while True:
		ser = serial.Serial(port, baudrate=9600, timeout=0.5)
		dataout = pynmea2.NMEAStreamReader()
		data=ser.readline()
		
		if data[0:6] == "$GPGGA":
			parsed_data = pynmea2.parse(data)
			lat_and_long =str(parsed_data.latitude) + "," + str(parsed_data.longitude)
			return lat_and_long				

def gpsCallBack(req):
	
	
	response = getGPSResponse()
	response.gps = str(getGps())

	return response



if __name__ == '__main__':
	try:
		rospy.init_node("gps_server")	
		get_gps = rospy.Service("get_gps", getGPS, gpsCallBack)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

