#!/usr/bin/env python3

import serial 
import time
import pynmea2

from readGPS.srv import (getCurrentGPS, getCurrentGPSResponse)

import rospy
from std_msgs.msg import String

class readGPS(object):

	def __init__(self):
		pass

	def getGPS(self):
		
		port="/dev/ttyUSB0"
		while True:
			ser = serial.Serial(port, baudrate=9600, timeout=0.5)
			dataout = pynmea2.NMEAStreamReader()
			data=ser.readline()
			
			if data[0:6] == "$GPGGA":
					parsed_data = pynmea2.parse(data)
			lat_and_long =str(parsed_data.latitude) + "," + str(parsed_data.longitude)
						
			return lat_and_long				

	def gpsCallBack(self, req):

		response = getCurrentGPSResponse()
		response.latitude = 12.33 #str(getGPS)
		response.longitude = 12.22

		return response



if __name__ == '__main__':
	try:
		rospy.init_node("gps_server")	
		monitor = readGPS()
		get_gps = rospy.Service("get_gps", getCurrentGPS, monitor.gpsCallBack)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

