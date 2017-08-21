#! /usr/bin/python
# Written by Dan Mandle http://dan.mandle.me September 2012
# License: GPL 2.0
 
import os
from gps import *
from time import *
import time
import threading
import subprocess
import logging
import sys
import math
from Adafruit_BNO055 import BNO055
from serial import Serial
import json
import calendar

#------------
# SET UP GPS |
#------------

print "configuring GPS to 5Hz..."

serialPort = Serial("/dev/ttyUSB0", 9600)
if (serialPort.isOpen() == False):
    serialPort.open()

#This set is used to set the rate the GPS reports
UPDATE_10_sec=  "$PMTK220,10000*2F\r\n" #Update Every 10 Seconds
UPDATE_5_sec=  "$PMTK220,5000*1B\r\n"   #Update Every 5 Seconds  
UPDATE_1_sec=  "$PMTK220,1000*1F\r\n"   #Update Every One Second
UPDATE_200_msec=  "$PMTK220,200*2C\r\n" #Update Every 200 Milliseconds
#This set is used to set the rate the GPS takes measurements
MEAS_10_sec = "$PMTK300,10000,0,0,0,0*2C\r\n" #Measure every 10 seconds
MEAS_5_sec = "$PMTK300,5000,0,0,0,0*18\r\n"   #Measure every 5 seconds
MEAS_1_sec = "$PMTK300,1000,0,0,0,0*1C\r\n"   #Measure once a second
MEAS_200_msec= "$PMTK300,200,0,0,0,0*2F\r\n"  #Meaure 5 times a second
#Set the Baud Rate of GPS
BAUD_57600 = "$PMTK251,57600*2C\r\n"          #Set Baud Rate at 57600
BAUD_9600 ="$PMTK251,9600*17\r\n"             #Set 9600 Baud Rate

serialPort.write(BAUD_57600)
time.sleep(1)
serialPort.baudrate = 57600
serialPort.write(UPDATE_200_msec)
time.sleep(1)
serialPort.write(MEAS_200_msec)
time.sleep(1)

serialPort.flushInput()
serialPort.flushOutput()

serialPort.close()

subprocess.call(['gpsctl', '-c', '0.2'])

print "done"

gpsd = None #seting the global variable
 
class GpsPoller(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		global gpsd			#bring it in scope
		gpsd = gps(mode=WATCH_ENABLE)	#starting the streem of info
		self.current_value = None
		self.running = True 		#set the thread running to true
 
	def run(self):
		global gpsd
		while gpsp.running:
			gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer

#------------
# SET UP IMU |
#------------

bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)
if not bno.begin():
	raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

data = {'dataPoints':[]}

#------
# MAIN |
#------

if __name__ == '__main__':
	gpsp = GpsPoller() # create the thread
	try:
		gpsp.start() # start it up
		t = None

		while True:
#      			if gpsd.utc[-7:-3] != t:
			if True:
				t = gpsd.utc[-7:-3]
#				print t
				point = ({
					'time': gpsd.utc,
					'accel': bno.read_accelerometer(),
					'euler': bno.read_euler(),
					'coordinate': (gpsd.fix.latitude, gpsd.fix.longitude),
					'speed': gpsd.fix.speed,
					'error': (gpsd.fix.eps, gpsd.fix.epx, gpsd.fix.epv, gpsd.fix.ept),
					'altitude': gpsd.fix.altitude,
					'track': gpsd.fix.track
					})
				data['dataPoints'].append(point)

#      	  			print point      
				time.sleep(0.2)
#				print time.time() - start

	except (KeyboardInterrupt, SystemExit):	#when you press ctrl+c
		print "\nKillingThread..."
		gpsp.running = False
		gpsp.join() 			#wait for thread to finish
		t = ''
		for i in range(len(data['dataPoints'])):
			if data['dataPoints'][i]['time'] != t:
				t = data['dataPoints'][i]['time']
				print(data['dataPoints'][i])

		with open(sys.argv[1], 'w') as outfile:
			json.dump(data, outfile)

		print "Done. \nExiting"


