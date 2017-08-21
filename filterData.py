#!/usr/bin/python
import matplotlib
#matplotlib.use('agg')
matplotlib.use('GTK')

import math
import json
import time
import numpy as np
import matplotlib.pyplot as plt
import sys
from numpy import ma
from datetime import datetime
from numpy.linalg import inv

# GLOBAL VARIABLES
GRAVITY = 9.80665
EARTH_RADIUS = 6371 * 1e3
LAT, LON = 0, 1

E_acc_std = 0.01 * GRAVITY
N_acc_std = 0.01 * GRAVITY
latLong_std = 1
predict_rate = 2

# HELPER METHODS
def geoAngle(latitudeOrLongitude):
	return (math.radians(latitudeOrLongitude))

def getDistanceMeters(origin, destination):
	deltaLat = geoAngle(destination[LAT] - origin[LAT])
	deltaLon = geoAngle(destination[LON] - origin[LON])
	a = (math.pow(math.sin(deltaLat / 2.0), 2) + 
		math.cos(geoAngle(origin[LAT])) * 
		math.cos(geoAngle(destination[LAT])) *
		math.pow(math.sin(deltaLon / 2.0), 2))
	c = 2 * math.atan2(math.sqrt(a), math.sqrt(1.0-a))
	return EARTH_RADIUS * c

def latitudeToMeters(latitude):
	distance = getDistanceMeters([0,0],[latitude,0])
	if latitude < 0:
		distance *= -1
	return distance

def longitudeToMeters(longitude):
	distance = getDistanceMeters([0,0],[0,longitude])
	if longitude < 0:
		distance *= -1
	return distance

def northAndEastAcceleration(acceleration, heading, roll, pitch):
	pi = math.pi
	x_acc, y_acc, z_acc = acceleration
	
	# project accelerations onto new axis where xy plane is parrallel to the ground
        x_acc_p = x_acc * math.cos(roll) + z_acc * math.sin(roll)
        y_acc_p = y_acc * math.cos(pitch) + z_acc * math.sin(pitch)

        # project onto plane such that N is 0 degrees and E is 90 degrees 
        # heading = 0 when y axis points north
        N_acc = y_acc_p * math.cos(heading) + x_acc_p * math.cos(heading + (pi/2.0))
        E_acc = y_acc_p * math.sin(heading) + x_acc_p * math.sin(heading + (pi/2.0))

	return [N_acc, E_acc]

def getPointAhead(origin, distanceMeters, bearing):
	radiusFraction = distanceMeters/EARTH_RADIUS
	lat1 = geoAngle(float(origin[LAT]))
	lng1 = geoAngle(float(origin[LON]))
	lat2_part1 = math.sin(lat1) * math.cos(radiusFraction)
	lat2_part2 = math.cos(lat1) * math.sin(radiusFraction) * math.cos(bearing)
	lat2 = math.asin(lat2_part1 + lat2_part2)
	lng2_part1 = math.sin(bearing) * math.sin(radiusFraction) * math.cos(lat1)
	lng2_part2 = math.cos(radiusFraction) - (math.sin(lat1) * math.sin(lat2))
	lng2 = lng1 + math.atan2(lng2_part1, lng2_part2)
	lng2 = ((lng2+3*math.pi) % (2*math.pi)) - math.pi
	return [float(math.degrees(lat2)), float(math.degrees(lng2))]

def pointPlusDistanceEast(origin, distance):
	return getPointAhead(origin, distance, math.pi/2)

def pointPlusDistanceNorth(origin, distance):
	return getPointAhead(origin, distance, 0.0)

def metersToGeopoint(latAsMeters, longAsMeters):
	point = [0, 0]
	pointEast = pointPlusDistanceEast(point, longAsMeters)
	pointNorthEast = pointPlusDistanceNorth(pointEast, latAsMeters)
	return pointNorthEast

def utcToSeconds(utc):
	t = utc[0:10] + ' ' +  utc[11:23]
	s = datetime.strptime(t, ('%Y-%m-%d %H:%M:%S.%f'))
        s = (time.mktime(s.timetuple()) + (s.microsecond / 1e6))
	return float(s)	

class KalmanFilter():
	
	def __init__ (self,
			initial_position,
			initial_velocity,
			pos_std,
			acc_std,
			initial_time):
		self.I = (np.identity(2)).astype(float)
		self.H = (np.identity(2)).astype(float)
		self.P = (np.identity(2)).astype(float)
		self.Q = np.asarray([[acc_std * acc_std, 0],[0, acc_std * acc_std]])
		self.R = np.asarray([[pos_std * pos_std, 0],[0, pos_std * pos_std]])
		self.u = (np.zeros((1, 1))).astype(float)
		self.z = (np.zeros((2, 1))).astype(float)
		self.A = (np.identity(2)).astype(float)
		self.B = (np.zeros((2, 1))).astype(float)
		self.currentState = ([initial_position, initial_velocity])
		self.timestamp = utcToSeconds(initial_time)

	def predict(self, acc, curr_time):
		t = utcToSeconds(curr_time)
		dt = t - self.timestamp
		self.recreateTransitionMatrix(dt)
		self.recreateControlMatrix(dt)
		self.u[0 , 0] = acc
		self.currentState = self.A * self.currentState + self.B * self.u
		self.P = np.float_(self.A * self.P * np.transpose(self.A) + self.P)
		self.timestamp = t
	
	def update(self, position, velocity, error_pos, error_vel):
		self.z[0, 0] = position
		self.z[1, 0] = velocity
		self.R[0, 0] = error_pos * error_pos
		self.R[1, 0] = error_vel * error_vel
		y = self.z - self.currentState
		s = self.P + self.R
		sInverse = inv(s)
		K = self.P * sInverse
		self.currentState = self.currentState + (K * y)
		
	def recreateControlMatrix(self, dt):
		self.B[0, 0] = 0.5 * dt * dt
		self.B[1, 0] = dt

	def recreateTransitionMatrix(self, dt):
		self.A[0, 0] = 1.0
		self.A[0, 1] = dt
		self.A[1, 0] = 0.0
		self.A[1, 1] = 1.0
	
	def getPredictedPosition(self):
		return self.currentState[0,0]
	
	def getPredictedVelocity(self):
		return self.currentState[1,0]



# MAIN
with open(sys.argv[1]) as json_data:
	data = json.load(json_data)['dataPoints']

N_kf, E_kf = '', ''

N_positions, E_positions = [],[] 	# will hold raw positions
N_pos_filtered, E_pos_filtered = [],[]  # will hold filtered positions

for i in range(0, len(data)):
	t = data[i]['time']
	if t != '':		
		latitude, longitude = data[i]['coordinate']	
#		heading = math.radians(data[i]['track'])
		heading = math.radians(data[i]['euler'][0])	
		roll = math.radians(data[i]['euler'][1])
		pitch = math.radians(data[i]['euler'][2])
		acceleration = data[i]['accel']
		speed = data[i]['speed']
		error = data[i]['error']
		error_pos = error[1]
		error_speed = error[0]

	 	N_pos = latitudeToMeters(latitude)
		N_vel = speed * math.cos(heading)
		E_pos = longitudeToMeters(longitude)
                E_vel = speed * math.sin(heading)
		N_acc, E_acc = northAndEastAcceleration(acceleration, heading, roll, pitch)

		N_positions.append(N_pos)
		E_positions.append(E_pos)

		if len(N_positions) == 1:
			N_kf = KalmanFilter(N_pos, N_vel, latLong_std, N_acc_std, t)
			E_kf = KalmanFilter(E_pos, E_vel, latLong_std, E_acc_std, t)

			N_pos_filtered.append(N_pos)
			E_pos_filtered.append(E_pos)
			
			start_time = utcToSeconds(t)

		elif int(utcToSeconds(t) * 10) % int(predict_rate * 10) == 0:
			N_kf.predict(N_acc * GRAVITY, t)
			E_kf.predict(E_acc * GRAVITY, t)

			N_kf.update(N_pos, N_vel, error_pos, error_speed)
			E_kf.update(E_pos, E_vel, error_pos, error_speed)

			N_pred_pos = N_kf.getPredictedPosition()
			E_pred_pos = E_kf.getPredictedPosition()
			
			N_pred_vel = N_kf.getPredictedVelocity()
			E_pred_vel = E_kf.getPredictedVelocity()
			
			N_pos_filtered.append(N_pred_pos)
			E_pos_filtered.append(E_pred_pos)			
		


# COLLECTING DATA FROM 3rd PARTY (STRAVA)

with open(sys.argv[2]) as route_data:
        route_data = json.load(route_data)['features'][0]['geometry']['coordinates'][0]

route_N_pos = []
route_E_pos = []

for i in range(len(route_data)):
	route_N_pos.append(latitudeToMeters(route_data[i][1]))
	route_E_pos.append(longitudeToMeters(route_data[i][0]))		



# GRAPHING

max_E = max(max(E_positions), max(E_pos_filtered), max(route_E_pos))
min_E = min(min(E_positions), min(E_pos_filtered), min(route_E_pos))
max_N = max(max(N_positions), max(N_pos_filtered), max(route_N_pos))
min_N = min(min(N_positions), min(N_pos_filtered), min(route_N_pos))
n = len(E_positions)

plt.figure()
plt.plot(E_positions, N_positions, 'r', linewidth = 2)
plt.plot(E_pos_filtered, N_pos_filtered, 'b', linewidth = 2)
plt.plot(route_E_pos, route_N_pos, 'g', linewidth=2)
plt.axis([max_E - 200, max_E - 100, max_N + 100, max_N + 200])
plt.title("meters east of (0,0) vs meters north of (0,0)")

plt.show()
