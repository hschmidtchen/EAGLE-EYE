#!/usr/bin/env python

# Copyright (c) 2011 Bastian Venthur
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


"""Demo app for the AR.Drone.

This simple application allows to control the drone and see the drone's video
stream.
"""


import cv2
import cv2.aruco as aruco
import libardrone.libardrone as libardrone
import numpy as np

import threading
import time

exitFlag = 0

class manualControlThread (threading.Thread):
	def __init__(self, threadID, name):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.running = True
		
	def run(self):
		
		global drone
		global exiting
		global manual_mode
		global thread_lock
		global thread_lock2
		global thread_lock4
		global key
		
		print "Starting " + self.name
		
		while self.running:
			# query pressed keys
			thread_lock4.acquire()
			k = key
			thread_lock4.release()
			# escape to stop program execution
			if k == 27: # 27=escape
				self.running = False
				thread_lock.acquire()
				exiting = True
				thread_lock.release()
				drone.reset()
			 # takeoff
			elif k == 13: # 13=enter
				print("return")
				drone.takeoff()
			# land
			elif k == 32: # 32=space
				print("space")
				drone.land()
			# emergency
			elif k == 8: # 8=backspace
				drone.reset()
			# switch control mode
			elif k == ord('m'):
				drone.hover()
				thread_lock2.acquire()
				manual_mode = not manual_mode
				thread_lock2.release()
	 		 # switch between manual and autonomous control
			elif manual_mode:
				# listen for additional key events for manual control
				# forward / backward
				if k == ord('w'):
					drone.move_forward()
				elif k == ord('s'):
					drone.move_backward()
				# left / right
				elif k == ord('a'):
					drone.move_left()
				elif k == ord('d'):
					drone.move_right()
				# up / down
				elif k == 2490368:
					drone.move_up()
				elif k == 2621440:
					drone.move_down()
				# turn left / turn right
				elif k == 2424832:
					drone.turn_left()
				elif k == 2555904:
					drone.turn_right()
				# speed
				elif k == ord('1'):
					drone.speed = 0.1
				elif k == ord('2'):
					drone.speed = 0.2
				elif k == ord('3'):
					drone.speed = 0.3
				elif k == ord('4'):
					drone.speed = 0.4
				elif k == ord('5'):
					drone.speed = 0.5
				elif k == ord('6'):
					drone.speed = 0.6
				elif k == ord('7'):
					drone.speed = 0.7
				elif k == ord('8'):
					drone.speed = 0.8
				elif k == ord('9'):
				  drone.speed = 0.9
				elif k == ord('0'):
				  drone.speed = 1.0
				else:
				  drone.hover()
			  
		print("Shutting down...")
		drone.halt()
		print("Ok.")		
		print "Exiting " + self.name
		
class automaticControlThread (threading.Thread):
	def __init__(self, threadID, name):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.running = True
		self.status = "Start"
		
		
	def run(self):
		
		global drone
		global thread_lock
		global thread_lock2
		global thread_lock2
		global exiting
		global manual_mode
		global W
		global H
		global new_frame
		global render_frame
		print "Starting " + self.name
		
		miss_counter = 0

		p = (-W	,0)	
		
		while self.running:
			thread_lock.acquire()
			if exiting:
				thread_lock.release()
				self.running = False
			else:
				thread_lock.release()
				
				thread_lock2.acquire()
				if not manual_mode:
					thread_lock2.release()

					# Tracking 
					if self.status == "Start":
						# To do with image recognition
						self.status = "Searching"
					elif self.status == "Searching":
						# To do turn and search marker
						p,area = self.getAndSearchImage()
						if p[0] >= -W/2:
							self.status = "Tracking"
						else:
							print 'turn left'
							drone.hover()
						print 'Searching'
					elif self.status == "Tracking":
						# To do image recognition and controler calculation
						if p[0] >= -W/2:
							miss_counter = 0
							self.controlStep(p,area)
							p,area = self.getAndSearchImage()
						elif miss_counter < 5:
							miss_counter += 1
						else:
							self.status = "Searching"
						print 'tracking'
					# Controler
				else:
					thread_lock2.release()
					try:
						# pull image
						pixelarray = drone.get_image()
						if pixelarray != None:
							frame = pixelarray[:,:,::-1].copy()
						#resize image
							resized=cv2.resize(frame,(W,H))
						
						thread_lock3.acquire()
						render_frame = resized
						new_frame = True
						thread_lock3.release()
					except:
						pass
		print "Exiting" + self.name
		
	def getAndSearchImage(self):
		global drone
		global thread_lock3
		global render_frame
		global new_frame
		global W
		global H
		try:
			# print pygame.image
			pixelarray = drone.get_image()
			if pixelarray != None:
				frame = pixelarray[:,:,::-1].copy()
			#resize image
				resized=cv2.resize(frame,(W,H))
			# aruco detection
				gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
				aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
				parameters =  aruco.DetectorParameters_create()
		
				#lists of ids and the corners beloning to each id
				corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
				#print(corners)



				# battery status
				#hud_color = (255, 0, 0) if drone.navdata.get('drone_state', dict()).get('emergency_mask', 1) else (10, 10, 255)
				#bat = drone.navdata.get(0, dict()).get('battery', 0)
				# f = pygame.font.Font(None, 20)
				# hud = f.render('Battery: %i%%' % bat, True, hud_color) 
				# screen.blit(hud, (10, 10))
				#cv2.imshow('Drone',gray)

				# iterate over all found markers
				selected_corners = []
				max_index = (0,0)
				counter = 0
				if len(corners) > 0:
					dim = corners[0].shape
					#print dim
					while counter<dim[0]:
						tmp_corners = ((corners[0])[counter])
						# A=(1/2)|[(x3-x1)(y4-y2) +(x4-x2)(y1-y3)]|
						area = 0.5*((tmp_corners[2][0] - tmp_corners[0][0]) * (tmp_corners[3][1] - tmp_corners[1][1]) + (tmp_corners[3][0] - tmp_corners[1][0]) * (tmp_corners[0][1] - tmp_corners[2][1]))
						if area > max_index[0]:
							max_index = (area,counter)
						counter +=1

					#dim2 = (corners[0])[0].shape
					#print dim2
					#dim3 = ((corners[0])[0])[0].shape
					#print dim3
					max_corners = ((corners[0])[max_index[1]])
					selected_corners = np.array([np.array([(corners[0])[max_index[1]]],dtype=np.float32)])#[max_index[0]*4:max_index[0]*4+3]
					#print selected_corners
					#print selected_corners.shape
				# draw all markers
				display = aruco.drawDetectedMarkers(resized, corners)

				# draw selected marker
				#display = aruco.drawDetectedMarkers(display, selected_corners, Scalar(0, 255, 0) )		

				thread_lock3.acquire()
				render_frame = display
				new_frame = True
				thread_lock3.release()

				if len(selected_corners) > 0:
					x,y = max_corners.sum(axis=0)/4
					area = max_index[0]
				else:
					x = -W
					y = -1
					area = -1
				return (x-W/2,y-H/2), area
		except:
			pass
		
	def controlStep(self,p,area):
		
		global tx_prev
		global uix_prev
		global ex_prev
		global uif_prev
		global ef_prev
		global uiy_prev
		global ey_prev
		x = p[0]	  
		y = p[1]
		
		move_command = False

		MAX_SPEED_ROT = 1.0
		MAX_SPEED_MOVE = 1.0

		# control direction

		K_px=1.0
		K_dx=1.0
		K_ix=1.0	
		ux_threshold = 30		
			
		#control x
		#error for x between the desired and actual output
		ex = 0 - x
		if tx_prev == 0:
			tx_prev = time.time() - 0.008
		tx = time.time() - tx_prev
		
		#Integration input
		uix = uix_prev + 1/K_ix * tx*ex
		#Derivation input
		udx = 1/K_dx * (ex-ex_prev)/tx
		
		#adjust previous values
		ex_prev = ex
		tx_prev += tx
		uix_prev = uix
	
		#calculate input for the system
		ux = K_px * (ex) #+ uix + udx)
		
		if ux < -ux_threshold or ux > ux_threshold:
			left_right = MAX_SPEED_ROT * ux / W * 2
			print 'left_right: '+str(MAX_SPEED_ROT  * ux / W * 2)
			move_command = True
		
		#control height
		K_py=0.5
		K_dy=1.0
		K_iy=1.0	
		uy_threshold = 0.15		
		
		#initialization
		
		
		#control y		
		#error for y between the desired and actual output
		ey = 0 - y
		ty = tx
	
		#Integration input
		uiy = uiy_prev + 1/K_iy * ty*ey
		#Derivation input
		udy = 1/K_dy * (ey-ey_prev)/ty
		
		#adjust previous values
		ey_prev = ey
		uiy_prev = uiy
		
		#calculate input for the system
		uy = 2.0/H*K_py * (ey) #+ uiy + udy)
		if uy < -uy_threshold or uy > uy_threshold:
			up_down = MAX_SPEED_MOVE * uy
			move_command = True
			print 'up_down: '+str(MAX_SPEED_MOVE * uy)
		
		# control forward
		K_pf=0.4
		K_df=1.0
		K_if=1.0	
		uf_threshold = 0.01	
			
		#control f
		#error for f between the desired and actual output
		ef = 0.2 - (area/(W*H)) **0.5
		tf = tx
		#Integration input
		uif = uif_prev + 1/K_if * tf*ef
		#Derivation input
		udf = 1/K_df * (ef-ef_prev)/tf
		
		#adjust previous values
		ef_prev = ef
		uif_prev = uif
	
		#calculate input for the system
		uf = K_pf * (ef) #+ uif + udf)
		
		if uf < -uf_threshold or uf > uf_threshold:
			front_back = MAX_SPEED_MOVE * uf 
			move_command = True		
			print 'front_back: '+str(MAX_SPEED_MOVE * uf)

		# apply control vectors
		drone.at(at_pcdm, move_command, 0, front_back, up_down, left_right)


def main():
	global drone
	global thread_lock
	global thread_lock2
	global thread_lock3
	global thread_lock4
	global exiting
	global manual_mode
	global W
	global H
	global render_frame
	global new_frame
	global key
	global tx_prev
	global uix_prev
	global ex_prev
	global uif_prev
	global ef_prev
	global uiy_prev
	global ey_prev

	W, H = 640, 360
	key = -1
	
	drone = libardrone.ARDrone(True)
	drone.reset()
	
	exiting = False
	manual_mode = True
	new_frame = False
	threads = []
	thread_lock = threading.Lock()
	thread_lock2 = threading.Lock()
	thread_lock3 = threading.Lock()
	thread_lock4 = threading.Lock()

	#initialization
	tx_prev = 0
	uix_prev = 0
	ex_prev = 0
	uif_prev = 0
	ef_prev = 0
	uiy_prev = 0
	ey_prev = 0		
	
	# Create new threads
	manual_control_thread = manualControlThread(1, "Manual Control Thread")
	automatic_control_thread = automaticControlThread(2, "Automatic Control Thread")
	
	# Start new Threads
	manual_control_thread.start()
	automatic_control_thread.start()
	
	# Add threads to thread list
	threads.append(manual_control_thread)
	threads.append(automatic_control_thread)

	thread_lock.acquire()
	while not exiting:		
		thread_lock.release()

		thread_lock4.acquire()
		key = cv2.waitKey(33)
		thread_lock4.release()

		thread_lock3.acquire()
		if new_frame:
			cv2.imshow('Drone',render_frame)	
		thread_lock3.release()
		thread_lock.acquire()

	# Wait for all threads to complete
	for t in threads:
		t.join()
	print "Exiting Main Program"

if __name__ == '__main__':
	main()
