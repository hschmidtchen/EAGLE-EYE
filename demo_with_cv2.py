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
		global key
		global thread_lock3

		print "Starting " + self.name
		
		while self.running:
			# query pressed keys
			thread_lock3.acquire()
			k = key
			thread_lock3.release()
			# escape to stop program execution
			if k == 27: # 27=escape
				self.running = False
				thread_lock.acquire()
				exiting = True
				thread_lock.release()
				thread_lock5.acquire()
				drone.reset()
				thread_lock5.release()
			 # takeoff
			elif k == 13: # 13=enter
				print("return")
				thread_lock5.acquire()
				drone.takeoff()
				thread_lock5.release()
			# land
			elif k == 32: # 32=space
				print("space")
				thread_lock5.acquire()
				drone.land()
				thread_lock5.release()
			# emergency
			elif k == 8: # 8=backspace
				thread_lock5.acquire()
				drone.reset()
				thread_lock5.release()
			# switch control mode
			elif k == ord('m'):
				thread_lock5.acquire()
				drone.hover()
				thread_lock5.release()
				thread_lock2.acquire()
				manual_mode = not manual_mode
				thread_lock2.release()
	 		 # switch between manual and autonomous control
			elif manual_mode:
				# listen for additional key events for manual control
				# forward / backward
				if k == ord('w'):
					thread_lock5.acquire()
					drone.move_forward()
					thread_lock5.release()
				elif k == ord('s'):
					thread_lock5.acquire()
					drone.move_backward()
					thread_lock5.release()
				# left / right
				elif k == ord('a'):
					thread_lock5.acquire()
					drone.move_left()
					thread_lock5.release()
				elif k == ord('d'):
					thread_lock5.acquire()
					drone.move_right()
					thread_lock5.release()
				# up / down
				elif k == 2490368 or k == 65362:
					thread_lock5.acquire()
					drone.move_up()
					thread_lock5.release()
				elif k == 2621440 or k == 65364:
					thread_lock5.acquire()
					drone.move_down()
					thread_lock5.release()
				# turn left / turn right
				elif k == 2424832 or k == 65361:
					thread_lock5.acquire()
					drone.turn_left()
					thread_lock5.release()
				elif k == 2555904 or k == 65363:
					thread_lock5.acquire()
					drone.turn_right()
					thread_lock5.release()
				# speed
				elif k == ord('1'):
					thread_lock5.acquire()
					drone.speed = 0.1
					thread_lock5.release()
				elif k == ord('2'):
					thread_lock5.acquire()
					drone.speed = 0.2
					thread_lock5.release()
				elif k == ord('3'):
					thread_lock5.acquire()
					drone.speed = 0.3
					thread_lock5.release()
				elif k == ord('4'):
					thread_lock5.acquire()
					drone.speed = 0.4
					thread_lock5.release()
				elif k == ord('5'):
					thread_lock5.acquire()
					drone.speed = 0.5
					thread_lock5.release()
				elif k == ord('6'):
					thread_lock5.acquire()
					drone.speed = 0.6
					thread_lock5.release()
				elif k == ord('7'):
					thread_lock5.acquire()
					drone.speed = 0.7
					thread_lock5.release()
				elif k == ord('8'):
					thread_lock5.acquire()
					drone.speed = 0.8
					thread_lock5.release()
				elif k == ord('9'):
					thread_lock5.acquire()
				  	drone.speed = 0.9
					thread_lock5.release()
				elif k == ord('0'):
					thread_lock5.acquire()
				  	drone.speed = 1.0
					thread_lock5.release()
				else:
					thread_lock5.acquire()
				  	drone.hover()
					thread_lock5.release()

			  
		print("Shutting down...")
		thread_lock5.acquire()
		drone.halt()
		thread_lock5.release()
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
		global thread_lock4
		global exiting
		global manual_mode
		global W
		global H
		global frame_to_render
		global new_frame_to_render
		
		print "Starting " + self.name
		
		p = (-W	, 0)	
		
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
						print'start'
					elif self.status == "Searching":
						# To do turn and search marker
						p,d = self.getAndSearchImage()
						if p[0] >= -W/2:
							self.status = "Tracking"
						else:
							thread_lock5.acquire()
							drone.turn_left()
							thread_lock5.release()
						print 'Searching'
					elif self.status == "Tracking":
						# To do image recognition and controler calculation
						if p[0] >= -W/2:
							controlStep(p,d)
							p,d = self.getAndSearchImage()
						else:
							self.status = "Searching"
						print 'tracking'
				else:
					thread_lock2.release()
					try:
						# print pygame.image
						thread_lock5.acquire()
						pixelarray = drone.get_image()
						thread_lock5.release()
						if pixelarray != None:
							frame = pixelarray[:,:,::-1].copy()
							#resize image
							resized=cv2.resize(frame,(W,H))

						#image conversion

						# battery status
						#	hud_color = (255, 0, 0) if drone.navdata.get('drone_state', dict()).get('emergency_mask', 1) else (10, 10, 255)
						#	bat = drone.navdata.get(0, dict()).get('battery', 0)
						# f = pygame.font.Font(None, 20)
						# hud = f.render('Battery: %i%%' % bat, True, hud_color) 
						# screen.blit(hud, (10, 10))

							thread_lock4.acquire()
							frame_to_render = frame
							new_frame_to_render = True
							thread_lock4.release()

					except:
						pass
				
		print "Exiting" + self.name
		
	def getAndSearchImage(self):
		 try:
			# print pygame.image
			thread_lock5.acquire()
			pixelarray = drone.get_image()
			thread_lock5.release()
			if pixelarray != None:
				frame = pixelarray[:,:,::-1].copy()
			#resize image
				resized=cv2.resize(frame,(W,H))
			# aruco detection
				gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
				aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
				parameters =  aruco.DetectorParameters_create()
		
				#lists of ids and the corners beloning to each id
				corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
				print(corners)
				gray = aruco.drawDetectedMarkers(gray, corners)

				# battery status
				#hud_color = (255, 0, 0) if drone.navdata.get('drone_state', dict()).get('emergency_mask', 1) else (10, 10, 255)
				#bat = drone.navdata.get(0, dict()).get('battery', 0)
				# f = pygame.font.Font(None, 20)
				# hud = f.render('Battery: %i%%' % bat, True, hud_color) 
				# screen.blit(hud, (10, 10))

				thread_lock4.acquire()
				frame_to_render = gray
				new_frame_to_render = True
				thread_lock4.release()

				x = 1
				y = 2
				hx = 3
				hy = 4
				return (x,y), (hx,hy)
		 except:
			pass
		
	def controlStep(self,p,d):
		
		x = p[0]	  
		y = p[1]
		
		hx = d[0]
		hy = d[1]

		K_px=2.0
		K_dx=0.0
		K_ix=1.0		
		
		#initialization
		tx_prev = 0
		uix_prev = 0
		ex_prev = 0
			
		#control x
		#error for x between the desired and actual output
		ex = 0 - x
		tx = time.time() - tx_prev
		
		#Integration input
		uix = uix_prev + 1/K_ix * tx*ex
		#Derivation input
		udx = 1/K_d * (ex-ex_prev)/tx
		
		#adjust previous values
		ex_prev = ex
		tx_prev += tx
		uix_prev = uix
		
		#calculate input for the system
		ux = K_p * (ex + uix + udx)
		
		if ux < -ux_threshold :
			thread_lock5.acquire()
			drone.speed = -MAX_SPEED_ROT * ux
			drone.turn_right()
			thread_lock5.release()
		elif ux > +ux_threshold :
			thread_lock5.acquire()
			drone.speed = MAX_SPEED_ROT * ux
			drone.turn_left()
			thread_lock5.release()
		
		K_py=2.0
		K_dy=0.0
		K_iy=1.0		
		
		#initialization
		ty_prev = 0
		uiy_prev = 0
		ey_prev = 0		
		
		#control y		
		#error for y between the desired and actual output
		ey = 0 - y
		ty = time.time() - ty_prev
		
		#Integration input
		uiy = uiy_prev + 1/K_iy * ty*ey
		#Derivation input
		udy = 1/K_dy * (ey-ey_prev)/ty
		
		#adjust previous values
		ey_prev = ey
		ty_prev += ty
		uiy_prev = uiy
		
		#calculate input for the system
		uy = K_py * (ey + uiy + udy)
		
		if uy < -uy_threshold :
			thread_lock5.acquire()
			drone.speed = -MAX_SPEED_ROT * uy
			drone.turn_up()
			thread_lock5.release()
		elif uy > +uy_threshold :
			thread_lock5.acquire()
			drone.speed = MAX_SPEED_ROT * uy
			drone.turn_down()
			thread_lock5.release()
		 

def main():
	global drone
	global thread_lock
	global thread_lock2
	global thread_lock3
	global thread_lock4
	global thread_lock5
	global exiting
	global manual_mode
	global W
	global H
	global key
	global frame_to_render
	global new_frame_to_render
	W, H = 320, 240
	new_frame_to_render = False
	key = -1
	
	
	
	exiting = False
	manual_mode = True
	
	threads = []
	thread_lock = threading.Lock()
	thread_lock2 = threading.Lock()
	thread_lock3 = threading.Lock()
	thread_lock4 = threading.Lock()
	thread_lock5 = threading.Lock()

	thread_lock5.acquire()
	drone = libardrone.ARDrone(True)
	drone.reset()
	thread_lock5.release()

	# Create new threads
	manual_control_thread = manualControlThread(1, "Manual Control Thread")
	automatic_control_thread = automaticControlThread(2, "Automatic Control Thread")
	
	# Start new Threads
	manual_control_thread.start()
	automatic_control_thread.start()
	
	# Add threads to thread list
	threads.append(manual_control_thread)
	threads.append(automatic_control_thread)
	
	while not exiting:
		thread_lock3.acquire()
		key = cv2.waitKey(33)
		thread_lock3.release()
		
		thread_lock4.acquire()
		if new_frame_to_render:
			cv2.imshow('Drone',frame_to_render)
			new_frame_to_render = False
		thread_lock4.release()			
	# Wait for all threads to complete
	for t in threads:
		t.join()
	print "Exiting Main Program"

if __name__ == '__main__':
	main()
