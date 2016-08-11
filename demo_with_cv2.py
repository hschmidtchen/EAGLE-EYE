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

from multiprocessing import Process, Queue, Lock, Value
import time

exitFlag = 0

<<<<<<< HEAD
def manualControlProcess(drone_lock, man_mode_value, key_value, exiting_value):
	
	running = True
	print "Starting Manual Control Process" 
	global drone

	while running:
		# query pressed keys
		k = key_value.value
		drone_lock.acquire()

		# escape to stop program execution
		if k == 27: # 27=escape
			running = False
			exiting_value.value = True
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
			man_mode_value.value = not man_mode_value.value
 		 # switch between manual and autonomous control
		elif man_mode_value.value:
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
			elif k == 2490368 or k == 65362:
				drone.move_up()
			elif k == 2621440 or k == 65364:
				drone.move_down()
			# turn left / turn right
			elif k == 2424832 or k == 65361:
				drone.turn_left()
			elif k == 2555904 or k == 65363:
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
		drone_lock.release()

		  
	print("Shutting down Manual Control Process...")
	drone_lock.acquire()
	drone.halt()
	drone_lock.release()	
	print "Exiting Manual Control Process"
		
=======
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
		
		print "Starting " + self.name
		
		while self.running:
			# query pressed keys
			k = cv2.waitKey(33)
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
		global exiting
		global manual_mode
		global W
		global H
		
		print "Starting " + self.name
		
		p[0] = -W		
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> parent of 96d4869... manual mode runs with threads
=======
>>>>>>> parent of 96d4869... manual mode runs with threads
=======
>>>>>>> parent of 96d4869... manual mode runs with threads
		
def automaticControlProcess(drone_lock, man_mode_value, exiting_value, W, H, frame_queue):
	
	global drone
	
	print "Starting Automatic Control Process"
	status = "Start"
	running = True
	p = (-W	, 0)	
	
	while running:
		if exiting_value.value:
			running = False
		else:		
			if not man_mode_value.value:
				# Tracking 
				if status == "Start":
					# To do with image recognition
					print'start'
				elif status == "Searching":
					# To do turn and search marker
					p,d = getAndSearchImage(drone_lock,W,H)
					if p[0] >= -W/2:
						status = "Tracking"
					else:
						drone_lock.acquire()
						drone.turn_left()
						drone_lock.release()
					print 'Searching'
				elif status == "Tracking":
					# To do image recognition and controler calculation
					if p[0] >= -W/2:
						controlStep(drone_lock,p,d)
						p,d = getAndSearchImage(drone_lock,W,H)
					else:
						self.status = "Searching"
					print 'tracking'
			else:
<<<<<<< HEAD
				try:
					# print pygame.image
					drone_lock.acquire()
					pixelarray = drone.get_image()
					drone_lock.release()
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

						frame_queue.put(resized)

				except:
					pass
			
	print "Exiting Automatic Control Process"
	
def getAndSearchImage(drone_lock,W,H):
	 try:
		# print pygame.image
		drone_lock.acquire()
		pixelarray = drone.get_image()
		drone_lock.release()
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
			print(corners)
			gray = aruco.drawDetectedMarkers(gray, corners)

			# battery status
			#hud_color = (255, 0, 0) if drone.navdata.get('drone_state', dict()).get('emergency_mask', 1) else (10, 10, 255)
			#bat = drone.navdata.get(0, dict()).get('battery', 0)
			# f = pygame.font.Font(None, 20)
			# hud = f.render('Battery: %i%%' % bat, True, hud_color) 
			# screen.blit(hud, (10, 10))

			frame_queue.put(resized)


			x = 1
			y = 2
			hx = 3
			hy = 4
			return (x,y), (hx,hy)
	 except:
		pass
	
def controlStep(drone_lock,p,d):
	
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
		drone_lock.acquire()
		drone.speed = -MAX_SPEED_ROT * ux
		drone.turn_right()
		drone_lock.release()
	elif ux > +ux_threshold :
		drone_lock.acquire()
		drone.speed = MAX_SPEED_ROT * ux
		drone.turn_left()
		drone_lock.release()
	
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
		drone_lock.acquire()
		drone.speed = -MAX_SPEED_ROT * uy
		drone.turn_up()
		drone_lock.release()
	elif uy > +uy_threshold :
		drone_lock.acquire()
		drone.speed = MAX_SPEED_ROT * uy
		drone.turn_down()
		drone_lock.release()
=======
				thread_lock.release()
				
				thread_lock2.acquire()
				if not manual_mode:
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
							drone.turn_left()
						print 'Searching'
					elif self.status == "Tracking":
						# To do image recognition and controler calculation
						if p[0] >= -W/2:
							controlStep(p,d)
							p,d = self.getAndSearchImage()
						else:
							self.status = "Searching"
						print 'tracking'
					# Controler
				
		print "Exiting" + self.name
		
	def getAndSearchImage(self):
		 try:
			# print pygame.image
			pixelarray = drone.get_image()
			if pixelarray != None:
				frame = pixelarray[:,:,::-1].copy()
			#resize image
				resized=cv2.resize(frame,(W,H))
				print 'got image'
			# aruco detection
				gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
				aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
				parameters =  aruco.DetectorParameters_create()
		
				#lists of ids and the corners beloning to each id
				corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
				print(corners)
				gray = aruco.drawDetectedMarkers(gray, corners)

				# battery status
				hud_color = (255, 0, 0) if drone.navdata.get('drone_state', dict()).get('emergency_mask', 1) else (10, 10, 255)
				bat = drone.navdata.get(0, dict()).get('battery', 0)
				# f = pygame.font.Font(None, 20)
				# hud = f.render('Battery: %i%%' % bat, True, hud_color) 
				# screen.blit(hud, (10, 10))
				cv2.imshow('Drone',gray)			
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
			drone.speed = -MAX_SPEED_ROT * ux
			drone.turn_right()
		elif ux > +ux_threshold :
			drone.speed = MAX_SPEED_ROT * ux
			drone.turn_left()
		
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
			drone.speed = -MAX_SPEED_ROT * uy
			drone.turn_up()
		elif uy > +uy_threshold :
			drone.speed = MAX_SPEED_ROT * uy
			drone.turn_down()
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> parent of 96d4869... manual mode runs with threads
=======
>>>>>>> parent of 96d4869... manual mode runs with threads
=======
>>>>>>> parent of 96d4869... manual mode runs with threads
		 

def main():
	global drone
<<<<<<< HEAD
	
	frame_queue = Queue()

	W, H = 320, 240
	key = Value('i',-1)
=======
	global thread_lock
	global thread_lock2
	global exiting
	global manual_mode
	global W
	global H
	W, H = 320, 240
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> parent of 96d4869... manual mode runs with threads
=======
>>>>>>> parent of 96d4869... manual mode runs with threads
=======
>>>>>>> parent of 96d4869... manual mode runs with threads
	
	
	drone = libardrone.ARDrone(True)
	drone.reset()
	
	exiting = Value('b', False)
	manual_mode = Value('b', True)
	
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	processes = []
	drone_lock = Lock()

	drone_lock.acquire()
	drone = libardrone.ARDrone(True)
	drone.reset()
	drone_lock.release()

	# Create new processes
	manual_control_process = Process(target=manualControlProcess, args=(drone_lock, manual_mode,key,exiting))
	automatic_control_process = Process(target=automaticControlProcess, args=(drone_lock, manual_mode, exiting, W, H, frame_queue))
=======
=======
>>>>>>> parent of 96d4869... manual mode runs with threads
=======
>>>>>>> parent of 96d4869... manual mode runs with threads
	threads = []
	thread_lock = threading.Lock()
	thread_lock2 = threading.Lock()

	
	# Create new threads
	manual_control_thread = manualControlThread(1, "Manual Control Thread", 1)
	automatic_control_thread = automaticControlThread(2, "Automatic Control Thread", 2)
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> parent of 96d4869... manual mode runs with threads
=======
>>>>>>> parent of 96d4869... manual mode runs with threads
=======
>>>>>>> parent of 96d4869... manual mode runs with threads
	
	# Start new processes
	manual_control_process.start()
	automatic_control_process.start()
	
	# Add processes to thread list
	processes.append(manual_control_process)
	processes.append(automatic_control_process)
	
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	while not exiting.value:
		key.value = cv2.waitKey(33)
		if not frame_queue.empty():
			cv2.imshow('Drone',frame_queue.get())			
	# Wait for all processes to complete
	for t in processes:
=======
=======
>>>>>>> parent of 96d4869... manual mode runs with threads
=======
>>>>>>> parent of 96d4869... manual mode runs with threads
	# Wait for all threads to complete
	for t in threads:
>>>>>>> parent of 96d4869... manual mode runs with threads
		t.join()
	print "Exiting Main Program"

if __name__ == '__main__':
	main()
