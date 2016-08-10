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
import libardrone.libardrone as libardrone
import numpy as np

def main():
    W, H = 320, 240
    drone = libardrone.ARDrone(True)
    drone.reset()
    running = True
    manual_mode = True
    while running:
	    # query pressed keys
            k = cv2.waitKey(33)
	    # escape to stop program execution
            if k == 27: # 27=escape
                running = False
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
		manual_mode = !manual_mode
	
 	    # switch between manual and autonomous control
	    elif manual_mode
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
		else
		    #autonomous mode takes over

            try:
            # print pygame.image
                pixelarray = drone.get_image()
                if pixelarray != None:
                    frame = pixelarray[:,:,::-1].copy()
                #resize image
                    resized=cv2.resize(frame,(W,H))
                    print 'got image'
                #image conversion
                
            # battery status
                hud_color = (255, 0, 0) if drone.navdata.get('drone_state', dict()).get('emergency_mask', 1) else (10, 10, 255)
                bat = drone.navdata.get(0, dict()).get('battery', 0)
            # f = pygame.font.Font(None, 20)
            # hud = f.render('Battery: %i%%' % bat, True, hud_color) 
            # screen.blit(hud, (10, 10))
            
                cv2.imshow('Drone',resized)
            except:
                pass


    print("Shutting down...")
    drone.halt()
    print("Ok.")

if __name__ == '__main__':
    main()
