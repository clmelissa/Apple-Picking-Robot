import numpy as np
import argparse
import cv2
import math
import serial
import time
import sys

# This should be the UNO, check which port
ser = serial.Serial("/dev/ttyACM1", 9600, timeout=2)

opening_kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (20,20))
closing_kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (45,45))
opening_kernel1 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
closing_kernel1 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (20,20))

def get_image_mask(hsv_img, lower_thres, upper_thres, first_camera):
	# blurring smooths the image and allows for slightly more accurate circle detection
	mask = cv2.GaussianBlur(hsv_img, (5, 5), 0)
	# construct a mask for the hsv image, then perform closing
	mask = cv2.inRange(mask, lower_thres, upper_thres)
	# mask = cv2.dilate(mask, None, iterations=2)
	if first_camera:
		mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, opening_kernel1)
		mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, closing_kernel1)
	else :
		mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, opening_kernel2)
		mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, closing_kernel2)
	
	return mask

def write_on_frame(frame, text, location):
	pass
	# cv2.putText(frame, text, location, cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),2,cv2.LINE_AA)

# Angle of scara arm to get to xed,yed
# We probably want to buffer it by 5 cm
# And then do autonomous navigation to accurately pick the apple
def invKin(xed,yed):
	L1 = 35.56
	L2 = 58.1
	th1d = atan2(yed,xed) - acos((pow(xed,2)+pow(yed,2)+pow(L1,2)-pow(L2,2))/(2*L1*sqrt(pow(xed,2)+pow(yed,2))))

	if ( th1d > pi/2 or th1d < -pi/2) :
		th1d = atan2(yed,xed) + acos((pow(xed,2)+pow(yed,2)+pow(L1,2)-pow(L2,2))/(2*L1*sqrt(pow(xed,2)+pow(yed,2))))
	
	th2d = acos( (pow(xed,2) + pow(yed,2) - pow(L1,2) - pow(L2,2) )/ (2*L1*L2) )

	th1d = th1d*180/pi
	th2d = th2d*180/pi
	th1d = 90-th1d
	return th1d,th2d

# Always end command with \n
def send_command_to_arduino(command):
	command = command+"\n"
	print("Sending to arduino : " + command)
	ser.write(command.encode())
	time.sleep(0.1)

def read_command_from_arduino():
	arduino_input = ""
	if (ser.in_waiting>0):
		arduino_input = ser.read(ser.in_waiting)  
		print("Read arduino_input: "+arduino_input)
	return arduino_input

def get_contours(frame, hsv_lower, hsv_upper, mode):
	# Rescale image to falf x and y
	# frame = cv2.resize(frame, (0,0), fx=0.5, fy=0.5) 

	# convert frame to the HSV color space
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 
	# get mask
	mask = get_image_mask(hsv, hsv_lower, hsv_upper, mode)

	# find contours in the mask 
	contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

	return contours

def get_arm_to_pos(s_angle,e_angle):
	send_command_to_arduino("starget " + str(s_angle))
	arduino_input = ""
	while("g" not in arduino_input):
		if (ser.in_waiting>0):
			arduino_input = ser.read(ser.in_waiting)
			print("Arduino command: "+arduino_input)  
		time.sleep(0.5)
	send_command_to_arduino("e sp 60")
	send_command_to_arduino("etarget " + str(e_angle))
	arduino_input = ""
	while("g" not in arduino_input):
		if (ser.in_waiting>0):
			arduino_input = ser.read(ser.in_waiting)
			print("Arduino command: "+arduino_input)  
		time.sleep(0.5)

	send_command_to_arduino("e sp 75")

def main():
	if len(sys.argv) < 2:
		print "ERROR: not enough argument"
		quit()
	mode = int(sys.argv[1])

	# get webcam feed
	# Mode 1 = camera on the vertical link
	if mode==1:
		print "Running Mode 1"
		camera = cv2.VideoCapture(1)
		camera_offset_x = 3.7
		camera_offset_y = 7.82
		circular_thres = 0.75
		radius_thres = 20
	# Mode 2 = camera on the arm link
	else:
		camera = cv2.VideoCapture(2)
		camera_offset_x = 0.1
		camera_offset_y = 3.8
		circular_thres = 0.55
		radius_thres = 40

	# define the lower and upper boundaries of the HSV color space
	hsv_lower = (0, 70, 60)
	hsv_upper = (18, 255, 255)

	# camera focal length calculated by experiment
	f = 748
	# finding distance by : Distance = (Apple Diameter x Camera Focal Length) / Pixel Diameter
	apple_diameter = 6.6 #in cm
	apple_const = apple_diameter*f

	str_input = ""

	# pixel tolerance
	x_tol = 30
	y_tol = 30

	print("Start program")

	send_command_to_arduino("start")


	while 1:
		if (ser.in_waiting>0):
			arduino_input = read_command_from_arduino()
			if "Set up done" in arduino_input or "run" in arduino_input:
				break
		else:
			time.sleep(0.5)

	shoulder = False
	elbow = False

	print("Start detection")
	send_command_to_arduino("e ccw") 
	send_command_to_arduino("s ccw")

	user_approve = ""

	scommand = "s run"
	ecommand = "e run"

	movement_thres_X = 0.15
	arrive = False
	close_to_apple = False
	fast_speed = True

	while True:
		
		arduino_input = read_command_from_arduino()  

		# if "run" == arduino_input:
		# Load an image in RGV 

		(grabbed, frame) = camera.read()
		
		contours = get_contours(frame,hsv_lower, hsv_upper, mode ==1)
	 
		# only proceed if contour was found
		for contour in contours:
			((x, y), radius) = cv2.minEnclosingCircle(contour)
			#calculate the circularity of the contour
			area = cv2.contourArea(contour)
			perimeter = cv2.arcLength(contour,True)
			if perimeter>0:
				circularity = 4*math.pi*area/perimeter/perimeter

				if radius > radius_thres and circularity > circular_thres:
					# draw the circle on the frame,
					# print("Detected an apple")
					# cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
					pixel_diameter = radius*2
					apple_dist = apple_const/pixel_diameter
					if mode == 2:
						if apple_dist < 19:
							send_command_to_arduino("s stop")
							send_command_to_arduino("e stop")
							print("arive")
							arrive = True
							# exit()
						elif apple_dist < 22:
							movement_thres_X = -0.1
							circular_thres = 0.2
							if not close_to_apple:
								close_to_apple = True
								closing_kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (60,60))
						elif apple_dist < 26 and fast_speed:
							send_command_to_arduino("s sp 78")
							send_command_to_arduino("e sp 85")
							movement_thres_X = 0.05
							radius_thres = 70
							circular_thres = 0.4
							fast_speed = False
						else:
							circular_thres = 0.65
					x_center = np.shape(frame)[1]/2
					y_center = np.shape(frame)[0]/2
					center = True

					# Calculating pixel/cm 
					pixel_ratio = (pixel_diameter)/apple_diameter
					# Distance between detected apple center and desired center          
					pixel_offset_y = pixel_ratio * camera_offset_y
					y_center = y_center+pixel_offset_y
					mov_y = (abs(y - y_center)/pixel_ratio )          

					# Distance between detected apple center and desired center            
					pixel_offset_x = pixel_ratio * camera_offset_x
					x_center = x_center+pixel_offset_x
					mov_x = (abs(x - x_center)/pixel_ratio ) - 2

					write_on_frame(frame, "Distance :"+str(apple_dist), (10,80)) 

					# Vertical distance
					# calculate how much is has to move up & down
					if mode == 1:
						if y < (y_center - 25):
							steps = int((mov_y)*990)
							# steps = 4000
							# write_on_frame(frame, "Up  " + str(mov_y)+ "cm", (400,200))   
							# write_on_frame(frame, "Up  " + str(steps)+ "steps", (300,200))   
							send_command_to_arduino("u " + str(steps))
							center = False
						elif y > (y_center + 25):
							steps = int(mov_y*970) +500
							# write_on_frame(frame, "Down  " + str(mov_y) + "cm", (400,200))  
							# write_on_frame(frame, "Down  " + str(steps) + "steps", (400,200))  
							send_command_to_arduino("d " + str(steps))
							center = False

						if not center:
							# cv2.imshow("Mask", mask)
							# cv2.imshow("Frame", frame)
							# wait until arduino send g to indicate it has finish moving
							arduino_input = ""
							while("g" not in arduino_input):
								if (ser.in_waiting>0):
									arduino_input = ser.read(ser.in_waiting)
									print("Arduino command: "+arduino_input)  
								time.sleep(0.5)
						
						print("moving arm links")
						
						# user_approve = raw_input("Go to mode 2? ")
						user_approve = "yyy"
						if "y" in user_approve:  
						#Switch to mode 2
							camera.release()
							mode = 2
							camera = cv2.VideoCapture(2)
							camera_offset_x = -0.1
							camera_offset_y = 3.8
							circular_thres = 0.6
							radius_thres = 30
							time.sleep(0.1)
							# elbow angle is 73.24
							# shoulder angle is -67.33

							get_arm_to_pos(-50,78)

							send_command_to_arduino("e ccw") 
							send_command_to_arduino("s ccw")

							time.sleep(0.1)
						else:
							camera.release()
							quit()
						

					# Horizontal distance
					if mode == 2:
						if arrive:
							if x < (x_center - 10) :
								print "end left"
								send_command_to_arduino("end")
							elif x > (x_center + 10) :
								print "end right"
								send_command_to_arduino("e ccw") 
								send_command_to_arduino("end")
							time.sleep(0.2)
							# user_approve = raw_input("Grip? ")
							user_approve = "yyy"
							if "y" in user_approve:  
								send_command_to_arduino("grip")
								arduino_input = ""
								while("g" not in arduino_input):
									if (ser.in_waiting>0):
										arduino_input = ser.read(ser.in_waiting)
										print("Arduino command: "+arduino_input)  
									time.sleep(0.5)

							send_command_to_arduino("s sp 60")
							send_command_to_arduino("e sp 70")
							# user_approve = raw_input( "carry the apple back?")
							time.sleep(1)
							send_command_to_arduino("etarget 40")

							send_command_to_arduino("starget -50")
							time.sleep(0.1)
							arduino_input = ""
							while("g" not in arduino_input):
								if (ser.in_waiting>0):
									arduino_input = ser.read(ser.in_waiting)
									print("Arduino command: "+arduino_input)  
								time.sleep(0.2)

							send_command_to_arduino("starget -70")
							time.sleep(0.1)

							arduino_input = ""
							while("g" not in arduino_input):
								if (ser.in_waiting>0):
									arduino_input = ser.read(ser.in_waiting)
									print("Arduino command: "+arduino_input)  
								time.sleep(0.2)

							send_command_to_arduino("release")
							quit()

						if x < (x_center - 25) and not elbow and mov_x > movement_thres_X:
							# write_on_frame(frame, "Left  " + str(mov_x)+ "cm", (10,200))   
							if shoulder:
								send_command_to_arduino("s stop") 
								shoulder = False
							# send_command_to_arduino("e cw")

							# user_approve = raw_input("About to send " + command + " : ")
							# if "y" in user_approve:   
								# print "run " + command
							send_command_to_arduino(ecommand)
							elbow = True
						elif x > (x_center + 25) and not shoulder and mov_x > movement_thres_X:
							# write_on_frame(frame, "right  " + str(mov_x)+ "cm", (10,200))  
							if elbow:
								send_command_to_arduino("e stop")  
								elbow = False
							# send_command_to_arduino("s ccw")
							command = "s run"
							# user_approve = raw_input("About to send " + command + " : ")
							# if "y" in user_approve:  
								# print "run " + command
							send_command_to_arduino(scommand)
							shoulder = True

						elif (x < (x_center + 25)) and (x > (x_center - 25)):
							write_on_frame(frame, "CENTER", (10,200))  
							# if shoulder:
							#   send_command_to_arduino("s stop")
							if elbow:
								send_command_to_arduino("e stop")
								elbow = False
							if not shoulder:
								# user_approve = raw_input("About to send " + command+ " to center it : ")
								# if "y" in user_approve: 
								send_command_to_arduino(scommand)
								shoulder = True

						# Only do one apple at a time
						# break;
					
				 
			# cv2.imshow("Mask", mask)
			# cv2.imshow("Frame", frame)
			# key = cv2.waitKey(1) & 0xFF
			apple_dist = -1
		 
			# if key == ord("s"):
			#   break
	 
	# cleanup the camera and close any open windows
	camera.release()
	cv2.destroyAllWindows()

if __name__ == "__main__":
		main()