import numpy as np
import argparse
import cv2
import math

def get_image_mask(hsv_img, lower_thres, upper_thres):
  # blurring smooths the image and allows for slightly more accurate circle detection
  mask = cv2.GaussianBlur(hsv_img, (5, 5), 0)
  # construct a mask for the hsv image, then perform closing
  mask = cv2.inRange(mask, lower_thres, upper_thres)
  # mask = cv2.dilate(mask, None, iterations=2)
  # opening_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4,4))
  # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, opening_kernel)
  closing_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (20,20))
  mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, closing_kernel)

  # mask = cv2.erode(mask, None, iterations=2)
  
  return mask

def main():
  # get webcam feed
  camera = cv2.VideoCapture(1)
  # define the lower and upper boundaries of the HSV color space
  hsv_lower = (5, 92, 45)
  hsv_upper = (25, 225, 150)

  # camera focal length calculated by experiment
  f = 744
  # finding distance by : Distance = (Apple Diameter x Camera Focal Length) / Pixel Diameter
  apple_diameter = 7.5 #in cm
  apple_const = apple_diameter*f
  apple_dist = -1

  while True:
    # Load an image in RGV 
    (grabbed, frame) = camera.read()
    # Rescale image to falf x and y
    # frame = cv2.resize(frame, (0,0), fx=0.5, fy=0.5) 

    # convert frame to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
   
    # get mask
    mask = get_image_mask(hsv, hsv_lower, hsv_upper)

    # find contours in the mask 
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
   
    # only proceed if contour was found
    for contour in contours:
      ((x, y), radius) = cv2.minEnclosingCircle(contour)
      #calculate the circularity of the contour
      area = cv2.contourArea(contour)
      perimeter = cv2.arcLength(contour,True)
      if perimeter>0:
        circularity = 4*math.pi*area/perimeter/perimeter

        if radius > 30 and circularity > 0.5:
          # draw the circle on the frame,
          cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
          pixel_diameter = radius*2
          apple_dist = apple_const/pixel_diameter

    cv2.imshow("Mask", mask)
    cv2.putText(frame, "Distance :"+str(apple_dist) ,(10,80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255),2,cv2.LINE_AA)
    cv2.imshow("Frame", frame)
    apple_dist = -1
    key = cv2.waitKey(1) & 0xFF
   
    if key == ord("s"):
      break
   
  # cleanup the camera and close any open windows
  camera.release()
  cv2.destroyAllWindows()

if __name__ == "__main__":
    main()