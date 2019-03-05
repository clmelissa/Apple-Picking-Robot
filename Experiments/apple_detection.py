import numpy as np
import argparse
import cv2
import math

def get_image_mask(hsv_img, lower_thres, upper_thres):
  # blurring smooths the image and allows for slightly more accurate circle detection, as part of the next step includes edge detection
  mask = cv2.GaussianBlur(hsv_img, (5, 5), 0)
  # construct a mask for the hsv image, then perform closing to remove 'salt and pepper'
  mask = cv2.inRange(mask, lower_thres, upper_thres)
  #mask = cv2.dilate(mask, None, iterations=2)
  #mask = cv2.erode(mask, None, iterations=2)
  kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
  mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
  
  #cv2.imshow("Mask", mask)
  return mask

def main():

  # define the lower and upper boundaries of the HSV color space
  hsv_lower = (0, 45, 45)
  hsv_upper = (18, 225, 150)

  # Load an image in RGV 
  frame = cv2.imread('WIN_20190120_13_04_59_Pro.jpg',1)
  # Rescale image to falf x and y
  frame = cv2.resize(frame, (0,0), fx=0.5, fy=0.5) 

  # convert frame to the HSV color space
  hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 
  # get mask
  mask = get_image_mask(hsv, hsv_lower, hsv_upper)

  # find contours in the mask 
  contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
 
  # only proceed if contour was found
  for contour in contours:
    ((x, y), radius) = cv2.minEnclosingCircle(contour)

    if radius > 20 :
      # draw the circle on the frame,
      cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)

  cv2.imshow("Mask", mask)
  cv2.imshow("Frame", frame)
  key = cv2.waitKey() & 0xFF
   
  if key == ord("s"):
      cv2.destroyAllWindows()

if __name__ == "__main__":
    main()