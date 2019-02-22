import numpy as np
import argparse
import cv2
import math

def get_image_mask(hsv_img, lower_thres, upper_thres):
  # blurring smooths the image and allows for slightly more accurate circle detection, as part of the next step includes edge detection
  mask = cv2.GaussianBlur(hsv_img, (5, 5), 0)
  # construct a mask for the hsv image, then perform closing to remove 'salt and pepper'
  mask = cv2.inRange(mask, lower_thres, upper_thres)
  # mask = cv2.dilate(mask, None, iterations=2)
  opening_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4,4))
  mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, opening_kernel)
  closing_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10,10))
  mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, closing_kernel)

  # mask = cv2.erode(mask, None, iterations=2)
  
  return mask

def main():
  # define the lower and upper boundaries of the HSV color space
  hsv_lower = (5, 92, 0)
  hsv_upper = (180, 255, 150)

  # Load an image in RGB 
  frame = cv2.imread('DSCN1771.JPG',1)
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
  

if __name__ == "__main__":
    main()