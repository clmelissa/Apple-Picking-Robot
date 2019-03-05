import cv2
import numpy as np


cap = cv2.VideoCapture(0)

def nothing(x):
    pass
# Creating a window for later use
cv2.namedWindow('win')

# Starting with 100's to prevent error while masking
h,s,v = 100,100,100

# Creating track bar
cv2.createTrackbar('h_lower', 'win', 0,179, nothing)
cv2.createTrackbar('s_lower', 'win', 0,254, nothing)
cv2.createTrackbar('v_lower', 'win', 0,254, nothing)
cv2.createTrackbar('h_upper', 'win', 1,180, nothing)
cv2.createTrackbar('s_upper', 'win', 1,255, nothing)
cv2.createTrackbar('v_upper', 'win', 1,255, nothing)

while(1):

    _, frame = cap.read()

    #converting to HSV
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    # get info from track bar
    h_lower = cv2.getTrackbarPos('h_lower','win')
    s_lower = cv2.getTrackbarPos('s_lower','win')
    v_lower = cv2.getTrackbarPos('v_lower','win')
    h_upper = cv2.getTrackbarPos('h_upper','win')
    s_upper = cv2.getTrackbarPos('s_upper','win')
    v_upper = cv2.getTrackbarPos('v_upper','win')

    # Normal masking algorithm
    lower_blue = np.array([h_lower,s_lower,v_lower])
    upper_blue = np.array([h_upper,s_upper,v_upper])

    mask = cv2.inRange(hsv,lower_blue, upper_blue)

    result = cv2.bitwise_and(frame,frame,mask = mask)

    cv2.imshow('win',result)

    k = cv2.waitKey(5) & 0xFF
    if k  == ord("s"):
        break

cap.release()
cv2.destroyAllWindows()