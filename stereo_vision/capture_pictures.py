import numpy as np
import cv2
import keyboard

LEFT_PATH = "left_orange_{:06d}.jpg"
RIGHT_PATH = "right_orange_{:06d}.jpg"

CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

# TODO: Use more stable identifiers
left = cv2.VideoCapture(1)
right = cv2.VideoCapture(2)

# Increase the resolution
left.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
left.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
right.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
right.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

# Use MJPEG to avoid overloading the USB 2.0 bus at this resolution
left.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
right.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

# The distortion in the left and right edges prevents a good calibration, so
# discard the edges
CROP_WIDTH = 960
def cropHorizontal(image):
    return image[:,
            int((CAMERA_WIDTH-CROP_WIDTH)/2):
            int(CROP_WIDTH+(CAMERA_WIDTH-CROP_WIDTH)/2)]

frameId = 0
pressed = 0
# Grab both frames first, then retrieve to minimize latency between cameras
while(True and frameId < 100):
    if not (left.grab() and right.grab()):
        print("No more frames")
        break
    
    

    _, leftFrame = left.retrieve()
    leftFrame = cropHorizontal(leftFrame)
    _, rightFrame = right.retrieve()
    rightFrame = cropHorizontal(rightFrame)
    
    #if(keyboard.isPressed('p')):
    
    cv2.imshow('left', leftFrame)
    cv2.imshow('right', rightFrame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    inputVal = raw_input() #See if picture is what we want, i.e blurry or whatever. If poor, hit other key apart from 's' (save) and it will retake the picture      
    if(inputVal == 's'):
        test = cv2.imwrite(LEFT_PATH.format(frameId), leftFrame)
        cv2.imwrite(RIGHT_PATH.format(frameId), rightFrame)
    	frameId += 1
        
    

left.release()
right.release()
cv2.destroyAllWindows()

