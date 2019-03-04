import sys
import numpy as np
import cv2
import time
from matplotlib import pyplot as plt
REMAP_INTERPOLATION = cv2.INTER_LINEAR

DEPTH_VISUALIZATION_SCALE = 2048*2

if len(sys.argv) != 2:
    print("Syntax: {0} CALIBRATION_FILE".format(sys.argv[0]))
    sys.exit(1)

calibration = np.load(sys.argv[1], allow_pickle=False)
imageSize = tuple(calibration["imageSize"])
leftMapX = calibration["leftMapX"]
leftMapY = calibration["leftMapY"]
leftROI = tuple(calibration["leftROI"])
rightMapX = calibration["rightMapX"]
rightMapY = calibration["rightMapY"]
rightROI = tuple(calibration["rightROI"])

print("image size:" + str(tuple(calibration["imageSize"])))
print(calibration["leftMapX"])
print(calibration["leftMapY"])
print(tuple(calibration["leftROI"]))
print(calibration["rightMapX"])
print(calibration["rightMapY"])
print(tuple(calibration["rightROI"]))


CAMERA_WIDTH = 960
CAMERA_HEIGHT = 720


left = cv2.VideoCapture(1)
right = cv2.VideoCapture(2)

#w = left.get(cv2.CV_CAP_PROP_FRAME_WIDTH)
#h = left.get(cv2.CV_CAP_PROP_FRAME_HEIGHT)
#print w,h

# Increase the resolution
left.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
left.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
right.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
right.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

# Use MJPEG to avoid overloading the USB 2.0 bus at this resolution
left.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
right.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
print("here")
# The distortion in the left and right edges prevents a good calibration, so
# discard the edges
CROP_WIDTH = 960
def cropHorizontal(image):
    return image[:,
            int((CAMERA_WIDTH-CROP_WIDTH)/2):
            int(CROP_WIDTH+(CAMERA_WIDTH-CROP_WIDTH)/2)]

# TODO: Why these values in particular?
# TODO: Try applying brightness/contrast/gamma adjustments to the images
# stereoMatcher = cv2.StereoBM_create()
# stereoMatcher.setMinDisparity(4)
# stereoMatcher.setNumDisparities(128)
# stereoMatcher.setBlockSize(21)
# stereoMatcher.setROI1(leftROI)
# stereoMatcher.setROI2(rightROI)
# stereoMatcher.setSpeckleRange(16)
# stereoMatcher.setSpeckleWindowSize(45)

minDisparity = 4
numDisparities = 128
blockSize = 21
ROI1 = leftROI
ROI2 = rightROI
speckleRange = 7
speckleWindowSize = 60

stereoMatcher = cv2.StereoBM_create()
stereoMatcher.setMinDisparity(minDisparity)
stereoMatcher.setNumDisparities(numDisparities)	
stereoMatcher.setBlockSize(blockSize)
stereoMatcher.setROI1(ROI1)
stereoMatcher.setROI2(ROI2)
stereoMatcher.setSpeckleRange(speckleRange)
stereoMatcher.setSpeckleWindowSize(speckleWindowSize)

imgL = cv2.imread('../opencv_left_0.png')
imgR = cv2.imread('../opencv_right_0.png')
disparity = stereoMatcher.compute(imgL,imgR)
plt.imshow(disparity,'gray')
plt.show()

# Grab both frames first, then retrieve to minimize latency between cameras
while(False): #Switch back later
    if not left.grab() or not right.grab():
        print("No more frames")
        break

    _, leftFrame = left.retrieve()
    leftFrame = cropHorizontal(leftFrame)
    leftHeight, leftWidth = leftFrame.shape[:2]
    _, rightFrame = right.retrieve()
    rightFrame = cropHorizontal(rightFrame)
    rightHeight, rightWidth = rightFrame.shape[:2]

    if (leftWidth, leftHeight) != imageSize:
        print("Left camera has different size than the calibration data")
        break

    if (rightWidth, rightHeight) != imageSize:
        print("Right camera has different size than the calibration data")
        break

    fixedLeft = cv2.remap(leftFrame, leftMapX, leftMapY, REMAP_INTERPOLATION)
    fixedRight = cv2.remap(rightFrame, rightMapX, rightMapY, REMAP_INTERPOLATION)

    grayLeft = cv2.cvtColor(fixedLeft, cv2.COLOR_BGR2GRAY)
    grayRight = cv2.cvtColor(fixedRight, cv2.COLOR_BGR2GRAY)
    depth = stereoMatcher.compute(grayLeft, grayRight)
    print(depth)
    cv2.imshow('left', grayLeft)
    cv2.imshow('right', grayRight)
    cv2.imshow('depth', depth / DEPTH_VISUALIZATION_SCALE)    
    
    #minDisparity, numDisparities, blockSize, speckleRange, speckleWindowSize = input("minDisparity, numDisparities, blockSize, ROI1, ROI2, speckleRange, speckleWindowSize").split(" ")

    stereoMatcher.setMinDisparity(minDisparity)
    stereoMatcher.setNumDisparities(numDisparities)	
    stereoMatcher.setBlockSize(blockSize)    
    stereoMatcher.setSpeckleRange(speckleRange)
    stereoMatcher.setSpeckleWindowSize(speckleWindowSize)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break    


	

left.release()
right.release()
cv2.destroyAllWindows()