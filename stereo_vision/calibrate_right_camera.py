import numpy as np
import cv2
import glob

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
boardWidth = 10-1
boardHeight = 7-1
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((boardWidth*boardHeight,3), np.float32)
objp[:,:2] = np.mgrid[0:boardHeight,0:boardWidth].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

imageDirectory = "capture/right/"
print("Reading images at {0}".format(imageDirectory))
images = glob.glob("{0}/*.jpg".format(imageDirectory))
testImage = "/home/vincent/Documents/4B/FYDP/Stero_Vision/AlbertArmeaStereoVision/capture/right/right_000000.jpg"
#testImage = "/home/vincent/Documents/4B/FYDP/Stero_Vision/opencv/samples/data/right01.jpg"
#images = glob.glob('/home/vincent/Documents/4B/FYDP/Stero_Vision/opencv/samples/data/right*.jpg')
print(len(images))
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    #ret, corners = cv2.findChessboardCorners(gray, (7,6),None)
    ret, corners = cv2.findChessboardCorners(gray, (boardHeight,boardWidth),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (boardHeight,boardWidth), corners2,ret)
        cv2.imshow('img',img)
        cv2.waitKey(500)
    else:
        print "false"

cv2.destroyAllWindows()
print(objpoints)
print(imgpoints)
print(gray.shape[::-1])
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
img = cv2.imread(testImage)
h,  w = img.shape[:2]
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
# undistort

dst = cv2.undistort(img, mtx, dist, None, newcameramtx)


# crop the image
x,y,w,h = roi
print(roi)
print(w)
print(h)
dst = dst[y:y+h, x:x+w]
cv2.imwrite('calibresult.png',dst)