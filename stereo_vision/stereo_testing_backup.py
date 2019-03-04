
import stereovision
import cv2
CAMERA_HEIGHT = 720
CAMERA_WIDTH = 1280


left = cv2.VideoCapture(1)
right = cv2.VideoCapture(2)

left.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
left.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

right.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
right.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

while(True):
	if not (left.grab() and right.grab()):
		print("No Frames")
		break


	_, leftFrame = left.retrieve()
	_, rightFrame = right.retrieve()

	cv2.imshow('left', leftFrame)
	cv2.imshow('right', rightFrame)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

left.release()
right.release()
cv2.destroyAllWindows()