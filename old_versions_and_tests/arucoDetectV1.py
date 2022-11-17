import numpy as np
import cv2
import time

def gainChange(x):
    cap.set(cv2.CAP_PROP_GAIN, x)
def exposureChange(x):
    cap.set(cv2.CAP_PROP_EXPOSURE, x)
def sharpnessChange(x):
    cap.set(cv2.CAP_PROP_SHARPNESS, x)
def focusChange(x):
    cap.set(cv2.CAP_PROP_FOCUS, x)

# cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
# cap = cv2.VideoCapture('v4l2src device=/dev/video2 ! image/jpeg, width=640, height=360, framerate=30/1 ! jpegdec ! appsink', cv2.CAP_GSTREAMER)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FPS, 30)
# cap.set(cv2.CAP_PROP_SETTINGS, 1)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # 1 = OFF, 3 = ON
cap.set(cv2.CAP_PROP_EXPOSURE, 78)
cap.set(cv2.CAP_PROP_GAIN, 70)
cap.set(cv2.CAP_PROP_SHARPNESS, 191)
cap.set(cv2.CAP_PROP_FOCUS, 19)

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)

print(width, height, fps)

# print(cap.getBackendName())

parameters = cv2.aruco.DetectorParameters_create()
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

calFile = np.load('c920_calibration_720p.npz')
cameraMatrix = calFile['cameraMatrix']
distCoeffs = calFile['distCoeffs']
rvecs = calFile['rvecs']
tvecs = calFile['tvecs']

cv2.namedWindow('frame')
cv2.createTrackbar('Gain','frame',int(cap.get(cv2.CAP_PROP_GAIN)),255,gainChange)
cv2.createTrackbar('Exposure','frame',int(cap.get(cv2.CAP_PROP_EXPOSURE)),2047,exposureChange)
cv2.createTrackbar('Sharpness','frame',int(cap.get(cv2.CAP_PROP_SHARPNESS)),255,sharpnessChange)
cv2.createTrackbar('Focus','frame',int(cap.get(cv2.CAP_PROP_FOCUS)),250,focusChange)
# cv2.createTrackbar('Max','frame',0,255,nothing)

while(1):
    # startTime = time.time()

    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    undistorted = cv2.undistort(gray, cameraMatrix, distCoeffs, None, None)
    # minThresh = cv2.getTrackbarPos('Min', 'frame')
    # maxThresh = cv2.getTrackbarPos('Max', 'frame')
    # frame = cv2.flip(frame, -1)
    # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # mask = cv2.inRange(hsv, lower_range, upper_range)
    # resized = cv2.resize(undistorted, (640, 360), interpolation=cv2.INTER_AREA)
    # binarized = cv2.threshold(frame, minThresh, maxThresh, cv2.THRESH_BINARY)
    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(undistorted, dictionary, parameters=parameters)
    if markerIds is not None:
        # print(markerCorners[0][0][0][1])
        pts2 = np.float32([[0 + 400, 0 + 400], [50 + 400, 0 + 400],
                       [50 + 400, 50 + 400], [0 + 400, 50 + 400]])
        matrix = cv2.getPerspectiveTransform(markerCorners[0][0], pts2, cv2.DECOMP_LU)
        # print(matrix)
        warped = cv2.warpPerspective(undistorted, matrix, (800, 800))

        cv2.imshow('warped', warped)

    frameDrawn = cv2.aruco.drawDetectedMarkers(undistorted, markerCorners, markerIds)


    # cv2.imshow('frame', frameDrawn[120:1080-120, 160:1920-160])
    cv2.imshow('frame', frameDrawn)

    # elapsedTime = 1 / (time.time() - startTime)

    # cv2.imshow('mask', mask)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
