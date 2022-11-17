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

print(cv2.ocl.haveOpenCL())
cv2.ocl.setUseOpenCL(False)
print(cv2.ocl.useOpenCL())

# cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
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

# calFile = np.load('c920_calibration_720p.npz')
# cameraMatrix = calFile['cameraMatrix']
# distCoeffs = calFile['distCoeffs']
# rvecs = calFile['rvecs']
# tvecs = calFile['tvecs']

perspFile = np.load("perspective_matrix.npz")
perspectiveMatrix = perspFile["perspectiveMatrix"]

cv2.namedWindow('frame')
cv2.createTrackbar('Gain','frame',int(cap.get(cv2.CAP_PROP_GAIN)),255,gainChange)
cv2.createTrackbar('Exposure','frame',int(cap.get(cv2.CAP_PROP_EXPOSURE)),2047,exposureChange)
cv2.createTrackbar('Sharpness','frame',int(cap.get(cv2.CAP_PROP_SHARPNESS)),255,sharpnessChange)
cv2.createTrackbar('Focus','frame',int(cap.get(cv2.CAP_PROP_FOCUS)),250,focusChange)

while(1):
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # undistorted = cv2.undistort(gray, cameraMatrix, distCoeffs, None, None)
    warped = cv2.warpPerspective(gray, perspectiveMatrix, (1200, 800))

    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(warped, dictionary, parameters=parameters)
    frameDrawn = cv2.aruco.drawDetectedMarkers(warped, markerCorners, markerIds)

    cv2.imshow('frame', frameDrawn)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
