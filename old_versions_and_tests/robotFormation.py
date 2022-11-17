import math
import numpy as np
import cv2
import serial
import time

def gainChange(x):
    cap.set(cv2.CAP_PROP_GAIN, x)
def exposureChange(x):
    cap.set(cv2.CAP_PROP_EXPOSURE, x)
def sharpnessChange(x):
    cap.set(cv2.CAP_PROP_SHARPNESS, x)
def focusChange(x):
    cap.set(cv2.CAP_PROP_FOCUS, x)

# https://stackoverflow.com/a/1878936
PI = math.pi
TAU = PI * 2
def smallestSignedAngleBetween(x, y):
    a = (x - y) % TAU
    b = (y - x) % TAU
    return -a if a < b else b

ser = serial.Serial('/dev/ttyACM0', 230400)
time.sleep(1)
def robotSendMove(id, velocity, direction):
    # lockRobot[id].acquire()

    vel = velocity.to_bytes(2, byteorder='little', signed=True)
    dir = direction.to_bytes(2, byteorder='little', signed=True)

    ser.write([ord('R'),id,vel[0],vel[1],dir[0],dir[1]])


class PID:
    sumError = 0
    pastError = 0

    def __init__(self, KP=0.0, KI=0.0, KD=0.0):
        self.KP = KP
        self.KI = KI
        self.KD = KD
    
    def control(self, error):
        self.sumError += error
        self.sumError = max(min(150, self.sumError), -150)
        if error == 0.0: self.sumError = 0.0
        print(self.sumError)

        if self.pastError == 0: self.pastError = error

        P = self.KP * error
        I = self.KI * self.sumError
        D = self.KD * (error - self.pastError)

        self.pastError = error

        return P + I + D

robotDirPID = [None, \
               PID(KP=0.15, KI=0.165, KD=0.0), \
               PID(KP=0.15, KI=0.165, KD=0.0), \
               PID(KP=0.15, KI=0.165, KD=0.0)]
def robotControl(markerCorners, markerIds):
    if markerIds is not None:
        print(markerIds[0])
        # print(markerCorners[0][0][3])
        # print(markerCorners[0][0][2])
        line = [markerCorners[0][0][3], markerCorners[0][0][2]]
        # m = (line[1][1] - line[0][1]) / (line[1][0] - line[0][0])
        angle = math.atan2(line[1][1] - line[0][1], line[1][0] - line[0][0])
        position = [line[0][0] + ((line[1][0] - line[0][0])/2), line[0][1] + ((line[1][1] - line[0][1])/2)]
        print(position)
        
        angleDelta = smallestSignedAngleBetween(angle, PI/2)
        if (abs(angleDelta) > (3 * PI/180)):    # Allow 3 degrees of error
            dirError = angleDelta * 128         # make the value larger so the PID work better
        else:
            dirError = 0.0

        print(angle * (180/PI))
        print(angleDelta * (180/PI))

        dirControl = int(robotDirPID[1].control(dirError))

        if dirControl > 127: dirControl = 127
        print(dirControl)
        robotSendMove(1, 0, dirControl)

    pass

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
cap.set(cv2.CAP_PROP_GAIN, 0)
cap.set(cv2.CAP_PROP_SHARPNESS, 191)
cap.set(cv2.CAP_PROP_FOCUS, 19)

parameters = cv2.aruco.DetectorParameters_create()
parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

calFile = np.load('c920_calibration_720p.npz')
cameraMatrix = calFile['cameraMatrix']
distCoeffs = calFile['distCoeffs']
rvecs = calFile['rvecs']
tvecs = calFile['tvecs']

perspFile = np.load("perspective_matrix.npz")
perspectiveMatrix = perspFile["perspectiveMatrix"]

cv2.namedWindow('frame')
cv2.createTrackbar('Gain','frame',int(cap.get(cv2.CAP_PROP_GAIN)),255,gainChange)
cv2.createTrackbar('Exposure','frame',int(cap.get(cv2.CAP_PROP_EXPOSURE)),2047,exposureChange)
cv2.createTrackbar('Sharpness','frame',int(cap.get(cv2.CAP_PROP_SHARPNESS)),255,sharpnessChange)
cv2.createTrackbar('Focus','frame',int(cap.get(cv2.CAP_PROP_FOCUS)),250,focusChange)
# cv2.createTrackbar('Max','frame',0,255,nothing)

while(1):
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # undistorted = cv2.undistort(gray, cameraMatrix, distCoeffs, None, None)
    warped = cv2.warpPerspective(gray, perspectiveMatrix, (1200, 800))
    # resized = cv2.resize(undistorted, (640, 360), interpolation=cv2.INTER_AREA)

    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(warped, dictionary, parameters=parameters)

    robotControl(markerCorners, markerIds)

    frameDrawn = cv2.aruco.drawDetectedMarkers(warped, markerCorners, markerIds)

    # cv2.imshow('frame', frameDrawn[120:1080-120, 160:1920-160])
    cv2.imshow('frame', frameDrawn)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
