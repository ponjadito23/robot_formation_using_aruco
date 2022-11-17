import threading
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

## Robot Sender Functions

serialLock = threading.Lock()
# lockRobot = [None,threading.Lock(),threading.Lock(),threading.Lock()]
markerWidthMillimeters = 97.5
markerWidthUnits = 60.0
unitPerMillimeter = markerWidthUnits / markerWidthMillimeters
pulsesPerRevolution = 60
wheelDiameter = 66.5 * unitPerMillimeter
distanceBetweenWheel = 165.0 * unitPerMillimeter
pulsesPerRad = pulsesPerRevolution / TAU
pulsesPerUnit = pulsesPerRevolution / (wheelDiameter * PI)
pulsesPerRadRobot = pulsesPerRad * (distanceBetweenWheel / wheelDiameter)

ser = serial.Serial('/dev/ttyACM0', 230400)

time.sleep(2.5)

def distanceToEncoderCount(distance):
    return round(distance * pulsesPerUnit)

def radiansToEncoderCount(rads):
    return round(rads * pulsesPerRadRobot)

def robotSendTranslate(id, distance):
    pulses = distanceToEncoderCount(distance)
    pulsesL = pulses.to_bytes(2, byteorder='little', signed=True)
    pulsesR = pulses.to_bytes(2, byteorder='little', signed=True)

    serialLock.acquire()
    packet = [ord('R'),id,pulsesL[0],pulsesL[1],pulsesR[0],pulsesR[1]]
    ser.write(packet)
    serialLock.release()

def robotSendRotate(id, radians):
    pulses = radiansToEncoderCount(radians)
    pulsesL = pulses.to_bytes(2, byteorder='little', signed=True)
    pulsesR = (-pulses).to_bytes(2, byteorder='little', signed=True)

    serialLock.acquire()
    packet = [ord('R'),id,pulsesL[0],pulsesL[1],pulsesR[0],pulsesR[1]]
    ser.write(packet)
    serialLock.release()

## End of Robot Sender Functions


robotPositions = [None, \
                  None, \
                  None, \
                  None]
robotMarkerCorners = [None, \
                      None, \
                      None, \
                      None]
robotPositionsLock = threading.Lock()
def robotGetPositions(markerCorners, markerIds):
    global robotPositions
    if markerIds is not None:
        # Sorts the markers by id
        # markerSorted = sorted(zip(markerIds, markerCorners))
        # markerCornersSorted = [markerCorners for markerIds, markerCorners in markerSorted]
        # markerIdsSorted = [markerIds for markerIds, markerCorners in markerSorted]
        # markerIdsSorted = np.concatenate(markerIdsSorted)
        for i in range(len(markerIds)):
            # print(markerIds[i][0])
            # We only use robots with id 1,2,3
            if markerIds[i][0] > 0 and markerIds[i][0] <= 3:
                line = [markerCorners[i][0][3], markerCorners[i][0][2]]
                # print(math.dist(line[0], line[1]))

                angle = math.atan2(line[1][1] - line[0][1], line[1][0] - line[0][0])
                position = [line[0][0] + ((line[1][0] - line[0][0])/2), line[0][1] + ((line[1][1] - line[0][1])/2)]

                robotPositionsLock.acquire()
                robotPositions[markerIds[i][0]] = [position, angle]
                robotMarkerCorners[markerIds[i][0]] = markerCorners[i][0]
                robotPositionsLock.release()
                # print(robotPositions[i])
                
                # angleDelta = smallestSignedAngleBetween(angle, PI/2)
                # if (abs(angleDelta) > (3 * PI/180)):
                #     dirError = angleDelta * 128
                # else:
                #     dirError = 0.0

                # print(angle * (180/PI))
                # print(angleDelta * (180/PI))
        # print(robotPositions[1])
        # print(robotPositions[2])
        # print(robotPositions[3])
        # dirControl = int(robotDirPID[1].control(dirError))

        # if dirControl > 127: dirControl = 127
        # print(dirControl)
        # robotSendMove(1, 0, dirControl)

# robotMoved = 1
def robotControl(id, x, y, finalAngle=None):
    global robotPositions
    # if robotPositions[id] is not None:
    #     angleDelta = smallestSignedAngleBetween(robotPositions[id][1], PI/2)

    #     if robotMoved == 0:
    #         robotSendRotate(id, angleDelta)

    #     # print(angleDelta * (180/PI))
    #     # robotMoved = 1
    robotPositionsLock.acquire()
    position = robotPositions[id]
    robotPositionsLock.release()

    angle = math.atan2(position[0][1] - y, position[0][0] - x)
    angleDelta = smallestSignedAngleBetween(position[1], angle - (PI/2))
    distance = math.dist(position[0], [x, y])

    robotSendRotate(id, angleDelta)
    time.sleep(2)
    robotSendTranslate(id, distance)



robotSelected = 0
def mouseCallback(event,x,y,flags,params):
    global robotPositionsLock, robotMarkerCorners, robotSelected
    if event == cv2.EVENT_RBUTTONDOWN:
        if robotSelected != 0:
            process = threading.Thread(target=robotControl, args=(robotSelected, x, y,))
            process.start()
            print(x, y)
    elif event == cv2.EVENT_LBUTTONDOWN:
        robotPositionsLock.acquire()
        corners = robotMarkerCorners
        robotPositionsLock.release()
        # print(corners)
        for i in range(len(corners)):
            if corners[i] is not None:
                square = np.array(corners[i], np.int32)
                collision = cv2.pointPolygonTest(square, [x, y], False)
                if collision > 0:
                    print(i,[x, y], square, collision)
                    robotSelected = i
                    return
        robotSelected = 0


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

# calFile = np.load('c920_calibration_720p.npz')
# cameraMatrix = calFile['cameraMatrix']
# distCoeffs = calFile['distCoeffs']
# rvecs = calFile['rvecs']
# tvecs = calFile['tvecs']

perspFile = np.load("perspective_matrix.npz")
perspectiveMatrix = perspFile["perspectiveMatrix"]

cv2.namedWindow('frame', cv2.WINDOW_GUI_NORMAL)
cv2.resizeWindow('frame', 1200, 900)
cv2.createTrackbar('Gain','frame',int(cap.get(cv2.CAP_PROP_GAIN)),255,gainChange)
cv2.createTrackbar('Exposure','frame',int(cap.get(cv2.CAP_PROP_EXPOSURE)),2047,exposureChange)
cv2.createTrackbar('Sharpness','frame',int(cap.get(cv2.CAP_PROP_SHARPNESS)),255,sharpnessChange)
cv2.createTrackbar('Focus','frame',int(cap.get(cv2.CAP_PROP_FOCUS)),250,focusChange)
cv2.setMouseCallback('frame', mouseCallback)
# cv2.createTrackbar('Max','frame',0,255,nothing)

while(1):
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # undistorted = cv2.undistort(gray, cameraMatrix, distCoeffs, None, None)
    warped = cv2.warpPerspective(gray, perspectiveMatrix, (1200, 800))
    # resized = cv2.resize(undistorted, (640, 360), interpolation=cv2.INTER_AREA)

    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(warped, dictionary, parameters=parameters)

    robotGetPositions(markerCorners, markerIds)
    # robotControl()

    frameDrawn = cv2.aruco.drawDetectedMarkers(warped, markerCorners, markerIds)

    if robotMarkerCorners[robotSelected] is not None:
        square = np.array(robotMarkerCorners[robotSelected], np.int32)
        square = square.reshape((-1,1,2))
        frameHighlight = cv2.polylines(frameDrawn,[square],True,(255,255,255))
        cv2.imshow('frame', frameHighlight)
    else:
        # cv2.imshow('frame', frameDrawn[120:1080-120, 160:1920-160])
        cv2.imshow('frame', frameDrawn)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
