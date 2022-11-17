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

## Funciones de envío de paquetes y utilidades para los robots

serialLock = threading.Lock()

# Formulas de conversión
markerWidthMillimeters = 97.5
markerWidthUnits = 60.0
unitPerMillimeter = markerWidthUnits / markerWidthMillimeters
pulsesPerRevolution = 60
wheelDiameter = 66.5 * unitPerMillimeter
distanceBetweenWheel = 165.0 * unitPerMillimeter
pulsesPerRad = pulsesPerRevolution / TAU
pulsesPerUnit = pulsesPerRevolution / (wheelDiameter * PI)
pulsesPerRadRobot = pulsesPerRad * (distanceBetweenWheel / wheelDiameter)

# Serial
ser = serial.Serial('/dev/ttyACM0', 230400)
time.sleep(2.5)     # Espera a que inicialice el Arduino Uno

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

## Fin de funciones de envío y utilidades para los robots

# Aquí se almacena la posición y dirección de cada robot
robotPositions = [None, \
                  None, \
                  None, \
                  None]
# Se almacenan los vértices de los marcadores por robot para usarse en cálculo de colisiones
# al seleccionar en ventana el robot y para resaltarlo
robotMarkerCorners = [None, \
                      None, \
                      None, \
                      None]
# Evita que múltiples hilos accedan a las mismas variables al mismo tiempo
robotPositionsLock = threading.Lock()
# Obtiene la posición y orientación de cada robot en base a el lado inferior de su marcador
def robotGetPositions(markerCorners, markerIds):
    global robotPositions
    if markerIds is not None:
        for i in range(len(markerIds)):
            if markerIds[i][0] > 0 and markerIds[i][0] <= 3:
                line = [markerCorners[i][0][3], markerCorners[i][0][2]]

                angle = math.atan2(line[1][1] - line[0][1], line[1][0] - line[0][0])
                position = [line[0][0] + ((line[1][0] - line[0][0])/2), line[0][1] + ((line[1][1] - line[0][1])/2)]

                robotPositionsLock.acquire()
                robotPositions[markerIds[i][0]] = [position, angle]
                robotMarkerCorners[markerIds[i][0]] = markerCorners[i][0]
                robotPositionsLock.release()

# Función sencilla para mover el robot a un punto cartesiano
def robotControl(id, x, y, finalAngle=None):
    global robotPositions
    
    robotPositionsLock.acquire()
    position = robotPositions[id]
    robotPositionsLock.release()

    angle = math.atan2(position[0][1] - y, position[0][0] - x)
    angleDelta = smallestSignedAngleBetween(position[1], angle - (PI/2))
    distance = math.dist(position[0], [x, y])

    robotSendRotate(id, angleDelta)
    time.sleep(2)
    robotSendTranslate(id, distance)

# Función de callback para eventos de mouse
# maneja la selección de un robot en la ventana y permite darles un punto a donde moverse
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
        for i in range(len(corners)):
            if corners[i] is not None:
                square = np.array(corners[i], np.int32)
                collision = cv2.pointPolygonTest(square, [x, y], False)
                if collision > 0:
                    print(i,[x, y], square, collision)
                    robotSelected = i
                    return
        robotSelected = 0

# Inicialización de la cámara con sus parámetros
cap = cv2.VideoCapture(2, cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # 1 = OFF, 3 = ON
cap.set(cv2.CAP_PROP_EXPOSURE, 78)
cap.set(cv2.CAP_PROP_GAIN, 0)
cap.set(cv2.CAP_PROP_SHARPNESS, 191)
cap.set(cv2.CAP_PROP_FOCUS, 19)

# Incialización de ArUco
parameters = cv2.aruco.DetectorParameters_create()
parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# Carga de matriz de perspectiva desde un archivo
perspFile = np.load("perspective_matrix.npz")
perspectiveMatrix = perspFile["perspectiveMatrix"]

# Creación de ventana principal con sus controles
cv2.namedWindow('frame', cv2.WINDOW_GUI_NORMAL)
cv2.resizeWindow('frame', 1200, 900)
cv2.createTrackbar('Gain','frame',int(cap.get(cv2.CAP_PROP_GAIN)),255,gainChange)
cv2.createTrackbar('Exposure','frame',int(cap.get(cv2.CAP_PROP_EXPOSURE)),2047,exposureChange)
cv2.createTrackbar('Sharpness','frame',int(cap.get(cv2.CAP_PROP_SHARPNESS)),255,sharpnessChange)
cv2.createTrackbar('Focus','frame',int(cap.get(cv2.CAP_PROP_FOCUS)),250,focusChange)
cv2.setMouseCallback('frame', mouseCallback)

while(1):
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    warped = cv2.warpPerspective(gray, perspectiveMatrix, (1200, 800))

    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(warped, dictionary, parameters=parameters)

    robotGetPositions(markerCorners, markerIds)

    frameDrawn = cv2.aruco.drawDetectedMarkers(warped, markerCorners, markerIds)

    if robotMarkerCorners[robotSelected] is not None:
        square = np.array(robotMarkerCorners[robotSelected], np.int32)
        square = square.reshape((-1,1,2))
        frameHighlight = cv2.polylines(frameDrawn,[square],True,(255,255,255))
        cv2.imshow('frame', frameHighlight)
    else:
        cv2.imshow('frame', frameDrawn)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
