import numpy as np
import cv2
import time

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

parameters = cv2.aruco.DetectorParameters_create()
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

while(1):
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    frameDrawn = cv2.aruco.drawDetectedMarkers(gray, markerCorners, markerIds)

    cv2.imshow('frame', frameDrawn)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
