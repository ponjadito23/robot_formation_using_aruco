import numpy as np
import cv2

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(11, 8, 32e-3, 25e-3, dictionary)
boardImage = board.draw((352*2,256*2))

parameters = cv2.aruco.DetectorParameters_create()
parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(boardImage, dictionary, parameters=parameters)

# frameDrawn = cv2.aruco.drawDetectedMarkers(boardImage, markerCorners, markerIds)

## Obtiene los puntos de referencia para obtener la misma perspectiva en la camara
if len(markerIds) == 44:
    markerSorted = sorted(zip(markerIds, markerCorners))
    markerCornersSorted = [markerCorners for markerIds, markerCorners in markerSorted]
    markerIdsSorted = [markerIds for markerIds, markerCorners in markerSorted]

    refPoints = np.reshape(markerCornersSorted, (-1, 2))

    for i in range(len(refPoints)):
        refPoints[i][0] = refPoints[i][0] + 1600
        refPoints[i][1] = refPoints[i][1] + 1200
    # matrix = cv2.getPerspectiveTransform(markerCorners[0][0], pts2, cv2.DECOMP_LU)
    # print(matrix)


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
cap.set(cv2.CAP_PROP_GAIN, 30)
cap.set(cv2.CAP_PROP_SHARPNESS, 191)
cap.set(cv2.CAP_PROP_FOCUS, 19)

# calFile = np.load('c920_calibration_720p.npz')
# cameraMatrix = calFile['cameraMatrix']
# distCoeffs = calFile['distCoeffs']
# rvecs = calFile['rvecs']
# tvecs = calFile['tvecs']

markerIds = []
while markerIds is None or len(markerIds) != 44:
# while 1:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # undistorted = cv2.undistort(gray, cameraMatrix, distCoeffs, None, None)

    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

    frameDrawn = cv2.aruco.drawDetectedMarkers(gray, markerCorners, markerIds)

    # cv2.imshow('frame', frameDrawn[0:720, 200:1280-200])
    cv2.imshow('frame', frameDrawn)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

markerSorted = sorted(zip(markerIds, markerCorners))
markerCornersSorted = [markerCorners for markerIds, markerCorners in markerSorted]
markerIdsSorted = [markerIds for markerIds, markerCorners in markerSorted]

dstPoints = np.reshape(markerCornersSorted, (-1, 2))

matrix = cv2.findHomography(dstPoints, refPoints*.3)
# print(markerCorners[0][0][0][1])
# pts2 = np.float32([[0 + 400, 0 + 400], [50 + 400, 0 + 400],
#                 [50 + 400, 50 + 400], [0 + 400, 50 + 400]])
# matrix = cv2.getPerspectiveTransform(markerCorners[0][0], pts2, cv2.DECOMP_LU)
# print(matrix)
np.savez("perspective_matrix.npz", perspectiveMatrix=matrix[0])
warped = cv2.warpPerspective(gray, matrix[0], (1200, 800))

cv2.imshow('warped', warped)

cv2.imshow('frame', frameDrawn)
cv2.waitKey(0)