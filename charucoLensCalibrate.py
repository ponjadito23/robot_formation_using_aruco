import time
import cv2
import numpy as np

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(11, 8, 15e-3, 12e-3, dictionary)
img = board.draw((260*4,200*4))

#Dump the calibration board to a file
cv2.imwrite('charuco.png',img)


#Start capturing images for calibration
cap = cv2.VideoCapture(2, cv2.CAP_V4L2)

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

allCorners = []
allIds = []
decimator = 0
for i in range(300):

    ret,frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    res = cv2.aruco.detectMarkers(gray,dictionary)

    if len(res[0])>0:
        res2 = cv2.aruco.interpolateCornersCharuco(res[0],res[1],gray,board)
        if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%3==0:
            allCorners.append(res2[1])
            allIds.append(res2[2])

        cv2.aruco.drawDetectedMarkers(gray,res[0],res[1])

    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    decimator+=1

imsize = gray.shape

#Calibration fails for lots of reasons. Release the video if we do
try:
    print("Trying cal")
    ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(allCorners,allIds,board,imsize,None,None)
    print("ret: ", ret)
    print("cameraMatrix: ", cameraMatrix)
    print("distCoeffs: ", distCoeffs)
    print("rvecs: ", rvecs)
    print("tvecs: ", tvecs)
    np.savez("c920_calibration_720p.npz", ret=ret, cameraMatrix=cameraMatrix, distCoeffs=distCoeffs, rvecs=rvecs, tvecs=tvecs)
except:
    cap.release()

cap.release()
cv2.destroyAllWindows()