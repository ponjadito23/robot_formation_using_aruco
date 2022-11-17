import numpy as np
import cv2

# cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
# cap = cv2.VideoCapture(1, cv2.CAP_GSTREAMER)
cap = cv2.VideoCapture('v4l2src device=/dev/video2 ! image/jpeg, width=640, height=360, framerate=30/1 ! jpegdec ! appsink', cv2.CAP_GSTREAMER)


# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
# cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
# cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
# cap.set(cv2.CAP_PROP_FPS, 30)



width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)

print(width, height, fps)

# print(cap.getBackendName())

lower_range = np.array([94, 100, 100], dtype=np.uint8)
upper_range = np.array([114, 255, 255], dtype=np.uint8)

while(1):
    ret, frame = cap.read()
    # frame = cv2.flip(frame, -1)
    # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # mask = cv2.inRange(hsv, lower_range, upper_range)
    # resized = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_AREA)
    cv2.imshow('frame', frame)
    # cv2.imshow('mask', mask)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
