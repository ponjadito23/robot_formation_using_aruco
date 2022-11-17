import threading
import serial
import time
import numpy as np

lockSerial = threading.Lock()
# lockRobot = [None,threading.Lock(),threading.Lock(),threading.Lock()]

ser = serial.Serial('/dev/ttyACM0', 230400)

time.sleep(2.5)

def robotControl(id, velocity, direction):
    # lockRobot[id].acquire()

    vel = velocity.to_bytes(2, byteorder='little', signed=True)
    dir = direction.to_bytes(2, byteorder='little', signed=True)

    lockSerial.acquire()
    ser.write([ord('R'),id,vel[0],vel[1],dir[0],dir[1]])
    lockSerial.release()

    # lockRobot[id].release()

# for i in range(1):
while True:
    robotControl(3, 100, 0)
    time.sleep(.05)
    # robotControl(1, 0, 0)
    # time.sleep(.05)
robotControl(3, 0, 0)
