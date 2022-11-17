import threading
import serial
import time
import math
import numpy as np

PI = math.pi
TAU = 2 * PI

lockSerial = threading.Lock()
# lockRobot = [None,threading.Lock(),threading.Lock(),threading.Lock()]

unitPerMillimeter = 2.0
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

    lockSerial.acquire()
    packet = [ord('R'),id,pulsesL[0],pulsesL[1],pulsesR[0],pulsesR[1]]
    ser.write(packet)
    lockSerial.release()

def robotSendRotate(id, radians):
    pulses = radiansToEncoderCount(radians)
    pulsesL = pulses.to_bytes(2, byteorder='little', signed=True)
    pulsesR = (-pulses).to_bytes(2, byteorder='little', signed=True)

    lockSerial.acquire()
    packet = [ord('R'),id,pulsesL[0],pulsesL[1],pulsesR[0],pulsesR[1]]
    ser.write(packet)
    lockSerial.release()

# for i in range(1):
robotSendRotate(3, 2*PI)
time.sleep(.2)

