from tkinter import Tk,Frame
import threading
import time
import numpy as np

inputBuffer = 0
stopRunning = 0
lock = threading.Lock()

def keyup(e):
    global inputBuffer
    # print('up', int(e.keycode))
    lock.acquire()
    if e.keycode == 37:
        inputBuffer = inputBuffer & 0b1110
    elif e.keycode == 38:
        inputBuffer = inputBuffer & 0b1101
    elif e.keycode == 39:
        inputBuffer = inputBuffer & 0b1011
    elif e.keycode == 40:
        inputBuffer = inputBuffer & 0b0111
    lock.release()

def keydown(e):
    global inputBuffer
    # print('down', int(e.keycode))
    lock.acquire()
    if e.keycode == 37:
        inputBuffer = inputBuffer | 0b0001
    elif e.keycode == 38:
        inputBuffer = inputBuffer | 0b0010
    elif e.keycode == 39:
        inputBuffer = inputBuffer | 0b0100
    elif e.keycode == 40:
        inputBuffer = inputBuffer | 0b1000
    lock.release()

def serialSender():
    global inputBuffer, stopRunning
    while not stopRunning:
        lock.acquire()
        print(format(inputBuffer, '#06b'))
        lock.release()
        time.sleep(.010)


root = Tk()
frame = Frame(root, width=100, height=100)
frame.bind("<KeyPress>", keydown)
frame.bind("<KeyRelease>", keyup)
frame.pack()
frame.focus_set()

process = threading.Thread(target=serialSender)
process.start()

root.mainloop()

stopRunning = 1