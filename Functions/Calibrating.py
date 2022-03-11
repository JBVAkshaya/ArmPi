#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()

# 夹持器夹取时闭合的角度
servo1 = 500

def initMove():
    Board.setBusServoPulse(1, servo1 - 50, 500)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

__isRunning = False
def reset():
    initMove()

def init():
    global __isRunning
    reset()
    __isRunning = True
    print("Calibration Init")

def start():
    global __isRunning
    __isRunning = True
    print("Calibration Start")

def stop():
    global __isRunning
    __isRunning = False
    print("Calibration Stop")

def exit():
    global __isRunning
    __isRunning = False
    print("Calibration Exit")

def run(img):
    global __isRunning
    
    if not __isRunning:
        return img
    
    img_h, img_w = img.shape[:2]
    cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 255), 2)
    cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 255), 2)
    return img

if __name__ == '__main__':
    init()
    start()
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)           
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()
