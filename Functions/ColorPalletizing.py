#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

__target_color = ('red', 'green', 'blue')
def setTargetColor(target_color):
    global __target_color

    #print("COLOR", target_color)
    __target_color = target_color
    return (True, ())

# Find the contour with the largest area
# argument is a list of contours to compare
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # iterate over all contours
        contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate the contour area
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:  # The contour with the largest area is valid only if the area is greater than 300 to filter out the noise
                area_max_contour = c

    return area_max_contour, contour_area_max  # returns the largest contour

# The angle at which the gripper closes when gripping
servo1 = 500

# initial position
def initMove():
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)

# Set the color of the RGB lights of the expansion board to match the color to be tracked
def set_rgb(color):
    if color == "red":
        Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
        Board.RGB.show()
    elif color == "green":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
        Board.RGB.show()
    elif color == "blue":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
        Board.RGB.show()
    else:
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
        Board.RGB.show()

count = 0
_stop = False
color_list = []
get_roi = False
__isRunning = False
move_square = False
detect_color = 'None'
start_pick_up = False
start_count_t1 = True
# placement coordinates
coordinate = {
    'red':   (-15 + 1, -7 - 0.5, 1.5),
    'green': (-15 + 1, -7 - 0.5, 1.5),
    'blue':  (-15 + 1, -7 - 0.5, 1.5),
}
z_r = coordinate['red'][2]
z_g = coordinate['green'][2]
z_b = coordinate['blue'][2]
z = z_r
def reset(): 
    global _stop
    global count
    global get_roi
    global color_list
    global move_square
    global detect_color
    global start_pick_up
    global start_count_t1
    global z_r, z_g, z_b, z
    
    count = 0
    _stop = False
    color_list = []
    get_roi = False
    move_square = False
    detect_color = 'None'
    start_pick_up = False
    start_count_t1 = True
    z_r = coordinate['red'][2]
    z_g = coordinate['green'][2]
    z_b = coordinate['blue'][2]
    z = z_r

def init():
    print("ColorPalletizing Init")
    initMove()

def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorPalletizing Start")

def stop():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorPalletizing Stop")

def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorPalletizing Exit")

rect = None
size = (640, 480)
rotation_angle = 0
unreachable = False
world_X, world_Y = 0, 0
def move():
    global rect
    global _stop
    global get_roi
    global move_square
    global __isRunning
    global unreachable
    global detect_color
    global start_pick_up
    global rotation_angle
    global world_X, world_Y
    global z_r, z_g, z_b, z
    
    dz = 2.5

    while True:
        if __isRunning:
            if detect_color != 'None' and start_pick_up:  # If it is detected that the block has not moved for a while, start the gripping
                set_rgb(detect_color)
                setBuzzer(0.1)
                # height accumulation
                z = z_r
                z_r += dz
                if z == 2 * dz + coordinate['red'][2]:
                    z_r = coordinate['red'][2]
                if z == coordinate['red'][2]:  
                    move_square = True
                    time.sleep(3)
                    move_square = False
                result = AK.setPitchRangeMoving((world_X, world_Y, 7), -90, -90, 0)  # Move to the target position, height 5cm
                if result == False:
                    unreachable = True
                else:
                    unreachable = False
                    time.sleep(result[2]/1000)

                    if not __isRunning:
                        continue
                    # Calculate the angle by which the gripper needs to be rotated
                    servo2_angle = getAngle(world_X, world_Y, rotation_angle)
                    Board.setBusServoPulse(1, servo1 - 280, 500)  # paws open
                    Board.setBusServoPulse(2, servo2_angle, 500)
                    time.sleep(0.5)

                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((world_X, world_Y, 2), -90, -90, 0, 1000)  # Lower the height to 2cm
                    time.sleep(1.5)

                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(1, servo1, 500)  # Gripper closed
                    time.sleep(0.8)

                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(2, 500, 500)
                    AK.setPitchRangeMoving((world_X, world_Y, 12), -90, -90, 0, 1000)  # The robotic arm is raised
                    time.sleep(1)

                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0, 1500) 
                    time.sleep(1.5)
                    
                    if not __isRunning:
                        continue                  
                    servo2_angle = getAngle(coordinate[detect_color][0], coordinate[detect_color][1], -90)
                    Board.setBusServoPulse(2, servo2_angle, 500)
                    time.sleep(0.5)

                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], z + 3), -90, -90, 0, 500)
                    time.sleep(0.5)
                    
                    if not __isRunning:
                        continue                
                    AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], z), -90, -90, 0, 1000)
                    time.sleep(0.8)

                    if not __isRunning:
                        continue 
                    Board.setBusServoPulse(1, servo1 - 200, 500)  # Claws open to drop objects
                    time.sleep(1)

                    if not __isRunning:
                        continue 
                    AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0, 800)
                    time.sleep(0.8)

                    initMove()  # return to original position
                    time.sleep(1.5)

                    detect_color = 'None'
                    get_roi = False
                    start_pick_up = False
                    set_rgb(detect_color)
        else:
            if _stop:
                _stop = False
                Board.setBusServoPulse(1, servo1 - 70, 300)
                time.sleep(0.5)
                Board.setBusServoPulse(2, 500, 500)
                AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                time.sleep(1.5)               
            time.sleep(0.01)

# run child thread
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

t1 = 0
roi = ()
center_list = []
last_x, last_y = 0, 0
draw_color = range_rgb["black"]
def run(img):
    global roi
    global rect
    global count
    global get_roi
    global move_square
    global center_list
    global unreachable
    global __isRunning
    global start_pick_up
    global last_x, last_y
    global rotation_angle
    global world_X, world_Y
    global start_count_t1, t1
    global detect_color, draw_color, color_list
 
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
    cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

    if not __isRunning:
        return img

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
    # If an area is detected with a recognized object, the area is detected until there are none
    if get_roi and not start_pick_up:
        get_roi = False
        frame_gb = getMaskROI(frame_gb, roi, size)    
    
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # Convert image to LAB space

    color_area_max = None
    max_area = 0
    areaMaxContour_max = 0
    
    if not start_pick_up:
        for i in color_range:
            if i in __target_color:
                frame_mask = cv2.inRange(frame_lab, color_range[i][0], color_range[i][1])  # Bitwise operations on the original image and mask
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # open operation
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # closed operation
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # find the outline
                areaMaxContour, area_max = getAreaMaxContour(contours)  # find the largest contour
                if areaMaxContour is not None:
                    if area_max > max_area:  # find the largest area
                        max_area = area_max
                        color_area_max = i
                        areaMaxContour_max = areaMaxContour
        if max_area > 2500:  # have found the largest area
            rect = cv2.minAreaRect(areaMaxContour_max)
            box = np.int0(cv2.boxPoints(rect))
            
            roi = getROI(box) # get roi region
            get_roi = True

            img_centerx, img_centery = getCenter(rect, roi, size, square_length)  # Get the coordinates of the center of the block
            
            world_x, world_y = convertCoordinate(img_centerx, img_centery, size) # Convert to real world coordinates

            if not start_pick_up:
                cv2.drawContours(img, [box], -1, range_rgb[color_area_max], 2)
                cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[color_area_max], 1) # draw center point
                distance = math.sqrt(pow(world_x - last_x, 2) + pow(world_y - last_y, 2)) # Compare the last coordinates to determine whether to move
            last_x, last_y = world_x, world_y

            if not start_pick_up:
                if color_area_max == 'red':  # red max
                    color = 1
                elif color_area_max == 'green':  # green max
                    color = 2
                elif color_area_max == 'blue':  # blue max
                    color = 3
                else:
                    color = 0
                color_list.append(color)
                # cumulative judgment
                if distance < 0.5:
                    count += 1
                    center_list.extend((world_x, world_y))
                    if start_count_t1:
                        start_count_t1 = False
                        t1 = time.time()
                    if time.time() - t1 > 0.5:
                        rotation_angle = rect[2]
                        start_count_t1 = True
                        world_X, world_Y = np.mean(np.array(center_list).reshape(count, 2), axis=0)
                        center_list = []
                        count = 0
                        start_pick_up = True
                else:
                    t1 = time.time()
                    start_count_t1 = True
                    center_list = []
                    count = 0

                if len(color_list) == 3:  # multiple judgments
                    # take the average
                    color = int(round(np.mean(np.array(color_list))))
                    color_list = []
                    if color == 1:
                        detect_color = 'red'
                        draw_color = range_rgb["red"]
                    elif color == 2:
                        detect_color = 'green'
                        draw_color = range_rgb["green"]
                    elif color == 3:
                        detect_color = 'blue'
                        draw_color = range_rgb["blue"]
                    else:
                        detect_color = 'None'
                        draw_color = range_rgb["black"]
        else:
            if not start_pick_up:
                draw_color = (0, 0, 0)
                detect_color = "None"
    
    if move_square:
        cv2.putText(img, "Make sure no blocks in the stacking area", (15, int(img.shape[0]/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)    
    
    cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
    
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
