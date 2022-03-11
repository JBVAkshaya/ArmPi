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
import HiwonderSDK.ASR as ASR
import HiwonderSDK.TTS as TTS
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()

try:
    my_asr = ASR.ASR()
    my_tts = TTS.TTS()
    my_asr.eraseWords()  #擦除
    my_asr.setMode(2)  #设置为口令模式
    my_asr.addWords(1, 'kai shi') #开始
    my_asr.addWords(2, 'fen jian hong se')  #分拣红色
    my_asr.addWords(3, 'fen jian lv se')  #分拣绿色
    my_asr.addWords(4, 'fen jian lan se')  #分拣蓝色
    my_asr.addWords(5, 'ting zhi fen jian')  #停止分拣
except:
    print('没有接传感器')

range_rgb = {
    'red':   (0, 0, 255),
    'blue':  (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

__target_color = ('red',)
def setTargetdetected_color(target_color):
    global __target_color

    print("tartget_color", target_color)
    __target_color = target_color
    return (True, ())

#找出面积最大的轮廓
#参数为要比较的轮廓的列表
def getAreaMaxContour(contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours : #历遍所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  #计算轮廓面积
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  #只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰
                    area_max_contour = c

        return area_max_contour, contour_area_max  #返回最大的轮廓

# 夹持器夹取时闭合的角度
servo1 = 500

# 初始位置
def initMove():
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

#设置扩展板的RGB灯颜色使其跟要追踪的颜色一致
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
get_roi = False
__isRunning = False
unreachable = False
detect_color = 'None'
start_pick_up = False
start_count_t1 = True
start_count_t2 = True
start_count_t3 = True
def reset():
    global _stop
    global count
    global get_roi
    global unreachable
    global start_pick_up
    global start_count_t1
    global start_count_t2
    global start_count_t3
    global __target_color, detect_color

    count = 0  
    _stop = False
    get_roi = False
    __isRunning = False
    unreachable = False
    detect_color = 'None'
    start_pick_up = False
    start_count_t1 = True
    start_count_t2 = True
    start_count_t3 = True
    __target_color = ()
    
def init():
    print("ASRControl Init")
    initMove()

def start():
    global __isRunning
    reset()
    my_tts.TTSModuleSpeak("[h0][v10][m53]", "开始语音控制")
    print("ASRControl Start")
    time.sleep(0.5)
    my_asr.setMode(3)
    my_asr.setMode(2)
    my_asr.getResult()
    __isRunning = True   

def stop():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    my_tts.TTSModuleSpeak("[h0][v10][m53]", "停止语音控制")
    print("ASRControl Stop")

def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("ASRControl Exit")

rect = None
size = (640, 480)
rotation_angle = 0
unreachable = False
world_X, world_Y = 0, 0
def move():   
    global rect
    global _stop
    global get_roi
    global __isRunning
    global unreachable
    global detect_color
    global start_pick_up  
    global rotation_angle
    global start_count_t1
    global start_count_t2
    global start_count_t3
    global world_X, world_Y
    
    #放置坐标
    coordinate = {
        'red':   (-15 + 0.5, 12 - 0.5, 1.5),
        'green': (-15 + 0.5, 6 - 0.5,  1.5),
        'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
    }
    while True:
        if __isRunning:        
            if detect_color != 'None' and start_pick_up:  #如果检测到方块没有移动一段时间后，开始夹取
                set_rgb(detect_color)
                result = AK.setPitchRangeMoving((world_X, world_Y, 7), -90, -90, 0)  #移到目标位置，高度7cm
                if result == False:
                    unreachable = True
                else:
                    unreachable = False
                    time.sleep(result[2]/1000)

                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(1, servo1 - 280, 500)  # 爪子张开
                    #计算夹持器需要旋转的角度
                    servo2_angle = getAngle(world_X, world_Y, rotation_angle)
                    Board.setBusServoPulse(2, servo2_angle, 500)
                    time.sleep(0.5) 

                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((world_X, world_Y, 2), -90, -90, 0, 1000)  #降低高度到2cm
                    time.sleep(1.5)

                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(1, servo1, 500)  #夹持器闭合
                    time.sleep(1)

                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(2, 500, 500)
                    AK.setPitchRangeMoving((world_X, world_Y, 12), -90, -90, 0, 1000)  #机械臂抬起
                    time.sleep(1)

                    if not __isRunning:
                        continue
                    #对不同颜色方块进行分类放置
                    result = AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0)   
                    time.sleep(result[2]/1000)
                    Board.setBusServoPulse(2, 500, 500)
                    
                    if not __isRunning:
                        continue                   
                    servo2_angle = getAngle(coordinate[detect_color][0], coordinate[detect_color][1], -90)
                    Board.setBusServoPulse(2, servo2_angle, 500)
                    time.sleep(0.5)
                    
                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], coordinate[detect_color][2] + 3), -90, -90, 0, 500)
                    time.sleep(0.5)
                    
                    if not __isRunning:
                        continue                    
                    AK.setPitchRangeMoving((coordinate[detect_color]), -90, -90, 0, 1000)
                    time.sleep(0.8)

                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(1, servo1 - 200, 500)  # 爪子张开  ，放下物体
                    time.sleep(0.8)

                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0, 800)
                    time.sleep(0.8)
                    
                    initMove()  #回到初始位置
                    time.sleep(1.5)
                    my_tts.TTSModuleSpeak("[h0][v10][m53]", "搬运完成")
                    time.sleep(1) 
                    get_roi = False    
                    detect_color = 'None'
                    start_count_t1 = True
                    start_count_t2 = True
                    start_count_t3 = True
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
          
#运行子线程
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()    

roi = ()
center_list = []
t1 = t2 = t3 = 0
last_x, last_y = 0, 0
def run(img):
    global roi
    global rect
    global count
    global _stop
    global __stop
    global get_roi
    global t1, t2, t3
    global center_list
    global unreachable
    global __isRunning
    global start_pick_up   
    global rotation_angle
    global last_x, last_y
    global world_X, world_Y 
    global __target_color, detect_color  
    global start_count_t1, start_count_t2, start_count_t3

    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
    cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

    if not __isRunning:
        return img

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
    #如果检测到某个区域有识别到的物体，则一直检测该区域直到没有为止
    if get_roi and start_pick_up:
        get_roi = False
        frame_gb = getMaskROI(frame_gb, roi, size)    
    
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  #将图像转换到LAB空间
    data = my_asr.getResult()
    if data == 2:
        my_tts.TTSModuleSpeak("[h0][v10][m53]", "好的")
        start_count_t2 = True
        __target_color = ('red',)
    elif data == 3:
        my_tts.TTSModuleSpeak("[h0][v10][m53]", "ok")
        __target_color = ('green',)
        start_count_t2 = True
    elif data == 4:
        my_tts.TTSModuleSpeak("[h0][v10][m53]", "收到")
        __target_color = ('blue',)
        start_count_t2 = True
    elif data == 5:
        my_tts.TTSModuleSpeak("[h0][v10][m53]", "好的")
        __target_color = ()

    if not start_pick_up:
        if __target_color != ():
            detect_color = __target_color[0]
            frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0], color_range[detect_color][1])  #对原图像和掩模进行位运算
            opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # 开运算
            closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # 闭运算
            contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
            areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓
            if area_max > 2500:  # 有找到最大面积
                rect = cv2.minAreaRect(areaMaxContour)
                box = np.int0(cv2.boxPoints(rect))
                
                roi = getROI(box) #获取roi区域
                get_roi = True

                img_centerx, img_centery = getCenter(rect, roi, size, square_length)  # 获取木块中心坐标
                world_x, world_y = convertCoordinate(img_centerx, img_centery, size) #转换为现实世界坐标
                if not start_pick_up: 
                    cv2.drawContours(img, [box], -1, range_rgb[detect_color], 2)
                    cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[detect_color], 1) #绘制中心点
                distance = math.sqrt(pow(world_x - last_x, 2) + pow(world_y - last_y, 2)) #对比上次坐标来判断是否移动
                last_x, last_y = world_x, world_y
                # 累计判断
                if not start_pick_up:
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
                            if detect_color == 'red':
                                my_tts.TTSModuleSpeak("[h0][v10][m53]", "找到红色")
                            elif detect_color == 'green':
                                my_tts.TTSModuleSpeak("[h0][v10][m53]", "找到绿色")
                            elif detect_color == 'blue':
                                my_tts.TTSModuleSpeak("[h0][v10][m53]", "找到蓝色")
                            time.sleep(1)
                            my_tts.TTSModuleSpeak("[h0][v10][m53]", "开始搬运")
                            time.sleep(1.5)
                            start_pick_up = True
                    else:
                        start_count_t1 = True
                        center_list = []
                        t1 = time.time()
                        count = 0
            else:
                if not start_pick_up:
                    if start_count_t2:
                        start_count_t2 = False
                        t2 = time.time()
                    else:
                        if 7 > time.time() - t2 >= 5:
                            if detect_color == 'red':
                                my_tts.TTSModuleSpeak("[h0][v10][m53]", "没有找到红色")
                            elif detect_color == 'green':
                                my_tts.TTSModuleSpeak("[h0][v10][m53]", "没有找到绿色")
                            elif detect_color == 'blue':
                                my_tts.TTSModuleSpeak("[h0][v10][m53]", "没有找到蓝色")
                            time.sleep(2)

    if start_pick_up and unreachable:
        if start_count_t3:
            start_count_t3 = False
            t3 = time.time()
        else:
            if time.time() - t3 < 2:
                cv2.putText(img, 'Unreachable!', (10, img.shape[:2][0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255))
            else:
                start_count_t3 = True
    else:
        start_count_t3 = True

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
