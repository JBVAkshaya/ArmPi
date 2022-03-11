#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append("/home/pi/ArmPi")
import cv2
import Camera
import threading
from time import sleep
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
from CameraCalibration.CalibrationConfig import *
import HiwonderSDK.Board as Board
import logging
import time
STANDARD_COORDS = {  # colors of the blocks that can be perceived
    "red": (-14.5, 11.5, 1.5),
    "green": (-14.5, 5.5, 1.5),
    "blue": (-14.5, -0.5, 1.5),
}


DEBUG = False

class StopError(Exception):
    """
    Dummy custom error to mark when arm must stop
    """

    pass

def get_mask(frame):
    kernel = np.ones((5, 5), np.uint8)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
    # defining the lower and upper values of HSV,
    # this will detect blue colour
    Lower_hsv = np.array([60, 40, 40])
    Upper_hsv = np.array([125, 255, 255])
    
    # creating the mask by eroding,morphing,
    # dilating process
    Mask = cv2.inRange(hsv, Lower_hsv, Upper_hsv)
    # gray = cv2.cvtColor(Frame, cv2.COLOR_BGR2GRAY)
    # (thresh, blackAndWhiteImage) = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    # Mask = cv2.bitwise_not(Mask)
    Mask = cv2.erode(Mask, kernel, iterations=1) 
    Mask = cv2.dilate(Mask, kernel, iterations=4)
    return Mask
    
def no_motion(my_camera):
    threshold = 6000
    # while threshold == 20:
    img = my_camera.frame
    status = False
    # print("img: ", img)
    if img is not None:
        # print("in if")
        f = img.copy()
        frame_i = get_mask(f) 
        fps = 16
        for i in range(0, fps*2):
            img = my_camera.frame
            if img is not None:
                f = img.copy()
                frame_new = get_mask(f)
                frame_final = (frame_new/255)-(frame_i/255)
                v = np.sum(np.abs(frame_final))
                # print("value:", v)

                if v < threshold:
                    status = True
                    logging.debug("current status: ", status)
                    
                else: 
                    status = False
                    logging.debug("current status: ", status)
                cv2.imshow('Frame', frame_new)
    print("status", status)            
    return status

class Motion(object):
    
    def __init__(self,stop_event, coords=STANDARD_COORDS):
        self.delay = 0.5
        self.coords = STANDARD_COORDS
        self.stop_event = stop_event
        self.AK = ArmIK()
        self.x, self.y, self.z = 0, 0, 0
        self._init_move()


    def __del__(self):
        self.stop_event.set()

    def check_stop(f):
        """
        Function wrapper that checks if the calling object
        (an instance of this class)'s stop_event is set,
        raising a StopError if so.
        This means we don't have to write "if not running"
        a million times, just decorate each func.
        AND, it really isn't necessary to call this on higher-level
        functions that only serve to call lower level ones,
        thus I only decorate the most commonly-used ones.
        """

        def wrapper(*args):
            # First arg is the calling object
            obj = args[0]

            # Ensure called on currect type of object
            assert isinstance(obj, Motion)

            if obj.stop_event.is_set():
                logging.debug(f"Big no no. Stop activated!!")
                raise StopError("The stop flag has been raised!")
            else:
                if DEBUG:
                    print(f, args)
                    input()
                return f(*args)

        return wrapper
    

    def move_arm(self, x, y, z, delay=None):
        
        self.x, self.y, self.z = x, y, z  # Current position

        # pitch to about ~ -90
        ret = self.AK.setPitchRangeMoving((x, y, z), -90, -90, 0, 1500)
        ret = ret is not False  # convert from *stuff/False to True/False

        sleep(self.delay if delay is None else delay)
        return ret
    
    def set_gripper(self, val_str, delay=None):
        #openieng and closing the gripper 
        
        assert isinstance(val_str, str)
        if val_str.lower() in ("open", "opened"):
            Board.setBusServoPulse(1, 300, 500)
        elif val_str.lower() in ("close", "closed"):
            Board.setBusServoPulse(1, 500, 500)
        else:
            raise ValueError

        sleep(self.delay if delay is None else delay)
    
    def set_wrist(self, x, y, theta, delay=None):
        #wrist angle calculations

        servo2_angle = getAngle(x, y, theta)
        Board.setBusServoPulse(2, servo2_angle, 500)

        sleep(self.delay if delay is None else delay)

    def _init_move(self):
        self.set_gripper("open")
        self.set_wrist(0, 10, 0)
        return self.move_arm(0, 10, 12)

    def raise_arm(self, z=12):
        return self.move_arm(self.x, self.y, z)

    def lower_arm(self, z=2):
        return self.move_arm(self.x, self.y, z)

    def grab_box(self, x, y, theta):
        """
        Move to box at x, y with 2D orientation theta,
        then pick up
        """
        self.raise_arm()
        self.set_gripper("open")
        self.set_wrist(x, y, theta)
        self.move_arm(x, y, self.z)
        self.lower_arm()
        self.set_gripper("closed")
        self.raise_arm()

    def place_box(self, color):
        """
        Assumes box is already in hand. Color is string "red",
        "blue", or "green". Place at corresponding coordinates.
        """
        x, y, z = self.coords[color]
        theta = -90

        self.set_wrist(x, y, theta)
        self.move_arm(x, y, self.z)
        self.lower_arm(z + 3)  # move just above target
        self.lower_arm(z)  # then place
        self.set_gripper("open")
        self.raise_arm()

    def stop_override(self):
        """
        To be called after stop event is set, forcefully moves
        arm to a reset position without checking event
        """
        # Open gripper, reset wrist
        Board.setBusServoPulse(1, 300, 500)
        Board.setBusServoPulse(2, 500, 500)

        # Lift up
        self.AK.setPitchRangeMoving((self.x, self.y, self.x), -90, -90, 0)
    
if __name__ == "__main__":
    stop_event = threading.Event()
    mover = Motion(stop_event)

    # # Dummy coord to test with
    # try:
    #     # mover.move_arm(0, 12, 20)
    #     mover.move_arm(20, 12, 20)
    #     mover.move_arm(20, 12, 12)
    #     mover.move_arm(-20, 12, 12)
    #     mover.move_arm(-20, 12, 20)
    #     mover.move_arm(0, 12, 20)
    # except:
    #     print("does not work")

    my_camera = Camera.Camera()
    my_camera.camera_open()
    # i = 0
    t1 = time.time()
    while True:
        if time.time()-t1 <20:
            status = no_motion(my_camera)
            # print(str(i), my_camera.frame)
            # img = my_camera.frame
            # i = i + 1
            #hand will begin to move
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            status = no_motion(my_camera)
            if status == True:
                mover.move_arm(20, 12, 20)
                mover.move_arm(20, 12, 12)
                mover.move_arm(-20, 12, 12)
                mover.move_arm(-20, 12, 20)
                mover.move_arm(0, 12, 20)

    my_camera.camera_close()
    cv2.destroyAllWindows()


    # try:
    #     mover.grab_box(10, 10, -45)
    #     mover.place_box("red")
    #     mover.grab_box(10, -10, -30)
    #     stop_event.set()
    #     mover.place_box("blue")
    # except KeyboardInterrupt:
    #     logging.debug("Ending by user.")
    # except StopError:
    #     logging.debug(f"Big no no. Stop activated!!")