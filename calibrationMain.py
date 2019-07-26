import sys
from ui.calibrationUI import Ui_MainWindow
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

import pyrealsense2 as rs
import numpy as np
import cv2

#from robot.controller import Controller
#from robot.gripper import Gripper
import time

class StreamThread(QThread):
    img_trigger = pyqtSignal(object)
    def __init__(self):
        super(StreamThread,self).__init__()
        self._mutex = QMutex()
        self._running = True
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        profile = self.pipeline.start(self.config)
    def __del__(self):
        self.pipeline.stop()
        self.wait()
    def run(self):
        try:
            while self.running():
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    print("continue")
                    continue
                # Convert images to numpy arrays
                color_image = np.uint8(color_frame.get_data()) 
                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                self.color_image = cv2.cvtColor(color_image,cv2.COLOR_BGR2RGB)
                self.img_trigger.emit(self.color_image)
                cv2.waitKey(5)
        except NameError as e:
            print(e)
            self.pipeline.stop()
    def running(self):
        try:
            self._mutex.lock()
            return self._running
        finally:
            self._mutex.unlock()
    def saveImg(self, path):
        cv2.imwrite(path, self.color_image)

class ArmThread(QThread):
    # control signal: working signal of robotarm
    def __init__(self):
        super(ArmThread,self).__init__()
        self._mutex = QMutex()
        self.cntrl = Controller()
        self.grip = Gripper()
        self.trans_mat = np.array([
            [0.6219, -0.0021, 0.],
            [-0.0028, -0.6218, 0.],
            [-337.3547, -163.6015, 1.0]
        ])
        self.pose = ['2883', '-246016', '166040', '-1709973', '-1929', '-104740']
        self._running = True
    def initArmGripper(self):
        # to init arm position
        # init gripper
        self.grip.gripper_reset()
        self.cntrl.power_on()
    
    def run(self):
        while self.running():
            if self.pose == None:
                #print("don't move")
                continue
            else:
                self.move(self.pose)

    def getPose(self):
        pose = self.cntrl.get_robot_pos()
        print("pos=",pose)
        return pose

    def gripperOn(self):
        self.grip.gripper_on()
        time.sleep(0.3)
    def gripperOff(self):
        self.grip.gripper_off()
        time.sleep(0.5)

    def move(self, pos):
        self.cntrl.move_robot_pos(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], 2000)
        self.cntrl.wait_move_end()

    def goHome(self):
        self.cntrl.move_robot_pos('2883', '-246016', '166040', '-1709973', '-1929', '-104740', 2000)
        self.cntrl.wait_move_end()

    def X_move(self, plus_or_minus, scale):
        # plus = True
        # minus = False
        current_pos = self.cntrl.get_robot_pos()
        if plus_or_minus:
            scale = scale
        else:
            scale = scale*(-1)
        new_pos = [current_pos[0]+scale, current_pos[1], current_pos[2], current_pos[3], current_pos[4],current_pos[5]]
        self.pose = new_pos
    
    def Y_move(self, plus_or_minus, scale):
        # plus = True
        # minus = False
        current_pos = self.cntrl.get_robot_pos()
        if plus_or_minus:
            scale = scale
        else:
            scale = scale*(-1)
        new_pos = [current_pos[0], current_pos[1]+scale, current_pos[2], current_pos[3], current_pos[4],current_pos[5]]
        self.pose = new_pos
    
    def Z_move(self, plus_or_minus, scale):
        # plus = True
        # minus = False
        current_pos = self.cntrl.get_robot_pos()
        if plus_or_minus:
            scale = scale
        else:
            scale = scale*(-1)
        new_pos = [current_pos[0], current_pos[1], current_pos[2]+scale, current_pos[3], current_pos[4],current_pos[5]]
        self.pose = new_pos

    def Rx_move(self, plus_or_minus, scale):
        # plus = True
        # minus = False
        current_pos = self.cntrl.get_robot_pos()
        if plus_or_minus:
            scale = scale
        else:
            scale = scale*(-1)
        new_pos = [current_pos[0], current_pos[1], current_pos[2], current_pos[3]+scale, current_pos[4],current_pos[5]]
        self.pose = new_pos

    def Ry_move(self, plus_or_minus, scale):
        # plus = True
        # minus = False
        current_pos = self.cntrl.get_robot_pos()
        if plus_or_minus:
            scale = scale
        else:
            scale = scale*(-1)
        new_pos = [current_pos[0], current_pos[1], current_pos[2], current_pos[3], current_pos[4]+scale,current_pos[5]]
        self.pose = new_pos

    def Rz_move(self, plus_or_minus, scale):
        # plus = True
        # minus = False
        current_pos = self.cntrl.get_robot_pos()
        if plus_or_minus:
            scale = scale
        else:
            scale = scale*(-1)
        new_pos = [current_pos[0], current_pos[1], current_pos[2], current_pos[3], current_pos[4],current_pos[5]+scale]
        self.pose = new_pos

    def running(self):
        try:
            self._mutex.lock()
            return self._running
        finally:
            self._mutex.unlock()


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        #self._arm = ArmThread()
        self._stream_thread = StreamThread()
        self._stream_thread.start()
        self._stream_thread.img_trigger.connect(self.updateRGBFrame)

        shift_scale = 10
        self.x_plus.clicked.connect(lambda: self.moveArm("x+", shift_scale))
        self.x_minus.clicked.connect(lambda: self.moveArm("x-", shift_scale))
        self.y_plus.clicked.connect(lambda: self.moveArm("y+", shift_scale))
        self.y_minus.clicked.connect(lambda: self.moveArm("y-", shift_scale))
        self.z_plus.clicked.connect(lambda: self.moveArm("z+", shift_scale))
        self.z_minus.clicked.connect(lambda: self.moveArm("z-", shift_scale))
        rotate_scale = 2
        self.rx_plus.clicked.connect(lambda: self.moveArm("rx+", rotate_scale))
        self.rx_minus.clicked.connect(lambda: self.moveArm("rx-", rotate_scale))
        self.ry_plus.clicked.connect(lambda: self.moveArm("ry+", rotate_scale))
        self.ry_minus.clicked.connect(lambda: self.moveArm("ry-", rotate_scale))
        self.rz_plus.clicked.connect(lambda: self.moveArm("rz+", rotate_scale))
        self.rz_minus.clicked.connect(lambda: self.moveArm("rz-", rotate_scale))

        self.gripper_on.clicked.connect(lambda: self.grip_ctrl(True))
        self.gripper_off.clicked.connect(lambda: self.grip_ctrl(False))

        self.save_btn.clicked.connect(self.save)

    def moveArm(self, arg, scale):
        if arg == "x+":
            #self._arm.X_move(True, scale)
            print("x+")
        elif arg == "x-":
            #self._arm.X_move(False, scale)
            print("x-")
        elif arg == "y+":
            #self._arm.Y_move(True, scale)
            print("y+")
        elif arg == "y-":
            #self._arm.Y_move(False, scale)
            print("y-")
        elif arg == "z+":
            #self._arm.Z_move(True, scale)
            print("z+")
        elif arg == "z-":
            #self._arm.Z_move(False, scale)
            print("z-")
        elif arg == "rx+":
            #self._arm.Rx_move(True, scale)
            print("rx+")
        elif arg == "rx-":
            #self._arm.Rx_move(False, scale)
            print("rx-")
        elif arg == "ry+":
            #self._arm.Ry_move(True, scale)
            print("ry+")
        elif arg == "ry-":
            #self._arm.Ry_move(False, scale)
            print("ry-")
        elif arg == "rz+":
            #self._arm.Rz_move(True, scale)
            print("rz+")
        elif arg == "rz-":
            #self._arm.Rz_move(False, scale)
            print("rz-")
        else:
            pass
    
    def grip_ctrl(self, on_off):
        if on_off:
            #self._arm.gripperOn()
            print("gripper on")
        else:
            #self._arm.gripperOff()
            print("gripper off")
    
    
    def save(self):
        # TODO: set save path with date and num
        #pose = self._arm.getPose()
        im = self._stream_thread.saveImg()
        print("save")

    def updateRGBFrame(self, rgb_image):        
        self._rgb_image = QImage(rgb_image[:], rgb_image.shape[1], rgb_image.shape[0], rgb_image.shape[1] * 3, QImage.Format_RGB888)        
        self.image_window.setPixmap(QPixmap.fromImage(self._rgb_image))
        QApplication.processEvents()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())