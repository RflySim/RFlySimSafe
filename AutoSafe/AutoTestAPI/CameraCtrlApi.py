
from typing import Any
import cv2
import keyboard
import math


class ImageCtrl:
    def __init__(self, screenWidth=640, screenHeight=480) -> None:
        self.image = None
        self.centX = int(screenWidth/2)
        self.centY = int(screenHeight/2)

        self.starMargin = 4  
        self.starWidth = 6 
        self.lineWidth = 1  
        self.lineColor = (0, 0, 255)
        self.valueColor = (0, 255, 255)
     
        self.rectWidthMargin = 60  
        self.rectHeightMargin = 40  
        self.rectWidth = 20  
        self.rectHeight = 20  
       
        self.CoordStepWidth = 5  
        self.CoordTopMargin = 35  
        self.CoordButtomMargin = 40  
        self.CoordLeftMargin = 20  
        self.CoordRightMargin = 40  
        self.calibrationLength = 10  

    def setRect(self, window_w, window_h):
        self.centX = int(window_w/2)
        self.centY = int(window_h/2)

    def DisplayImg(self, img):
        self.image = img

    def drawCross(self):
        cv2.line(self.image, (self.centX, self.centY-self.starMargin),
                 (self.centX, self.centY-self.starMargin), self.lineColor, self.lineWidth)
        cv2.line(self.image, (self.centX, self.centY+self.starMargin),
                 (self.centX, self.centY+self.starMargin+self.starWidth), self.lineColor, self.lineWidth)
        cv2.line(self.image, (self.centX-self.starMargin, self.centY),
                 (self.centX-self.starMargin, self.centY), self.lineColor, self.lineWidth)
        cv2.line(self.image, (self.centX+self.starMargin, self.centY),
                 (self.centX+self.starMargin, self.centY), self.lineColor, self.lineWidth)
    

    def drawCrossRect(self):
        cv2.line(self.image, (self.centX-self.rectWidthMargin, self.centY-self.rectHeightMargin),
                 (self.centX-self.rectWidthMargin+self.rectWidth, self.centY-self.rectHeightMargin), self.lineColor, self.lineWidth)
        cv2.line(self.image, (self.centX-self.rectWidthMargin, self.centY-self.rectHeightMargin),
                 (self.centX-self.rectWidthMargin, self.centY-self.rectHeightMargin+self.rectHeight), self.lineColor, self.lineWidth)

        cv2.line(self.image, (self.centX-self.rectWidthMargin, self.centY+self.rectHeightMargin),
                 (self.centX-self.rectWidthMargin+self.rectWidth, self.centY+self.rectHeightMargin), self.lineColor, self.lineWidth)
        cv2.line(self.image, (self.centX-self.rectWidthMargin, self.centY+self.rectHeightMargin),
                 (self.centX-self.rectWidthMargin, self.centY+self.rectHeightMargin-self.rectHeight), self.lineColor, self.lineWidth)

        cv2.line(self.image, (self.centX+self.rectWidthMargin, self.centY-self.rectHeightMargin),
                 (self.centX+self.rectWidthMargin-self.rectWidth, self.centY-self.rectHeightMargin), self.lineColor, self.lineWidth)
        cv2.line(self.image, (self.centX+self.rectWidthMargin, self.centY-self.rectHeightMargin),
                 (self.centX+self.rectWidthMargin, self.centY-self.rectHeightMargin+self.rectHeight), self.lineColor, self.lineWidth)

        cv2.line(self.image, (self.centX+self.rectWidthMargin, self.centY+self.rectHeightMargin),
                 (self.centX+self.rectWidthMargin-self.rectWidth, self.centY+self.rectHeightMargin), self.lineColor, self.lineWidth)
        cv2.line(self.image, (self.centX+self.rectWidthMargin, self.centY+self.rectHeightMargin),
                 (self.centX+self.rectWidthMargin, self.centY+self.rectHeightMargin-self.rectHeight), self.lineColor, self.lineWidth)
    

    def drawPosture(self, roll, pitch, yaw):
        startPoint = [self.CoordLeftMargin, self.CoordTopMargin]
        rollStr = 'Roll(rad)  :' + str(round(roll, 6))
        pitchStr = 'Pitch(rad):' + str(round(pitch, 6))
        yawStr = 'Yaw(rad) :' + str(round(yaw, 6))
        cv2.putText(self.image, rollStr,
                    (startPoint[0], startPoint[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.valueColor)
        cv2.putText(self.image, pitchStr,
                    (startPoint[0], startPoint[1]+20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.valueColor)
        cv2.putText(self.image, yawStr,
                    (startPoint[0], startPoint[1]+40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.valueColor)

    
    def drawCoordinate(self, CoordName, orientation, value, step=1, largeStep=4, minNum=0, maxNum=100, stepLength=2,):
        """
        @description  :
        @param  :
            CoordName: Coordinate axis name
             screenWidth: window width
             screenHeight: window height
             orientation: orientation
             value: value
             step: minimum scale
             largeStep: large scale
             minNum: minimum value
             maxNum: maximum value
             stepLength: minimum scale length
        @Returns  :
        """
        stepNum = int(abs(maxNum-minNum)/step)  
        CoordinateLength = stepNum*stepLength  
        CoordNamePoint = [0, 0]  
        CoordOriginPoint = [0, 0, 0, 0]  
        valuePoint = [0, 0, 0, 0]  
        textPoint = [0, 0]
        valueLength = int(abs(value-minNum)/step*stepLength)  

        if orientation == 'top':
            CoordNamePoint = [
                self.centX-int(CoordinateLength/2)-len(CoordName)*10, self.CoordTopMargin]
            CoordOriginPoint = [
                self.centX-int(CoordinateLength/2), self.CoordTopMargin, self.centX+int(CoordinateLength/2), self.CoordTopMargin]
            valuePoint = [self.centX-int(CoordinateLength/2)+valueLength, self.CoordTopMargin, self.centX-int(
                CoordinateLength/2)+valueLength, self.CoordTopMargin-self.calibrationLength-5]
            textPoint = [
                self.centX-int(CoordinateLength/2)+valueLength, self.CoordTopMargin-self.calibrationLength]

            for i in range(stepNum+1):

                addLenth = i*stepLength
                if (i == 0 or (i != 0 and (i % largeStep) == 0)):
                    cv2.line(self.image, (CoordOriginPoint[0]+addLenth, self.CoordTopMargin), (CoordOriginPoint[0] +
                             addLenth, self.CoordTopMargin-self.calibrationLength-4), self.lineColor, self.lineWidth)  # largeStep
                else:
                    cv2.line(self.image, (CoordOriginPoint[0]+addLenth, self.CoordTopMargin), (CoordOriginPoint[0] +
                             addLenth, self.CoordTopMargin-self.calibrationLength), self.lineColor, self.lineWidth)  # calibration
        elif orientation == 'left':
            CoordNamePoint = [self.CoordLeftMargin,
                              self.centY-int(CoordinateLength/2)-5]
            CoordOriginPoint = [self.CoordLeftMargin, self.centY-int(
                CoordinateLength/2), self.CoordLeftMargin, self.centY+int(CoordinateLength/2)]
            valuePoint = [self.CoordLeftMargin, self.centY-int(CoordinateLength/2)+valueLength,
                          self.CoordLeftMargin+self.calibrationLength+5, self.centY-int(CoordinateLength/2)+valueLength]
            textPoint = [self.CoordLeftMargin+self.calibrationLength+10,
                         self.centY-int(CoordinateLength/2)+valueLength]

            for i in range(stepNum+1):
                addLenth = i*stepLength
                if (i == 0 or (i != 0 and (i % largeStep) == 0)):
                    cv2.line(self.image, (CoordOriginPoint[0], CoordOriginPoint[1]+addLenth), (CoordOriginPoint[0] +
                             self.calibrationLength+4, CoordOriginPoint[1]+addLenth), self.lineColor, self.lineWidth)  # largeStep
                else:
                    cv2.line(self.image, (CoordOriginPoint[0], CoordOriginPoint[1]+addLenth), (CoordOriginPoint[0] +
                             self.calibrationLength+4, CoordOriginPoint[1]+addLenth), self.lineColor, self.lineWidth)  # calibration

        elif orientation == 'buttom':
            value
        elif orientation == 'right':
            value
        else:
            return
        cv2.putText(self.image, CoordName, (CoordNamePoint[0], CoordNamePoint[1]),
                    cv2.FONT_HERSHEY_COMPLEX, 0.4, self.valueColor)  # Coordinate name
        cv2.line(self.image, (CoordOriginPoint[0], CoordOriginPoint[1]), (
            CoordOriginPoint[2], CoordOriginPoint[3]), self.lineColor, self.lineWidth)  # Coordinate

        cv2.line(self.image, (valuePoint[0], valuePoint[1]), (
            valuePoint[2], valuePoint[3]), self.valueColor, self.lineWidth)  # value
        cv2.putText(self.image, str(
            value), (textPoint[0], textPoint[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.valueColor)  # value-text


class KeyCtrl:
    '''
        This class is to control the camera movement of the pod through the keyboard
             Up (↑) and down (↓) keys control the pitch angle (pitch);
             Left (←) and right (→) keys control the yaw angle (yaw);
             Right Ctrl button + left (←) right (→) control roll angle (roll)
             Focus adjustment alt+up, alt+down
    '''

    def __init__(self):
        self.AngEular = [0, 0, 0]
        self.offset = 1  
        self.angle_scalar = 1  
        self.FOV_offset = 0.01
        self.FOV_scalar = 0  
        self.mode = 1  
        self.press_up = keyboard.KeyboardEvent('down', 0, 'up')  
        self.press_down = keyboard.KeyboardEvent('down', 1, 'down')  
        self.press_right = keyboard.KeyboardEvent('down', 2, 'right')  
        self.press_left = keyboard.KeyboardEvent('down', 3, 'left')  
        self.press_r_ctrl = keyboard.KeyboardEvent(
            'down', 4, 'right ctrl')  
        self.press_r_alt = keyboard.KeyboardEvent(
            'down', 5, 'right alt')  
        self.is_press_ctrl = False
        self.is_press_alt = False
        self.count = 0  
        self.CameraFOV = 90  # Focal length Unit: degree
        self.initkey()

    def getYaw(self):
        return self.AngEular[2]

    def getPitch(self):
        return self.AngEular[1]

    def getRoll(self):
        return self.AngEular[0]

    def getCameraFOV(self):
        return self.CameraFOV

    def getRadiansAngEular(self):
        return [math.radians(self.AngEular[0]), math.radians(
            self.AngEular[1]), math.radians(self.AngEular[2])]

    def upPress(self):
        self.AngEular[1] += self.angle_scalar * self.offset
        if(self.AngEular[1] > 180):
            self.AngEular[1] -= 360

    def downPress(self):
        self.AngEular[1] -= self.angle_scalar * self.offset
        if(self.AngEular[1] < -180):
            self.AngEular[1] += 360

    def rightPress(self):
        self.AngEular[2] += self.angle_scalar * self.offset
        if(self.AngEular[2] > 180):
            self.AngEular[2] -= 360

    def leftPress(self):
        self.AngEular[2] -= self.angle_scalar * self.offset
        if(self.AngEular[2] < -180):
            self.AngEular[2] += 360

    def ctrlLeftPress(self):
        self.AngEular[0] += self.angle_scalar*self.offset
        if(self.AngEular[0] > 180):
            self.AngEular[0] -= 360

    def ctrlRightPress(self):
        self.AngEular[0] -= self.angle_scalar*self.offset
        if(self.AngEular[0] < -180):
            self.AngEular[0] += 360

    def altUpPress(self):  
        self.FOV_scalar -= self.FOV_offset
        if(self.FOV_scalar < -1):
            self.FOV_offset = -1
        self.CameraFOV = self.FOV_scalar * 90 + 90

    def altDownPress(self):
        self.FOV_scalar += self.FOV_offset
        if(self.FOV_scalar > 1):
            self.FOV_scalar = 1
        self.CameraFOV = self.FOV_scalar * 90 + 90

    def Reset(self):
        yaw = self.getYaw()
        roll = self.getRoll()
        pitch = self.getPitch()
        self.inner(yaw, 2)
        self.inner(pitch, 1)
        self.inner(roll, 0)

    def inner(self, angle, i):
        if(abs(angle / self.offset) < 1):
            self.AngEular[i] = 0
        elif(angle < 0):
            self.AngEular[i] += self.angle_scalar * self.offset
        else:
            self.AngEular[i] -= self.angle_scalar * self.offset

    def fun(angle):
        if angle < -math.pi:
            return angle+2*math.pi
        elif angle > math.pi:
            return angle - 2*math.pi

    def CheckAngleWid(self):
        '''每个角度范围(-pi,pi)'''
        self.AngEular[0] = self.fun(self.AngEular[0])
        self.AngEular[1] = self.fun(self.AngEular[1])
        self.AngEular[2] = self.fun(self.AngEular[2])

    def callback(self, key_v):
        if(key_v.event_type == 'down'):  
            if(key_v.name == self.press_r_ctrl.name):
                self.is_press_ctrl = True
            if(key_v.name == self.press_r_alt):
                self.is_press_alt = True

            self.count += 1
            if(self.count % 100 == 0):
                self.count = 0
            else:
                return
            if(key_v.name == self.press_up.name and not self.is_press_alt):
                self.upPress()
            if(key_v.name == self.press_down.name and not self.is_press_alt):
                self.downPress()
            if(key_v.name == self.press_left.name and not self.is_press_ctrl):
                self.leftPress()
            if(key_v.name == self.press_right.name and not self.is_press_alt):
                self.rightPress()
            if(key_v.name == self.press_up.name and self.is_press_alt):
                self.altUpPress()
            if(key_v.name == self.press_down.name and self.is_press_alt):
                self.altDownPress()

        elif (key_v.event_type == 'up'): 
            if(key_v.name == self.press_r_ctrl.name):
                self.is_press_ctrl = False
            if(key_v.name == self.press_r_alt):
                self.is_press_alt = False
        self.CheckAngleWid()

    def initkey(self):
        keyboard.add_hotkey('left', self.leftPress)
        keyboard.add_hotkey('right', self.rightPress)
        keyboard.add_hotkey('up', self.upPress)
        keyboard.add_hotkey('down', self.downPress)
        keyboard.add_hotkey('ctrl+left', self.ctrlLeftPress)
        keyboard.add_hotkey('ctrl+right', self.ctrlRightPress)
        keyboard.add_hotkey('alt+up', self.altUpPress)
        keyboard.add_hotkey('alt+down', self.altDownPress)
