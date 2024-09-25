
from typing import Any
import cv2
import keyboard
import math


class ImageCtrl:
    def __init__(self, screenWidth=640, screenHeight=480) -> None:
        self.image = None
        self.centX = int(screenWidth/2)
        self.centY = int(screenHeight/2)
        #   十字星信息
        self.starMargin = 4  # 中心点间隔
        self.starWidth = 6  # 十字星的宽度
        self.lineWidth = 1  # 线的宽度
        self.lineColor = (0, 0, 255)
        self.valueColor = (0, 255, 255)
        #   十字星-矩形框信息
        self.rectWidthMargin = 60  # 宽度间隔
        self.rectHeightMargin = 40  # 高度间隔
        self.rectWidth = 20  # 矩形线条-宽
        self.rectHeight = 20  # 矩形线条-高
        #   坐标轴
        self.CoordStepWidth = 5  # 坐标轴单位宽度
        self.CoordTopMargin = 35  # top margin
        self.CoordButtomMargin = 40  # buttom margin
        self.CoordLeftMargin = 20  # left margin
        self.CoordRightMargin = 40  # right margin
        self.calibrationLength = 10  # 刻度的长度

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
    #   绘制十字星外围矩形框信息

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
    #   当前姿态信息

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

    #   绘制坐标轴
    def drawCoordinate(self, CoordName, orientation, value, step=1, largeStep=4, minNum=0, maxNum=100, stepLength=2,):
        """
        @description  :
        @param  :
            CoordName:  坐标轴名称
            screenWidth：  窗口宽度
            screenHeight：  窗口高度
            orientation：   方位
            value：值
            step：最小刻度
            largeStep：大刻度
            minNum：最小值
            maxNum：最大值
            stepLength：最小刻度长度
        @Returns  :
        """
        stepNum = int(abs(maxNum-minNum)/step)  # 刻度数
        CoordinateLength = stepNum*stepLength  # 坐标轴总长度
        CoordNamePoint = [0, 0]  # 轴名称坐标
        CoordOriginPoint = [0, 0, 0, 0]  # 原点-终点坐标
        valuePoint = [0, 0, 0, 0]  # 值的起点和终点
        textPoint = [0, 0]
        valueLength = int(abs(value-minNum)/step*stepLength)  # 值的长度

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
        该类是通过键盘控制吊舱相机运动
            上(↑)下(↓)键控制俯仰角(pitch)；
            左(←)右(→)键控制偏航角(yaw);
            右Ctrl建 + 左(←)右(→) 控制横滚角(roll)
            焦距调节 alt+上, alt+下
    '''

    def __init__(self):
        self.AngEular = [0, 0, 0]
        # self.offset = math.pi / 180  # 每次偏移量1个degrees
        self.offset = 1  # 每次偏移量1个degrees
        self.angle_scalar = 1  # 最后每次增加角度为 angle_scalar * offset
        self.FOV_offset = 0.01
        self.FOV_scalar = 0  # 这个值应该再（-1，1）之间
        self.mode = 1  # 0:表示松开键盘保持模式,1 表示松开键盘，恢复模式
        self.press_up = keyboard.KeyboardEvent('down', 0, 'up')  # ↑被按下
        self.press_down = keyboard.KeyboardEvent('down', 1, 'down')  # ↓被按下
        self.press_right = keyboard.KeyboardEvent('down', 2, 'right')  # →被按下
        self.press_left = keyboard.KeyboardEvent('down', 3, 'left')  # 左被按下
        self.press_r_ctrl = keyboard.KeyboardEvent(
            'down', 4, 'right ctrl')  # 右ctrl被按下
        self.press_r_alt = keyboard.KeyboardEvent(
            'down', 5, 'right alt')  # 左alt被按下
        self.is_press_ctrl = False
        self.is_press_alt = False
        self.count = 0  # 因为读键盘的频率太快，所以需要做些处理
        self.CameraFOV = 90  # 焦距  单位:度
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

    def altUpPress(self):  # 需要根据需求来确定这个值
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
        if(key_v.event_type == 'down'):  # 按下键
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

        elif (key_v.event_type == 'up'):  # 松开键
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
