import time
import re,sys,os
import cv2.cv2 as cv2
import numpy as np
import math
import random

try:
    import include.AutoREG as AutoREG
    import include.VisionCaptureApi as VisionCaptureApi
    import include.CameraCtrlApi as CameraCtrlApi
    import include.PX4MavCtrlV4 as PX4MavCtrl
    import include.UE4CtrlAPI as UE4CtrlAPI
except ImportError:
    import AutoREG 
    import UE4CtrlAPI
    import CameraCtrlApi 
    import VisionCaptureApi 
    import PX4MavCtrlV4 as PX4MavCtrl


class MavVIS:
    def __init__(self,conf): # ['Quadcopter', 'SITL', 1]
        self.conf = conf
        self.vis = VisionCaptureApi.VisionCaptureApi()
        self.ue = UE4CtrlAPI.UE4CtrlAPI()

        model_path = os.path.join(sys.path[0],'..','model',AutoREG.MAV_FRAME_DICT[AutoREG.RFLYSIM_FRAME[conf[0]]][0])
        vs_json = 'Config.json'
        vsp = [filename for filename in os.listdir(model_path) if vs_json in filename]
        self.vspath = os.path.join(model_path, vsp[0])

        self.ue.sendUE4Cmd(b'r.setres 1280x720w', 0)
        self.ue.sendUE4Cmd(b't.MaxFPS 30', 0)  

        self.vis.jsonLoad(-1,self.vspath)
        isSuss = self.vis.sendReqToUE4()

        if not isSuss:
            sys.exit(0)

        self.vis.startImgCap()

        self.LastSensorAngEular = [0., 0., 0.]  
        self.CoordName = 'posture'

    def visShow(self):
        for i in range(len(self.vis.hasData)):
            if self.vis.hasData[i]:
                # Process your image here
                cv2.imshow('Img'+str(i),self.vis.Img[i])
                cv2.waitKey(1)
                
            if i==0: # 更新0号相机的参数
                # 以下代码用于实时更新相机参数（位置、焦距、角度、装载飞机和形式）
                vs = self.vis.VisSensor[0] #获取第0号相机基本参数
                # 修改其中的可变部分，只修改需要改变的部分即可
                vs.TargetCopter=1  #修改视角绑定的飞机ID
                vs.TargetMountType=0  # 修改视角绑定类型，固连飞机还是地面
                vs.CameraFOV=90   # 修改视角的视场角（焦距），可以模拟对焦相机
                vs.SensorPosXYZ=[0.3,-0.15,0]  # 修改相机的位置，可以调整相机初始位置
                vs.SensorAngEular=[0,0,0]  # 修改相机的姿态，可以模拟云台转动
                self.vis.sendUpdateUEImage(vs) # 发送更新数据


    def gasuss_noise(self,image, mu, sigma):
        image = np.array(image / 255.0, dtype=float)
        noise = np.random.normal(mu, sigma, image.shape)
        gauss_noise = image + noise 
        if gauss_noise.min() < 0:
            low_clip = -1.
        else:
            low_clip = 0.
        gauss_noise = np.clip(gauss_noise, low_clip, 1.0)
        gauss_noise = np.uint8(gauss_noise * 255)
        
        return gauss_noise

    def Podfault(self,PodfalutId):
        self.key_ctrl = CameraCtrlApi.KeyCtrl()
        self.img_ctrl = CameraCtrlApi.ImageCtrl()
        time.sleep(2)
        SensorAngEular = self.key_ctrl.AngEular
        if PodfalutId == 123549:
            for y in range(len(self.vis.hasData)):
                self.vis.hasData[y] = False
        elif PodfalutId == 124350:
            SensorAngEular[0] = SensorAngEular[0] + random.randint(-10, 10)
            SensorAngEular[1] = SensorAngEular[1] + random.randint(-10, 10)
            SensorAngEular[2] = SensorAngEular[2] + random.randint(-10, 10)
        elif PodfalutId == 124351:
            for y in range(len(self.vis.hasData)):
                if self.vis.hasData[y]:
                    self.vis.Img[y] = self.gasuss_noise(self.vis.Img[y], 0.0, 0.5)

        for y in range(len(self.vis.hasData)):
            if self.vis.hasData[y]:
                # for j in range(self.mavNum):
                # Process your image here
                self.img_ctrl.DisplayImg(self.vis.Img[y])
                self.img_ctrl.setRect(
                    self.vis.VisSensor[y].DataWidth, self.vis.VisSensor[y].DataHeight)
                self.img_ctrl.drawCross()
                self.img_ctrl.drawCrossRect()
                self.img_ctrl.drawCoordinate(
                    'FOV(deg):', 'top', math.ceil(self.key_ctrl.getCameraFOV()), 15, 3, 0, 180, 10)
                self.img_ctrl.drawPosture(
                    SensorAngEular[0], SensorAngEular[1], SensorAngEular[2])

                if abs(self.LastSensorAngEular[0] - SensorAngEular[0]) > 0.00000001:
                    self.CoordName = 'Roll(deg)'
                    showValue = SensorAngEular[0]
                elif abs(self.LastSensorAngEular[1] - SensorAngEular[1]) > 0.00000001:  
                    self.CoordName = 'Pitch(deg)'
                    showValue = SensorAngEular[1]
                elif abs(self.LastSensorAngEular[2] - SensorAngEular[2]) > 0.0000000001: 
                    self.CoordName = 'Yaw(deg)'
                    showValue = SensorAngEular[2]
                else:
                    if self.CoordName == 'Roll(deg)':
                        showValue = SensorAngEular[0]
                    elif self.CoordName == 'Pitch(deg)':
                        showValue = SensorAngEular[1]
                    elif self.CoordName == 'Yaw(deg)':
                        showValue = SensorAngEular[2]
                    else:
                        showValue = 0
                self.img_ctrl.drawCoordinate(self.CoordName, 'left', round(
                    showValue), 30, 3, -180, 180, 10)
                self.LastSensorAngEular[0] = SensorAngEular[0]
                self.LastSensorAngEular[1] = SensorAngEular[1]
                self.LastSensorAngEular[2] = SensorAngEular[2]
                cv2.imshow('Img'+str(y), self.vis.Img[y])
                cv2.waitKey(1)
            else:
                print("Sensor" + str(y) + ": No data!")
            if y == 0:  
                # 以下代码用于实时更新相机参数（位置、焦距、角度、装载飞机和形式）
                vs = self.vis.VisSensor[0]  # 获取第0号相机基本参数
                # 修改其中的可变部分，只修改需要改变的部分即可
                # vs.TargetCopter=1  #修改视角绑定的飞机ID
                # vs.TargetMountType=0  # 修改视角绑定类型，固连飞机还是地面
                vs.CameraFOV = self.key_ctrl.CameraFOV   # 修改视角的视场角（焦距），可以模拟对焦相机  单位：度,范围：0到180度。
                # vs.SensorPosXYZ=[0.3,-0.15,0]  # 修改相机的位置，可以调整相机初始位置
                vs.SensorAngEular = SensorAngEular  # 修改相机的姿态，可以模拟云台转动 单位：弧度   -180到180
                self.vis.sendUpdateUEImage(vs)  # 发送更新数据


