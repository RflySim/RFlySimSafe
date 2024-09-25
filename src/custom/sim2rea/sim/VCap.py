import time
import math
import sys
import VisionCaptureApi
import numpy as np
import PX4MavCtrlV4 as PX4MavCtrl
import cv2
import UE4CtrlAPI
import os
ue = UE4CtrlAPI.UE4CtrlAPI()

#Create a new MAVLink communication instance, UDP sending port (CopterSim’s receving port) is 20100
mav = PX4MavCtrl.PX4MavCtrler(1)

# The IP should be specified by the other computer
vis = VisionCaptureApi.VisionCaptureApi()

# Send command to UE4 Window 1 to change resolution 
ue.sendUE4Cmd('r.setres 1280x720w',0) # 设置UE4窗口分辨率，注意本窗口仅限于显示，取图分辨率在json中配置，本窗口设置越小，资源需求越少。
ue.sendUE4Cmd('t.MaxFPS 30',0) # 设置UE4最大刷新频率，同时也是取图频率
time.sleep(2)    

# VisionCaptureApi 中的配置函数
vis.jsonLoad() # 加载Config.json中的传感器配置文件

# vis.RemotSendIP = '192.168.3.80'
# 注意，手动修改RemotSendIP的值，可以将图片发送到本地址
# 如果不修改这个值，那么发送的IP地址为json文件中SendProtocol[1:4]定义的IP
# 图片的发送端口，为json中SendProtocol[5]定义好的。

isSuss = vis.sendReqToUE4() # 向RflySim3D发送取图请求，并验证
if not isSuss: # 如果请求取图失败，则退出
    sys.exit(0)
vis.startImgCap() # 开启取图，并启用共享内存图像转发，转发到填写的目录


#mav.InitMavLoop(UDPMode), where UDPMode=0,1,2,3,4
# Use MAVLink_Full Mode to control PX4
# In this mode, this script will send MAVLinkOffboard message to PX4 directly
# and receive MAVLink data from PX4
# Obviously, MAVLink_Full mode is slower than UDP mode, but the functions and data are more comprehensive
mav.InitMavLoop() # the same as mav.InitMavLoop() in other PythonVision demos

time.sleep(1)
print('Start Offboard Send!')
mav.initOffboard()
time.sleep(1)

# Check if the PX4'EKF has correctlly initialized, which is necessary for offboard control
if not mav.isPX4Ekf3DFixed:
    print('CopterSim/PX4 still not 3DFxied, please wait and try again.')
    sys.exit(0)
else:
    print('CopterSim/PX4 3D Fixed, ready to fly.')


output_folder = os.path.join(sys.path[0],'VCap')
os.makedirs(output_folder, exist_ok=True)

# 下面的程序非必需，仅用于观察图像用，在电脑性能不足时，请删除
lastTime = time.time()
lastClock=time.time()
png_ind = 1

start_time = time.time()
'''
2,1;1,1,1;2,3,0,0,-1;1,1,7;2,6,123544,123544,123544, 3, 0,50, 1,-1,-1;1,1,5;2,3,0,0,0;1,1,3
'''

flag1 = False
flag2 = False
flag3 = False

while True:
    lastTime = lastTime + 1/30.0
    sleepTime = lastTime - time.time()
    if sleepTime > 0:
        time.sleep(sleepTime)
    else:
        lastTime = time.time()  

    for i in range(len(vis.hasData)):
        if vis.hasData[i]:
            # 保存图像到文件夹
            image_filename = os.path.join(output_folder, f'Img_{png_ind}.png')
            cv2.imwrite(image_filename, vis.Img[i])

            # Process your image here
            cv2.imshow('Img'+str(i),vis.Img[i])
            cv2.waitKey(1)
    
    png_ind += 1
    
    if time.time() - start_time > 5 and flag1 == False:
        flag1 = True
        mav.SendPosNED(0,0,-2)
        print('起飞至1m')
    
    if time.time() - start_time > 12 and flag2 == False:
        flag2 = True
        silInts = [123544,123544,123544,0,0,0,0,0]
        silFloats = [3,0,50,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1]
        mav.sendSILIntFloat(silInts,silFloats)
        print('开始注入故障')

    if time.time() - start_time > 17 and flag3 == False:
        mav.sendMavLand(0,0,0)
        print('开始降落')

    
            
            

#mav.endOffboard()
#mav.stopRun()