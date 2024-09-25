# import required libraries
import time
import math
import numpy as np
# import RflySim APIs
import PX4MavCtrlV4 as PX4MavCtrl

# Create MAVLink control API instance
mav1 = PX4MavCtrl.PX4MavCtrler(20100)
# mav2 = PX4MavCtrl.PX4MavCtrler(20102)
# mav2 = PX4MavCtrl.PX4MavCtrler(20104)
# mavN --> 20100 + (N-1)*2


# Init MAVLink data receiving loop
mav1.InitMavLoop()
#mav2.InitMavLoop(), ...

time.sleep(0.5)
mav1.InitTrueDataLoop()
time.sleep(0.5)

# mav1.initOffboard()

lastTime = time.time()
startTime = time.time()
# time interval of the timer
timeInterval = 1/30.0 #here is 0.0333s (30Hz)

# flags for vehicle 1
flag = 0
flagTime=startTime

targetPos = [150,0,-30]
# Start a endless loop with 30Hz, timeInterval=1/30.0
while True:
    lastTime = lastTime + timeInterval
    sleepTime = lastTime - time.time()
    if sleepTime > 0:
        time.sleep(sleepTime) # sleep until the desired clock
    else:
        lastTime = time.time()
    # The following code will be executed 30Hz (0.0333s)

    # Create the mission to vehicle 1
    if time.time() - startTime > 5 and flag == 0:
        # The following code will be executed at 5s
        print("5s, Arm the drone")
        flag = 1
        flagTime=time.time()
        mav1.SendMavArm(True) # Arm the drone
        mav1.sendMavTakeOff(targetPos[0], targetPos[1], targetPos[2])
        print("开始起飞")

    if flag==1:
        curPos=mav1.uavPosNED
        dis = math.sqrt((curPos[0]-targetPos[0])**2+(curPos[1]-targetPos[1])**2)
        if dis < 50:
            print('curPos:',curPos)
            print("到达起飞位置")
            flag = 2
            flagTime=time.time()
            flagI=0
            mav1.initOffboard()
            print("开始进入Offboard模式")   
            
            print("开始进入航路寻迹模式")

            mav1.SendPosNED(100, 100, -30)
            #设置盘旋半径为20m
            mav1.SendCruiseRadius(20)

    if flag == 2:
        mav1.sendMavLand(0,0,0)
        flag = 3
#     if time.time() - startTime > 20 and flag==1:
#         #np.zeros()
#         silInt=np.zeros(8).astype(int).tolist()
#         silFloat=np.zeros(20).astype(float).tolist()
#         # GPS 故障ID为123548
#         # GPS 故障参数有三个：第一个为干扰偏差、第二个为GPS定位类型3DFix,默认为3、第三个为卫星数量GPSSatsVisible，默认为10
#         # silInt[0:2]=[123548,123548] # 故障ID
#         # silFloat[0:3]=[0,3,5] # 故障参数
#         # mav1.sendSILIntFloat(silInt,silFloat)
#         print('Inject a fault, and start loging')
#         flag=2
    

#     if flag==2:
#         # print(mav1.uavPosNED,mav1.truePosNED)

#         if time.time() - startTime > 50:
#             break
        
        
# print('Sim End')

# mav1.endMavLoop()
# mav1.EndTrueDataLoop()