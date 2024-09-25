# import required libraries
import time
import math

# import RflySim APIs
import PX4MavCtrlV4 as PX4MavCtrl

# Create MAVLink control API instance
mav1 = PX4MavCtrl.PX4MavCtrler(20100)
mav2 = PX4MavCtrl.PX4MavCtrler(20102)
# mav2 = PX4MavCtrl.PX4MavCtrler(20104)
# mavN --> 20100 + (N-1)*2


# Init MAVLink data receiving loop
mav1.InitMavLoop()
mav2.InitMavLoop()

lastTime = time.time()
startTime = time.time()
# time interval of the timer
timeInterval = 1/30.0 #here is 0.0333s (30Hz)

# 圆形轨迹
n = 30
r = 400
missionPoints=[]
for i in range(n):
    angle = 2*math.pi*i/n
    x=r*math.sin(angle)
    y=r*math.cos(angle)
    missionPoints.append([x,y,-100])

# flags for vehicle 1
flag = 0
flagTime=startTime
flagI=0

# flags for vehicle 2
flag2 = 0
flagTime2=startTime
flagI2=0


# Start a endless loop with 30Hz, timeInterval=1/30.0
ctrlLast = [0,0,0,0]
targetPos=[0,0,0]
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
        mav2.SendMavArm(True), ...
        
        print("Arm the drone!")
        
        # 发送命令让飞机起飞，输入的五项分别是，最小俯仰角（单位rad），偏航角（单位rad），期望位置（单位m）X，Y，Z（相对于解锁位置）
        # 如果要发送绝对的GPS坐标作为起飞目标点，请用sendMavTakeOffGPS命令，最后三位分别是经度、维度、和高度，建议先从uavPosGPSHome向量中提取解锁GPS坐标，在此基础上用绝对坐标
        targetPos=[200, 0, -100]
        mav1.sendMavTakeOff(targetPos[0],targetPos[1],targetPos[2])
        time.sleep(0.5)
        print("开始起飞")
        mav1.SendMavArm(True) # Arm the drone

        time.sleep(1)
        targetPos=[200, 0, -100]
        mav2.sendMavTakeOff(targetPos[0],targetPos[1],targetPos[2])
        time.sleep(0.5)
        print("开始起飞")
        mav2.SendMavArm(True) # Arm the drone        
    
    if flag==1:
        curPos=mav1.uavPosNED
        dis = math.sqrt((curPos[0]-targetPos[0])**2+(curPos[1]-targetPos[1])**2)
        if dis < 50:
            print("到达起飞位置")
            flag = 2
            flagTime=time.time()
            flagI=0
            mav1.initOffboard()
            print("开始进入Offboard模式")   
                        
            
            print("开始进入航路寻迹模式")
            targetPos=missionPoints[flagI]
            #mav1.SendPosNEDExt(targetPos[0], targetPos[1], targetPos[2], 3)  # For PX4 1.22
            mav1.SendPosNED(targetPos[0], targetPos[1], targetPos[2])
            #设置盘旋半径为20m
            mav1.SendCruiseRadius(20)
            
            mav2.SendPosNED(targetPos[0]+30, targetPos[1], targetPos[2])
            #设置盘旋半径为20m
            mav2.SendCruiseRadius(20)
            
    
    if flag==2:
        if flagI>=len(missionPoints):
            flag=3
            flagTime=time.time()
        else:
            targetPos=missionPoints[flagI]
            curPos=mav1.uavPosNED
            dis = math.sqrt((curPos[0]-targetPos[0])**2+(curPos[1]-targetPos[1])**2)
            if dis < 50:
                flagI=flagI+1
                print("航路点切换"+str(flagI)+": "+str(targetPos[0])+", "+str(targetPos[1])+", "+str(targetPos[2]))
                spd = 10+flagI/3.0
                mav1.SendCruiseSpeed(spd)
                print("current speed: "+str(math.sqrt(mav1.uavVelNED[0]**2+mav1.uavVelNED[1]**2))+", Target speed: "+str(spd))
            else:
                #mav1.SendPosNEDExt(targetPos[0], targetPos[1], targetPos[2], 3) 
                mav1.SendPosNED(targetPos[0], targetPos[1], targetPos[2])
            
    if flag==3:
        pass
        # ...
        
        
    # Create the mission for vehicle 2
    # if time.time() - startTime > 5 and flag2 == 0:
    #     # The following code will be executed at 5s
    #     print("5s, Arm the drone")
    #     flag2 = 1
    
    # if flag2==1:
    #    pass
    
    
    
    # Create the mission for vehicle 3
    # if time.time() - startTime > 5 and flag3 == 0:
    #     # The following code will be executed at 5s
    #     print("5s, Arm the drone")
    #     flag3 = 1
    
    # if flag3==1:
    #    pass    
    