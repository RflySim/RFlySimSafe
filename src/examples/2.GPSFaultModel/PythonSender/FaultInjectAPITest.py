# import required libraries
import time
import math
import numpy as np
# import RflySim APIs
import PX4MavCtrlV4 as PX4MavCtrl

# Create MAVLink control API instance
mav1 = PX4MavCtrl.PX4MavCtrler(1)
# mav2 = PX4MavCtrl.PX4MavCtrler(2)
# mav2 = PX4MavCtrl.PX4MavCtrler(3)
# mavN --> 20100 + (N-1)*2

# Init MAVLink data receiving loop
mav1.InitMavLoop()
#mav2.InitMavLoop(), ...

time.sleep(0.5)
mav1.InitTrueDataLoop()
time.sleep(0.5)

mav1.initOffboard()

lastTime = time.time()
startTime = time.time()
# time interval of the timer
timeInterval = 1/30.0 #here is 0.0333s (30Hz)

# flags for vehicle 1
flag = 0
flagTime=startTime

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
        #mav2.SendMavArm(True), ...
        
        print("Arm the drone!")
        
        # Takeoff to ten meters height
        mav1.SendPosNED(0,0,-10)
        time.sleep(0.5)
        mav1.SendMavArm(True) # Arm the drone
    
    if time.time() - startTime > 20 and flag == 1:
        flag = 2
        mav1.SendVelNED(0,3,0)
        print("Send Vel Ctrl!")

        
    if time.time() - startTime > 25 and flag==2:
        #np.zeros()
        silInt=np.zeros(8).astype(int).tolist()
        silFloat=np.zeros(20).astype(float).tolist()
        silInt[0:2]=[123546,123546]
        silFloat[0:4]=[100,3,10,0]
        mav1.sendSILIntFloat(silInt,silFloat)
        print('Inject a fault, and start loging')
        flag=3
    
    if flag==3:
        print(mav1.uavPosNED,mav1.truePosNED)
        if time.time() - startTime > 50:
            break
        
print('Sim End')

mav1.endMavLoop()
mav1.EndTrueDataLoop()