import socket
import threading
import time
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2
import struct
import math
import sys
import copy

# PX4 main mode enumeration
class PX4_CUSTOM_MAIN_MODE:
    PX4_CUSTOM_MAIN_MODE_MANUAL = 1
    PX4_CUSTOM_MAIN_MODE_ALTCTL = 2
    PX4_CUSTOM_MAIN_MODE_POSCTL = 3
    PX4_CUSTOM_MAIN_MODE_AUTO = 4
    PX4_CUSTOM_MAIN_MODE_ACRO = 5
    PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
    PX4_CUSTOM_MAIN_MODE_STABILIZED = 7
    PX4_CUSTOM_MAIN_MODE_RATTITUDE = 8
    PX4_CUSTOM_MAIN_MODE_SIMPLE = 9

# PX4 sub mode enumeration
class PX4_CUSTOM_SUB_MODE_AUTO:
    PX4_CUSTOM_SUB_MODE_AUTO_READY = 1
    PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF = 2
    PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3
    PX4_CUSTOM_SUB_MODE_AUTO_MISSION = 4
    PX4_CUSTOM_SUB_MODE_AUTO_RTL = 5
    PX4_CUSTOM_SUB_MODE_AUTO_LAND = 6
    PX4_CUSTOM_SUB_MODE_AUTO_RTGS = 7
    PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET = 8
    PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND = 9

# define a class for MAVLink initialization
class fifo(object):
    def __init__(self):
        self.buf = []

    def write(self, data):
        self.buf += data
        return len(data)

    def read(self):
        return self.buf.pop(0)

class PX4SILIntFloat:
    # //输出到CopterSim DLL模型的SILints和SILFloats数据
    # struct PX4SILIntFloat{
    #     int checksum;//1234567897
    #     int CopterID;
    #     int inSILInts[8];
    #     float inSILFLoats[20];
    # };
    #struct.pack 10i20f
    def __init__(self):
        self.checksum=0
        self.CopterID=0
        self.inSILInts=[0,0,0,0,0,0,0,0]
        self.inSILFLoats=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        
    def __init__(self,iv):
        self.checksum=iv[0]
        self.CopterID=iv[1]
        self.inSILInts=iv[2:10]
        self.inSILFLoats=iv[10:30]


# struct reqVeCrashData {
# 	int checksum; //数据包校验码1234567897
# 	int copterID; //当前飞机的ID号
# 	int vehicleType; //当前飞机的样式
# 	int CrashType;//碰撞物体类型，-2表示地面，-1表示场景静态物体，0表示无碰撞，1以上表示被碰飞机的ID号
# 	double runnedTime; //当前飞机的时间戳
# 	float VelE[3]; // 当前飞机的速度
# 	float PosE[3]; //当前飞机的位置
# 	float CrashPos[3];//碰撞点的坐标
# 	float targetPos[3];//被碰物体的中心坐标
# 	float AngEuler[3]; //当前飞机的欧拉角
# 	float MotorRPMS[8]; //当前飞机的电机转速
#   float ray[6]; //飞机的前后左右上下扫描线
# 	char CrashedName[16];//被碰物体的名字
#  } 4i1d29f20s

class reqVeCrashData:
    def __init__(self):
        self.checksum=1234567897
        self.copterID=0
        self.vehicleType=0
        self.CrashType=0
        self.runnedTime=0
        self.VelE=[0,0,0]
        self.PosE=[0,0,0]
        self.CrashPos=[0,0,0]
        self.targetPos=[0,0,0]
        self.AngEuler=[0,0,0]
        self.MotorRPMS=[0,0,0,0,0,0,0,0]
        self.ray=[0,0,0,0,0,0]
        self.CrashedName=''
    def __init__(self,iv):
        self.checksum=iv[0]
        self.copterID=iv[1]
        self.vehicleType=iv[2]
        self.CrashType=iv[3]
        self.runnedTime=iv[4]
        self.VelE=iv[5:8]
        self.PosE=iv[8:11]
        self.CrashPos=iv[11:14]
        self.targetPos=iv[14:17]
        self.AngEuler=iv[17:20]
        self.MotorRPMS=iv[20:28]
        self.ray=iv[28:34]
        self.CrashedName=iv[34].decode('UTF-8')
        
# 注意:本条消息会发送给指定远端电脑的端口20005
# struct RflyTimeStmp{
#     int checksum; //校验位，取123456789
#     int copterID; //当前飞机的ID号
#     long long SysStartTime; //Windows下的开始仿真时的时间戳（单位毫秒，格林尼治标准起点）
#     long long SysCurrentTime;//Windows下的当前时间戳（单位毫秒，格林尼治标准起点）
#     long long HeartCount; //心跳包的计数器
# } 2i3q
class RflyTimeStmp:
    def __init__(self):
        self.checksum=1234567897
        self.copterID=0
        self.SysStartTime=0
        self.SysCurrentTime=0
        self.HeartCount=0
       
class PX4ExtMsg:
    def __init__(self):
        self.checksum=0
        self.CopterID=0
        self.runnedTime=0
        self.controls=[0,0,0,0,0,0,0,0]   
       
       
        
# PX4 MAVLink listen and control API and RflySim3D control API
class PX4MavCtrler:
    """Create a new MAVLink communication instance, 
    For UDP connect
    use format PX4MavCtrler(port,IP), e.g., PX4MavCtrler(20100,'127.0.0.1')
    For hardware connection
    Windows use format PX4MavCtrler('COM3') or PX4MavCtrler('COM3:115200') for Pixhawk USB port connection
    Windows use format 'COM4:57600' for Pixhawk serial port connection
    Linux use format PX4MavCtrler('/dev/ttyUSB0') or PX4MavCtrler('/dev/ttyUSB0:115200') for USB, or '/dev/ttyAMA0:57600' for Serial port (RaspberryPi example)
    """

    # constructor function
    def __init__(self, port=20100,ip='127.0.0.1'):
        self.isInPointMode=False
        self.isCom=False
        self.baud=115200
        if(isinstance(port, str)):
            self.isCom=True
            strlist = port.split(':')
            
            if(len(strlist)>=2):
                if strlist[1].isdigit():
                    self.baud=int(strlist[1])
                port=strlist[0]
                
        self.CopterID=1
        if not self.isCom:
            self.CopterID=int((port-20100)/2)+1
                
        self.f = fifo
        self.stopFlag = False
        self.mav0 = mavlink2.MAVLink(self.f)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create socket
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) 
        self.udp_socketUDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create socket
        self.udp_socketUDP.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) 
        self.udp_socketTrue = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create socket
        self.udp_socketPX4 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create socket
        self.udp_socketUE4 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create socket
        
        self.ip = ip  # IP address and port number to send data
        self.port = port
        
        self.uavTimeStmp=0
        self.trueTimeStmp=0
        self.uavAngEular = [0, 0, 0]  # Estimated Eular angles from PX4
        self.trueAngEular = [0, 0, 0] # True simulated Eular angles from CopterSim's DLL model
        self.uavAngRate = [0, 0, 0]  # Estimated angular rate from PX4
        self.trueAngRate = [0, 0, 0] # True simulated angular rate from CopterSim's DLL model
        self.uavPosNED = [0, 0, 0] # Estimated Local Pos (related to takeoff position) from PX4 in NED frame
        self.truePosNED = [0, 0, 0] # True simulated position (related to UE4 map center) from CopterSim's DLL model
        self.uavVelNED = [0, 0, 0] # Estimated local velocity from PX4 in NED frame
        self.trueVelNED = [0, 0, 0] # True simulated speed from CopterSim's DLL model  in NED frame
        self.isVehicleCrash=False # is the vehicle crashing with other
        self.isVehicleCrashID=-10 # the vehicle to collide
        self.uavPosGPS = [0, 0, 0] # Estimated GPS position from PX4 in NED frame
        self.uavPosGPSHome = [0, 0, 0] # Estimated GPS home (takeoff) position from PX4 in NED frame
        self.uavGlobalPos = [0, 0, 0] # Estimated global position from PX4 that transferred to UE4 map
        self.trueAngQuatern = [0, 0, 0, 0] # True simulated AngQuatern from CopterSim's DLL model  
        self.trueMotorRPMS = [0, 0, 0, 0, 0, 0, 0, 0] # True simulated MotorRPMS from CopterSim's DLL model 
        self.trueAccB = [0, 0, 0] # True simulated acc from CopterSim's DLL model 
        self.truePosGPS = [0, 0, 0] # True simulated PosGPS from CopterSim's DLL model
        self.trueSimulinkData=[0]*32 #create 32D data
        self.trueGpsUeCenter=[40.1540302,116.2593683,-50]
        self.uavThrust=0
        
        self.EnList = [0,1,0,0,0,1]
        self.type_mask = self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[0,0,0]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yaw=0
        self.yawrate = 0
        self.isInOffboard = False
        self.isArmed = False
        self.hasSendDisableRTLRC = False
        self.UDPMode=2
        self.startTime= time.time()
        self.MaxSpeed=5
        self.stopFlagTrueData=False
        self.hasTrueDataRec=False
        self.isPX4Ekf3DFixed = False
        
        self.RCPWMs=[0, 0, 0,0, 0, 0,0, 0]
        self.isRCLoop=False
        self.tRCHz=0.03
        
        self.inSilVect = []
        self.inReqVect = []
        self.inReqUpdateVect=[]
        
        self.stopFlagUE4=True
        
        self.isFailsafeEn = False
        #print("MAV Instance Created！")
        
        self.RflyTime =RflyTimeStmp()
        
        self.px4ext = PX4ExtMsg()
        self.px4extTrue=False
        
        self.offMode=0
        
        
    def sendStartMsg(self,copterID=-1):
        """ send start signals to the network for copters calling waitForStartMsg()
        if copterID=-1, then all copters will start to run
        if copterID>0, then only the copter with specified copterID will start to run
        """
        buf = struct.pack("3i",1234567890,1,copterID)
        """ The struct form is 
        struct startSignal{
            int checksum; // set to 1234567890 to verify the data
            int isStart; // should start to run
            int copterID; // the copter's ID to start
        }
        """
        self.udp_socket.sendto(buf, ('224.0.0.10', 20007)) # multicast address '224.0.0.10' and port 20007 are adopted here
        print("Send start Msg")
        time.sleep(0.03)

    def waitForStartMsg(self):
        """ Program will block until the start signal from sendStartMsg() is received
        """
        MYPORT = 20007
        MYGROUP = '224.0.0.10'
        ANY = '0.0.0.0'
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        sock.bind((ANY,MYPORT))
        status = sock.setsockopt(socket.IPPROTO_IP,
            socket.IP_ADD_MEMBERSHIP,
            socket.inet_aton(MYGROUP) + socket.inet_aton(ANY))
    
        sock.setblocking(1)
        #ts = time.time()
        
        print("Waiting for start Msg")
        while True:
            try:
                buf,addr = sock.recvfrom(65500)
                if len(buf)==12:
                    """The struct form is 
                        struct startSignal{
                            int checksum; // set to 1234567890 to verify the data
                            int isStart; // should start to run
                            int copterID; // the copter's ID to start
                        }
                    """
                    checksum,isStart,ID=struct.unpack('3i',buf)
                    if checksum==1234567890 and isStart:
                        if ID<0 or ID==self.CopterID:
                            print('Got start Msg, continue to run.')
                            break
            except:
                print("Error to listen to Start Msg!")
                sys.exit(0)


    def initPointMassModel(self,intAlt=0,intState=[0,0,0]):
        """ Init and start the point mass model for UAV control
        intAlt (unit m) is the init height of the vehicle on the UE4 map, which can be obtained from the CopterSim or UE4
        intState contains the PosX (m), PosY (m) and Yaw (degree) of the vehicle, which denotes the initial state of the vehicle
        it is the same as the value on CopterSim UI.
        """
        if self.isCom:
            print('Cannot run Pixhawk in this PointMass Mode!')
            sys.exit(0)
        self.isInPointMode=True
        self.intAlt=intAlt # Init altitude from CopterSim ground height of current map
        self.intStateX=intState[0] # Init PosX of CopterSim
        self.intStateY=intState[1] # Init PosY of CopterSim
        self.intStateYaw=intState[2] # Init Yaw angle of CopterSim
        self.t3 = threading.Thread(target=self.PointMassModelLoop, args=())
        self.t3.start()
        
        
    def EndPointMassModel(self):
        """ End the point mass model
        """
        self.isInPointMode=False
        time.sleep(0.5)
        self.t3.join()

    def yawSat(self,yaw):
        """ satuate the yaw angle from -pi to pi
        """
        if yaw>math.pi:
            yaw = yaw-math.pi*2
            yaw=self.yawSat(yaw)
        elif yaw <-math.pi:
            yaw = yaw+math.pi*2
            yaw=self.yawSat(yaw)
        return yaw

    def PointMassModelLoop(self):
        """ This is the dead loop for point mass model
        """
        # Offboard message sending loop, 100Hz
        self.startTime3 = time.time()
        self.startTime= time.time()
        self.lastTime3 = self.startTime3
        velE=[0,0,0]
        velOff=[0,0,0]
        yawOff=0
        yawRateOff=0
        iNum=0
        while True:
            if not self.isInPointMode:
                break
            self.startTime3 = self.startTime3 + 0.01
            sleepTime = self.startTime3 - time.time()
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                self.startTime3 = time.time()
            
            dt=time.time()-self.lastTime3
            
            velOff=list(self.vel)
            yawOff=self.yaw
            yawRateOff=self.yawrate
            
            # Calculate desired speed according to target Position
            if self.EnList[1]!=0: # if speed mode
                velOff=list(self.vel)
                yawRateOff=self.yawrate
            elif self.EnList[0]!=0: # if position mode
                targetPosE=list(self.pos)
                if self.coordinate_frame == mavlink2.MAV_FRAME_BODY_NED:
                    targetPosE[0] = self.pos[0]*math.cos(self.uavAngEular[2])+self.pos[1]*math.sin(self.uavAngEular[2])
                    targetPosE[1] = self.pos[0]*math.sin(self.uavAngEular[2])+self.pos[1]*math.cos(self.uavAngEular[2])
                velOff[0]=self.sat((targetPosE[0]-self.uavPosNED[0])*0.5,self.MaxSpeed)
                velOff[1]=self.sat((targetPosE[1]-self.uavPosNED[1])*0.5,self.MaxSpeed)
                velOff[2]=self.sat((targetPosE[2]-self.uavPosNED[2])*0.5,self.MaxSpeed)
                yawRateOff=self.sat((yawOff-self.uavAngEular[2])*2,math.pi/4)
            else:
                velOff=[0,0,0]
                yawOff=0
                yawRateOff=0
            
            # Calulate vehicle motion according to desired speed
            velE=list(velOff)
            if self.coordinate_frame == mavlink2.MAV_FRAME_BODY_NED:
                velE[0] = velOff[0]*math.cos(self.uavAngEular[2])+velOff[1]*math.sin(self.uavAngEular[2])
                velE[1] = velOff[0]*math.sin(self.uavAngEular[2])+velOff[1]*math.cos(self.uavAngEular[2])
            
            self.uavVelNED[0]=self.sat(self.uavVelNED[0]*0.97+velE[0]*0.03,15)
            self.uavVelNED[1]=self.sat(self.uavVelNED[1]*0.97+velE[1]*0.03,15)
            self.uavVelNED[2]=self.sat(self.uavVelNED[2]*0.97+velE[2]*0.03,10)
            
            self.uavAngRate[2]=self.sat(self.uavAngRate[2]*0.97+yawRateOff*0.03,math.pi)
            
            # if reach ground
            if self.uavPosNED[2]>0 and velOff[2]>0:
                self.uavVelNED=list([0,0,0])
                self.uavAngRate[2]=0
            
            self.uavPosNED[0]=self.uavVelNED[0]*dt+self.uavPosNED[0]
            self.uavPosNED[1]=self.uavVelNED[1]*dt+self.uavPosNED[1]
            self.uavPosNED[2]=self.uavVelNED[2]*dt+self.uavPosNED[2]
            self.uavAngEular[2]=self.uavAngRate[2]*dt+self.uavAngEular[2]
                
            self.uavAngEular[2]=self.yawSat(self.uavAngEular[2])
            bodyVx = self.uavVelNED[0]*math.cos(-self.uavAngEular[2])+self.uavVelNED[1]*math.sin(-self.uavAngEular[2])
            bodyVy = self.uavVelNED[0]*math.sin(-self.uavAngEular[2])+self.uavVelNED[1]*math.cos(-self.uavAngEular[2])
            
            
            # Calulate desired angle according to speed
            self.uavAngEular[0]=bodyVy/15*math.pi/3
            self.uavAngEular[1]=-bodyVx/15*math.pi/3
            
            
            # calculate vehicle state for UE4
            if self.uavPosNED[2]<-0.01:
                MotorRPMS=[1000,1000,1000,1000,1000,1000,1000,1000]
            else:
                MotorRPMS=[0,0,0,0,0,0,0,0]
            
            self.trueVelNED=list(self.uavVelNED)
            self.trueAngRat=list(self.uavAngRate)
            self.truePosNED[0]=self.intStateX+self.uavPosNED[0]
            self.truePosNED[1]=self.intStateY+self.uavPosNED[1]
            self.truePosNED[2]=self.intAlt+self.uavPosNED[2]
            self.trueAngEular[0]=self.uavAngEular[0]
            self.trueAngEular[1]=self.uavAngEular[1]
            self.trueAngEular[2]=self.yawSat(self.uavAngEular[2]+self.intStateYaw)
            self.uavGlobalPos=list(self.truePosNED)
            self.uavTimeStmp=time.time()
            self.trueTimeStmp=time.time()
            
            sendUE4Msg=True
            if self.CopterID>4:
                sendUE4Msg=False
                iNum=iNum+1
                if iNum%3==0:
                    sendUE4Msg=True
            # if vehicle number<=4, send UE4 msg with 100Hz
            # if vehicle number is too large, send UE4 msg with 33Hz to save network
            if sendUE4Msg:
                # Send vehicle to UE4
                #sendUE4PosNew(self,copterID,vehicleType,PosE,AngEuler,Vel,PWMs,runnedTime
                self.sendUE4PosNew(self.CopterID,3,self.truePosNED,self.trueAngEular,self.trueVelNED,MotorRPMS)
            self.lastTime3=time.time()
        #print("Point Mode Stoped.")


    def InitTrueDataLoop(self):
        """ Initialize UDP True data listen loop from CopterSim through 30100 series ports
        """
        self.udp_socketTrue.bind(('0.0.0.0', self.port+1+10000))
        self.stopFlagTrueData=False
        self.tTrue = threading.Thread(target=self.getTrueDataMsg, args=())
        self.tTrue.start()
        
        self.udp_socketPX4.bind(('0.0.0.0', self.port+1+20000))
        self.stopFlagPX4Data=False
        self.tPX4 = threading.Thread(target=self.getPX4DataMsg, args=())
        self.tPX4.start()        
        

    def EndTrueDataLoop(self):
        """ End the true data mode
        """
        self.stopFlagTrueData=True
        time.sleep(0.5)
        self.tTrue.join()
        self.hasTrueDataRec=False
        self.udp_socketTrue.close()
        
    def initUE4MsgRec(self):
        """ Initialize the UDP data linsening from UE4,
        currently, the crash data is listened
        """
        self.stopFlagUE4=False
        self.inSilVect = []
        self.inReqVect = []
        MYPORT = 20006
        MYGROUP = '224.0.0.10'
        ANY = '0.0.0.0'
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.udp_socketUE4.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.udp_socketUE4.bind((ANY,MYPORT))
        status = self.udp_socketUE4.setsockopt(socket.IPPROTO_IP,
            socket.IP_ADD_MEMBERSHIP,
            socket.inet_aton(MYGROUP) + socket.inet_aton(ANY))
        self.t4 = threading.Thread(target=self.UE4MsgRecLoop, args=())
        self.t4.start() 
        
    def endUE4MsgRec(self):      
        """ End UE4 message listening
        """  
        self.stopFlagUE4=True
        time.sleep(0.5)
        self.t4.join()
        self.udp_socketUE4.close()
        
    def UE4MsgRecLoop(self):
        """ UE4 message listening dead loop
        """
        lastTime = time.time()
        while True:
            if self.stopFlagUE4:
                break
            lastTime = lastTime + 0.01
            sleepTime = lastTime - time.time()
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                lastTime = time.time()
            # print(time.time())
            
            # struct CopterSimCrash {
            # 	int checksum;
            # 	int CopterID;
            # 	int TargetID;
            # }
            while True:
                if self.stopFlagUE4:
                    break

                try:
                    buf,addr = self.udp_socketUE4.recvfrom(65500)
                    #print('Data Received!')
                    if len(buf)==12:
                        checksum,CopterID,targetID = struct.unpack('iii',buf[0:12])
                        if checksum==1234567890:
                            if targetID>-0.5 and CopterID==self.CopterID:
                                self.isVehicleCrash=True
                                self.isVehicleCrashID=targetID
                            print('Vehicle #',CopterID,' Crashed with vehicle #',targetID)
                            
                    if len(buf)==120:
                        iValue = struct.unpack('10i20f',buf[0:120])
                        if iValue[0] == 1234567897:
                            isCopterExist=False
                            for i in range(len(self.inSilVect)): #遍历数据列表，飞机ID有没有出现过
                                if self.inSilVect[i].CopterID == iValue[1]: #如果出现过，就直接更新数据
                                    isCopterExist=True
                                    self.inSilVect[i].checksum=iValue[0]
                                    self.inSilVect[i].inSILInts=iValue[2:10]
                                    self.inSilVect[i].inSILFLoats=iValue[10:30]
                                    break
                            if not isCopterExist:#如果没有出现过，就创建一个结构体
                                vsr=PX4SILIntFloat(iValue)
                                self.inSilVect = self.inSilVect +  [copy.deepcopy(vsr)] #扩充列表，增加一个元素
                                
                    if len(buf)==160:
                        
                        isCopterExist=False
                        iValue=struct.unpack('4i1d29f20s',buf[0:160])
                        if(iValue[0]==1234567897):
                            vsr=reqVeCrashData(iValue)
                            #print(vsr.copterID,vsr.vehicleType)
                            for i in range(len(self.inReqVect)): #遍历数据列表，飞机ID有没有出现过
                                if self.inReqVect[i].copterID == iValue[1]: #如果出现过，就直接更新数据
                                    isCopterExist=True
                                    self.inReqVect[i]=copy.deepcopy(vsr)
                                    self.inReqUpdateVect[i]=True
                                    break
                            if not isCopterExist:#如果没有出现过，就创建一个结构体
                                self.inReqVect = self.inReqVect +  [copy.deepcopy(vsr)] #扩充列表，增加一个元素
                                self.inReqUpdateVect = self.inReqUpdateVect +[True]

                except:
                    self.stopFlagUE4=True
                    print('Error Data')
                    
                    break
            
        

    def InitMavLoop(self,UDPMode=2):
        """ Initialize MAVLink listen loop from CopterSim
            0 and 1 for UDP_Full and UDP_Simple Modes, 2 and 3 for MAVLink_Full and MAVLink_Simple modes, 4 for MAVLink_NoSend
            The default mode is MAVLink_Full
        """
        self.UDPMode=UDPMode
        if UDPMode>1.5: # UDPMode should lisen to PX4
            if self.isCom:
                self.the_connection = mavutil.mavlink_connection(self.port,self.baud)
            else:
                self.the_connection = mavutil.mavlink_connection('udpin:0.0.0.0:'+str(self.port+1))
        else:
            if not self.isCom:
                self.udp_socketUDP.bind(('0.0.0.0', self.port+1))
        self.lastTime = 0
        self.t1 = threading.Thread(target=self.getMavMsg, args=())
        self.t1.start()
        self.t2 = threading.Thread(target=self.OffboardSendMode, args=())
        self.startTime = time.time()
        self.lastTime2 = 0
        self.startTime2 = time.time()
        self.isFailsafeEn = False

    def endMavLoop(self):
        """ The same as stopRun(), stop message listenning from 20100 or serial port
        """
        self.stopRun()

    # saturation function
    def sat(self,inPwm=0,thres=1):
        """Saturation function for value inPwm with range thres
        if inPwm>thres, then inPwm=thres
        if inPwm<-thres,then inPwm=-thres
        """
        outPwm= inPwm
        if inPwm>thres:
            outPwm = thres
        elif inPwm<-thres:
            outPwm = -thres
        return outPwm

    # send MAVLink command long message to Pixhawk (routed through CopterSim)
    def SendMavCmdLong(self, command, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
        """ Send command long message to PX4, the mavlink command definitions can be found at
        https://mavlink.io/en/messages/common.html#COMMAND_LONG
        https://mavlink.io/en/messages/common.html#MAV_CMD
        """
        if self.isInPointMode or self.UDPMode<1.5:
            return
        if self.isCom:
            self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component,
                                            command, 0,
                                            param1, param2, param3, param4, param5, param6, param7)
        else: 
            buf = self.mav0.command_long_encode(self.the_connection.target_system, self.the_connection.target_component,
                                                command, 0,
                                                param1, param2, param3, param4, param5, param6, param7).pack(self.mav0)
            self.udp_socket.sendto(buf, (self.ip, self.port))

    # send command to make Pixhawk enter Offboard mode
    def sendMavOffboardCmd(self,type_mask,coordinate_frame, x,  y,  z,  vx,  vy,  vz,  afx,  afy,  afz,  yaw, yaw_rate):
        """ send offboard command to PX4, the definition of the mavlink message can be found at
        https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
        """
        time_boot_ms = int((time.time()-self.startTime)*1000)
        if self.isInPointMode or self.UDPMode<1.5:
            return
        if self.isCom:
            self.the_connection.mav.set_position_target_local_ned_send(time_boot_ms,self.the_connection.target_system,
                                                                    self.the_connection.target_component,
                                                                    coordinate_frame,type_mask,x,  y,  z,  vx,  vy,  vz,  afx,
                                                                    afy,  afz,  yaw, yaw_rate)
        else:
            buf = self.mav0.set_position_target_local_ned_encode(time_boot_ms,self.the_connection.target_system,
                                                                    self.the_connection.target_component,
                                                                    coordinate_frame,type_mask,x,  y,  z,  vx,  vy,  vz,  afx,
                                                                    afy,  afz,  yaw, yaw_rate).pack(self.mav0)
            self.udp_socket.sendto(buf, (self.ip, self.port))

    def TypeMask(self,EnList):
        """ Obtain the bitmap for offboard message
        https://mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK
        """
        enPos = EnList[0]
        enVel = EnList[1]
        enAcc = EnList[2]
        enForce = EnList[3]
        enYaw = EnList[4]
        EnYawrate= EnList[5]
        y=int(0)
        if not enPos:
            y = y | 7

        if not enVel:
            y = y | (7<<3)

        if not enAcc:
            y = y | (7<<6)

        if not enForce:
            y = y | (1<<9)

        if not enYaw:
            y = y | (1<<10)

        if not EnYawrate:
            y = y|(1<<11)
        type_mask = y
        return int(type_mask)        

    # set the control sigals for Offboard sending loop
    def sendMavOffboardAPI(self,type_mask=0,coordinate_frame=0,pos=[0,0,0],vel=[0,0,0],acc=[0,0,0],yaw=0,yawrate=0):
        """send offboard command to PX4, the definition of the mavlink message can be found at
        https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
        """
        if self.isInPointMode:
            return
        time_boot_ms = int((time.time()-self.startTime)*1000)
        if self.isCom:
            
            if self.offMode==0:            
                self.the_connection.mav.set_position_target_local_ned_send(int(time_boot_ms),self.the_connection.target_system,
                                                                    self.the_connection.target_component,
                                                                    coordinate_frame,type_mask,pos[0],  pos[1],  pos[2],
                                                                    vel[0],  vel[1],  vel[2],  acc[0],
                                                                    acc[1],  acc[2],  yaw, yawrate)
            elif self.offMode==1: 
                lat_int=int(pos[0]*1e7)
                lon_int=int(pos[1]*1e7)
                self.the_connection.mav.set_position_target_global_int_send(int(time_boot_ms),self.the_connection.target_system,
                                                                    self.the_connection.target_component,
                                                                    coordinate_frame,type_mask,lat_int,  lon_int,  pos[2],
                                                                    vel[0],  vel[1],  vel[2],  acc[0],
                                                                    acc[1],  acc[2],  yaw, yawrate)  
            elif self.offMode==2: 
                self.the_connection.mav.set_attitude_target_send(int(time_boot_ms),self.the_connection.target_system,
                                                                        self.the_connection.target_component,type_mask,
                                                                        [yawrate,pos[0],pos[1],pos[2]],vel[0],  vel[1], vel[2],yaw)                                  
        else:
            if self.UDPMode>1.5:
                if self.offMode==0:
                    buf = self.mav0.set_position_target_local_ned_encode(int(time_boot_ms),self.the_connection.target_system,
                                                                        self.the_connection.target_component,
                                                                        coordinate_frame,type_mask,pos[0],  pos[1],  pos[2],
                                                                        vel[0],  vel[1],  vel[2],  acc[0],
                                                                        acc[1],  acc[2],  yaw, yawrate).pack(self.mav0)
                elif self.offMode==1: 
                    lat_int=int(pos[0]*1e7)
                    lon_int=int(pos[1]*1e7)   
                    buf = self.mav0.set_position_target_global_int_encode(int(time_boot_ms),self.the_connection.target_system,
                                                                        self.the_connection.target_component,
                                                                        coordinate_frame,type_mask,lat_int,  lon_int,  pos[2],
                                                                        vel[0],  vel[1],  vel[2],  acc[0],
                                                                        acc[1],  acc[2],  yaw, yawrate).pack(self.mav0)   
                elif self.offMode==2:
                    buf = self.mav0.set_attitude_target_encode(int(time_boot_ms),self.the_connection.target_system,
                                                                        self.the_connection.target_component,type_mask,
                                                                        [yawrate,pos[0],pos[1],pos[2]],vel[0],  vel[1], vel[2],yaw).pack(self.mav0)   
                self.udp_socket.sendto(buf, (self.ip, self.port))
            else:
                # UDP_Full Mode
                if self.UDPMode==0:
                    # struct inHILCMDData{
                    #     uint32_t time_boot_ms;
                    #     uint32_t copterID;
                    #     uint32_t modes;
                    #     uint32_t flags;
                    #     float ctrls[16];
                    # };
                    # typedef struct _netDataShortShort {
                    #     TargetType tg;
                    #     int        len;
                    #     char       payload[PAYLOAD_LEN_SHORT_SHORT];
                    # }netDataShortShort;
                    ctrls=pos+vel+acc+[yaw,yawrate]+[0,0,0,0,0]
                    buf0 = struct.pack("4I16f",time_boot_ms,self.CopterID,type_mask,coordinate_frame,*ctrls)
                    # buf for remaining 192-152=40bytes of payload[192] of netDataShort
                    buf1 = bytes([0]*(112-len(buf0)))
                    # buf for tg and len in netDataShort
                    buf2 = struct.pack("2i",3,len(buf0))
                    # buf for netDataShort
                    buf=buf2+buf0+buf1
                    self.udp_socket.sendto(buf, (self.ip, self.port))
                else: # UDP_Simple Mode
                    
                    # struct inOffboardShortData{
                    #     int checksum;
                    #     int ctrlMode;
                    #     float controls[4];
                    # };
                    
                    checksum=1234567890
                    ctrlMode=0
                    ctrls=vel+[yawrate]
                    
                    if self.EnList[0]==1 and coordinate_frame == 8: # POS, MAV_FRAME_BODY_NED
                        ctrlMode=3
                        ctrls=pos+[yaw]
                    elif self.EnList[0]==0 and coordinate_frame == 8: # Vel, MAV_FRAME_BODY_NED
                        ctrlMode=1
                        ctrls=vel+[yawrate]
                        #print(ctrlMode,ctrls)
                    elif self.EnList[0]==1 and coordinate_frame == 1: # POS, MAV_FRAME_LOCAL_NED
                        ctrlMode=2
                        ctrls=pos+[yaw]
                    else: # Vel, MAV_FRAME_LOCAL_NED
                        ctrlMode=0
                        ctrls=vel+[yawrate]
                    buf = struct.pack("2i4f",checksum,ctrlMode,ctrls[0],ctrls[1],ctrls[2],ctrls[3])
                    self.udp_socket.sendto(buf, (self.ip, self.port))
                    #print(ctrlMode,ctrls)


    # struct PX4UorbRflyCtrl {
    #     int checksum; #1234567896
    #     int CopterID;
    #     uint32_t modes;
    #     uint32_t flags;
    #     float data[16];
    # }2i2I16f
    def sendPX4UorbRflyCtrl(self,data=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],modes=1,flags=1):
        checksum=1234567896
        buf = struct.pack("2i2I16f",checksum,self.CopterID,modes,flags,*data)
        self.udp_socket.sendto(buf, (self.ip, self.port+10000))

    # send velocity control signal in earth north-east-down (NED) frame to Pixhawk
    def SendVelNED(self,vx=0,vy=0,vz=0,yawrate=0):
        """ Send targe vehicle speed (m/s) to PX4 in the earth north-east-down (NED) frame with yawrate (rad/s)
        when the vehicle fly upward, the vz < 0
        """
        self.EnList = [0,1,0,0,0,1]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.pos=[0,0,0]
        self.vel = [vx,vy,vz]
        self.acc = [0, 0, 0]
        self.yaw = 0
        self.yawrate = yawrate

    # send velocity control signal in earth north-east-down (NED) frame to Pixhawk
    def SendVelNEDNoYaw(self,vx,vy,vz):
        """ Send targe vehicle speed (m/s) to PX4 in the earth north-east-down (NED) frame without yaw control
        when the vehicle fly upward, the vz < 0
        """
        self.EnList = [0,1,0,0,0,0]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.pos=[0,0,0]
        self.vel = [vx,vy,vz]
        self.acc = [0, 0, 0]
        self.yaw = 0
        self.yawrate = 0


    def sendUE4Cmd(self,cmd,windowID=-1):
        """send command to control the display style of RflySim3D
            The available command are list as follows, the command string is as b'RflyShowTextTime txt time'
            RflyShowTextTime(String txt, float time)\\ let UE4 show txt with time second
            RflyShowText(String txt)\\  let UE4 show txt 5 second
            RflyChangeMapbyID(int id)\\ Change the map to ID (int number)
            RflyChangeMapbyName(String txt)\\ Change to map with name txt
            RflyChangeViewKeyCmd(String key, int num) \\ the same as press key + num on UE4
            RflyCameraPosAngAdd(float x, float y, float z,float roll,float pitch,float yaw) \\ move the camera with x-y-z(m) and roll-pitch-yaw(degree) related to current pos
            RflyCameraPosAng(float x, float y, float z, float roll, float pitch, float yaw) \\ set the camera with x-y-z(m) and roll-pitch-yaw(degree) related to UE origin
            RflyCameraFovDegrees(float degrees) \\ change the cameras fov (degree)
            RflyChange3DModel(int CopterID, int veTypes=0) \\ change the vehicle 3D model to ID
            RflyChangeVehicleSize(int CopterID, float size=0) \\change vhielce's size
            RflyMoveVehiclePosAng(int CopterID, int isFitGround, float x, float y, float z, float roll, float pitch, float yaw) \\ move the vehicle's  x-y-z(m) and roll-pitch-yaw(degree) related to current pos
            RflySetVehiclePosAng(int CopterID, int isFitGround, float x, float y, float z, float roll, float pitch, float yaw) \\ set the vehilce's x-y-z(m) and roll-pitch-yaw(degree) related to UE origin
            RflyScanTerrainH(float xLeftBottom(m), float yLeftBottom(m), float xRightTop(m), float yRightTop(m), float scanHeight(m), float scanInterval(m)) \\ send command to let UE4 scan the map to generate png and txt files
            RflyCesiumOriPos(double lat, double lon, double Alt) \\ change the lat, lon, Alt (degrees) of the Cesium map origin
            RflyClearCapture \\ clear the image capture unit
            struct Ue4CMD{
                int checksum;
                char data[52];
            }
        """
        buf = struct.pack("i52s",1234567890,cmd)
        if windowID<0:
            if self.ip=='127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009)) #multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            if self.ip!='127.0.0.1' and self.ip!='255.255.255.255':
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID)) #ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID)) #specify PC's IP to send


    def sendUE4Attatch(self,CopterIDs,AttatchIDs,AttatchTypes,windowID=-1):
        """ Send msg to UE4 to attach a vehicle to another (25 vehicles);
        CopterIDs,AttatchIDs,AttatchTypes can be a list with max len 25
        """
        # change the 1D variable to 1D list
        if isinstance(CopterIDs,int):
            CopterIDs=[CopterIDs]

        if isinstance(AttatchIDs,int):
            AttatchIDs=[AttatchIDs]
            
        if isinstance(AttatchTypes,int):
            AttatchTypes=[AttatchTypes]            
        
        if not isinstance(CopterIDs,list) or not isinstance(AttatchIDs,list) or not isinstance(AttatchTypes,list):
            print('Error: Wrong sendUE4Attatch input Type');
            return
                
        if len(CopterIDs)!=len(AttatchIDs) or len(CopterIDs)!=len(AttatchTypes) or len(CopterIDs)>25:
            print('Error: Wrong sendUE4Attatch input dimension');
            return
        
        vLen=len(CopterIDs)
        if vLen<25: # Extend the IDs to 25D
            CopterIDs = CopterIDs + [0]*(25-vLen)
            AttatchIDs = AttatchIDs + [0]*(25-vLen)
            AttatchTypes = AttatchTypes + [0]*(25-vLen)
        
        if vLen>25:
            CopterIDs=CopterIDs[0:25]
            AttatchIDs=AttatchIDs[0:25]
            AttatchTypes=AttatchTypes[0:25]
        
        # struct VehicleAttatch25 {
        # 	int checksum;//1234567892
        # 	int CopterIDs[25];
        # 	int AttatchIDs[25];
        # 	int AttatchTypes[25];//0：正常模式，1：相对位置不相对姿态，2：相对位置+偏航（不相对俯仰和滚转），3：相对位置+全姿态（俯仰滚转偏航）
        # }i25i25i25i
        
        buf = struct.pack("i25i25i25i",1234567892,*CopterIDs,*AttatchIDs,*AttatchTypes)
        if windowID<0:
            if self.ip=='127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009)) #multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            if self.ip!='127.0.0.1' and self.ip!='255.255.255.255':
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID)) #ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID)) #specify PC's IP to send
        

    def sendUE4Pos(self,copterID=1,vehicleType=3,MotorRPMSMean=0,PosE=[0,0,0],AngEuler=[0,0,0],windowID=-1):
        """send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
        """
        VelE=[0,0,0]
        self.sendUE4PosNew(copterID,vehicleType,PosE,AngEuler,VelE,[MotorRPMSMean]*8,-1,windowID)

    # send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
    def sendUE4Pos2Ground(self,copterID=1,vehicleType=3,MotorRPMSMean=0,PosE=[0,0,0],AngEuler=[0,0,0],windowID=-1):
        """send the position & angle information to RflySim3D to create a new 3D model or update the old model's states on the ground
            checksum =1234567891 is adopted here to tell UE4 to genearete a object always fit the ground
            struct SOut2SimulatorSimple {
                int checkSum; //1234567890 for normal object, 1234567891 for fit ground object
                int copterID;  //Vehicle ID
                int vehicleType;  //Vehicle type
                float MotorRPMSMean; // mean motor speed
                float PosE[3];   //NED vehicle position in earth frame (m)
                float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
            }  3i7f
        """
        buf = struct.pack("3i7f",1234567891,copterID,vehicleType,MotorRPMSMean,PosE[0],PosE[1],PosE[2],AngEuler[0],AngEuler[1],AngEuler[2])
        if windowID<0:
            if self.ip=='127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009)) #multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            if self.ip!='127.0.0.1' and self.ip!='255.255.255.255':
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID)) #ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID)) #specify PC's IP to send


    def sendUE4PosScale(self,copterID=1,vehicleType=3,MotorRPMSMean=0,PosE=[0,0,0],AngEuler=[0,0,0],Scale=[1,1,1],windowID=-1):
        """send the position & angle information to RflySim3D to create a new 3D model or update the old model's states with changing the scale
            struct SOut2SimulatorSimple1 {
                int checkSum; //1234567890 for normal object, 1234567891 for fit ground object
                int copterID;  //Vehicle ID
                int vehicleType;  //Vehicle type
                float MotorRPMSMean; // mean motor speed
                float PosE[3];   //NED vehicle position in earth frame (m)
                float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
                float Scale[3];
            }  3i10f
        """
        buf = struct.pack("3i10f",1234567890,copterID,vehicleType,MotorRPMSMean,PosE[0],PosE[1],PosE[2],AngEuler[0],AngEuler[1],AngEuler[2],Scale[0],Scale[1],Scale[2])
        if windowID<0:
            if self.ip=='127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009)) #multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            if self.ip!='127.0.0.1' and self.ip!='255.255.255.255':
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID)) #ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID)) #specify PC's IP to send

    def sendUE4PosScale2Ground(self,copterID=1,vehicleType=3,MotorRPMSMean=0,PosE=[0,0,0],AngEuler=[0,0,0],Scale=[1,1,1],windowID=-1):
        """send the position & angle information to RflySim3D to create a new 3D model or update the old model's states with changing the scale
            checksum =1234567891 is adopted here to tell UE4 to genearete a object always fit the ground
            struct SOut2SimulatorSimple1 {
                int checkSum; //1234567890 for normal object, 1234567891 for fit ground object
                int copterID;  //Vehicle ID
                int vehicleType;  //Vehicle type
                float MotorRPMSMean; // mean motor speed
                float PosE[3];   //NED vehicle position in earth frame (m)
                float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
                float Scale[3];
            }  3i10f
        """
        buf = struct.pack("3i10f",1234567891,copterID,vehicleType,MotorRPMSMean,PosE[0],PosE[1],PosE[2],AngEuler[0],AngEuler[1],AngEuler[2],Scale[0],Scale[1],Scale[2])
        if windowID<0:
            if self.ip=='127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009)) #multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            if self.ip!='127.0.0.1' and self.ip!='255.255.255.255':
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID)) #ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID)) #specify PC's IP to send

    # send velocity control signal in body front-right-down (FRD) frame
    def SendVelFRD(self,vx=0,vy=0,vz=0,yawrate=0):
        """ Send vehicle targe speed (m/s) to PX4 in the body forward-rightward-downward (FRD) frame with yawrate control (rad/s)
        when the vehicle fly upward, the vz < 0
        """
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        self.EnList = [0,1,0,0,0,1]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[0,0,0]
        self.vel = [vx,vy,vz]
        self.acc = [0, 0, 0]
        self.yaw = 0
        self.yawrate = yawrate

    # send velocity control signal in body front-right-down (FRD) frame
    def SendAttPX4(self,att=[0,0,0,0],thrust=0.5,CtrlFlag=0):
        """ Send vehicle targe attitude to PX4 in the body forward-rightward-downward (FRD) frame 
        """
        #CtrlFlag is a flag to determine the definition of input att
        #CtrlFlag 0 : att is a 3D vector for euler angles, roll,pitch,yaw, unit is degree
        #CtrlFlag 1 : att is a 3D vector for euler angles, roll,pitch,yaw, unit is rad
        #CtrlFlag 2 : att is a 4D vector for quaternion
        #CtrlFlag 3 : att is a 3D vector for rotation rate, roll,pitch,yaw, unit is rad/s
        #CtrlFlag 4 : att is a 3D vector for rotation rate, roll,pitch,yaw, unit is degree/s
        
        # thrust is defined as "Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)" from PX4 web
        
        # https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET
        self.offMode=2 # SET_ATTITUDE_TARGET
        self.EnList = [0,0,0,0,0,0]
        
        y=int(0)
        if CtrlFlag<3: # Ignore body rate
            y = y | 7
        else:
            y = y | (1<<7) # Ignore body attitude

        self.type_mask=y
        
        quat=[1,0,0,0] #Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        if CtrlFlag==0 or CtrlFlag==1:
            roll=att[0]
            pitch=att[1]
            yaw=att[2]
            if CtrlFlag==0: # convert to unit rad
                roll=att[0]/180.0*math.pi
                pitch=att[1]/180.0*math.pi
                yaw=att[2]/180.0*math.pi
            
            sp2=math.sin(pitch/2)
            sy2=math.sin(yaw/2)
            cr2=math.cos(roll/2)
            cp2=math.cos(pitch/2)
            cy2=math.cos(yaw/2)
            sr2=math.sin(roll/2)
        
            qx=sp2*sy2*cr2+cp2*cy2*sr2
            qy=sp2*cy2*cr2+cp2*sy2*sr2
            qz=cp2*sy2*cr2-sp2*cy2*sr2
            qw=cp2*cy2*cr2-sp2*sy2*sr2
            quat=[qw,qx,qy,qz]
        if CtrlFlag==2:
            quat=att
        
        self.pos=[quat[1],quat[2],quat[3]]
        self.yawrate = quat[0]
        
        rate=[0,0,0]
        if CtrlFlag==3:
            rate=[att[0],att[1],att[2]]
        if CtrlFlag==4:
            rate=[att[0]/180.0*math.pi,att[1]/180.0*math.pi,att[2]/180.0*math.pi]
        self.vel = rate
        self.acc = [0, 0, 0]
        self.yaw = thrust
        


    def SendAccPX4(self,afx=0,afy=0,afz=0,yawValue=0,yawType=0,frameType=0):
        """ Send a targe acceleration (m/s^2) to PX4
        """
        # From PX4 Web: Acceleration setpoint values are mapped to create a normalized 
        # thrust setpoint (i.e. acceleration setpoints are not "properly" supported).
        # yawType 0: No yaw and yawrate
        # yawType 1: Yaw control
        # yawType 2: yaw Rate Ctrl
        # frameType 0: earth NED frame
        # frameType 1: body forward-rightward-downward (FRD) frame
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        if yawType==0:
            self.EnList = [0,0,1,1,0,0]
            self.yaw = 0
            self.yawrate = 0  
        elif yawType==1:
            self.EnList = [0,0,1,1,1,0]
            self.yaw = yawValue
            self.yawrate = 0  
        else:
            self.EnList = [0,0,1,1,0,1]
            self.yaw = 0
            self.yawrate = yawValue  
            
        self.type_mask=self.TypeMask(self.EnList)
        if frameType==0:
            self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        else:
            self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[0,0,0]
        self.vel = [0,0,0]
        self.acc = [afx, afy, afz]
        
        
    # send velocity control signal in body front-right-down (FRD) frame
    def SendVelNoYaw(self,vx,vy,vz):
        """ Send vehicle targe speed (m/s) to PX4 in the body forward-rightward-downward (FRD) frame without yawrate control (rad)
        when the vehicle fly upward, the vz < 0
        """
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        self.EnList = [0,1,0,0,0,0]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[0,0,0]
        self.vel = [vx,vy,vz]
        self.acc = [0, 0, 0]
        self.yaw = 0
        self.yawrate = 0

    # send target position in earth NED frame
    def SendPosNED(self,x=0,y=0,z=0,yaw=0):
        """ Send vehicle targe position (m) to PX4 in the earth north-east-down (NED) frame with yaw control (rad)
        when the vehicle fly above the ground, then z < 0
        """
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        self.EnList = [1,0,0,0,1,0]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.pos=[x,y,z]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yawrate = 0
        self.yaw = yaw

    # send target position in earth NED frame
    def SendPosGlobal(self,lat=0,lon=0,alt=0,yawValue=0,yawType=0):
        """ Send vehicle targe position (m) to PX4 in the earth north-east-down (NED) frame with yaw control (rad)
        when the vehicle fly above the ground, then z < 0
        """
        # yawType 0: No yaw and yawrate
        # yawType 1: Yaw control
        # yawType 2: yaw Rate Ctrl
        self.offMode=1 # SET_POSITION_TARGET_GLOBAL_INT
        if yawType==0:
            self.EnList = [1,0,0,0,0,0]
            self.yaw = 0
            self.yawrate = 0  
        elif yawType==1:
            self.EnList = [1,0,0,0,1,0]
            self.yaw = yawValue
            self.yawrate = 0  
        else:
            self.EnList = [1,0,0,0,0,1]
            self.yaw = 0
            self.yawrate = yawValue  
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_GLOBAL_INT
        self.pos=[lat,lon,alt]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]

    # send target position in earth NED frame
    def SendPosNEDNoYaw(self,x=0,y=0,z=0):
        """ Send vehicle targe position (m) to PX4 in the earth north-east-down (NED) frame without yaw control (rad)
        when the vehicle fly above the ground, then z < 0
        """
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        self.EnList = [1,0,0,0,0,0]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.pos=[x,y,z]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yawrate = 0
        self.yaw = 0

    # send target position in body FRD frame
    def SendPosFRD(self,x=0,y=0,z=0,yaw=0):
        """ Send vehicle targe position (m) to PX4 in the body forward-rightward-downward (FRD) frame with yaw control (rad)
        when the vehicle fly above the ground, then z < 0
        """
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        self.EnList = [1,0,0,0,1,0]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[x,y,z]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yawrate = 0
        self.yaw = yaw

    # send target position in body FRD frame
    def SendPosFRDNoYaw(self,x=0,y=0,z=0):
        """ Send vehicle targe position (m) to PX4 in the body forward-rightward-downward (FRD) frame without yaw control (rad)
        when the vehicle fly above the ground, then z < 0
        """
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        self.EnList = [1,0,0,0,0,0]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[x,y,z]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yawrate = 0
        self.yaw = 0

    def SendPosNEDExt(self,x=0,y=0,z=0,mode=3,isNED=True):
        """ Send vehicle targe position (m) to PX4 
        when the vehicle fly above the ground, then z < 0
        """
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        self.EnList = [1,0,0,0,1,0]
        self.type_mask=self.TypeMask(self.EnList)
        if mode==0:
            # Gliding setpoint
            self.type_mask=int(292) # only for fixed Wing
        elif mode==1:
            # Takeoff setpoints
            self.type_mask=int(4096) # only for fixed Wing
        elif mode==2:
            # Land setpoints
            self.type_mask=int(8192) # only for fixed Wing
        elif mode==3:
            # Loiter setpoints
            # for Rover:  Loiter setpoint (vehicle stops when close enough to setpoint).
            # for fixed wing:  Loiter setpoint (fly a circle centred on setpoint).
            self.type_mask=int(12288)    
        elif mode==4:
            # Idle setpoint 
            # only for fixed wing
            # Idle setpoint (zero throttle, zero roll / pitch).
            self.type_mask=int(16384)   
        if isNED:
            self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        else:
            self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[x,y,z]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yawrate = 0
        self.yaw = 0       
        
    def SendCruiseSpeed(self,Speed=0): 
        """ Send command to change the Cruise speed (m/s) of the aircraft
        """
        #def SendCruiseSpeed(self,Speed,Type=1,Throttle=-1,Relative=0): 
        #  type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed); min:0 max:3 increment:1
        #  Speed (-1 indicates no change); min: -1; Unit: m/s
        #  Throttle (-1 indicates no change); min: -1; Unit: %
        #  Relative	0: absolute, 1: relative; min:0 max:1 increment:1
        #self.SendMavCmdLong(mavlink2.MAV_CMD_DO_CHANGE_SPEED, Type,Speed,Throttle,Relative,0,0,0)
        #self.sendMavSetParam('NAV_LOITER_RAD'.encode(), Speed, mavlink2.MAV_PARAM_TYPE_REAL32)
        self.sendMavSetParam('FW_AIRSPD_TRIM'.encode(), Speed, mavlink2.MAV_PARAM_TYPE_REAL32)

    def SendCopterSpeed(self,Speed=0): 
        """ send command to set the maximum speed of the multicopter
        """
        # 最小3，最大20，默认5
        self.sendMavSetParam('MPC_XY_VEL_MAX'.encode(), Speed, mavlink2.MAV_PARAM_TYPE_REAL32)
        self.MaxSpeed=Speed

    def SendGroundSpeed(self,Speed=0): 
        """ Send command to change the ground speed (m/s) of the aircraft
        """
        self.sendMavSetParam('GND_SPEED_TRIM'.encode(), Speed, mavlink2.MAV_PARAM_TYPE_REAL32)
        self.MaxSpeed=Speed

    def SendCruiseRadius(self,rad=0): 
        """ Send command to change the Cruise Radius (m) of the aircraft
        """
        self.sendMavSetParam('NAV_LOITER_RAD'.encode(), rad, mavlink2.MAV_PARAM_TYPE_REAL32)


    def sendTakeoffMode(self):
        """ Send command to make the aircraft takeoff
        """
        self.SendSetMode(PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO,PX4_CUSTOM_SUB_MODE_AUTO.PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF)

    # initialize Offboard in Pixhawk and start sending data loop in Python
    def initOffboard(self):
        """ Send command to make px4 enter offboard mode, and start to send offboard message 30Hz
        """
        
        # if not self.isPX4Ekf3DFixed:
        #     print('CopterSim still not 3DFxied, please wait and try again.');
        #     sys.exit(0)
        
        self.EnList = [0,1,0,0,0,1]
        self.type_mask = self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.pos=[0,0,0]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yaw=0
        self.yawrate = 0
        self.isInOffboard = True
        if self.UDPMode>1.5:
            self.SendSetMode(PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO,PX4_CUSTOM_SUB_MODE_AUTO.PX4_CUSTOM_SUB_MODE_AUTO_LOITER)
            time.sleep(0.5)
            self.t2.start()
            if not self.hasSendDisableRTLRC:
                self.sendMavSetParam('NAV_RCL_ACT'.encode(),0,mavlink2.MAV_PARAM_TYPE_INT32)
                self.sendMavSetParam('NAV_DLL_ACT'.encode(), 0, mavlink2.MAV_PARAM_TYPE_INT32)
                self.sendMavSetParam('COM_RCL_EXCEPT'.encode(), 4/7.1362e+44, mavlink2.MAV_PARAM_TYPE_INT32)
                #self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_GUIDED_ENABLE, True)
                self.hasSendDisableRTLRC = True
            self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                    self.yawrate)
            time.sleep(0.1)
            self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                    self.yawrate)
            time.sleep(0.1)

            if not self.isArmed:
                self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            time.sleep(0.1)
            self.SendSetMode(PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD)
        else:
            self.t2.start()
            self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                    self.yawrate)
            self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                    self.yawrate)

    def initOffboard2(self):
        """ Send command to make px4 enter offboard mode, and start to send offboard message 30Hz
        """
        self.isInOffboard = True
        self.t2.start()
        self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                self.yawrate)
        self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                self.yawrate)
        if self.UDPMode>1.5:
            if not self.hasSendDisableRTLRC:
                self.sendMavSetParam('NAV_RCL_ACT'.encode(),0,mavlink2.MAV_PARAM_TYPE_INT32)
                self.sendMavSetParam('NAV_DLL_ACT'.encode(), 0, mavlink2.MAV_PARAM_TYPE_INT32)
                self.sendMavSetParam('COM_RCL_EXCEPT'.encode(), 4/7.1362e+44, mavlink2.MAV_PARAM_TYPE_INT32)
                #self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_GUIDED_ENABLE, True)
                self.hasSendDisableRTLRC = True
            if not self.isArmed:
                self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 1)  
            time.sleep(0.5)
            self.SendSetMode(PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD)      

    def sendMavTakeOff(self,xM=0,yM=0,zM=0):
        """ Send command to make aircraft takeoff to the desired local position (m)
        """
        pitchRad=15.0/180*math.pi
        yawRad=0
        lat=self.uavPosGPSHome[0]+xM/6381372/math.pi*180
        lon=self.uavPosGPSHome[1]+yM/6381372/math.pi*180
        alt=self.uavPosGPSHome[2]-zM
        self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_TAKEOFF, pitchRad/math.pi*180,0,0,yawRad/math.pi*180,lat,lon,alt)

    def sendMavTakeOffGPS(self,lat,lon,alt):
        """ Send command to make aircraft takeoff to the desired global position (degree)
        """
        pitchRad=15.0/180*math.pi
        yawRad=0
        self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_TAKEOFF, pitchRad/math.pi*180,0,0,yawRad/math.pi*180,lat,lon,alt)

    def sendMavLand(self,xM,yM,zM):
        """ Send command to make aircraft land to the desired local position (m)
        """
        yawRad=0
        lat=self.uavPosGPSHome[0]+xM/6381372/math.pi*180
        lon=self.uavPosGPSHome[1]+yM/6381372/math.pi*180
        alt=self.uavPosGPSHome[2]-zM
        self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_LAND , 0,0,0,yawRad/math.pi*180,lat,lon,alt)
    
    def sendMavLandGPS(self,lat,lon,alt):
        """ Send command to make aircraft land to the desired global position (degree)
        """
        yawRad=0
        self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_LAND , 0,0,0,yawRad/math.pi*180,lat,lon,alt)


    # stop Offboard mode
    def endOffboard(self):
        """ Send command to px4 to out offboard mode, and stop the message sending loop
        """
        self.isInOffboard = False
        if self.UDPMode>1.5 and self.hasSendDisableRTLRC:
            self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_GUIDED_ENABLE, False)
            self.hasSendDisableRTLRC = False
        self.t2.join()

    # send command pixhawk to modify its parameters
    def sendMavSetParam(self,param_id, param_value, param_type):
        """ Send command to px4 to change desired parameter
        the following mavlink message is adopted
        https://mavlink.io/en/messages/common.html#PARAM_SET
        the parameter list can be found in QGC
        """
        if self.isInPointMode or self.UDPMode<1.5:
            return
        if self.isCom:
            self.the_connection.mav.param_set_send(self.the_connection.target_system,self.the_connection.target_component,param_id,param_value, param_type)
        else:            
            buf = self.mav0.param_set_encode(self.the_connection.target_system,self.the_connection.target_component,param_id,param_value, param_type).pack(self.mav0)
            self.udp_socket.sendto(buf, (self.ip, self.port))

    # send hil_actuator_controls message to Pixhawk (for rfly_ctrl uORB message)
    def SendHILCtrlMsg(self,ctrls):
        """ Send hil_actuator_controls command to PX4, which will be transferred to uORB message rfly_ctrl
        https://mavlink.io/en/messages/common.html#HIL_ACTUATOR_CONTROLS
        """
        time_boot_ms = int((time.time()-self.startTime)*1000)
        controls = [1500,1500,1100,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500]
        for i in range(len(ctrls)):
            if i<len(controls):
                controls[i]=ctrls[i]
        if self.isCom:
            self.the_connection.mav.hil_actuator_controls_send(time_boot_ms,controls,1,1)
        else:
            buf = self.mav0.hil_actuator_controls_encode(time_boot_ms,controls,1,1).pack(self.mav0)
            self.udp_socket.sendto(buf, (self.ip, self.port))
        #print("Msg Send.")

    # send debug_vect message to Pixhawk to update rfly_ctrl uORB message
    def SendHILCtrlMsg1(self):
        """  Send debug_vect command to PX4, which will be transferred to uORB message rfly_ctrl
        https://mavlink.io/en/messages/common.html#DEBUG_VECT
        """
        time_boot_ms = int((time.time()-self.startTime)*1000)
        name = b'hello'
        if self.isCom:
            self.the_connection.mav.debug_vect_send(name, time_boot_ms, 1100, 1500, 1700)
        else:
            buf = self.mav0.debug_vect_encode(name, time_boot_ms, 1100, 1500, 1700).pack(self.mav0)
            self.udp_socket.sendto(buf, (self.ip, self.port))
        #print("Msg1 Send.")

    # send MAVLink command to Pixhawk to Arm/Disarm the drone
    def SendMavArm(self, isArm=0):
        """ Send command to PX4 to arm or disarm the drone
        """
        if (isArm):
            self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        else:
            self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196.0)

    def initRCSendLoop(self, Hz=30):
        self.isRCLoop=True
        self.tRCHz=Hz
        #self.sendMavSetParam('NAV_RCL_ACT'.encode(),0,mavlink2.MAV_PARAM_TYPE_INT32)
        #self.sendMavSetParam('NAV_DLL_ACT'.encode(), 0, mavlink2.MAV_PARAM_TYPE_INT32)
        #time.sleep(0.2)
        self.tRC = threading.Thread(target=self.RcSendLoop, args=())
        self.tRC.start()
    
    def endRCSendLoop(self):
        self.isRCLoop=False
        time.sleep(0.5)
        self.tRC.join()

    def RcSendLoop(self):
        lastTime = time.time()
        while True:
            if not self.isRCLoop:
                break
            lastTime = lastTime + (1.0/self.tRCHz)
            sleepTime = lastTime - time.time()
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                lastTime = time.time()
                
            #print('RC send')
            self.SendRcOverride(self.RCPWMs[0],self.RCPWMs[1],self.RCPWMs[2],self.RCPWMs[3],self.RCPWMs[4],self.RCPWMs[5],self.RCPWMs[6],self.RCPWMs[7])
            self.sendMavManualCtrl((self.RCPWMs[1]-1500)*2,(self.RCPWMs[0]-1500)*2,(self.RCPWMs[2] - 1000),(self.RCPWMs[3]-1500)*2)
        
        
    def SendRCPwms(self, Pwms):
        for i in range(len(Pwms)):
            if i<len(self.RCPWMs):
                self.RCPWMs[i]=Pwms[i]

    # send MAVLink rc_channels_override message to override the RC signals
    def SendRcOverride(self, ch1=1500, ch2=1500, ch3=1100, ch4=1500, ch5=1100, ch6=1100, ch7=1500, ch8=1500):
        """ Send MAVLink command to PX4 to override the RC signal 
        ch1~ch8 range from 1000 to 2000
        https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
        """
        if self.isCom:
            self.the_connection.mav.rc_channels_override_send(self.the_connection.target_system,
                                                        self.the_connection.target_component, ch1, ch2,
                                                        ch3, ch4, ch5, ch6, ch7, ch8)
        else:
            buf = self.mav0.rc_channels_override_encode(self.the_connection.target_system,
                                                        self.the_connection.target_component, ch1, ch2,
                                                        ch3, ch4, ch5, ch6, ch7, ch8).pack(self.mav0)
            self.udp_socket.sendto(buf, (self.ip, self.port))

    # send MAVLink message manual_control to send normalized and calibrated RC sigals to pixhawk
    def sendMavManualCtrl(self, x=0,y=0,z=0,r=0):
        """ Send MAVLink command to PX4 to override the manual control signal 
        x,y,z,r range from -1000 to 1000
        https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
        """
        if self.isInPointMode or self.UDPMode<1.5:
            return
        if self.isCom:
            self.the_connection.mav.manual_control_encode(self.the_connection.target_system, x,y,z,r,0)
        else:
            buf = self.mav0.manual_control_encode(self.the_connection.target_system, x,y,z,r,0).pack(self.mav0)
            self.udp_socket.sendto(buf, (self.ip, self.port))

    # send MAVLink command to change current flight mode
    def SendSetMode(self,mainmode,cusmode=0):
        """ Send MAVLink command to PX4 to change flight mode
        https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE
        """
        basemode = mavlink2.MAV_MODE_FLAG_HIL_ENABLED | mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        self.SendMavCmdLong(mavlink2.MAV_CMD_DO_SET_MODE, basemode, mainmode, cusmode)

    # Stop MAVLink data listening loop
    def stopRun(self):
        """ stop mavlink listening loop for InitMavLoop(), the same as endMavLoop()
        """
        if self.isArmed:
            self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196.0)        
        self.stopFlag=True
        time.sleep(0.5)
        self.t1.join()
        if(self.isInOffboard):
            self.endOffboard()
        if self.UDPMode>1.5:
            self.the_connection.close()
        else:
            if not self.isCom:
                self.udp_socketUDP.close()

    # Update Pixhawk states from MAVLink for 100Hz
    def getTrueDataMsg(self):
        """ Start loop to listen True data from 30100 serial data
        """
        lastTime = time.time()
        while True:
            if self.stopFlagTrueData:
                break
            lastTime = lastTime + 0.01
            sleepTime = lastTime - time.time()
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                lastTime = time.time()
            # print(time.time())
            
            # struct SOut2Simulator {
            #     int copterID;  //Vehicle ID
            #     int vehicleType;  //Vehicle type
            #     double runnedTime; //Current Time stamp (s)
            #     float VelE[3];   //NED vehicle velocity in earth frame (m/s)
            #     float PosE[3];   //NED vehicle position in earth frame (m)
            #     float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
            #     float AngQuatern[4]; //Vehicle attitude in Quaternion
            #     float MotorRPMS[8];  //Motor rotation speed (RPM)
            #     float AccB[3];       //Vehicle acceleration in body frame x y z (m/s/s)
            #     float RateB[3];      //Vehicle angular speed in body frame x y z (rad/s)
            #     double PosGPS[3];    //vehicle longitude, latitude and altitude (degree,degree,m)
                
            # }
            # typedef struct _netDataShort {
            #     int tg;
            #     int        len;
            #     char       payload[192];
            # }netDataShort;
            while True:
                if self.stopFlagTrueData:
                    break

                try:
                    buf,addr = self.udp_socketTrue.recvfrom(65500)
                    if len(buf)==192+8:
                        #print(len(buf[0:8]))
                        tg,strLen = struct.unpack('ii',buf[0:8])
                        if strLen==152:
                            UIV=struct.unpack('2i1d27f3d',buf[8:8+152])
                            #print(self.uavGlobalPos[2])
                            self.trueTimeStmp = UIV[2]
                            self.trueAngEular[0]=UIV[9]
                            self.trueAngEular[1]=UIV[10]
                            self.trueAngEular[2]=UIV[11]
                            self.truePosNED[0]=UIV[6]
                            self.truePosNED[1]=UIV[7]
                            self.truePosNED[2]=UIV[8]
                            self.trueVelNED[0]=UIV[3]
                            self.trueVelNED[1]=UIV[4]
                            self.trueVelNED[2]=UIV[5]
                            self.trueAngRate[0]=UIV[27]
                            self.trueAngRate[1]=UIV[28]
                            self.trueAngRate[2]=UIV[29]
                            self.trueAngQuatern[0]=UIV[12]
                            self.trueAngQuatern[1]=UIV[13]
                            self.trueAngQuatern[2]=UIV[14]
                            self.trueAngQuatern[3]=UIV[15]
                            self.trueMotorRPMS[0]=UIV[16]
                            self.trueMotorRPMS[1]=UIV[17]
                            self.trueMotorRPMS[2]=UIV[18]
                            self.trueMotorRPMS[3]=UIV[19]
                            self.trueMotorRPMS[4]=UIV[20]
                            self.trueMotorRPMS[5]=UIV[21]
                            self.trueMotorRPMS[6]=UIV[22]
                            self.trueMotorRPMS[7]=UIV[23]
                            self.trueAccB[0]=UIV[24]
                            self.trueAccB[1]=UIV[25]
                            self.trueAccB[2]=UIV[26]
                            self.truePosGPS[0]=UIV[30]
                            self.truePosGPS[1]=UIV[31]
                            self.truePosGPS[2]=UIV[32]
                            if not self.hasTrueDataRec:
                                self.hasTrueDataRec=True
                        if tg>-0.5:
                            self.isVehicleCrash=True
                            self.isVehicleCrashID=tg
                    # struct Ue4CMDGPS{
                    #     int checksum;//校验码，用于确认数据正确性，这里取值 1234567890
                    #     int CopterID;
                    #     double GPS[3];
                    # } ii3d
                    if len(buf)==32:
                        UIV=struct.unpack('ii3d',buf)
                        checksum=UIV[0]
                        CopterID=UIV[1]
                        if checksum==1234567890:
                            self.trueGpsUeCenter[0]=UIV[2]
                            self.trueGpsUeCenter[1]=UIV[3]
                            self.trueGpsUeCenter[2]=UIV[4]                 
                        
                    if len(buf)==12:
                        checksum,CopterID,targetID = struct.unpack('iii',buf[0:12])
                        if checksum==1234567890:
                            if targetID>-0.5:
                                self.isVehicleCrash=True
                                self.isVehicleCrashID=targetID
                    # struct outCopterStruct{
                    #     int checksum; //1234567890
                    #     int CopterID;
                    #     double data[32]; //data
                    # } ii32d  4+4+8*32      
                    if len(buf)==264:
                        UIV=struct.unpack('ii32d',buf)
                        checksum=UIV[0]
                        CopterID=UIV[1]
                        if checksum==1234567890:
                            self.trueSimulinkData=UIV[2:32]
                except:
                    self.stopFlagTrueData=True
                    break
            
            
            

    # Update Pixhawk states from MAVLink for 100Hz
    def getPX4DataMsg(self):
        """ Start loop to listen True data from 40100 serial data
        """
        lastTime = time.time()
        while True:
            if self.stopFlagPX4Data:
                break
            lastTime = lastTime + 0.01
            sleepTime = lastTime - time.time()
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                lastTime = time.time()
            # print(time.time())
            
            # struct PX4ExtMsg {
            #     int checksum;  //1234567898
            #     int CopterID;
            #     double runnedTime; //Current  stamp (s)
            #     float controls[8];
            # }iid8f
            while True:
                if self.stopFlagPX4Data:
                    break

                try:
                    buf,addr = self.udp_socketPX4.recvfrom(65500)
                    if len(buf)==48:
                        #print('Data Receved!')
                        UIV = struct.unpack('iid8f',buf)
                        checksum=UIV[0]
                        if checksum==1234567898:
                            #print('Data Receved!')
                            self.px4ext.checksum=checksum
                            self.px4ext.CopterID=UIV[1]
                            self.px4ext.runnedTime=UIV[2]
                            self.px4ext.controls=UIV[3:11]
                            self.px4extTrue=True
                except:
                    self.stopFlagPX4Data=True
                    break
            
    def StartTimeStmplisten(self,cpID=0):
        """Start to listen to 20005 port to get RflyTimeStmp of CopterID
        if cpID == 0 then only current CopterID will be listened.
        if cpID >0 then timestmp of desired CopterID will be listened.
        """
        self.udp_time = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建套接字
        self.udp_time.bind(('0.0.0.0',20005))
        self.tTimeStmp = threading.Thread(target=self.TimeStmploop, args=())
        self.cpID=cpID
        if cpID==0:
            self.cpID=self.CopterID
        self.tTimeStmp.start()
    
    def TimeStmploop(self):
        print("Start lisening to timeStmp Msg")
        while True:
            try:
                buf,addr = self.udp_time.recvfrom(65500)
                if len(buf)==32:
                    #print(len(buf[0:12]))
                    TimeData=struct.unpack('2i3q',buf)                   
                    if TimeData[0]==123456789:
                        
                        cpIDTmp = TimeData[1]
                        if cpIDTmp == self.cpID:
                            self.RflyTime.checksum=TimeData[0]
                            self.RflyTime.copterID=TimeData[1]
                            self.RflyTime.SysStartTime=TimeData[2]
                            self.RflyTime.SysCurrentTime=TimeData[3]
                            self.RflyTime.HeartCount=TimeData[4]
     
            except:
                print("Error to listen to Time Msg!")
                sys.exit(0)

            
    # Update Pixhawk states from MAVLink for 100Hz
    def getMavMsg(self):
        """ Start loop to listen mavlink data from 20100 series port or COM port
        """
        lastTime = time.time()
        while True:
            if self.stopFlag:
                break
            lastTime = lastTime + 0.01
            sleepTime = lastTime - time.time()
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                lastTime = time.time()
            # print(time.time())
            while True:
                if self.stopFlag:
                    break
                
                if self.UDPMode>1.5: # If Use MAVLink Mode
                    msg = self.the_connection.recv_match(
                        type=['ATTITUDE', 'LOCAL_POSITION_NED','HEARTBEAT','HOME_POSITION','GLOBAL_POSITION_INT','HIL_ACTUATOR_CONTROLS','STATUSTEXT','ATTITUDE_TARGET'],
                        blocking=False)
                    if msg is not None:
                        if msg.get_type() == "ATTITUDE":
                            self.uavTimeStmp = msg.time_boot_ms
                            self.uavAngEular[0] = msg.roll
                            self.uavAngEular[1] = msg.pitch
                            self.uavAngEular[2] = msg.yaw
                            self.uavAngRate[0] = msg.rollspeed
                            self.uavAngRate[1] = msg.pitchspeed
                            self.uavAngRate[2] = msg.yawspeed
                        if msg.get_type() == "LOCAL_POSITION_NED":
                            self.uavPosNED[0] = msg.x
                            self.uavPosNED[1] = msg.y
                            self.uavPosNED[2] = msg.z
                            self.uavVelNED[0] = msg.vx
                            self.uavVelNED[1] = msg.vy
                            self.uavVelNED[2] = msg.vz
                            if not (abs(self.uavPosGPSHome[0])<1 and abs(self.uavPosGPSHome[1])<1):
                                self.uavGlobalPos[0]=(self.uavPosGPSHome[0]-self.trueGpsUeCenter[0])/180*math.pi*6362000+self.uavPosNED[0]
                                self.uavGlobalPos[1]=(self.uavPosGPSHome[1]-self.trueGpsUeCenter[1])/180*math.pi*4.8823e6+self.uavPosNED[1]
                                self.uavGlobalPos[2]=-(self.uavPosGPSHome[2]-self.trueGpsUeCenter[2])+self.uavPosNED[2]

                        if msg.get_type() == "HOME_POSITION":
                            self.uavPosGPSHome[0] = msg.latitude/1e7
                            self.uavPosGPSHome[1] = msg.longitude/1e7
                            self.uavPosGPSHome[2] = msg.altitude/1e3

                        if msg.get_type() == "GLOBAL_POSITION_INT":
                            self.uavPosGPS[0] = msg.lat/1e7
                            self.uavPosGPS[1] = msg.lon/1e7
                            self.uavPosGPS[2] = msg.alt/1e3

                        if msg.get_type() == "HEARTBEAT":
                            isArmed = msg.base_mode & mavlink2.MAV_MODE_FLAG_SAFETY_ARMED
                            if not self.isArmed and isArmed:
                                print("PX4 Armed!")
                            if self.isArmed and not isArmed:
                                print("PX4 DisArmed!")
                            self.isArmed = isArmed
                            #print("HeartBeat!")

                        if msg.get_type() == "HIL_ACTUATOR_CONTROLS":
                            if msg.flags == 1234567890:
                                if not self.hasTrueDataRec:
                                    if msg.mode == 2 and msg.controls[15]>-0.5:
                                        self.isVehicleCrash = True
                                        self.isVehicleCrashID=int(msg.controls[15])
                                    self.trueTimeStmp = msg.time_usec/1000.0
                                    self.trueAngEular[0]=msg.controls[0]
                                    self.trueAngEular[1]=msg.controls[1]
                                    self.trueAngEular[2]=msg.controls[2]
                                    self.truePosNED[0]=msg.controls[3]
                                    self.truePosNED[1]=msg.controls[4]
                                    self.truePosNED[2]=msg.controls[5]
                                    self.trueVelNED[0]=msg.controls[6]
                                    self.trueVelNED[1]=msg.controls[7]
                                    self.trueVelNED[2]=msg.controls[8]
                                    self.trueAngRate[0]=msg.controls[9]
                                    self.trueAngRate[1]=msg.controls[10]
                                    self.trueAngRate[2]=msg.controls[11]
                                if not self.isPX4Ekf3DFixed and msg.mode != 2 and msg.controls[15]>0.5:
                                    self.isPX4Ekf3DFixed=True
                        if msg.get_type() == "STATUSTEXT":
                            #print(msg.text)
                            if msg.text.find('Failsafe')!= -1:
                                self.isFailsafeEn=True
                                print(msg.text)
                        
                        if msg.get_type() == "ATTITUDE_TARGET":
                            self.uavThrust = msg.thrust
                            #print(msg.thrust)
                    else:
                        break
                else: 
                    if self.UDPMode==0:
                        # if use UDP Mode
                        #II3i3i3iiiiii3f3f3ffff
                        # struct outHILStateData{ // mavlink data forward from Pixhawk
                        #     uint32_t time_boot_ms; //Timestamp of the message
                        #     uint32_t copterID;     //Copter ID start from 1
                        #     int32_t GpsPos[3];     //Estimated GPS position，lat&long: deg*1e7, alt: m*1e3 and up is positive
                        #     int32_t GpsVel[3];     //Estimated GPS velocity, NED, m/s*1e2->cm/s
                        #     int32_t gpsHome[3];     //Home GPS position, lat&long: deg*1e7, alt: m*1e3 and up is positive
                        #     int32_t relative_alt;  //alt: m*1e3 and up is positive
                        #     int32_t hdg;           //Course angle, NED,deg*1000, 0~360
                        #     int32_t satellites_visible; //GPS Raw data, sum of satellite
                        #     int32_t fix_type;     //GPS Raw data, Fixed type, 3 for fixed (good precision)
                        #     int32_t resrveInit;       //Int, reserve for the future use
                        #     float AngEular[3];    //Estimated Euler angle, unit: rad/s
                        #     float localPos[3];    //Estimated locoal position, NED, unit: m
                        #     float localVel[3];    //Estimated locoal velocity, NED, unit: m/s
                        #     float pos_horiz_accuracy;   //GPS horizontal accuracy, unit: m
                        #     float pos_vert_accuracy; //GPS vertical accuracy, unit: m
                        #     float resrveFloat;      //float,reserve for the future use
                        # }
                        # typedef struct _netDataShortShort {
                        #     TargetType tg;
                        #     int        len;
                        #     char       payload[112];
                        # }netDataShortShort;
                        try:
                            buf,addr = self.udp_socketUDP.recvfrom(65500)
                            if len(buf)==112+8:
                                #print(len(buf[0:8]))
                                tg,strLen = struct.unpack('ii',buf[0:8])
                                if strLen==112:
                                    UIV=struct.unpack('2I14i12f',buf[8:120])
                                    #GpsPos[0]=
                                    #time_boot_ms,copterID,GpsPos,GpsVel,gpsHome,relative_alt,hdg,satellites_visible,fix_type,resrveInit,AngEular,localPos,localVel,pos_horiz_accuracy,pos_vert_accuracy,resrveFloat
                                    self.uavTimeStmp = UIV[0]
                                    for idx in range(3):
                                        self.uavAngEular[idx]=UIV[16+idx]
                                        self.uavPosNED[idx]=UIV[19+idx]
                                        self.uavVelNED[idx]=UIV[22+idx]
                                    self.uavPosGPS[0] = UIV[2]/1e7
                                    self.uavPosGPS[1] = UIV[3]/1e7
                                    self.uavPosGPS[2] = UIV[4]/1e3
                                    self.uavPosGPSHome[0] = UIV[8]/1e7
                                    self.uavPosGPSHome[1] = UIV[9]/1e7
                                    self.uavPosGPSHome[2] = UIV[10]/1e3
                                    if not (abs(self.uavPosGPSHome[0])<1 and abs(self.uavPosGPSHome[1])<1):
                                        self.uavGlobalPos[0]=(self.uavPosGPSHome[0]-self.trueGpsUeCenter[0])/180*math.pi*6362000+self.uavPosNED[0]
                                        self.uavGlobalPos[1]=(self.uavPosGPSHome[1]-self.trueGpsUeCenter[1])/180*math.pi*4.8823e6+self.uavPosNED[1]
                                        self.uavGlobalPos[2]=-(self.uavPosGPSHome[2]-self.trueGpsUeCenter[2])+self.uavPosNED[2]
                                    if not self.isPX4Ekf3DFixed and UIV[27]>0.5:
                                        self.isPX4Ekf3DFixed=True
                                    #print(self.uavGlobalPos[2])
                                    if tg>-0.5:
                                        self.isVehicleCrash=True
                                        self.isVehicleCrashID=tg
                                    
                        except:
                            self.stopFlag=True
                            break
                    else:
                        try:
                            buf,addr = self.udp_socketUDP.recvfrom(65500)
                            if len(buf)==52:
                                #print(len(buf[0:8]))
                                UIV=struct.unpack('4i9f',buf)
                                checksum=UIV[0]
                                #GpsPos[0]=
                                #time_boot_ms,copterID,GpsPos,GpsVel,gpsHome,relative_alt,hdg,satellites_visible,fix_type,resrveInit,AngEular,localPos,localVel,pos_horiz_accuracy,pos_vert_accuracy,resrveFloat
                                if checksum==1234567890:
                                    self.uavTimeStmp =time.time()
                                    for idx in range(3):
                                        self.uavAngEular[idx]=UIV[4+idx]
                                        self.uavPosNED[idx]=UIV[7+idx]
                                        self.uavVelNED[idx]=UIV[10+idx]
                                    self.uavPosGPSHome[0] = UIV[1]/1e7
                                    self.uavPosGPSHome[1] = UIV[2]/1e7
                                    self.uavPosGPSHome[2] = UIV[3]/1e3
                                    if not (abs(self.uavPosGPSHome[0])<1 and abs(self.uavPosGPSHome[1])<1):
                                        self.uavGlobalPos[0]=(self.uavPosGPSHome[0]-self.trueGpsUeCenter[0])/180*math.pi*6362000+self.uavPosNED[0]
                                        self.uavGlobalPos[1]=(self.uavPosGPSHome[1]-self.trueGpsUeCenter[1])/180*math.pi*4.8823e6+self.uavPosNED[1]
                                        self.uavGlobalPos[2]=-(self.uavPosGPSHome[2]-self.trueGpsUeCenter[2])+self.uavPosNED[2]
                                    #print(self.uavGlobalPos[2])
                                elif checksum==1234567891:
                                    if not self.isPX4Ekf3DFixed:
                                        self.isPX4Ekf3DFixed=True
                                elif checksum==1234567892 and UIV[2]>-0.5:
                                    self.isVehicleCrash=True
                                    self.isVehicleCrashID=UIV[2]
                                    
                        except:
                            self.stopFlag=True
                            break
        #print("Mavlink Stoped.")

    # Offboard message sending loop, 100Hz
    def OffboardSendMode(self):
        lastTime2 = time.time()
        
        interTime=0.01
        # If in Simple mode, we use 30Hz to reduce network load 
        if self.UDPMode==1 or self.UDPMode==3 or self.UDPMode==4:
            interTime=1/30.0
        
        while True:
            if not self.isInOffboard:
                break
            lastTime2 = lastTime2 + interTime
            sleepTime = lastTime2 - time.time()
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                lastTime2 = time.time()
            if self.isInOffboard:
                self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw, self.yawrate)
        #print("Offboard Stoped.")



    def sendUE4PosScale100(self,copterID,vehicleType,PosE,AngEuler,MotorRPMSMean,Scale,isFitGround=False,windowID=-1):
        """send the position & angle information to RflySim3D to create 100 vehicles once
            #struct Multi3DData100New {
            #    int checksum;
            #    uint16 copterID[100];
            #    uint16 vehicleType[100];
            #    float PosE[300];
            #    float AngEuler[300];
            #    uint16 Scale[300];
            #    float MotorRPMSMean[100];
            #}
            #checksum是数据校验位，1234567890表示正常数据，1234567891表示飞机始终贴合地面
            #copterID是100维整型数组，飞机ID，这个从1给到100即可,如果ID给0则不显示次飞机
            #vehicleType是100维整型数组，飞机类型，四旋翼给3即可
            #MotorRPMSMean是100维数组，飞机螺旋桨转速，单位RPM，四旋翼默认给1000即可
            #PosE是300维数组，飞机位置，可以随机生成，单位是m
            #AngEuler是300维数组，飞机姿态角，单位弧度，默认都给0，表示正常放置
            #Scale是300维的数组，数据是xyz显示的尺寸*100，默认都给100表示实际大小即1倍
        """
        
        checksum = 1234567890
        if isFitGround:
            checksum = 1234567891
        buf = struct.pack("i200H600f300H100f",checksum,*copterID,*vehicleType,*PosE,*AngEuler,*Scale,*MotorRPMSMean)
        if windowID<0:
            if self.ip=='127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009)) #multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            if self.ip!='127.0.0.1' and self.ip!='255.255.255.255':
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID)) #ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID)) #specify PC's IP to send


    def sendUE4PosScalePwm20(self,copterID,vehicleType,PosE,AngEuler,Scale,PWMs,isFitGround=False,windowID=-1):
        """send the position & angle information to RflySim3D to create 20 vehicles once
            # struct Multi3DData2New {
            #   int checksum;
            # 	uint16 copterID[20];
            # 	uint16 vehicleType[20];
            # 	float PosE[60];
            # 	float AngEuler[60];
            # 	uint16 Scale[60];
            # 	float PWMs[160];
            # }
            #checksum是数据校验位，1234567890表示正常数据，1234567891表示飞机始终贴合地面
            #copterID是20维整型数组，飞机ID，这个从1给到20即可,如果ID给0则不显示次飞机
            #vehicleType是20维整型数组，飞机类型，四旋翼给3即可
            #PosE是60维数组，飞机位置，可以随机生成，单位是m
            #AngEuler是60维数组，飞机姿态角，单位弧度，默认都给0，表示正常放置
            #Scale是300维的数组，数据是xyz显示的尺寸*100，默认都给100表示实际大小即1倍
            #PWMs是160维数组，对应20个飞机各8个旋翼转速，单位RPM，四旋翼默认给1000即可
        """
        checksum = 1234567890
        if isFitGround:
            checksum = 1234567891
        buf = struct.pack("i40H120f60H160f",checksum,*copterID,*vehicleType,*PosE,*AngEuler,*Scale,*PWMs)
        if windowID<0:
            if self.ip=='127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009)) #multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            if self.ip!='127.0.0.1' and self.ip!='255.255.255.255':
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID)) #ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID)) #specify PC's IP to send
            
            
    def sendRebootPix(self,copterID,delay=-1):
        """ Send message to restart simulation to CopterID
        """
        checksum = 987654321 #checksum for reboot message
        buf = struct.pack("2i",checksum,delay)
        self.udp_socket.sendto(buf, ('255.255.255.255', 20100+copterID*2-2))

    # send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
    def sendUE4PosFull(self,copterID,vehicleType,MotorRPMS,VelE,PosE,RateB,AngEuler,windowID=-1):
        """send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
            # struct SOut2Simulator {
            #     int copterID;  //Vehicle ID
            #     int vehicleType;  //Vehicle type
            #     double runnedTime; //Current Time stamp (s)
            #     float VelE[3];   //NED vehicle velocity in earth frame (m/s)
            #     float PosE[3];   //NED vehicle position in earth frame (m)
            #     float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
            #     float AngQuatern[4]; //Vehicle attitude in Quaternion
            #     float MotorRPMS[8];  //Motor rotation speed (RPM)
            #     float AccB[3];       //Vehicle acceleration in body frame x y z (m/s/s)
            #     float RateB[3];      //Vehicle angular speed in body frame x y z (rad/s)
            #     double PosGPS[3];    //vehicle longitude, latitude and altitude (degree,degree,m)
                
            # }
            # typedef struct _netDataShort {
            #     int tg;
            #     int        len;
            #     char       payload[192];
            # }netDataShort;
        
        """
        runnedTime = time.time()-self.startTime
        #VelE=[0,0,0]
        AngQuatern=[0,0,0,0]
        AccB=[0,0,0]
        #RateB=[0,0,0]
        PosGPS=PosE
        # buf for SOut2Simulator, len=152
        buf0 = struct.pack("2i1d27f3d",copterID,vehicleType,runnedTime,*VelE,*PosE,*AngEuler,*AngQuatern,*MotorRPMS,*AccB,*RateB,*PosGPS)
        # buf for remaining 192-152=40bytes of payload[192] of netDataShort
        buf1 = bytes([0]*(192-len(buf0)))
        # buf for tg and len in netDataShort
        buf2 = struct.pack("2i",2,len(buf0))
        # buf for netDataShort
        buf=buf2+buf0+buf1
        #print(len(buf))
        if windowID<0:
            if self.ip=='127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009)) #multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            if self.ip!='127.0.0.1' and self.ip!='255.255.255.255':
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID)) #ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID)) #specify PC's IP to send
        #print('Message Send')
    

    def sendUE4ExtAct(self,copterID=1,ActExt=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],windowID=-1):
        # struct Ue4ExtMsg {
        #     int checksum;//1234567894
        #     int CopterID;
        #     double runnedTime; //Current  stamp (s)
        #     float ExtToUE4[16];
        # }
        #struct.pack 2i1d16f
        runnedTime = time.time()-self.startTime
        checkSum=1234567894
        buf = struct.pack("2i1d16f",checkSum,copterID,runnedTime,*ActExt)
        if windowID<0:
            if self.ip=='127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009)) #multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            if self.ip!='127.0.0.1' and self.ip!='255.255.255.255':
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID)) #ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID)) #specify PC's IP to send
        #print('Message Send')
    
    
    def sendUE4PosSimple(self,copterID,vehicleType,PWMs,VelE,PosE,AngEuler,runnedTime=-1,windowID=-1):
        """ send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
            # //输出到模拟器的数据
            # struct SOut2SimulatorSimpleTime {
            #     int checkSum;
            #     int copterID;  //Vehicle ID
            #     int vehicleType;  //Vehicle type
            #     float PWMs[8];
            #     float PosE[3];   //NED vehicle position in earth frame (m)
            #     float VelE[3];
            #     float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
            #     double runnedTime; //Current Time stamp (s)
            # };
            #struct.pack 3i17f1d
        """
        if runnedTime<0:
            runnedTime = time.time()-self.startTime
        checkSum=1234567890
        # pack for SOut2SimulatorSimpleTime
        buf = struct.pack("3i17f1d",checkSum,copterID,vehicleType,*PWMs,*PosE,*VelE,*AngEuler,runnedTime)
        #print(len(buf))
        if windowID<0:
            if self.ip=='127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009)) #multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            if self.ip!='127.0.0.1' and self.ip!='255.255.255.255':
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID)) #ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID)) #specify PC's IP to send
        #print('Message Send')
    
    def sendUE4PosNew(self,copterID=1,vehicleType=3,PosE=[0,0,0],AngEuler=[0,0,0],VelE=[0,0,0],PWMs=[0]*8,runnedTime=-1,windowID=-1):
        """ send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
            # //输出到模拟器的数据
            # struct SOut2SimulatorSimpleTime {
            #     int checkSum; //1234567890
            #     int copterID;  //Vehicle ID
            #     int vehicleType;  //Vehicle type
            #     float PWMs[8];
            #     float VelE[3];
            #     float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
            #     double PosE[3];   //NED vehicle position in earth frame (m)
            #     double runnedTime; //Current Time stamp (s)
            # };
            #struct.pack 3i14f4d
        """
        if runnedTime<0:
            runnedTime = time.time()-self.startTime
        checkSum=1234567890
        # pack for SOut2SimulatorSimpleTime
        buf = struct.pack("3i14f4d",checkSum,copterID,vehicleType,*PWMs,*VelE,*AngEuler,*PosE,runnedTime)
        #print(len(buf))
        if windowID<0:
            if self.ip=='127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009)) #multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            if self.ip!='127.0.0.1' and self.ip!='255.255.255.255':
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID)) #ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID)) #specify PC's IP to send
        #print('Message Send')

    # //输出到CopterSim DLL模型的SILints和SILFloats数据
    # struct PX4SILIntFloat{
    #     int checksum;//1234567897
    #     int CopterID;
    #     int inSILInts[8];
    #     float inSILFLoats[20];
    # };
    #struct.pack 10i20f
    
    # inSILInts and inSILFLoats will send to DLL model's inputs
    # CopterID is the vehicle you want to send, if copterID then it will send to yourself.
    def sendSILIntFloat(self,inSILInts,inSILFLoats,copterID=-1):
        checkSum=1234567897
        ID=copterID;
        if copterID<=0:
            ID=self.CopterID
        PortNum = 30100+(ID-1)*2 
        # pack for SOut2SimulatorSimpleTime
        buf = struct.pack("10i20f",checkSum,ID,*inSILInts,*inSILFLoats)
        self.udp_socket.sendto(buf, (self.ip, PortNum))
        
        
    # //输出到CopterSim DLL模型的ModelInParams参数
    # struct PX4ModelInParams{
    #     int checksum;//1234567891
    #     int Bitmask;
    #     double InParams[32];
    # };
    #struct.pack 2i32d
    
    # inSILInts and inSILFLoats will send to DLL model's inputs
    # CopterID is the vehicle you want to send, if copterID then it will send to yourself.
    # bitmask has 32bits, each bit == 1 means modify the params in ModelInParams
    def sendModelInParams(self,Bitmask,InParams,copterID=-1):
        checkSum=1234567891
        ID=copterID;
        if copterID<=0:
            ID=self.CopterID
        PortNum = 30100+(ID-1)*2 
        # pack for SOut2SimulatorSimpleTime
        buf = struct.pack("2i32d",checkSum,Bitmask,*InParams)
        self.udp_socket.sendto(buf, (self.ip, PortNum))
        
        
    def getUE4Pos(self,CopterID=1):
        if self.stopFlagUE4: #如果没有启用监听程序
            self.initUE4MsgRec()
            time.sleep(1)
            
        for i in range(len(self.inReqVect)): #遍历数据列表，飞机ID有没有出现过
            if self.inReqVect[i].copterID == CopterID:
                posE = self.inReqVect[i].PosE
                return [posE[0],posE[1],posE[2],1]
        return [0,0,0,0]
    
        
    def getUE4Data(self,CopterID=1):
        if self.stopFlagUE4: #如果没有启用监听程序
            self.initUE4MsgRec()
            time.sleep(1)
            
        for i in range(len(self.inReqVect)): #遍历数据列表，飞机ID有没有出现过
            if self.inReqVect[i].copterID == CopterID:
                return self.inReqVect[i]
        return 0