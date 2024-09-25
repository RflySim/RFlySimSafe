import time
import numpy as np
import math

try:
    import include.UE4CtrlAPI as UE4CtrlAPI
except ImportError:
    import UE4CtrlAPI

ue = UE4CtrlAPI.UE4CtrlAPI()

class Sleep:
    def __init__(self,mav):
        self.CID = 1
        self.mav = mav
        self.isDone = 0 
        self.WaitFlag = 0
        self.WaitResetFlag = 0
        self.start_time = 0
    
    def Wait(self,times): 
        self.isDone = 0
        if self.WaitFlag == 0:
            print('wait {}s'.format(times[0]))
            self.start_time = time.time() + times[0]
            self.WaitFlag = 1
    
        if self.start_time - time.time() < 0: 
            self.isDone = 1
            self.WaitFlag = 0
    
    def WaitReset(self,targetPos): 
        self.isDone = 0
        curPos=self.mav.uavPosNED
        if self.WaitResetFlag == 0:
            print('wait reset')
            ue.sendUE4Cmd('RflyShowTextTime "Wait Reset:%.2f %.2f %.2f " 10'%(targetPos[0],targetPos[1],targetPos[2]))
            self.WaitResetFlag = 1

        dis = math.sqrt((curPos[0]-targetPos[0])**2+(curPos[1]-targetPos[1])**2)
        if dis < 5:
            print('Arrive at the destination')
            ue.sendUE4Cmd('RflyShowTextTime "Arrive at the destination" 10')
            self.isDone = 1
            self.WaitResetFlag = 0

class Command:
    def __init__(self,mav):
        self.CID = 2
        self.mav = mav
        self.ARMFLAG = False 
        self.isDone = 0 
        self.RECORDFLAG = False
        self.LANDFLAG = False
        self.LANDFLAGTAG = False
        self.silInt = np.zeros(8).astype(int).tolist()
        self.silFloats = np.zeros(20).astype(float).tolist()
        self.INJECTFLAG = False
        self.FAULTID = 0
        self.isInitOff = 0

    def Arm(self): 
        self.isDone = 0
        self.mav.SendMavArm(1)
        print('Armed')
        ue.sendUE4Cmd('RflyShowTextTime "Armed" 10')
        self.ARMFLAG = True
        self.isDone = 1
        self.RECORDFLAG = True

    def DisArm(self): 
        self.isDone = 0
        self.mav.SendMavArm(0) 
        print('DisArmed') 
        ue.sendUE4Cmd('RflyShowTextTime "DisArmed" 10')
        self.isDone = 1

    def QuadPos(self,pos):
        self.isDone = 0
        # self.mav.SendMavArm(1)
        self.mav.SendPosNED(pos[0],pos[1],pos[2])
        print('Send Pos {}'.format(pos))
        ue.sendUE4Cmd('RflyShowTextTime "Send Pos Cmd:%.2f %.2f %.2f" 10'%(pos[0],pos[1],pos[2]))
        self.isDone = 1

    def FixWingPos(self,pos): 
        self.isDone = 0
        if self.isInitOff == 0:
            self.mav.initOffboard()
            self.isInitOff = 1
        print('start init Offboard mode')
        self.mav.SendPosNED(pos[0],pos[1],pos[2])
        print('Send Pos {}'.format(pos))
        ue.sendUE4Cmd('RflyShowTextTime "Send Pos Cmd:%.2f %.2f %.2f" 10'%(pos[0],pos[1],pos[2]))
        ue.sendUE4Cmd('RflyShowTextTime "Start Init Offboard Mode" 10')
        self.isDone = 1
    
    def USVPos(self,pos): 
        self.isDone = 0
        self.mav.SendPosNED(pos[0],pos[1],pos[2])
        print('Send Pos {}'.format(pos))
        ue.sendUE4Cmd('RflyShowTextTime "Send Pos Cmd:%.2f %.2f %.2f" 10'%(pos[0],pos[1],pos[2]))
        self.isDone = 1

    def QuadVel(self,vel): 
        self.isDone = 0
        self.mav.SendVelNED(vel[0],vel[1],vel[2])
        print('Send Vel {}'.format(vel))
        ue.sendUE4Cmd('RflyShowTextTime "Send Vel Cmd:%.2f %.2f %.2f" 10'%(vel[0],vel[1],vel[2]))
        self.isDone = 1

    def USVVel(self,vel): 
        self.isDone = 0
        self.mav.SendVelNEDNoYaw(vel[0],vel[1],vel[2])
        print('Send vel {}'.format(vel))
        ue.sendUE4Cmd('RflyShowText Send Speed Command')
        ue.sendUE4Cmd('RflyShowTextTime "Send Vel Cmd:%.2f %.2f %.2f" 10'%(vel[0],vel[1],vel[2]))
        self.isDone = 1
    
    def USVGroundSpeed(self,vel): 
        self.isDone = 0
        self.mav.SendGroundSpeed(vel[0])
        print('Set GroundSpeed: {}'.format(vel[0]))
        ue.sendUE4Cmd('RflyShowTextTime "Set GroundSpeed:%.2f " 10'%(vel[0]))
        self.isDone = 1

    def UAVLand(self,pos): 
        self.isDone = 0
        self.LANDFLAG = True
        if self.LANDFLAGTAG == False:
            self.mav.sendMavLand(pos[0],pos[1],pos[2])
            print('Start Landing')
            ue.sendUE4Cmd('RflyShowTextTime "Start Landing:%.2f %.2f %.2f" 10'%(pos[0],pos[1],pos[2]))
            self.LANDFLAGTAG = True
        if abs(self.mav.truePosNED[2]) < 1.5:
            print('Landed')
            self.isDone = 1
            self.LANDFLAGTAG = False

    def FixWingTakeOff(self,targetpos): 
        self.isDone = 0
        self.mav.sendMavTakeOff(targetpos[0],targetpos[1],targetpos[2])
        ue.sendUE4Cmd('RflyShowTextTime "TakeOff cmd:%.2f %.2f %.2f" 10'%(targetpos[0],targetpos[1],targetpos[2]))
        print('Start TakeOff')
        ue.sendUE4Cmd('RflySetActuatorPWMs 1 500')
        self.isDone = 1

    def FixWingSetCruiseRadius(self,radius): 
        self.isDone = 0
        self.mav.SendCruiseRadius(radius[0])
        print('CruiseRadius is {}'.format(radius[0]))
        ue.sendUE4Cmd('RflyShowTextTime "Set CruiseRadius:%.2f" 10'%(radius[0]))
        self.isDone = 1
        
    def FaultInject(self,param): 
        self.isDone = 0
        self.inInts = np.array([])
        self.inFloats = np.array([])
        for i in range(len(param)):
            if param[i] >= 123450:
                self.inInts = np.append(self.inInts,param[i])
            else:
                self.inFloats = np.append(self.inFloats,param[i])
        
        for i in range(len(self.inInts)):
            self.silInt[i] = self.inInts[i].astype(int)
        for i in range(len(self.inFloats)):
            self.silFloats[i] = self.inFloats[i].astype(np.double)

        if self.silInt[0] == 123450 or self.silInt[0] == 123451:
            ue.sendUE4Cmd('RflySetActuatorPWMsExt 1 1')

        print('Start Inject Fault')
        ue.sendUE4Cmd('RflyShowTextTime "Fault Params: %.2f %.2f %.2f" 10'%(self.silFloats[0],self.silFloats[1],self.silFloats[2]))
        ue.sendUE4Cmd('RflyShowTextTime "Start Inject %d Fault" 10'%(self.silInt[0]))
        self.mav.sendSILIntFloat(self.silInt,self.silFloats)
        self.mav.SendMavCmdLong(183,999,999,999,999,999,999,999)
        self.FAULTID = self.silInt[0]
        self.isDone = 1
        self.INJECTFLAG = True

class CmdCtrl:
    def __init__(self,mav,frame):
        self.mav = mav
        self.frame = frame
        self.CID = {
        '1':Sleep(mav),
        '2':Command(mav)
        }
        self.CID1 = self.CID['1']
        self.CID2 = self.CID['2']
        self.FID = 0
    
    def GetWaitseq(self):
        Waitseq = {
                '1':self.CID1.Wait,
                '2':self.CID1.WaitReset
        }
        return Waitseq

    def GetCmdseq(self):
        if self.frame == 1: 
            Cmdseq = {
                '1':self.CID2.Arm,
                '2':self.CID2.DisArm,
                '3':self.CID2.QuadPos,
                '4':self.CID2.QuadVel,
                '5':self.CID2.UAVLand,
                '6':self.CID2.FaultInject
            }
        elif self.frame == 2: 
            Cmdseq = {
                '1':self.CID2.Arm,
                '2':self.CID2.DisArm,
                '3':self.CID2.FixWingTakeOff,
                '4':self.CID2.FixWingPos,
                '5':self.CID2.UAVLand,
                '6':self.CID2.FaultInject
            }
        elif self.frame == 3:
            Cmdseq = {
                '1':self.CID2.Arm,
                '2':self.CID2.DisArm,
                '3':self.CID2.USVPos,
                '4':self.CID2.USVVel,
                '5':self.CID2.USVGroundSpeed,
                '6':self.CID2.FaultInject
            }
        return Cmdseq

    def FIDPro(self,cmdCID):
        if cmdCID == '1':
            self.FID = CmdCtrl.GetWaitseq(self)
        elif cmdCID == '2':
            self.FID = CmdCtrl.GetCmdseq(self)
        return self.FID
