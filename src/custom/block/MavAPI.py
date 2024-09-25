import os
import re
import cv2
import sys
import time
import json
import threading
import numpy as np
sys.path.append(os.getcwd())
import src.openSHA.ProfustSA.Ass as ProfustSA
from scipy.interpolate import interp1d

try:
    import include.AutoREG as AutoREG
    import include.AutoMavDB as AutoMavDB
    import include.AutoMavCmd as AutoMavCmd
    import include.AutoVisConf as AutoVis
    import include.UE4CtrlAPI as UE4CtrlAPI
    import include.PX4MavCtrlV4 as PX4MavCtrl
except ImportError:
    import AutoREG 
    import AutoMavDB 
    import AutoMavCmd 
    import UE4CtrlAPI
    import AutoVisConf as AutoVis
    import PX4MavCtrlV4 as PX4MavCtrl

ue = UE4CtrlAPI.UE4CtrlAPI()

Fault_Modes = {
    123450: 'Motor Fault',
    123548: 'GPS Fault',
    123544: 'Accelerometer Fault',
    123547: 'Barometer Fault',
    123546: 'Magnetometer Fault',
    123545: 'Gyroscope Fault'
}

def GetPath(frame, SimMode):
    dbf = 'db.json'
    jsonpath = os.path.join(sys.path[0], dbf)

    model_path = os.path.join(os.getcwd(),'src','model',frame)
    conf = SimMode + '.bat'  
    batp = [filename for filename in os.listdir(model_path) if conf in filename]  
    batpath = os.path.join(model_path, batp[0])
    return jsonpath, batpath

class MavAPI():
    def __init__(self, frame = 'Quadcopter', SimMode = 'SITL', ID = 1) -> None:
        self.mavid = ID 
        self.conf = [frame, SimMode, 1]
        self.MAVDBobj = AutoMavDB.MAVDB(self.conf)
        self.jsonpath, self.batpath = GetPath(frame, SimMode)

        AutoREG.MAV_NUM = 1
        AutoREG.TEST_MODE = 1
        self.is_SPo = False
        self.is_SVo = False
        self.PVCmdTag = False
        self.SPVOTag = False
        self.FaultInTag = False
        self.ArmedTag = False
        self.DisarmedTag = False
        self.LandTag = False
        
        # Initialize Control instruction sequence index
        self.MavCaseInd = 0
        self.MavCmdInd = 0
        self.MavCmd, self.caseLen, self.CaseID = self.GetCmd()
        self.MavCmdNum = len(self.MavCmd)

        
    
    def IninMavEnv(self):
        if type(self.mavid) != str:
            self.mav = PX4MavCtrl.PX4MavCtrler(self.mavid)
        else:
            self.mav = PX4MavCtrl.PX4MavCtrler(Com=self.mavid)
        time.sleep(0.5)

        self.mav.InitMavLoop()
        time.sleep(0.5)

        self.mav.initOffboard()
        time.sleep(0.5)

        self.mav.InitTrueDataLoop()
        time.sleep(0.5)
    
    def EndMavEnv(self):
        self.mav.EndTrueDataLoop()
        time.sleep(0.5)
        self.mav.endMavLoop() 
        time.sleep(0.5)

        time.sleep(5)
    
    def InitMavConf(self):
        self.CFID = AutoMavCmd.CmdCtrl(self.mav, AutoREG.RFLYSIM_FRAME[self.conf[0]])
        self.CID1OBJ = self.CFID.CID1
        self.CID2OBJ = self.CFID.CID2

        # Initialize flight truth data
        self.WCSVel = np.array([0,0,0]) 
        self.WCSAng = np.array([0,0,0]) 
        self.WCSAcc = np.array([0,0,0]) 
        self.WCSEular = np.array([0,0,0]) 
        self.WCSPos = np.array([0,0,0]) 
        self.MotorRPM = np.array([0,0,0,0,0,0,0,0]) 
        self.FallVEL = 0 
        self.FallEnergy = 0 
        self.m = 1.515 

        # Initialize Flight index variable
        self.FLYIND = 0 
        self.RECORDIND = 0 
        self.RECORDFLAG = False
        self.RECORDTIME = 0
        self.EXITUND = 0 
        self.EXITFLAG = False
        self.EXITTIME = 0
        self.INJECTIND = 0 
        self.INJECTFLAG = False
        self.INJECTTIME = 0
        self.FALLIND = 0 
        self.FALLFLAG = False
        self.FALLTIME = 0
        self.LANDIND = 0 
        self.LANDFLAG = False
        self.LANDTIME = 0
        self.SPVoIND = 0
        self.SPVoTIME = 0
        self.SPVoModeFLAG = False

        # Fault test status parameters
        self.ARMEDERROR = False

        self.result_data = None

        # Start time and end time (unlock after startup to prevent the ground station from not starting timing)
        self.startTime = time.time()
        self.endTime = time.time()
        self.lastTime = time.time()
        self.mav.SendMavArm(1)
        ue.sendUE4Cmd('RflyShowTextTime "Sim Start" 10')
        print(f'mav{self.mavid} Sim start')


        print(f'Start mav{self.mavid} caseID {self.CaseID}')
        print(f'mav{self.mavid} cmd',self.MavCmd)

    def GetCmd(self):
        with open(self.jsonpath, "r",encoding='utf-8') as f:
            json_data = json.load(f)

        caseID = json_data.get('TEST_CASE')
        caseLen = len(caseID)
        caseCmd = next((item["ControlSequence"] for item in json_data.get('FAULT_CASE') if item["CaseID"] == int(caseID)), None)

        case = re.split(';',caseCmd)
        cmd = np.array([])
        for i in range(len(case)):
            cmd = np.append(cmd, case[i])
        return cmd, caseLen, caseID

    def AutoMavRun(self):
        print(f'hello, mav{self.mavid}')

        while True: 
            if  self.MavCaseInd >= self.caseLen:
                print(f'mav{self.mavid} all case test finish!')
                break

            self.IninMavEnv()
            self.InitMavConf()

            while True:
                # 250HZ receiving data
                self.lastTime = self.lastTime + (1.0/250)
                sleepTime = self.lastTime - time.time()
                if sleepTime > 0:
                    time.sleep(sleepTime)
                else:
                    self.lastTime = time.time()
                
                if self.MavCmdInd >= self.MavCmdNum and self.EXITFLAG == False:
                    self.EXITFLAG = True
                    print(f'mav{self.mavid}: CaseID {self.CaseID} test completed')
                    self.endTime = time.time()
                    self.SPVobreak = True
                    self.PVCmdbreak = True
                    break
                
                
                # Start recording data after the unlocking command is issued
                if self.CID2OBJ.ARMFLAG == True:
                    VEL = np.array(self.mav.trueVelNED)
                    ANGRATE = np.array(self.mav.trueAngRate) 
                    ACC = np.array(self.mav.trueAccB)
                    MOTORRPM = np.array(self.mav.trueMotorRPMS)
                    EULAR = np.array(self.mav.trueAngEular)
                    POS = np.array(self.mav.truePosNED)
                    self.FLYIND = self.FLYIND + 1
                    self.WCSVel = np.row_stack((self.WCSVel,VEL)) 
                    self.WCSAng = np.row_stack((self.WCSAng,ANGRATE)) 
                    self.WCSAcc = np.row_stack((self.WCSAcc,ACC)) 
                    self.WCSEular = np.row_stack((self.WCSEular,EULAR)) 
                    self.WCSPos = np.row_stack((self.WCSPos,POS)) 
                    self.MotorRPM = np.row_stack((self.MotorRPM,MOTORRPM)) 

                # Processing instruction sequence
                self.TRIGGERMAVCMD(self.MavCmd[self.MavCmdInd])

                if self.PVCmdTag == True:
                    if (re.findall(r'-?\d+\.?[0-9]*',self.MavCmd[self.MavCmdInd])[0] == '1' and self.CID1OBJ.isDone == 1):
                        self.PVCmdbreak = True

                 # If one instruction sequence is completed, the next instruction is processed
                if (re.findall(r'-?\d+\.?[0-9]*',self.MavCmd[self.MavCmdInd])[0] == '1' and self.CID1OBJ.isDone == 1) or \
                    (re.findall(r'2,1',self.MavCmd[self.MavCmdInd]) and self.ArmedTag == True) or \
                    (re.findall(r'2,2',self.MavCmd[self.MavCmdInd]) and self.DisarmedTag == True) or \
                    (re.findall(r'2,5',self.MavCmd[self.MavCmdInd]) and self.LandTag == True) or \
                    (re.findall(r'2,[34],(-?\d+\.?\d*)', self.MavCmd[self.MavCmdInd]) and self.PVCmdTag == True) or \
                    (re.findall(r'2,3,SPo',self.MavCmd[self.MavCmdInd]) and self.SPVOTag == True) or \
                    (re.findall(r'2,4,SVo',self.MavCmd[self.MavCmdInd]) and self.SPVOTag == True) or \
                    (re.findall(r'2,6,FInR:',self.MavCmd[self.MavCmdInd]) and self.FaultInTag == True) or\
                    re.findall(r'-?\d+\.?[0-9]*',self.MavCmd[self.MavCmdInd])[0] == '2' and self.CID2OBJ.isDone == 1:
                    self.MavCmdInd = self.MavCmdInd + 1
                        
                    print('++++++++++++++++++++++++++++++++++++++++++++++++++++++')

                if (self.is_SPo or self.is_SVo) and self.SPVoModeFLAG == False:
                    self.SPVoIND = self.FLYIND 
                    self.SPVoTIME = round(time.time() - self.startTime)
                    self.SPVoModeFLAG = True

                if self.CID2OBJ.RECORDFLAG == True and self.RECORDFLAG == False:
                    self.RECORDIND = self.FLYIND 
                    self.RECORDTIME = round(time.time() - self.startTime)
                    self.RECORDFLAG = True
                
                if self.CID2OBJ.INJECTFLAG == True and self.INJECTFLAG == False:
                    self.INJECTIND = self.FLYIND 
                    self.INJECTTIME = round(time.time() - self.startTime)
                    self.INJECTFLAG = True
                
                if self.CID2OBJ.LANDFLAG == True and self.LANDFLAG == False:
                    self.LANDIND = self.FLYIND 
                    self.LANDTIME = round(time.time() - self.startTime)
                    self.LANDFLAG = True

                if self.MavCmdInd >= self.MavCmdNum and self.EXITFLAG == False:
                    self.EXITUND = self.FLYIND 
                    self.EXITTIME = round(time.time() - self.startTime)
                    self.EXITFLAG = True
                    print(f'mav{self.mavid}: CaseID {self.CaseID} test completed')
                    ue.sendUE4Cmd('RflyShowTextTime "CaseID %s test completed!" 10'%(self.CaseID))
                    self.endTime = time.time()

                    self.SPVobreak = True
                    break

                # Judgment of crash, if the landing speed is greater than 3.5, the aircraft is considered to have crashed
                if  abs(np.array(self.mav.truePosNED[2])) < 1.2 and self.FALLFLAG == False  and abs(np.array(self.mav.trueVelNED[2])) > 3.5: 
                    self.FALLTIME =  round(time.time() - self.startTime)
                    self.FALLIND = self.FLYIND
                    self.FALLFLAG = True
                    self.FallVEL = round(np.max(self.WCSVel),2) 
                    self.FallEnergy = round(0.5*self.m*(self.FallVEL**2),2) 
                    self.EXITUND = self.FLYIND 
                    self.EXITTIME = round(time.time() - self.startTime)
                    self.EXITFLAG = True
                    self.endTime = time.time()

                    AutoREG.TEST_RESULT['Is_Fall'] = 'Yes'
                    AutoREG.TEST_RESULT['Fall Time'] = self.EXITTIME
                    AutoREG.TEST_RESULT['Fall Vel'] = self.FallVEL
                    AutoREG.TEST_RESULT['Fall Energy'] = self.FallEnergy

                    print("{}s,Crash! Exit the test".format(self.EXITTIME))
                    ue.sendUE4Cmd('RflyShowTextTime "Crash!" 10')

                    self.SPVobreak = True
                    break

                # If the unlocking is abnormal, exit the test and start a new test again
                if self.mav.isArmerror == 1:
                    self.ARMEDERROR = True
                    AutoREG.ARMED_WARN = True
                    # break
                
                if self.mav.isFailsafeEn == True:
                    AutoREG.TEST_RESULT['Failsafe Trigger'] = 'Yes!' + self.mav.FailsafeInfo


            if self.ARMEDERROR == False:

                if self.MAVDBobj.VISIONFLAG == True:
                    cv2.destroyAllWindows() 
                
                '''Start data processing thread'''

                self.EndMavEnv()

                self.MavCaseInd += 1
            
            print(f'mav{self.mavid} next round')


    def FaultInject_Real(self, modes, flags, ctrls):
        self.mav.SendHILCtrlMsg(ctrls,modes,flags)
        time.sleep(0.5)
        print(f"Start Fault Inject! \n Fault Mode:{modes} \n Fault Flags:{flags} \n Fault Params:{ctrls}")
        self.mav.SendMavCmdLong(777,modes,flags,ctrls[0],ctrls[1],ctrls[2],ctrls[3],ctrls[4])
        self.FaultInTag = True

    def Get_SPVo(self, spv_x, spv_y, spv_z):
        self.spv_x = spv_x
        self.spv_y = spv_y
        self.spv_z = spv_z
        AutoREG.SPVoMode = True

    def SPVoThrd(self):
        self.TSPo = threading.Thread(target=self.SPVoPub, args=(self.MavCmd[self.MavCmdInd],))
        self.TSPo.start()

    def SPVoPub(self, ctrlseq):
        cmdseq = ctrlseq.split(',')
        cmdCID = cmdseq[0]
        FID = self.CFID.FIDPro(cmdCID)
        self.SPVobreak = False
        self.set_point_mission = np.array([])
        
        for x, y, z in zip(self.spv_x, self.spv_y, self.spv_z):
            if self.SPVobreak:
                self.SPVOTag = False
                break
                
            param = [x, y, z]
            self.set_point_mission = np.append(self.set_point_mission, param)

            FID[cmdseq[1]](param)
            time.sleep(0.5)
    
    def PVCmdThrd(self):
        self.TPV = threading.Thread(target=self.PVCmdPub, args=(self.MavCmd[self.MavCmdInd],))
        self.TPV.start()

    def TLand(self):
        tland = threading.Thread(target=self.LandCmd, args=())
        tland.start()

    def LandCmd(self):
        pos = self.mav.uavPosNED
        stime = time.time()
        while True:
            if time.time() - stime > 5:
                self.LandTag = True
                break

            self.mav.sendMavLand(pos[0], pos[1], 0)
            time.sleep(1)

    def PVCmdPub(self, ctrlseq):
        cmdseq = ctrlseq.split(',')
        cmdCID = cmdseq[0]
        FID = self.CFID.FIDPro(cmdCID)
        self.PVCmdbreak = False
        self.set_point_cmd = np.array([])

        while True:
            if self.PVCmdbreak and self.CID2OBJ.isDone == 1:
                self.PVCmdTag = False
                break

            param = [float(par) for par in cmdseq[2:]]
            self.set_point_cmd = np.append(self.set_point_cmd, param)

            FID[cmdseq[1]](param)
            time.sleep(2)

    def TRIGGERMAVCMD(self, ctrlseq):
        cmdseq = ctrlseq # '2,3,0,0,-20'

        is_armed = re.findall(r'2,1',cmdseq)
        is_SPo = re.findall(r'SPo',cmdseq)
        is_SVo = re.findall(r'SVo',cmdseq)
        is_FInR = re.findall('FInR:',cmdseq)
        is_PVCmd = re.findall(r'2,[34]', cmdseq)
        is_disarmed = re.findall(r'2,2',cmdseq)
        is_land = re.findall(r'2,5',cmdseq)

        if is_armed:
            self.ArmedTag = True
            print('Start send RC armed ctrl')
        elif is_disarmed:
            self.DisarmedTag = True
            print('Stop send RC disarmed ctrl, Exit Test!')
            time.sleep(0.5)
        elif is_land:
            print('Start Send Land Mode!')
            self.LandTag = True
            self.TLand()
        elif is_SPo:
            print('Start Send SPO Mission Point!')
            self.SPVOTag = True
            self.SPVoThrd()
        elif is_SVo:
            self.SPVoThrd()
            self.SPVOTag = True
            print('Start Send SVO Mission Point!')
        elif is_FInR:
            cmd = cmdseq.split('FInR:')[1].split(',')
            modes = int(cmd[0])
            flags = int(cmd[1])
            ctrls = [float(ctrl) for ctrl in cmd[2:]]
            print('Start FaultInjection!')
            self.FaultInject_Real(modes, flags, ctrls) 
        elif is_PVCmd:
            print('Start Send PVCmd Mode!')
            self.PVCmdTag = True
            self.PVCmdThrd()
        else:
            cmdseq = re.findall(r'-?\d+\.?[0-9]*',cmdseq) # ['2', '3', '0', '0', '-20']
            cmdCID = cmdseq[0]
            if  cmdCID in self.CFID.CID:
                FID = self.CFID.FIDPro(cmdCID)
                # if has param
                if len(cmdseq) > 2:
                    # get param
                    param = cmdseq[2:len(cmdseq)]
                    param = [float(val) for val in param]
                    FID[cmdseq[1]](param)
                else:
                    FID[cmdseq[1]]()
            else:
                print(f'mav{self.mavid} Command input error, please re-enter')
    