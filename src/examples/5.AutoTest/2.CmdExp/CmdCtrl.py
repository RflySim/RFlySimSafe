import AutoMavCmd
import re, json
import PX4MavCtrlV4
import sys, os
import time


def TRIGGERMAVCMD(ctrlseq):
    cmdseq = ctrlseq # '2,3,0,0,-20'
    cmdseq = re.findall(r'-?\d+\.?[0-9]*',cmdseq) # ['2', '3', '0', '0', '-20']
    cmdCID = cmdseq[0]
    if  cmdCID in CFID.CID:
        FID = CFID.FIDPro(cmdCID)
        # if has param
        if len(cmdseq) > 2:
            # get param
            param = cmdseq[2:len(cmdseq)]
            param = [float(val) for val in param]
            FID[cmdseq[1]](param)

        else:
            FID[cmdseq[1]]()
    else:
        print(f'mav Command input error, please re-enter') 


mav = PX4MavCtrlV4.PX4MavCtrler(20100)

mav.InitMavLoop()
time.sleep(0.5)

mav.initOffboard()
time.sleep(0.5)

frame = 1 # Quadcopter

CFID = AutoMavCmd.CmdCtrl(mav,frame)
CID1OBJ = CFID.CID1
CID2OBJ = CFID.CID2



jsonpath = os.path.dirname(__file__) + '/db.json'     
with open(jsonpath, "r",encoding='utf-8') as f:
    json_data = json.load(f)
json_case = json_data.get('FAULT_CASE')
test_case_1 = json_case[0]['ControlSequence']
MavCmd = test_case_1.split(';')
MavCmdInd = 0
MavCmdNum = len(MavCmd) - 1

lastTime = time.time()
sleepTime = time.time()

while True:
    # 250HZ receiving data
    lastTime = lastTime + (1.0/250)
    sleepTime = lastTime - time.time()
    if sleepTime > 0:
        time.sleep(sleepTime)
    else:
        lastTime = time.time()

    # Processing instruction sequence
    TRIGGERMAVCMD(MavCmd[MavCmdInd])

    # If one instruction sequence is completed, the next instruction is processed
    if re.findall(r'-?\d+\.?[0-9]*',MavCmd[MavCmdInd])[0] == '1' and CID1OBJ.isDone == 1 or re.findall(r'-?\d+\.?[0-9]*',MavCmd[MavCmdInd])[0] == '2' and CID2OBJ.isDone == 1:
        MavCmdInd = MavCmdInd + 1

    if MavCmdInd > MavCmdNum:
        break




