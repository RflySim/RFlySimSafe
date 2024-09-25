import subprocess
import time
import os, sys
sys.path.append(os.getcwd())

try:
    import AutoREG 
except ImportError:
    import include.AutoREG as AutoREG

def GetPath(frame, SimMode, filename):
    dbf = 'db.json'
    jsonpath = os.path.join(sys.path[0], '..', filename, dbf)

    model_path = os.path.join(sys.path[0],'..','..','model',frame)
    conf = SimMode + '.bat'  
    batp = [filename for filename in os.listdir(model_path) if conf in filename]  
    batpath = os.path.join(model_path, batp[0])
    return jsonpath, batpath

class SimAPI:
    def __init__(self, frame = 'Quadcopter', SimMode = 'SITL', filename = 'None') -> None:
        self.frame = frame
        self.SimMode = SimMode
        self.jsonpath, self.batpath = GetPath(frame, SimMode, filename)

    def SimStart(self):
        self.child = subprocess.Popen(self.batpath,shell=True,stdout=subprocess.PIPE)
        print('Get all SimProcessStart theard instance! Start open RflySim simulation tools!')
        time.sleep(AutoREG.SIM_WAIT_TIME_REG)
    
    def SimEnd(self):
        # Exit the simulation software
        print(f'Exit {self.conf[1]} simulation software')
        os.system('tasklist|find /i "CopterSim.exe" && taskkill /im "CopterSim.exe"')
        os.system('tasklist|find /i "QGroundControl.exe" && taskkill /f /im "QGroundControl.exe"')
        os.system('tasklist|find /i "RflySim3D.exe" && taskkill /f /im "RflySim3D.exe"')
        # self.child.terminate()
        # self.child.kill()
        print('All closed')
        time.sleep(5)
