
import socket
import threading
import time
from pymavlink.dialects.v20 import common as mavlink2
import os 
import time
import sys
import struct
import math
import sys
import copy
import shutil


# PX4 MAVLink listen and control API and RflySim3D control API
class QGCCtrlAPI:

    # constructor function
    def __init__(self,ID=1):

        self.f = fifo
        self.mav0 = mavlink2.MAVLink(self.f)
        self.QGCPath=os.environ['USERPROFILE']+'\\Documents\\QGroundControl\\Logs'
        self.udp_socketQGC = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create socket
        self.udp_socketQGC.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def SendQgcCmdLong(self, command, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
            buf = self.mav0.command_long_encode(255, 0,
                                                command, 0,
                                                param1, param2, param3, param4, param5, param6, param7).pack(self.mav0)
            self.udp_socketQGC.sendto(buf, ('127.0.0.1', 14550))

    #获取日志文件名
    def getTxtContent(self,flagFilePath):
        file=open(flagFilePath, mode='r')
        lines=file.readlines()
        file.close()
        os.remove(flagFilePath)
        return lines[0].replace('\n', '')


    #将日志文件复制指定位置
    def copyLogFile(self,source,target):
        shutil.copyfile(source, target)

    # 请求QGC日志，设定请求日志的ID号（默认为1），并设置超时时间100s
    def ReqQgcLog(self,timeout=180,CopterID=1):

        outLogName=''

        LogFilePath = self.QGCPath+'\\log.txt'
        hasLogFile=False

        errFilePath = self.QGCPath+'\\error.txt'
        hasErrFile=False

        shutil.rmtree

        # print('Delete log.txt and error.txt')
        if os.path.exists(LogFilePath):
            os.remove(LogFilePath)

        if os.path.exists(errFilePath):
            os.remove(errFilePath)


        # 发送下载日志的请求
        self.SendQgcCmdLong(42700,CopterID,0,0,0,0,0,0)
        print('Send requst to QGC for log downloading...')


        startTime=time.time()
        while time.time()-startTime<=timeout: # 一直等待，直到超时时间
            if(os.path.exists(LogFilePath)):
                hasLogFile=True
                break

            if(os.path.exists(errFilePath)):
                hasErrFile=True
                break
            
            time.sleep(1)
        
        if hasLogFile:
            logName=self.getTxtContent(LogFilePath)
            if os.path.exists(self.QGCPath+'\\'+logName):
                # shutil.copyfile(self.QGCPath+'\\'+logName,sys.path[0]+'\\'+logName)
                # print('Download log '+logName+' successfully.')
                outLogName=logName
                return outLogName
            else:
                print('Error content in log.txt: '+logName)

        if hasErrFile:
            errMsg = self.getTxtContent(errFilePath)
            print(errMsg)

        # 超时的情况
        if ~hasErrFile and ~hasLogFile:
            print('Timeout for waiting log download..')

        return outLogName



# define a class for MAVLink initialization
class fifo(object):
    def __init__(self):
        self.buf = []

    def write(self, data):
        self.buf += data
        return len(data)

    def read(self):
        return self.buf.pop(0)
