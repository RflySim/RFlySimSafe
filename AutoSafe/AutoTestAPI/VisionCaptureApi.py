
from email import header
from re import A
import socket
import threading
import time
import cv2
import numpy as np
import struct
import mmap
import json
import sys
import os
import math
import copy
import platform
import AutoREG

isLinux = False
if platform.system().lower() == 'linux':
    isLinux = True
    try:
        from logging import exception
        from typing import Any
        from xml.etree.ElementTree import tostring
        import yaml
        import rospy
        import sensor_msgs.msg as sensor
        import std_msgs.msg as std_msg

    except ImportError:
        print('Faild to load ROS labs')


# IsEnable ROS image topic forwarding
isEnableRosTrans = False




class Queue:
    """pacth
    """

    def __init__(self):
        self.items = []

    def enqueue(self, item):
        self.items.insert(0, item)

    def dequeue(self):
        return self.items.pop()

    def is_empty(self):
        return self.items == []

    def size(self):
        return len(self.items)


class RflyTimeStmp:
    def __init__(self):
        self.checksum = 1234567897
        self.copterID = 0
        self.SysStartTime = 0
        self.SysCurrentTime = 0
        self.HeartCount = 0


class VisionSensorReq:
    """ This is a class (C++ struct) that sent to UE4 to request and set camera parameters.
        #struct VisionSensorReq {
         # uint16 checksum; //Data checksum, 12345
         # uint16 SeqID; //Memory serial number ID
         # uint16 TypeID; //sensor type ID
         # uint16 TargetCopter; //bound target aircraft //can be changed
         # uint16 TargetMountType; //Bound type //Can be changed
         # uint16 DataWidth; // data or image width
         # uint16 DataHeight; // data or image height
         # uint16 DataCheckFreq; //Check data update frequency
         # uint16 SendProtocol[8]; //Transmission type (shared memory, UDP transmission without compression, UDP video streaming), IP address, port number,...
         # float CameraFOV; //Camera field of view (only for vision sensors) //Can be changed
         # float SensorPosXYZ[3]; // sensor installation position // can be changed
         # float SensorAngEular[3]; //Sensor installation angle //Can be changed
         # float otherParams[8]; //Reserved eight data bits
         # }16H15f
    """

    def __init__(self):
        self.checksum = 12345
        self.SeqID = 0
        self.TypeID = 1
        self.TargetCopter = 1
        self.TargetMountType = 0
        self.DataWidth = 0
        self.DataHeight = 0
        self.DataCheckFreq = 0
        self.SendProtocol = [0, 0, 0, 0, 0, 0, 0, 0]
        self.CameraFOV = 90
        self.SensorPosXYZ = [0, 0, 0]
        self.SensorAngEular = [0, 0, 0]
        self.otherParams = [0, 0, 0, 0, 0, 0, 0, 0]


class imuDataCopter:
    """ This is a class (C++ struct) for IMU data receive from CopterSim
        #struct imuDataCopter{
         # int checksum; //Data checksum 1234567898
         # int seq; //message serial number
         # double timestamp;//Timestamp
         # float acc[3];
         # float rate[3];
         # } //2i1d6f  
    """

    def __init__(self):
        self.checksum = 1234567898
        self.seq = 0
        self.timestmp = 0
        self.acc = [0, 0, 0]
        self.rate = [0, 0, 0]
        if isEnableRosTrans:
            self.time_record = -1
            self.isUseTimeAlign = True  
            self.rostime = rospy.Time.now()
            self.imu_pub = rospy.Publisher(
                "/rflysim/imu", sensor.Imu, queue_size=1)
            self.time_queue = Queue()
            self.newest_time_img = -1
            self.test_imu_time = 0
            self.test_sum = 0
            self.count = 0
            self.ros_imu = sensor.Imu()
            self.imu_frame_id = "imu"
            self.ros_imu.header.frame_id = self.imu_frame_id

    def AlignTime(self, img_time): 
        self.newest_time_img = img_time
        # print("queue size: ",self.time_queue.size())
        # print("current <img:%f,imu:%f>"% (img_time,self.test_imu_time))
        # self.test_sum += abs(self.newest_time_img - self.test_imu_time)
        # self.count +=1
        # if(self.count == 10):
        #     print("====",self.test_sum/self.count)
        #     self.count = 0
        #     self.test_sum = 0

        pass

    def Imu2ros(self):
        if isEnableRosTrans:
            # ros_imu = sensor.Imu()
            acc = [0, 0, 0]
            rate = [0, 0, 0]
            if(self.isUseTimeAlign):
                self.time_queue.enqueue((self.timestmp, [self.acc, self.rate]))
                if(self.newest_time_img == -1):
                   
                    return
                if(self.time_record == -1):
                    imu = self.time_queue.dequeue()
                    self.time_record = imu[0]
                    self.rostime = rospy.Time.now()
                    # return
                imu = self.time_queue.dequeue()
                self.test_imu_time = imu[0]
                self.ros_imu.header.stamp = self.rostime + \
                    rospy.Duration(imu[0] - self.time_record)
                acc[0] = imu[1][0][0]
                acc[1] = imu[1][0][1]
                acc[2] = imu[1][0][2]
                rate[0] = imu[1][1][0]
                rate[1] = imu[1][1][1]
                rate[2] = imu[1][1][2]
            else:
                if(self.time_record == -1):
                    self.time_record = self.timestmp
                    self.rostime = rospy.Time.now()
                self.ros_imu.header.stamp = self.rostime + \
                    rospy.Duration(self.timestmp - self.time_record)
                acc[0] = self.acc[0]
                acc[1] = self.acc[1]
                acc[2] = self.acc[2]
                rate[0] = self.rate[0]
                rate[1] = self.rate[1]
                rate[2] = self.rate[2]
            self.ros_imu.orientation.w = 0
            self.ros_imu.orientation.x = 0
            self.ros_imu.orientation.y = 0
            self.ros_imu.orientation.z = 0
            self.ros_imu.orientation_covariance[0] = -1
            self.ros_imu.orientation_covariance[1] = -1
            self.ros_imu.orientation_covariance[2] = -1
            self.ros_imu.orientation_covariance[3] = -1
            self.ros_imu.orientation_covariance[4] = -1
            self.ros_imu.orientation_covariance[5] = -1
            self.ros_imu.orientation_covariance[6] = -1
            self.ros_imu.orientation_covariance[7] = -1
            self.ros_imu.orientation_covariance[8] = -1
            self.ros_imu.linear_acceleration.x = acc[0]
            self.ros_imu.linear_acceleration.y = acc[1]
            self.ros_imu.linear_acceleration.z = acc[2]
            self.ros_imu.angular_velocity.x = rate[0]
            self.ros_imu.angular_velocity.y = rate[1]
            self.ros_imu.angular_velocity.z = rate[2]
            self.ros_imu.angular_velocity_covariance[0] = 0.001
            self.ros_imu.angular_velocity_covariance[4] = 0.001
            self.ros_imu.angular_velocity_covariance[8] = 0.001
            # self.ros_imu.header.stamp.secs = self.timestmp
            self.imu_pub.publish(self.ros_imu)


class SensorReqCopterSim:
    """This is a class (C++ struct) that sent to UE4 to request sensor data.
        # struct SensorReqCopterSim{
        #     uint16_t checksum;
        #     uint16_t sensorType;
        #     uint16_t updateFreq;
        #     uint16_t port;
        #     uint8_t IP[4];
        #     float Params[6];
        # } //4H4B6f    
    """

    def __init__(self):
        self.checksum = 12345
        self.sensorType = 0
        self.updateFreq = 100
        self.port = 9998
        self.IP = [127, 0, 0, 1]
        self.Params = [0, 0, 0, 0, 0, 0]

class reqVeCrashData:
    def __init__(self):
        self.checksum = 1234567897
        self.copterID = 0
        self.vehicleType = 0
        self.CrashType = 0
        self.runnedTime = 0
        self.VelE = [0, 0, 0]
        self.PosE = [0, 0, 0]
        self.CrashPos = [0, 0, 0]
        self.targetPos = [0, 0, 0]
        self.AngEuler = [0, 0, 0]
        self.MotorRPMS = [0, 0, 0, 0, 0, 0, 0, 0]
        self.ray = [0, 0, 0, 0, 0, 0]
        self.CrashedName = ''

    def __init__(self, iv):
        self.checksum = iv[0]
        self.copterID = iv[1]
        self.vehicleType = iv[2]
        self.CrashType = iv[3]
        self.runnedTime = iv[4]
        self.VelE = iv[5:8]
        self.PosE = iv[8:11]
        self.CrashPos = iv[11:14]
        self.targetPos = iv[14:17]
        self.AngEuler = iv[17:20]
        self.MotorRPMS = iv[20:28]
        self.ray = iv[28:34]
        self.CrashedName = iv[34].decode('UTF-8')


class PX4SILIntFloat:
    def __init__(self):
        self.checksum = 0
        self.CopterID = 0
        self.inSILInts = [0, 0, 0, 0, 0, 0, 0, 0]
        self.inSILFLoats = [0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    def __init__(self, iv):
        self.checksum = iv[0]
        self.CopterID = iv[1]
        self.inSILInts = iv[2:10]
        self.inSILFLoats = iv[10:30]


class VisionCaptureApi:
    """ This is the API class for python to request image from UE4
    """

    def __init__(self):
        if isEnableRosTrans:
            rospy.init_node("RecvRFlySim3DData", anonymous=True)
            self.time_record = Any
            self.rostime = Any
   
        self.udp_socket = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM) 
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.udp_imu = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM)  
        self.VisSensor = []
        self.Img = []
        self.ImgData = []
        self.hasData = []
        self.timeStmp = []
        self.IpList = []
        self.portList = []
        self.hasReqUE4 = False
        self.sleepCheck = 0.005
        self.ip = '127.0.0.1'
        self.isRemoteSend = False
        self.RemotSendIP = ''
        self.isUE4DirectUDP = False
        self.imu = imuDataCopter()
        self.hasIMUData = False
        self.RflyTime = RflyTimeStmp()
        self.inSilVect = []
        self.inReqVect = []
        self.inReqUpdateVect = []
        self.stopFlagUE4 = True
        self.startTime = time.time()
        self.isPrintTime = False
        self.lastIMUTime = time.time()
        self.sensors_num = 0
        if isEnableRosTrans:
            self.sensor_frame_id = ["map"]
            self.imu_frame_id = "imu"
            try:
                file = open(r"tf_cfg.yaml")
                y = yaml.safe_load(file)
                self.imu.imu_frame_id = y["imu_frame_id"]
                self.sensors_frame_id = y["sensors_frame_id"]
                self.sensors_num = y["sensors_num"]
            except IOError:
                pass

 

    def addVisSensor(self, vsr=VisionSensorReq()):
        """ Add a new VisionSensorReq struct to the list
        """
        if isinstance(vsr, VisionSensorReq):
            self.VisSensor = self.VisSensor + [copy.deepcopy(vsr)]
        else:
            raise Exception("Wrong data input to addVisSensor()")

    def sendUE4Cmd(self, cmd, windowID=-1):
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
        buf = struct.pack("i52s", 1234567890, cmd)
        if windowID < 0:
            if self.ip == '127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                # multicast address, send to all RflySim3Ds on all PC in LAN
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009))
        else:
            if self.ip != '127.0.0.1' and self.ip != '255.255.255.255':
                # ensure this PC can reciver message under specify IP mode
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID))
            # specify PC's IP to send
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))

    def sendUE4Attatch(self, CopterIDs, AttatchIDs, AttatchTypes, windowID=-1):
        """ Send msg to UE4 to attach a vehicle to another (25 vehicles);
        CopterIDs,AttatchIDs,AttatchTypes can be a list with max len 25
        """
        # change the 1D variable to 1D list
        if isinstance(CopterIDs, int):
            CopterIDs = [CopterIDs]

        if isinstance(AttatchIDs, int):
            AttatchIDs = [AttatchIDs]

        if isinstance(AttatchTypes, int):
            AttatchTypes = [AttatchTypes]

        if not isinstance(CopterIDs, list) or not isinstance(AttatchIDs, list) or not isinstance(AttatchTypes, list):
            print('Error: Wrong sendUE4Attatch input Type')
            return

        if len(CopterIDs) != len(AttatchIDs) or len(CopterIDs) != len(AttatchTypes) or len(CopterIDs) > 25:
            print('Error: Wrong sendUE4Attatch input dimension')
            return

        vLen = len(CopterIDs)
        if vLen < 25:  # Extend the IDs to 25D
            CopterIDs = CopterIDs + [0]*(25-vLen)
            AttatchIDs = AttatchIDs + [0]*(25-vLen)
            AttatchTypes = AttatchTypes + [0]*(25-vLen)

        if vLen > 25:
            CopterIDs = CopterIDs[0:25]
            AttatchIDs = AttatchIDs[0:25]
            AttatchTypes = AttatchTypes[0:25]



        buf = struct.pack("i25i25i25i", 1234567892, *
                          CopterIDs, *AttatchIDs, *AttatchTypes)
        if windowID < 0:
            if self.ip == '127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                # multicast address, send to all RflySim3Ds on all PC in LAN
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009))
        else:
            if self.ip != '127.0.0.1' and self.ip != '255.255.255.255':
                # ensure this PC can reciver message under specify IP mode
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID))
            # specify PC's IP to send
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))

    def sendUE4Pos(self, copterID=1, vehicleType=3, MotorRPMSMean=0, PosE=[0, 0, 0], AngEuler=[0, 0, 0], windowID=-1):
        """send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
        """
        VelE = [0, 0, 0]
        self.sendUE4PosNew(copterID, vehicleType, PosE, AngEuler, VelE, [
                           MotorRPMSMean]*8, -1, windowID)

    # send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
    def sendUE4Pos2Ground(self, copterID=1, vehicleType=3, MotorRPMSMean=0, PosE=[0, 0, 0], AngEuler=[0, 0, 0], windowID=-1):
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
        buf = struct.pack("3i7f", 1234567891, copterID, vehicleType, MotorRPMSMean,
                          PosE[0], PosE[1], PosE[2], AngEuler[0], AngEuler[1], AngEuler[2])
        if windowID < 0:
            if self.ip == '127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                # multicast address, send to all RflySim3Ds on all PC in LAN
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009))
        else:
            if self.ip != '127.0.0.1' and self.ip != '255.255.255.255':
                # ensure this PC can reciver message under specify IP mode
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID))
            # specify PC's IP to send
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))

    def sendUE4PosScale(self, copterID=1, vehicleType=3, MotorRPMSMean=0, PosE=[0, 0, 0], AngEuler=[0, 0, 0], Scale=[1, 1, 1], windowID=-1):
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
        buf = struct.pack("3i10f", 1234567890, copterID, vehicleType, MotorRPMSMean,
                          PosE[0], PosE[1], PosE[2], AngEuler[0], AngEuler[1], AngEuler[2], Scale[0], Scale[1], Scale[2])
        if windowID < 0:
            if self.ip == '127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                # multicast address, send to all RflySim3Ds on all PC in LAN
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009))
        else:
            if self.ip != '127.0.0.1' and self.ip != '255.255.255.255':
                # ensure this PC can reciver message under specify IP mode
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID))
            # specify PC's IP to send
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))

    def sendUE4PosScale2Ground(self, copterID=1, vehicleType=3, MotorRPMSMean=0, PosE=[0, 0, 0], AngEuler=[0, 0, 0], Scale=[1, 1, 1], windowID=-1):
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
        buf = struct.pack("3i10f", 1234567891, copterID, vehicleType, MotorRPMSMean,
                          PosE[0], PosE[1], PosE[2], AngEuler[0], AngEuler[1], AngEuler[2], Scale[0], Scale[1], Scale[2])
        if windowID < 0:
            if self.ip == '127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                # multicast address, send to all RflySim3Ds on all PC in LAN
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009))
        else:
            if self.ip != '127.0.0.1' and self.ip != '255.255.255.255':
                # ensure this PC can reciver message under specify IP mode
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID))
            # specify PC's IP to send
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))

    # send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
    def sendUE4PosFull(self, copterID, vehicleType, MotorRPMS, VelE, PosE, RateB, AngEuler, windowID=-1):
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
        runnedTime = -1
        # VelE=[0,0,0]
        AngQuatern = [0, 0, 0, 0]
        AccB = [0, 0, 0]
        # RateB=[0,0,0]
        PosGPS = PosE
        # buf for SOut2Simulator, len=152
        buf0 = struct.pack("2i1d27f3d", copterID, vehicleType, runnedTime, *VelE,
                           *PosE, *AngEuler, *AngQuatern, *MotorRPMS, *AccB, *RateB, *PosGPS)
        # buf for remaining 192-152=40bytes of payload[192] of netDataShort
        buf1 = bytes([0]*(192-len(buf0)))
        # buf for tg and len in netDataShort
        buf2 = struct.pack("2i", 2, len(buf0))
        # buf for netDataShort
        buf = buf2+buf0+buf1
        # print(len(buf))
        if windowID < 0:
            if self.ip == '127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                # multicast address, send to all RflySim3Ds on all PC in LAN
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009))
        else:
            if self.ip != '127.0.0.1' and self.ip != '255.255.255.255':
                # ensure this PC can reciver message under specify IP mode
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID))
            # specify PC's IP to send
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))
        #print('Message Send')

    def sendUE4ExtAct(self, copterID=1, ActExt=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], windowID=-1):
        # struct Ue4ExtMsg {
        #     int checksum;//1234567894
        #     int CopterID;
        #     double runnedTime; //Current  stamp (s)
        #     float ExtToUE4[16];
        # }
        # struct.pack 2i1d16f
        runnedTime = time.time()-self.startTime
        checkSum = 1234567894
        buf = struct.pack("2i1d16f", checkSum, copterID, runnedTime, *ActExt)
        if windowID < 0:
            if self.ip == '127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                # multicast address, send to all RflySim3Ds on all PC in LAN
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009))
        else:
            if self.ip != '127.0.0.1' and self.ip != '255.255.255.255':
                # ensure this PC can reciver message under specify IP mode
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID))
            # specify PC's IP to send
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))
        #print('Message Send')

    def sendUE4PosSimple(self, copterID, vehicleType, PWMs, VelE, PosE, AngEuler, runnedTime=-1, windowID=-1):
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
        # if runnedTime<0:
        #     runnedTime = time.time()-self.startTime
        checkSum = 1234567890
        # pack for SOut2SimulatorSimpleTime
        buf = struct.pack("3i17f1d", checkSum, copterID, vehicleType,
                          *PWMs, *PosE, *VelE, *AngEuler, runnedTime)
        # print(len(buf))
        if windowID < 0:
            if self.ip == '127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                # multicast address, send to all RflySim3Ds on all PC in LAN
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009))
        else:
            if self.ip != '127.0.0.1' and self.ip != '255.255.255.255':
                # ensure this PC can reciver message under specify IP mode
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID))
            # specify PC's IP to send
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))
        #print('Message Send')

    def sendUE4PosNew(self, copterID=1, vehicleType=3, PosE=[0, 0, 0], AngEuler=[0, 0, 0], VelE=[0, 0, 0], PWMs=[0]*8, runnedTime=-1, windowID=-1):
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
        # if runnedTime<0:
        #     runnedTime = time.time()-self.startTime
        checkSum = 1234567890
        # pack for SOut2SimulatorSimpleTime
        buf = struct.pack("3i14f4d", checkSum, copterID, vehicleType,
                          *PWMs, *VelE, *AngEuler, *PosE, runnedTime)
        # print(len(buf))
        if windowID < 0:
            if self.ip == '127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                # multicast address, send to all RflySim3Ds on all PC in LAN
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009))
        else:
            if self.ip != '127.0.0.1' and self.ip != '255.255.255.255':
                # ensure this PC can reciver message under specify IP mode
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID))
            # specify PC's IP to send
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))
        #print('Message Send')

    def sendUE4PosScale100(self, copterID, vehicleType, PosE, AngEuler, MotorRPMSMean, Scale, isFitGround=False, windowID=-1):
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
        """

        checksum = 1234567890
        if isFitGround:
            checksum = 1234567891
        buf = struct.pack("i200H600f300H100f", checksum, *copterID,
                          *vehicleType, *PosE, *AngEuler, *Scale, *MotorRPMSMean)
        if windowID < 0:
            if self.ip == '127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                # multicast address, send to all RflySim3Ds on all PC in LAN
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009))
        else:
            if self.ip != '127.0.0.1' and self.ip != '255.255.255.255':
                # ensure this PC can reciver message under specify IP mode
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID))
            # specify PC's IP to send
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))

    def sendUE4PosScalePwm20(self, copterID, vehicleType, PosE, AngEuler, Scale, PWMs, isFitGround=False, windowID=-1):
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
        """
        checksum = 1234567890
        if isFitGround:
            checksum = 1234567891
        buf = struct.pack("i40H120f60H160f", checksum, *copterID,
                          *vehicleType, *PosE, *AngEuler, *Scale, *PWMs)
        if windowID < 0:
            if self.ip == '127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                # multicast address, send to all RflySim3Ds on all PC in LAN
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009))
        else:
            if self.ip != '127.0.0.1' and self.ip != '255.255.255.255':
                # ensure this PC can reciver message under specify IP mode
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID))
            # specify PC's IP to send
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))

    def sendReqToCopterSim(self, srcs=SensorReqCopterSim(), copterID=1):
        """ send UDP message SensorReqCopterSim to CopterSim to request a sensor data
        the copterID specify the index of CopterSim to request
        """
        if type(srcs).__name__ != 'SensorReqCopterSim':
            print('Error: input is not SensorReqCopterSim class')
            return
        u16Value = [srcs.checksum, srcs.sensorType, srcs.updateFreq, srcs.port]
        u8Value = srcs.IP
        fValue = srcs.Params
        buf = struct.pack("4H4B6f", *u16Value, *u8Value, *fValue)
        self.udp_socket.sendto(buf, ('255.255.255.255', 30100+(copterID-1)*2))

    def sendImuReqCopterSim(self, copterID=1, IP='', port=31000, freq=200):
        """send command to CopterSim to request IMU data
        copterID is the CopterID
        IP is the IP that copterSim send back to
        port is the port that CopterSim send to 
        freq is the frequency of the send data
        This function will init a thread to listen IMU data
        """
        self.sendImuReqClient(copterID, IP, port, freq)
        self.sendImuReqServe(copterID, port)

    def sendImuReqClient(self, copterID=1, IP='', port=31000, freq=200):
        """send command to CopterSim to request IMU data
        copterID is the CopterID
        IP is the IP that copterSim send back to
        port is the port that CopterSim send to 
        freq is the frequency of the send data
        """

        # if RemotSendIP has been set, the IMU rev IP will be RemotSendIP
        # else use local IP address 127.0.0.1
        if IP == '' and self.RemotSendIP != '':
            IP = self.RemotSendIP
        elif IP == '':
            IP = '127.0.0.1'

        srcs = SensorReqCopterSim()
        srcs.sensorType = 0  
        srcs.updateFreq = freq
        if IP != '':
            cList = IP.split('.')
            if len(cList) == 4:
                srcs.IP[0] = int(cList[0])
                srcs.IP[1] = int(cList[1])
                srcs.IP[2] = int(cList[2])
                srcs.IP[3] = int(cList[3])
        srcs.port = port+copterID-1
        self.sendReqToCopterSim(srcs, copterID)  

    def sendImuReqServe(self, copterID=1, port=31000):
        """send command to CopterSim to request IMU data
        copterID is the CopterID
        port is the port that CopterSim send to 
        This function will init a thread to listen IMU data
        """
        self.udp_imu.bind(('0.0.0.0', port))
        self.tIMU = threading.Thread(target=self.getIMUDataLoop, args=())
        self.tIMU.start()

    def getIMUDataLoop(self):
        print("Start lisening to IMU Msg")
        while True:
            try:
                buf, addr = self.udp_imu.recvfrom(65500)
                if len(buf) == 40:
                    # print(len(buf[0:12]))
                    IMUData = struct.unpack('2i1d6f', buf)
                    if IMUData[0] == 1234567898:
                        self.imu.checksum = IMUData[0]
                        self.imu.seq = IMUData[1]
                        self.imu.timestmp = IMUData[2]
                        if self.isPrintTime:
                            self.lastIMUTime = time.time()
                            print('IMU:', self.imu.timestmp)
                        self.imu.acc[0] = IMUData[3]
                        self.imu.acc[1] = IMUData[4]
                        self.imu.acc[2] = IMUData[5]
                        self.imu.rate[0] = IMUData[6]
                        self.imu.rate[1] = IMUData[7]
                        self.imu.rate[2] = IMUData[8]
                        if not self.hasIMUData:
                            self.hasIMUData = True
                            print("Got CopterSim IMU Msg!")
                        if isEnableRosTrans and self.hasIMUData:
                            self.imu.Imu2ros()
                          

            except:
                print("Error to listen to IMU Msg!")
                sys.exit(0)

    def StartTimeStmplisten(self, cpID=1):
        """Start to listen to 20005 port to get RflyTimeStmp of CopterID
        if cpID == 0 then only current CopterID will be listened.
        if cpID >0 then timestmp of desired CopterID will be listened.
        """
        self.udp_time = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM)  
        self.udp_time.bind(('0.0.0.0', 20005))
        self.tTimeStmp = threading.Thread(target=self.TimeStmploop, args=())
        self.cpID = cpID
        self.tTimeStmp.start()

    def TimeStmploop(self):
        print("Start lisening to timeStmp Msg")
        while True:
            try:
                buf, addr = self.udp_time.recvfrom(65500)
                if len(buf) == 32:
                    # print(len(buf[0:12]))
                    TimeData = struct.unpack('2i3q', buf)
                    if TimeData[0] == 123456789:

                        cpIDTmp = TimeData[1]
                        if cpIDTmp == self.cpID:
                            self.RflyTime.checksum = TimeData[0]
                            self.RflyTime.copterID = TimeData[1]
                            self.RflyTime.SysStartTime = TimeData[2]
                            self.RflyTime.SysCurrentTime = TimeData[3]
                            self.RflyTime.HeartCount = TimeData[4]

            except:
                print("Error to listen to Time Msg!")
                sys.exit(0)

    def sendUpdateUEImage(self, vs=VisionSensorReq(), windID=0, IP='127.0.0.1'):
        if not isinstance(vs, VisionSensorReq):
            raise Exception("Wrong data input to addVisSensor()")
        intValue = [vs.checksum, vs.SeqID, vs.TypeID, vs.TargetCopter, vs.TargetMountType,
                    vs.DataWidth, vs.DataHeight, vs.DataCheckFreq]+vs.SendProtocol
        floValue = [vs.CameraFOV] + vs.SensorPosXYZ + \
            vs.SensorAngEular+vs.otherParams
        buf = struct.pack("16H15f", *intValue, *floValue)
        self.udp_socket.sendto(buf, (IP, 20010+windID))
        if self.RemotSendIP != '' and self.RemotSendIP != '127.0.0.1':
            self.udp_socket.sendto(buf, (self.RemotSendIP, 20010+windID))

    def sendReqToUE4(self, windID=0):
        """ send VisSensor list to RflySim3D to request image
        windID specify the index of RflySim3D window to send image
        """

        if len(self.VisSensor) <= 0:
            print('Error: No sensor is obtained.')
            return False

        # EmptyMem = np.zeros(66,dtype=np.int).tolist()
        # buf = struct.pack("66i",*EmptyMem)
        # self.mm0.seek(0)
        # self.mm0.write(buf)
        # self.mm0.seek(0)
        contSeq0 = False
        if self.isUE4DirectUDP or self.RemotSendIP != '':
            for i in range(len(self.VisSensor)):
                if self.VisSensor[i].SendProtocol[0] == 0:  
                    self.VisSensor[i].SendProtocol[0] = 1
                if self.RemotSendIP != '' and (self.VisSensor[i].SendProtocol[0] == 1 or self.VisSensor[i].SendProtocol[0] == 2 or self.VisSensor[i].SendProtocol[0] == 3):
                    cList = self.RemotSendIP.split('.')
                    if len(cList) == 4:
                        self.VisSensor[i].SendProtocol[1] = int(cList[0])
                        self.VisSensor[i].SendProtocol[2] = int(cList[1])
                        self.VisSensor[i].SendProtocol[3] = int(cList[2])
                        self.VisSensor[i].SendProtocol[4] = int(cList[3])
                if self.VisSensor[i].SeqID == 0:
                    contSeq0 = True

        if contSeq0:
            self.sendUE4Cmd(b'RflyClearCapture', windID)

        for i in range(len(self.VisSensor)):

            vs = self.VisSensor[i]
            intValue = [vs.checksum, vs.SeqID, vs.TypeID, vs.TargetCopter, vs.TargetMountType,
                        vs.DataWidth, vs.DataHeight, vs.DataCheckFreq]+vs.SendProtocol
            floValue = [vs.CameraFOV] + vs.SensorPosXYZ + \
                vs.SensorAngEular+vs.otherParams
            buf = struct.pack("16H15f", *intValue, *floValue)
            self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windID))

        time.sleep(1)

        if isLinux:  
            # Linux mmap
            # SHARE_MEMORY_FILE_SIZE_BYTES = 66*4
            f = open('/dev/shm/UE4CommMemData', 'r+b')
            fd = f.fileno()
            self.mm0 = mmap.mmap(fd, 66*4)
        else:  # Windows下共享内存代码
            self.mm0 = mmap.mmap(0, 66*4, 'UE4CommMemData')  

        Data = np.frombuffer(self.mm0, dtype=np.int32)
        checksum = Data[0]
        totalNum = Data[1]
        ckCheck = False
        # print(Data)
        if checksum == 1234567890:
            ckCheck = True
            for i in range(len(self.VisSensor)):
                isSucc = False
                vs = self.VisSensor[i]
                idx = vs.SeqID
                width = Data[2+idx*2]
                height = Data[2+idx*2+1]
                if width == vs.DataWidth and height == vs.DataHeight:
                    if idx <= totalNum:
                        isSucc = True
                if not isSucc:
                    ckCheck = False
                    break
        if not ckCheck:
            print('Error: Sensor req failed from UE4.')
            return False
        print('Sensor req success from UE4.')
        self.hasReqUE4 = True
        return True

    def img_udp_thrdNew(self, udpSok, idx, typeID):
        CheckSum = 1234567890
        fhead_size = struct.calcsize('4i1d')
        imgPackUnit = 60000

        seqList = []
        dataList = []
        timeList = []
        recPackNum = 0
        timeStmpStore = 0

        while True:
            buf, addr = udpSok.recvfrom(imgPackUnit+2000)  
            # print(len(buf))
            if len(buf) < fhead_size:  
                continue
            dd = struct.unpack('4i1d', buf[0:fhead_size])  
            # print(dd)
            if dd[0] != CheckSum or dd[1] != len(buf):  
                print('Wrong Data!')
                continue
            packSeq = dd[2]  
            if packSeq == 0:  
                seqList = []  
                dataList = []  
                seqList = seqList + [packSeq]  
                dataList = dataList+[buf[fhead_size:]]  
                timeStmpStore = dd[4] 
                recPackNum = dd[3] 
            else:  
                if recPackNum == 0:
                    continue

               
                if not math.isclose(timeStmpStore, dd[4], rel_tol=0.00001):
                    continue  
                seqList = seqList + [packSeq]  
                dataList = dataList+[buf[fhead_size:]]  
            # if typeID==2:
                # print(seqList,recPackNum,len(dataList))
            if len(seqList) == recPackNum:  
                recPackNum = 0
                #print('Start Img Cap')
                data_total = b''
                dataOk = True
                for i in range(len(seqList)):
                    if seqList.count(i) < 1:
                        dataOk = False  
                        print('Failed to process img pack')
                        break
                    idx0 = seqList.index(i)  
                    data_total = data_total+dataList[idx0]
                # if typeID==2:
                #    print(len(data_total))
                if dataOk:  
                    # if typeID==2:
                    #    print('Start img cap',self.VisSensor[idx].SendProtocol[0])
                    if self.VisSensor[idx].SendProtocol[0] == 1 or self.VisSensor[idx].SendProtocol[0] == 3:
                        if self.VisSensor[idx].TypeID == 1 or self.VisSensor[idx].TypeID == 2 or self.VisSensor[idx].TypeID == 3:
                            nparr = np.frombuffer(data_total, np.uint8)
                            colorType = cv2.IMREAD_COLOR
                            if typeID == 2:
                                colorType = cv2.IMREAD_ANYDEPTH
                            elif typeID == 3:
                                colorType = cv2.IMREAD_GRAYSCALE
                            self.Img[idx] = cv2.imdecode(nparr, colorType)
                            if self.Img[idx] is None:
                                print('Wrong Img decode!')
                                self.hasData[idx] = False
                            else:
                                self.hasData[idx] = True
                                self.timeStmp[idx] = timeStmpStore
                                if self.isPrintTime:
                                    dTime = time.time()-self.lastIMUTime
                                    print('Img', idx, ':', '{:.5f}'.format(
                                        timeStmpStore), ', dTimeIMU: ', dTime)

                        if self.VisSensor[idx].SendProtocol[0] == 1 and (self.VisSensor[idx].TypeID == 4 or self.VisSensor[idx].TypeID == 5 or self.VisSensor[idx].TypeID == 6):
                            # print('')
                            posAng = np.frombuffer(
                                data_total, dtype=np.float32, count=6)  # pos ang
                            PointNum = np.frombuffer(
                                data_total, dtype=np.int32, count=1, offset=4*6)  # num
                            PointNum = PointNum[0]
                            #print('PointNum: ',PointNum)
                            #print('posAng: ', posAng)
                            self.ImgData[idx] = posAng.tolist() + [PointNum]

                            L = np.frombuffer(
                                data_total, dtype=np.int16, count=PointNum*3, offset=4*7)  # cloud
                            # reshape array to 4 channel image array H X W X 4
                            self.Img[idx] = L.reshape(PointNum, 3)
                            self.Img[idx] = self.Img[idx] / 32767.0 * \
                                self.VisSensor[idx].otherParams[0]
                            self.hasData[idx] = True
                            self.timeStmp[idx] = timeStmpStore

                    elif self.VisSensor[idx].SendProtocol[0] == 2:
                        dtyp = np.uint8
                        dim = 3
                        if(typeID == 1):
                            dtyp = np.uint8
                            dim = 3
                        elif(typeID == 2):
                            dtyp = np.uint16
                            dim = 1
                        elif(typeID == 3):
                            dtyp = np.uint8
                            dim = 1
                        DataWidth = self.VisSensor[idx].DataWidth
                        DataHeight = self.VisSensor[idx].DataHeight
                        L = np.frombuffer(data_total, dtype=dtyp)
                        # colorType=cv2.IMREAD_COLOR
                        # if typeID==2 or typeID==3:
                        #     colorType=cv2.IMREAD_GRAYSCALE
                        # self.Img[idx] = cv2.imdecode(nparr, colorType)
                        self.Img[idx] = L.reshape(DataHeight, DataWidth, dim)
                        self.hasData[idx] = True
                        self.timeStmp[idx] = timeStmpStore
                        if self.isPrintTime:
                            dTime = time.time()-self.lastIMUTime
                            print('Img', idx, ':', timeStmpStore,
                                  ', dTimeIMU: ', dTime)

                    if isEnableRosTrans and self.hasData[idx]:  
                        if(self.VisSensor[idx].TypeID >= 1):  
                            self.imu.AlignTime(timeStmpStore) 
                        topic_name = "/rflysim/sensor" + \
                            str(self.VisSensor[idx].SeqID)
                        frame_id = "map" 
                       
                        if(len(self.VisSensor) == self.sensors_num):
                            frame_id = self.sensors_frame_id[idx]
                        header = std_msg.Header()
                        header.frame_id = frame_id
                        if(self.time_record[idx] < 0.0000001):
                            self.time_record[idx] = timeStmpStore
                            self.rostime[idx] = rospy.Time.now()
                            continue
                        header.stamp = self.rostime[idx] + rospy.Duration(
                            timeStmpStore-self.time_record[idx])
                        type_id = self.VisSensor[idx].TypeID

                        # print('Img',idx,':',header.stamp.to_sec())

                        type = Any
                        msg = Any
                        if(type_id == 1 or type_id == 2 or type_id == 3):
                            encoding_ = "bgr8"
                            type = sensor.Image
                            msg = sensor.Image()
                            byte_num = 1
                            msg.header = header
                            if(type_id == 1):
                                topic_name += "/img_rgb"
                                # msg.encoding = "bgr8"
                                byte_num = 3
                            elif(type_id == 2):
                                encoding_ = "mono16"
                                topic_name += "/img_depth"
                                byte_num = 2
                            else:
                                encoding_ = "mono8"
                                topic_name += "/img_gray"
                            msg.height = self.Img[idx].shape[0]
                            msg.width = self.Img[idx].shape[1]
                            msg.encoding = encoding_
                            msg.data = self.Img[idx].tostring()
                            msg.step = msg.width * byte_num
                            # print(encoding_)
                        if(type_id == 4 or type_id == 5 or type_id == 6):
                            type = sensor.PointCloud2
                            msg = sensor.PointCloud2()
                            msg.header = header
                            if(type_id == 4):
                                topic_name += "/vehicle_lidar"
                            if(type_id == 5):
                                topic_name += "/global_lidar"
                            if(type_id == 6):
                                topic_name += "/livox_lidar"
                            msg.height = 1
                            msg.width = self.Img[idx].shape[0]
                            msg.fields = [
                                sensor.PointField(
                                    'x', 0, sensor.PointField.FLOAT32, 1),
                                sensor.PointField(
                                    'y', 4, sensor.PointField.FLOAT32, 1),
                                sensor.PointField(
                                    'z', 8, sensor.PointField.FLOAT32, 1)
                            ]
                            msg.is_bigendian = False
                            msg.point_step = 12
                            msg.row_step = msg.point_step * \
                                self.Img[idx].shape[0]
                            msg.is_dense = False
                            msg.data = np.asarray(
                                self.Img[idx], np.float32).tostring()
                        pub = rospy.Publisher(topic_name, type, queue_size=10)
                        pub.publish(msg)

    def img_mem_thrd(self, idxList):
        mmList = []
        for i in range(len(idxList)):
            idx = idxList[i]
            SeqID = self.VisSensor[idx].SeqID
            DataWidth = self.VisSensor[idx].DataWidth
            DataHeight = self.VisSensor[idx].DataHeight
            typeID = self.VisSensor[idx].TypeID
            dim = 3
            dimSize = 1
            otherSize = 0
            if(typeID == 1):
                dim = 3
                dimSize = 1
            elif(typeID == 2):
                dim = 1
                dimSize = 2
            elif(typeID == 3):
                dim = 1
                dimSize = 1
            elif(typeID == 4 or typeID == 5 or typeID == 6):
                dim = 3
                dimSize = 2
                otherSize = 4*7
            if isLinux:
                # Linux
                dataLen = DataWidth*DataHeight*dim*dimSize+1+8+otherSize
                f = open("/dev/shm/" + 'RflySim3DImg_' + str(SeqID), 'r+b')
                fd = f.fileno()
                mm = mmap.mmap(fd, dataLen)
                #mm = mmap_file.read(SHARE_MEMORY_FILE_SIZE_BYTES)
            else:
                mm = mmap.mmap(0, DataWidth*DataHeight*dim*dimSize +
                               1+8+otherSize, 'RflySim3DImg_'+str(SeqID))
            mmList = mmList+[mm]
        # cv2.IMWRITE_PAM_FORMAT_GRAYSCALE
        while True:
            if AutoREG.ALLSTATE.ALL_DOWN == True:
                sys.exit(0)
            for kk in range(len(idxList)):
                mm = mmList[kk]
                idx = idxList[kk]
                DataWidth = self.VisSensor[idx].DataWidth
                DataHeight = self.VisSensor[idx].DataHeight
                typeID = self.VisSensor[idx].TypeID
                dtyp = np.uint8
                dim = 3
                if(typeID == 1):
                    dtyp = np.uint8
                    dim = 3
                elif(typeID == 2):
                    dtyp = np.uint16
                    dim = 1
                elif(typeID == 3):
                    dtyp = np.uint8
                    dim = 1
                elif(typeID == 4 or typeID == 5 or typeID == 6):
                    dtyp = np.int16
                    dim = 3
                for ii in range(3):  
                    flag = np.frombuffer(mm, dtype=np.uint8, count=1)
                    # print(flag[0])
                    if(flag[0] == 2): 
                        # print(flag[0])
                        mm.seek(0)
                        mm.write_byte(3) 

                        
                        #L=np.frombuffer(mm,dtype = np.uint8)
                        # struct.unpack('d',L[1:9]) 
                        self.timeStmp[idx] = np.frombuffer(
                            mm, dtype=np.float64, count=1, offset=1)
                        if self.isPrintTime:
                            dTime = time.time()-self.lastIMUTime
                            print('Img', idx, ':',
                                  self.timeStmp[idx], ', dTimeIMU: ', dTime)

                        mm.seek(0)
                        mm.write_byte(4)  
                        if typeID == 1 or typeID == 2 or typeID == 3:
                            L = np.frombuffer(mm, dtype=dtyp, offset=9)
                            # reshape array to 4 channel image array H X W X 4
                            self.Img[idx] = L.reshape(
                                DataHeight, DataWidth, dim)
                            if len(self.Img[idx]) > 0:
                                self.hasData[idx] = True
                            if self.isRemoteSend:
                                self.sendImgUDPNew(idx)

                        elif typeID == 4 or typeID == 5 or typeID == 6:
                            posAng = np.frombuffer(
                                mm, dtype=np.float32, count=6, offset=9)  # pos ang
                            PointNum = np.frombuffer(
                                mm, dtype=np.int32, count=1, offset=9+4*6)  # num
                            PointNum = PointNum[0]
                            #print('PointNum: ',PointNum)
                            #print('posAng: ', posAng)
                            self.ImgData[idx] = posAng.tolist() + [PointNum]

                            L = np.frombuffer(
                                mm, dtype=dtyp, count=PointNum*dim, offset=9+4*7)  # cloud
                            # reshape array to 4 channel image array H X W X 4
                            self.Img[idx] = L.reshape(PointNum, dim)
                            self.Img[idx] = self.Img[idx] / 32767.0 * \
                                self.VisSensor[idx].otherParams[0]
                            if len(self.Img[idx]) > 0:
                                self.hasData[idx] = True
                            if self.isRemoteSend:
                                # pos ang num cloud
                                L = np.frombuffer(
                                    mm, dtype=np.uint8, count=PointNum*dim*2+4*7, offset=9)
                                self.sendImgBuffer(idx, L.tostring())

                        
                        # print("readImg"+str(idx))
                        break
            time.sleep(0.001)

    def startImgCap(self, isRemoteSend=False):
        """ start loop to receive image from UE4,
        isRemoteSend=true will forward image from memory to UDP port
        """
        self.isRemoteSend = isRemoteSend
        memList = []
        udpList = []
        if isEnableRosTrans:
            self.time_record = np.zeros(len(self.VisSensor))
            self.rostime = np.ndarray(len(self.time_record), dtype=rospy.Time)

        for i in range(len(self.VisSensor)):
            self.Img = self.Img + [0]
            self.ImgData = self.ImgData + [0]
            self.hasData = self.hasData + [False]
            self.timeStmp = self.timeStmp + [0]
            IP = str(self.VisSensor[i].SendProtocol[1])+'.'+str(self.VisSensor[i].SendProtocol[2])+'.'+str(
                self.VisSensor[i].SendProtocol[3])+'.'+str(self.VisSensor[i].SendProtocol[4])
            if IP == '0.0.0.0':
                IP = '127.0.0.1'
            if self.RemotSendIP != '':
                IP = self.RemotSendIP
            self.IpList = self.IpList + [IP]
            self.portList = self.portList + [self.VisSensor[i].SendProtocol[5]]
            if self.VisSensor[i].SendProtocol[0] == 0:
                memList = memList + [i]
            else:
                udpList = udpList + [i]

        if len(memList) > 0:
            self.t_menRec = threading.Thread(
                target=self.img_mem_thrd, args=(memList,))
            self.t_menRec.start()

        if len(udpList) > 0:
            #print('Enter UDP capture')
            for i in range(len(udpList)):
                udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                udp.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 60000*100)
                udp.bind(('0.0.0.0', self.portList[udpList[i]]))
                typeID = self.VisSensor[udpList[i]].TypeID
                t_udpRec = threading.Thread(
                    target=self.img_udp_thrdNew, args=(udp, udpList[i], typeID,))
                t_udpRec.start()

    def sendImgUDPNew(self, idx):
        img_encode = cv2.imencode('.png', self.Img[idx])[1]
        data_encode = np.array(img_encode)
        data = data_encode.tostring()
        self.sendImgBuffer(idx, data)

    def sendImgBuffer(self, idx, data):
        imgPackUnit = 60000
        imgLen = len(data)
        imgpackNum = imgLen//imgPackUnit+1
        IP = self.IpList[idx]
        if self.RemotSendIP != '':
            IP = self.RemotSendIP

        CheckSum = 1234567890
        timeStmpSend = self.timeStmp[idx]

       
        for i in range(imgpackNum):
            dataSend = []
            if imgPackUnit*(i+1) > len(data):  
                dataSend = data[imgPackUnit*i:]
            else: 
                dataSend = data[imgPackUnit*i:imgPackUnit*(i+1)]
            PackLen = 4*4+8*1+len(dataSend) 
            fhead = struct.pack('4i1d', CheckSum, PackLen, i,
                                imgpackNum, timeStmpSend)  
            dataSend = fhead+dataSend  
            self.udp_socket.sendto(dataSend, (IP, self.portList[idx]))  

    def jsonLoad(self, ChangeMode=-1, jsonPath=''):
        """ load config.json file to create camera list for image capture,
        if ChangeMode>=0, then the SendProtocol[0] will be set to ChangeMode to change the transfer style
        """
        if len(jsonPath) == 0:
            jsonPath = sys.path[0]+'/Config.json'
        else:
            if jsonPath.find(':')==-1:
                jsonPath = sys.path[0]+'/'+jsonPath
        if not os.path.exists(jsonPath):
            print("The json file does not exist!")
            return False
        with open(jsonPath, "r", encoding='utf-8') as f:
            jsData = json.loads(f.read())
            if len(jsData["VisionSensors"]) <= 0:
                print("No sensor data is found!")
                return False
            for i in range(len(jsData["VisionSensors"])):
                visSenStruct = VisionSensorReq()
                if isinstance(jsData["VisionSensors"][i]["SeqID"], int):
                    visSenStruct.SeqID = jsData["VisionSensors"][i]["SeqID"]
                else:
                    print("Json data format is wrong!")
                    continue

                if isinstance(jsData["VisionSensors"][i]["TypeID"], int):
                    visSenStruct.TypeID = jsData["VisionSensors"][i]["TypeID"]
                else:
                    print("Json data format is wrong!")
                    continue

                if isinstance(jsData["VisionSensors"][i]["TargetCopter"], int):
                    visSenStruct.TargetCopter = jsData["VisionSensors"][i]["TargetCopter"]
                else:
                    print("Json data format is wrong!")
                    continue

                if isinstance(jsData["VisionSensors"][i]["TargetMountType"], int):
                    visSenStruct.TargetMountType = jsData["VisionSensors"][i]["TargetMountType"]
                else:
                    print("Json data format is wrong!")
                    continue

                if isinstance(jsData["VisionSensors"][i]["DataWidth"], int):
                    visSenStruct.DataWidth = jsData["VisionSensors"][i]["DataWidth"]
                else:
                    print("Json data format is wrong!")
                    continue

                if isinstance(jsData["VisionSensors"][i]["DataHeight"], int):
                    visSenStruct.DataHeight = jsData["VisionSensors"][i]["DataHeight"]
                else:
                    print("Json data format is wrong!")
                    continue

                if isinstance(jsData["VisionSensors"][i]["DataCheckFreq"], int):
                    visSenStruct.DataCheckFreq = jsData["VisionSensors"][i]["DataCheckFreq"]
                else:
                    print("Json data format is wrong!")
                    continue

                if isinstance(jsData["VisionSensors"][i]["CameraFOV"], float) or isinstance(jsData["VisionSensors"][i]["CameraFOV"], int):
                    visSenStruct.CameraFOV = jsData["VisionSensors"][i]["CameraFOV"]
                else:
                    print("Json data format is wrong!")
                    continue

                if len(jsData["VisionSensors"][i]["SendProtocol"]) == 8:
                    visSenStruct.SendProtocol = jsData["VisionSensors"][i]["SendProtocol"]
                    if ChangeMode != -1:
                        
                        visSenStruct.SendProtocol[0] = ChangeMode
                else:
                    print("Json data format is wrong!")
                    continue

                if len(jsData["VisionSensors"][i]["SensorPosXYZ"]) == 3:
                    visSenStruct.SensorPosXYZ = jsData["VisionSensors"][i]["SensorPosXYZ"]
                else:
                    print("Json data format is wrong!")
                    continue

                if len(jsData["VisionSensors"][i]["SensorAngEular"]) == 3:
                    visSenStruct.SensorAngEular = jsData["VisionSensors"][i]["SensorAngEular"]
                else:
                    print("Json data format is wrong!")
                    continue

                if len(jsData["VisionSensors"][i]["otherParams"]) == 8:
                    visSenStruct.otherParams = jsData["VisionSensors"][i]["otherParams"]
                else:
                    print("Json data format is wrong!")
                    continue
                self.VisSensor = self.VisSensor+[visSenStruct]
        if(len(self.VisSensor)) <= 0:
            print("No sensor is obtained.")
            return False
        print('Got', len(self.VisSensor), 'vision sensors from json')
        return True

    def initUE4MsgRec(self):
        """ Initialize the UDP data linsening from UE4,
        currently, the crash data is listened
        """
        self.stopFlagUE4 = False
        self.inSilVect = []
        self.inReqVect = []
        MYPORT = 20006
        MYGROUP = '224.0.0.10'
        ANY = '0.0.0.0'
        sock = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.udp_socketUE4.setsockopt(
            socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_socketUE4.bind((ANY, MYPORT))
        status = self.udp_socketUE4.setsockopt(socket.IPPROTO_IP,
                                               socket.IP_ADD_MEMBERSHIP,
                                               socket.inet_aton(MYGROUP) + socket.inet_aton(ANY))
        self.t4 = threading.Thread(target=self.UE4MsgRecLoop, args=())
        self.t4.start()

    def endUE4MsgRec(self):
        """ End UE4 message listening
        """
        self.stopFlagUE4 = True
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
                    buf, addr = self.udp_socketUE4.recvfrom(65500)
                    #print('Data Received!')
                    if len(buf) == 12:
                        checksum, CopterID, targetID = struct.unpack(
                            'iii', buf[0:12])
                        if checksum == 1234567890:
                            if targetID > -0.5 and CopterID == self.CopterID:
                                self.isVehicleCrash = True
                                self.isVehicleCrashID = targetID
                            print('Vehicle #', CopterID,
                                  ' Crashed with vehicle #', targetID)

                    if len(buf) == 120:
                        iValue = struct.unpack('10i20f', buf[0:120])
                        if iValue[0] == 1234567897:
                            isCopterExist = False
                            for i in range(len(self.inSilVect)): 
                                if self.inSilVect[i].CopterID == iValue[1]: 
                                    isCopterExist = True
                                    self.inSilVect[i].checksum = iValue[0]
                                    self.inSilVect[i].inSILInts = iValue[2:10]
                                    self.inSilVect[i].inSILFLoats = iValue[10:30]
                                    break
                            if not isCopterExist:  
                                vsr = PX4SILIntFloat(iValue)
                                self.inSilVect = self.inSilVect + \
                                    [copy.deepcopy(vsr)]  

                    if len(buf) == 160:

                        isCopterExist = False
                        iValue = struct.unpack('4i1d29f20s', buf[0:160])
                        if(iValue[0] == 1234567897):
                            vsr = reqVeCrashData(iValue)
                            # print(vsr.copterID,vsr.vehicleType)
                            for i in range(len(self.inReqVect)):  
                                if self.inReqVect[i].copterID == iValue[1]: 
                                    isCopterExist = True
                                    self.inReqVect[i] = copy.deepcopy(vsr)
                                    self.inReqUpdateVect[i] = True
                                    break
                            if not isCopterExist: 
                                self.inReqVect = self.inReqVect + \
                                    [copy.deepcopy(vsr)] 
                                self.inReqUpdateVect = self.inReqUpdateVect + \
                                    [True]

                except:
                    self.stopFlagUE4 = True
                    print('Error Data')

                    break

    def getUE4Pos(self, CopterID=1):
        if self.stopFlagUE4:  
            self.initUE4MsgRec()
            time.sleep(1)

        for i in range(len(self.inReqVect)):  
            if self.inReqVect[i].copterID == CopterID:
                posE = self.inReqVect[i].PosE
                return [posE[0], posE[1], posE[2], 1]
        return [0, 0, 0, 0]

    def getUE4Data(self, CopterID=1):
        if self.stopFlagUE4: 
            self.initUE4MsgRec()
            time.sleep(1)

        for i in range(len(self.inReqVect)):  
            if self.inReqVect[i].copterID == CopterID:
                return self.inReqVect[i]
        return 0
