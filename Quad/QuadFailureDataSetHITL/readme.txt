(一)ControlSequence Description
The ControlSequence consists of different instructions, where each instruction ends with a semicolon (;), and each instruction consists of a different string.

The first bit of each instruction represents a manipulation class. The meaning of the existing control class is analyzed as follows:
1: Time class (including waiting time, waiting for reset function)
2: Control category (including arm,disarm, position control, speed control, landing, fault injection)
 
The second digit of each instruction represents the specific function, and the following digits represent the parameters of the corresponding function. The existing functions and meanings are analyzed as follows (taking unmanned ships as an example):
1: Time class:
	1: Waiting time: Wait (times)
	2: Wait for reset: WaitReset (targetpos)
2: Control class:
	1: Unlock: Arm (void)
	2: Lock: DisArm(void)
	3: Position control: PosCtrl (pos)
	4: Speed control: VelCtrl (pos)
	5: Spot landing: Land (pos)
	6: Fault injection: FaultInject(param)

An example of an quadrotor (motor failure) is as follows:
2,1;1,1,5;2,3,0,0,-15;1,1,15;2,6,123450,1,0.8;1,1,10
The analysis is as follows: (the first digit of each instruction represents a class, the second digit represents the corresponding function of this class, and the subsequent digits are the parameters of the function, if there is no parameter, it does not need to be set)
Arm(2,1) -> Wait for 5s (1,1,5) -> Send takeoff command, fly to [0,0,-15] (2,3,0,0,-15) -> Wait for 15s (1,1,15)  -> Fault injection, inject motor fault (123450 is motor fault) (2,6,123450,1,0.8), -> wait for 10s (1, 1, 10)


(二)Data Set Description
Data Set: ST001
Subsystem: Power
FaultID: 123450
FaultType: Motor Fault
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,6,123450,1,0.8;1,1,10

Data Set: ST002
Subsystem: Power
FaultID: 123450
FaultType: Motor Fault
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,6,123450,1,0.6;1,1,10

Data Set: ST003
Subsystem: Power
FaultID: 123450
FaultType: Motor Fault
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,6,123450,1,0.4;1,1,10

Data Set: ST004
Subsystem: Power
FaultID: 123450
FaultType: Motor Fault
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,6,123450,1,0.2;1,1,10

Data Set: ST005
Subsystem: Power
FaultID: 123450
FaultType: Motor Fault
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,6,123450,1,0;1,1,10

Data Set: ST006
Subsystem: Battery
FaultID: 123453
FaultType: Battery Fault
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,6,123453;1,1,10

Data Set: ST007
Subsystem: Battery
FaultID: 123454
FaultType: Low Voltage
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,6,123454,0.5;1,1,10

Data Set: ST008
Subsystem: Battery
FaultID: 123454
FaultType: Low Voltage
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,6,123454,0;1,1,10

Data Set: ST009
Subsystem: Battery
FaultID: 123455
FaultType: Low Power
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,6,123455,0.5;1,1,10

Data Set: ST010
Subsystem: Battery
FaultID: 123455
FaultType: Low Power
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,6,123455,0;1,1,10

Data Set: ST011
Subsystem: GPS
FaultID: 123548
FaultType: Noise Interference
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,3,250,0,-15;2,6,123548,123548,20,3,10;1,1,15

Data Set: ST012
Subsystem: GPS
FaultID: 123548
FaultType: Noise Interference
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,3,250,0,-15;2,6,123548,123548,50,3,10;1,1,15

Data Set: ST013
Subsystem: GPS
FaultID: 123548
FaultType: Noise Interference
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,3,250,0,-15;2,6,123548,123548,150,3,10;1,1,15

Data Set: ST014
Subsystem: GPS
FaultID: 123548
FaultType: Search Star Fault
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,3,250,0,-15;2,6,123548,123548,0,3,5;1,1,15

Data Set: ST015
Subsystem: GPS
FaultID: 123548
FaultType: Search Star Fault
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,3,250,0,-15;2,6,123548,123548,0,3,0;1,1,15

Data Set: ST016
Subsystem: Load
FaultID: 123456
FaultType: Load Leak
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,6,123456,1.5;1,1,15

Data Set: ST017
Subsystem: Load
FaultID: 123458
FaultType: Load Drift
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,6,123458,0.7,0.5;1,1,15

Data Set: ST018
Subsystem: Load
FaultID: 123458
FaultType: Load Drift
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,6,123458,1,0.5;1,1,15

Data Set: ST019
Subsystem: Wind
FaultID: 123459
FaultType: Const Wind
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,6,123459,123459,3,4,5;1,1,15

Data Set: ST020
Subsystem: Wind
FaultID: 123540
FaultType: Gust Wind
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,6,123540,15,30;1,1,15

Data Set: ST021
Subsystem: Wind
FaultID: 123542
FaultType: Turbulent Wind
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,6,123542,5;1,1,15

Data Set: ST022
Subsystem: Sensor
FaultID: 123544
FaultType: Accelerometer Noise Gain
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,6,123544,5;1,1,15

Data Set: ST023
Subsystem: Sensor
FaultID: 123544
FaultType: Accelerometer Noise Gain
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,6,123544,15;1,1,15

Data Set: ST024
Subsystem: Sensor
FaultID: 123544
FaultType: Accelerometer Noise Gain
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,6,123544,30;1,1,15

Data Set: ST025
Subsystem: Sensor
FaultID: 123545
FaultType: Gyroscope Noise Gain
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,6,123545,5;1,1,15

Data Set: ST026
Subsystem: Sensor
FaultID: 123545
FaultType: Gyroscope Noise Gain
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,6,123545,15;1,1,15

Data Set: ST027
Subsystem: Sensor
FaultID: 123545
FaultType: Gyroscope Noise Gain
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,6,123545,30;1,1,15

Data Set: ST028
Subsystem: Sensor
FaultID: 123546
FaultType: Magnetometer Noise Gain
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,6,123546,10;1,1,15

Data Set: ST029
Subsystem: Sensor
FaultID: 123546
FaultType: Magnetometer Noise Gain
ControlSequence: 2,1;1,1,5;2,3,0,0,-15;1,1,15;2,6,123546,30;1,1,15

(三)Data Description
Each data folder contains flight log data and truth data. The truth data includes position, velocity, angular velocity and GPS data.
The truth position data file is the position data of the x, y, and z axes from left to right;
The truth velocity data file is the velocity data of the x, y, and z axes from left to right;
The truth angular velocity data files are the angular velocity data of roll, pitch, and yaw from left to right;
The truth GPS data files are longitude, latitude and height data from left to right;

