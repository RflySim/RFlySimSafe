%InitFunction
load MavLinkStruct;
% Define the 32-D ModelInParams vector for external modification
FaultParamAPI.FaultInParams = zeros(32,1);

%steering engine struct
SteerEngineFault.FaultID = 123450;
SteerEngineFault.SteerNum = int32(5);
% Motor Fault Struct
MotorFault.FaultID=123451;
MotorFault.MotorNum = int32(1);
%Battery Fault Struct
BatteryFault.UseCusTomHoverTimeFaultID = 123452;
BatteryFault.PowOffFaultID = 123453;
BatteryFault.LowVoltageFaultID = 123454;
BatteryFault.LowCapacityFaultID = 123455;
%Load Fault Struct
LoadFault.LoadFallFaultID = 123456;
LoadFault.LoadShiftFaultID = 123457;
LoadFault.LoadLeakFaultID = 123458;
%Wind Fault Struct
WindFault.ConstWindFaultID = 123459;
WindFault.GustWindFaultID = 123540;
WindFault.TurbWindFaultID = 123541;
WindFault.SheerWindFaultID = 123542;
WindFault.NoiseWindFaultID = 123543;
%Sensor Fault Struct
SensorFault.AccNoiseFaultID = 123544;
SensorFault.GyroNoiseFaultID = 123545;
SensorFault.MagNoiseFaultID = 123546;
SensorFault.BaroNoiseFaultID = 123547;
SensorFault.GPSNoiseFaultID = 123548;


FaultParamAPI.FaultInParams(1)=1; % Band-Limited White Noise,FaultID = 123450
FaultParamAPI.FaultInParams(2)=2;
FaultParamAPI.FaultInParams(3)=2;
FaultParamAPI.FaultInParams(4)=3; %Wind speed at 6 m defines the low-altitude intensity (m/s),FaultID = 123541
FaultParamAPI.FaultInParams(5)=1; %Wind direction at 6 m (degrees clockwise from north)��FaultID = 123541
FaultParamAPI.FaultInParams(6)=3; %Wind speed at 6 m altitude (m/s)��FaultID = 123542
FaultParamAPI.FaultInParams(7)=1; %Wind direction at 6 m altitude (degrees clockwise from north)��FaultID = 123542
FaultParamAPI.FaultInParams(8)=2; %Wind speed at 6 m defines the low-altitude intensity (m/s)��FaultID = 123540
FaultParamAPI.FaultInParams(9)=1; %Wind direction at 6 m (degrees clockwise from north)��FaultID = 123540
FaultParamAPI.FaultInParams(10)=0.707; %Accelerometer damping ratio
FaultParamAPI.FaultInParams(11)=0; % Accelerometer measurement bias-x
FaultParamAPI.FaultInParams(12)=0; % Accelerometer measurement bias-y
FaultParamAPI.FaultInParams(13)=0; % Accelerometer measurement bias-z
FaultParamAPI.FaultInParams(14)=0.707; % Gyro damping ratio
FaultParamAPI.FaultInParams(15)=0; % Gyro measurement bias-x
FaultParamAPI.FaultInParams(16)=0; % Gyro measurement bias-y
FaultParamAPI.FaultInParams(17)=0; % Gyro measurement bias-z


%Initial condition
ModelInit_PosE=[0;0;0];
ModelInit_VelB=[1.000000000000000e-03;0;0];
ModelInit_AngEuler=[0;0;0];
ModelInit_RateB=[0;0;0];
ModelInit_Inputs = [0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0];

ModelParam_uavType = int8(100); %������������X�ͣ����嶨����ĵ�"���Ͷ����ĵ�.docx"
%Environment Parameter
ModelParam_envLongitude = 116.259368300000;
ModelParam_envLatitude = 40.1540302;
ModelParam_GPSLatLong = [ModelParam_envLatitude ModelParam_envLongitude];
ModelParam_envAltitude = -50;     %�ο��߶ȣ���ֵ


ModelParam_GlobalNoiseGainSwitch =0.4;
ModelParam_timeSampBaro = 0.01;
ModelParam_timeSampTurbWind = 0.01;
ModelParam_BusSampleRate = 0.001;
ModelParam_GPSEphFinal=0.3;
ModelParam_GPSEpvFinal=0.4;
ModelParam_GPSFix3DFix=3;
ModelParam_GPSSatsVisible=10;

%Noise Parameter
ModelParam_noisePowerAccel = [0.001,0.001,0.003];%˳�� xyz ��ͬ  ��Ҫ�޸�����
ModelParam_noiseSampleTimeAccel = 0.001;
%ModelParam_noiseLowPassFilterCoeAccel = 0.0001;
ModelParam_noisePowerOffGainAccel = 0.04;
ModelParam_noisePowerOffGainAccelZ = 0.03;
ModelParam_noisePowerOnGainAccel = 0.8;
ModelParam_noisePowerOnGainAccelZ = 4.5;


ModelParam_noisePowerGyro = [0.00001,0.00001,0.00001];%��Ҫ�޸�����
ModelParam_noiseSampleTimeGyro = 0.001;
%ModelParam_noiseLowPassFilterCoeGyro = 0.0001;
ModelParam_noisePowerOffGainGyro = 0.02;
ModelParam_noisePowerOffGainGyroZ = 0.025;
ModelParam_noisePowerOnGainGyro = 2;%3.2;
ModelParam_noisePowerOnGainGyroZ = 1;



ModelParam_noisePowerMag = [0.00001,0.00001,0.00001];%��Ҫ�޸�����
ModelParam_noiseSampleTimeMag = 0.01;
%ModelParam_noiseLowPassFilterCoeMag = 0.02;%��ʱû��ʹ��
ModelParam_noisePowerOffGainMag = 0.02;
ModelParam_noisePowerOffGainMagZ = 0.035;
ModelParam_noisePowerOnGainMag = 0.025;
ModelParam_noisePowerOnGainMagZ = 0.05;



ModelParam_noisePowerIMU=0;%IMU�����������ǰ������������Ǿ�����һ��

ModelParam_noiseUpperGPS=0.5;  %GPS��λ�������������������������x,y,z�Ĳ������ޣ���λ��m
ModelParam_noiseGPSSampTime=0.2;%Ĭ��0.05

ModelParam_noiseUpperBaro=0; %��ѹ������������������������߶ȵĲ������ޣ���λ��m
ModelParam_noiseBaroSampTime=0.5;%��ѹ����������Ƶ�ʣ���Ĭ��0.05
ModelParam_noiseBaroCoupleWithSpeed=0;%��ѹ�Ʋ����߶��붯ѹ��ϵ��Ҳ���Ƿ�������ѹ�Ƶ���ģ�͵�ϵ������ǰ����0.008�ɻ�10m/s����1m

ModelParam_noiseUpperWindBodyRatio=0;%�粨��ϵ��������*(1+��ϵ��)
ModelParam_noiseWindSampTime=0.001;


%%ModelParam_envGravityAcc = 9.81;
ModelParam_envAirDensity = 1.225;    %��û���õ�
ModelParam_envDiffPressure = 0; % Differential pressure (airspeed) in millibar
ModelParam_noiseTs = 0.001;