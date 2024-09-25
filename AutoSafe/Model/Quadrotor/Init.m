load MavLinkStruct;
%load path;

% Define the 32-D ModelInParams vector for external modification
FaultParamAPI.FaultInParams = zeros(32,1);

MotorFaultTemp.FaultID=123450;
MotorFaultTemp.MotorNum=int32(4);

% Motor Fault Struct
MotorFault1=MotorFaultTemp;
%Prop Fault Stuct
PropFault.FaultID = 123451;
PropFault.PropNum = int32(4);
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
FaultParamAPI.FaultInParams(5)=1; %Wind direction at 6 m (degrees clockwise from north)，FaultID = 123541
FaultParamAPI.FaultInParams(6)=3; %Wind speed at 6 m altitude (m/s)，FaultID = 123542
FaultParamAPI.FaultInParams(7)=1; %Wind direction at 6 m altitude (degrees clockwise from north)，FaultID = 123542
FaultParamAPI.FaultInParams(8)=2; %Wind speed at 6 m defines the low-altitude intensity (m/s)，FaultID = 123540
FaultParamAPI.FaultInParams(9)=1; %Wind direction at 6 m (degrees clockwise from north)，FaultID = 123540
FaultParamAPI.FaultInParams(10)=0.707; %Accelerometer damping ratio
FaultParamAPI.FaultInParams(11)=0; % Accelerometer measurement bias-x
FaultParamAPI.FaultInParams(12)=0; % Accelerometer measurement bias-y
FaultParamAPI.FaultInParams(13)=0; % Accelerometer measurement bias-z
FaultParamAPI.FaultInParams(14)=0.707; % Gyro damping ratio
FaultParamAPI.FaultInParams(15)=0; % Gyro measurement bias-x
FaultParamAPI.FaultInParams(16)=0; % Gyro measurement bias-y
FaultParamAPI.FaultInParams(17)=0; % Gyro measurement bias-z



%Initial condition
ModelInit_PosE=[0,0,0];
ModelInit_VelB=[0,0,0];
ModelInit_AngEuler=[0,0,0];
ModelInit_RateB=[0,0,0];
ModelInit_RPM = 0; %Initial motor speed (rad/s)
ModelInit_Inputs = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
%Model Parameter
% UAV TYPE.
ModelParam_uavType = int8(3); %这里是四旋翼X型，具体定义见文档"机型定义文档.docx"
ModelParam_uavMotNumbs = int8(4);
%ModelParam_ControlMode = int8(1); %整型 1表示Auto模式，0表示Manual模式
ModelParam_motorMinThr=0.05;
ModelParam_motorCr=842.1;
ModelParam_motorWb=22.83;
ModelParam_motorT= 0.0214;%0.0261;
ModelParam_motorJm =0.0001287;
ModelParam_rotorCm=2.783e-07;
ModelParam_rotorCt=1.681e-05;

%ModelParam_uavCtrlEn = int8(0);
ModelParam_uavCd = 0.055;
ModelParam_uavCCm = [0.0035 0.0039 0.0034];
ModelParam_uavMass=1.515;%%1.5;
ModelParam_uavDearo = 0.12;%%unit m
ModelParam_uavR=0.225;
ModelParam_uavJxx = 0.0211;%%1.491E-2;
ModelParam_uavJyy = 0.0219;%%1.491E-2;
ModelParam_uavJzz = 0.0366;%%2.712E-2;
ModelParam_uavJ= [ModelParam_uavJxx,0,0;0,ModelParam_uavJyy,0;0,0,ModelParam_uavJzz];
ModelParam_GlobalNoiseGainSwitch =0.4;

%Environment Parameter
ModelParam_envLongitude = 116.259368300000;
ModelParam_envLatitude = 40.1540302;
ModelParam_GPSLatLong = [ModelParam_envLatitude ModelParam_envLongitude];
ModelParam_envAltitude = -50;     %参考高度，负值


%SimParam.timeACC=1;
%SimParam.timeStep=0.0001;
% SimParam.sonarSamp=0.1;
% SimParam.AngEularSamp=0.004;
% SimParam.AngRateSamp=0.004;
% SimParam.AccSensSamp=0.004;
% SimParam.GyroSensSamp=0.004;
% SimParam.AngQuaternSamp=0.004;
% SimParam.BaroSamp=0.008;
% SimParam.AngEulerSamp=0.004;
ModelParam_timeSampBaro = 0.01;
ModelParam_timeSampTurbWind = 0.01;
ModelParam_BusSampleRate = 0.001;



%%%ModelParam_BattModelEnable=int8(0);
%ModelParam_BattAuxCurrent=0.5;
%ModelParam_BattCells=3;
%ModelParam_BattCapacity=0.55;   %%这一项从模型配置界面传输给定
ModelParam_BattHoverMinutes=18;
ModelParam_BattHoverThr=0.609;

%GPS Parameter
ModelParam_GPSEphFinal=0.3;
ModelParam_GPSEpvFinal=0.4;
ModelParam_GPSFix3DFix=3;
ModelParam_GPSSatsVisible=10;


%Noise Parameter
ModelParam_noisePowerAccel = [0.001,0.001,0.003];%顺序 xyz 下同  不要修改这里
ModelParam_noiseSampleTimeAccel = 0.001;
%ModelParam_noiseLowPassFilterCoeAccel = 0.0001;
ModelParam_noisePowerOffGainAccel = 0.04;
ModelParam_noisePowerOffGainAccelZ = 0.03;
ModelParam_noisePowerOnGainAccel = 0.8;
ModelParam_noisePowerOnGainAccelZ = 4.5;


ModelParam_noisePowerGyro = [0.00001,0.00001,0.00001];%不要修改这里
ModelParam_noiseSampleTimeGyro = 0.001;
%ModelParam_noiseLowPassFilterCoeGyro = 0.0001;
ModelParam_noisePowerOffGainGyro = 0.02;
ModelParam_noisePowerOffGainGyroZ = 0.025;
ModelParam_noisePowerOnGainGyro = 2;%3.2;
ModelParam_noisePowerOnGainGyroZ = 1;



ModelParam_noisePowerMag = [0.00001,0.00001,0.00001];%不要修改这里
ModelParam_noiseSampleTimeMag = 0.01;
%ModelParam_noiseLowPassFilterCoeMag = 0.02;%暂时没有使用
ModelParam_noisePowerOffGainMag = 0.02;
ModelParam_noisePowerOffGainMagZ = 0.035;
ModelParam_noisePowerOnGainMag = 0.025;
ModelParam_noisePowerOnGainMagZ = 0.05;



ModelParam_noisePowerIMU=0;%IMU噪声，这里是白噪声，这里是经过归一化

ModelParam_noiseUpperGPS=0.5;  %GPS定位误差噪声，均匀噪声，这里填x,y,z的波动上限，单位是m
ModelParam_noiseGPSSampTime=0.2;%默认0.05

ModelParam_noiseUpperBaro=0; %气压计噪声，均匀噪声，这里填高度的波动上限，单位是m
ModelParam_noiseBaroSampTime=0.5;%气压计噪声更新频率，，默认0.05
ModelParam_noiseBaroCoupleWithSpeed=0;%气压计测量高度与动压关系，也就是风速与气压计掉高模型的系数，当前参数0.008飞机10m/s掉高1m

ModelParam_noiseUpperWindBodyRatio=0;%风波动系数，风速*(1+该系数)
ModelParam_noiseWindSampTime=0.001;




%%ModelParam_envGravityAcc = 9.81;
ModelParam_envAirDensity = 1.225;    %还没有用到
ModelParam_envDiffPressure = 0; % Differential pressure (airspeed) in millibar
ModelParam_noiseTs = 0.001;

%ModelParam_FailModelStartT = 5;
%ModelParam_FailModelLastT = 5;

ModelFailEnable = 0;

