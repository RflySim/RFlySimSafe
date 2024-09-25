//
// File: QuadModelv.cpp
//
// Code generated for Simulink model 'QuadModelv'.
//
// Model version                  : 7.223
// Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
// C/C++ source code generated on : Fri Dec  9 15:07:01 2022
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "QuadModelv.h"
#include "rtwtypes.h"
#include <cmath>
#include <cstring>
#include <cfloat>
#include <stddef.h>
#define NumBitsPerChar                 8U

// Private macros used by the generated code to access rtModel
#ifndef rtmSetFirstInitCond
#define rtmSetFirstInitCond(rtm, val)  ((rtm)->Timing.firstInitCondFlag = (val))
#endif

#ifndef rtmIsFirstInitCond
#define rtmIsFirstInitCond(rtm)        ((rtm)->Timing.firstInitCondFlag)
#endif

#ifndef rtmIsMajorTimeStep
#define rtmIsMajorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
#define rtmIsMinorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
#define rtmSetTPtr(rtm, val)           ((rtm)->Timing.t = (val))
#endif

#ifndef GRAVITY2_TYPEDEF

typedef enum { WGS84TAYLORSERIES = 1, WGS84CLOSEAPPROX,
  WGS84EXACT } GravityTypeIdx;

typedef enum { METRIC = 1, ENGLISH } UnitIdx;

typedef enum { JANUARY = 1, FEBRUARY, MARCH, APRIL, MAY, JUNE, JULY,
  AUGUST, SEPTEMBER, OCTOBER, NOVEMBER, DECEMBER } MonthIdx;

#define GRAVITY2_TYPEDEF
#endif                                 // GRAVITY2_TYPEDEF

#ifndef WGS84_DEFINE
#define WGS84_A                        6378137.0                 // Semi-major Axis (m) 
#define WGS84_INV_F                    298.257223563             // Reciprocal of Flattening 
#define WGS84_W_DEF                    7292115.0e-11             // WGS 84 Angular Velocity of Earth (rad/sec)
#define WGS84_GM_DEF                   3986004.418e+8            // Earth's Gravitational Const. (m^3/sec^2) 
#define WGS84_GM_PRM                   3986000.9e+8              // Earth's Grav. Const. (m^3/sec^2) [no atmos]
#define WGS84_W_PRM                    7292115.1467e-11          // IAU Angular Velocity of Earth (rad/sec) 
#define WGS84_G_E                      9.7803253359              // Theoretical (Normal) Gravity at the Equator
                                                                 // (on the Ellipsoid) (m/s^2) 
#define WGS84_K                        0.00193185265241          // Theoretical (Normal) Gravity Formula Const.
#define WGS84_E_2                      6.69437999014e-3          // First Eccentricity Squared 
#define WGS84_EL                       5.2185400842339e+5        // Linear Eccentricity 
#define WGS84_B                        6356752.3142              // Semi-minor Axis (m) 
#define WGS84_B_A                      0.996647189335            // Axis Ratio
#define M2FT                           1.0/0.3048
#define AERO_PI                        3.14159265358979323846
#define DEG2RAD                        AERO_PI/180.0
#define YEAR2000                       2000
#define WGS84_DEFINE
#endif                                 // WGS84_DEFINE

// Block parameters (default storage)
P_QuadModelv_T MulticopterModelClass::QuadModelv_P = {
  // Variable: FaultParamAPI
  //  Referenced by:
  //    '<S295>/Constant'
  //    '<S295>/Gain'
  //    '<S522>/Constant'
  //    '<S522>/Gain'
  //    '<S94>/Wind direction'
  //    '<S94>/Windspeed at 20ft (6m)'
  //    '<S95>/Wind direction'
  //    '<S95>/Windspeed at 20ft (6m)'
  //    '<S105>/Wind Direction'
  //    '<S105>/Wind speed at reference height'
  //    '<S298>/Gust start time'
  //    '<S524>/Gust start time'
  //    '<S433>/2*zeta * wn'
  //    '<S444>/2*zeta * wn'

  {
    { 1.0, 2.0, 2.0, 3.0, 1.0, 3.0, 1.0, 2.0, 1.0, 0.707, 0.0, 0.0, 0.0, 0.707,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0 }
  },

  // Variable: PropFault
  //  Referenced by: '<S522>/FaultID1'

  {
    123451.0,
    4
  },

  // Variable: MotorFault1
  //  Referenced by: '<S295>/MotorNum'

  {
    123450.0,
    4
  },

  // Variable: ModelFailEnable
  //  Referenced by: '<S12>/ModelFailEnable'

  0.0,

  // Variable: ModelInit_AngEuler
  //  Referenced by: '<S15>/Initial Euler Angles'

  { 0.0, 0.0, 0.0 },

  // Variable: ModelInit_PosE
  //  Referenced by: '<S14>/xe,ye,ze'

  { 0.0, 0.0, 0.0 },

  // Variable: ModelInit_RPM
  //  Referenced by: '<S296>/ModelInit_RPM'

  0.0,

  // Variable: ModelInit_RateB
  //  Referenced by: '<S14>/p,q,r '

  { 0.0, 0.0, 0.0 },

  // Variable: ModelInit_VelB
  //  Referenced by: '<S14>/ub,vb,wb'

  { 0.0, 0.0, 0.0 },

  // Variable: ModelParam_BattHoverMinutes
  //  Referenced by: '<S2>/ModelParam.BattHoverMinutes'

  18.0,

  // Variable: ModelParam_BattHoverThr
  //  Referenced by: '<S2>/ModelParam.BattHoverThr'

  0.609,

  // Variable: ModelParam_GPSEphFinal
  //  Referenced by: '<S330>/ModelParam.GPSEphFinal'

  0.3,

  // Variable: ModelParam_GPSEpvFinal
  //  Referenced by: '<S330>/ModelParam.GPSEpvFinal'

  0.4,

  // Variable: ModelParam_GPSFix3DFix
  //  Referenced by: '<S330>/ModelParam_GPSFix3DFix'

  3.0,

  // Variable: ModelParam_GPSLatLong
  //  Referenced by:
  //    '<S63>/ref_position'
  //    '<S379>/ref_position'

  { 40.1540302, 116.2593683 },

  // Variable: ModelParam_GPSSatsVisible
  //  Referenced by: '<S330>/ModelParam.GPSSatsVisible'

  10.0,

  // Variable: ModelParam_envAltitude
  //  Referenced by:
  //    '<S5>/ModelParam.envAltitude'
  //    '<S374>/ModelParam.envAltitude'

  -50.0,

  // Variable: ModelParam_motorCr
  //  Referenced by: '<S296>/ModelParam.motorCr'

  842.1,

  // Variable: ModelParam_motorJm
  //  Referenced by: '<S7>/ModelParam.motorJm'

  0.0001287,

  // Variable: ModelParam_motorMinThr
  //  Referenced by:
  //    '<S313>/Constant'
  //    '<S315>/Constant'
  //    '<S317>/Constant'
  //    '<S319>/Constant'
  //    '<S321>/Constant'
  //    '<S323>/Constant'
  //    '<S325>/Constant'
  //    '<S327>/Constant'

  0.05,

  // Variable: ModelParam_motorT
  //  Referenced by: '<S296>/ModelParam.motorT'

  0.0214,

  // Variable: ModelParam_motorWb
  //  Referenced by: '<S296>/ModelParam.motorWb'

  22.83,

  // Variable: ModelParam_noisePowerOffGainAccel
  //  Referenced by: '<S403>/AccelNoiseGainFunction'

  0.04,

  // Variable: ModelParam_noisePowerOffGainAccelZ
  //  Referenced by: '<S403>/AccelNoiseGainFunction'

  0.03,

  // Variable: ModelParam_noisePowerOffGainGyro
  //  Referenced by: '<S403>/GyroNoiseGainFunction'

  0.02,

  // Variable: ModelParam_noisePowerOffGainGyroZ
  //  Referenced by: '<S403>/GyroNoiseGainFunction'

  0.025,

  // Variable: ModelParam_noisePowerOffGainMag
  //  Referenced by: '<S403>/MagNoiseGainFunction'

  0.02,

  // Variable: ModelParam_noisePowerOffGainMagZ
  //  Referenced by: '<S403>/MagNoiseGainFunction'

  0.035,

  // Variable: ModelParam_noisePowerOnGainAccel
  //  Referenced by: '<S403>/AccelNoiseGainFunction'

  0.8,

  // Variable: ModelParam_noisePowerOnGainAccelZ
  //  Referenced by: '<S403>/AccelNoiseGainFunction'

  4.5,

  // Variable: ModelParam_noisePowerOnGainGyro
  //  Referenced by: '<S403>/GyroNoiseGainFunction'

  2.0,

  // Variable: ModelParam_noisePowerOnGainGyroZ
  //  Referenced by: '<S403>/GyroNoiseGainFunction'

  1.0,

  // Variable: ModelParam_noisePowerOnGainMag
  //  Referenced by: '<S403>/MagNoiseGainFunction'

  0.025,

  // Variable: ModelParam_noisePowerOnGainMagZ
  //  Referenced by: '<S403>/MagNoiseGainFunction'

  0.05,

  // Variable: ModelParam_rotorCm
  //  Referenced by: '<S7>/ModelParam.rotorCm'

  2.783E-7,

  // Variable: ModelParam_rotorCt
  //  Referenced by: '<S7>/ModelParam.rotorCt'

  1.681E-5,

  // Variable: ModelParam_uavCCm
  //  Referenced by: '<S7>/ModelParam.uavCCm'

  { 0.0035, 0.0039, 0.0034 },

  // Variable: ModelParam_uavCd
  //  Referenced by: '<S7>/ModelParam.uavCd'

  0.055,

  // Variable: ModelParam_uavDearo
  //  Referenced by: '<S7>/ModelParam.uavDearo'

  0.12,

  // Variable: ModelParam_uavJ
  //  Referenced by: '<S6>/uavJ'

  { 0.0211, 0.0, 0.0, 0.0, 0.0219, 0.0, 0.0, 0.0, 0.0366 },

  // Variable: ModelParam_uavMass
  //  Referenced by: '<S6>/ModelParam.uavMass'

  1.515,

  // Variable: ModelParam_uavR
  //  Referenced by: '<S6>/ModelParam.uavR'

  0.225,

  // Variable: ModelParam_uavMotNumbs
  //  Referenced by: '<S2>/ModelParam.uavMotNumbs'

  4,

  // Variable: ModelParam_uavType
  //  Referenced by:
  //    '<S7>/ModelParam.uavType'
  //    '<S12>/UAVType'

  3,

  // Mask Parameter: BandLimitedWhiteNoise_Cov
  //  Referenced by: '<S297>/Output'

  1.0,

  // Mask Parameter: BandLimitedWhiteNoise_Cov_a
  //  Referenced by: '<S523>/Output'

  1.0,

  // Mask Parameter: DrydenWindTurbulenceModelDiscre
  //  Referenced by: '<S183>/Medium//High Altitude'

  533.4,

  // Mask Parameter: DrydenWindTurbulenceModelDisc_m
  //  Referenced by: '<S144>/Medium//High Altitude'

  533.4,

  // Mask Parameter: DrydenWindTurbulenceModelDisc_n
  //  Referenced by: '<S157>/White Noise'

  { 23301.0, 23542.0, 82443.0, 33344.0 },

  // Mask Parameter: DrydenWindTurbulenceModelDisc_i
  //  Referenced by: '<S118>/White Noise'

  { 23341.0, 23342.0, 23343.0, 23344.0 },

  // Mask Parameter: DrydenWindTurbulenceModelDisc_g
  //  Referenced by:
  //    '<S147>/Constant1'
  //    '<S147>/Constant2'
  //    '<S147>/Constant3'
  //    '<S148>/Constant3'

  1.0,

  // Mask Parameter: DrydenWindTurbulenceModelDisc_k
  //  Referenced by:
  //    '<S108>/Constant1'
  //    '<S108>/Constant2'
  //    '<S108>/Constant3'
  //    '<S109>/Constant3'

  1.0,

  // Mask Parameter: WhiteNoise_Ts
  //  Referenced by: '<S157>/Constant1'

  0.01,

  // Mask Parameter: WhiteNoise_Ts_a
  //  Referenced by: '<S118>/Constant1'

  0.01,

  // Mask Parameter: DrydenWindTurbulenceModelDis_n3
  //  Referenced by: '<S164>/Probability of  Exceedance'

  3.0,

  // Mask Parameter: DrydenWindTurbulenceModelDis_gr
  //  Referenced by: '<S125>/Probability of  Exceedance'

  4.0,

  // Mask Parameter: DrydenWindTurbulenceModelDis_np
  //  Referenced by: '<S94>/Wingspan'

  1.0,

  // Mask Parameter: DrydenWindTurbulenceModelDisc_d
  //  Referenced by: '<S95>/Wingspan'

  1.0,

  // Mask Parameter: ThreeaxisInertialMeasurementUni
  //  Referenced by: '<S427>/Measurement bias'

  { 0.0, 0.0, 0.0 },

  // Mask Parameter: ThreeaxisInertialMeasurementU_h
  //  Referenced by: '<S427>/Scale factors & Cross-coupling  errors'

  { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 },

  // Mask Parameter: DirectionCosineMatrixtoQuaterni
  //  Referenced by:
  //    '<S203>/If Warning//Error'
  //    '<S203>/Constant'

  1.0,

  // Mask Parameter: DirectionCosineMatrixtoQuater_p
  //  Referenced by:
  //    '<S241>/If Warning//Error'
  //    '<S241>/Constant'

  1.0,

  // Mask Parameter: DirectionCosineMatrixtoQuater_c
  //  Referenced by:
  //    '<S337>/If Warning//Error'
  //    '<S337>/Constant'

  1.0,

  // Mask Parameter: CompareToConstant_const
  //  Referenced by: '<S82>/Constant'

  180.0,

  // Mask Parameter: CompareToConstant_const_i
  //  Referenced by: '<S80>/Constant'

  90.0,

  // Mask Parameter: CompareToConstant_const_j
  //  Referenced by: '<S83>/Constant'

  180.0,

  // Mask Parameter: CompareToConstant_const_ji
  //  Referenced by: '<S76>/Constant'

  180.0,

  // Mask Parameter: CompareToConstant_const_k
  //  Referenced by: '<S74>/Constant'

  90.0,

  // Mask Parameter: CompareToConstant_const_f
  //  Referenced by: '<S77>/Constant'

  180.0,

  // Mask Parameter: CompareToConstant_const_n
  //  Referenced by: '<S460>/Constant'

  180.0,

  // Mask Parameter: CompareToConstant_const_a
  //  Referenced by: '<S458>/Constant'

  90.0,

  // Mask Parameter: CompareToConstant_const_e
  //  Referenced by: '<S461>/Constant'

  180.0,

  // Mask Parameter: CompareToConstant_const_b
  //  Referenced by: '<S395>/Constant'

  180.0,

  // Mask Parameter: CompareToConstant_const_g
  //  Referenced by: '<S393>/Constant'

  90.0,

  // Mask Parameter: CompareToConstant_const_iz
  //  Referenced by: '<S396>/Constant'

  180.0,

  // Mask Parameter: CompareToConstant_const_bz
  //  Referenced by: '<S389>/Constant'

  180.0,

  // Mask Parameter: CompareToConstant_const_d
  //  Referenced by: '<S387>/Constant'

  90.0,

  // Mask Parameter: CompareToConstant_const_ex
  //  Referenced by: '<S390>/Constant'

  180.0,

  // Mask Parameter: Distanceintogustx_d_m
  //  Referenced by: '<S298>/Distance into gust (x)'

  120.0,

  // Mask Parameter: Distanceintogusty_d_m
  //  Referenced by: '<S298>/Distance into gust (y)'

  120.0,

  // Mask Parameter: Distanceintogustz_d_m
  //  Referenced by: '<S298>/Distance into gust (z)'

  80.0,

  // Mask Parameter: DiscreteWindGustModel_d_m
  //  Referenced by: '<S298>/pi//Gust length'

  { 120.0, 120.0, 80.0 },

  // Mask Parameter: Distanceintogustx_d_m_i
  //  Referenced by: '<S524>/Distance into gust (x)'

  120.0,

  // Mask Parameter: Distanceintogusty_d_m_g
  //  Referenced by: '<S524>/Distance into gust (y)'

  120.0,

  // Mask Parameter: Distanceintogustz_d_m_n
  //  Referenced by: '<S524>/Distance into gust (z)'

  80.0,

  // Mask Parameter: DiscreteWindGustModel_d_m_p
  //  Referenced by: '<S524>/pi//Gust length'

  { 120.0, 120.0, 80.0 },

  // Mask Parameter: ThreeaxisInertialMeasurementU_b
  //  Referenced by: '<S428>/Measurement bias'

  { 0.0, 0.0, 0.0 },

  // Mask Parameter: ThreeaxisInertialMeasurementU_c
  //  Referenced by: '<S428>/g-sensitive bias'

  { 0.0, 0.0, 0.0 },

  // Mask Parameter: ThreeaxisInertialMeasurementU_m
  //  Referenced by: '<S428>/Scale factors & Cross-coupling  errors '

  { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 },

  // Mask Parameter: ThreeaxisInertialMeasurementU_e
  //  Referenced by: '<S427>/wl_ins'

  { 0.0, 0.0, 0.0 },

  // Mask Parameter: CustomVariableMass6DOFQuaternio
  //  Referenced by: '<S26>/High Gain Quaternion Normalization'

  1.0,

  // Mask Parameter: CheckAltitude_max
  //  Referenced by: '<S445>/max_val'

  850000.0,

  // Mask Parameter: CheckLatitude_max
  //  Referenced by: '<S446>/max_val'

  90.0,

  // Mask Parameter: CheckLongitude_max
  //  Referenced by: '<S447>/max_val'

  180.0,

  // Mask Parameter: Istimewithinmodellimits_max
  //  Referenced by: '<S449>/max_val'

  5.0,

  // Mask Parameter: CheckAltitude_min
  //  Referenced by: '<S445>/min_val'

  -1000.0,

  // Mask Parameter: CheckLatitude_min
  //  Referenced by: '<S446>/min_val'

  -90.0,

  // Mask Parameter: CheckLongitude_min
  //  Referenced by: '<S447>/min_val'

  -180.0,

  // Mask Parameter: Istimewithinmodellimits_min
  //  Referenced by: '<S449>/min_val'

  0.0,

  // Mask Parameter: FlatEarthtoLLA_psi
  //  Referenced by: '<S63>/ref_rotation'

  0.0,

  // Mask Parameter: FlatEarthtoLLA_psi_e
  //  Referenced by: '<S379>/ref_rotation'

  0.0,

  // Mask Parameter: WhiteNoise_pwr
  //  Referenced by: '<S157>/Constant'

  { 0.01, 0.01, 0.01, 0.01 },

  // Mask Parameter: WhiteNoise_pwr_d
  //  Referenced by: '<S118>/Constant'

  { 0.01, 0.01, 0.01, 0.01 },

  // Mask Parameter: BandLimitedWhiteNoise_seed
  //  Referenced by: '<S297>/White Noise'

  23341.0,

  // Mask Parameter: BandLimitedWhiteNoise_seed_h
  //  Referenced by: '<S523>/White Noise'

  23341.0,

  // Mask Parameter: DirectionCosineMatrixtoQuater_i
  //  Referenced by: '<S203>/If Warning//Error'

  4.4408920985006262E-16,

  // Mask Parameter: DirectionCosineMatrixtoQuate_i5
  //  Referenced by: '<S241>/If Warning//Error'

  4.4408920985006262E-16,

  // Mask Parameter: DirectionCosineMatrixtoQuater_b
  //  Referenced by: '<S337>/If Warning//Error'

  4.4408920985006262E-16,

  // Mask Parameter: DiscreteWindGustModel_v_m
  //  Referenced by: '<S298>/Gust magnitude//2.0'

  { 3.5, 3.5, 3.0 },

  // Mask Parameter: DiscreteWindGustModel_v_m_b
  //  Referenced by: '<S524>/Gust magnitude//2.0'

  { 3.5, 3.5, 3.0 },

  // Mask Parameter: ThreeaxisInertialMeasurement_c1
  //  Referenced by:
  //    '<S433>/2*zeta * wn'
  //    '<S433>/wn^2'

  190.0,

  // Mask Parameter: ThreeaxisInertialMeasurementU_i
  //  Referenced by:
  //    '<S444>/2*zeta * wn'
  //    '<S444>/wn^2'

  190.0,

  // Mask Parameter: DiscreteWindGustModel_Gx
  //  Referenced by: '<S298>/Constant'

  true,

  // Mask Parameter: DiscreteWindGustModel_Gx_p
  //  Referenced by: '<S524>/Constant'

  true,

  // Mask Parameter: DiscreteWindGustModel_Gy
  //  Referenced by: '<S298>/Constant1'

  true,

  // Mask Parameter: DiscreteWindGustModel_Gy_e
  //  Referenced by: '<S524>/Constant1'

  true,

  // Mask Parameter: DiscreteWindGustModel_Gz
  //  Referenced by: '<S298>/Constant2'

  true,

  // Mask Parameter: DiscreteWindGustModel_Gz_o
  //  Referenced by: '<S524>/Constant2'

  true,

  // Expression: 1
  //  Referenced by: '<S43>/Constant'

  1.0,

  // Expression: 1
  //  Referenced by: '<S44>/Constant'

  1.0,

  // Expression: -90
  //  Referenced by: '<S72>/Bias'

  -90.0,

  // Expression: -1
  //  Referenced by: '<S72>/Gain'

  -1.0,

  // Expression: +90
  //  Referenced by: '<S72>/Bias1'

  90.0,

  // Expression: 180
  //  Referenced by: '<S75>/Bias'

  180.0,

  // Expression: -180
  //  Referenced by: '<S75>/Bias1'

  -180.0,

  // Expression: 180
  //  Referenced by: '<S73>/Bias'

  180.0,

  // Expression: -180
  //  Referenced by: '<S73>/Bias1'

  -180.0,

  // Expression: -90
  //  Referenced by: '<S78>/Bias'

  -90.0,

  // Expression: -1
  //  Referenced by: '<S78>/Gain'

  -1.0,

  // Expression: +90
  //  Referenced by: '<S78>/Bias1'

  90.0,

  // Expression: 360
  //  Referenced by: '<S81>/Constant2'

  360.0,

  // Expression: 180
  //  Referenced by: '<S81>/Bias'

  180.0,

  // Expression: -180
  //  Referenced by: '<S81>/Bias1'

  -180.0,

  // Expression: 180
  //  Referenced by: '<S69>/Constant'

  180.0,

  // Expression: 0
  //  Referenced by: '<S69>/Constant1'

  0.0,

  // Expression: 360
  //  Referenced by: '<S79>/Constant2'

  360.0,

  // Expression: 180
  //  Referenced by: '<S79>/Bias'

  180.0,

  // Expression: -180
  //  Referenced by: '<S79>/Bias1'

  -180.0,

  // Expression: 1
  //  Referenced by: '<S129>/Gain'

  1.0,

  // Expression: 1
  //  Referenced by: '<S137>/Gain'

  1.0,

  // Expression: 1
  //  Referenced by: '<S168>/Gain'

  1.0,

  // Expression: 1
  //  Referenced by: '<S176>/Gain'

  1.0,

  // Expression: -90
  //  Referenced by: '<S385>/Bias'

  -90.0,

  // Expression: -1
  //  Referenced by: '<S385>/Gain'

  -1.0,

  // Expression: +90
  //  Referenced by: '<S385>/Bias1'

  90.0,

  // Expression: 180
  //  Referenced by: '<S388>/Bias'

  180.0,

  // Expression: -180
  //  Referenced by: '<S388>/Bias1'

  -180.0,

  // Expression: 180
  //  Referenced by: '<S386>/Bias'

  180.0,

  // Expression: -180
  //  Referenced by: '<S386>/Bias1'

  -180.0,

  // Expression: -90
  //  Referenced by: '<S391>/Bias'

  -90.0,

  // Expression: -1
  //  Referenced by: '<S391>/Gain'

  -1.0,

  // Expression: +90
  //  Referenced by: '<S391>/Bias1'

  90.0,

  // Expression: 360
  //  Referenced by: '<S394>/Constant2'

  360.0,

  // Expression: 180
  //  Referenced by: '<S394>/Bias'

  180.0,

  // Expression: -180
  //  Referenced by: '<S394>/Bias1'

  -180.0,

  // Expression: 180
  //  Referenced by: '<S382>/Constant'

  180.0,

  // Expression: 0
  //  Referenced by: '<S382>/Constant1'

  0.0,

  // Expression: 360
  //  Referenced by: '<S392>/Constant2'

  360.0,

  // Expression: 180
  //  Referenced by: '<S392>/Bias'

  180.0,

  // Expression: -180
  //  Referenced by: '<S392>/Bias1'

  -180.0,

  // Expression: -90
  //  Referenced by: '<S456>/Bias'

  -90.0,

  // Expression: -1
  //  Referenced by: '<S456>/Gain'

  -1.0,

  // Expression: +90
  //  Referenced by: '<S456>/Bias1'

  90.0,

  // Expression: 180
  //  Referenced by: '<S459>/Bias'

  180.0,

  // Expression: -180
  //  Referenced by: '<S459>/Bias1'

  -180.0,

  // Expression: 180
  //  Referenced by: '<S457>/Bias'

  180.0,

  // Expression: -180
  //  Referenced by: '<S457>/Bias1'

  -180.0,

  // Expression: ones(1,maxdef+1)
  //  Referenced by: '<S478>/pp[13]'

  { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 },

  // Expression: 1
  //  Referenced by: '<S478>/Constant'

  1.0,

  // Expression: ones(1,maxdef+1)
  //  Referenced by: '<S479>/pp[13]'

  { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 },

  // Expression: k
  //  Referenced by: '<S479>/k[13][13]'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.33333333333333331,
    0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.26666666666666666, 0.2, 0.0, -0.33333333333333331, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.25714285714285712, 0.22857142857142856,
    0.14285714285714285, 0.0, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.25396825396825395, 0.23809523809523808, 0.19047619047619047,
    0.1111111111111111, 0.0, -0.14285714285714285, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.25252525252525254, 0.24242424242424243, 0.21212121212121213,
    0.16161616161616163, 0.090909090909090912, 0.0, -0.1111111111111111, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.25174825174825177, 0.24475524475524477,
    0.22377622377622378, 0.1888111888111888, 0.13986013986013987,
    0.076923076923076927, 0.0, -0.090909090909090912, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.25128205128205128, 0.24615384615384617, 0.23076923076923078,
    0.20512820512820512, 0.16923076923076924, 0.12307692307692308,
    0.066666666666666666, 0.0, -0.076923076923076927, 0.0, 0.0, 0.0, 0.0,
    0.25098039215686274, 0.24705882352941178, 0.23529411764705882,
    0.21568627450980393, 0.18823529411764706, 0.15294117647058825,
    0.10980392156862745, 0.058823529411764705, 0.0, -0.066666666666666666, 0.0,
    0.0, 0.0, 0.25077399380804954, 0.24767801857585139, 0.23839009287925697,
    0.22291021671826625, 0.20123839009287925, 0.17337461300309598,
    0.13931888544891641, 0.099071207430340563, 0.052631578947368418, 0.0,
    -0.058823529411764705, 0.0, 0.0, 0.25062656641604009, 0.24812030075187969,
    0.24060150375939848, 0.22807017543859648, 0.21052631578947367,
    0.18796992481203006, 0.16040100250626566, 0.12781954887218044,
    0.090225563909774431, 0.047619047619047616, 0.0, -0.052631578947368418, 0.0,
    0.25051759834368531, 0.2484472049689441, 0.24223602484472051,
    0.2318840579710145, 0.21739130434782608, 0.19875776397515527,
    0.17598343685300208, 0.14906832298136646, 0.11801242236024845,
    0.082815734989648032, 0.043478260869565216, 0.0, -0.047619047619047616 },

  // Expression: 0
  //  Referenced by: '<S474>/bpp'

  0.0,

  // Expression: fm(2)
  //  Referenced by: '<S474>/Constant'

  1.0,

  // Expression: 1
  //  Referenced by: '<S474>/Constant1'

  1.0,

  // Expression: ones(1,maxdef+1)
  //  Referenced by: '<S474>/Unit Delay1'

  { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 },

  // Expression: 1
  //  Referenced by: '<S482>/Constant'

  1.0,

  // Expression: 1
  //  Referenced by: '<S482>/Gain1'

  1.0,

  // Expression: 1
  //  Referenced by: '<S482>/Gain2'

  1.0,

  // Expression: 1
  //  Referenced by: '<S484>/Constant'

  1.0,

  // Expression: 1
  //  Referenced by: '<S485>/Constant'

  1.0,

  // Expression: 0
  //  Referenced by: '<S488>/Constant1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S488>/Constant'

  0.0,

  // Expression: k
  //  Referenced by: '<S488>/k[13][13]'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.33333333333333331,
    0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.26666666666666666, 0.2, 0.0, -0.33333333333333331, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.25714285714285712, 0.22857142857142856,
    0.14285714285714285, 0.0, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.25396825396825395, 0.23809523809523808, 0.19047619047619047,
    0.1111111111111111, 0.0, -0.14285714285714285, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.25252525252525254, 0.24242424242424243, 0.21212121212121213,
    0.16161616161616163, 0.090909090909090912, 0.0, -0.1111111111111111, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.25174825174825177, 0.24475524475524477,
    0.22377622377622378, 0.1888111888111888, 0.13986013986013987,
    0.076923076923076927, 0.0, -0.090909090909090912, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.25128205128205128, 0.24615384615384617, 0.23076923076923078,
    0.20512820512820512, 0.16923076923076924, 0.12307692307692308,
    0.066666666666666666, 0.0, -0.076923076923076927, 0.0, 0.0, 0.0, 0.0,
    0.25098039215686274, 0.24705882352941178, 0.23529411764705882,
    0.21568627450980393, 0.18823529411764706, 0.15294117647058825,
    0.10980392156862745, 0.058823529411764705, 0.0, -0.066666666666666666, 0.0,
    0.0, 0.0, 0.25077399380804954, 0.24767801857585139, 0.23839009287925697,
    0.22291021671826625, 0.20123839009287925, 0.17337461300309598,
    0.13931888544891641, 0.099071207430340563, 0.052631578947368418, 0.0,
    -0.058823529411764705, 0.0, 0.0, 0.25062656641604009, 0.24812030075187969,
    0.24060150375939848, 0.22807017543859648, 0.21052631578947367,
    0.18796992481203006, 0.16040100250626566, 0.12781954887218044,
    0.090225563909774431, 0.047619047619047616, 0.0, -0.052631578947368418, 0.0,
    0.25051759834368531, 0.2484472049689441, 0.24223602484472051,
    0.2318840579710145, 0.21739130434782608, 0.19875776397515527,
    0.17598343685300208, 0.14906832298136646, 0.11801242236024845,
    0.082815734989648032, 0.043478260869565216, 0.0, -0.047619047619047616 },

  // Expression: zeros(maxdef+1,maxdef+1)
  //  Referenced by: '<S472>/dp[13][13]'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0 },

  // Expression: snorm
  //  Referenced by: '<S472>/snorm[169]'

  { 1.0, 1.0, 1.5, 2.5, 4.375, 7.875, 14.4375, 26.8125, 50.2734375, 94.9609375,
    180.42578125, 344.44921875, 660.1943359375, 0.0, 1.0, 1.7320508075688772,
    3.0618621784789726, 5.5339859052946636, 10.16658128379447,
    18.903124741692839, 35.469603513959669, 67.03125, 127.40346687426536,
    243.28607380714598, 466.38644692864216, 897.027461585248, 0.0, 0.0,
    0.8660254037844386, 1.9364916731037085, 3.9131189606246322,
    7.685213074469698, 14.944232269507859, 28.960809996010127,
    56.082367403612253, 108.65004161512664, 210.69192030396434,
    409.04797337487776, 795.12986069746626, 0.0, 0.0, 0.0, 0.79056941504209488,
    2.0916500663351889, 4.7062126492541738, 9.9628215130052382,
    20.478385136833914, 41.41957332816817, 82.982839993569982,
    165.28034045942309, 327.9680080977904, 649.22081265302029, 0.0, 0.0, 0.0,
    0.0, 0.739509972887452, 2.2185299186623553, 5.4568620790707172,
    12.348930874776167, 26.736219617835371, 56.375738371688975,
    116.87084953567937, 239.5139682335957, 486.91560948976519, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.7015607600201138, 2.3268138086232852, 6.1744654373880836,
    14.830586268334102, 33.69094768709671, 73.915615322315773,
    158.42359886807964, 334.02135244518831, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.671693289381396, 2.4218245962496963, 6.8652274293172546, 17.39793057467611,
    41.320085114855779, 94.117642301250768, 208.29891011946015, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.6472598492877496, 2.5068266169601756,
    7.5335249254737544, 20.043185339772048, 49.604352946160631,
    117.05388227149012, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.6267066542400439, 2.5839777317091466, 8.1825961504123, 22.760038068635609,
    58.526941135745062, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.60904939217552367, 2.65478475211798, 8.8149248398872544,
    25.543251233216804, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.59362791713657326, 2.72034486491732, 9.4324706362690076, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.579979473934679,
    2.7814838439702596, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.56776801212685635 },

  // Expression: zeros(maxdef+1,maxdef+1)
  //  Referenced by: '<S472>/Unit Delay'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0 },

  // Expression: snorm
  //  Referenced by: '<S472>/Unit Delay1'

  { 1.0, 1.0, 1.5, 2.5, 4.375, 7.875, 14.4375, 26.8125, 50.2734375, 94.9609375,
    180.42578125, 344.44921875, 660.1943359375, 0.0, 1.0, 1.7320508075688772,
    3.0618621784789726, 5.5339859052946636, 10.16658128379447,
    18.903124741692839, 35.469603513959669, 67.03125, 127.40346687426536,
    243.28607380714598, 466.38644692864216, 897.027461585248, 0.0, 0.0,
    0.8660254037844386, 1.9364916731037085, 3.9131189606246322,
    7.685213074469698, 14.944232269507859, 28.960809996010127,
    56.082367403612253, 108.65004161512664, 210.69192030396434,
    409.04797337487776, 795.12986069746626, 0.0, 0.0, 0.0, 0.79056941504209488,
    2.0916500663351889, 4.7062126492541738, 9.9628215130052382,
    20.478385136833914, 41.41957332816817, 82.982839993569982,
    165.28034045942309, 327.9680080977904, 649.22081265302029, 0.0, 0.0, 0.0,
    0.0, 0.739509972887452, 2.2185299186623553, 5.4568620790707172,
    12.348930874776167, 26.736219617835371, 56.375738371688975,
    116.87084953567937, 239.5139682335957, 486.91560948976519, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.7015607600201138, 2.3268138086232852, 6.1744654373880836,
    14.830586268334102, 33.69094768709671, 73.915615322315773,
    158.42359886807964, 334.02135244518831, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.671693289381396, 2.4218245962496963, 6.8652274293172546, 17.39793057467611,
    41.320085114855779, 94.117642301250768, 208.29891011946015, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.6472598492877496, 2.5068266169601756,
    7.5335249254737544, 20.043185339772048, 49.604352946160631,
    117.05388227149012, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.6267066542400439, 2.5839777317091466, 8.1825961504123, 22.760038068635609,
    58.526941135745062, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.60904939217552367, 2.65478475211798, 8.8149248398872544,
    25.543251233216804, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.59362791713657326, 2.72034486491732, 9.4324706362690076, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.579979473934679,
    2.7814838439702596, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.56776801212685635 },

  // Expression: 0
  //  Referenced by: '<S472>/Merge1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S472>/Merge'

  0.0,

  // Expression: 1
  //  Referenced by: '<S501>/Gain'

  1.0,

  // Expression: zeros(maxdef+1,maxdef+1)
  //  Referenced by: '<S500>/zeros(maxdef+1,maxdef+1)'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0 },

  // Expression: zeros(maxdef+1,maxdef+1)
  //  Referenced by: '<S473>/tc[13][13]'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0 },

  // Expression: zeros(maxdef+1,maxdef+1)
  //  Referenced by: '<S473>/Unit Delay'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0 },

  // Expression: c
  //  Referenced by: '<S473>/c[maxdef][maxdef]'

  { 0.0, 4796.3, -4923.1812154337767, -348.13372969305919, 1567.7782069699783,
    476.81266220996059, -379.95280730802608, -1925.9994708080098, 677.015625,
    -2777.3955778589848, 802.84404356358175, -0.0, -897.027461585248, -29438.2,
    -1493.5, -553.21702793749932, 477.34519742006415, -738.01423597380563,
    1510.1443691332956, 490.17081843985773, -564.73579492219744,
    -1026.3073234861042, 1162.5554452818549, -84.276768121585746,
    859.00074408724333, 238.53895820923987, -3666.75, 5221.6135695778939,
    1454.0566529540724, -424.8520036436218, 377.9611669867686,
    -564.2748966455755, 588.80275141860955, 122.87031082100349,
    550.88032526463667, 979.19751192412582, 760.28956611334615,
    -196.78080485867423, 1168.5974627754365, 3379.5, -7200.2750989111519,
    2369.4912112096977, 460.34857037901179, -244.03829105285917,
    35.496478698597684, -366.1554455056451, 302.54880643201608,
    -387.67518445861288, -383.35502092748504, 514.23173795698926,
    -263.46536505695531, -1071.2143408774834, 3970.3125, 4509.091715634092,
    460.96541356158167, -701.95776226208943, 51.543845110255411,
    70.577012458023447, 18.84719184984861, 21.610629030858291,
    240.25549754701245, -232.4675390409673, -583.93336104629464,
    110.89651920765574, 100.20640573355649, -1834.0875, 3660.9859202943885,
    1473.2553463758411, -664.98784733961486, -348.75290321372222,
    5.4020178521548763, 41.577814612708416, -67.084541316116585,
    41.191364575903528, 137.44365153994127, -24.792051068913466,
    -18.823528460250156, 145.80923708362209, 1001.9625000000001,
    1279.7415450126052, 1080.4679930854181, -1286.2002573289762,
    -154.97488304560835, 31.644667797276679, -47.220038243512143,
    -1.8770535629344738, -23.062804876033614, 7.5335249254737544,
    -84.1813784270426, -104.16914118693732, -11.705388227149014, 2190.58125,
    -2692.1429067095391, -205.6217509716719, 1068.9717041427305,
    185.23396312164252, 56.187635480231556, -7.2654737887490892,
    3.8188331107977227, 1.5040959701761054, -10.077513153665672,
    -23.729528836195669, -34.140057102953413, 17.558082340723519, 1216.6171875,
    596.578125, -947.792009121047, -128.40067731732134, -553.43974608919211,
    197.24679736884357, 79.636638180080155, -40.861273856450865,
    -1.3160839739040922, 5.1769198334919508, -2.9202632273297784,
    -22.918804583706862, 5.1086502466433608, 522.28515625, 1121.1505084935352,
    325.95012484537995, -265.54508797942395, 33.825443023013385,
    -444.72050946967653, -1.739793057467611, 65.541666851621656,
    -23.514197358553233, -6.3341136786254468, -5.2239256708018447,
    -5.44068972983464, -8.489223572642107, -360.8515625, -1484.0450502235904,
    42.138384060792873, 99.168204275653849, -58.435424767839685,
    133.04810758016839, -28.924059580399042, 44.095007747498506,
    19.638230760989519, -4.7786125538123647, -2.1370605016916637,
    -1.3339527900497614, -0.556296768794052, 1033.34765625, -652.941025700099,
    -940.81033876221875, 688.73281700535983, -191.61117458687659,
    95.054159320847774, -65.882349610875536, 4.9604352946160635,
    38.692064716680534, -1.762984967977451, 1.088137945966928,
    2.0299281587713764, 0.45421440970148508, -1320.388671875, -89.7027461585248,
    397.56493034873313, 779.06497518362437, -438.22404854078866,
    300.61921720066948, 20.829891011946017, 70.232329362894077,
    -23.410776454298027, -12.771625616608402, 1.8864941272538016,
    -2.5033354595732336, -0.0 },

  // Expression: cd
  //  Referenced by: '<S473>/cd[maxdef][maxdef]'

  { 0.0, -30.2, -51.268703904038766, 19.902104160113321, -2.2135943621178655,
    2.0333162567588938, 5.6709374225078513, 21.2817621083758, -26.8125,
    -38.221040062279606, 0.0, 0.0, -0.0, 7.0, 9.0, -14.982239485470789,
    -1.5491933384829668, 22.696089971622865, 17.675990071280303,
    -22.41634840426179, 14.480404998005064, 33.649420442167347,
    10.865004161512665, 21.069192030396437, 40.904797337487778, 0.0, -16.5,
    -10.738715006927039, 0.25980762113533157, -1.5811388300841898,
    7.9482702520737174, -0.0, -11.955385815606286, -16.382708109467131,
    -4.1419573328168173, -33.193135997427994, -33.056068091884619, 0.0,
    -64.922081265302026, 6.0, -17.452614417330143, 3.872983346207417,
    -8.6962635654630436, -2.5882849051060819, 7.3211487315857724,
    2.1827448316282871, -2.4697861749552334, 16.041731770701222,
    16.912721511506692, 11.687084953567938, 23.951396823359573,
    48.691560948976523, -3.5, -4.9805873147651978, -25.435273244060109,
    10.876580344942983, -2.9580398915498081, -0.42093645601206825,
    0.46536276172465707, -6.7919119811268924, -2.9661172536668206,
    3.3690947687096711, -7.3915615322315773, -0.0, -0.0, -2.3625,
    6.0999487702766819, -6.1481704595757591, 0.47062126492541739,
    2.6622359023948263, 0.9821850640281593, 0.87320127619581489,
    0.24218245962496965, -3.4326137146586273, -0.0, 4.1320085114855782, -0.0,
    0.0, -11.55, -9.45156237084642, -1.494423226950786, 15.940514420808382,
    -8.7309793265131486, 0.0, 0.80603194725767524, 0.12945196985754992,
    1.2534133084800878, -0.75335249254737546, -0.0, 4.9604352946160635, -0.0,
    -8.04375, -7.0939207027919338, -8.6882429988030374, 18.430546623150523,
    1.2348930874776167, -3.70467926243285, -2.1796421366247265,
    0.45308189450142466, 0.062670665424004388, 1.2919888658545733,
    -0.81825961504123, -0.0, 0.0, -5.02734375, 13.40625, -11.216473480722451,
    20.709786664084085, -2.6736219617835371, 5.9322345073336411,
    2.7460909717269022, -0.25068266169601755, 0.25068266169601755,
    0.12180987843510474, 0.530956950423596, -0.88149248398872548, 0.0,
    -9.49609375, -12.740346687426538, -0.0, 33.193135997427994,
    -22.55029534867559, 0.0, 5.2193791724028324, 0.0, -0.0, -0.1827148176526571,
    -0.0, -0.0, -0.0, 0.0, -0.0, -21.069192030396437, 33.056068091884619,
    -11.687084953567938, -14.783123064463155, -0.0, -2.0043185339772047,
    -1.63651923008246, -0.265478475211798, -0.0, -0.0579979473934679, 0.0, -0.0,
    0.0, -0.0, 0.0, -0.0, -15.842359886807964, 0.0, -0.0, -0.0,
    -0.88149248398872548, -0.0, -0.0579979473934679, -0.056776801212685635, 0.0,
    0.0, -0.0, 0.0, -48.691560948976523, -0.0, 0.0, -0.0, 0.0, -0.0, -0.0, -0.0,
    -0.056776801212685635 },

  // Expression: zeros(maxdef+1,maxdef+1)
  //  Referenced by: '<S500>/Unit Delay'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0 },

  // Expression: 0
  //  Referenced by: '<S470>/bt'

  0.0,

  // Expression: 0
  //  Referenced by: '<S470>/bp'

  0.0,

  // Expression: 0
  //  Referenced by: '<S470>/br'

  0.0,

  // Expression: 0
  //  Referenced by: '<S470>/bpp'

  0.0,

  // Expression: 1
  //  Referenced by: '<S476>/Constant1'

  1.0,

  // Expression: 0
  //  Referenced by: '<S476>/Merge'

  0.0,

  // Expression: fm
  //  Referenced by: '<S471>/fm'

  { 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0 },

  // Expression: 0
  //  Referenced by: '<S476>/Merge1'

  0.0,

  // Expression: fn
  //  Referenced by: '<S471>/fn'

  { 0.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0 },

  // Expression: 0
  //  Referenced by: '<S477>/Constant1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S471>/Unit Delay1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S471>/Unit Delay3'

  0.0,

  // Expression: 0
  //  Referenced by: '<S471>/Unit Delay2'

  0.0,

  // Expression: 0
  //  Referenced by: '<S471>/Unit Delay4'

  0.0,

  // Expression: [0 0 0 0]
  //  Referenced by: '<S462>/bt,bp,br,bpp'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: 0
  //  Referenced by: '<S462>/Unit Delay'

  0.0,

  // Expression: [0 0 0 0]
  //  Referenced by: '<S462>/Unit Delay2'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: 6378.137
  //  Referenced by: '<S463>/r'

  6378.137,

  // Expression: 1
  //  Referenced by: '<S463>/ct'

  1.0,

  // Expression: 0
  //  Referenced by: '<S463>/st'

  0.0,

  // Expression: 0
  //  Referenced by: '<S463>/sa'

  0.0,

  // Expression: 0
  //  Referenced by: '<S463>/ca'

  0.0,

  // Expression: 6356.7523142
  //  Referenced by: '<S463>/b'

  6356.7523142,

  // Expression: 6378.137
  //  Referenced by: '<S463>/a'

  6378.137,

  // Expression: 2
  //  Referenced by: '<S508>/Gain'

  2.0,

  // Expression: 1
  //  Referenced by: '<S510>/Constant'

  1.0,

  // Expression: (1:(maxdef-1))
  //  Referenced by: '<S511>/sp[11]'

  { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0 },

  // Expression: (1:(maxdef-1))
  //  Referenced by: '<S511>/cp[11]'

  { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0 },

  // Expression: maxdef-1
  //  Referenced by: '<S511>/For Iterator'

  11.0,

  // Expression: [1:maxdef-1]
  //  Referenced by: '<S511>/Constant'

  { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0 },

  // Expression: 0
  //  Referenced by: '<S511>/Unit Delay1'

  0.0,

  // Expression: 1
  //  Referenced by: '<S511>/cp[m-1] sp[m-1]'

  1.0,

  // Expression: [1:maxdef-1]
  //  Referenced by: '<S511>/Constant1'

  { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0 },

  // Expression: [0 0 (1:(maxdef-1))]
  //  Referenced by: '<S464>/sp[13]'

  { 0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0 },

  // Expression: [1 1 (1:(maxdef-1))]
  //  Referenced by: '<S464>/cp[13]'

  { 1.0, 1.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0 },

  // Expression: 1
  //  Referenced by: '<S464>/Gain'

  1.0,

  // Expression: 1
  //  Referenced by: '<S464>/Gain1'

  1.0,

  // Expression: 1
  //  Referenced by: '<S464>/cp[1]'

  1.0,

  // Expression: 0
  //  Referenced by: '<S464>/sp[1]'

  0.0,

  // Expression: 1E6
  //  Referenced by: '<S12>/Gain_clock'

  1.0E+6,

  // Expression: 0
  //  Referenced by: '<S433>/Integrator, Second-Order Limited'

  0.0,

  // Expression: 0
  //  Referenced by: '<S433>/Integrator, Second-Order Limited'

  0.0,

  // Expression: dtype_a
  //  Referenced by: '<S429>/Constant'

  1.0,

  // Expression: 0.5
  //  Referenced by: '<S25>/1//2'

  0.5,

  // Expression: 2
  //  Referenced by: '<S30>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S33>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S28>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S34>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S29>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S32>/Gain'

  2.0,

  // Expression: 1
  //  Referenced by: '<S84>/Constant2'

  1.0,

  // Expression: R
  //  Referenced by: '<S84>/Constant1'

  6.378137E+6,

  // Expression: 1
  //  Referenced by: '<S87>/Constant'

  1.0,

  // Expression: 1
  //  Referenced by: '<S89>/Constant'

  1.0,

  // Expression: F
  //  Referenced by: '<S88>/Constant'

  0.0033528106647474805,

  // Expression: 1
  //  Referenced by: '<S88>/f'

  1.0,

  // Expression: 1
  //  Referenced by: '<S84>/Constant'

  1.0,

  // Expression: 1
  //  Referenced by: '<S84>/Constant3'

  1.0,

  // Expression: 360
  //  Referenced by: '<S75>/Constant2'

  360.0,

  // Expression: 180
  //  Referenced by: '<S68>/Constant'

  180.0,

  // Expression: 0
  //  Referenced by: '<S68>/Constant1'

  0.0,

  // Expression: 360
  //  Referenced by: '<S73>/Constant2'

  360.0,

  // Expression: 100000
  //  Referenced by: '<S5>/Saturation_1'

  100000.0,

  // Expression: 0
  //  Referenced by: '<S5>/Saturation_1'

  0.0,

  // Expression: juliandate(year,month,day)
  //  Referenced by: '<S66>/Julian Date'

  2.4532885E+6,

  // Expression: [1 0 0 0]
  //  Referenced by: '<S194>/Merge'

  { 1.0, 0.0, 0.0, 0.0 },

  // Expression: [1 0 0 0]
  //  Referenced by: '<S193>/Merge'

  { 1.0, 0.0, 0.0, 0.0 },

  // Expression: 0
  //  Referenced by: '<Root>/Memory'

  0.0,

  // Expression: 1/4.2
  //  Referenced by: '<S2>/MotorFault3'

  0.23809523809523808,

  // Expression: 0
  //  Referenced by: '<S297>/White Noise'

  0.0,

  // Computed Parameter: WhiteNoise_StdDev
  //  Referenced by: '<S297>/White Noise'

  1.0,

  // Expression: 1.0
  //  Referenced by: '<S298>/2'

  1.0,

  // Expression: 0
  //  Referenced by: '<S523>/White Noise'

  0.0,

  // Computed Parameter: WhiteNoise_StdDev_b
  //  Referenced by: '<S523>/White Noise'

  1.0,

  // Expression: 1.0
  //  Referenced by: '<S524>/2'

  1.0,

  // Expression: [1,1,1]
  //  Referenced by: '<S67>/Constant_[1 1 1]'

  { 1.0, 1.0, 1.0 },

  // Expression: [-1,-1,-0.5]
  //  Referenced by: '<S67>/Uniform Random Number'

  { -1.0, -1.0, -0.5 },

  // Expression: [1,1,0.5]
  //  Referenced by: '<S67>/Uniform Random Number'

  { 1.0, 1.0, 0.5 },

  // Expression: [564565,6846798,46545]
  //  Referenced by: '<S67>/Uniform Random Number'

  { 564565.0, 6.846798E+6, 46545.0 },

  // Expression: -1
  //  Referenced by: '<S67>/Gain_-1'

  -1.0,

  // Expression: 10000
  //  Referenced by: '<S67>/Saturation_2'

  10000.0,

  // Expression: 0
  //  Referenced by: '<S67>/Saturation_2'

  0.0,

  // Expression: 5
  //  Referenced by: '<S67>/Constant_V'

  5.0,

  // Expression: max_height_low
  //  Referenced by: '<S182>/Limit Function 10ft to 1000ft'

  1000.0,

  // Expression: 10
  //  Referenced by: '<S182>/Limit Function 10ft to 1000ft'

  10.0,

  // Expression: max_height_low
  //  Referenced by: '<S165>/Limit Height h<1000ft'

  1000.0,

  // Expression: 0
  //  Referenced by: '<S165>/Limit Height h<1000ft'

  0.0,

  // Expression: 0.1
  //  Referenced by: '<S165>/sigma_wg '

  0.1,

  // Expression: h_vec
  //  Referenced by: '<S164>/PreLook-Up Index Search  (altitude)'

  { 500.0, 1750.0, 3750.0, 7500.0, 15000.0, 25000.0, 35000.0, 45000.0, 55000.0,
    65000.0, 75000.0, 80000.0 },

  // Expression: [1:7]
  //  Referenced by: '<S164>/PreLook-Up Index Search  (prob of exceed)'

  { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0 },

  // Expression: sigma_vec'
  //  Referenced by: '<S164>/Medium//High Altitude Intensity'

  { 3.2, 2.2, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.2, 3.6, 3.3,
    1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.6, 6.9, 7.4, 6.7, 4.6, 2.7,
    0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 8.6, 9.6, 10.6, 10.1, 8.0, 6.6, 5.0, 4.2, 2.7,
    0.0, 0.0, 0.0, 11.8, 13.0, 16.0, 15.1, 11.6, 9.7, 8.1, 8.2, 7.9, 4.9, 3.2,
    2.1, 15.6, 17.6, 23.0, 23.6, 22.1, 20.0, 16.0, 15.1, 12.1, 7.9, 6.2, 5.1,
    18.7, 21.5, 28.4, 30.2, 30.7, 31.0, 25.2, 23.1, 17.5, 10.7, 8.4, 7.2 },

  // Expression: 0
  //  Referenced by: '<S157>/White Noise'

  0.0,

  // Computed Parameter: WhiteNoise_StdDev_j
  //  Referenced by: '<S157>/White Noise'

  1.0,

  // Expression: 1
  //  Referenced by: '<S154>/Lv'

  1.0,

  // Expression: 1
  //  Referenced by: '<S154>/Lw'

  1.0,

  // Expression: eye(3)
  //  Referenced by: '<S67>/Constant_DCM'

  { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 },

  // Expression: max_height_low
  //  Referenced by: '<S143>/Limit Function 10ft to 1000ft'

  1000.0,

  // Expression: 10
  //  Referenced by: '<S143>/Limit Function 10ft to 1000ft'

  10.0,

  // Expression: max_height_low
  //  Referenced by: '<S126>/Limit Height h<1000ft'

  1000.0,

  // Expression: 0
  //  Referenced by: '<S126>/Limit Height h<1000ft'

  0.0,

  // Expression: 0.1
  //  Referenced by: '<S126>/sigma_wg '

  0.1,

  // Expression: h_vec
  //  Referenced by: '<S125>/PreLook-Up Index Search  (altitude)'

  { 500.0, 1750.0, 3750.0, 7500.0, 15000.0, 25000.0, 35000.0, 45000.0, 55000.0,
    65000.0, 75000.0, 80000.0 },

  // Expression: [1:7]
  //  Referenced by: '<S125>/PreLook-Up Index Search  (prob of exceed)'

  { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0 },

  // Expression: sigma_vec'
  //  Referenced by: '<S125>/Medium//High Altitude Intensity'

  { 3.2, 2.2, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.2, 3.6, 3.3,
    1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.6, 6.9, 7.4, 6.7, 4.6, 2.7,
    0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 8.6, 9.6, 10.6, 10.1, 8.0, 6.6, 5.0, 4.2, 2.7,
    0.0, 0.0, 0.0, 11.8, 13.0, 16.0, 15.1, 11.6, 9.7, 8.1, 8.2, 7.9, 4.9, 3.2,
    2.1, 15.6, 17.6, 23.0, 23.6, 22.1, 20.0, 16.0, 15.1, 12.1, 7.9, 6.2, 5.1,
    18.7, 21.5, 28.4, 30.2, 30.7, 31.0, 25.2, 23.1, 17.5, 10.7, 8.4, 7.2 },

  // Expression: 0
  //  Referenced by: '<S118>/White Noise'

  0.0,

  // Computed Parameter: WhiteNoise_StdDev_k
  //  Referenced by: '<S118>/White Noise'

  1.0,

  // Expression: 1
  //  Referenced by: '<S115>/Lv'

  1.0,

  // Expression: 1
  //  Referenced by: '<S115>/Lw'

  1.0,

  // Expression: inf
  //  Referenced by: '<S105>/3ft-->inf'

  0.0,

  // Expression: 3
  //  Referenced by: '<S105>/3ft-->inf'

  3.0,

  // Expression: 1/z0
  //  Referenced by: '<S105>/h//z0'

  6.666666666666667,

  // Expression: 20/z0
  //  Referenced by: '<S105>/ref_height//z0'

  133.33333333333334,

  // Expression: 0
  //  Referenced by: '<S105>/Wdeg1'

  0.0,

  // Expression: 1
  //  Referenced by: '<S427>/Zero-Order Hold1'

  1.0,

  // Expression: 1
  //  Referenced by: '<S427>/Zero-Order Hold2'

  1.0,

  // Expression: 1
  //  Referenced by: '<S427>/Zero-Order Hold'

  1.0,

  // Expression: [0 0 0]
  //  Referenced by: '<S407>/center of gravity'

  { 0.0, 0.0, 0.0 },

  // Expression: 1
  //  Referenced by: '<S427>/Zero-Order Hold4'

  1.0,

  // Expression: [1 -1 1]
  //  Referenced by: '<S427>/Gain'

  { 1.0, -1.0, 1.0 },

  // Expression: zeros(3,3)
  //  Referenced by: '<S1>/Constant_dI//dt'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  // Expression: 1
  //  Referenced by: '<S427>/Zero-Order Hold3'

  1.0,

  // Expression: 0.5
  //  Referenced by: '<S429>/Switch'

  0.5,

  // Expression: a_sath
  //  Referenced by: '<S427>/Saturation'

  { 160.0, 160.0, 160.0 },

  // Expression: a_satl
  //  Referenced by: '<S427>/Saturation'

  { -160.0, -160.0, -160.0 },

  // Expression: -[0.1,0.1,0.2]
  //  Referenced by: '<S407>/Uniform Random Number5'

  { -0.1, -0.1, -0.2 },

  // Expression: [0.1,0.1,0.2]
  //  Referenced by: '<S407>/Uniform Random Number5'

  { 0.1, 0.1, 0.2 },

  // Expression: [12233,645554,678766]
  //  Referenced by: '<S407>/Uniform Random Number5'

  { 12233.0, 645554.0, 678766.0 },

  // Expression: 5
  //  Referenced by: '<S407>/Gain10'

  5.0,

  // Expression: 0
  //  Referenced by: '<S444>/Integrator, Second-Order Limited'

  0.0,

  // Expression: 0
  //  Referenced by: '<S444>/Integrator, Second-Order Limited'

  0.0,

  // Expression: dtype_g
  //  Referenced by: '<S442>/Constant'

  1.0,

  // Expression: 1
  //  Referenced by: '<S428>/Zero-Order Hold'

  1.0,

  // Expression: 1
  //  Referenced by: '<S428>/Zero-Order Hold1'

  1.0,

  // Expression: 0.5
  //  Referenced by: '<S442>/Switch'

  0.5,

  // Expression: g_sath
  //  Referenced by: '<S428>/Saturation'

  { 4.36, 4.36, 4.36 },

  // Expression: g_satl
  //  Referenced by: '<S428>/Saturation'

  { -4.36, -4.36, -4.36 },

  // Expression: -[0.01,0.01,0.01]
  //  Referenced by: '<S407>/Uniform Random Number1'

  { -0.01, -0.01, -0.01 },

  // Expression: [0.01,0.01,0.01]
  //  Referenced by: '<S407>/Uniform Random Number1'

  { 0.01, 0.01, 0.01 },

  // Expression: [3243,44556,2334343]
  //  Referenced by: '<S407>/Uniform Random Number1'

  { 3243.0, 44556.0, 2.334343E+6 },

  // Expression: 5
  //  Referenced by: '<S407>/Gain6'

  5.0,

  // Expression: epoch
  //  Referenced by: '<S454>/epoch'

  2015.0,

  // Expression: dyear
  //  Referenced by: '<S420>/Decimal Year'

  2017.8383561643836,

  // Expression: -1000
  //  Referenced by: '<S468>/otime'

  -1000.0,

  // Expression: 180
  //  Referenced by: '<S450>/Constant'

  180.0,

  // Expression: 360
  //  Referenced by: '<S459>/Constant2'

  360.0,

  // Expression: 0
  //  Referenced by: '<S450>/Constant1'

  0.0,

  // Expression: 360
  //  Referenced by: '<S457>/Constant2'

  360.0,

  // Expression: -1000
  //  Referenced by: '<S467>/olon'

  -1000.0,

  // Expression: -1000
  //  Referenced by: '<S466>/olat'

  -1000.0,

  // Expression: 0.001
  //  Referenced by: '<S420>/Gain'

  0.001,

  // Expression: -1000
  //  Referenced by: '<S466>/oalt'

  -1000.0,

  // Expression: 6371.2
  //  Referenced by: '<S454>/re'

  6371.2,

  // Expression: 1
  //  Referenced by: '<S407>/Gain_Mag'

  1.0,

  // Expression: 1E-5
  //  Referenced by: '<S407>/nT2Gauss'

  1.0E-5,

  // Expression: -[0.01,0.01,0.01]
  //  Referenced by: '<S407>/Uniform Random Number7'

  { -0.01, -0.01, -0.01 },

  // Expression: [0.01,0.01,0.01]
  //  Referenced by: '<S407>/Uniform Random Number7'

  { 0.01, 0.01, 0.01 },

  // Expression: [45465,454534,1234232]
  //  Referenced by: '<S407>/Uniform Random Number7'

  { 45465.0, 454534.0, 1.234232E+6 },

  // Expression: 2
  //  Referenced by: '<S407>/Gain11'

  2.0,

  // Expression: T0
  //  Referenced by: '<S417>/Sea Level  Temperature'

  288.15,

  // Expression: T0
  //  Referenced by: '<S64>/Sea Level  Temperature'

  288.15,

  // Expression: h_trop
  //  Referenced by: '<S64>/Limit  altitude  to troposhere'

  11000.0,

  // Expression: h0
  //  Referenced by: '<S64>/Limit  altitude  to troposhere'

  0.0,

  // Expression: L
  //  Referenced by: '<S64>/Lapse Rate'

  0.0065,

  // Expression: 1/T0
  //  Referenced by: '<S64>/1//T0'

  0.00347041471455839,

  // Expression: g/(L*R)
  //  Referenced by: '<S64>/Constant'

  5.2558756014667134,

  // Expression: rho0
  //  Referenced by: '<S64>/rho0'

  1.225,

  // Expression: h_trop
  //  Referenced by: '<S64>/Altitude of Troposphere'

  11000.0,

  // Expression: 0
  //  Referenced by: '<S64>/Limit  altitude  to Stratosphere'

  0.0,

  // Expression: h_trop-h_strat
  //  Referenced by: '<S64>/Limit  altitude  to Stratosphere'

  -9000.0,

  // Expression: g/R
  //  Referenced by: '<S64>/g//R'

  0.034163191409533639,

  // Expression: 1/2
  //  Referenced by: '<S409>/1//2rhoV^2'

  0.5,

  // Expression: -2
  //  Referenced by: '<S407>/Uniform Random Number'

  -2.0,

  // Expression: 2
  //  Referenced by: '<S407>/Uniform Random Number'

  2.0,

  // Expression: 15634
  //  Referenced by: '<S407>/Uniform Random Number'

  15634.0,

  // Expression: 0.2
  //  Referenced by: '<S407>/Gain5'

  0.2,

  // Expression: 0.3
  //  Referenced by: '<S415>/Constant2'

  0.3,

  // Expression: h_trop
  //  Referenced by: '<S417>/Limit  altitude  to troposhere'

  11000.0,

  // Expression: h0
  //  Referenced by: '<S417>/Limit  altitude  to troposhere'

  0.0,

  // Expression: L
  //  Referenced by: '<S417>/Lapse Rate'

  0.0065,

  // Expression: 1/T0
  //  Referenced by: '<S417>/1//T0'

  0.00347041471455839,

  // Expression: g/(L*R)
  //  Referenced by: '<S417>/Constant'

  5.2558756014667134,

  // Expression: P0
  //  Referenced by: '<S417>/P0'

  101325.0,

  // Expression: h_trop
  //  Referenced by: '<S417>/Altitude of Troposphere'

  11000.0,

  // Expression: 0
  //  Referenced by: '<S417>/Limit  altitude  to Stratosphere'

  0.0,

  // Expression: h_trop-h_strat
  //  Referenced by: '<S417>/Limit  altitude  to Stratosphere'

  -9000.0,

  // Expression: g/R
  //  Referenced by: '<S417>/g//R'

  0.034163191409533639,

  // Expression: 0.01
  //  Referenced by: '<S407>/Gain'

  0.01,

  // Expression: -1
  //  Referenced by: '<S407>/Uniform Random Number4'

  -1.0,

  // Expression: 1
  //  Referenced by: '<S407>/Uniform Random Number4'

  1.0,

  // Expression: 25634
  //  Referenced by: '<S407>/Uniform Random Number4'

  25634.0,

  // Expression: 0.01
  //  Referenced by: '<S407>/Gain9'

  0.01,

  // Expression: 0.5
  //  Referenced by: '<S416>/Constant'

  0.5,

  // Expression: 0.7
  //  Referenced by: '<S416>/Gain2'

  0.7,

  // Expression: 0.3
  //  Referenced by: '<S416>/Constant2'

  0.3,

  // Expression: 0.01
  //  Referenced by: '<S407>/Gain1'

  0.01,

  // Expression: 10
  //  Referenced by: '<S5>/Constant1'

  10.0,

  // Expression: 8191
  //  Referenced by: '<S403>/Constant'

  8191.0,

  // Expression: [1 0 0 0]
  //  Referenced by: '<S329>/Merge'

  { 1.0, 0.0, 0.0, 0.0 },

  // Expression: 1E7
  //  Referenced by: '<S332>/Gain'

  1.0E+7,

  // Expression: 1E7
  //  Referenced by: '<S332>/Gain1'

  1.0E+7,

  // Expression: 1E3
  //  Referenced by: '<S332>/Gain3'

  1000.0,

  // Expression: 100
  //  Referenced by: '<S332>/Gain4'

  100.0,

  // Expression: 1/2
  //  Referenced by: '<S519>/1//2rhoV^2'

  0.5,

  // Expression: 2/1.225
  //  Referenced by: '<S332>/Gain2'

  1.6326530612244896,

  // Expression: 100
  //  Referenced by: '<S332>/Gain7'

  100.0,

  // Expression: [-1,-1,-2]
  //  Referenced by: '<S374>/Uniform Random Number5'

  { -1.0, -1.0, -2.0 },

  // Expression: [1,1,2]
  //  Referenced by: '<S374>/Uniform Random Number5'

  { 1.0, 1.0, 2.0 },

  // Expression: [1452,787,69]
  //  Referenced by: '<S374>/Uniform Random Number5'

  { 1452.0, 787.0, 69.0 },

  // Expression: 0.2
  //  Referenced by: '<S374>/BiasGain2'

  0.2,

  // Expression: 1
  //  Referenced by: '<S397>/Constant2'

  1.0,

  // Expression: R
  //  Referenced by: '<S397>/Constant1'

  6.378137E+6,

  // Expression: 1
  //  Referenced by: '<S400>/Constant'

  1.0,

  // Expression: 1
  //  Referenced by: '<S402>/Constant'

  1.0,

  // Expression: F
  //  Referenced by: '<S401>/Constant'

  0.0033528106647474805,

  // Expression: 1
  //  Referenced by: '<S401>/f'

  1.0,

  // Expression: 1
  //  Referenced by: '<S397>/Constant'

  1.0,

  // Expression: 1
  //  Referenced by: '<S397>/Constant3'

  1.0,

  // Expression: 360
  //  Referenced by: '<S388>/Constant2'

  360.0,

  // Expression: 1E7
  //  Referenced by: '<S330>/latScale'

  1.0E+7,

  // Expression: 180
  //  Referenced by: '<S381>/Constant'

  180.0,

  // Expression: 0
  //  Referenced by: '<S381>/Constant1'

  0.0,

  // Expression: 360
  //  Referenced by: '<S386>/Constant2'

  360.0,

  // Expression: 1E7
  //  Referenced by: '<S330>/lonScale'

  1.0E+7,

  // Expression: 100000
  //  Referenced by: '<S374>/Saturation'

  100000.0,

  // Expression: 0
  //  Referenced by: '<S374>/Saturation'

  0.0,

  // Expression: 1E3
  //  Referenced by: '<S330>/altScale'

  1000.0,

  // Expression: 100
  //  Referenced by: '<S330>/Gain6'

  100.0,

  // Expression: 100
  //  Referenced by: '<S330>/Gain8'

  100.0,

  // Computed Parameter: TransferFcn4_A
  //  Referenced by: '<S376>/Transfer Fcn4'

  -20.0,

  // Computed Parameter: TransferFcn4_C
  //  Referenced by: '<S376>/Transfer Fcn4'

  20.0,

  // Computed Parameter: TransferFcn1_A
  //  Referenced by: '<S376>/Transfer Fcn1'

  -20.0,

  // Computed Parameter: TransferFcn1_C
  //  Referenced by: '<S376>/Transfer Fcn1'

  20.0,

  // Computed Parameter: TransferFcn2_A
  //  Referenced by: '<S376>/Transfer Fcn2'

  -20.0,

  // Computed Parameter: TransferFcn2_C
  //  Referenced by: '<S376>/Transfer Fcn2'

  20.0,

  // Expression: 1E2
  //  Referenced by: '<S330>/VelScale'

  100.0,

  // Expression: 1E2
  //  Referenced by: '<S330>/VeScale'

  100.0,

  // Expression: 1E2
  //  Referenced by: '<S330>/AngleScale'

  100.0,

  // Expression: 1E3
  //  Referenced by: '<S330>/Gain1'

  1000.0,

  // Expression: 1E3
  //  Referenced by: '<S330>/Gain3'

  1000.0,

  // Expression: 1E3
  //  Referenced by: '<S330>/VeScale1'

  1000.0,

  // Expression: 1E3
  //  Referenced by: '<S330>/VelScale1'

  1000.0,

  // Expression: 1E5
  //  Referenced by: '<S330>/headMotAngle'

  100000.0,

  // Expression: 1E5
  //  Referenced by: '<S330>/headVehAngleScale'

  100000.0,

  // Expression: 100
  //  Referenced by: '<S407>/magDecGain'

  100.0,

  // Expression: 60/2/pi
  //  Referenced by: '<S12>/Gain'

  9.5492965855137211,

  // Expression: 1
  //  Referenced by: '<S12>/CopterID'

  1.0,

  // Expression: 1
  //  Referenced by: '<S26>/Constant'

  1.0,

  // Expression: [-1,-1,-2]
  //  Referenced by: '<S330>/Uniform Random Number4'

  { -1.0, -1.0, -2.0 },

  // Expression: [1,1,2]
  //  Referenced by: '<S330>/Uniform Random Number4'

  { 1.0, 1.0, 2.0 },

  // Expression: [5445,45433,33433]
  //  Referenced by: '<S330>/Uniform Random Number4'

  { 5445.0, 45433.0, 33433.0 },

  // Expression: 0.1
  //  Referenced by: '<S330>/BiasGain1'

  0.1,

  // Computed Parameter: Constant1_Value_l
  //  Referenced by: '<S480>/Constant1'

  1,

  // Computed Parameter: Constant_Value_op
  //  Referenced by: '<S481>/Constant'

  1,

  // Computed Parameter: Constant_Value_mh
  //  Referenced by: '<S479>/Constant'

  1,

  // Computed Parameter: Constant_Value_os
  //  Referenced by: '<S490>/Constant'

  1,

  // Computed Parameter: Gain_Gain_bd
  //  Referenced by: '<S490>/Gain'

  13,

  // Computed Parameter: Constant_Value_es
  //  Referenced by: '<S493>/Constant'

  1,

  // Computed Parameter: Gain_Gain_hz
  //  Referenced by: '<S492>/Gain'

  13,

  // Computed Parameter: Constant_Value_py
  //  Referenced by: '<S496>/Constant'

  1,

  // Computed Parameter: Constant1_Value_c
  //  Referenced by: '<S496>/Constant1'

  1,

  // Computed Parameter: Constant1_Value_fy
  //  Referenced by: '<S497>/Constant1'

  2,

  // Computed Parameter: Constant_Value_g2
  //  Referenced by: '<S495>/Constant'

  1,

  // Computed Parameter: Constant1_Value_m
  //  Referenced by: '<S494>/Constant1'

  1,

  // Computed Parameter: Gain_Gain_aa
  //  Referenced by: '<S494>/Gain'

  13,

  // Computed Parameter: Constant1_Value_hs
  //  Referenced by: '<S498>/Constant1'

  2,

  // Computed Parameter: Constant_Value_f
  //  Referenced by: '<S472>/Constant'

  1,

  // Computed Parameter: Constant_Value_lp
  //  Referenced by: '<S489>/Constant'

  1,

  // Computed Parameter: Gain_Gain_ob
  //  Referenced by: '<S489>/Gain'

  13,

  // Computed Parameter: Constant_Value_g0
  //  Referenced by: '<S499>/Constant'

  1,

  // Computed Parameter: Constant1_Value_g
  //  Referenced by: '<S499>/Constant1'

  1,

  // Computed Parameter: Constant_Value_nf
  //  Referenced by: '<S501>/Constant'

  1,

  // Computed Parameter: tc_old_Threshold
  //  Referenced by: '<S500>/tc_old'

  1,

  // Computed Parameter: Constant_Value_j
  //  Referenced by: '<S471>/Constant'

  1,

  // Computed Parameter: Constant1_Value_ja
  //  Referenced by: '<S471>/Constant1'

  1,

  // Computed Parameter: Constant_Value_cy
  //  Referenced by: '<S470>/Constant'

  1,

  // Computed Parameter: Constant_Value_pe
  //  Referenced by: '<S475>/Constant'

  1,

  // Computed Parameter: Gain_Gain_ij
  //  Referenced by: '<S475>/Gain'

  13,

  // Computed Parameter: Constant_Value_a
  //  Referenced by: '<S477>/Constant'

  1,

  // Computed Parameter: ForIterator_IterationLimit_h
  //  Referenced by: '<S462>/For Iterator'

  12,

  // Computed Parameter: Constant_Value_lq
  //  Referenced by: '<S462>/Constant'

  1,

  // Computed Parameter: arn_Threshold
  //  Referenced by: '<S462>/ar(n)'

  1,

  // Expression: FaultParamStruct.LoadFallFaultID
  //  Referenced by: '<S187>/FaultID'

  123456,

  // Expression: FaultParamStruct.LoadShiftFaultID
  //  Referenced by: '<S187>/FaultID1'

  123457,

  // Expression: FaultParamStruct.LoadLeakFaultID
  //  Referenced by: '<S187>/FaultID2'

  123458,

  // Expression: FaultParamStruct.FaultID
  //  Referenced by: '<S522>/FaultID'

  123451,

  // Expression: FaultParamStruct.FaultID
  //  Referenced by: '<S295>/FaultID'

  123450,

  // Expression: FaultParamStruct.UseCusTomHoverTimeFaultID
  //  Referenced by: '<S57>/FaultID'

  123452,

  // Expression: FaultParamStruct.PowOffFaultID
  //  Referenced by: '<S57>/FaultID1'

  123453,

  // Expression: FaultParamStruct.LowVoltageFaultID
  //  Referenced by: '<S57>/FaultID2'

  123454,

  // Expression: FaultParamStruct.LowCapacityFaultID
  //  Referenced by: '<S57>/FaultID3'

  123455,

  // Expression: FaultParamStruct.NoiseWindFaultID
  //  Referenced by: '<S67>/FaultID4'

  123543,

  // Expression: FaultParamStruct.ConstWindFaultID
  //  Referenced by: '<S67>/FaultID'

  123459,

  // Expression: FaultParamStruct.GustWindFaultID
  //  Referenced by: '<S67>/FaultID1'

  123540,

  // Expression: FaultParamStruct.TurbWindFaultID
  //  Referenced by: '<S67>/FaultID2'

  123541,

  // Expression: FaultParamStruct.SheerWindFaultID
  //  Referenced by: '<S67>/FaultID3'

  123542,

  // Computed Parameter: FaultID_Value_f
  //  Referenced by: '<S407>/FaultID'

  123544,

  // Computed Parameter: FaultID1_Value_i
  //  Referenced by: '<S407>/FaultID1'

  123545,

  // Computed Parameter: FaultID2_Value_d
  //  Referenced by: '<S407>/FaultID2'

  123546,

  // Computed Parameter: FaultID3_Value_l
  //  Referenced by: '<S407>/FaultID3'

  123547,

  // Expression: FaultParamStruct.GPSNoiseFaultID
  //  Referenced by: '<S374>/FaultID'

  123548,

  // Computed Parameter: Gain5_Gain_p
  //  Referenced by: '<S332>/Gain5'

  101.971619F,

  // Computed Parameter: MediumHighAltitudeIntensity_max
  //  Referenced by: '<S164>/Medium//High Altitude Intensity'

  { 11U, 6U },

  // Computed Parameter: MediumHighAltitudeIntensity_m_i
  //  Referenced by: '<S125>/Medium//High Altitude Intensity'

  { 11U, 6U },

  // Start of '<S524>/Distance into gust (z)'
  {
    // Expression: [0]
    //  Referenced by: '<S529>/x'

    0.0,

    // Expression: 0
    //  Referenced by: '<S529>/Distance into Gust (x) (Limited to gust length d) '

    0.0,

    // Expression: 0
    //  Referenced by: '<S529>/Distance into Gust (x) (Limited to gust length d) '

    0.0
  }
  ,

  // End of '<S524>/Distance into gust (z)'

  // Start of '<S524>/Distance into gust (y)'
  {
    // Expression: [0]
    //  Referenced by: '<S528>/x'

    0.0,

    // Expression: 0
    //  Referenced by: '<S528>/Distance into Gust (x) (Limited to gust length d) '

    0.0,

    // Expression: 0
    //  Referenced by: '<S528>/Distance into Gust (x) (Limited to gust length d) '

    0.0
  }
  ,

  // End of '<S524>/Distance into gust (y)'

  // Start of '<S524>/Distance into gust (x)'
  {
    // Expression: [0]
    //  Referenced by: '<S527>/x'

    0.0,

    // Expression: 0
    //  Referenced by: '<S527>/Distance into Gust (x) (Limited to gust length d)'

    0.0,

    // Expression: 0
    //  Referenced by: '<S527>/Distance into Gust (x) (Limited to gust length d)'

    0.0
  }
  ,

  // End of '<S524>/Distance into gust (x)'

  // Start of '<S337>/If Warning//Error'
  {
    // Expression: 0
    //  Referenced by: '<S363>/Constant1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S362>/Constant1'

    0.0,

    // Expression: -1
    //  Referenced by: '<S365>/Bias'

    -1.0,

    // Expression: -eye(3)
    //  Referenced by: '<S364>/Bias1'

    { -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0 }
  }
  ,

  // End of '<S337>/If Warning//Error'

  // Start of '<S329>/Negative Trace'
  {
    // Expression: 1
    //  Referenced by: '<S352>/Constant'

    1.0,

    // Expression: 0.5
    //  Referenced by: '<S340>/Gain'

    0.5,

    // Expression: 0.5
    //  Referenced by: '<S351>/Constant1'

    0.5,

    // Expression: [0 1]
    //  Referenced by: '<S351>/Constant2'

    { 0.0, 1.0 },

    // Expression: 1
    //  Referenced by: '<S340>/Gain1'

    1.0,

    // Expression: 1
    //  Referenced by: '<S340>/Gain3'

    1.0,

    // Expression: 1
    //  Referenced by: '<S340>/Gain4'

    1.0,

    // Expression: 1
    //  Referenced by: '<S357>/Constant'

    1.0,

    // Expression: 0.5
    //  Referenced by: '<S341>/Gain'

    0.5,

    // Expression: 0.5
    //  Referenced by: '<S356>/Constant1'

    0.5,

    // Expression: [0 1]
    //  Referenced by: '<S356>/Constant2'

    { 0.0, 1.0 },

    // Expression: 1
    //  Referenced by: '<S341>/Gain1'

    1.0,

    // Expression: 1
    //  Referenced by: '<S341>/Gain2'

    1.0,

    // Expression: 1
    //  Referenced by: '<S341>/Gain3'

    1.0,

    // Expression: 1
    //  Referenced by: '<S347>/Constant'

    1.0,

    // Expression: 0.5
    //  Referenced by: '<S339>/Gain'

    0.5,

    // Expression: 0.5
    //  Referenced by: '<S346>/Constant1'

    0.5,

    // Expression: [0 1]
    //  Referenced by: '<S346>/Constant2'

    { 0.0, 1.0 },

    // Expression: 1
    //  Referenced by: '<S339>/Gain1'

    1.0,

    // Expression: 1
    //  Referenced by: '<S339>/Gain2'

    1.0,

    // Expression: 1
    //  Referenced by: '<S339>/Gain3'

    1.0
  }
  ,

  // End of '<S329>/Negative Trace'

  // Start of '<S329>/Positive Trace'
  {
    // Expression: 1
    //  Referenced by: '<S336>/Constant'

    1.0,

    // Expression: 0.5
    //  Referenced by: '<S336>/Gain'

    0.5,

    // Expression: 2
    //  Referenced by: '<S336>/Gain1'

    2.0
  }
  ,

  // End of '<S329>/Positive Trace'

  // Start of '<S296>/MotorNonlinearDynamic8'
  {
    // Expression: 0.95
    //  Referenced by: '<S312>/Signal_Saturation'

    0.95,

    // Expression: 0
    //  Referenced by: '<S312>/Signal_Saturation'

    0.0
  }
  ,

  // End of '<S296>/MotorNonlinearDynamic8'

  // Start of '<S296>/MotorNonlinearDynamic7'
  {
    // Expression: 0.95
    //  Referenced by: '<S311>/Signal_Saturation'

    0.95,

    // Expression: 0
    //  Referenced by: '<S311>/Signal_Saturation'

    0.0
  }
  ,

  // End of '<S296>/MotorNonlinearDynamic7'

  // Start of '<S296>/MotorNonlinearDynamic6'
  {
    // Expression: 0.95
    //  Referenced by: '<S310>/Signal_Saturation'

    0.95,

    // Expression: 0
    //  Referenced by: '<S310>/Signal_Saturation'

    0.0
  }
  ,

  // End of '<S296>/MotorNonlinearDynamic6'

  // Start of '<S296>/MotorNonlinearDynamic5'
  {
    // Expression: 0.95
    //  Referenced by: '<S309>/Signal_Saturation'

    0.95,

    // Expression: 0
    //  Referenced by: '<S309>/Signal_Saturation'

    0.0
  }
  ,

  // End of '<S296>/MotorNonlinearDynamic5'

  // Start of '<S296>/MotorNonlinearDynamic4'
  {
    // Expression: 0.95
    //  Referenced by: '<S308>/Signal_Saturation'

    0.95,

    // Expression: 0
    //  Referenced by: '<S308>/Signal_Saturation'

    0.0
  }
  ,

  // End of '<S296>/MotorNonlinearDynamic4'

  // Start of '<S296>/MotorNonlinearDynamic3'
  {
    // Expression: 0.95
    //  Referenced by: '<S307>/Signal_Saturation'

    0.95,

    // Expression: 0
    //  Referenced by: '<S307>/Signal_Saturation'

    0.0
  }
  ,

  // End of '<S296>/MotorNonlinearDynamic3'

  // Start of '<S296>/MotorNonlinearDynamic2'
  {
    // Expression: 0.95
    //  Referenced by: '<S306>/Signal_Saturation'

    0.95,

    // Expression: 0
    //  Referenced by: '<S306>/Signal_Saturation'

    0.0
  }
  ,

  // End of '<S296>/MotorNonlinearDynamic2'

  // Start of '<S296>/MotorNonlinearDynamic1'
  {
    // Expression: 0.95
    //  Referenced by: '<S305>/Signal_Saturation'

    0.95,

    // Expression: 0
    //  Referenced by: '<S305>/Signal_Saturation'

    0.0
  }
  ,

  // End of '<S296>/MotorNonlinearDynamic1'

  // Start of '<S298>/Distance into gust (z)'
  {
    // Expression: [0]
    //  Referenced by: '<S303>/x'

    0.0,

    // Expression: 0
    //  Referenced by: '<S303>/Distance into Gust (x) (Limited to gust length d) '

    0.0,

    // Expression: 0
    //  Referenced by: '<S303>/Distance into Gust (x) (Limited to gust length d) '

    0.0
  }
  ,

  // End of '<S298>/Distance into gust (z)'

  // Start of '<S298>/Distance into gust (y)'
  {
    // Expression: [0]
    //  Referenced by: '<S302>/x'

    0.0,

    // Expression: 0
    //  Referenced by: '<S302>/Distance into Gust (x) (Limited to gust length d) '

    0.0,

    // Expression: 0
    //  Referenced by: '<S302>/Distance into Gust (x) (Limited to gust length d) '

    0.0
  }
  ,

  // End of '<S298>/Distance into gust (y)'

  // Start of '<S298>/Distance into gust (x)'
  {
    // Expression: [0]
    //  Referenced by: '<S301>/x'

    0.0,

    // Expression: 0
    //  Referenced by: '<S301>/Distance into Gust (x) (Limited to gust length d)'

    0.0,

    // Expression: 0
    //  Referenced by: '<S301>/Distance into Gust (x) (Limited to gust length d)'

    0.0
  }
  ,

  // End of '<S298>/Distance into gust (x)'

  // Start of '<S241>/If Warning//Error'
  {
    // Expression: 0
    //  Referenced by: '<S267>/Constant1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S266>/Constant1'

    0.0,

    // Expression: -1
    //  Referenced by: '<S269>/Bias'

    -1.0,

    // Expression: -eye(3)
    //  Referenced by: '<S268>/Bias1'

    { -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0 }
  }
  ,

  // End of '<S241>/If Warning//Error'

  // Start of '<S194>/Negative Trace'
  {
    // Expression: 1
    //  Referenced by: '<S256>/Constant'

    1.0,

    // Expression: 0.5
    //  Referenced by: '<S244>/Gain'

    0.5,

    // Expression: 0.5
    //  Referenced by: '<S255>/Constant1'

    0.5,

    // Expression: [0 1]
    //  Referenced by: '<S255>/Constant2'

    { 0.0, 1.0 },

    // Expression: 1
    //  Referenced by: '<S244>/Gain1'

    1.0,

    // Expression: 1
    //  Referenced by: '<S244>/Gain3'

    1.0,

    // Expression: 1
    //  Referenced by: '<S244>/Gain4'

    1.0,

    // Expression: 1
    //  Referenced by: '<S261>/Constant'

    1.0,

    // Expression: 0.5
    //  Referenced by: '<S245>/Gain'

    0.5,

    // Expression: 0.5
    //  Referenced by: '<S260>/Constant1'

    0.5,

    // Expression: [0 1]
    //  Referenced by: '<S260>/Constant2'

    { 0.0, 1.0 },

    // Expression: 1
    //  Referenced by: '<S245>/Gain1'

    1.0,

    // Expression: 1
    //  Referenced by: '<S245>/Gain2'

    1.0,

    // Expression: 1
    //  Referenced by: '<S245>/Gain3'

    1.0,

    // Expression: 1
    //  Referenced by: '<S251>/Constant'

    1.0,

    // Expression: 0.5
    //  Referenced by: '<S243>/Gain'

    0.5,

    // Expression: 0.5
    //  Referenced by: '<S250>/Constant1'

    0.5,

    // Expression: [0 1]
    //  Referenced by: '<S250>/Constant2'

    { 0.0, 1.0 },

    // Expression: 1
    //  Referenced by: '<S243>/Gain1'

    1.0,

    // Expression: 1
    //  Referenced by: '<S243>/Gain2'

    1.0,

    // Expression: 1
    //  Referenced by: '<S243>/Gain3'

    1.0
  }
  ,

  // End of '<S194>/Negative Trace'

  // Start of '<S194>/Positive Trace'
  {
    // Expression: 1
    //  Referenced by: '<S240>/Constant'

    1.0,

    // Expression: 0.5
    //  Referenced by: '<S240>/Gain'

    0.5,

    // Expression: 2
    //  Referenced by: '<S240>/Gain1'

    2.0
  }
  ,

  // End of '<S194>/Positive Trace'

  // Start of '<S203>/If Warning//Error'
  {
    // Expression: 0
    //  Referenced by: '<S229>/Constant1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S228>/Constant1'

    0.0,

    // Expression: -1
    //  Referenced by: '<S231>/Bias'

    -1.0,

    // Expression: -eye(3)
    //  Referenced by: '<S230>/Bias1'

    { -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0 }
  }
  ,

  // End of '<S203>/If Warning//Error'

  // Start of '<S193>/Negative Trace'
  {
    // Expression: 1
    //  Referenced by: '<S218>/Constant'

    1.0,

    // Expression: 0.5
    //  Referenced by: '<S206>/Gain'

    0.5,

    // Expression: 0.5
    //  Referenced by: '<S217>/Constant1'

    0.5,

    // Expression: [0 1]
    //  Referenced by: '<S217>/Constant2'

    { 0.0, 1.0 },

    // Expression: 1
    //  Referenced by: '<S206>/Gain1'

    1.0,

    // Expression: 1
    //  Referenced by: '<S206>/Gain3'

    1.0,

    // Expression: 1
    //  Referenced by: '<S206>/Gain4'

    1.0,

    // Expression: 1
    //  Referenced by: '<S223>/Constant'

    1.0,

    // Expression: 0.5
    //  Referenced by: '<S207>/Gain'

    0.5,

    // Expression: 0.5
    //  Referenced by: '<S222>/Constant1'

    0.5,

    // Expression: [0 1]
    //  Referenced by: '<S222>/Constant2'

    { 0.0, 1.0 },

    // Expression: 1
    //  Referenced by: '<S207>/Gain1'

    1.0,

    // Expression: 1
    //  Referenced by: '<S207>/Gain2'

    1.0,

    // Expression: 1
    //  Referenced by: '<S207>/Gain3'

    1.0,

    // Expression: 1
    //  Referenced by: '<S213>/Constant'

    1.0,

    // Expression: 0.5
    //  Referenced by: '<S205>/Gain'

    0.5,

    // Expression: 0.5
    //  Referenced by: '<S212>/Constant1'

    0.5,

    // Expression: [0 1]
    //  Referenced by: '<S212>/Constant2'

    { 0.0, 1.0 },

    // Expression: 1
    //  Referenced by: '<S205>/Gain1'

    1.0,

    // Expression: 1
    //  Referenced by: '<S205>/Gain2'

    1.0,

    // Expression: 1
    //  Referenced by: '<S205>/Gain3'

    1.0
  }
  ,

  // End of '<S193>/Negative Trace'

  // Start of '<S193>/Positive Trace'
  {
    // Expression: 1
    //  Referenced by: '<S202>/Constant'

    1.0,

    // Expression: 0.5
    //  Referenced by: '<S202>/Gain'

    0.5,

    // Expression: 2
    //  Referenced by: '<S202>/Gain1'

    2.0
  }
  ,

  // End of '<S193>/Positive Trace'

  // Start of '<S153>/Interpolate  velocities'
  {
    // Expression: max_height_low
    //  Referenced by: '<S174>/max_height_low'

    1000.0,

    // Expression: min_height_high
    //  Referenced by: '<S174>/min_height_high'

    2000.0
  }
  ,

  // End of '<S153>/Interpolate  velocities'

  // Start of '<S152>/Interpolate  rates'
  {
    // Expression: max_height_low
    //  Referenced by: '<S166>/max_height_low'

    1000.0,

    // Expression: min_height_high
    //  Referenced by: '<S166>/min_height_high'

    2000.0
  }
  ,

  // End of '<S152>/Interpolate  rates'

  // Start of '<S148>/Hwgw(z)'
  {
    // Expression: 0
    //  Referenced by: '<S163>/wgw'

    0.0,

    // Expression: 2*dt
    //  Referenced by: '<S163>/2'

    0.02,

    // Expression: 1
    //  Referenced by: '<S163>/Constant'

    1.0,

    // Expression: dt
    //  Referenced by: '<S163>/dt'

    0.01,

    // Expression: 0
    //  Referenced by: '<S163>/Unit Delay'

    0.0
  }
  ,

  // End of '<S148>/Hwgw(z)'

  // Start of '<S148>/Hvgw(z)'
  {
    // Expression: 0
    //  Referenced by: '<S162>/vgw'

    0.0,

    // Expression: 2*dt
    //  Referenced by: '<S162>/2'

    0.02,

    // Expression: 1
    //  Referenced by: '<S162>/Constant'

    1.0,

    // Expression: dt
    //  Referenced by: '<S162>/dt'

    0.01,

    // Expression: 0
    //  Referenced by: '<S162>/Unit Delay'

    0.0
  }
  ,

  // End of '<S148>/Hvgw(z)'

  // Start of '<S148>/Hugw(z)'
  {
    // Expression: 0
    //  Referenced by: '<S161>/ugw'

    0.0,

    // Expression: 2*dt
    //  Referenced by: '<S161>/2'

    0.02,

    // Expression: 1
    //  Referenced by: '<S161>/Constant'

    1.0,

    // Expression: dt
    //  Referenced by: '<S161>/dt'

    0.01,

    // Expression: 0
    //  Referenced by: '<S161>/Unit Delay'

    0.0
  }
  ,

  // End of '<S148>/Hugw(z)'

  // Start of '<S147>/Hrgw'
  {
    // Expression: 0
    //  Referenced by: '<S160>/rgw'

    0.0,

    // Expression: 1
    //  Referenced by: '<S160>/Constant'

    1.0,

    // Expression: 3/pi
    //  Referenced by: '<S160>/dt1'

    0.954929658551372,

    // Expression: dt
    //  Referenced by: '<S160>/dt'

    0.01,

    // Expression: 0
    //  Referenced by: '<S160>/Unit Delay'

    0.0,

    // Expression: 0
    //  Referenced by: '<S160>/Unit Delay1'

    0.0
  }
  ,

  // End of '<S147>/Hrgw'

  // Start of '<S147>/Hqgw'
  {
    // Expression: 0
    //  Referenced by: '<S159>/qgw'

    0.0,

    // Expression: 1
    //  Referenced by: '<S159>/Constant'

    1.0,

    // Expression: 4/pi
    //  Referenced by: '<S159>/dt1'

    1.2732395447351628,

    // Expression: dt
    //  Referenced by: '<S159>/dt'

    0.01,

    // Expression: 0
    //  Referenced by: '<S159>/Unit Delay'

    0.0,

    // Expression: 0
    //  Referenced by: '<S159>/Unit Delay1'

    0.0
  }
  ,

  // End of '<S147>/Hqgw'

  // Start of '<S147>/Hpgw'
  {
    // Expression: 0
    //  Referenced by: '<S158>/pgw'

    0.0,

    // Expression: 2.6
    //  Referenced by: '<S158>/Constant2'

    2.6,

    // Expression: 2*dt
    //  Referenced by: '<S158>/2'

    0.02,

    // Expression: 1
    //  Referenced by: '<S158>/Constant'

    1.0,

    // Expression: 0.95
    //  Referenced by: '<S158>/Constant1'

    0.95,

    // Expression: 1/3
    //  Referenced by: '<S158>/Constant3'

    0.33333333333333331,

    // Expression: dt
    //  Referenced by: '<S158>/dt'

    0.01,

    // Expression: 0
    //  Referenced by: '<S158>/Unit Delay'

    0.0
  }
  ,

  // End of '<S147>/Hpgw'

  // Start of '<S114>/Interpolate  velocities'
  {
    // Expression: max_height_low
    //  Referenced by: '<S135>/max_height_low'

    1000.0,

    // Expression: min_height_high
    //  Referenced by: '<S135>/min_height_high'

    2000.0
  }
  ,

  // End of '<S114>/Interpolate  velocities'

  // Start of '<S113>/Interpolate  rates'
  {
    // Expression: max_height_low
    //  Referenced by: '<S127>/max_height_low'

    1000.0,

    // Expression: min_height_high
    //  Referenced by: '<S127>/min_height_high'

    2000.0
  }
  ,

  // End of '<S113>/Interpolate  rates'

  // Start of '<S109>/Hwgw(z)'
  {
    // Expression: 0
    //  Referenced by: '<S124>/wgw'

    0.0,

    // Expression: 2*dt
    //  Referenced by: '<S124>/2'

    0.02,

    // Expression: 1
    //  Referenced by: '<S124>/Constant'

    1.0,

    // Expression: dt
    //  Referenced by: '<S124>/dt'

    0.01,

    // Expression: 0
    //  Referenced by: '<S124>/Unit Delay'

    0.0
  }
  ,

  // End of '<S109>/Hwgw(z)'

  // Start of '<S109>/Hvgw(z)'
  {
    // Expression: 0
    //  Referenced by: '<S123>/vgw'

    0.0,

    // Expression: 2*dt
    //  Referenced by: '<S123>/2'

    0.02,

    // Expression: 1
    //  Referenced by: '<S123>/Constant'

    1.0,

    // Expression: dt
    //  Referenced by: '<S123>/dt'

    0.01,

    // Expression: 0
    //  Referenced by: '<S123>/Unit Delay'

    0.0
  }
  ,

  // End of '<S109>/Hvgw(z)'

  // Start of '<S109>/Hugw(z)'
  {
    // Expression: 0
    //  Referenced by: '<S122>/ugw'

    0.0,

    // Expression: 2*dt
    //  Referenced by: '<S122>/2'

    0.02,

    // Expression: 1
    //  Referenced by: '<S122>/Constant'

    1.0,

    // Expression: dt
    //  Referenced by: '<S122>/dt'

    0.01,

    // Expression: 0
    //  Referenced by: '<S122>/Unit Delay'

    0.0
  }
  ,

  // End of '<S109>/Hugw(z)'

  // Start of '<S108>/Hrgw'
  {
    // Expression: 0
    //  Referenced by: '<S121>/rgw'

    0.0,

    // Expression: 1
    //  Referenced by: '<S121>/Constant'

    1.0,

    // Expression: 3/pi
    //  Referenced by: '<S121>/dt1'

    0.954929658551372,

    // Expression: dt
    //  Referenced by: '<S121>/dt'

    0.01,

    // Expression: 0
    //  Referenced by: '<S121>/Unit Delay'

    0.0,

    // Expression: 0
    //  Referenced by: '<S121>/Unit Delay1'

    0.0
  }
  ,

  // End of '<S108>/Hrgw'

  // Start of '<S108>/Hqgw'
  {
    // Expression: 0
    //  Referenced by: '<S120>/qgw'

    0.0,

    // Expression: 1
    //  Referenced by: '<S120>/Constant'

    1.0,

    // Expression: 4/pi
    //  Referenced by: '<S120>/dt1'

    1.2732395447351628,

    // Expression: dt
    //  Referenced by: '<S120>/dt'

    0.01,

    // Expression: 0
    //  Referenced by: '<S120>/Unit Delay'

    0.0,

    // Expression: 0
    //  Referenced by: '<S120>/Unit Delay1'

    0.0
  }
  ,

  // End of '<S108>/Hqgw'

  // Start of '<S108>/Hpgw'
  {
    // Expression: 0
    //  Referenced by: '<S119>/pgw'

    0.0,

    // Expression: 2.6
    //  Referenced by: '<S119>/Constant2'

    2.6,

    // Expression: 2*dt
    //  Referenced by: '<S119>/2'

    0.02,

    // Expression: 1
    //  Referenced by: '<S119>/Constant'

    1.0,

    // Expression: 0.95
    //  Referenced by: '<S119>/Constant1'

    0.95,

    // Expression: 1/3
    //  Referenced by: '<S119>/Constant3'

    0.33333333333333331,

    // Expression: dt
    //  Referenced by: '<S119>/dt'

    0.01,

    // Expression: 0
    //  Referenced by: '<S119>/Unit Delay'

    0.0
  }
  // End of '<S108>/Hpgw'
};

extern real_T rt_powd_snf(real_T u0, real_T u1);
extern real_T rt_modd_snf(real_T u0, real_T u1);
extern real_T rt_atan2d_snf(real_T u0, real_T u1);
extern void rt_mrdivide_U1d1x3_U2d_9vOrDY9Z(const real_T u0[3], const real_T u1
  [9], real_T y[3]);
extern real_T rt_roundd_snf(real_T u);
extern real32_T rt_roundf_snf(real32_T u);
extern real_T rt_urand_Upu32_Yd_f_pw_snf(uint32_T *u);
extern real_T rt_nrand_Upu32_Yd_f_pw_snf(uint32_T *u);
void wgs84_taylor_series(real_T *h, real_T *phi, real_T opt_m2ft, real_T *y,
  int_T k);
static uint32_T plook_bincpa(real_T u, const real_T bp[], uint32_T maxIndex,
  real_T *fraction, uint32_T *prevIndex);
static real_T intrp2d_la_pw(const uint32_T bpIndex[], const real_T frac[], const
  real_T table[], const uint32_T stride, const uint32_T maxIndex[]);
static uint32_T binsearch_u32d_prevIdx(real_T u, const real_T bp[], uint32_T
  startIndex, uint32_T maxIndex);
static void mul_wide_s32(int32_T in0, int32_T in1, uint32_T *ptrOutBitsHi,
  uint32_T *ptrOutBitsLo);
static int32_T mul_s32_sat(int32_T a, int32_T b);

// private model entry point functions
extern void QuadModelv_derivatives();
static void rate_scheduler(RT_MODEL_QuadModelv_T *const QuadModelv_M);

//===========*
//  Constants *
// ===========
#define RT_PI                          3.14159265358979323846
#define RT_PIF                         3.1415927F
#define RT_LN_10                       2.30258509299404568402
#define RT_LN_10F                      2.3025851F
#define RT_LOG10E                      0.43429448190325182765
#define RT_LOG10EF                     0.43429449F
#define RT_E                           2.7182818284590452354
#define RT_EF                          2.7182817F

//
//  UNUSED_PARAMETER(x)
//    Used to specify that a function parameter (argument) is required but not
//    accessed by the function body.

#ifndef UNUSED_PARAMETER
#if defined(__LCC__)
#define UNUSED_PARAMETER(x)                                      // do nothing
#else

//
//  This is the semi-ANSI standard way of indicating that an
//  unused function parameter is required.

#define UNUSED_PARAMETER(x)            (void) (x)
#endif
#endif

extern "C" {
  real_T rtInf;
  real_T rtMinusInf;
  real_T rtNaN;
  real32_T rtInfF;
  real32_T rtMinusInfF;
  real32_T rtNaNF;
}
//=========*
//  Asserts *
// =========
#ifndef utAssert
#if defined(DOASSERTS)
#if !defined(PRINT_ASSERTS)
#include <assert.h>
#define utAssert(exp)                  assert(exp)
#else
#include <stdio.h>

static void _assert(char *statement, char *file, int line)
{
  printf("%s in %s on line %d\n", statement, file, line);
}

#define utAssert(_EX)                  ((_EX) ? (void)0 : _assert(#_EX, __FILE__, __LINE__))
#endif

#else
#define utAssert(exp)                                            // do nothing
#endif
#endif

extern "C" {
  //
  // Initialize rtInf needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real_T rtGetInf(void)
  {
    size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
    real_T inf = 0.0;
    if (bitsPerReal == 32U) {
      inf = rtGetInfF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0x7FF00000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      inf = tmpVal.fltVal;
    }

    return inf;
  }

  //
  // Initialize rtInfF needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real32_T rtGetInfF(void)
  {
    IEEESingle infF;
    infF.wordL.wordLuint = 0x7F800000U;
    return infF.wordL.wordLreal;
  }

  //
  // Initialize rtMinusInf needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real_T rtGetMinusInf(void)
  {
    size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
    real_T minf = 0.0;
    if (bitsPerReal == 32U) {
      minf = rtGetMinusInfF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0xFFF00000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      minf = tmpVal.fltVal;
    }

    return minf;
  }

  //
  // Initialize rtMinusInfF needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real32_T rtGetMinusInfF(void)
  {
    IEEESingle minfF;
    minfF.wordL.wordLuint = 0xFF800000U;
    return minfF.wordL.wordLreal;
  }
}
  extern "C"
{
  //
  // Initialize rtNaN needed by the generated code.
  // NaN is initialized as non-signaling. Assumes IEEE.
  //
  static real_T rtGetNaN(void)
  {
    size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
    real_T nan = 0.0;
    if (bitsPerReal == 32U) {
      nan = rtGetNaNF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0xFFF80000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      nan = tmpVal.fltVal;
    }

    return nan;
  }

  //
  // Initialize rtNaNF needed by the generated code.
  // NaN is initialized as non-signaling. Assumes IEEE.
  //
  static real32_T rtGetNaNF(void)
  {
    IEEESingle nanF = { { 0.0F } };

    nanF.wordL.wordLuint = 0xFFC00000U;
    return nanF.wordL.wordLreal;
  }
}

extern "C" {
  //
  // Initialize the rtInf, rtMinusInf, and rtNaN needed by the
  // generated code. NaN is initialized as non-signaling. Assumes IEEE.
  //
  static void rt_InitInfAndNaN(size_t realSize)
  {
    (void) (realSize);
    rtNaN = rtGetNaN();
    rtNaNF = rtGetNaNF();
    rtInf = rtGetInf();
    rtInfF = rtGetInfF();
    rtMinusInf = rtGetMinusInf();
    rtMinusInfF = rtGetMinusInfF();
  }

  // Test if value is infinite
  static boolean_T rtIsInf(real_T value)
  {
    return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
  }

  // Test if single-precision value is infinite
  static boolean_T rtIsInfF(real32_T value)
  {
    return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
  }

  // Test if value is not a number
  static boolean_T rtIsNaN(real_T value)
  {
    boolean_T result = (boolean_T) 0;
    size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
    if (bitsPerReal == 32U) {
      result = rtIsNaNF((real32_T)value);
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.fltVal = value;
      result = (boolean_T)((tmpVal.bitVal.words.wordH & 0x7FF00000) ==
                           0x7FF00000 &&
                           ( (tmpVal.bitVal.words.wordH & 0x000FFFFF) != 0 ||
                            (tmpVal.bitVal.words.wordL != 0) ));
    }

    return result;
  }

  // Test if single-precision value is not a number
  static boolean_T rtIsNaNF(real32_T value)
  {
    IEEESingle tmp;
    tmp.wordL.wordLreal = value;
    return (boolean_T)( (tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
                       (tmp.wordL.wordLuint & 0x007FFFFF) != 0 );
  }
}
//
//         Taylor Series expansion approximation of WGS84 model of
//         ellipsoid normal gravity
//
  void wgs84_taylor_series(real_T *h, real_T *phi, real_T opt_m2ft, real_T *y,
  int_T k)
{
  real_T gamma_ts, m, sinphi, sin2phi;
  int_T i;
  for (i = 0; i < k; i++ ) {
    sinphi = std::sin(phi[i]);
    sin2phi = sinphi*sinphi;

    // Calculate theoretical normal gravity (gamma) /eq. 4-1/
    gamma_ts = (WGS84_G_E)*( 1.0 + (WGS84_K)*sin2phi )/( std::sqrt(1.0 -
      (WGS84_E_2)*sin2phi) );
    m = (WGS84_A)*(WGS84_A)*(WGS84_B)*(WGS84_W_DEF)*(WGS84_W_DEF)/
      (WGS84_GM_DEF);

    // Return normal gravity as the output /eq. 4-3/
    y[i] = opt_m2ft*gamma_ts*( 1.0 - 2.0*( 1.0 + 1.0/(WGS84_INV_F) + m -
      2.0*sin2phi/(WGS84_INV_F) )*h[i]/(WGS84_A) + 3.0*h[i]*h[i]/(
      (WGS84_A)*(WGS84_A)) );
  }
}

static uint32_T plook_bincpa(real_T u, const real_T bp[], uint32_T maxIndex,
  real_T *fraction, uint32_T *prevIndex)
{
  uint32_T bpIndex;

  // Prelookup - Index and Fraction
  // Index Search method: 'binary'
  // Extrapolation method: 'Clip'
  // Use previous index: 'on'
  // Use last breakpoint for index at or above upper limit: 'on'
  // Remove protection against out-of-range input in generated code: 'off'

  if (u <= bp[0U]) {
    bpIndex = 0U;
    *fraction = 0.0;
  } else if (u < bp[maxIndex]) {
    bpIndex = binsearch_u32d_prevIdx(u, bp, *prevIndex, maxIndex);
    *fraction = (u - bp[bpIndex]) / (bp[bpIndex + 1U] - bp[bpIndex]);
  } else {
    bpIndex = maxIndex;
    *fraction = 0.0;
  }

  *prevIndex = bpIndex;
  return bpIndex;
}

static real_T intrp2d_la_pw(const uint32_T bpIndex[], const real_T frac[], const
  real_T table[], const uint32_T stride, const uint32_T maxIndex[])
{
  real_T y;
  real_T yL_0d0;
  uint32_T offset_1d;

  // Column-major Interpolation 2-D
  // Interpolation method: 'Linear point-slope'
  // Use last breakpoint for index at or above upper limit: 'on'
  // Overflow mode: 'portable wrapping'

  offset_1d = bpIndex[1U] * stride + bpIndex[0U];
  if (bpIndex[0U] == maxIndex[0U]) {
    y = table[offset_1d];
  } else {
    yL_0d0 = table[offset_1d];
    y = (table[offset_1d + 1U] - yL_0d0) * frac[0U] + yL_0d0;
  }

  if (bpIndex[1U] == maxIndex[1U]) {
  } else {
    offset_1d += stride;
    if (bpIndex[0U] == maxIndex[0U]) {
      yL_0d0 = table[offset_1d];
    } else {
      yL_0d0 = table[offset_1d];
      yL_0d0 += (table[offset_1d + 1U] - yL_0d0) * frac[0U];
    }

    y += (yL_0d0 - y) * frac[1U];
  }

  return y;
}

static uint32_T binsearch_u32d_prevIdx(real_T u, const real_T bp[], uint32_T
  startIndex, uint32_T maxIndex)
{
  uint32_T bpIndex;
  uint32_T found;
  uint32_T iLeft;
  uint32_T iRght;

  // Binary Search using Previous Index
  bpIndex = startIndex;
  iLeft = 0U;
  iRght = maxIndex;
  found = 0U;
  while (found == 0U) {
    if (u < bp[bpIndex]) {
      iRght = bpIndex - 1U;
      bpIndex = ((bpIndex + iLeft) - 1U) >> 1U;
    } else if (u < bp[bpIndex + 1U]) {
      found = 1U;
    } else {
      iLeft = bpIndex + 1U;
      bpIndex = ((bpIndex + iRght) + 1U) >> 1U;
    }
  }

  return bpIndex;
}

static void mul_wide_s32(int32_T in0, int32_T in1, uint32_T *ptrOutBitsHi,
  uint32_T *ptrOutBitsLo)
{
  uint32_T absIn0;
  uint32_T absIn1;
  uint32_T in0Hi;
  uint32_T in0Lo;
  uint32_T in1Hi;
  uint32_T productHiLo;
  uint32_T productLoHi;
  absIn0 = in0 < 0 ? ~static_cast<uint32_T>(in0) + 1U : static_cast<uint32_T>
    (in0);
  absIn1 = in1 < 0 ? ~static_cast<uint32_T>(in1) + 1U : static_cast<uint32_T>
    (in1);
  in0Hi = absIn0 >> 16U;
  in0Lo = absIn0 & 65535U;
  in1Hi = absIn1 >> 16U;
  absIn0 = absIn1 & 65535U;
  productHiLo = in0Hi * absIn0;
  productLoHi = in0Lo * in1Hi;
  absIn0 *= in0Lo;
  absIn1 = 0U;
  in0Lo = (productLoHi << /*MW:OvBitwiseOk*/ 16U) + /*MW:OvCarryOk*/ absIn0;
  if (in0Lo < absIn0) {
    absIn1 = 1U;
  }

  absIn0 = in0Lo;
  in0Lo += /*MW:OvCarryOk*/ productHiLo << /*MW:OvBitwiseOk*/ 16U;
  if (in0Lo < absIn0) {
    absIn1++;
  }

  absIn0 = (((productLoHi >> 16U) + (productHiLo >> 16U)) + in0Hi * in1Hi) +
    absIn1;
  if (static_cast<int32_T>((in0 != 0) && ((in1 != 0) && ((in0 > 0) != (in1 > 0)))))
  {
    absIn0 = ~absIn0;
    in0Lo = ~in0Lo;
    in0Lo++;
    if (in0Lo == 0U) {
      absIn0++;
    }
  }

  *ptrOutBitsHi = absIn0;
  *ptrOutBitsLo = in0Lo;
}

static int32_T mul_s32_sat(int32_T a, int32_T b)
{
  int32_T result;
  uint32_T u32_chi;
  uint32_T u32_clo;
  mul_wide_s32(a, b, &u32_chi, &u32_clo);
  if ((static_cast<int32_T>(u32_chi) > 0) || ((u32_chi == 0U) && (u32_clo >=
        2147483648U))) {
    result = MAX_int32_T;
  } else if ((static_cast<int32_T>(u32_chi) < -1) || ((static_cast<int32_T>
               (u32_chi) == -1) && (u32_clo < 2147483648U))) {
    result = MIN_int32_T;
  } else {
    result = static_cast<int32_T>(u32_clo);
  }

  return result;
}

//
//         This function updates active task flag for each subrate.
//         The function is called at model base rate, hence the
//         generated code self-manages all its subrates.
//
static void rate_scheduler(RT_MODEL_QuadModelv_T *const QuadModelv_M)
{
  // Compute which subrates run during the next base time step.  Subrates
  //  are an integer multiple of the base rate counter.  Therefore, the subtask
  //  counter is reset when it reaches its limit (zero means run).

  (QuadModelv_M->Timing.TaskCounters.TID[2])++;
  if ((QuadModelv_M->Timing.TaskCounters.TID[2]) > 2) {// Sample time: [0.003s, 0.0s] 
    QuadModelv_M->Timing.TaskCounters.TID[2] = 0;
  }

  (QuadModelv_M->Timing.TaskCounters.TID[3])++;
  if ((QuadModelv_M->Timing.TaskCounters.TID[3]) > 9) {// Sample time: [0.01s, 0.0s] 
    QuadModelv_M->Timing.TaskCounters.TID[3] = 0;
  }

  (QuadModelv_M->Timing.TaskCounters.TID[4])++;
  if ((QuadModelv_M->Timing.TaskCounters.TID[4]) > 99) {// Sample time: [0.1s, 0.0s] 
    QuadModelv_M->Timing.TaskCounters.TID[4] = 0;
  }

  (QuadModelv_M->Timing.TaskCounters.TID[5])++;
  if ((QuadModelv_M->Timing.TaskCounters.TID[5]) > 199) {// Sample time: [0.2s, 0.0s] 
    QuadModelv_M->Timing.TaskCounters.TID[5] = 0;
  }
}

//
// This function updates continuous states using the ODE4 fixed-step
// solver algorithm
//
void MulticopterModelClass::rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE4_IntgData *id = static_cast<ODE4_IntgData *>(rtsiGetSolverData(si));
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T temp;
  int_T i;
  int_T nXc = 42;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  // Save the state values at time t in y, we'll use x as ynew.
  (void) std::memcpy(y, x,
                     static_cast<uint_T>(nXc)*sizeof(real_T));

  // Assumes that rtsiSetT and ModelOutputs are up-to-date
  // f0 = f(t,y)
  rtsiSetdX(si, f0);
  QuadModelv_derivatives();

  // f1 = f(t + (h/2), y + (h/2)*f0)
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  this->step();
  QuadModelv_derivatives();

  // f2 = f(t + (h/2), y + (h/2)*f1)
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  this->step();
  QuadModelv_derivatives();

  // f3 = f(t + h, y + h*f2)
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  this->step();
  QuadModelv_derivatives();

  // tnew = t + h
  // ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3)
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    real_T tmp;
    real_T tmp_0;
    tmp = std::abs(u0);
    tmp_0 = std::abs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = (rtNaN);
    } else {
      y = std::pow(u0, u1);
    }
  }

  return y;
}

//
// System initialize for enable system:
//    '<S108>/Hpgw'
//    '<S147>/Hpgw'
//
void MulticopterModelClass::QuadModelv_Hpgw_Init(B_Hpgw_QuadModelv_T *localB,
  DW_Hpgw_QuadModelv_T *localDW, P_Hpgw_QuadModelv_T *localP)
{
  // InitializeConditions for UnitDelay: '<S119>/Unit Delay'
  localDW->UnitDelay_DSTATE[0] = localP->UnitDelay_InitialCondition;

  // SystemInitialize for Sum: '<S119>/Sum' incorporates:
  //   Outport: '<S119>/pgw'

  localB->Sum[0] = localP->pgw_Y0;

  // InitializeConditions for UnitDelay: '<S119>/Unit Delay'
  localDW->UnitDelay_DSTATE[1] = localP->UnitDelay_InitialCondition;

  // SystemInitialize for Sum: '<S119>/Sum' incorporates:
  //   Outport: '<S119>/pgw'

  localB->Sum[1] = localP->pgw_Y0;
}

//
// System reset for enable system:
//    '<S108>/Hpgw'
//    '<S147>/Hpgw'
//
void MulticopterModelClass::QuadModelv_Hpgw_Reset(DW_Hpgw_QuadModelv_T *localDW,
  P_Hpgw_QuadModelv_T *localP)
{
  // InitializeConditions for UnitDelay: '<S119>/Unit Delay'
  localDW->UnitDelay_DSTATE[0] = localP->UnitDelay_InitialCondition;
  localDW->UnitDelay_DSTATE[1] = localP->UnitDelay_InitialCondition;
}

//
// Disable for enable system:
//    '<S108>/Hpgw'
//    '<S147>/Hpgw'
//
void MulticopterModelClass::QuadModelv_Hpgw_Disable(B_Hpgw_QuadModelv_T *localB,
  DW_Hpgw_QuadModelv_T *localDW, P_Hpgw_QuadModelv_T *localP)
{
  // Disable for Sum: '<S119>/Sum' incorporates:
  //   Outport: '<S119>/pgw'

  localB->Sum[0] = localP->pgw_Y0;
  localB->Sum[1] = localP->pgw_Y0;
  localDW->Hpgw_MODE = false;
}

//
// Outputs for enable system:
//    '<S108>/Hpgw'
//    '<S147>/Hpgw'
//
void MulticopterModelClass::QuadModelv_Hpgw(real_T rtu_Enable, const real_T
  rtu_L_wg[2], real_T rtu_sigma_wg, real_T rtu_sigma_wg_e, real_T rtu_Noise,
  real_T rtu_wingspan, B_Hpgw_QuadModelv_T *localB, DW_Hpgw_QuadModelv_T
  *localDW, P_Hpgw_QuadModelv_T *localP)
{
  // Outputs for Enabled SubSystem: '<S108>/Hpgw' incorporates:
  //   EnablePort: '<S119>/Enable'

  if ((rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) &&
      rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if (rtu_Enable > 0.0) {
      if (!localDW->Hpgw_MODE) {
        QuadModelv_Hpgw_Reset(localDW, localP);
        localDW->Hpgw_MODE = true;
      }
    } else if (localDW->Hpgw_MODE) {
      QuadModelv_Hpgw_Disable(localB, localDW, localP);
    }
  }

  if (localDW->Hpgw_MODE) {
    real_T rtb_ap_idx_0;
    real_T rtb_ap_idx_1;
    real_T rtb_sp;
    real_T rtb_sp_idx_0;
    real_T tmp;

    // Product: '<S119>/w2'
    rtb_sp = rtu_L_wg[0] * rtu_wingspan;

    // Product: '<S119>/w1' incorporates:
    //   Constant: '<S119>/Constant2'
    //   Sqrt: '<S119>/sqrt'

    rtb_ap_idx_0 = localP->Constant2_Value / std::sqrt(rtb_sp);

    // Product: '<S119>/w2'
    rtb_sp_idx_0 = rtb_sp;
    rtb_sp = rtu_L_wg[1] * rtu_wingspan;

    // Product: '<S119>/w1' incorporates:
    //   Constant: '<S119>/Constant2'
    //   Sqrt: '<S119>/sqrt'

    rtb_ap_idx_1 = localP->Constant2_Value / std::sqrt(rtb_sp);
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
      // UnitDelay: '<S119>/Unit Delay'
      localB->UnitDelay[0] = localDW->UnitDelay_DSTATE[0];
      localB->UnitDelay[1] = localDW->UnitDelay_DSTATE[1];
    }

    // Product: '<S119>/w4'
    rtb_sp_idx_0 *= rtu_wingspan;

    // Math: '<S119>/Math Function' incorporates:
    //   Constant: '<S119>/Constant3'

    tmp = std::floor(localP->Constant3_Value);
    if ((rtb_sp_idx_0 < 0.0) && (localP->Constant3_Value > tmp)) {
      rtb_sp_idx_0 = -rt_powd_snf(-rtb_sp_idx_0, localP->Constant3_Value);
    } else {
      rtb_sp_idx_0 = rt_powd_snf(rtb_sp_idx_0, localP->Constant3_Value);
    }

    // Sum: '<S119>/Sum' incorporates:
    //   Constant: '<S119>/Constant'
    //   Constant: '<S119>/Constant1'
    //   Gain: '<S119>/2'
    //   Gain: '<S119>/dt'
    //   Product: '<S119>/Lug//V1'
    //   Product: '<S119>/Lug//V2'
    //   Product: '<S119>/w3'
    //   Sqrt: '<S119>/sqrt1'
    //   Sum: '<S119>/Sum1'

    localB->Sum[0] = localP->Constant1_Value / rtb_sp_idx_0 * rtu_sigma_wg * std::
      sqrt(localP->u_Gain * rtb_ap_idx_0) * rtu_Noise + (localP->Constant_Value
      - localP->dt_Gain * rtb_ap_idx_0) * localB->UnitDelay[0];

    // Product: '<S119>/w4'
    rtb_sp_idx_0 = rtb_sp * rtu_wingspan;

    // Math: '<S119>/Math Function' incorporates:
    //   Constant: '<S119>/Constant3'

    if ((rtb_sp_idx_0 < 0.0) && (localP->Constant3_Value > tmp)) {
      rtb_sp_idx_0 = -rt_powd_snf(-rtb_sp_idx_0, localP->Constant3_Value);
    } else {
      rtb_sp_idx_0 = rt_powd_snf(rtb_sp_idx_0, localP->Constant3_Value);
    }

    // Sum: '<S119>/Sum' incorporates:
    //   Constant: '<S119>/Constant'
    //   Constant: '<S119>/Constant1'
    //   Gain: '<S119>/2'
    //   Gain: '<S119>/dt'
    //   Product: '<S119>/Lug//V1'
    //   Product: '<S119>/Lug//V2'
    //   Product: '<S119>/w3'
    //   Sqrt: '<S119>/sqrt1'
    //   Sum: '<S119>/Sum1'

    localB->Sum[1] = localP->Constant1_Value / rtb_sp_idx_0 * rtu_sigma_wg_e *
      std::sqrt(localP->u_Gain * rtb_ap_idx_1) * rtu_Noise +
      (localP->Constant_Value - localP->dt_Gain * rtb_ap_idx_1) *
      localB->UnitDelay[1];
  }

  // End of Outputs for SubSystem: '<S108>/Hpgw'
}

//
// Update for enable system:
//    '<S108>/Hpgw'
//    '<S147>/Hpgw'
//
void MulticopterModelClass::QuadModelv_Hpgw_Update(B_Hpgw_QuadModelv_T *localB,
  DW_Hpgw_QuadModelv_T *localDW)
{
  // Update for Enabled SubSystem: '<S108>/Hpgw' incorporates:
  //   EnablePort: '<S119>/Enable'

  if (localDW->Hpgw_MODE && (rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0)) {
    // Update for UnitDelay: '<S119>/Unit Delay'
    localDW->UnitDelay_DSTATE[0] = localB->Sum[0];
    localDW->UnitDelay_DSTATE[1] = localB->Sum[1];
  }

  // End of Update for SubSystem: '<S108>/Hpgw'
}

//
// System initialize for enable system:
//    '<S108>/Hqgw'
//    '<S147>/Hqgw'
//
void MulticopterModelClass::QuadModelv_Hqgw_Init(B_Hqgw_QuadModelv_T *localB,
  DW_Hqgw_QuadModelv_T *localDW, P_Hqgw_QuadModelv_T *localP)
{
  // InitializeConditions for UnitDelay: '<S120>/Unit Delay'
  localDW->UnitDelay_DSTATE[0] = localP->UnitDelay_InitialCondition;

  // InitializeConditions for UnitDelay: '<S120>/Unit Delay1'
  localDW->UnitDelay1_DSTATE[0] = localP->UnitDelay1_InitialCondition;

  // SystemInitialize for Sum: '<S120>/Sum1' incorporates:
  //   Outport: '<S120>/qgw'

  localB->Sum1[0] = localP->qgw_Y0;

  // InitializeConditions for UnitDelay: '<S120>/Unit Delay'
  localDW->UnitDelay_DSTATE[1] = localP->UnitDelay_InitialCondition;

  // InitializeConditions for UnitDelay: '<S120>/Unit Delay1'
  localDW->UnitDelay1_DSTATE[1] = localP->UnitDelay1_InitialCondition;

  // SystemInitialize for Sum: '<S120>/Sum1' incorporates:
  //   Outport: '<S120>/qgw'

  localB->Sum1[1] = localP->qgw_Y0;
}

//
// System reset for enable system:
//    '<S108>/Hqgw'
//    '<S147>/Hqgw'
//
void MulticopterModelClass::QuadModelv_Hqgw_Reset(DW_Hqgw_QuadModelv_T *localDW,
  P_Hqgw_QuadModelv_T *localP)
{
  // InitializeConditions for UnitDelay: '<S120>/Unit Delay'
  localDW->UnitDelay_DSTATE[0] = localP->UnitDelay_InitialCondition;

  // InitializeConditions for UnitDelay: '<S120>/Unit Delay1'
  localDW->UnitDelay1_DSTATE[0] = localP->UnitDelay1_InitialCondition;

  // InitializeConditions for UnitDelay: '<S120>/Unit Delay'
  localDW->UnitDelay_DSTATE[1] = localP->UnitDelay_InitialCondition;

  // InitializeConditions for UnitDelay: '<S120>/Unit Delay1'
  localDW->UnitDelay1_DSTATE[1] = localP->UnitDelay1_InitialCondition;
}

//
// Disable for enable system:
//    '<S108>/Hqgw'
//    '<S147>/Hqgw'
//
void MulticopterModelClass::QuadModelv_Hqgw_Disable(B_Hqgw_QuadModelv_T *localB,
  DW_Hqgw_QuadModelv_T *localDW, P_Hqgw_QuadModelv_T *localP)
{
  // Disable for Sum: '<S120>/Sum1' incorporates:
  //   Outport: '<S120>/qgw'

  localB->Sum1[0] = localP->qgw_Y0;
  localB->Sum1[1] = localP->qgw_Y0;
  localDW->Hqgw_MODE = false;
}

//
// Outputs for enable system:
//    '<S108>/Hqgw'
//    '<S147>/Hqgw'
//
void MulticopterModelClass::QuadModelv_Hqgw(real_T rtu_Enable, real_T rtu_V,
  const real_T rtu_wg[2], real_T rtu_wingspan, B_Hqgw_QuadModelv_T *localB,
  DW_Hqgw_QuadModelv_T *localDW, P_Hqgw_QuadModelv_T *localP)
{
  // Outputs for Enabled SubSystem: '<S108>/Hqgw' incorporates:
  //   EnablePort: '<S120>/Enable'

  if ((rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) &&
      rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if (rtu_Enable > 0.0) {
      if (!localDW->Hqgw_MODE) {
        QuadModelv_Hqgw_Reset(localDW, localP);
        localDW->Hqgw_MODE = true;
      }
    } else if (localDW->Hqgw_MODE) {
      QuadModelv_Hqgw_Disable(localB, localDW, localP);
    }
  }

  if (localDW->Hqgw_MODE) {
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
      // Gain: '<S120>/dt1'
      localB->dt1 = localP->dt1_Gain * rtu_wingspan;

      // Sum: '<S120>/Sum2' incorporates:
      //   Constant: '<S120>/Constant'
      //   Gain: '<S120>/dt'
      //   Product: '<S120>/w1'

      localB->Sum2 = localP->Constant_Value - rtu_V / localB->dt1 *
        localP->dt_Gain;
    }

    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
      // Product: '<S120>/Lug//V2' incorporates:
      //   UnitDelay: '<S120>/Unit Delay'

      localB->LugV2[0] = localB->Sum2 * localDW->UnitDelay_DSTATE[0];

      // UnitDelay: '<S120>/Unit Delay1'
      localB->UnitDelay1[0] = localDW->UnitDelay1_DSTATE[0];

      // Product: '<S120>/Lug//V2' incorporates:
      //   UnitDelay: '<S120>/Unit Delay'

      localB->LugV2[1] = localB->Sum2 * localDW->UnitDelay_DSTATE[1];

      // UnitDelay: '<S120>/Unit Delay1'
      localB->UnitDelay1[1] = localDW->UnitDelay1_DSTATE[1];
    }

    // Sum: '<S120>/Sum1' incorporates:
    //   Product: '<S120>/w2'
    //   Sum: '<S120>/Sum3'

    localB->Sum1[0] = localB->LugV2[0] - (rtu_wg[0] - localB->UnitDelay1[0]) /
      localB->dt1;
    localB->Sum1[1] = localB->LugV2[1] - (rtu_wg[1] - localB->UnitDelay1[1]) /
      localB->dt1;
  }

  // End of Outputs for SubSystem: '<S108>/Hqgw'
}

//
// Update for enable system:
//    '<S108>/Hqgw'
//    '<S147>/Hqgw'
//
void MulticopterModelClass::QuadModelv_Hqgw_Update(const real_T rtu_wg[2],
  B_Hqgw_QuadModelv_T *localB, DW_Hqgw_QuadModelv_T *localDW)
{
  // Update for Enabled SubSystem: '<S108>/Hqgw' incorporates:
  //   EnablePort: '<S120>/Enable'

  if (localDW->Hqgw_MODE && (rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0)) {
    // Update for UnitDelay: '<S120>/Unit Delay'
    localDW->UnitDelay_DSTATE[0] = localB->Sum1[0];

    // Update for UnitDelay: '<S120>/Unit Delay1'
    localDW->UnitDelay1_DSTATE[0] = rtu_wg[0];

    // Update for UnitDelay: '<S120>/Unit Delay'
    localDW->UnitDelay_DSTATE[1] = localB->Sum1[1];

    // Update for UnitDelay: '<S120>/Unit Delay1'
    localDW->UnitDelay1_DSTATE[1] = rtu_wg[1];
  }

  // End of Update for SubSystem: '<S108>/Hqgw'
}

//
// System initialize for enable system:
//    '<S108>/Hrgw'
//    '<S147>/Hrgw'
//
void MulticopterModelClass::QuadModelv_Hrgw_Init(B_Hrgw_QuadModelv_T *localB,
  DW_Hrgw_QuadModelv_T *localDW, P_Hrgw_QuadModelv_T *localP)
{
  // InitializeConditions for UnitDelay: '<S121>/Unit Delay'
  localDW->UnitDelay_DSTATE[0] = localP->UnitDelay_InitialCondition;

  // InitializeConditions for UnitDelay: '<S121>/Unit Delay1'
  localDW->UnitDelay1_DSTATE[0] = localP->UnitDelay1_InitialCondition;

  // SystemInitialize for Sum: '<S121>/Sum1' incorporates:
  //   Outport: '<S121>/rgw'

  localB->Sum1[0] = localP->rgw_Y0;

  // InitializeConditions for UnitDelay: '<S121>/Unit Delay'
  localDW->UnitDelay_DSTATE[1] = localP->UnitDelay_InitialCondition;

  // InitializeConditions for UnitDelay: '<S121>/Unit Delay1'
  localDW->UnitDelay1_DSTATE[1] = localP->UnitDelay1_InitialCondition;

  // SystemInitialize for Sum: '<S121>/Sum1' incorporates:
  //   Outport: '<S121>/rgw'

  localB->Sum1[1] = localP->rgw_Y0;
}

//
// System reset for enable system:
//    '<S108>/Hrgw'
//    '<S147>/Hrgw'
//
void MulticopterModelClass::QuadModelv_Hrgw_Reset(DW_Hrgw_QuadModelv_T *localDW,
  P_Hrgw_QuadModelv_T *localP)
{
  // InitializeConditions for UnitDelay: '<S121>/Unit Delay'
  localDW->UnitDelay_DSTATE[0] = localP->UnitDelay_InitialCondition;

  // InitializeConditions for UnitDelay: '<S121>/Unit Delay1'
  localDW->UnitDelay1_DSTATE[0] = localP->UnitDelay1_InitialCondition;

  // InitializeConditions for UnitDelay: '<S121>/Unit Delay'
  localDW->UnitDelay_DSTATE[1] = localP->UnitDelay_InitialCondition;

  // InitializeConditions for UnitDelay: '<S121>/Unit Delay1'
  localDW->UnitDelay1_DSTATE[1] = localP->UnitDelay1_InitialCondition;
}

//
// Disable for enable system:
//    '<S108>/Hrgw'
//    '<S147>/Hrgw'
//
void MulticopterModelClass::QuadModelv_Hrgw_Disable(B_Hrgw_QuadModelv_T *localB,
  DW_Hrgw_QuadModelv_T *localDW, P_Hrgw_QuadModelv_T *localP)
{
  // Disable for Sum: '<S121>/Sum1' incorporates:
  //   Outport: '<S121>/rgw'

  localB->Sum1[0] = localP->rgw_Y0;
  localB->Sum1[1] = localP->rgw_Y0;
  localDW->Hrgw_MODE = false;
}

//
// Outputs for enable system:
//    '<S108>/Hrgw'
//    '<S147>/Hrgw'
//
void MulticopterModelClass::QuadModelv_Hrgw(real_T rtu_Enable, real_T rtu_V,
  const real_T rtu_vg[2], real_T rtu_wingspan, B_Hrgw_QuadModelv_T *localB,
  DW_Hrgw_QuadModelv_T *localDW, P_Hrgw_QuadModelv_T *localP)
{
  // Outputs for Enabled SubSystem: '<S108>/Hrgw' incorporates:
  //   EnablePort: '<S121>/Enable'

  if ((rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) &&
      rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if (rtu_Enable > 0.0) {
      if (!localDW->Hrgw_MODE) {
        QuadModelv_Hrgw_Reset(localDW, localP);
        localDW->Hrgw_MODE = true;
      }
    } else if (localDW->Hrgw_MODE) {
      QuadModelv_Hrgw_Disable(localB, localDW, localP);
    }
  }

  if (localDW->Hrgw_MODE) {
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
      // Gain: '<S121>/dt1'
      localB->dt1 = localP->dt1_Gain * rtu_wingspan;

      // Sum: '<S121>/Sum2' incorporates:
      //   Constant: '<S121>/Constant'
      //   Gain: '<S121>/dt'
      //   Product: '<S121>/w1'

      localB->Sum2 = localP->Constant_Value - rtu_V / localB->dt1 *
        localP->dt_Gain;
    }

    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
      // Product: '<S121>/Lug//V2' incorporates:
      //   UnitDelay: '<S121>/Unit Delay'

      localB->LugV2[0] = localB->Sum2 * localDW->UnitDelay_DSTATE[0];

      // UnitDelay: '<S121>/Unit Delay1'
      localB->UnitDelay1[0] = localDW->UnitDelay1_DSTATE[0];

      // Product: '<S121>/Lug//V2' incorporates:
      //   UnitDelay: '<S121>/Unit Delay'

      localB->LugV2[1] = localB->Sum2 * localDW->UnitDelay_DSTATE[1];

      // UnitDelay: '<S121>/Unit Delay1'
      localB->UnitDelay1[1] = localDW->UnitDelay1_DSTATE[1];
    }

    // Sum: '<S121>/Sum1' incorporates:
    //   Product: '<S121>/w2'
    //   Sum: '<S121>/Sum3'

    localB->Sum1[0] = (rtu_vg[0] - localB->UnitDelay1[0]) / localB->dt1 +
      localB->LugV2[0];
    localB->Sum1[1] = (rtu_vg[1] - localB->UnitDelay1[1]) / localB->dt1 +
      localB->LugV2[1];
  }

  // End of Outputs for SubSystem: '<S108>/Hrgw'
}

//
// Update for enable system:
//    '<S108>/Hrgw'
//    '<S147>/Hrgw'
//
void MulticopterModelClass::QuadModelv_Hrgw_Update(const real_T rtu_vg[2],
  B_Hrgw_QuadModelv_T *localB, DW_Hrgw_QuadModelv_T *localDW)
{
  // Update for Enabled SubSystem: '<S108>/Hrgw' incorporates:
  //   EnablePort: '<S121>/Enable'

  if (localDW->Hrgw_MODE && (rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0)) {
    // Update for UnitDelay: '<S121>/Unit Delay'
    localDW->UnitDelay_DSTATE[0] = localB->Sum1[0];

    // Update for UnitDelay: '<S121>/Unit Delay1'
    localDW->UnitDelay1_DSTATE[0] = rtu_vg[0];

    // Update for UnitDelay: '<S121>/Unit Delay'
    localDW->UnitDelay_DSTATE[1] = localB->Sum1[1];

    // Update for UnitDelay: '<S121>/Unit Delay1'
    localDW->UnitDelay1_DSTATE[1] = rtu_vg[1];
  }

  // End of Update for SubSystem: '<S108>/Hrgw'
}

//
// System initialize for enable system:
//    '<S109>/Hugw(z)'
//    '<S148>/Hugw(z)'
//
void MulticopterModelClass::QuadModelv_Hugwz_Init(B_Hugwz_QuadModelv_T *localB,
  DW_Hugwz_QuadModelv_T *localDW, P_Hugwz_QuadModelv_T *localP)
{
  // InitializeConditions for UnitDelay: '<S122>/Unit Delay'
  localDW->UnitDelay_DSTATE[0] = localP->UnitDelay_InitialCondition;

  // SystemInitialize for Sum: '<S122>/Sum' incorporates:
  //   Outport: '<S122>/ugw'

  localB->Sum[0] = localP->ugw_Y0;

  // InitializeConditions for UnitDelay: '<S122>/Unit Delay'
  localDW->UnitDelay_DSTATE[1] = localP->UnitDelay_InitialCondition;

  // SystemInitialize for Sum: '<S122>/Sum' incorporates:
  //   Outport: '<S122>/ugw'

  localB->Sum[1] = localP->ugw_Y0;
}

//
// System reset for enable system:
//    '<S109>/Hugw(z)'
//    '<S148>/Hugw(z)'
//
void MulticopterModelClass::QuadModelv_Hugwz_Reset(DW_Hugwz_QuadModelv_T
  *localDW, P_Hugwz_QuadModelv_T *localP)
{
  // InitializeConditions for UnitDelay: '<S122>/Unit Delay'
  localDW->UnitDelay_DSTATE[0] = localP->UnitDelay_InitialCondition;
  localDW->UnitDelay_DSTATE[1] = localP->UnitDelay_InitialCondition;
}

//
// Disable for enable system:
//    '<S109>/Hugw(z)'
//    '<S148>/Hugw(z)'
//
void MulticopterModelClass::QuadModelv_Hugwz_Disable(B_Hugwz_QuadModelv_T
  *localB, DW_Hugwz_QuadModelv_T *localDW, P_Hugwz_QuadModelv_T *localP)
{
  // Disable for Sum: '<S122>/Sum' incorporates:
  //   Outport: '<S122>/ugw'

  localB->Sum[0] = localP->ugw_Y0;
  localB->Sum[1] = localP->ugw_Y0;
  localDW->Hugwz_MODE = false;
}

//
// Outputs for enable system:
//    '<S109>/Hugw(z)'
//    '<S148>/Hugw(z)'
//
void MulticopterModelClass::QuadModelv_Hugwz(real_T rtu_Enable, real_T rtu_V,
  real_T rtu_L_ug, real_T rtu_L_ug_i, real_T rtu_sigma_ug, real_T rtu_sigma_ug_j,
  real_T rtu_Noise, B_Hugwz_QuadModelv_T *localB, DW_Hugwz_QuadModelv_T *localDW,
  P_Hugwz_QuadModelv_T *localP)
{
  // Outputs for Enabled SubSystem: '<S109>/Hugw(z)' incorporates:
  //   EnablePort: '<S122>/Enable'

  if ((rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) &&
      rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if (rtu_Enable > 0.0) {
      if (!localDW->Hugwz_MODE) {
        QuadModelv_Hugwz_Reset(localDW, localP);
        localDW->Hugwz_MODE = true;
      }
    } else if (localDW->Hugwz_MODE) {
      QuadModelv_Hugwz_Disable(localB, localDW, localP);
    }
  }

  if (localDW->Hugwz_MODE) {
    real_T rtb_VLug_idx_0;
    real_T rtb_VLug_idx_1;

    // Product: '<S122>/V//Lug'
    rtb_VLug_idx_0 = rtu_V / rtu_L_ug;
    rtb_VLug_idx_1 = rtu_V / rtu_L_ug_i;
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
      // UnitDelay: '<S122>/Unit Delay'
      localB->UnitDelay[0] = localDW->UnitDelay_DSTATE[0];
      localB->UnitDelay[1] = localDW->UnitDelay_DSTATE[1];
    }

    // Sum: '<S122>/Sum' incorporates:
    //   Constant: '<S122>/Constant'
    //   Gain: '<S122>/2'
    //   Gain: '<S122>/dt'
    //   Product: '<S122>/Lug//V1'
    //   Product: '<S122>/Lug//V2'
    //   Sqrt: '<S122>/sqrt'
    //   Sum: '<S122>/Sum1'

    localB->Sum[0] = (localP->Constant_Value - localP->dt_Gain * rtb_VLug_idx_0)
      * localB->UnitDelay[0] + std::sqrt(localP->u_Gain * rtb_VLug_idx_0) *
      rtu_Noise * rtu_sigma_ug;
    localB->Sum[1] = (localP->Constant_Value - localP->dt_Gain * rtb_VLug_idx_1)
      * localB->UnitDelay[1] + std::sqrt(localP->u_Gain * rtb_VLug_idx_1) *
      rtu_Noise * rtu_sigma_ug_j;
  }

  // End of Outputs for SubSystem: '<S109>/Hugw(z)'
}

//
// Update for enable system:
//    '<S109>/Hugw(z)'
//    '<S148>/Hugw(z)'
//
void MulticopterModelClass::QuadModelv_Hugwz_Update(B_Hugwz_QuadModelv_T *localB,
  DW_Hugwz_QuadModelv_T *localDW)
{
  // Update for Enabled SubSystem: '<S109>/Hugw(z)' incorporates:
  //   EnablePort: '<S122>/Enable'

  if (localDW->Hugwz_MODE && (rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0)) {
    // Update for UnitDelay: '<S122>/Unit Delay'
    localDW->UnitDelay_DSTATE[0] = localB->Sum[0];
    localDW->UnitDelay_DSTATE[1] = localB->Sum[1];
  }

  // End of Update for SubSystem: '<S109>/Hugw(z)'
}

//
// System initialize for enable system:
//    '<S109>/Hvgw(z)'
//    '<S148>/Hvgw(z)'
//
void MulticopterModelClass::QuadModelv_Hvgwz_Init(B_Hvgwz_QuadModelv_T *localB,
  DW_Hvgwz_QuadModelv_T *localDW, P_Hvgwz_QuadModelv_T *localP)
{
  // InitializeConditions for UnitDelay: '<S123>/Unit Delay'
  localDW->UnitDelay_DSTATE[0] = localP->UnitDelay_InitialCondition;

  // SystemInitialize for Sum: '<S123>/Sum' incorporates:
  //   Outport: '<S123>/vgw'

  localB->Sum[0] = localP->vgw_Y0;

  // InitializeConditions for UnitDelay: '<S123>/Unit Delay'
  localDW->UnitDelay_DSTATE[1] = localP->UnitDelay_InitialCondition;

  // SystemInitialize for Sum: '<S123>/Sum' incorporates:
  //   Outport: '<S123>/vgw'

  localB->Sum[1] = localP->vgw_Y0;
}

//
// System reset for enable system:
//    '<S109>/Hvgw(z)'
//    '<S148>/Hvgw(z)'
//
void MulticopterModelClass::QuadModelv_Hvgwz_Reset(DW_Hvgwz_QuadModelv_T
  *localDW, P_Hvgwz_QuadModelv_T *localP)
{
  // InitializeConditions for UnitDelay: '<S123>/Unit Delay'
  localDW->UnitDelay_DSTATE[0] = localP->UnitDelay_InitialCondition;
  localDW->UnitDelay_DSTATE[1] = localP->UnitDelay_InitialCondition;
}

//
// Disable for enable system:
//    '<S109>/Hvgw(z)'
//    '<S148>/Hvgw(z)'
//
void MulticopterModelClass::QuadModelv_Hvgwz_Disable(B_Hvgwz_QuadModelv_T
  *localB, DW_Hvgwz_QuadModelv_T *localDW, P_Hvgwz_QuadModelv_T *localP)
{
  // Disable for Sum: '<S123>/Sum' incorporates:
  //   Outport: '<S123>/vgw'

  localB->Sum[0] = localP->vgw_Y0;
  localB->Sum[1] = localP->vgw_Y0;
  localDW->Hvgwz_MODE = false;
}

//
// Outputs for enable system:
//    '<S109>/Hvgw(z)'
//    '<S148>/Hvgw(z)'
//
void MulticopterModelClass::QuadModelv_Hvgwz(real_T rtu_Enable, real_T
  rtu_sigma_vg, real_T rtu_sigma_vg_e, const real_T rtu_L_vg[2], real_T rtu_V,
  real_T rtu_Noise, B_Hvgwz_QuadModelv_T *localB, DW_Hvgwz_QuadModelv_T *localDW,
  P_Hvgwz_QuadModelv_T *localP)
{
  // Outputs for Enabled SubSystem: '<S109>/Hvgw(z)' incorporates:
  //   EnablePort: '<S123>/Enable'

  if ((rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) &&
      rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if (rtu_Enable > 0.0) {
      if (!localDW->Hvgwz_MODE) {
        QuadModelv_Hvgwz_Reset(localDW, localP);
        localDW->Hvgwz_MODE = true;
      }
    } else if (localDW->Hvgwz_MODE) {
      QuadModelv_Hvgwz_Disable(localB, localDW, localP);
    }
  }

  if (localDW->Hvgwz_MODE) {
    real_T rtb_VLvg_idx_0;
    real_T rtb_VLvg_idx_1;

    // Product: '<S123>/V//Lvg'
    rtb_VLvg_idx_0 = rtu_V / rtu_L_vg[0];
    rtb_VLvg_idx_1 = rtu_V / rtu_L_vg[1];
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
      // UnitDelay: '<S123>/Unit Delay'
      localB->UnitDelay[0] = localDW->UnitDelay_DSTATE[0];
      localB->UnitDelay[1] = localDW->UnitDelay_DSTATE[1];
    }

    // Sum: '<S123>/Sum' incorporates:
    //   Constant: '<S123>/Constant'
    //   Gain: '<S123>/2'
    //   Gain: '<S123>/dt'
    //   Product: '<S123>/Lug//V1'
    //   Product: '<S123>/Lug//V2'
    //   Sqrt: '<S123>/sqrt'
    //   Sum: '<S123>/Sum1'

    localB->Sum[0] = (localP->Constant_Value - localP->dt_Gain * rtb_VLvg_idx_0)
      * localB->UnitDelay[0] + std::sqrt(localP->u_Gain * rtb_VLvg_idx_0) *
      rtu_Noise * rtu_sigma_vg;
    localB->Sum[1] = (localP->Constant_Value - localP->dt_Gain * rtb_VLvg_idx_1)
      * localB->UnitDelay[1] + std::sqrt(localP->u_Gain * rtb_VLvg_idx_1) *
      rtu_Noise * rtu_sigma_vg_e;
  }

  // End of Outputs for SubSystem: '<S109>/Hvgw(z)'
}

//
// Update for enable system:
//    '<S109>/Hvgw(z)'
//    '<S148>/Hvgw(z)'
//
void MulticopterModelClass::QuadModelv_Hvgwz_Update(B_Hvgwz_QuadModelv_T *localB,
  DW_Hvgwz_QuadModelv_T *localDW)
{
  // Update for Enabled SubSystem: '<S109>/Hvgw(z)' incorporates:
  //   EnablePort: '<S123>/Enable'

  if (localDW->Hvgwz_MODE && (rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0)) {
    // Update for UnitDelay: '<S123>/Unit Delay'
    localDW->UnitDelay_DSTATE[0] = localB->Sum[0];
    localDW->UnitDelay_DSTATE[1] = localB->Sum[1];
  }

  // End of Update for SubSystem: '<S109>/Hvgw(z)'
}

//
// System initialize for enable system:
//    '<S109>/Hwgw(z)'
//    '<S148>/Hwgw(z)'
//
void MulticopterModelClass::QuadModelv_Hwgwz_Init(B_Hwgwz_QuadModelv_T *localB,
  DW_Hwgwz_QuadModelv_T *localDW, P_Hwgwz_QuadModelv_T *localP)
{
  // InitializeConditions for UnitDelay: '<S124>/Unit Delay'
  localDW->UnitDelay_DSTATE[0] = localP->UnitDelay_InitialCondition;

  // SystemInitialize for Sum: '<S124>/Sum' incorporates:
  //   Outport: '<S124>/wgw'

  localB->Sum[0] = localP->wgw_Y0;

  // InitializeConditions for UnitDelay: '<S124>/Unit Delay'
  localDW->UnitDelay_DSTATE[1] = localP->UnitDelay_InitialCondition;

  // SystemInitialize for Sum: '<S124>/Sum' incorporates:
  //   Outport: '<S124>/wgw'

  localB->Sum[1] = localP->wgw_Y0;
}

//
// System reset for enable system:
//    '<S109>/Hwgw(z)'
//    '<S148>/Hwgw(z)'
//
void MulticopterModelClass::QuadModelv_Hwgwz_Reset(DW_Hwgwz_QuadModelv_T
  *localDW, P_Hwgwz_QuadModelv_T *localP)
{
  // InitializeConditions for UnitDelay: '<S124>/Unit Delay'
  localDW->UnitDelay_DSTATE[0] = localP->UnitDelay_InitialCondition;
  localDW->UnitDelay_DSTATE[1] = localP->UnitDelay_InitialCondition;
}

//
// Disable for enable system:
//    '<S109>/Hwgw(z)'
//    '<S148>/Hwgw(z)'
//
void MulticopterModelClass::QuadModelv_Hwgwz_Disable(B_Hwgwz_QuadModelv_T
  *localB, DW_Hwgwz_QuadModelv_T *localDW, P_Hwgwz_QuadModelv_T *localP)
{
  // Disable for Sum: '<S124>/Sum' incorporates:
  //   Outport: '<S124>/wgw'

  localB->Sum[0] = localP->wgw_Y0;
  localB->Sum[1] = localP->wgw_Y0;
  localDW->Hwgwz_MODE = false;
}

//
// Outputs for enable system:
//    '<S109>/Hwgw(z)'
//    '<S148>/Hwgw(z)'
//
void MulticopterModelClass::QuadModelv_Hwgwz(real_T rtu_Enable, real_T rtu_V,
  const real_T rtu_L_wg[2], real_T rtu_sigma_wg, real_T rtu_sigma_wg_j, real_T
  rtu_Noise, B_Hwgwz_QuadModelv_T *localB, DW_Hwgwz_QuadModelv_T *localDW,
  P_Hwgwz_QuadModelv_T *localP)
{
  // Outputs for Enabled SubSystem: '<S109>/Hwgw(z)' incorporates:
  //   EnablePort: '<S124>/Enable'

  if ((rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) &&
      rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if (rtu_Enable > 0.0) {
      if (!localDW->Hwgwz_MODE) {
        QuadModelv_Hwgwz_Reset(localDW, localP);
        localDW->Hwgwz_MODE = true;
      }
    } else if (localDW->Hwgwz_MODE) {
      QuadModelv_Hwgwz_Disable(localB, localDW, localP);
    }
  }

  if (localDW->Hwgwz_MODE) {
    real_T rtb_VLwg_idx_0;
    real_T rtb_VLwg_idx_1;

    // Product: '<S124>/V//Lwg'
    rtb_VLwg_idx_0 = rtu_V / rtu_L_wg[0];
    rtb_VLwg_idx_1 = rtu_V / rtu_L_wg[1];
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
      // UnitDelay: '<S124>/Unit Delay'
      localB->UnitDelay[0] = localDW->UnitDelay_DSTATE[0];
      localB->UnitDelay[1] = localDW->UnitDelay_DSTATE[1];
    }

    // Sum: '<S124>/Sum' incorporates:
    //   Constant: '<S124>/Constant'
    //   Gain: '<S124>/2'
    //   Gain: '<S124>/dt'
    //   Product: '<S124>/Lug//V1'
    //   Product: '<S124>/Lug//V2'
    //   Sqrt: '<S124>/sqrt'
    //   Sum: '<S124>/Sum1'

    localB->Sum[0] = (localP->Constant_Value - localP->dt_Gain * rtb_VLwg_idx_0)
      * localB->UnitDelay[0] + std::sqrt(localP->u_Gain * rtb_VLwg_idx_0) *
      rtu_Noise * rtu_sigma_wg;
    localB->Sum[1] = (localP->Constant_Value - localP->dt_Gain * rtb_VLwg_idx_1)
      * localB->UnitDelay[1] + std::sqrt(localP->u_Gain * rtb_VLwg_idx_1) *
      rtu_Noise * rtu_sigma_wg_j;
  }

  // End of Outputs for SubSystem: '<S109>/Hwgw(z)'
}

//
// Update for enable system:
//    '<S109>/Hwgw(z)'
//    '<S148>/Hwgw(z)'
//
void MulticopterModelClass::QuadModelv_Hwgwz_Update(B_Hwgwz_QuadModelv_T *localB,
  DW_Hwgwz_QuadModelv_T *localDW)
{
  // Update for Enabled SubSystem: '<S109>/Hwgw(z)' incorporates:
  //   EnablePort: '<S124>/Enable'

  if (localDW->Hwgwz_MODE && (rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0)) {
    // Update for UnitDelay: '<S124>/Unit Delay'
    localDW->UnitDelay_DSTATE[0] = localB->Sum[0];
    localDW->UnitDelay_DSTATE[1] = localB->Sum[1];
  }

  // End of Update for SubSystem: '<S109>/Hwgw(z)'
}

//
// Output and update for action system:
//    '<S113>/Low altitude  rates'
//    '<S152>/Low altitude  rates'
//
void MulticopterModelClass::QuadModelv_Lowaltituderates(const real_T rtu_DCM[9],
  const real_T rtu_pgw_hl[2], const real_T rtu_qgw_hl[2], const real_T
  rtu_rgw_hl[2], real_T rtu_Winddirection, real_T rty_pgwqgwrgw[3])
{
  real_T rtb_TrigonometricFunction1_o1;
  real_T rtb_TrigonometricFunction1_o2;
  real_T rtb_VectorConcatenate_idx_0;
  real_T rtb_VectorConcatenate_idx_2;

  // SignalConversion generated from: '<S133>/Vector Concatenate'
  rtb_VectorConcatenate_idx_2 = rtu_rgw_hl[0];

  // Trigonometry: '<S134>/Trigonometric Function1'
  rtb_TrigonometricFunction1_o1 = std::sin(rtu_Winddirection);
  rtb_TrigonometricFunction1_o2 = std::cos(rtu_Winddirection);

  // Sum: '<S134>/Sum' incorporates:
  //   Product: '<S134>/Product1'
  //   Product: '<S134>/Product2'

  rtb_VectorConcatenate_idx_0 = rtu_pgw_hl[0] * rtb_TrigonometricFunction1_o2 -
    rtb_TrigonometricFunction1_o1 * rtu_qgw_hl[0];

  // Sum: '<S134>/Sum1' incorporates:
  //   Product: '<S134>/Product1'
  //   Product: '<S134>/Product2'

  rtb_TrigonometricFunction1_o1 = rtb_TrigonometricFunction1_o1 * rtu_pgw_hl[0]
    + rtu_qgw_hl[0] * rtb_TrigonometricFunction1_o2;

  // Reshape: '<S133>/Reshape1' incorporates:
  //   Concatenate: '<S133>/Vector Concatenate'
  //   Product: '<S133>/Product'

  for (int32_T i = 0; i < 3; i++) {
    rty_pgwqgwrgw[i] = 0.0;
    rty_pgwqgwrgw[i] += rtu_DCM[i] * rtb_VectorConcatenate_idx_0;
    rty_pgwqgwrgw[i] += rtu_DCM[i + 3] * rtb_TrigonometricFunction1_o1;
    rty_pgwqgwrgw[i] += rtu_DCM[i + 6] * rtb_VectorConcatenate_idx_2;
  }

  // End of Reshape: '<S133>/Reshape1'
}

//
// Output and update for action system:
//    '<S113>/Interpolate  rates'
//    '<S152>/Interpolate  rates'
//
void MulticopterModelClass::QuadModelv_Interpolaterates(const real_T rtu_pgw_hl
  [2], const real_T rtu_qgw_hl[2], const real_T rtu_rgw_hl[2], const real_T
  rtu_DCM[9], real_T rtu_Winddirection, real_T rtu_Altitude, real_T
  rty_pgwqgwrgw[3], P_Interpolaterates_QuadModelv_T *localP)
{
  real_T rtb_Product_fi[3];
  real_T rtb_TrigonometricFunction_o1;
  real_T rtb_TrigonometricFunction_o2;
  real_T rtb_VectorConcatenate_p_idx_0;

  // Trigonometry: '<S132>/Trigonometric Function'
  rtb_TrigonometricFunction_o1 = std::sin(rtu_Winddirection);
  rtb_TrigonometricFunction_o2 = std::cos(rtu_Winddirection);

  // Sum: '<S132>/Sum' incorporates:
  //   Product: '<S132>/Product1'
  //   Product: '<S132>/Product2'

  rtb_VectorConcatenate_p_idx_0 = rtu_pgw_hl[0] * rtb_TrigonometricFunction_o2 -
    rtb_TrigonometricFunction_o1 * rtu_qgw_hl[0];

  // Sum: '<S132>/Sum1' incorporates:
  //   Product: '<S132>/Product1'
  //   Product: '<S132>/Product2'

  rtb_TrigonometricFunction_o1 = rtb_TrigonometricFunction_o1 * rtu_pgw_hl[0] +
    rtu_qgw_hl[0] * rtb_TrigonometricFunction_o2;

  // SignalConversion generated from: '<S131>/Vector Concatenate'
  rtb_TrigonometricFunction_o2 = rtu_rgw_hl[0];

  // Product: '<S131>/Product' incorporates:
  //   Concatenate: '<S131>/Vector Concatenate'

  for (int32_T i = 0; i < 3; i++) {
    rtb_Product_fi[i] = (rtu_DCM[i + 3] * rtb_TrigonometricFunction_o1 +
                         rtu_DCM[i] * rtb_VectorConcatenate_p_idx_0) + rtu_DCM[i
      + 6] * rtb_TrigonometricFunction_o2;
  }

  // End of Product: '<S131>/Product'

  // Sum: '<S127>/Sum1' incorporates:
  //   Constant: '<S127>/max_height_low'

  rtb_TrigonometricFunction_o1 = rtu_Altitude - localP->max_height_low_Value;

  // Sum: '<S127>/Sum' incorporates:
  //   Constant: '<S127>/max_height_low'
  //   Constant: '<S127>/min_height_high'

  rtb_TrigonometricFunction_o2 = localP->min_height_high_Value -
    localP->max_height_low_Value;

  // Sum: '<S127>/Sum3' incorporates:
  //   Product: '<S127>/Product1'
  //   Sum: '<S127>/Sum2'

  rty_pgwqgwrgw[0] = (rtu_pgw_hl[1] - rtb_Product_fi[0]) *
    rtb_TrigonometricFunction_o1 / rtb_TrigonometricFunction_o2 +
    rtb_Product_fi[0];
  rty_pgwqgwrgw[1] = (rtu_qgw_hl[1] - rtb_Product_fi[1]) *
    rtb_TrigonometricFunction_o1 / rtb_TrigonometricFunction_o2 +
    rtb_Product_fi[1];
  rty_pgwqgwrgw[2] = (rtu_rgw_hl[1] - rtb_Product_fi[2]) *
    rtb_TrigonometricFunction_o1 / rtb_TrigonometricFunction_o2 +
    rtb_Product_fi[2];
}

//
// Output and update for action system:
//    '<S114>/Low altitude  velocities'
//    '<S153>/Low altitude  velocities'
//
void MulticopterModelClass::QuadModel_Lowaltitudevelocities(const real_T
  rtu_DCM[9], const real_T rtu_ugw_hl[2], const real_T rtu_vgw_hl[2], const
  real_T rtu_wgw_hl[2], real_T rtu_Winddirection, real_T rty_ugwvgwwgw[3])
{
  real_T rtb_TrigonometricFunction_o1;
  real_T rtb_TrigonometricFunction_o2;
  real_T rtb_VectorConcatenate_i_idx_0;
  real_T rtb_VectorConcatenate_i_idx_2;

  // SignalConversion generated from: '<S141>/Vector Concatenate'
  rtb_VectorConcatenate_i_idx_2 = rtu_wgw_hl[0];

  // Trigonometry: '<S142>/Trigonometric Function'
  rtb_TrigonometricFunction_o1 = std::sin(rtu_Winddirection);
  rtb_TrigonometricFunction_o2 = std::cos(rtu_Winddirection);

  // Sum: '<S142>/Sum' incorporates:
  //   Product: '<S142>/Product1'
  //   Product: '<S142>/Product2'

  rtb_VectorConcatenate_i_idx_0 = rtu_ugw_hl[0] * rtb_TrigonometricFunction_o2 -
    rtb_TrigonometricFunction_o1 * rtu_vgw_hl[0];

  // Sum: '<S142>/Sum1' incorporates:
  //   Product: '<S142>/Product1'
  //   Product: '<S142>/Product2'

  rtb_TrigonometricFunction_o1 = rtb_TrigonometricFunction_o1 * rtu_ugw_hl[0] +
    rtu_vgw_hl[0] * rtb_TrigonometricFunction_o2;

  // Reshape: '<S141>/Reshape1' incorporates:
  //   Concatenate: '<S141>/Vector Concatenate'
  //   Product: '<S141>/Product'

  for (int32_T i = 0; i < 3; i++) {
    rty_ugwvgwwgw[i] = 0.0;
    rty_ugwvgwwgw[i] += rtu_DCM[i] * rtb_VectorConcatenate_i_idx_0;
    rty_ugwvgwwgw[i] += rtu_DCM[i + 3] * rtb_TrigonometricFunction_o1;
    rty_ugwvgwwgw[i] += rtu_DCM[i + 6] * rtb_VectorConcatenate_i_idx_2;
  }

  // End of Reshape: '<S141>/Reshape1'
}

//
// Output and update for action system:
//    '<S114>/Interpolate  velocities'
//    '<S153>/Interpolate  velocities'
//
void MulticopterModelClass::QuadModel_Interpolatevelocities(const real_T
  rtu_ugw_hl[2], const real_T rtu_vgw_hl[2], const real_T rtu_wgw_hl[2], const
  real_T rtu_DCM[9], real_T rtu_Winddirection, real_T rtu_Altitude, real_T
  rty_ugwvgwwgw[3], P_Interpolatevelocities_QuadM_T *localP)
{
  real_T rtb_Product_f2[3];
  real_T rtb_TrigonometricFunction_o1;
  real_T rtb_TrigonometricFunction_o2;
  real_T rtb_VectorConcatenate_f_idx_0;

  // Trigonometry: '<S140>/Trigonometric Function'
  rtb_TrigonometricFunction_o1 = std::sin(rtu_Winddirection);
  rtb_TrigonometricFunction_o2 = std::cos(rtu_Winddirection);

  // Sum: '<S140>/Sum' incorporates:
  //   Product: '<S140>/Product1'
  //   Product: '<S140>/Product2'

  rtb_VectorConcatenate_f_idx_0 = rtu_ugw_hl[0] * rtb_TrigonometricFunction_o2 -
    rtb_TrigonometricFunction_o1 * rtu_vgw_hl[0];

  // Sum: '<S140>/Sum1' incorporates:
  //   Product: '<S140>/Product1'
  //   Product: '<S140>/Product2'

  rtb_TrigonometricFunction_o1 = rtb_TrigonometricFunction_o1 * rtu_ugw_hl[0] +
    rtu_vgw_hl[0] * rtb_TrigonometricFunction_o2;

  // SignalConversion generated from: '<S139>/Vector Concatenate'
  rtb_TrigonometricFunction_o2 = rtu_wgw_hl[0];

  // Product: '<S139>/Product' incorporates:
  //   Concatenate: '<S139>/Vector Concatenate'

  for (int32_T i = 0; i < 3; i++) {
    rtb_Product_f2[i] = (rtu_DCM[i + 3] * rtb_TrigonometricFunction_o1 +
                         rtu_DCM[i] * rtb_VectorConcatenate_f_idx_0) + rtu_DCM[i
      + 6] * rtb_TrigonometricFunction_o2;
  }

  // End of Product: '<S139>/Product'

  // Sum: '<S135>/Sum1' incorporates:
  //   Constant: '<S135>/max_height_low'

  rtb_TrigonometricFunction_o1 = rtu_Altitude - localP->max_height_low_Value;

  // Sum: '<S135>/Sum' incorporates:
  //   Constant: '<S135>/max_height_low'
  //   Constant: '<S135>/min_height_high'

  rtb_TrigonometricFunction_o2 = localP->min_height_high_Value -
    localP->max_height_low_Value;

  // Sum: '<S135>/Sum3' incorporates:
  //   Product: '<S135>/Product1'
  //   Sum: '<S135>/Sum2'

  rty_ugwvgwwgw[0] = (rtu_ugw_hl[1] - rtb_Product_f2[0]) *
    rtb_TrigonometricFunction_o1 / rtb_TrigonometricFunction_o2 +
    rtb_Product_f2[0];
  rty_ugwvgwwgw[1] = (rtu_vgw_hl[1] - rtb_Product_f2[1]) *
    rtb_TrigonometricFunction_o1 / rtb_TrigonometricFunction_o2 +
    rtb_Product_f2[1];
  rty_ugwvgwwgw[2] = (rtu_wgw_hl[1] - rtb_Product_f2[2]) *
    rtb_TrigonometricFunction_o1 / rtb_TrigonometricFunction_o2 +
    rtb_Product_f2[2];
}

//
// Output and update for action system:
//    '<S193>/Positive Trace'
//    '<S194>/Positive Trace'
//    '<S329>/Positive Trace'
//
void MulticopterModelClass::QuadModelv_PositiveTrace(real_T rtu_traceDCM, const
  real_T rtu_DCM[9], real_T *rty_qwqxqyqz, real_T rty_qwqxqyqz_a[3],
  P_PositiveTrace_QuadModelv_T *localP)
{
  real_T rtb_Gain1_o;

  // Sqrt: '<S202>/sqrt' incorporates:
  //   Constant: '<S202>/Constant'
  //   Sum: '<S202>/Sum'

  rtb_Gain1_o = std::sqrt(rtu_traceDCM + localP->Constant_Value);

  // Gain: '<S202>/Gain'
  *rty_qwqxqyqz = localP->Gain_Gain * rtb_Gain1_o;

  // Gain: '<S202>/Gain1'
  rtb_Gain1_o *= localP->Gain1_Gain;

  // Product: '<S202>/Product' incorporates:
  //   Sum: '<S224>/Add'
  //   Sum: '<S225>/Add'
  //   Sum: '<S226>/Add'

  rty_qwqxqyqz_a[0] = (rtu_DCM[7] - rtu_DCM[5]) / rtb_Gain1_o;
  rty_qwqxqyqz_a[1] = (rtu_DCM[2] - rtu_DCM[6]) / rtb_Gain1_o;
  rty_qwqxqyqz_a[2] = (rtu_DCM[3] - rtu_DCM[1]) / rtb_Gain1_o;
}

//
// System initialize for action system:
//    '<S193>/Negative Trace'
//    '<S194>/Negative Trace'
//    '<S329>/Negative Trace'
//
void MulticopterModelClass::QuadModelv_NegativeTrace_Init
  (DW_NegativeTrace_QuadModelv_T *localDW)
{
  // Start for If: '<S201>/Find Maximum Diagonal Value'
  localDW->FindMaximumDiagonalValue_Active = -1;
}

//
// Output and update for action system:
//    '<S193>/Negative Trace'
//    '<S194>/Negative Trace'
//    '<S329>/Negative Trace'
//
void MulticopterModelClass::QuadModelv_NegativeTrace(const real_T rtu_DCM[9],
  real_T rty_qwqxqyqz[4], DW_NegativeTrace_QuadModelv_T *localDW,
  P_NegativeTrace_QuadModelv_T *localP)
{
  int8_T rtAction;

  // If: '<S201>/Find Maximum Diagonal Value'
  if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if ((rtu_DCM[4] > rtu_DCM[0]) && (rtu_DCM[4] > rtu_DCM[8])) {
      rtAction = 0;
    } else if (rtu_DCM[8] > rtu_DCM[0]) {
      rtAction = 1;
    } else {
      rtAction = 2;
    }

    localDW->FindMaximumDiagonalValue_Active = rtAction;
  } else {
    rtAction = localDW->FindMaximumDiagonalValue_Active;
  }

  switch (rtAction) {
   case 0:
    {
      real_T rtb_Product_dje;
      real_T rtb_Switch_c_idx_0;

      // Outputs for IfAction SubSystem: '<S201>/Maximum Value at DCM(2,2)' incorporates:
      //   ActionPort: '<S206>/Action Port'

      // Sqrt: '<S206>/sqrt' incorporates:
      //   Constant: '<S218>/Constant'
      //   Sum: '<S218>/Add'

      rtb_Product_dje = std::sqrt(((rtu_DCM[4] - rtu_DCM[0]) - rtu_DCM[8]) +
        localP->Constant_Value);

      // Gain: '<S206>/Gain'
      rty_qwqxqyqz[2] = localP->Gain_Gain * rtb_Product_dje;

      // Switch: '<S217>/Switch' incorporates:
      //   Constant: '<S217>/Constant1'
      //   Constant: '<S217>/Constant2'

      if (rtb_Product_dje != 0.0) {
        rtb_Switch_c_idx_0 = localP->Constant1_Value;
      } else {
        rtb_Switch_c_idx_0 = localP->Constant2_Value[0];
        rtb_Product_dje = localP->Constant2_Value[1];
      }

      // End of Switch: '<S217>/Switch'

      // Product: '<S217>/Product'
      rtb_Product_dje = rtb_Switch_c_idx_0 / rtb_Product_dje;

      // Gain: '<S206>/Gain1' incorporates:
      //   Product: '<S206>/Product'
      //   Sum: '<S216>/Add'

      rty_qwqxqyqz[1] = (rtu_DCM[1] + rtu_DCM[3]) * rtb_Product_dje *
        localP->Gain1_Gain;

      // Gain: '<S206>/Gain3' incorporates:
      //   Product: '<S206>/Product'
      //   Sum: '<S215>/Add'

      rty_qwqxqyqz[3] = (rtu_DCM[5] + rtu_DCM[7]) * rtb_Product_dje *
        localP->Gain3_Gain;

      // Gain: '<S206>/Gain4' incorporates:
      //   Product: '<S206>/Product'
      //   Sum: '<S214>/Add'

      rty_qwqxqyqz[0] = (rtu_DCM[2] - rtu_DCM[6]) * rtb_Product_dje *
        localP->Gain4_Gain;

      // End of Outputs for SubSystem: '<S201>/Maximum Value at DCM(2,2)'
    }
    break;

   case 1:
    {
      real_T rtb_Product_dje;
      real_T rtb_Switch_c_idx_0;

      // Outputs for IfAction SubSystem: '<S201>/Maximum Value at DCM(3,3)' incorporates:
      //   ActionPort: '<S207>/Action Port'

      // Sqrt: '<S207>/sqrt' incorporates:
      //   Constant: '<S223>/Constant'
      //   Sum: '<S223>/Add'

      rtb_Product_dje = std::sqrt(((rtu_DCM[8] - rtu_DCM[0]) - rtu_DCM[4]) +
        localP->Constant_Value_n);

      // Gain: '<S207>/Gain'
      rty_qwqxqyqz[3] = localP->Gain_Gain_c * rtb_Product_dje;

      // Switch: '<S222>/Switch' incorporates:
      //   Constant: '<S222>/Constant1'
      //   Constant: '<S222>/Constant2'

      if (rtb_Product_dje != 0.0) {
        rtb_Switch_c_idx_0 = localP->Constant1_Value_a;
      } else {
        rtb_Switch_c_idx_0 = localP->Constant2_Value_c[0];
        rtb_Product_dje = localP->Constant2_Value_c[1];
      }

      // End of Switch: '<S222>/Switch'

      // Product: '<S222>/Product'
      rtb_Product_dje = rtb_Switch_c_idx_0 / rtb_Product_dje;

      // Gain: '<S207>/Gain1' incorporates:
      //   Product: '<S207>/Product'
      //   Sum: '<S219>/Add'

      rty_qwqxqyqz[1] = (rtu_DCM[2] + rtu_DCM[6]) * rtb_Product_dje *
        localP->Gain1_Gain_n;

      // Gain: '<S207>/Gain2' incorporates:
      //   Product: '<S207>/Product'
      //   Sum: '<S220>/Add'

      rty_qwqxqyqz[2] = (rtu_DCM[5] + rtu_DCM[7]) * rtb_Product_dje *
        localP->Gain2_Gain;

      // Gain: '<S207>/Gain3' incorporates:
      //   Product: '<S207>/Product'
      //   Sum: '<S221>/Add'

      rty_qwqxqyqz[0] = (rtu_DCM[3] - rtu_DCM[1]) * rtb_Product_dje *
        localP->Gain3_Gain_m;

      // End of Outputs for SubSystem: '<S201>/Maximum Value at DCM(3,3)'
    }
    break;

   case 2:
    {
      real_T rtb_Product_dje;
      real_T rtb_Switch_c_idx_0;

      // Outputs for IfAction SubSystem: '<S201>/Maximum Value at DCM(1,1)' incorporates:
      //   ActionPort: '<S205>/Action Port'

      // Sqrt: '<S205>/sqrt' incorporates:
      //   Constant: '<S213>/Constant'
      //   Sum: '<S213>/Add'

      rtb_Product_dje = std::sqrt(((rtu_DCM[0] - rtu_DCM[4]) - rtu_DCM[8]) +
        localP->Constant_Value_m);

      // Gain: '<S205>/Gain'
      rty_qwqxqyqz[1] = localP->Gain_Gain_l * rtb_Product_dje;

      // Switch: '<S212>/Switch' incorporates:
      //   Constant: '<S212>/Constant1'
      //   Constant: '<S212>/Constant2'

      if (rtb_Product_dje != 0.0) {
        rtb_Switch_c_idx_0 = localP->Constant1_Value_i;
      } else {
        rtb_Switch_c_idx_0 = localP->Constant2_Value_b[0];
        rtb_Product_dje = localP->Constant2_Value_b[1];
      }

      // End of Switch: '<S212>/Switch'

      // Product: '<S212>/Product'
      rtb_Product_dje = rtb_Switch_c_idx_0 / rtb_Product_dje;

      // Gain: '<S205>/Gain1' incorporates:
      //   Product: '<S205>/Product'
      //   Sum: '<S211>/Add'

      rty_qwqxqyqz[2] = (rtu_DCM[1] + rtu_DCM[3]) * rtb_Product_dje *
        localP->Gain1_Gain_b;

      // Gain: '<S205>/Gain2' incorporates:
      //   Product: '<S205>/Product'
      //   Sum: '<S209>/Add'

      rty_qwqxqyqz[3] = (rtu_DCM[2] + rtu_DCM[6]) * rtb_Product_dje *
        localP->Gain2_Gain_d;

      // Gain: '<S205>/Gain3' incorporates:
      //   Product: '<S205>/Product'
      //   Sum: '<S210>/Add'

      rty_qwqxqyqz[0] = (rtu_DCM[7] - rtu_DCM[5]) * rtb_Product_dje *
        localP->Gain3_Gain_n;

      // End of Outputs for SubSystem: '<S201>/Maximum Value at DCM(1,1)'
    }
    break;
  }

  // End of If: '<S201>/Find Maximum Diagonal Value'
}

//
// Output and update for action system:
//    '<S203>/If Warning//Error'
//    '<S241>/If Warning//Error'
//    '<S337>/If Warning//Error'
//
void MulticopterModelClass::QuadModelv_IfWarningError(const real_T rtu_dcm[9],
  real_T rtp_action, real_T rtp_tolerance, P_IfWarningError_QuadModelv_T *localP)
{
  real_T rtu_dcm_0[9];
  boolean_T rtb_Compare_k[9];
  boolean_T tmp;

  // Bias: '<S230>/Bias1' incorporates:
  //   Math: '<S230>/Math Function'
  //   Product: '<S230>/Product'

  for (int32_T i = 0; i < 3; i++) {
    for (int32_T i_0 = 0; i_0 < 3; i_0++) {
      int32_T rtu_dcm_tmp;
      rtu_dcm_tmp = 3 * i_0 + i;
      rtu_dcm_0[rtu_dcm_tmp] = ((rtu_dcm[3 * i + 1] * rtu_dcm[3 * i_0 + 1] +
        rtu_dcm[3 * i] * rtu_dcm[3 * i_0]) + rtu_dcm[3 * i + 2] * rtu_dcm[3 *
        i_0 + 2]) + localP->Bias1_Bias[rtu_dcm_tmp];
    }
  }

  // End of Bias: '<S230>/Bias1'

  // RelationalOperator: '<S236>/Compare' incorporates:
  //   Abs: '<S230>/Abs2'
  //   Constant: '<S236>/Constant'

  for (int32_T i = 0; i < 9; i++) {
    rtb_Compare_k[i] = (std::abs(rtu_dcm_0[i]) > rtp_tolerance);
  }

  // End of RelationalOperator: '<S236>/Compare'

  // Logic: '<S230>/Logical Operator1' incorporates:
  //   RelationalOperator: '<S236>/Compare'

  tmp = rtb_Compare_k[0];
  for (int32_T i = 0; i < 8; i++) {
    tmp = (tmp || rtb_Compare_k[i + 1]);
  }

  // If: '<S227>/If' incorporates:
  //   Abs: '<S231>/Abs1'
  //   Bias: '<S231>/Bias'
  //   Constant: '<S238>/Constant'
  //   Logic: '<S230>/Logical Operator1'
  //   Product: '<S237>/Product'
  //   Product: '<S237>/Product1'
  //   Product: '<S237>/Product2'
  //   Product: '<S237>/Product3'
  //   Product: '<S237>/Product4'
  //   Product: '<S237>/Product5'
  //   RelationalOperator: '<S238>/Compare'
  //   Reshape: '<S237>/Reshape'
  //   Sum: '<S237>/Sum'

  if (std::abs((((((rtu_dcm[0] * rtu_dcm[4] * rtu_dcm[8] - rtu_dcm[0] * rtu_dcm
                    [5] * rtu_dcm[7]) - rtu_dcm[1] * rtu_dcm[3] * rtu_dcm[8]) +
                  rtu_dcm[2] * rtu_dcm[3] * rtu_dcm[7]) + rtu_dcm[1] * rtu_dcm[5]
                 * rtu_dcm[6]) - rtu_dcm[2] * rtu_dcm[4] * rtu_dcm[6]) +
               localP->Bias_Bias) > rtp_tolerance) {
    // Outputs for IfAction SubSystem: '<S227>/If Not Proper' incorporates:
    //   ActionPort: '<S229>/Action Port'

    // If: '<S229>/If' incorporates:
    //   Constant: '<S229>/Constant'

    if (rtp_action == 2.0) {
      // Outputs for IfAction SubSystem: '<S229>/Warning' incorporates:
      //   ActionPort: '<S235>/Action Port'

      // Assertion: '<S235>/Assertion' incorporates:
      //   Constant: '<S229>/Constant1'

      utAssert(localP->Constant1_Value != 0.0);

      // End of Outputs for SubSystem: '<S229>/Warning'
    } else if (rtp_action == 3.0) {
      // Outputs for IfAction SubSystem: '<S229>/Error' incorporates:
      //   ActionPort: '<S234>/Action Port'

      // Assertion: '<S234>/Assertion' incorporates:
      //   Constant: '<S229>/Constant1'

      utAssert(localP->Constant1_Value != 0.0);

      // End of Outputs for SubSystem: '<S229>/Error'
    }

    // End of If: '<S229>/If'
    // End of Outputs for SubSystem: '<S227>/If Not Proper'
  } else if (tmp) {
    // Outputs for IfAction SubSystem: '<S227>/Else If Not Orthogonal' incorporates:
    //   ActionPort: '<S228>/Action Port'

    // If: '<S228>/If' incorporates:
    //   Constant: '<S228>/Constant'

    if (rtp_action == 2.0) {
      // Outputs for IfAction SubSystem: '<S228>/Warning' incorporates:
      //   ActionPort: '<S233>/Action Port'

      // Assertion: '<S233>/Assertion' incorporates:
      //   Constant: '<S228>/Constant1'

      utAssert(localP->Constant1_Value_k != 0.0);

      // End of Outputs for SubSystem: '<S228>/Warning'
    } else if (rtp_action == 3.0) {
      // Outputs for IfAction SubSystem: '<S228>/Error' incorporates:
      //   ActionPort: '<S232>/Action Port'

      // Assertion: '<S232>/Assertion' incorporates:
      //   Constant: '<S228>/Constant1'

      utAssert(localP->Constant1_Value_k != 0.0);

      // End of Outputs for SubSystem: '<S228>/Error'
    }

    // End of If: '<S228>/If'
    // End of Outputs for SubSystem: '<S227>/Else If Not Orthogonal'
  }

  // End of If: '<S227>/If'
}

//
// System initialize for enable system:
//    '<S298>/Distance into gust (x)'
//    '<S524>/Distance into gust (x)'
//
void MulticopterModelClass::QuadMode_Distanceintogustx_Init
  (B_Distanceintogustx_QuadModel_T *localB, P_Distanceintogustx_QuadModel_T
   *localP, X_Distanceintogustx_QuadModel_T *localX)
{
  // InitializeConditions for Integrator: '<S301>/Distance into Gust (x) (Limited to gust length d)' 
  localX->DistanceintoGustxLimitedtogus_p =
    localP->DistanceintoGustxLimitedtogustl;

  // SystemInitialize for Integrator: '<S301>/Distance into Gust (x) (Limited to gust length d)' incorporates:
  //   Outport: '<S301>/x'

  localB->DistanceintoGustxLimitedtogustl = localP->x_Y0;
}

//
// System reset for enable system:
//    '<S298>/Distance into gust (x)'
//    '<S524>/Distance into gust (x)'
//
void MulticopterModelClass::QuadMod_Distanceintogustx_Reset
  (P_Distanceintogustx_QuadModel_T *localP, X_Distanceintogustx_QuadModel_T
   *localX)
{
  // InitializeConditions for Integrator: '<S301>/Distance into Gust (x) (Limited to gust length d)' 
  localX->DistanceintoGustxLimitedtogus_p =
    localP->DistanceintoGustxLimitedtogustl;
}

//
// Disable for enable system:
//    '<S298>/Distance into gust (x)'
//    '<S524>/Distance into gust (x)'
//
void MulticopterModelClass::QuadM_Distanceintogustx_Disable
  (DW_Distanceintogustx_QuadMode_T *localDW)
{
  localDW->Distanceintogustx_MODE = false;
}

//
// Outputs for enable system:
//    '<S298>/Distance into gust (x)'
//    '<S524>/Distance into gust (x)'
//
void MulticopterModelClass::QuadModelv_Distanceintogustx(boolean_T rtu_Enable,
  real_T rtp_d_m, B_Distanceintogustx_QuadModel_T *localB,
  DW_Distanceintogustx_QuadMode_T *localDW, P_Distanceintogustx_QuadModel_T
  *localP, X_Distanceintogustx_QuadModel_T *localX)
{
  // Outputs for Enabled SubSystem: '<S298>/Distance into gust (x)' incorporates:
  //   EnablePort: '<S301>/Enable'

  if ((rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) &&
      rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if (rtu_Enable) {
      if (!localDW->Distanceintogustx_MODE) {
        QuadMod_Distanceintogustx_Reset(localP, localX);
        localDW->Distanceintogustx_MODE = true;
      }
    } else if (localDW->Distanceintogustx_MODE) {
      QuadM_Distanceintogustx_Disable(localDW);
    }
  }

  if (localDW->Distanceintogustx_MODE) {
    // Integrator: '<S301>/Distance into Gust (x) (Limited to gust length d)'
    // Limited  Integrator
    if (localX->DistanceintoGustxLimitedtogus_p >= rtp_d_m) {
      localX->DistanceintoGustxLimitedtogus_p = rtp_d_m;
    } else if (localX->DistanceintoGustxLimitedtogus_p <=
               localP->DistanceintoGustxLimitedtogus_d) {
      localX->DistanceintoGustxLimitedtogus_p =
        localP->DistanceintoGustxLimitedtogus_d;
    }

    // Integrator: '<S301>/Distance into Gust (x) (Limited to gust length d)'
    localB->DistanceintoGustxLimitedtogustl =
      localX->DistanceintoGustxLimitedtogus_p;
  }

  // End of Outputs for SubSystem: '<S298>/Distance into gust (x)'
}

//
// Derivatives for enable system:
//    '<S298>/Distance into gust (x)'
//    '<S524>/Distance into gust (x)'
//
void MulticopterModelClass::QuadMod_Distanceintogustx_Deriv(real_T rtu_V, real_T
  rtp_d_m, DW_Distanceintogustx_QuadMode_T *localDW,
  P_Distanceintogustx_QuadModel_T *localP, X_Distanceintogustx_QuadModel_T
  *localX, XDot_Distanceintogustx_QuadMo_T *localXdot)
{
  if (localDW->Distanceintogustx_MODE) {
    boolean_T lsat;
    boolean_T usat;

    // Derivatives for Integrator: '<S301>/Distance into Gust (x) (Limited to gust length d)' 
    lsat = (localX->DistanceintoGustxLimitedtogus_p <=
            localP->DistanceintoGustxLimitedtogus_d);
    usat = (localX->DistanceintoGustxLimitedtogus_p >= rtp_d_m);
    if (((!lsat) && (!usat)) || (lsat && (rtu_V > 0.0)) || (usat && (rtu_V < 0.0)))
    {
      localXdot->DistanceintoGustxLimitedtogus_p = rtu_V;
    } else {
      // in saturation
      localXdot->DistanceintoGustxLimitedtogus_p = 0.0;
    }

    // End of Derivatives for Integrator: '<S301>/Distance into Gust (x) (Limited to gust length d)' 
  } else {
    localXdot->DistanceintoGustxLimitedtogus_p = 0.0;
  }
}

//
// System initialize for enable system:
//    '<S298>/Distance into gust (y)'
//    '<S298>/Distance into gust (z)'
//    '<S524>/Distance into gust (y)'
//    '<S524>/Distance into gust (z)'
//
void MulticopterModelClass::QuadMode_Distanceintogusty_Init
  (B_Distanceintogusty_QuadModel_T *localB, P_Distanceintogusty_QuadModel_T
   *localP, X_Distanceintogusty_QuadModel_T *localX)
{
  // InitializeConditions for Integrator: '<S302>/Distance into Gust (x) (Limited to gust length d) ' 
  localX->DistanceintoGustxLimitedtogustl =
    localP->DistanceintoGustxLimitedtogustl;

  // SystemInitialize for Integrator: '<S302>/Distance into Gust (x) (Limited to gust length d) ' incorporates:
  //   Outport: '<S302>/x'

  localB->DistanceintoGustxLimitedtogustl = localP->x_Y0;
}

//
// System reset for enable system:
//    '<S298>/Distance into gust (y)'
//    '<S298>/Distance into gust (z)'
//    '<S524>/Distance into gust (y)'
//    '<S524>/Distance into gust (z)'
//
void MulticopterModelClass::QuadMod_Distanceintogusty_Reset
  (P_Distanceintogusty_QuadModel_T *localP, X_Distanceintogusty_QuadModel_T
   *localX)
{
  // InitializeConditions for Integrator: '<S302>/Distance into Gust (x) (Limited to gust length d) ' 
  localX->DistanceintoGustxLimitedtogustl =
    localP->DistanceintoGustxLimitedtogustl;
}

//
// Disable for enable system:
//    '<S298>/Distance into gust (y)'
//    '<S298>/Distance into gust (z)'
//    '<S524>/Distance into gust (y)'
//    '<S524>/Distance into gust (z)'
//
void MulticopterModelClass::QuadM_Distanceintogusty_Disable
  (DW_Distanceintogusty_QuadMode_T *localDW)
{
  localDW->Distanceintogusty_MODE = false;
}

//
// Outputs for enable system:
//    '<S298>/Distance into gust (y)'
//    '<S298>/Distance into gust (z)'
//    '<S524>/Distance into gust (y)'
//    '<S524>/Distance into gust (z)'
//
void MulticopterModelClass::QuadModelv_Distanceintogusty(boolean_T rtu_Enable,
  real_T rtp_d_m, B_Distanceintogusty_QuadModel_T *localB,
  DW_Distanceintogusty_QuadMode_T *localDW, P_Distanceintogusty_QuadModel_T
  *localP, X_Distanceintogusty_QuadModel_T *localX)
{
  // Outputs for Enabled SubSystem: '<S298>/Distance into gust (y)' incorporates:
  //   EnablePort: '<S302>/Enable'

  if ((rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) &&
      rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if (rtu_Enable) {
      if (!localDW->Distanceintogusty_MODE) {
        QuadMod_Distanceintogusty_Reset(localP, localX);
        localDW->Distanceintogusty_MODE = true;
      }
    } else if (localDW->Distanceintogusty_MODE) {
      QuadM_Distanceintogusty_Disable(localDW);
    }
  }

  if (localDW->Distanceintogusty_MODE) {
    // Integrator: '<S302>/Distance into Gust (x) (Limited to gust length d) '
    // Limited  Integrator
    if (localX->DistanceintoGustxLimitedtogustl >= rtp_d_m) {
      localX->DistanceintoGustxLimitedtogustl = rtp_d_m;
    } else if (localX->DistanceintoGustxLimitedtogustl <=
               localP->DistanceintoGustxLimitedtogus_k) {
      localX->DistanceintoGustxLimitedtogustl =
        localP->DistanceintoGustxLimitedtogus_k;
    }

    // Integrator: '<S302>/Distance into Gust (x) (Limited to gust length d) '
    localB->DistanceintoGustxLimitedtogustl =
      localX->DistanceintoGustxLimitedtogustl;
  }

  // End of Outputs for SubSystem: '<S298>/Distance into gust (y)'
}

//
// Derivatives for enable system:
//    '<S298>/Distance into gust (y)'
//    '<S298>/Distance into gust (z)'
//    '<S524>/Distance into gust (y)'
//    '<S524>/Distance into gust (z)'
//
void MulticopterModelClass::QuadMod_Distanceintogusty_Deriv(real_T rtu_V, real_T
  rtp_d_m, DW_Distanceintogusty_QuadMode_T *localDW,
  P_Distanceintogusty_QuadModel_T *localP, X_Distanceintogusty_QuadModel_T
  *localX, XDot_Distanceintogusty_QuadMo_T *localXdot)
{
  if (localDW->Distanceintogusty_MODE) {
    boolean_T lsat;
    boolean_T usat;

    // Derivatives for Integrator: '<S302>/Distance into Gust (x) (Limited to gust length d) ' 
    lsat = (localX->DistanceintoGustxLimitedtogustl <=
            localP->DistanceintoGustxLimitedtogus_k);
    usat = (localX->DistanceintoGustxLimitedtogustl >= rtp_d_m);
    if (((!lsat) && (!usat)) || (lsat && (rtu_V > 0.0)) || (usat && (rtu_V < 0.0)))
    {
      localXdot->DistanceintoGustxLimitedtogustl = rtu_V;
    } else {
      // in saturation
      localXdot->DistanceintoGustxLimitedtogustl = 0.0;
    }

    // End of Derivatives for Integrator: '<S302>/Distance into Gust (x) (Limited to gust length d) ' 
  } else {
    localXdot->DistanceintoGustxLimitedtogustl = 0.0;
  }
}

//
// System initialize for atomic system:
//    '<S305>/Motor_Dynamics'
//    '<S306>/Motor_Dynamics'
//    '<S307>/Motor_Dynamics'
//    '<S308>/Motor_Dynamics'
//    '<S309>/Motor_Dynamics'
//    '<S310>/Motor_Dynamics'
//    '<S311>/Motor_Dynamics'
//    '<S312>/Motor_Dynamics'
//
void MulticopterModelClass::QuadModelv_Motor_Dynamics_Init
  (DW_Motor_Dynamics_QuadModelv_T *localDW, X_Motor_Dynamics_QuadModelv_T
   *localX) const
{
  // InitializeConditions for Integrator: '<S314>/Integrator'
  if (rtmIsFirstInitCond((&QuadModelv_M))) {
    localX->Integrator_CSTATE = 0.0;
  }

  localDW->Integrator_IWORK = 1;

  // End of InitializeConditions for Integrator: '<S314>/Integrator'
}

//
// Outputs for atomic system:
//    '<S305>/Motor_Dynamics'
//    '<S306>/Motor_Dynamics'
//    '<S307>/Motor_Dynamics'
//    '<S308>/Motor_Dynamics'
//    '<S309>/Motor_Dynamics'
//    '<S310>/Motor_Dynamics'
//    '<S311>/Motor_Dynamics'
//    '<S312>/Motor_Dynamics'
//
void MulticopterModelClass::QuadModelv_Motor_Dynamics(real_T rtu_motor_rate_d,
  real_T rtu_ModelInit_RPM, real_T rtu_motorT, B_Motor_Dynamics_QuadModelv_T
  *localB, DW_Motor_Dynamics_QuadModelv_T *localDW,
  X_Motor_Dynamics_QuadModelv_T *localX)
{
  // Integrator: '<S314>/Integrator'
  if (localDW->Integrator_IWORK != 0) {
    localX->Integrator_CSTATE = rtu_ModelInit_RPM;
  }

  // Integrator: '<S314>/Integrator'
  localB->x = localX->Integrator_CSTATE;

  // Product: '<S314>/Divide' incorporates:
  //   Sum: '<S314>/Add'

  localB->dratedt = (rtu_motor_rate_d - localB->x) / rtu_motorT;
}

//
// Update for atomic system:
//    '<S305>/Motor_Dynamics'
//    '<S306>/Motor_Dynamics'
//    '<S307>/Motor_Dynamics'
//    '<S308>/Motor_Dynamics'
//    '<S309>/Motor_Dynamics'
//    '<S310>/Motor_Dynamics'
//    '<S311>/Motor_Dynamics'
//    '<S312>/Motor_Dynamics'
//
void MulticopterModelClass::QuadModel_Motor_Dynamics_Update
  (DW_Motor_Dynamics_QuadModelv_T *localDW)
{
  // Update for Integrator: '<S314>/Integrator'
  localDW->Integrator_IWORK = 0;
}

//
// Derivatives for atomic system:
//    '<S305>/Motor_Dynamics'
//    '<S306>/Motor_Dynamics'
//    '<S307>/Motor_Dynamics'
//    '<S308>/Motor_Dynamics'
//    '<S309>/Motor_Dynamics'
//    '<S310>/Motor_Dynamics'
//    '<S311>/Motor_Dynamics'
//    '<S312>/Motor_Dynamics'
//
void MulticopterModelClass::QuadModelv_Motor_Dynamics_Deriv
  (B_Motor_Dynamics_QuadModelv_T *localB, XDot_Motor_Dynamics_QuadModel_T
   *localXdot)
{
  // Derivatives for Integrator: '<S314>/Integrator'
  localXdot->Integrator_CSTATE = localB->dratedt;
}

//
// System initialize for atomic system:
//    '<S296>/MotorNonlinearDynamic1'
//    '<S296>/MotorNonlinearDynamic2'
//    '<S296>/MotorNonlinearDynamic3'
//    '<S296>/MotorNonlinearDynamic4'
//    '<S296>/MotorNonlinearDynamic5'
//    '<S296>/MotorNonlinearDynamic6'
//    '<S296>/MotorNonlinearDynamic7'
//    '<S296>/MotorNonlinearDynamic8'
//
void MulticopterModelClass::Qua_MotorNonlinearDynamic1_Init
  (DW_MotorNonlinearDynamic1_Qua_T *localDW, X_MotorNonlinearDynamic1_Quad_T
   *localX) const
{
  // SystemInitialize for Atomic SubSystem: '<S305>/Motor_Dynamics'
  QuadModelv_Motor_Dynamics_Init(&localDW->Motor_Dynamics,
    &localX->Motor_Dynamics);

  // End of SystemInitialize for SubSystem: '<S305>/Motor_Dynamics'
}

//
// Outputs for atomic system:
//    '<S296>/MotorNonlinearDynamic1'
//    '<S296>/MotorNonlinearDynamic2'
//    '<S296>/MotorNonlinearDynamic3'
//    '<S296>/MotorNonlinearDynamic4'
//    '<S296>/MotorNonlinearDynamic5'
//    '<S296>/MotorNonlinearDynamic6'
//    '<S296>/MotorNonlinearDynamic7'
//    '<S296>/MotorNonlinearDynamic8'
//
void MulticopterModelClass::QuadMode_MotorNonlinearDynamic1(real_T rtu_Wb,
  real_T rtu_Cr, real_T rtu_motorT, real_T rtu_ModelInit_RPM, real_T rtu_PWM,
  B_MotorNonlinearDynamic1_Quad_T *localB, DW_MotorNonlinearDynamic1_Qua_T
  *localDW, P_MotorNonlinearDynamic1_Quad_T *localP, P_QuadModelv_T
  *QuadModelv_P, X_MotorNonlinearDynamic1_Quad_T *localX)
{
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    real_T rtb_Sum1_i;

    // Saturate: '<S305>/Signal_Saturation'
    if (rtu_PWM > localP->Signal_Saturation_UpperSat) {
      rtb_Sum1_i = localP->Signal_Saturation_UpperSat;
    } else if (rtu_PWM < localP->Signal_Saturation_LowerSat) {
      rtb_Sum1_i = localP->Signal_Saturation_LowerSat;
    } else {
      rtb_Sum1_i = rtu_PWM;
    }

    // End of Saturate: '<S305>/Signal_Saturation'

    // Product: '<S305>/Product' incorporates:
    //   Constant: '<S313>/Constant'
    //   Product: '<S305>/Product1'
    //   RelationalOperator: '<S313>/Compare'
    //   Sum: '<S305>/Sum1'

    localB->Product = (rtu_Cr * rtb_Sum1_i + rtu_Wb) * static_cast<real_T>
      (rtb_Sum1_i >= QuadModelv_P->ModelParam_motorMinThr);
  }

  // Outputs for Atomic SubSystem: '<S305>/Motor_Dynamics'
  QuadModelv_Motor_Dynamics(localB->Product, rtu_ModelInit_RPM, rtu_motorT,
    &localB->Motor_Dynamics, &localDW->Motor_Dynamics, &localX->Motor_Dynamics);

  // End of Outputs for SubSystem: '<S305>/Motor_Dynamics'
}

//
// Update for atomic system:
//    '<S296>/MotorNonlinearDynamic1'
//    '<S296>/MotorNonlinearDynamic2'
//    '<S296>/MotorNonlinearDynamic3'
//    '<S296>/MotorNonlinearDynamic4'
//    '<S296>/MotorNonlinearDynamic5'
//    '<S296>/MotorNonlinearDynamic6'
//    '<S296>/MotorNonlinearDynamic7'
//    '<S296>/MotorNonlinearDynamic8'
//
void MulticopterModelClass::Q_MotorNonlinearDynamic1_Update
  (DW_MotorNonlinearDynamic1_Qua_T *localDW)
{
  // Update for Atomic SubSystem: '<S305>/Motor_Dynamics'
  QuadModel_Motor_Dynamics_Update(&localDW->Motor_Dynamics);

  // End of Update for SubSystem: '<S305>/Motor_Dynamics'
}

//
// Derivatives for atomic system:
//    '<S296>/MotorNonlinearDynamic1'
//    '<S296>/MotorNonlinearDynamic2'
//    '<S296>/MotorNonlinearDynamic3'
//    '<S296>/MotorNonlinearDynamic4'
//    '<S296>/MotorNonlinearDynamic5'
//    '<S296>/MotorNonlinearDynamic6'
//    '<S296>/MotorNonlinearDynamic7'
//    '<S296>/MotorNonlinearDynamic8'
//
void MulticopterModelClass::Qu_MotorNonlinearDynamic1_Deriv
  (B_MotorNonlinearDynamic1_Quad_T *localB, XDot_MotorNonlinearDynamic1_Q_T
   *localXdot)
{
  // Derivatives for Atomic SubSystem: '<S305>/Motor_Dynamics'
  QuadModelv_Motor_Dynamics_Deriv(&localB->Motor_Dynamics,
    &localXdot->Motor_Dynamics);

  // End of Derivatives for SubSystem: '<S305>/Motor_Dynamics'
}

//
// Output and update for atomic system:
//    '<S408>/Acc NoiseFun'
//    '<S414>/Acc NoiseFun'
//    '<S418>/Acc NoiseFun'
//
void MulticopterModelClass::QuadModelv_AccNoiseFun(const real_T rtu_u[3],
  boolean_T rtu_isAccFault, const real_T rtu_AccFaultParams[20],
  B_AccNoiseFun_QuadModelv_T *localB)
{
  // MATLAB Function 'SensorFault/AccNoiseSwitch1/Acc NoiseFun': '<S422>:1'
  // '<S422>:1:3' y = 0.2*u;
  localB->y[0] = 0.2 * rtu_u[0];
  localB->y[1] = 0.2 * rtu_u[1];
  localB->y[2] = 0.2 * rtu_u[2];

  // '<S422>:1:5' if isAccFault
  if (rtu_isAccFault) {
    real_T a;

    // '<S422>:1:6' y = (0.2+0.8*AccFaultParams(1))*u;
    a = 0.8 * rtu_AccFaultParams[0] + 0.2;
    localB->y[0] = a * rtu_u[0];
    localB->y[1] = a * rtu_u[1];
    localB->y[2] = a * rtu_u[2];
  }
}

real_T rt_modd_snf(real_T u0, real_T u1)
{
  real_T y;
  y = u0;
  if (u1 == 0.0) {
    if (u0 == 0.0) {
      y = u1;
    }
  } else if (rtIsNaN(u0) || rtIsNaN(u1) || rtIsInf(u0)) {
    y = (rtNaN);
  } else if (u0 == 0.0) {
    y = 0.0 / u1;
  } else if (rtIsInf(u1)) {
    if ((u1 < 0.0) != (u0 < 0.0)) {
      y = u1;
    }
  } else {
    boolean_T yEq;
    y = std::fmod(u0, u1);
    yEq = (y == 0.0);
    if ((!yEq) && (u1 > std::floor(u1))) {
      real_T q;
      q = std::abs(u0 / u1);
      yEq = !(std::abs(q - std::floor(q + 0.5)) > DBL_EPSILON * q);
    }

    if (yEq) {
      y = u1 * 0.0;
    } else if ((u0 < 0.0) != (u1 < 0.0)) {
      y += u1;
    }
  }

  return y;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T u0_0;
    int32_T u1_0;
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = std::atan2(static_cast<real_T>(u0_0), static_cast<real_T>(u1_0));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = std::atan2(u0, u1);
  }

  return y;
}

// Function for MATLAB Function: '<Root>/CollisionDetection'
real_T MulticopterModelClass::QuadModelv_eml_rand_mt19937ar(uint32_T state[625])
{
  real_T r;
  uint32_T u[2];

  // ========================= COPYRIGHT NOTICE ============================
  //  This is a uniform (0,1) pseudorandom number generator based on:
  //
  //  A C-program for MT19937, with initialization improved 2002/1/26.
  //  Coded by Takuji Nishimura and Makoto Matsumoto.
  //
  //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
  //  All rights reserved.
  //
  //  Redistribution and use in source and binary forms, with or without
  //  modification, are permitted provided that the following conditions
  //  are met:
  //
  //    1. Redistributions of source code must retain the above copyright
  //       notice, this list of conditions and the following disclaimer.
  //
  //    2. Redistributions in binary form must reproduce the above copyright
  //       notice, this list of conditions and the following disclaimer
  //       in the documentation and/or other materials provided with the
  //       distribution.
  //
  //    3. The names of its contributors may not be used to endorse or
  //       promote products derived from this software without specific
  //       prior written permission.
  //
  //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
  //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  //
  // =============================   END   =================================
  int32_T exitg1;
  do {
    int32_T k;
    uint32_T mti;
    exitg1 = 0;
    for (k = 0; k < 2; k++) {
      uint32_T y;
      mti = state[624] + 1U;
      if (state[624] + 1U >= 625U) {
        for (int32_T kk = 0; kk < 227; kk++) {
          mti = (state[kk + 1] & 2147483647U) | (state[kk] & 2147483648U);
          if ((mti & 1U) == 0U) {
            mti >>= 1U;
          } else {
            mti = mti >> 1U ^ 2567483615U;
          }

          state[kk] = state[kk + 397] ^ mti;
        }

        for (int32_T kk = 0; kk < 396; kk++) {
          mti = (state[kk + 227] & 2147483648U) | (state[kk + 228] & 2147483647U);
          if ((mti & 1U) == 0U) {
            mti >>= 1U;
          } else {
            mti = mti >> 1U ^ 2567483615U;
          }

          state[kk + 227] = state[kk] ^ mti;
        }

        mti = (state[623] & 2147483648U) | (state[0] & 2147483647U);
        if ((mti & 1U) == 0U) {
          mti >>= 1U;
        } else {
          mti = mti >> 1U ^ 2567483615U;
        }

        state[623] = state[396] ^ mti;
        mti = 1U;
      }

      y = state[static_cast<int32_T>(mti) - 1];
      state[624] = mti;
      y ^= y >> 11U;
      y ^= y << 7U & 2636928640U;
      y ^= y << 15U & 4022730752U;
      u[k] = y >> 18U ^ y;
    }

    r = (static_cast<real_T>(u[0] >> 5U) * 6.7108864E+7 + static_cast<real_T>(u
          [1] >> 6U)) * 1.1102230246251565E-16;
    if (r == 0.0) {
      boolean_T b_isvalid;
      b_isvalid = ((state[624] >= 1U) && (state[624] < 625U));
      if (b_isvalid) {
        boolean_T exitg2;
        b_isvalid = false;
        k = 1;
        exitg2 = false;
        while ((!exitg2) && (k < 625)) {
          if (state[k - 1] == 0U) {
            k++;
          } else {
            b_isvalid = true;
            exitg2 = true;
          }
        }
      }

      if (!b_isvalid) {
        mti = 5489U;
        state[0] = 5489U;
        for (k = 0; k < 623; k++) {
          mti = ((mti >> 30U ^ mti) * 1812433253U + k) + 1U;
          state[k + 1] = mti;
        }

        state[624] = 624U;
      }
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return r;
}

// Function for MATLAB Function: '<Root>/CollisionDetection'
real_T MulticopterModelClass::QuadModelv_eml_rand_mcg16807(uint32_T *state)
{
  real_T r;
  int32_T hi;
  uint32_T a;
  uint32_T b;
  hi = static_cast<int32_T>(*state / 127773U);
  a = (*state - hi * 127773U) * 16807U;
  b = 2836U * hi;
  if (a < b) {
    a = ~(b - a) & 2147483647U;
  } else {
    a -= b;
  }

  r = static_cast<real_T>(a) * 4.6566128752457969E-10;
  *state = a;
  return r;
}

// Function for MATLAB Function: '<Root>/CollisionDetection'
real_T MulticopterModelClass::QuadModelv_rand(void)
{
  real_T r;
  switch (QuadModelv_DW.method_l) {
   case 4U:
    r = QuadModelv_eml_rand_mcg16807(&QuadModelv_DW.state_m);
    break;

   case 5U:
    {
      uint32_T b;
      uint32_T c;
      b = 69069U * QuadModelv_DW.state_fo[0] + 1234567U;
      c = QuadModelv_DW.state_fo[1] << 13 ^ QuadModelv_DW.state_fo[1];
      c ^= c >> 17;
      c ^= c << 5;
      QuadModelv_DW.state_fo[0] = b;
      QuadModelv_DW.state_fo[1] = c;
      r = static_cast<real_T>(b + c) * 2.328306436538696E-10;
    }
    break;

   default:
    r = QuadModelv_eml_rand_mt19937ar(QuadModelv_DW.state_p);
    break;
  }

  return r;
}

// Function for MATLAB Function: '<Root>/CollisionDetection'
void MulticopterModelClass::QuadModelv_rand_k(real_T r[3])
{
  switch (QuadModelv_DW.method_l) {
   case 4U:
    r[0] = QuadModelv_eml_rand_mcg16807(&QuadModelv_DW.state_m);
    r[1] = QuadModelv_eml_rand_mcg16807(&QuadModelv_DW.state_m);
    r[2] = QuadModelv_eml_rand_mcg16807(&QuadModelv_DW.state_m);
    break;

   case 5U:
    {
      uint32_T b;
      uint32_T c;
      b = 69069U * QuadModelv_DW.state_fo[0] + 1234567U;
      c = QuadModelv_DW.state_fo[1] << 13 ^ QuadModelv_DW.state_fo[1];
      c ^= c >> 17;
      c ^= c << 5;
      QuadModelv_DW.state_fo[0] = b;
      QuadModelv_DW.state_fo[1] = c;
      r[0] = static_cast<real_T>(b + c) * 2.328306436538696E-10;
      b = 69069U * QuadModelv_DW.state_fo[0] + 1234567U;
      c = QuadModelv_DW.state_fo[1] << 13 ^ QuadModelv_DW.state_fo[1];
      c ^= c >> 17;
      c ^= c << 5;
      QuadModelv_DW.state_fo[0] = b;
      QuadModelv_DW.state_fo[1] = c;
      r[1] = static_cast<real_T>(b + c) * 2.328306436538696E-10;
      b = 69069U * QuadModelv_DW.state_fo[0] + 1234567U;
      c = QuadModelv_DW.state_fo[1] << 13 ^ QuadModelv_DW.state_fo[1];
      c ^= c >> 17;
      c ^= c << 5;
      QuadModelv_DW.state_fo[0] = b;
      QuadModelv_DW.state_fo[1] = c;
      r[2] = static_cast<real_T>(b + c) * 2.328306436538696E-10;
    }
    break;

   default:
    r[0] = QuadModelv_eml_rand_mt19937ar(QuadModelv_DW.state_p);
    r[1] = QuadModelv_eml_rand_mt19937ar(QuadModelv_DW.state_p);
    r[2] = QuadModelv_eml_rand_mt19937ar(QuadModelv_DW.state_p);
    break;
  }
}

// Function for MATLAB Function: '<Root>/Crash_Motor_Fault'
real_T MulticopterModelClass::QuadModelv_rand_n(void)
{
  real_T r;
  uint32_T u[2];
  switch (QuadModelv_DW.method_a) {
   case 4U:
    {
      int32_T k;
      uint32_T mti;
      uint32_T y;
      k = static_cast<int32_T>(QuadModelv_DW.state_c / 127773U);
      mti = (QuadModelv_DW.state_c - k * 127773U) * 16807U;
      y = 2836U * k;
      if (mti < y) {
        mti = ~(y - mti) & 2147483647U;
      } else {
        mti -= y;
      }

      r = static_cast<real_T>(mti) * 4.6566128752457969E-10;
      QuadModelv_DW.state_c = mti;
    }
    break;

   case 5U:
    {
      uint32_T mti;
      uint32_T y;
      mti = 69069U * QuadModelv_DW.state_f[0] + 1234567U;
      y = QuadModelv_DW.state_f[1] << 13 ^ QuadModelv_DW.state_f[1];
      y ^= y >> 17;
      y ^= y << 5;
      QuadModelv_DW.state_f[0] = mti;
      QuadModelv_DW.state_f[1] = y;
      r = static_cast<real_T>(mti + y) * 2.328306436538696E-10;
    }
    break;

   default:
    {
      // ========================= COPYRIGHT NOTICE ============================ 
      //  This is a uniform (0,1) pseudorandom number generator based on:        
      //                                                                         
      //  A C-program for MT19937, with initialization improved 2002/1/26.       
      //  Coded by Takuji Nishimura and Makoto Matsumoto.                        
      //                                                                         
      //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      
      //  All rights reserved.                                                   
      //                                                                         
      //  Redistribution and use in source and binary forms, with or without     
      //  modification, are permitted provided that the following conditions     
      //  are met:                                                               
      //                                                                         
      //    1. Redistributions of source code must retain the above copyright    
      //       notice, this list of conditions and the following disclaimer.     
      //                                                                         
      //    2. Redistributions in binary form must reproduce the above copyright 
      //       notice, this list of conditions and the following disclaimer      
      //       in the documentation and/or other materials provided with the     
      //       distribution.                                                     
      //                                                                         
      //    3. The names of its contributors may not be used to endorse or       
      //       promote products derived from this software without specific      
      //       prior written permission.                                         
      //                                                                         
      //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    
      //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      
      //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  
      //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  
      //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  
      //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       
      //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  
      //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  
      //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    
      //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
      //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
      //                                                                         
      // =============================   END   ================================= 
      int32_T exitg1;
      do {
        int32_T k;
        uint32_T mti;
        exitg1 = 0;
        for (k = 0; k < 2; k++) {
          uint32_T y;
          mti = QuadModelv_DW.state_g[624] + 1U;
          if (QuadModelv_DW.state_g[624] + 1U >= 625U) {
            for (int32_T kk = 0; kk < 227; kk++) {
              mti = (QuadModelv_DW.state_g[kk + 1] & 2147483647U) |
                (QuadModelv_DW.state_g[kk] & 2147483648U);
              if ((mti & 1U) == 0U) {
                mti >>= 1U;
              } else {
                mti = mti >> 1U ^ 2567483615U;
              }

              QuadModelv_DW.state_g[kk] = QuadModelv_DW.state_g[kk + 397] ^ mti;
            }

            for (int32_T kk = 0; kk < 396; kk++) {
              mti = (QuadModelv_DW.state_g[kk + 227] & 2147483648U) |
                (QuadModelv_DW.state_g[kk + 228] & 2147483647U);
              if ((mti & 1U) == 0U) {
                mti >>= 1U;
              } else {
                mti = mti >> 1U ^ 2567483615U;
              }

              QuadModelv_DW.state_g[kk + 227] = QuadModelv_DW.state_g[kk] ^ mti;
            }

            mti = (QuadModelv_DW.state_g[623] & 2147483648U) |
              (QuadModelv_DW.state_g[0] & 2147483647U);
            if ((mti & 1U) == 0U) {
              mti >>= 1U;
            } else {
              mti = mti >> 1U ^ 2567483615U;
            }

            QuadModelv_DW.state_g[623] = QuadModelv_DW.state_g[396] ^ mti;
            mti = 1U;
          }

          y = QuadModelv_DW.state_g[static_cast<int32_T>(mti) - 1];
          QuadModelv_DW.state_g[624] = mti;
          y ^= y >> 11U;
          y ^= y << 7U & 2636928640U;
          y ^= y << 15U & 4022730752U;
          u[k] = y >> 18U ^ y;
        }

        r = (static_cast<real_T>(u[0] >> 5U) * 6.7108864E+7 + static_cast<real_T>
             (u[1] >> 6U)) * 1.1102230246251565E-16;
        if (r == 0.0) {
          boolean_T b_isvalid;
          b_isvalid = ((QuadModelv_DW.state_g[624] >= 1U) &&
                       (QuadModelv_DW.state_g[624] < 625U));
          if (b_isvalid) {
            boolean_T exitg2;
            b_isvalid = false;
            k = 1;
            exitg2 = false;
            while ((!exitg2) && (k < 625)) {
              if (QuadModelv_DW.state_g[k - 1] == 0U) {
                k++;
              } else {
                b_isvalid = true;
                exitg2 = true;
              }
            }
          }

          if (!b_isvalid) {
            mti = 5489U;
            QuadModelv_DW.state_g[0] = 5489U;
            for (k = 0; k < 623; k++) {
              mti = ((mti >> 30U ^ mti) * 1812433253U + k) + 1U;
              QuadModelv_DW.state_g[k + 1] = mti;
            }

            QuadModelv_DW.state_g[624] = 624U;
          }
        } else {
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    break;
  }

  return r;
}

// Function for MATLAB Function: '<S67>/MATLAB Function'
real_T MulticopterModelClass::QuadModelv_rand_p(void)
{
  real_T r;
  uint32_T u[2];
  switch (QuadModelv_DW.method) {
   case 4U:
    {
      int32_T k;
      uint32_T mti;
      uint32_T y;
      k = static_cast<int32_T>(QuadModelv_DW.state / 127773U);
      mti = (QuadModelv_DW.state - k * 127773U) * 16807U;
      y = 2836U * k;
      if (mti < y) {
        mti = ~(y - mti) & 2147483647U;
      } else {
        mti -= y;
      }

      r = static_cast<real_T>(mti) * 4.6566128752457969E-10;
      QuadModelv_DW.state = mti;
    }
    break;

   case 5U:
    {
      uint32_T mti;
      uint32_T y;
      mti = 69069U * QuadModelv_DW.state_i[0] + 1234567U;
      y = QuadModelv_DW.state_i[1] << 13 ^ QuadModelv_DW.state_i[1];
      y ^= y >> 17;
      y ^= y << 5;
      QuadModelv_DW.state_i[0] = mti;
      QuadModelv_DW.state_i[1] = y;
      r = static_cast<real_T>(mti + y) * 2.328306436538696E-10;
    }
    break;

   default:
    {
      // ========================= COPYRIGHT NOTICE ============================ 
      //  This is a uniform (0,1) pseudorandom number generator based on:        
      //                                                                         
      //  A C-program for MT19937, with initialization improved 2002/1/26.       
      //  Coded by Takuji Nishimura and Makoto Matsumoto.                        
      //                                                                         
      //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      
      //  All rights reserved.                                                   
      //                                                                         
      //  Redistribution and use in source and binary forms, with or without     
      //  modification, are permitted provided that the following conditions     
      //  are met:                                                               
      //                                                                         
      //    1. Redistributions of source code must retain the above copyright    
      //       notice, this list of conditions and the following disclaimer.     
      //                                                                         
      //    2. Redistributions in binary form must reproduce the above copyright 
      //       notice, this list of conditions and the following disclaimer      
      //       in the documentation and/or other materials provided with the     
      //       distribution.                                                     
      //                                                                         
      //    3. The names of its contributors may not be used to endorse or       
      //       promote products derived from this software without specific      
      //       prior written permission.                                         
      //                                                                         
      //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    
      //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      
      //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  
      //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  
      //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  
      //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       
      //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  
      //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  
      //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    
      //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
      //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
      //                                                                         
      // =============================   END   ================================= 
      int32_T exitg1;
      do {
        int32_T k;
        uint32_T mti;
        exitg1 = 0;
        for (k = 0; k < 2; k++) {
          uint32_T y;
          mti = QuadModelv_DW.state_a[624] + 1U;
          if (QuadModelv_DW.state_a[624] + 1U >= 625U) {
            for (int32_T kk = 0; kk < 227; kk++) {
              mti = (QuadModelv_DW.state_a[kk + 1] & 2147483647U) |
                (QuadModelv_DW.state_a[kk] & 2147483648U);
              if ((mti & 1U) == 0U) {
                mti >>= 1U;
              } else {
                mti = mti >> 1U ^ 2567483615U;
              }

              QuadModelv_DW.state_a[kk] = QuadModelv_DW.state_a[kk + 397] ^ mti;
            }

            for (int32_T kk = 0; kk < 396; kk++) {
              mti = (QuadModelv_DW.state_a[kk + 227] & 2147483648U) |
                (QuadModelv_DW.state_a[kk + 228] & 2147483647U);
              if ((mti & 1U) == 0U) {
                mti >>= 1U;
              } else {
                mti = mti >> 1U ^ 2567483615U;
              }

              QuadModelv_DW.state_a[kk + 227] = QuadModelv_DW.state_a[kk] ^ mti;
            }

            mti = (QuadModelv_DW.state_a[623] & 2147483648U) |
              (QuadModelv_DW.state_a[0] & 2147483647U);
            if ((mti & 1U) == 0U) {
              mti >>= 1U;
            } else {
              mti = mti >> 1U ^ 2567483615U;
            }

            QuadModelv_DW.state_a[623] = QuadModelv_DW.state_a[396] ^ mti;
            mti = 1U;
          }

          y = QuadModelv_DW.state_a[static_cast<int32_T>(mti) - 1];
          QuadModelv_DW.state_a[624] = mti;
          y ^= y >> 11U;
          y ^= y << 7U & 2636928640U;
          y ^= y << 15U & 4022730752U;
          u[k] = y >> 18U ^ y;
        }

        r = (static_cast<real_T>(u[0] >> 5U) * 6.7108864E+7 + static_cast<real_T>
             (u[1] >> 6U)) * 1.1102230246251565E-16;
        if (r == 0.0) {
          boolean_T b_isvalid;
          b_isvalid = ((QuadModelv_DW.state_a[624] >= 1U) &&
                       (QuadModelv_DW.state_a[624] < 625U));
          if (b_isvalid) {
            boolean_T exitg2;
            b_isvalid = false;
            k = 1;
            exitg2 = false;
            while ((!exitg2) && (k < 625)) {
              if (QuadModelv_DW.state_a[k - 1] == 0U) {
                k++;
              } else {
                b_isvalid = true;
                exitg2 = true;
              }
            }
          }

          if (!b_isvalid) {
            mti = 5489U;
            QuadModelv_DW.state_a[0] = 5489U;
            for (k = 0; k < 623; k++) {
              mti = ((mti >> 30U ^ mti) * 1812433253U + k) + 1U;
              QuadModelv_DW.state_a[k + 1] = mti;
            }

            QuadModelv_DW.state_a[624] = 624U;
          }
        } else {
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    break;
  }

  return r;
}

void rt_mrdivide_U1d1x3_U2d_9vOrDY9Z(const real_T u0[3], const real_T u1[9],
  real_T y[3])
{
  real_T A[9];
  real_T a21;
  real_T maxval;
  int32_T r1;
  int32_T r2;
  int32_T r3;
  std::memcpy(&A[0], &u1[0], 9U * sizeof(real_T));
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = std::abs(u1[0]);
  a21 = std::abs(u1[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (std::abs(u1[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  A[r2] = u1[r2] / u1[r1];
  A[r3] /= A[r1];
  A[r2 + 3] -= A[r1 + 3] * A[r2];
  A[r3 + 3] -= A[r1 + 3] * A[r3];
  A[r2 + 6] -= A[r1 + 6] * A[r2];
  A[r3 + 6] -= A[r1 + 6] * A[r3];
  if (std::abs(A[r3 + 3]) > std::abs(A[r2 + 3])) {
    int32_T rtemp;
    rtemp = r2 + 1;
    r2 = r3;
    r3 = rtemp - 1;
  }

  A[r3 + 3] /= A[r2 + 3];
  A[r3 + 6] -= A[r3 + 3] * A[r2 + 6];
  y[r1] = u0[0] / A[r1];
  y[r2] = u0[1] - A[r1 + 3] * y[r1];
  y[r3] = u0[2] - A[r1 + 6] * y[r1];
  y[r2] /= A[r2 + 3];
  y[r3] -= A[r2 + 6] * y[r2];
  y[r3] /= A[r3 + 6];
  y[r2] -= A[r3 + 3] * y[r3];
  y[r1] -= y[r3] * A[r3];
  y[r1] -= y[r2] * A[r2];
}

real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (std::abs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = std::floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = std::ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

real32_T rt_roundf_snf(real32_T u)
{
  real32_T y;
  if (std::abs(u) < 8.388608E+6F) {
    if (u >= 0.5F) {
      y = std::floor(u + 0.5F);
    } else if (u > -0.5F) {
      y = u * 0.0F;
    } else {
      y = std::ceil(u - 0.5F);
    }
  } else {
    y = u;
  }

  return y;
}

// Function for MATLAB Function: '<S12>/Model Fail Assessment'
real_T MulticopterModelClass::QuadModelv_norm(const real_T x[3])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.3121686421112381E-170;
  absxk = std::abs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = std::abs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = std::abs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * std::sqrt(y);
}

// Function for MATLAB Function: '<S8>/MATLAB Function'
void MulticopterModelClass::QuadModelv_eml_find(const boolean_T x[8], int32_T
  i_data[], int32_T *i_size)
{
  int32_T idx;
  int32_T ii;
  boolean_T exitg1;
  idx = 0;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii < 8)) {
    if (x[ii]) {
      idx++;
      i_data[idx - 1] = ii + 1;
      if (idx >= 8) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }

  if (idx < 1) {
    *i_size = 0;
  } else {
    *i_size = idx;
  }
}

// Function for MATLAB Function: '<S8>/MATLAB Function'
boolean_T MulticopterModelClass::QuadModelv_any(const int32_T *x_size)
{
  return *x_size >= 1;
}

real_T rt_urand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  uint32_T hi;
  uint32_T lo;

  // Uniform random number generator (random number between 0 and 1)

  // #define IA      16807                      magic multiplier = 7^5
  // #define IM      2147483647                 modulus = 2^31-1
  // #define IQ      127773                     IM div IA
  // #define IR      2836                       IM modulo IA
  // #define S       4.656612875245797e-10      reciprocal of 2^31-1
  // test = IA * (seed % IQ) - IR * (seed/IQ)
  // seed = test < 0 ? (test + IM) : test
  // return (seed*S)

  lo = *u % 127773U * 16807U;
  hi = *u / 127773U * 2836U;
  if (lo < hi) {
    *u = 2147483647U - (hi - lo);
  } else {
    *u = lo - hi;
  }

  return static_cast<real_T>(*u) * 4.6566128752457969E-10;
}

real_T rt_nrand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  real_T si;
  real_T sr;
  real_T y;

  // Normal (Gaussian) random number generator
  do {
    sr = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = sr * sr + si * si;
  } while (si > 1.0);

  y = std::sqrt(-2.0 * std::log(si) / si) * sr;
  return y;
}

// Model step function
void MulticopterModelClass::step()
{
  // local block i/o variables
  real_T rtb_DataTypeConversion1[3];

  // local scratch DWork variables
  int32_T ForEach_itr;
  real_T rtb_tc_old[169];
  real_T rtb_y_i[23];
  real_T LoadLeakParams[20];
  real_T LoadShiftParams[20];
  real_T fParamTmp[20];
  real_T inSILFloats[20];
  real_T rtb_FaultParam[20];
  real_T rtb_FaultParam_ak[20];
  real_T rtb_MatrixConcatenation[18];
  real_T rtb_TmpSignalConversionAtppnInp[13];
  real_T rtb_Assignment[11];
  real_T rtb_Assignment1[11];
  real_T VectorConcatenate[9];
  real_T VectorConcatenate_p[9];
  real_T rtb_uavlostJ[9];
  real_T Memory[8];
  real_T rtb_Gain_ok[8];
  real_T rtb_PWMOut[8];
  real_T ddm[6];
  real_T tz[4];
  real_T rtb_DataTypeConversion2[3];
  real_T rtb_Mb[3];
  real_T rtb_Merge[3];
  real_T rtb_Sum4_a[3];
  real_T rtb_Windb[3];
  real_T rtb_sincos_o1_f[3];
  real_T tmp[3];
  real_T frac[2];
  real_T frac_0[2];
  real_T rtb_Sum_d[2];
  real_T FaultNum;
  real_T LoadLeakRatio;
  real_T VectorConcatenate_n_idx_0;
  real_T VectorConcatenate_n_idx_1;
  real_T VectorConcatenate_n_idx_2;
  real_T VectorConcatenate_tmp;
  real_T VectorConcatenate_tmp_0;
  real_T VectorConcatenate_tmp_1;
  real_T Y;
  real_T Z;
  real_T j;
  real_T q0_tmp;
  real_T rtb_Add;
  real_T rtb_Add_br;
  real_T rtb_DataTypeConversion_idx_0;
  real_T rtb_DataTypeConversion_idx_1;
  real_T rtb_DataTypeConversion_idx_2;
  real_T rtb_Product3_o;
  real_T rtb_Rn;
  real_T rtb_Saturation_1;
  real_T rtb_Sum1_c;
  real_T rtb_Sum_o1;
  real_T rtb_TmpSignalConversionAtSFun_0;
  real_T rtb_TmpSignalConversionAtSFun_1;
  real_T rtb_TmpSignalConversionAtSFun_2;
  real_T rtb_UniformRandomNumber7_idx_2;
  real_T rtb_count;
  real_T rtb_fcn3;
  real_T rtb_q2dot;
  real_T rtb_q2dot_tmp;
  real_T rtb_q3dot;
  real_T rtb_sincos_o2_idx_0_tmp;
  real_T uavlostJzx;
  real_T y;
  int32_T tmp_data[8];
  int32_T b_t;
  int32_T i;
  int32_T loop_ub;
  int32_T r;
  int32_T r_0;
  int32_T r_1;
  int32_T rtb_alt;
  real32_T tmp_0;
  real32_T tmp_1;
  real32_T tmp_2;
  uint32_T bpIndex[2];
  uint32_T bpIndex_0[2];
  uint32_T rtb_time_usec;
  int8_T rtAction;
  int8_T rtb_landed;
  uint8_T ForIterator_IterationMarker[6];
  boolean_T Memory_0[8];
  boolean_T hFaultTmp;
  boolean_T rtb_Compare_b5;
  boolean_T rtb_RelationalOperator;
  boolean_T rtb_hasFault;
  boolean_T rtb_hasFault_b;
  static const int8_T c[10] = { 3, 3, 4, 4, 6, 6, 6, 8, 8, 8 };

  static const real_T d[80] = { 1.0471975511965976, 1.0471975511965976,
    0.78539816339744828, 1.5707963267948966, 1.5707963267948966, 0.0,
    1.0471975511965976, 0.39269908169872414, 0.0, 0.78539816339744828,
    3.1415926535897931, 3.1415926535897931, 3.9269908169872414, 4.71238898038469,
    4.71238898038469, 3.1415926535897931, 1.0471975511965976, 3.5342917352885173,
    3.1415926535897931, -0.78539816339744828, -1.0471975511965976,
    -1.0471975511965976, 5.497787143782138, 0.0, 5.7595865315812871,
    4.1887902047863905, 3.1415926535897931, 1.1780972450961724,
    0.78539816339744828, 3.9269908169872414, 0.0, 0.0, 2.3561944901923448,
    3.1415926535897931, 2.617993877991494, 1.0471975511965976,
    3.1415926535897931, 2.748893571891069, 2.3561944901923448,
    2.3561944901923448, 0.0, 0.0, 0.0, 0.0, 0.52359877559829893,
    -1.0471975511965976, -1.0471975511965976, -0.39269908169872414,
    -0.78539816339744828, -0.78539816339744828, 0.0, 0.0, 0.0, 0.0,
    3.6651914291880923, 2.0943951023931957, -1.0471975511965976,
    4.3196898986859651, 3.9269908169872414, 0.78539816339744828, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 5.1050880620834143, 4.71238898038469, 2.3561944901923448,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.9634954084936207, 1.5707963267948966,
    3.9269908169872414 };

  static const int8_T b[80] = { -1, 1, -1, -1, 1, 1, 1, 1, 1, 1, -1, 1, -1, -1,
    -1, -1, -1, 1, 1, -1, -1, 1, 1, 1, 1, 1, 1, -1, -1, 1, 0, 0, 1, 1, -1, -1,
    -1, -1, -1, -1, 0, 0, 0, 0, -1, -1, 1, -1, -1, 1, 0, 0, 0, 0, 1, 1, -1, -1,
    -1, -1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, -1 };

  if (rtmIsMajorTimeStep((&QuadModelv_M))) {
    // set solver stop time
    rtsiSetSolverStopTime(&(&QuadModelv_M)->solverInfo,(((&QuadModelv_M)
      ->Timing.clockTick0+1)*(&QuadModelv_M)->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep((&QuadModelv_M))) {
    (&QuadModelv_M)->Timing.t[0] = rtsiGetT(&(&QuadModelv_M)->solverInfo);
  }

  // Clock: '<S12>/Clock1' incorporates:
  //   Clock: '<Root>/Clock'
  //   Clock: '<S298>/Time'
  //   Clock: '<S2>/Clock1'
  //   Clock: '<S524>/Time'
  //   Clock: '<S67>/Clock1'
  //   Clock: '<S6>/Clock'

  FaultNum = (&QuadModelv_M)->Timing.t[0];

  // DataTypeConversion: '<S12>/Data Type Conversion1' incorporates:
  //   Clock: '<S12>/Clock1'
  //   Gain: '<S12>/Gain_clock'

  rtb_TmpSignalConversionAtSFun_0 = std::floor(QuadModelv_P.Gain_clock_Gain *
    FaultNum);
  if (rtIsNaN(rtb_TmpSignalConversionAtSFun_0) || rtIsInf
      (rtb_TmpSignalConversionAtSFun_0)) {
    rtb_TmpSignalConversionAtSFun_0 = 0.0;
  } else {
    rtb_TmpSignalConversionAtSFun_0 = std::fmod(rtb_TmpSignalConversionAtSFun_0,
      4.294967296E+9);
  }

  rtb_time_usec = rtb_TmpSignalConversionAtSFun_0 < 0.0 ? static_cast<uint32_T>(
    -static_cast<int32_T>(static_cast<uint32_T>(-rtb_TmpSignalConversionAtSFun_0)))
    : static_cast<uint32_T>(rtb_TmpSignalConversionAtSFun_0);

  // End of DataTypeConversion: '<S12>/Data Type Conversion1'
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Gain: '<S25>/1//2' incorporates:
    //   Constant: '<S15>/Initial Euler Angles'

    rtb_sincos_o1_f[0] = QuadModelv_P.u2_Gain * QuadModelv_P.ModelInit_AngEuler
      [2];
    rtb_sincos_o1_f[1] = QuadModelv_P.u2_Gain * QuadModelv_P.ModelInit_AngEuler
      [1];
    rtb_sincos_o1_f[2] = QuadModelv_P.u2_Gain * QuadModelv_P.ModelInit_AngEuler
      [0];

    // Trigonometry: '<S25>/sincos'
    rtb_TmpSignalConversionAtSFun_0 = std::cos(rtb_sincos_o1_f[0]);
    LoadLeakRatio = std::sin(rtb_sincos_o1_f[0]);
    rtb_TmpSignalConversionAtSFun_1 = std::cos(rtb_sincos_o1_f[1]);
    rtb_Saturation_1 = std::sin(rtb_sincos_o1_f[1]);
    rtb_TmpSignalConversionAtSFun_2 = std::cos(rtb_sincos_o1_f[2]);
    Y = std::sin(rtb_sincos_o1_f[2]);

    // Fcn: '<S25>/q0' incorporates:
    //   Fcn: '<S25>/q1'

    rtb_Product3_o = rtb_TmpSignalConversionAtSFun_0 *
      rtb_TmpSignalConversionAtSFun_1;
    q0_tmp = LoadLeakRatio * rtb_Saturation_1;

    // Fcn: '<S25>/q0'
    QuadModelv_B.q0 = rtb_Product3_o * rtb_TmpSignalConversionAtSFun_2 + q0_tmp *
      Y;

    // Fcn: '<S25>/q1'
    QuadModelv_B.q1 = rtb_Product3_o * Y - q0_tmp *
      rtb_TmpSignalConversionAtSFun_2;

    // Fcn: '<S25>/q2' incorporates:
    //   Fcn: '<S25>/q3'

    LoadLeakRatio *= rtb_TmpSignalConversionAtSFun_1;
    rtb_Saturation_1 *= rtb_TmpSignalConversionAtSFun_0;

    // Fcn: '<S25>/q2'
    QuadModelv_B.q2 = rtb_Saturation_1 * rtb_TmpSignalConversionAtSFun_2 +
      LoadLeakRatio * Y;

    // Fcn: '<S25>/q3'
    QuadModelv_B.q3 = LoadLeakRatio * rtb_TmpSignalConversionAtSFun_2 -
      rtb_Saturation_1 * Y;
  }

  // Integrator: '<S15>/q0 q1 q2 q3' incorporates:
  //   SignalConversion generated from: '<S15>/q0 q1 q2 q3'

  if (QuadModelv_DW.q0q1q2q3_IWORK != 0) {
    QuadModelv_X.q0q1q2q3_CSTATE[0] = QuadModelv_B.q0;
    QuadModelv_X.q0q1q2q3_CSTATE[1] = QuadModelv_B.q1;
    QuadModelv_X.q0q1q2q3_CSTATE[2] = QuadModelv_B.q2;
    QuadModelv_X.q0q1q2q3_CSTATE[3] = QuadModelv_B.q3;
  }

  // Sqrt: '<S38>/sqrt' incorporates:
  //   Integrator: '<S15>/q0 q1 q2 q3'
  //   Product: '<S39>/Product'
  //   Product: '<S39>/Product1'
  //   Product: '<S39>/Product2'
  //   Product: '<S39>/Product3'
  //   Sum: '<S39>/Sum'

  rtb_TmpSignalConversionAtSFun_2 = std::sqrt(((QuadModelv_X.q0q1q2q3_CSTATE[0] *
    QuadModelv_X.q0q1q2q3_CSTATE[0] + QuadModelv_X.q0q1q2q3_CSTATE[1] *
    QuadModelv_X.q0q1q2q3_CSTATE[1]) + QuadModelv_X.q0q1q2q3_CSTATE[2] *
    QuadModelv_X.q0q1q2q3_CSTATE[2]) + QuadModelv_X.q0q1q2q3_CSTATE[3] *
    QuadModelv_X.q0q1q2q3_CSTATE[3]);

  // Product: '<S37>/Product' incorporates:
  //   Integrator: '<S15>/q0 q1 q2 q3'

  Y = QuadModelv_X.q0q1q2q3_CSTATE[0] / rtb_TmpSignalConversionAtSFun_2;

  // Product: '<S37>/Product1' incorporates:
  //   Integrator: '<S15>/q0 q1 q2 q3'

  rtb_Saturation_1 = QuadModelv_X.q0q1q2q3_CSTATE[1] /
    rtb_TmpSignalConversionAtSFun_2;

  // Product: '<S37>/Product2' incorporates:
  //   Integrator: '<S15>/q0 q1 q2 q3'

  rtb_Product3_o = QuadModelv_X.q0q1q2q3_CSTATE[2] /
    rtb_TmpSignalConversionAtSFun_2;

  // Product: '<S37>/Product3' incorporates:
  //   Integrator: '<S15>/q0 q1 q2 q3'

  rtb_TmpSignalConversionAtSFun_2 = QuadModelv_X.q0q1q2q3_CSTATE[3] /
    rtb_TmpSignalConversionAtSFun_2;

  // Product: '<S27>/Product3' incorporates:
  //   Product: '<S31>/Product3'

  LoadLeakRatio = Y * Y;

  // Product: '<S27>/Product2' incorporates:
  //   Product: '<S31>/Product2'

  q0_tmp = rtb_Saturation_1 * rtb_Saturation_1;

  // Product: '<S27>/Product1' incorporates:
  //   Product: '<S31>/Product1'
  //   Product: '<S35>/Product1'

  VectorConcatenate_tmp_0 = rtb_Product3_o * rtb_Product3_o;

  // Product: '<S27>/Product' incorporates:
  //   Product: '<S31>/Product'
  //   Product: '<S35>/Product'

  VectorConcatenate_tmp_1 = rtb_TmpSignalConversionAtSFun_2 *
    rtb_TmpSignalConversionAtSFun_2;

  // Sum: '<S27>/Sum' incorporates:
  //   Concatenate: '<S36>/Vector Concatenate'
  //   Product: '<S27>/Product'
  //   Product: '<S27>/Product1'
  //   Product: '<S27>/Product2'
  //   Product: '<S27>/Product3'

  VectorConcatenate[0] = ((LoadLeakRatio + q0_tmp) - VectorConcatenate_tmp_0) -
    VectorConcatenate_tmp_1;

  // Product: '<S30>/Product3' incorporates:
  //   Product: '<S28>/Product3'

  VectorConcatenate_tmp = rtb_TmpSignalConversionAtSFun_2 * Y;

  // Product: '<S30>/Product2' incorporates:
  //   Product: '<S28>/Product2'

  VectorConcatenate_n_idx_0 = rtb_Saturation_1 * rtb_Product3_o;

  // Gain: '<S30>/Gain' incorporates:
  //   Concatenate: '<S36>/Vector Concatenate'
  //   Product: '<S30>/Product2'
  //   Product: '<S30>/Product3'
  //   Sum: '<S30>/Sum'

  VectorConcatenate[1] = (VectorConcatenate_n_idx_0 - VectorConcatenate_tmp) *
    QuadModelv_P.Gain_Gain_pu;

  // Product: '<S33>/Product2' incorporates:
  //   Product: '<S29>/Product2'

  VectorConcatenate_n_idx_1 = rtb_Saturation_1 * rtb_TmpSignalConversionAtSFun_2;

  // Product: '<S33>/Product1' incorporates:
  //   Product: '<S29>/Product1'

  rtb_Add_br = Y * rtb_Product3_o;

  // Gain: '<S33>/Gain' incorporates:
  //   Concatenate: '<S36>/Vector Concatenate'
  //   Product: '<S33>/Product1'
  //   Product: '<S33>/Product2'
  //   Sum: '<S33>/Sum'

  VectorConcatenate[2] = (rtb_Add_br + VectorConcatenate_n_idx_1) *
    QuadModelv_P.Gain_Gain_o;

  // Gain: '<S28>/Gain' incorporates:
  //   Concatenate: '<S36>/Vector Concatenate'
  //   Sum: '<S28>/Sum'

  VectorConcatenate[3] = (VectorConcatenate_tmp + VectorConcatenate_n_idx_0) *
    QuadModelv_P.Gain_Gain_n;

  // Sum: '<S31>/Sum' incorporates:
  //   Concatenate: '<S36>/Vector Concatenate'
  //   Sum: '<S35>/Sum'

  LoadLeakRatio -= q0_tmp;
  VectorConcatenate[4] = (LoadLeakRatio + VectorConcatenate_tmp_0) -
    VectorConcatenate_tmp_1;

  // Product: '<S34>/Product1' incorporates:
  //   Product: '<S32>/Product1'

  q0_tmp = Y * rtb_Saturation_1;

  // Product: '<S34>/Product2' incorporates:
  //   Product: '<S32>/Product2'

  VectorConcatenate_tmp = rtb_Product3_o * rtb_TmpSignalConversionAtSFun_2;

  // Gain: '<S34>/Gain' incorporates:
  //   Concatenate: '<S36>/Vector Concatenate'
  //   Product: '<S34>/Product1'
  //   Product: '<S34>/Product2'
  //   Sum: '<S34>/Sum'

  VectorConcatenate[5] = (VectorConcatenate_tmp - q0_tmp) *
    QuadModelv_P.Gain_Gain_nd;

  // Gain: '<S29>/Gain' incorporates:
  //   Concatenate: '<S36>/Vector Concatenate'
  //   Sum: '<S29>/Sum'

  VectorConcatenate[6] = (VectorConcatenate_n_idx_1 - rtb_Add_br) *
    QuadModelv_P.Gain_Gain_af;

  // Gain: '<S32>/Gain' incorporates:
  //   Concatenate: '<S36>/Vector Concatenate'
  //   Sum: '<S32>/Sum'

  VectorConcatenate[7] = (q0_tmp + VectorConcatenate_tmp) *
    QuadModelv_P.Gain_Gain_c;

  // Sum: '<S35>/Sum' incorporates:
  //   Concatenate: '<S36>/Vector Concatenate'

  VectorConcatenate[8] = (LoadLeakRatio - VectorConcatenate_tmp_0) +
    VectorConcatenate_tmp_1;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Constant: '<S187>/FaultID'
    QuadModelv_B.FaultID = QuadModelv_P.FaultID_Value;
  }

  for (i = 0; i < 20; i++) {
    // DataTypeConversion: '<Root>/Data Type Conversion1' incorporates:
    //   Inport: '<Root>/inSILFloats'

    inSILFloats[i] = QuadModelv_U.inSILFloats[i];
  }

  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // MATLAB Function: '<S187>/FaultParamsExtract3' incorporates:
    //   Inport: '<Root>/inSILInts'

    // MATLAB Function 'LoadFault/FaultParamsExtract3': '<S191>:1'
    // '<S191>:1:5' if isempty(hFault)
    // '<S191>:1:8' if isempty(fParam)
    // '<S191>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S191>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S191>:1:14' j=1;
    j = 1.0;

    // '<S191>:1:15' for i=1:8
    for (b_t = 0; b_t < 8; b_t++) {
      // '<S191>:1:16' if inInts(i) == FaultID
      if (QuadModelv_U.inSILInts[b_t] == QuadModelv_B.FaultID) {
        // '<S191>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S191>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        rtb_alt = (b_t + 1) << 1;
        fParamTmp[static_cast<int32_T>(2.0 * j - 1.0) - 1] = inSILFloats[rtb_alt
          - 2];

        // '<S191>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[static_cast<int32_T>(2.0 * j) - 1] = inSILFloats[rtb_alt - 1];

        // '<S191>:1:20' j=j+1;
        j++;
      }
    }

    // '<S191>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S191>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_nj = true;

      // '<S191>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = inSILFloats[16];
      fParamTmp[17] = inSILFloats[17];
      fParamTmp[18] = inSILFloats[18];
      fParamTmp[19] = inSILFloats[19];

      // '<S191>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_p[0], &fParamTmp[0], 20U * sizeof(real_T));
    }

    // MATLAB Function: '<S187>/FaultParamsExtract1' incorporates:
    //   Constant: '<S187>/FaultID1'
    //   Inport: '<Root>/inSILInts'

    // '<S191>:1:29' hasFault_isloadFall=hFault;
    // '<S191>:1:30' FaultParam=fParam;
    // MATLAB Function 'LoadFault/FaultParamsExtract1': '<S189>:1'
    // '<S189>:1:5' if isempty(hFault)
    // '<S189>:1:8' if isempty(fParam)
    // '<S189>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S189>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S189>:1:14' j=1;
    j = 1.0;

    // '<S189>:1:15' for i=1:8
    for (b_t = 0; b_t < 8; b_t++) {
      // '<S189>:1:16' if inInts(i) == FaultID
      if (QuadModelv_U.inSILInts[b_t] == QuadModelv_P.FaultID1_Value) {
        // '<S189>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S189>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        fParamTmp[static_cast<int32_T>(2.0 * j - 1.0) - 1] = inSILFloats[((b_t +
          1) << 1) - 2];

        // '<S189>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[static_cast<int32_T>(2.0 * j) - 1] = inSILFloats[((b_t + 1) <<
          1) - 1];

        // '<S189>:1:20' j=j+1;
        j++;
      }
    }

    // '<S189>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S189>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_j = true;

      // '<S189>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = inSILFloats[16];
      fParamTmp[17] = inSILFloats[17];
      fParamTmp[18] = inSILFloats[18];
      fParamTmp[19] = inSILFloats[19];

      // '<S189>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_f[0], &fParamTmp[0], 20U * sizeof(real_T));
    }

    // MATLAB Function: '<S187>/FaultParamsExtract2' incorporates:
    //   Constant: '<S187>/FaultID2'
    //   Inport: '<Root>/inSILInts'

    // '<S189>:1:29' hasFault_isloadShift=hFault;
    // '<S189>:1:30' FaultParam=fParam;
    // MATLAB Function 'LoadFault/FaultParamsExtract2': '<S190>:1'
    // '<S190>:1:5' if isempty(hFault)
    // '<S190>:1:8' if isempty(fParam)
    // '<S190>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S190>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S190>:1:14' j=1;
    j = 1.0;

    // '<S190>:1:15' for i=1:8
    for (b_t = 0; b_t < 8; b_t++) {
      // '<S190>:1:16' if inInts(i) == FaultID
      if (QuadModelv_U.inSILInts[b_t] == QuadModelv_P.FaultID2_Value) {
        // '<S190>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S190>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        fParamTmp[static_cast<int32_T>(2.0 * j - 1.0) - 1] = inSILFloats[((b_t +
          1) << 1) - 2];

        // '<S190>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[static_cast<int32_T>(2.0 * j) - 1] = inSILFloats[((b_t + 1) <<
          1) - 1];

        // '<S190>:1:20' j=j+1;
        j++;
      }
    }

    // '<S190>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S190>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_no = true;

      // '<S190>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = inSILFloats[16];
      fParamTmp[17] = inSILFloats[17];
      fParamTmp[18] = inSILFloats[18];
      fParamTmp[19] = inSILFloats[19];

      // '<S190>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_px[0], &fParamTmp[0], 20U * sizeof
                  (real_T));
    }

    // MATLAB Function: '<S6>/Model Failure' incorporates:
    //   Concatenate: '<S36>/Vector Concatenate'
    //   Constant: '<S6>/ModelParam.uavMass'
    //   Constant: '<S6>/ModelParam.uavR'
    //   MATLAB Function: '<S187>/FaultParamsExtract1'
    //   MATLAB Function: '<S187>/FaultParamsExtract2'
    //   MATLAB Function: '<S187>/FaultParamsExtract3'

    // '<S190>:1:29' hasFault_isloadLeak=hFault;
    // '<S190>:1:30' FaultParam=fParam;
    std::memcpy(&fParamTmp[0], &QuadModelv_DW.fParam_p[0], 20U * sizeof(real_T));
    std::memcpy(&LoadShiftParams[0], &QuadModelv_DW.fParam_f[0], 20U * sizeof
                (real_T));
    std::memcpy(&LoadLeakParams[0], &QuadModelv_DW.fParam_px[0], 20U * sizeof
                (real_T));

    //  This function aims to calculate the variation of mass and moment of
    //  inertia caused by model failure including LoadFall, LoadShift, and
    //  LoadLeak.
    // MATLAB Function 'Fail Model/Model Failure': '<S188>:1'
    // '<S188>:1:7' if isempty(tOld)
    // '<S188>:1:11' deltaM=[0;0;0];
    QuadModelv_B.deltaM[0] = 0.0;
    QuadModelv_B.deltaM[1] = 0.0;
    QuadModelv_B.deltaM[2] = 0.0;

    // '<S188>:1:13' if isLoadFall
    if (QuadModelv_DW.hFault_nj) {
      // '<S188>:1:14' if LoadFallParams(1)>0.95
      if (QuadModelv_DW.fParam_p[0] > 0.95) {
        // '<S188>:1:15' LoadFallParams(1)=0.95;
        fParamTmp[0] = 0.95;
      } else if (QuadModelv_DW.fParam_p[0] < 0.0) {
        // '<S188>:1:16' elseif LoadFallParams(1)<0
        // '<S188>:1:17' LoadFallParams(1)=0;
        fParamTmp[0] = 0.0;
      }

      // '<S188>:1:19' uavlostMass=LoadFallParams(1)*uavMass;
      QuadModelv_B.uavlostMass = fParamTmp[0] * QuadModelv_P.ModelParam_uavMass;

      // '<S188>:1:20' uavlostJ=zeros(3,3);
      std::memset(&rtb_uavlostJ[0], 0, 9U * sizeof(real_T));
    } else if (QuadModelv_DW.hFault_j) {
      // '<S188>:1:21' elseif isLoadShift
      // '<S188>:1:22' if LoadShiftParams(1)>0.95
      if (QuadModelv_DW.fParam_f[0] > 0.95) {
        // '<S188>:1:23' LoadShiftParams(1)=0.95;
        LoadShiftParams[0] = 0.95;
      } else if (QuadModelv_DW.fParam_f[0] < 0.0) {
        // '<S188>:1:24' elseif LoadShiftParams(1)<0
        // '<S188>:1:25' LoadShiftParams(1)=0;
        LoadShiftParams[0] = 0.0;
      }

      // '<S188>:1:28' uavlostMass= 0;
      QuadModelv_B.uavlostMass = 0.0;

      // '<S188>:1:29' loadMoveMass= LoadShiftParams(1)*uavMass;
      LoadLeakRatio = LoadShiftParams[0] * QuadModelv_P.ModelParam_uavMass;

      // '<S188>:1:31' X=uavR*LoadShiftParams(2);
      rtb_Saturation_1 = QuadModelv_P.ModelParam_uavR * QuadModelv_DW.fParam_f[1];

      // '<S188>:1:32' Y=uavR*LoadShiftParams(3);
      Y = QuadModelv_P.ModelParam_uavR * QuadModelv_DW.fParam_f[2];

      // '<S188>:1:33' Z=uavR*LoadShiftParams(4);
      Z = QuadModelv_P.ModelParam_uavR * QuadModelv_DW.fParam_f[3];

      // '<S188>:1:35' uavlostJxx= loadMoveMass*(Y^2+Z^2);
      // '<S188>:1:36' uavlostJyy= loadMoveMass*(X^2+Z^2);
      // '<S188>:1:37' uavlostJzz= loadMoveMass*(X^2+Y^2);
      // '<S188>:1:38' uavlostJyz= loadMoveMass*Y*Z;
      rtb_TmpSignalConversionAtSFun_2 = LoadLeakRatio * Y * Z;

      // '<S188>:1:39' uavlostJzx= loadMoveMass*Z*X;
      uavlostJzx = LoadLeakRatio * Z * rtb_Saturation_1;

      // '<S188>:1:40' uavlostJxy= loadMoveMass*X*Y;
      rtb_Product3_o = LoadLeakRatio * rtb_Saturation_1 * Y;

      // '<S188>:1:41' uavlostJ=-[uavlostJxx,-uavlostJxy,-uavlostJzx;-uavlostJxy,uavlostJyy,-uavlostJyz;-uavlostJzx,-uavlostJyz,uavlostJzz]; 
      q0_tmp = Z * Z;
      VectorConcatenate_tmp_0 = Y * Y;
      rtb_uavlostJ[0] = -((VectorConcatenate_tmp_0 + q0_tmp) * LoadLeakRatio);
      rtb_uavlostJ[3] = rtb_Product3_o;
      rtb_uavlostJ[6] = uavlostJzx;
      rtb_uavlostJ[1] = rtb_Product3_o;
      rtb_Product3_o = rtb_Saturation_1 * rtb_Saturation_1;
      rtb_uavlostJ[4] = -((rtb_Product3_o + q0_tmp) * LoadLeakRatio);
      rtb_uavlostJ[7] = rtb_TmpSignalConversionAtSFun_2;
      rtb_uavlostJ[2] = uavlostJzx;
      rtb_uavlostJ[5] = rtb_TmpSignalConversionAtSFun_2;
      rtb_uavlostJ[8] = -((rtb_Product3_o + VectorConcatenate_tmp_0) *
                          LoadLeakRatio);

      // '<S188>:1:43' deltaF = dcm*loadMoveMass*[0;0;9.8];
      for (i = 0; i < 3; i++) {
        rtb_sincos_o1_f[i] = (VectorConcatenate[i + 3] * LoadLeakRatio * 0.0 +
                              VectorConcatenate[i] * LoadLeakRatio * 0.0) +
          VectorConcatenate[i + 6] * LoadLeakRatio * 9.8;
      }

      // '<S188>:1:45' deltaM = cross([X;Y;Z],deltaF);
      QuadModelv_B.deltaM[0] = Y * rtb_sincos_o1_f[2] - rtb_sincos_o1_f[1] * Z;
      QuadModelv_B.deltaM[1] = rtb_sincos_o1_f[0] * Z - rtb_Saturation_1 *
        rtb_sincos_o1_f[2];
      QuadModelv_B.deltaM[2] = rtb_Saturation_1 * rtb_sincos_o1_f[1] -
        rtb_sincos_o1_f[0] * Y;

      //  变化的力矩
    } else if (QuadModelv_DW.hFault_no) {
      // '<S188>:1:47' elseif isLoadLeak
      // '<S188>:1:48' uavlostJ=zeros(3,3);
      std::memset(&rtb_uavlostJ[0], 0, 9U * sizeof(real_T));

      // '<S188>:1:49' if LoadLeakParams(1)>0.99
      if (QuadModelv_DW.fParam_px[0] > 0.99) {
        // '<S188>:1:50' LoadLeakParams(1)=0.99;
        LoadLeakParams[0] = 0.99;
      } else if (QuadModelv_DW.fParam_px[0] < 0.0) {
        // '<S188>:1:51' elseif LoadLeakParams(1)<0
        // '<S188>:1:52' LoadLeakParams(1)=0;
        LoadLeakParams[0] = 0.0;
      }

      // '<S188>:1:54' loadmass0=LoadLeakParams(1)*uavMass;
      // '<S188>:1:56' if tOld<0.01
      if (QuadModelv_DW.tOld < 0.01) {
        // '<S188>:1:57' tOld=T;
        QuadModelv_DW.tOld = FaultNum;
      }

      // '<S188>:1:60' dt=T-tOld;
      // '<S188>:1:62' LoadLeakRatio = LoadLeakParams(2)*dt;
      LoadLeakRatio = (FaultNum - QuadModelv_DW.tOld) * QuadModelv_DW.fParam_px
        [1];

      // '<S188>:1:63' if LoadLeakRatio > 0.99
      if (LoadLeakRatio > 0.99) {
        // '<S188>:1:64' LoadLeakRatio = 0.99;
        LoadLeakRatio = 0.99;
      }

      // '<S188>:1:66' uavlostMass = LoadLeakRatio*loadmass0;
      QuadModelv_B.uavlostMass = LoadLeakParams[0] *
        QuadModelv_P.ModelParam_uavMass * LoadLeakRatio;
    } else {
      // '<S188>:1:68' else
      // '<S188>:1:69' uavlostMass=0;
      QuadModelv_B.uavlostMass = 0.0;

      // '<S188>:1:70' uavlostJ=zeros(3,3);
      std::memset(&rtb_uavlostJ[0], 0, 9U * sizeof(real_T));

      // '<S188>:1:71' tOld=0;
      QuadModelv_DW.tOld = 0.0;
    }

    // '<S188>:1:74' if ~isLoadLeak
    if (!QuadModelv_DW.hFault_no) {
      // reset
      // '<S188>:1:75' tOld=0;
      QuadModelv_DW.tOld = 0.0;
    }

    // End of MATLAB Function: '<S6>/Model Failure'

    // Sum: '<S6>/Add5' incorporates:
    //   Constant: '<S6>/ModelParam.uavMass'

    QuadModelv_B.m = QuadModelv_P.ModelParam_uavMass - QuadModelv_B.uavlostMass;

    // UnitConversion: '<S85>/Unit Conversion' incorporates:
    //   Constant: '<S63>/ref_rotation'

    // Unit Conversion - from: deg to: rad
    // Expression: output = (0.0174533*input) + (0)
    rtb_Rn = 0.017453292519943295 * QuadModelv_P.FlatEarthtoLLA_psi;

    // Trigonometry: '<S70>/SinCos'
    QuadModelv_B.SinCos_o1 = std::sin(rtb_Rn);

    // Trigonometry: '<S70>/SinCos'
    QuadModelv_B.SinCos_o2 = std::cos(rtb_Rn);

    // Sum: '<S88>/Sum' incorporates:
    //   Constant: '<S88>/Constant'
    //   Constant: '<S88>/f'

    rtb_Rn = QuadModelv_P.f_Value - QuadModelv_P.Constant_Value_c;

    // Sqrt: '<S89>/sqrt' incorporates:
    //   Constant: '<S89>/Constant'
    //   Product: '<S89>/Product1'
    //   Sum: '<S89>/Sum1'

    rtb_Rn = std::sqrt(QuadModelv_P.Constant_Value_n5 - rtb_Rn * rtb_Rn);

    // Switch: '<S81>/Switch' incorporates:
    //   Abs: '<S81>/Abs'
    //   Bias: '<S81>/Bias'
    //   Bias: '<S81>/Bias1'
    //   Constant: '<S63>/ref_position'
    //   Constant: '<S81>/Constant2'
    //   Constant: '<S82>/Constant'
    //   Math: '<S81>/Math Function1'
    //   RelationalOperator: '<S82>/Compare'

    if (std::abs(QuadModelv_P.ModelParam_GPSLatLong[0]) >
        QuadModelv_P.CompareToConstant_const) {
      LoadLeakRatio = rt_modd_snf(QuadModelv_P.ModelParam_GPSLatLong[0] +
        QuadModelv_P.Bias_Bias_m, QuadModelv_P.Constant2_Value) +
        QuadModelv_P.Bias1_Bias_h;
    } else {
      LoadLeakRatio = QuadModelv_P.ModelParam_GPSLatLong[0];
    }

    // End of Switch: '<S81>/Switch'

    // Abs: '<S78>/Abs1'
    rtb_Saturation_1 = std::abs(LoadLeakRatio);

    // RelationalOperator: '<S80>/Compare' incorporates:
    //   Constant: '<S80>/Constant'

    rtb_Compare_b5 = (rtb_Saturation_1 > QuadModelv_P.CompareToConstant_const_i);

    // Switch: '<S78>/Switch'
    if (rtb_Compare_b5) {
      // Signum: '<S78>/Sign1'
      if (!rtIsNaN(LoadLeakRatio)) {
        if (LoadLeakRatio < 0.0) {
          LoadLeakRatio = -1.0;
        } else {
          LoadLeakRatio = (LoadLeakRatio > 0.0);
        }
      }

      // End of Signum: '<S78>/Sign1'

      // Switch: '<S78>/Switch' incorporates:
      //   Bias: '<S78>/Bias'
      //   Bias: '<S78>/Bias1'
      //   Gain: '<S78>/Gain'
      //   Product: '<S78>/Divide1'

      QuadModelv_B.Switch = ((rtb_Saturation_1 + QuadModelv_P.Bias_Bias_g) *
        QuadModelv_P.Gain_Gain_m + QuadModelv_P.Bias1_Bias_g) * LoadLeakRatio;
    } else {
      // Switch: '<S78>/Switch'
      QuadModelv_B.Switch = LoadLeakRatio;
    }

    // End of Switch: '<S78>/Switch'

    // UnitConversion: '<S86>/Unit Conversion'
    // Unit Conversion - from: deg to: rad
    // Expression: output = (0.0174533*input) + (0)
    rtb_Sum1_c = 0.017453292519943295 * QuadModelv_B.Switch;

    // Trigonometry: '<S87>/Trigonometric Function1'
    uavlostJzx = std::sin(rtb_Sum1_c);

    // Product: '<S87>/Product1' incorporates:
    //   Product: '<S84>/Product2'

    rtb_TmpSignalConversionAtSFun_2 = rtb_Rn * rtb_Rn;

    // Sum: '<S87>/Sum1' incorporates:
    //   Constant: '<S87>/Constant'
    //   Product: '<S87>/Product1'

    uavlostJzx = QuadModelv_P.Constant_Value_kx -
      rtb_TmpSignalConversionAtSFun_2 * uavlostJzx * uavlostJzx;

    // Product: '<S84>/Product1' incorporates:
    //   Constant: '<S84>/Constant1'
    //   Sqrt: '<S84>/sqrt'

    Z = QuadModelv_P.Constant1_Value_d / std::sqrt(uavlostJzx);

    // Trigonometry: '<S84>/Trigonometric Function1' incorporates:
    //   Constant: '<S84>/Constant'
    //   Constant: '<S84>/Constant2'
    //   Product: '<S84>/Product3'
    //   Sum: '<S84>/Sum1'

    QuadModelv_B.TrigonometricFunction1 = rt_atan2d_snf
      (QuadModelv_P.Constant2_Value_mm, (QuadModelv_P.Constant_Value_lo -
        rtb_TmpSignalConversionAtSFun_2) * Z / uavlostJzx);

    // Trigonometry: '<S84>/Trigonometric Function2' incorporates:
    //   Constant: '<S84>/Constant3'
    //   Product: '<S84>/Product4'
    //   Trigonometry: '<S84>/Trigonometric Function'

    QuadModelv_B.TrigonometricFunction2 = rt_atan2d_snf
      (QuadModelv_P.Constant3_Value, Z * std::cos(rtb_Sum1_c));

    // Switch: '<S69>/Switch1' incorporates:
    //   Constant: '<S69>/Constant'
    //   Constant: '<S69>/Constant1'

    if (rtb_Compare_b5) {
      rtb_TmpSignalConversionAtSFun_0 = QuadModelv_P.Constant_Value_n;
    } else {
      rtb_TmpSignalConversionAtSFun_0 = QuadModelv_P.Constant1_Value;
    }

    // End of Switch: '<S69>/Switch1'

    // Sum: '<S69>/Sum' incorporates:
    //   Constant: '<S63>/ref_position'

    LoadLeakRatio = rtb_TmpSignalConversionAtSFun_0 +
      QuadModelv_P.ModelParam_GPSLatLong[1];

    // Switch: '<S79>/Switch' incorporates:
    //   Abs: '<S79>/Abs'
    //   Constant: '<S83>/Constant'
    //   RelationalOperator: '<S83>/Compare'

    if (std::abs(LoadLeakRatio) > QuadModelv_P.CompareToConstant_const_j) {
      // Switch: '<S79>/Switch' incorporates:
      //   Bias: '<S79>/Bias'
      //   Bias: '<S79>/Bias1'
      //   Constant: '<S79>/Constant2'
      //   Math: '<S79>/Math Function1'

      QuadModelv_B.Switch_a = rt_modd_snf(LoadLeakRatio +
        QuadModelv_P.Bias_Bias_c, QuadModelv_P.Constant2_Value_b) +
        QuadModelv_P.Bias1_Bias_n;
    } else {
      // Switch: '<S79>/Switch'
      QuadModelv_B.Switch_a = LoadLeakRatio;
    }

    // End of Switch: '<S79>/Switch'
  }

  // Sum: '<S63>/Sum' incorporates:
  //   Integrator: '<S14>/xe,ye,ze'
  //   Product: '<S70>/rad lat'
  //   Product: '<S70>/x*cos'
  //   Product: '<S70>/y*sin'
  //   Sum: '<S70>/Sum'
  //   UnitConversion: '<S71>/Unit Conversion'

  // Unit Conversion - from: rad to: deg
  // Expression: output = (57.2958*input) + (0)
  rtb_Sum_d[0] = (QuadModelv_X.xeyeze_CSTATE[0] * QuadModelv_B.SinCos_o2 -
                  QuadModelv_X.xeyeze_CSTATE[1] * QuadModelv_B.SinCos_o1) *
    QuadModelv_B.TrigonometricFunction1 * 57.295779513082323 +
    QuadModelv_B.Switch;

  // Switch: '<S75>/Switch' incorporates:
  //   Abs: '<S75>/Abs'
  //   Bias: '<S75>/Bias'
  //   Bias: '<S75>/Bias1'
  //   Constant: '<S75>/Constant2'
  //   Constant: '<S76>/Constant'
  //   Math: '<S75>/Math Function1'
  //   RelationalOperator: '<S76>/Compare'

  if (std::abs(rtb_Sum_d[0]) > QuadModelv_P.CompareToConstant_const_ji) {
    LoadLeakRatio = rt_modd_snf(rtb_Sum_d[0] + QuadModelv_P.Bias_Bias_i,
      QuadModelv_P.Constant2_Value_n) + QuadModelv_P.Bias1_Bias_o;
  } else {
    LoadLeakRatio = rtb_Sum_d[0];
  }

  // End of Switch: '<S75>/Switch'

  // Abs: '<S72>/Abs1'
  rtb_Saturation_1 = std::abs(LoadLeakRatio);

  // Switch: '<S72>/Switch' incorporates:
  //   Bias: '<S72>/Bias'
  //   Bias: '<S72>/Bias1'
  //   Constant: '<S68>/Constant'
  //   Constant: '<S68>/Constant1'
  //   Constant: '<S74>/Constant'
  //   Gain: '<S72>/Gain'
  //   Product: '<S72>/Divide1'
  //   RelationalOperator: '<S74>/Compare'
  //   Switch: '<S68>/Switch1'

  if (rtb_Saturation_1 > QuadModelv_P.CompareToConstant_const_k) {
    // Signum: '<S72>/Sign1'
    if (!rtIsNaN(LoadLeakRatio)) {
      if (LoadLeakRatio < 0.0) {
        LoadLeakRatio = -1.0;
      } else {
        LoadLeakRatio = (LoadLeakRatio > 0.0);
      }
    }

    // End of Signum: '<S72>/Sign1'
    LoadLeakRatio *= (rtb_Saturation_1 + QuadModelv_P.Bias_Bias) *
      QuadModelv_P.Gain_Gain + QuadModelv_P.Bias1_Bias;
    rtb_Rn = QuadModelv_P.Constant_Value_gw;
  } else {
    rtb_Rn = QuadModelv_P.Constant1_Value_nq;
  }

  // End of Switch: '<S72>/Switch'

  // Sum: '<S68>/Sum' incorporates:
  //   Integrator: '<S14>/xe,ye,ze'
  //   Product: '<S70>/rad long '
  //   Product: '<S70>/x*sin'
  //   Product: '<S70>/y*cos'
  //   Sum: '<S63>/Sum'
  //   Sum: '<S70>/Sum1'
  //   UnitConversion: '<S71>/Unit Conversion'

  Y = ((QuadModelv_X.xeyeze_CSTATE[0] * QuadModelv_B.SinCos_o1 +
        QuadModelv_X.xeyeze_CSTATE[1] * QuadModelv_B.SinCos_o2) *
       QuadModelv_B.TrigonometricFunction2 * 57.295779513082323 +
       QuadModelv_B.Switch_a) + rtb_Rn;

  // Switch: '<S73>/Switch' incorporates:
  //   Abs: '<S73>/Abs'
  //   Bias: '<S73>/Bias'
  //   Bias: '<S73>/Bias1'
  //   Constant: '<S73>/Constant2'
  //   Constant: '<S77>/Constant'
  //   Math: '<S73>/Math Function1'
  //   RelationalOperator: '<S77>/Compare'

  if (std::abs(Y) > QuadModelv_P.CompareToConstant_const_f) {
    Y = rt_modd_snf(Y + QuadModelv_P.Bias_Bias_l, QuadModelv_P.Constant2_Value_o)
      + QuadModelv_P.Bias1_Bias_c;
  }

  // End of Switch: '<S73>/Switch'

  // Sum: '<S63>/Sum1' incorporates:
  //   Constant: '<S5>/ModelParam.envAltitude'
  //   Integrator: '<S14>/xe,ye,ze'
  //   UnaryMinus: '<S63>/Ze2height'

  rtb_Saturation_1 = -QuadModelv_X.xeyeze_CSTATE[2] -
    QuadModelv_P.ModelParam_envAltitude;

  // Saturate: '<S5>/Saturation_1'
  if (rtb_Saturation_1 > QuadModelv_P.Saturation_1_UpperSat) {
    rtb_Saturation_1 = QuadModelv_P.Saturation_1_UpperSat;
  } else if (rtb_Saturation_1 < QuadModelv_P.Saturation_1_LowerSat) {
    rtb_Saturation_1 = QuadModelv_P.Saturation_1_LowerSat;
  }

  // End of Saturate: '<S5>/Saturation_1'

  // UnitConversion: '<S91>/Unit Conversion' incorporates:
  //   SignalConversion generated from: '<Root>/MavVehile3DInfo'

  // Unit Conversion - from: deg to: rad
  // Expression: output = (0.0174533*input) + (0)
  rtb_Product3_o = 0.017453292519943295 * LoadLeakRatio;

  // UnitConversion: '<S93>/Unit Conversion' incorporates:
  //   SignalConversion generated from: '<Root>/MavVehile3DInfo'

  // Unit Conversion - from: deg to: rad
  // Expression: output = (0.0174533*input) + (0)
  rtb_TmpSignalConversionAtSFun_2 = 0.017453292519943295 * Y;

  // Selector: '<S66>/Selector' incorporates:
  //   SignalConversion generated from: '<Root>/MavVehile3DInfo'

  j = rtb_Saturation_1;

  // S-Function (saerogravity2): '<S66>/WGS84 Gravity S-Function' incorporates:
  //   Constant: '<S66>/Julian Date'


  // S-Function Block: <S66>/WGS84 Gravity S-Function
  {
    int_T i;
    real_T GM, opt_m2ft, deg2rad;
    real_T *phi_ptr, *height_ptr;
    boolean_T phi_wrapped = false;
    real_T *phi = (real_T *) &QuadModelv_DW.WGS84GravitySFunction_phi;
    real_T *h = (real_T *) &QuadModelv_DW.WGS84GravitySFunction_h;
    height_ptr = (real_T *) &j;
    phi_ptr = (real_T *) &rtb_Product3_o;

    // get unit conversion factor
    opt_m2ft = 1.0;
    deg2rad = 1.0;

    // Use Earth's Atmosphere in Gravitational Const?
    GM = ( 1.0 == 0 ) ? WGS84_GM_DEF : WGS84_GM_PRM;

    // WGS84TAYLORSERIES
    {
      for (i = 0; i < 1; i++ ) {
        real_T fphi, pi_2;

        // create short variables for latitude (phi) and height (h)
        phi[i] = phi_ptr[i] * deg2rad;
        h[i] = height_ptr[i] / opt_m2ft;
        if (phi[i] > AERO_PI || phi[i] < -AERO_PI) {
          phi[i] = std::fmod(phi[i]+AERO_PI, 2.0*AERO_PI) - AERO_PI;
        }

        fphi = std::abs(phi[i]);
        pi_2 = AERO_PI/2.0;
        if (fphi > pi_2 ) {
          real_T sign = phi[i]/fphi;
          phi_wrapped = true;
          phi[i] = sign*(pi_2 - (fphi - pi_2));
        } else {
          phi_wrapped = false;
        }
      }

      wgs84_taylor_series(h, phi, opt_m2ft, &QuadModelv_B.MatrixConcatenate[2],
                          1);

      // north and east components are zero
      QuadModelv_B.MatrixConcatenate[0] =
        QuadModelv_DW.WGS84GravitySFunction_gamma_phi;
      QuadModelv_B.MatrixConcatenate[1] =
        QuadModelv_DW.WGS84GravitySFunction_gamma_phi;
    }
  }

  for (i = 0; i < 3; i++) {
    // Product: '<S7>/Product1'
    rtb_sincos_o1_f[i] = QuadModelv_B.m * QuadModelv_B.MatrixConcatenate[i];

    // Product: '<S22>/Product' incorporates:
    //   Concatenate: '<S36>/Vector Concatenate'
    //   Integrator: '<S14>/ub,vb,wb'
    //   Math: '<S14>/Transpose'

    QuadModelv_B.Product[i] = 0.0;
    QuadModelv_B.Product[i] += VectorConcatenate[3 * i] *
      QuadModelv_X.ubvbwb_CSTATE[0];
    QuadModelv_B.Product[i] += VectorConcatenate[3 * i + 1] *
      QuadModelv_X.ubvbwb_CSTATE[1];
    QuadModelv_B.Product[i] += VectorConcatenate[3 * i + 2] *
      QuadModelv_X.ubvbwb_CSTATE[2];
  }

  // Product: '<S47>/Product' incorporates:
  //   DotProduct: '<S26>/Dot Product'
  //   Integrator: '<S15>/q0 q1 q2 q3'

  q0_tmp = QuadModelv_X.q0q1q2q3_CSTATE[0] * QuadModelv_X.q0q1q2q3_CSTATE[0];
  j = q0_tmp;

  // Product: '<S47>/Product1' incorporates:
  //   DotProduct: '<S26>/Dot Product'
  //   Integrator: '<S15>/q0 q1 q2 q3'

  VectorConcatenate_tmp_0 = QuadModelv_X.q0q1q2q3_CSTATE[1] *
    QuadModelv_X.q0q1q2q3_CSTATE[1];

  // Product: '<S47>/Product2' incorporates:
  //   DotProduct: '<S26>/Dot Product'
  //   Integrator: '<S15>/q0 q1 q2 q3'

  VectorConcatenate_tmp_1 = QuadModelv_X.q0q1q2q3_CSTATE[2] *
    QuadModelv_X.q0q1q2q3_CSTATE[2];

  // Product: '<S47>/Product3' incorporates:
  //   DotProduct: '<S26>/Dot Product'
  //   Integrator: '<S15>/q0 q1 q2 q3'

  VectorConcatenate_tmp = QuadModelv_X.q0q1q2q3_CSTATE[3] *
    QuadModelv_X.q0q1q2q3_CSTATE[3];

  // Sqrt: '<S46>/sqrt' incorporates:
  //   Product: '<S47>/Product1'
  //   Product: '<S47>/Product2'
  //   Product: '<S47>/Product3'
  //   Sum: '<S47>/Sum'

  j = std::sqrt(((VectorConcatenate_tmp_0 + j) + VectorConcatenate_tmp_1) +
                VectorConcatenate_tmp);

  // Product: '<S41>/Product' incorporates:
  //   Integrator: '<S15>/q0 q1 q2 q3'

  rtb_q3dot = QuadModelv_X.q0q1q2q3_CSTATE[0] / j;

  // Product: '<S41>/Product1' incorporates:
  //   Integrator: '<S15>/q0 q1 q2 q3'

  rtb_Product3_o = QuadModelv_X.q0q1q2q3_CSTATE[1] / j;

  // Product: '<S41>/Product2' incorporates:
  //   Integrator: '<S15>/q0 q1 q2 q3'

  rtb_TmpSignalConversionAtSFun_2 = QuadModelv_X.q0q1q2q3_CSTATE[2] / j;

  // Product: '<S41>/Product3' incorporates:
  //   Integrator: '<S15>/q0 q1 q2 q3'

  j = QuadModelv_X.q0q1q2q3_CSTATE[3] / j;

  // Fcn: '<S24>/fcn2' incorporates:
  //   Fcn: '<S24>/fcn5'

  rtb_Add_br = rtb_q3dot * rtb_q3dot;
  VectorConcatenate_n_idx_2 = rtb_Product3_o * rtb_Product3_o;
  rtb_UniformRandomNumber7_idx_2 = rtb_TmpSignalConversionAtSFun_2 *
    rtb_TmpSignalConversionAtSFun_2;

  // Trigonometry: '<S40>/Trigonometric Function1' incorporates:
  //   Concatenate: '<S40>/Vector Concatenate'
  //   Fcn: '<S24>/fcn1'
  //   Fcn: '<S24>/fcn2'

  VectorConcatenate_n_idx_0 = rt_atan2d_snf((rtb_Product3_o *
    rtb_TmpSignalConversionAtSFun_2 + rtb_q3dot * j) * 2.0, ((rtb_Add_br +
    VectorConcatenate_n_idx_2) - rtb_UniformRandomNumber7_idx_2) - j * j);

  // Fcn: '<S24>/fcn3'
  rtb_fcn3 = (rtb_Product3_o * j - rtb_q3dot * rtb_TmpSignalConversionAtSFun_2) *
    -2.0;

  // If: '<S42>/If'
  if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if (rtb_fcn3 > 1.0) {
      rtAction = 0;
    } else if (rtb_fcn3 < -1.0) {
      rtAction = 1;
    } else {
      rtAction = 2;
    }

    QuadModelv_DW.If_ActiveSubsystem = rtAction;
  } else {
    rtAction = QuadModelv_DW.If_ActiveSubsystem;
  }

  switch (rtAction) {
   case 0:
    // Outputs for IfAction SubSystem: '<S42>/If Action Subsystem' incorporates:
    //   ActionPort: '<S43>/Action Port'

    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
      // Merge: '<S42>/Merge' incorporates:
      //   Constant: '<S43>/Constant'

      QuadModelv_B.Merge_e2 = QuadModelv_P.Constant_Value;
    }

    // End of Outputs for SubSystem: '<S42>/If Action Subsystem'
    break;

   case 1:
    // Outputs for IfAction SubSystem: '<S42>/If Action Subsystem1' incorporates:
    //   ActionPort: '<S44>/Action Port'

    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
      // Merge: '<S42>/Merge' incorporates:
      //   Constant: '<S44>/Constant'

      QuadModelv_B.Merge_e2 = QuadModelv_P.Constant_Value_l;
    }

    // End of Outputs for SubSystem: '<S42>/If Action Subsystem1'
    break;

   case 2:
    // Outputs for IfAction SubSystem: '<S42>/If Action Subsystem2' incorporates:
    //   ActionPort: '<S45>/Action Port'

    // Merge: '<S42>/Merge' incorporates:
    //   SignalConversion generated from: '<S45>/In'

    QuadModelv_B.Merge_e2 = rtb_fcn3;

    // End of Outputs for SubSystem: '<S42>/If Action Subsystem2'
    break;
  }

  // End of If: '<S42>/If'

  // Trigonometry: '<S40>/trigFcn' incorporates:
  //   Concatenate: '<S40>/Vector Concatenate'

  if (QuadModelv_B.Merge_e2 > 1.0) {
    rtb_TmpSignalConversionAtSFun_0 = 1.0;
  } else if (QuadModelv_B.Merge_e2 < -1.0) {
    rtb_TmpSignalConversionAtSFun_0 = -1.0;
  } else {
    rtb_TmpSignalConversionAtSFun_0 = QuadModelv_B.Merge_e2;
  }

  VectorConcatenate_n_idx_1 = std::asin(rtb_TmpSignalConversionAtSFun_0);

  // End of Trigonometry: '<S40>/trigFcn'

  // Trigonometry: '<S40>/Trigonometric Function3' incorporates:
  //   Concatenate: '<S40>/Vector Concatenate'
  //   Fcn: '<S24>/fcn4'
  //   Fcn: '<S24>/fcn5'

  VectorConcatenate_n_idx_2 = rt_atan2d_snf((rtb_TmpSignalConversionAtSFun_2 * j
    + rtb_q3dot * rtb_Product3_o) * 2.0, ((rtb_Add_br -
    VectorConcatenate_n_idx_2) - rtb_UniformRandomNumber7_idx_2) + j * j);

  // Trigonometry: '<S195>/sincos' incorporates:
  //   DataTypeConversion: '<S8>/Data Type Conversion1'
  //   MATLAB Function: '<S7>/OnGroundFaceup'
  //   SignalConversion generated from: '<S197>/ SFunction '

  // MATLAB Function 'Force and Moment Model/OnGroundFaceup': '<S197>:1'
  // '<S197>:1:3' targetRoll = 0;
  //  if cos(EulerAng(1))<0
  //      targetRoll=pi;
  //  end
  // '<S197>:1:8' targetPitch = 0;
  //  if cos(EulerAng(2))<0
  //          targetPitch=pi;
  //  end
  // '<S197>:1:13' y = [targetRoll;targetPitch;EulerAng(3)];
  rtb_DataTypeConversion2[2] = std::sin(VectorConcatenate_n_idx_0);
  rtb_DataTypeConversion1[2] = std::cos(VectorConcatenate_n_idx_0);

  // Product: '<S277>/u(5)*u(6)' incorporates:
  //   Concatenate: '<S286>/Vector Concatenate'

  VectorConcatenate_p[0] = rtb_DataTypeConversion1[2];

  // Sum: '<S280>/Sum' incorporates:
  //   Concatenate: '<S286>/Vector Concatenate'
  //   Product: '<S280>/u(3)*u(4)'
  //   Product: '<S280>/u(6)*u(1)*u(2)'

  VectorConcatenate_p[1] = 0.0 * rtb_DataTypeConversion1[2] * 0.0 -
    rtb_DataTypeConversion2[2];

  // Sum: '<S283>/Sum' incorporates:
  //   Concatenate: '<S286>/Vector Concatenate'
  //   Product: '<S283>/u(1)*u(3)'
  //   Product: '<S283>/u(2)*u(4)*u(6)'

  VectorConcatenate_p[2] = 0.0 * rtb_DataTypeConversion2[2] + 0.0 *
    rtb_DataTypeConversion1[2];

  // Product: '<S278>/u(3)*u(5)' incorporates:
  //   Concatenate: '<S286>/Vector Concatenate'

  VectorConcatenate_p[3] = rtb_DataTypeConversion2[2];

  // Sum: '<S281>/Sum' incorporates:
  //   Concatenate: '<S286>/Vector Concatenate'
  //   Product: '<S281>/u(1)*u(2)*u(3)'
  //   Product: '<S281>/u(4)*u(6)'

  VectorConcatenate_p[4] = 0.0 * rtb_DataTypeConversion2[2] +
    rtb_DataTypeConversion1[2];

  // Sum: '<S284>/Sum' incorporates:
  //   Concatenate: '<S286>/Vector Concatenate'
  //   Product: '<S284>/u(1)*u(6)'
  //   Product: '<S284>/u(2)*u(3)*u(4)'

  VectorConcatenate_p[5] = 0.0 * rtb_DataTypeConversion2[2] - 0.0 *
    rtb_DataTypeConversion1[2];

  // UnaryMinus: '<S279>/Unary Minus' incorporates:
  //   Concatenate: '<S286>/Vector Concatenate'

  VectorConcatenate_p[6] = -0.0;

  // Product: '<S282>/u(1)*u(5)' incorporates:
  //   Concatenate: '<S286>/Vector Concatenate'

  VectorConcatenate_p[7] = 0.0;

  // Product: '<S285>/u(4)*u(5)' incorporates:
  //   Concatenate: '<S286>/Vector Concatenate'

  VectorConcatenate_p[8] = 1.0;

  // Sum: '<S242>/Add' incorporates:
  //   Product: '<S277>/u(5)*u(6)'

  rtb_Add = rtb_DataTypeConversion1[2] + VectorConcatenate_p[4];

  // If: '<S194>/If' incorporates:
  //   Merge: '<S194>/Merge'
  //   Sum: '<S242>/Add'

  if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    rtAction = static_cast<int8_T>(!(rtb_Add + 1.0 > 0.0));
    QuadModelv_DW.If_ActiveSubsystem_h = rtAction;
  } else {
    rtAction = QuadModelv_DW.If_ActiveSubsystem_h;
  }

  switch (rtAction) {
   case 0:
    // Outputs for IfAction SubSystem: '<S194>/Positive Trace' incorporates:
    //   ActionPort: '<S240>/Action Port'

    QuadModelv_PositiveTrace(rtb_Add + 1.0, VectorConcatenate_p,
      &QuadModelv_B.Merge[0], &QuadModelv_B.Merge[1],
      &QuadModelv_P.PositiveTrace_d);

    // End of Outputs for SubSystem: '<S194>/Positive Trace'
    break;

   case 1:
    // Outputs for IfAction SubSystem: '<S194>/Negative Trace' incorporates:
    //   ActionPort: '<S239>/Action Port'

    QuadModelv_NegativeTrace(VectorConcatenate_p, QuadModelv_B.Merge,
      &QuadModelv_DW.NegativeTrace_d, &QuadModelv_P.NegativeTrace_d);

    // End of Outputs for SubSystem: '<S194>/Negative Trace'
    break;
  }

  // End of If: '<S194>/If'

  // Product: '<S288>/Product3'
  j = QuadModelv_B.Merge[3] * QuadModelv_B.Merge[3];

  // Sum: '<S288>/Sum' incorporates:
  //   Product: '<S288>/Product'
  //   Product: '<S288>/Product1'
  //   Product: '<S288>/Product2'

  rtb_Product3_o = ((QuadModelv_B.Merge[0] * QuadModelv_B.Merge[0] +
                     QuadModelv_B.Merge[1] * QuadModelv_B.Merge[1]) +
                    QuadModelv_B.Merge[2] * QuadModelv_B.Merge[2]) + j;

  // Product: '<S199>/Divide'
  rtb_q2dot = QuadModelv_B.Merge[0] / rtb_Product3_o;

  // Sum: '<S204>/Add' incorporates:
  //   Sum: '<S338>/Add'

  rtb_UniformRandomNumber7_idx_2 = (VectorConcatenate[0] + VectorConcatenate[4])
    + VectorConcatenate[8];

  // If: '<S193>/If' incorporates:
  //   Merge: '<S193>/Merge'
  //   Sum: '<S204>/Add'

  if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    rtAction = static_cast<int8_T>(!(rtb_UniformRandomNumber7_idx_2 > 0.0));
    QuadModelv_DW.If_ActiveSubsystem_l = rtAction;
  } else {
    rtAction = QuadModelv_DW.If_ActiveSubsystem_l;
  }

  switch (rtAction) {
   case 0:
    // Outputs for IfAction SubSystem: '<S193>/Positive Trace' incorporates:
    //   ActionPort: '<S202>/Action Port'

    QuadModelv_PositiveTrace(rtb_UniformRandomNumber7_idx_2, VectorConcatenate,
      &QuadModelv_B.Merge_c[0], &QuadModelv_B.Merge_c[1],
      &QuadModelv_P.PositiveTrace);

    // End of Outputs for SubSystem: '<S193>/Positive Trace'
    break;

   case 1:
    // Outputs for IfAction SubSystem: '<S193>/Negative Trace' incorporates:
    //   ActionPort: '<S201>/Action Port'

    QuadModelv_NegativeTrace(VectorConcatenate, QuadModelv_B.Merge_c,
      &QuadModelv_DW.NegativeTrace, &QuadModelv_P.NegativeTrace);

    // End of Outputs for SubSystem: '<S193>/Negative Trace'
    break;
  }

  // End of If: '<S193>/If'

  // UnaryMinus: '<S287>/Unary Minus'
  j = -QuadModelv_B.Merge[1];

  // Product: '<S199>/Divide1'
  j /= rtb_Product3_o;

  // Product: '<S199>/Divide2' incorporates:
  //   UnaryMinus: '<S287>/Unary Minus1'

  rtb_TmpSignalConversionAtSFun_2 = -QuadModelv_B.Merge[2] / rtb_Product3_o;

  // Product: '<S199>/Divide3' incorporates:
  //   UnaryMinus: '<S287>/Unary Minus2'

  rtb_Rn = -QuadModelv_B.Merge[3] / rtb_Product3_o;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    for (i = 0; i < 20; i++) {
      // DataTypeConversion: '<Root>/Data Type Conversion2' incorporates:
      //   Inport: '<Root>/inFloatsCollision'

      QuadModelv_B.DataTypeConversion2[i] = QuadModelv_U.inFloatsCollision[i];
    }

    // MATLAB Function: '<Root>/CollisionDetection' incorporates:
    //   Concatenate: '<S36>/Vector Concatenate'

    //  vect=[1;1;1;1;1;1;1;1];
    // 是否碰撞标志位
    // MATLAB Function 'CollisionDetection': '<S3>:1'
    // '<S3>:1:6' if isempty(isCol)
    // 输出力和力矩暂存向量
    // '<S3>:1:12' if isempty(fOut)
    // 碰撞冲量矩阵
    // '<S3>:1:18' if isempty(mv0)
    // 碰撞时间
    // '<S3>:1:24' if isempty(tColi)
    // 如果没有发送碰撞
    // '<S3>:1:29' if isCol<0.5
    if ((QuadModelv_DW.isCol < 0.5) && (std::abs
         (QuadModelv_B.DataTypeConversion2[0] - 12345.0) < 1.0) &&
        (QuadModelv_B.DataTypeConversion2[1] > 0.5)) {
      // 如果收到碰撞消息，且被碰撞物体为编号飞机（带CopterID的值大于0）
      // '<S3>:1:31' if abs(uFloats(1)-12345)<1 && uFloats(2)>0.5
      // 如果收到碰撞消息，12345是数据校验位，uFloats(2)是碰撞到飞机的ID
      // '<S3>:1:32' isCol=int8(1);
      QuadModelv_DW.isCol = 1;

      // 设置为碰撞模式
      // 被碰撞物体是活动的还是固定的
      // '<S3>:1:34' if uFloats(3)>0
      if (QuadModelv_B.DataTypeConversion2[2] > 0.0) {
        // 被碰撞物与本物体的质量比有正值，说明碰撞物也是飞机，需要用冲量定理
        // '<S3>:1:35' massOb=mass*uFloats(3)^2;
        rtb_Add_br = QuadModelv_B.DataTypeConversion2[2] *
          QuadModelv_B.DataTypeConversion2[2] * QuadModelv_B.m;

        // '<S3>:1:36' veOb = uFloats(7:9);
        // '<S3>:1:37' veNew=(mass*ve+massOb*veOb)/(mass+massOb);
        rtb_Product3_o = QuadModelv_B.m + rtb_Add_br;
        QuadModelv_DW.mv0[0] = (QuadModelv_B.m * QuadModelv_B.Product[0] +
          rtb_Add_br * QuadModelv_B.DataTypeConversion2[6]) / rtb_Product3_o;
        QuadModelv_DW.mv0[1] = (QuadModelv_B.m * QuadModelv_B.Product[1] +
          rtb_Add_br * QuadModelv_B.DataTypeConversion2[7]) / rtb_Product3_o;
        QuadModelv_DW.mv0[2] = (QuadModelv_B.m * QuadModelv_B.Product[2] +
          rtb_Add_br * QuadModelv_B.DataTypeConversion2[8]) / rtb_Product3_o;
      } else {
        // '<S3>:1:38' else
        // 被碰撞物为树、房子等固定物体，质量无穷大，直接反弹回去（1/10的速度）
        // '<S3>:1:39' veNew=-ve/10.0;
        QuadModelv_DW.mv0[0] = -QuadModelv_B.Product[0] / 10.0;
        QuadModelv_DW.mv0[1] = -QuadModelv_B.Product[1] / 10.0;
        QuadModelv_DW.mv0[2] = -QuadModelv_B.Product[2] / 10.0;
      }

      // '<S3>:1:42' mv0=mass*(veNew-ve);
      QuadModelv_DW.mv0[0] = (QuadModelv_DW.mv0[0] - QuadModelv_B.Product[0]) *
        QuadModelv_B.m;
      QuadModelv_DW.mv0[1] = (QuadModelv_DW.mv0[1] - QuadModelv_B.Product[1]) *
        QuadModelv_B.m;
      QuadModelv_DW.mv0[2] = (QuadModelv_DW.mv0[2] - QuadModelv_B.Product[2]) *
        QuadModelv_B.m;

      // '<S3>:1:43' tColi=t;
      QuadModelv_DW.tColi = FaultNum;

      // 记录碰撞时间
    }

    // 如果已经发生碰撞
    // '<S3>:1:48' if isCol>=0.5
    if (QuadModelv_DW.isCol >= 0.5) {
      // '<S3>:1:49' if (t-tColi)>0.05
      if (FaultNum - QuadModelv_DW.tColi > 0.05) {
        // '<S3>:1:50' fOut=[0;0;0;0;0;0];
        for (i = 0; i < 6; i++) {
          QuadModelv_DW.fOut[i] = 0.0;
        }
      } else {
        // '<S3>:1:51' else
        // 用瞬时的作用力模拟冲量作用（0.05s）
        // '<S3>:1:53' mv=DCM*mv0;
        for (i = 0; i < 3; i++) {
          rtb_DataTypeConversion2[i] = (VectorConcatenate[i + 3] *
            QuadModelv_DW.mv0[1] + VectorConcatenate[i] * QuadModelv_DW.mv0[0])
            + VectorConcatenate[i + 6] * QuadModelv_DW.mv0[2];
        }

        // '<S3>:1:54' mv(1)=mv(1)*(0.7+0.3*rand());
        rtb_DataTypeConversion2[0] *= 0.3 * QuadModelv_rand() + 0.7;

        // '<S3>:1:55' mv(2)=mv(2)*(0.7+0.3*rand());
        rtb_DataTypeConversion2[1] *= 0.3 * QuadModelv_rand() + 0.7;

        // '<S3>:1:56' mv(3)=mv(3)*(0.7+0.3*rand());
        rtb_DataTypeConversion2[2] *= 0.3 * QuadModelv_rand() + 0.7;

        // '<S3>:1:57' fOut(1:3) = mv/0.05;
        QuadModelv_DW.fOut[0] = rtb_DataTypeConversion2[0] / 0.05;
        QuadModelv_DW.fOut[1] = rtb_DataTypeConversion2[1] / 0.05;
        QuadModelv_DW.fOut[2] = rtb_DataTypeConversion2[2] / 0.05;

        // 冲量除以时间得到力
        // '<S3>:1:58' fOut(4:6) = rand(3,1)*mass;
        QuadModelv_rand_k(rtb_DataTypeConversion2);
        QuadModelv_DW.fOut[3] = rtb_DataTypeConversion2[0] * QuadModelv_B.m;
        QuadModelv_DW.fOut[4] = rtb_DataTypeConversion2[1] * QuadModelv_B.m;
        QuadModelv_DW.fOut[5] = rtb_DataTypeConversion2[2] * QuadModelv_B.m;
      }
    }

    // 下面处理飞机与场景固定物体之间的碰撞
    // '<S3>:1:63' fm = fOut;
    // tz分别表示距地面高度和飞机XYZ坐标
    // '<S3>:1:65' tz=[100;0;0;0];
    tz[0] = 100.0;
    tz[3] = 0.0;

    // '<S3>:1:66' ddm=[0;0;0;0;0;0];
    for (i = 0; i < 6; i++) {
      ddm[i] = 0.0;
    }

    // '<S3>:1:67' if abs(uFloats(1)-12345)<1
    if (std::abs(QuadModelv_B.DataTypeConversion2[0] - 12345.0) < 1.0) {
      // 如果收到碰撞消息
      // '<S3>:1:68' kRa=40;
      // '<S3>:1:69' kRaVe=5;
      // '<S3>:1:70' if uFloats(2)<0.5
      if (QuadModelv_B.DataTypeConversion2[1] < 0.5) {
        // 如果碰撞的是场景固定物体
        // 向下照射射线返回长度
        // '<S3>:1:72' tz(1)=uFloats(15);
        tz[0] = QuadModelv_B.DataTypeConversion2[14];

        // 飞机在UE4中的坐标
        // '<S3>:1:74' tz(2:4)=uFloats(4:6);
        tz[3] = QuadModelv_B.DataTypeConversion2[5];
      } else {
        // '<S3>:1:75' else
        // '<S3>:1:76' kRa=20;
      }

      // 前后左右是以地面坐标系的北东地为准
      // 前障碍物碰撞反馈力
      // '<S3>:1:80' if uFloats(10)<0
      if (QuadModelv_B.DataTypeConversion2[9] < 0.0) {
        // 距前墙面距离，小于0说明已经接触
        // '<S3>:1:81' z=uFloats(10);
        // '<S3>:1:82' if z>-0.01
        if (QuadModelv_B.DataTypeConversion2[9] > -0.01) {
          // '<S3>:1:83' ddm(1)=-10*ve(1);
          ddm[0] = -10.0 * QuadModelv_B.Product[0];
        } else if (QuadModelv_B.DataTypeConversion2[9] > -0.04) {
          // '<S3>:1:84' elseif z>-0.04
          // '<S3>:1:85' ddm(1)=200*z-20*ve(1);
          ddm[0] = 200.0 * QuadModelv_B.DataTypeConversion2[9] - 20.0 *
            QuadModelv_B.Product[0];
        } else {
          // '<S3>:1:86' else
          // '<S3>:1:87' ddm(1)=500*z-50*ve(1);
          ddm[0] = 500.0 * QuadModelv_B.DataTypeConversion2[9] - 50.0 *
            QuadModelv_B.Product[0];
        }
      }

      // 后障碍物碰撞反馈力
      // '<S3>:1:91' if uFloats(11)<0
      if (QuadModelv_B.DataTypeConversion2[10] < 0.0) {
        //          ddm(1)=-uFloats(11)*mass*kRa;
        //          if ve(1)<0
        //              ddm(1)=ddm(1)-ve(1)*mass*kRaVe;
        //          end
        // '<S3>:1:96' z=uFloats(11);
        // '<S3>:1:97' if z>-0.01
        if (QuadModelv_B.DataTypeConversion2[10] > -0.01) {
          // '<S3>:1:98' ddm(1)=-10*ve(1);
          ddm[0] = -10.0 * QuadModelv_B.Product[0];
        } else if (QuadModelv_B.DataTypeConversion2[10] > -0.04) {
          // '<S3>:1:99' elseif z>-0.04
          // '<S3>:1:100' ddm(1)=-200*z-20*ve(1);
          ddm[0] = -200.0 * QuadModelv_B.DataTypeConversion2[10] - 20.0 *
            QuadModelv_B.Product[0];
        } else {
          // '<S3>:1:101' else
          // '<S3>:1:102' ddm(1)=-500*z-50*ve(1);
          ddm[0] = -500.0 * QuadModelv_B.DataTypeConversion2[10] - 50.0 *
            QuadModelv_B.Product[0];
        }
      }

      // 左障碍物碰撞反馈力
      // '<S3>:1:107' if uFloats(12)<0
      if (QuadModelv_B.DataTypeConversion2[11] < 0.0) {
        //          ddm(2)=-uFloats(12)*mass*kRa;
        //          if ve(2)<0
        //              ddm(2)=ddm(2)-ve(2)*mass*kRaVe;
        //          end
        // '<S3>:1:112' z=uFloats(12);
        // '<S3>:1:113' if z>-0.01
        if (QuadModelv_B.DataTypeConversion2[11] > -0.01) {
          // '<S3>:1:114' ddm(2)=-10*ve(2);
          ddm[1] = -10.0 * QuadModelv_B.Product[1];
        } else if (QuadModelv_B.DataTypeConversion2[11] > -0.04) {
          // '<S3>:1:115' elseif z>-0.04
          // '<S3>:1:116' ddm(2)=-200*z-20*ve(2);
          ddm[1] = -200.0 * QuadModelv_B.DataTypeConversion2[11] - 20.0 *
            QuadModelv_B.Product[1];
        } else {
          // '<S3>:1:117' else
          // '<S3>:1:118' ddm(2)=-500*z-50*ve(2);
          ddm[1] = -500.0 * QuadModelv_B.DataTypeConversion2[11] - 50.0 *
            QuadModelv_B.Product[1];
        }
      }

      // 右障碍物碰撞反馈力
      // '<S3>:1:122' if uFloats(13)<0
      if (QuadModelv_B.DataTypeConversion2[12] < 0.0) {
        //          ddm(2)=uFloats(13)*mass*kRa;
        //          if ve(2)>0
        //              ddm(2)=ddm(2)-ve(2)*mass*kRaVe;
        //          end
        // '<S3>:1:127' z=uFloats(13);
        // '<S3>:1:128' if z>-0.01
        if (QuadModelv_B.DataTypeConversion2[12] > -0.01) {
          // '<S3>:1:129' ddm(2)=-10*ve(2);
          ddm[1] = -10.0 * QuadModelv_B.Product[1];
        } else if (QuadModelv_B.DataTypeConversion2[12] > -0.04) {
          // '<S3>:1:130' elseif z>-0.04
          // '<S3>:1:131' ddm(2)=200*z-20*ve(2);
          ddm[1] = 200.0 * QuadModelv_B.DataTypeConversion2[12] - 20.0 *
            QuadModelv_B.Product[1];
        } else {
          // '<S3>:1:132' else
          // '<S3>:1:133' ddm(2)=500*z-50*ve(2);
          ddm[1] = 500.0 * QuadModelv_B.DataTypeConversion2[12] - 50.0 *
            QuadModelv_B.Product[1];
        }
      }

      // 上障碍物碰撞反馈力
      // '<S3>:1:138' if uFloats(14)<0
      if (QuadModelv_B.DataTypeConversion2[13] < 0.0) {
        //          ddm(3)=-uFloats(14)*mass*kRa;
        //          if ve(3)<0
        //              ddm(3)=ddm(3)-ve(3)*mass*kRaVe;
        //          end
        // '<S3>:1:144' z=uFloats(14);
        // '<S3>:1:145' if z>-0.01
        if (QuadModelv_B.DataTypeConversion2[13] > -0.01) {
          // '<S3>:1:146' ddm(3)=-10*ve(3);
          ddm[2] = -10.0 * QuadModelv_B.Product[2];
        } else if (QuadModelv_B.DataTypeConversion2[13] > -0.04) {
          // '<S3>:1:147' elseif z>-0.04
          // '<S3>:1:148' ddm(3)=-200*z-20*ve(3);
          ddm[2] = -200.0 * QuadModelv_B.DataTypeConversion2[13] - 20.0 *
            QuadModelv_B.Product[2];
        } else {
          // '<S3>:1:149' else
          // '<S3>:1:150' ddm(3)=-500*z-50*ve(3);
          ddm[2] = -500.0 * QuadModelv_B.DataTypeConversion2[13] - 50.0 *
            QuadModelv_B.Product[2];
        }
      }
    }

    // 力从地面坐标系映射机体坐标系
    // '<S3>:1:155' ddmm=DCM*ddm(1:3);
    // '<S3>:1:156' fm=fm+[ddmm;0;0;0];
    for (i = 0; i < 3; i++) {
      QuadModelv_B.fm[i] = ((VectorConcatenate[i + 3] * ddm[1] +
        VectorConcatenate[i] * ddm[0]) + VectorConcatenate[i + 6] * ddm[2]) +
        QuadModelv_DW.fOut[i];
    }

    QuadModelv_B.fm[3] = QuadModelv_DW.fOut[3];
    QuadModelv_B.fm[4] = QuadModelv_DW.fOut[4];
    QuadModelv_B.fm[5] = QuadModelv_DW.fOut[5];

    // '<S3>:1:158' isCrash=isCol;
    QuadModelv_B.isCrash = QuadModelv_DW.isCol;

    // End of MATLAB Function: '<Root>/CollisionDetection'

    // MATLAB Function: '<Root>/MATLAB Function' incorporates:
    //   Inport: '<Root>/TerrainZ'

    QuadModelv_B.y_k = QuadModelv_U.TerrainZ;

    // MATLAB Function 'MATLAB Function': '<S9>:1'
    // '<S9>:1:2' y=TerrainZ;
    // '<S9>:1:3' if tz(1)<=99
    if (tz[0] <= 99.0) {
      // '<S9>:1:4' y=tz(4)+tz(1);
      QuadModelv_B.y_k = tz[0] + tz[3];
    }

    // End of MATLAB Function: '<Root>/MATLAB Function'

    // MATLAB Function: '<S7>/Ground Model' incorporates:
    //   Integrator: '<S14>/p,q,r '
    //   Integrator: '<S14>/xe,ye,ze'
    //   Product: '<S290>/Product'
    //   Product: '<S290>/Product1'
    //   Product: '<S290>/Product2'
    //   Product: '<S290>/Product3'
    //   Product: '<S291>/Product'
    //   Product: '<S291>/Product1'
    //   Product: '<S291>/Product2'
    //   Product: '<S291>/Product3'
    //   Product: '<S292>/Product'
    //   Product: '<S292>/Product1'
    //   Product: '<S292>/Product2'
    //   Product: '<S292>/Product3'
    //   Sum: '<S290>/Sum'
    //   Sum: '<S291>/Sum'
    //   Sum: '<S292>/Sum'

    //  persistent last;
    //  if isempty(last)
    //      last=TerrainZ;
    //  end
    //  y=TerrainZ;
    //  if tz(1)<10
    //      if tz(1)<0
    //          y=last;
    //      else
    //          y=tz(3)+tz(1);
    //      end
    //  end
    //  This is a ground model that can provides reaction force and reaction torque 
    // MATLAB Function 'Force and Moment Model/Ground Model': '<S196>:1'
    //  persistent z0;
    //  if isempty(z0)
    //       z0=0;
    //  end
    // '<S196>:1:10' if isempty(takeoffFlag)
    // '<S196>:1:15' if isempty(landFlag)
    // '<S196>:1:19' z=Xe(3)-terrainZ;
    rtb_Add_br = QuadModelv_X.xeyeze_CSTATE[2] - QuadModelv_B.y_k;

    // '<S196>:1:21' if (z<-0.3)&&(Ve(3)<-0.1)&&(takeoffFlag<0.5)
    if ((rtb_Add_br < -0.3) && (QuadModelv_B.Product[2] < -0.1) &&
        (QuadModelv_DW.takeoffFlag < 0.5)) {
      // '<S196>:1:22' takeoffFlag=int8(1);
      QuadModelv_DW.takeoffFlag = 1;

      // '<S196>:1:23' landFlag=int8(0);
      QuadModelv_DW.landFlag = 0;
    }

    // '<S196>:1:26' if (z>-0.05)&&(Ve(3)>0.1)&&(landFlag<0.5)
    if ((rtb_Add_br > -0.05) && (QuadModelv_B.Product[2] > 0.1) &&
        (QuadModelv_DW.landFlag < 0.5)) {
      // '<S196>:1:27' landFlag=int8(1);
      QuadModelv_DW.landFlag = 1;

      // '<S196>:1:28' takeoffFlag=int8(0);
      QuadModelv_DW.takeoffFlag = 0;
    }

    // '<S196>:1:31' takeoff = takeoffFlag;
    // '<S196>:1:32' landed = landFlag;
    rtb_landed = QuadModelv_DW.landFlag;

    // '<S196>:1:35' F1=[0;0;0];
    // '<S196>:1:36' M1=[0;0;0];
    QuadModelv_B.F1[0] = 0.0;
    QuadModelv_B.M1[0] = 0.0;
    QuadModelv_B.F1[1] = 0.0;
    QuadModelv_B.M1[1] = 0.0;
    QuadModelv_B.F1[2] = 0.0;
    QuadModelv_B.M1[2] = 0.0;

    // '<S196>:1:38' if z>=0
    if (rtb_Add_br >= 0.0) {
      //      if z>=0.3
      //          if Ve(3)>=5
      //              F1(3)=-mg(3)-1000*Ve(3);
      //          else
      //              F1(3)=-mg(3)-500*z-100*Ve(3);
      //          end
      //      elseif z>=0.1
      //          if Ve(3)>=2
      //              F1(3)=-mg(3)-100*Ve(3);
      //          else
      //              F1(3)=-mg(3)-50*z-100*Ve(3);
      //          end
      //      else
      //           if Ve(3)>=1
      //              F1(3)=-mg(3)-50*Ve(3);
      //          else
      //              F1(3)=-mg(3)-5*z-50*Ve(3);
      //          end
      //      end
      // '<S196>:1:58' if z<0.05
      if (rtb_Add_br < 0.05) {
        // '<S196>:1:59' F1(3)=-mg(3)-10*Ve(3);
        QuadModelv_B.F1[2] = -rtb_sincos_o1_f[2] - 10.0 * QuadModelv_B.Product[2];
      } else if (rtb_Add_br < 0.1) {
        // '<S196>:1:60' elseif z<0.1
        // '<S196>:1:61' F1(3)=-mg(3)-0.5*z-20*Ve(3);
        QuadModelv_B.F1[2] = (-rtb_sincos_o1_f[2] - 0.5 * rtb_Add_br) - 20.0 *
          QuadModelv_B.Product[2];
      } else {
        // '<S196>:1:62' else
        // '<S196>:1:63' F1(3)=-mg(3)-1*z-50*Ve(3);
        QuadModelv_B.F1[2] = (-rtb_sincos_o1_f[2] - rtb_Add_br) - 50.0 *
          QuadModelv_B.Product[2];
      }

      // '<S196>:1:66' F1(1)=-50*Ve(1);
      QuadModelv_B.F1[0] = -50.0 * QuadModelv_B.Product[0];

      // '<S196>:1:67' F1(2)=-50*Ve(2);
      QuadModelv_B.F1[1] = -50.0 * QuadModelv_B.Product[1];

      //      if ~(isnan(Quet(2))||isnan(Quet(3))||isnan(Quet(4)))
      //          M1=-200*[Quet(2);Quet(3);Quet(4)]-10*wb;
      //      end
      // '<S196>:1:73' M1=-20*[Quet(2);Quet(3);Quet(4)]-5*wb;
      // '<S196>:1:74' M1 = M1 * abs(mg(3))*0.0333;
      rtb_Add_br = std::abs(rtb_sincos_o1_f[2]);
      QuadModelv_B.M1[0] = ((((rtb_q2dot * QuadModelv_B.Merge_c[1] + j *
        QuadModelv_B.Merge_c[0]) + rtb_TmpSignalConversionAtSFun_2 *
        QuadModelv_B.Merge_c[3]) - rtb_Rn * QuadModelv_B.Merge_c[2]) * -20.0 -
                            5.0 * QuadModelv_X.pqr_CSTATE[0]) * rtb_Add_br *
        0.0333;
      QuadModelv_B.M1[1] = ((((rtb_q2dot * QuadModelv_B.Merge_c[2] - j *
        QuadModelv_B.Merge_c[3]) + rtb_TmpSignalConversionAtSFun_2 *
        QuadModelv_B.Merge_c[0]) + rtb_Rn * QuadModelv_B.Merge_c[1]) * -20.0 -
                            5.0 * QuadModelv_X.pqr_CSTATE[1]) * rtb_Add_br *
        0.0333;
      QuadModelv_B.M1[2] = ((((rtb_q2dot * QuadModelv_B.Merge_c[3] + j *
        QuadModelv_B.Merge_c[2]) - rtb_TmpSignalConversionAtSFun_2 *
        QuadModelv_B.Merge_c[1]) + rtb_Rn * QuadModelv_B.Merge_c[0]) * -20.0 -
                            5.0 * QuadModelv_X.pqr_CSTATE[2]) * rtb_Add_br *
        0.0333;
    }

    // End of MATLAB Function: '<S7>/Ground Model'
  }

  // DataTypeConversion: '<S8>/Data Type Conversion1' incorporates:
  //   Sum: '<S7>/Add4'

  rtb_DataTypeConversion1[0] = QuadModelv_B.F1[0] + rtb_sincos_o1_f[0];
  rtb_DataTypeConversion1[1] = QuadModelv_B.F1[1] + rtb_sincos_o1_f[1];
  rtb_DataTypeConversion1[2] = QuadModelv_B.F1[2] + rtb_sincos_o1_f[2];

  // Product: '<S7>/Product' incorporates:
  //   Concatenate: '<S36>/Vector Concatenate'

  for (i = 0; i < 3; i++) {
    rtb_DataTypeConversion2[i] = (VectorConcatenate[i + 3] *
      rtb_DataTypeConversion1[1] + VectorConcatenate[i] *
      rtb_DataTypeConversion1[0]) + VectorConcatenate[i + 6] *
      rtb_DataTypeConversion1[2];
  }

  // End of Product: '<S7>/Product'
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // MATLAB Function: '<S522>/FaultParamsExtract' incorporates:
    //   Constant: '<S522>/FaultID'
    //   Inport: '<Root>/inSILInts'

    // MATLAB Function 'PropFault/FaultParamsExtract': '<S525>:1'
    // '<S525>:1:5' if isempty(hFault)
    // '<S525>:1:8' if isempty(fParam)
    // '<S525>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S525>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S525>:1:14' j=1;
    j = 1.0;

    // '<S525>:1:15' for i=1:8
    for (b_t = 0; b_t < 8; b_t++) {
      // '<S525>:1:16' if inInts(i) == FaultID
      if (QuadModelv_U.inSILInts[b_t] == QuadModelv_P.FaultID_Value_d) {
        // '<S525>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S525>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        fParamTmp[static_cast<int32_T>(2.0 * j - 1.0) - 1] = inSILFloats[((b_t +
          1) << 1) - 2];

        // '<S525>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[static_cast<int32_T>(2.0 * j) - 1] = inSILFloats[((b_t + 1) <<
          1) - 1];

        // '<S525>:1:20' j=j+1;
        j++;
      }
    }

    // '<S525>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S525>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault = true;

      // '<S525>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = inSILFloats[16];
      fParamTmp[17] = inSILFloats[17];
      fParamTmp[18] = inSILFloats[18];
      fParamTmp[19] = inSILFloats[19];

      // '<S525>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam[0], &fParamTmp[0], 20U * sizeof(real_T));
    }

    // '<S525>:1:29' hasFault=hFault;
    rtb_hasFault = QuadModelv_DW.hFault;

    // '<S525>:1:30' FaultParam=fParam;
    std::memcpy(&rtb_FaultParam[0], &QuadModelv_DW.fParam[0], 20U * sizeof
                (real_T));

    // End of MATLAB Function: '<S522>/FaultParamsExtract'

    // Constant: '<S296>/ModelInit_RPM'
    QuadModelv_B.ModelInit_RPM = QuadModelv_P.ModelInit_RPM;

    // MATLAB Function: '<S295>/FaultParamsExtract' incorporates:
    //   Constant: '<S295>/FaultID'
    //   Inport: '<Root>/inSILInts'

    // MATLAB Function 'MotorFault/FaultParamsExtract': '<S299>:1'
    // '<S299>:1:5' if isempty(hFault)
    // '<S299>:1:8' if isempty(fParam)
    // '<S299>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S299>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S299>:1:14' j=1;
    j = 1.0;

    // '<S299>:1:15' count = 0;
    rtb_count = 0.0;

    // '<S299>:1:16' for i=1:8
    for (b_t = 0; b_t < 8; b_t++) {
      // '<S299>:1:17' if inInts(i) == FaultID
      if (QuadModelv_U.inSILInts[b_t] == QuadModelv_P.FaultID_Value_n) {
        // '<S299>:1:18' hFaultTmp=true;
        hFaultTmp = true;

        // '<S299>:1:19' fParamTmp(2*j-1)=inFloats(2*i-1);
        rtb_alt = (b_t + 1) << 1;
        fParamTmp[static_cast<int32_T>(2.0 * j - 1.0) - 1] = inSILFloats[rtb_alt
          - 2];

        // '<S299>:1:20' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[static_cast<int32_T>(2.0 * j) - 1] = inSILFloats[rtb_alt - 1];

        // '<S299>:1:21' count = count + 1;
        rtb_count++;

        // '<S299>:1:22' j=j+1;
        j++;
      }
    }

    // '<S299>:1:25' if hFaultTmp
    if (hFaultTmp) {
      // '<S299>:1:26' hFault=hFaultTmp;
      QuadModelv_DW.hFault_m = true;

      // '<S299>:1:27' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = inSILFloats[16];
      fParamTmp[17] = inSILFloats[17];
      fParamTmp[18] = inSILFloats[18];
      fParamTmp[19] = inSILFloats[19];

      // '<S299>:1:28' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_g[0], &fParamTmp[0], 20U * sizeof(real_T));
    }

    // '<S299>:1:31' hasFault=hFault;
    rtb_hasFault_b = QuadModelv_DW.hFault_m;

    // '<S299>:1:32' FaultParam=fParam;
    std::memcpy(&rtb_FaultParam_ak[0], &QuadModelv_DW.fParam_g[0], 20U * sizeof
                (real_T));

    // End of MATLAB Function: '<S295>/FaultParamsExtract'

    // Memory: '<Root>/Memory'
    // MATLAB Function 'Crash_Motor_Fault': '<S4>:1'
    // '<S4>:1:2' y = u;
    std::memcpy(&Memory[0], &QuadModelv_DW.Memory_PreviousInput[0], sizeof
                (real_T) << 3U);

    // MATLAB Function: '<Root>/Crash_Motor_Fault' incorporates:
    //   Memory: '<Root>/Memory'

    std::memcpy(&QuadModelv_B.y_mh[0], &QuadModelv_DW.Memory_PreviousInput[0],
                sizeof(real_T) << 3U);

    // 如果飞机相撞则损坏三个电机
    // '<S4>:1:7' if isCrash>0.5
    if (QuadModelv_B.isCrash > 0.5) {
      // '<S4>:1:8' y(1)=rand()/2.0;
      QuadModelv_B.y_mh[0] = QuadModelv_rand_n() / 2.0;

      // '<S4>:1:9' y(2)=rand()/5.0;
      QuadModelv_B.y_mh[1] = QuadModelv_rand_n() / 5.0;

      // '<S4>:1:10' y(3)=rand()/10.0+u(3)*0.9;
      QuadModelv_B.y_mh[2] = QuadModelv_rand_n() / 10.0 + Memory[2] * 0.9;
    }

    // End of MATLAB Function: '<Root>/Crash_Motor_Fault'

    // MATLAB Function: '<S57>/FaultParamsExtract' incorporates:
    //   Constant: '<S57>/FaultID'
    //   Inport: '<Root>/inSILInts'

    // MATLAB Function 'BatteryFault/FaultParamsExtract': '<S59>:1'
    // '<S59>:1:5' if isempty(hFault)
    // '<S59>:1:8' if isempty(fParam)
    // '<S59>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S59>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S59>:1:14' j=1;
    j = 1.0;

    // '<S59>:1:15' for i=1:8
    for (b_t = 0; b_t < 8; b_t++) {
      // '<S59>:1:16' if inInts(i) == FaultID
      if (QuadModelv_U.inSILInts[b_t] == QuadModelv_P.FaultID_Value_h) {
        // '<S59>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S59>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        fParamTmp[static_cast<int32_T>(2.0 * j - 1.0) - 1] = inSILFloats[((b_t +
          1) << 1) - 2];

        // '<S59>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[static_cast<int32_T>(2.0 * j) - 1] = inSILFloats[((b_t + 1) <<
          1) - 1];

        // '<S59>:1:20' j=j+1;
        j++;
      }
    }

    // '<S59>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S59>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_n5 = true;

      // '<S59>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = inSILFloats[16];
      fParamTmp[17] = inSILFloats[17];
      fParamTmp[18] = inSILFloats[18];
      fParamTmp[19] = inSILFloats[19];

      // '<S59>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_n[0], &fParamTmp[0], 20U * sizeof(real_T));
    }

    // '<S59>:1:29' hasFault_isUseCustomHovTime=hFault;
    QuadModelv_B.hasFault_isUseCustomHovTime = QuadModelv_DW.hFault_n5;

    // '<S59>:1:30' FaultParam=fParam;
    std::memcpy(&QuadModelv_B.FaultParam_e[0], &QuadModelv_DW.fParam_n[0], 20U *
                sizeof(real_T));

    // End of MATLAB Function: '<S57>/FaultParamsExtract'

    // MATLAB Function: '<S57>/FaultParamsExtract1' incorporates:
    //   Constant: '<S57>/FaultID1'
    //   Inport: '<Root>/inSILInts'

    // MATLAB Function 'BatteryFault/FaultParamsExtract1': '<S60>:1'
    // '<S60>:1:5' if isempty(hFault)
    // '<S60>:1:8' if isempty(fParam)
    // '<S60>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S60>:1:13' fParamTmp=zeros(20,1);
    // '<S60>:1:14' j=1;
    // '<S60>:1:15' for i=1:8
    for (b_t = 0; b_t < 8; b_t++) {
      // '<S60>:1:16' if inInts(i) == FaultID
      if (QuadModelv_U.inSILInts[b_t] == QuadModelv_P.FaultID1_Value_h) {
        // '<S60>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S60>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        // '<S60>:1:19' fParamTmp(2*j)=inFloats(2*i);
        // '<S60>:1:20' j=j+1;
      }
    }

    // '<S60>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S60>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_m1 = true;

      // '<S60>:1:25' fParamTmp(17:20) = inFloats(17:20);
      // '<S60>:1:26' fParam=fParamTmp;
    }

    // '<S60>:1:29' hasFault_isPowOff=hFault;
    QuadModelv_B.hasFault_isPowOff = QuadModelv_DW.hFault_m1;

    // End of MATLAB Function: '<S57>/FaultParamsExtract1'

    // MATLAB Function: '<S57>/FaultParamsExtract2' incorporates:
    //   Constant: '<S57>/FaultID2'
    //   Inport: '<Root>/inSILInts'

    // '<S60>:1:30' FaultParam=fParam;
    // MATLAB Function 'BatteryFault/FaultParamsExtract2': '<S61>:1'
    // '<S61>:1:5' if isempty(hFault)
    // '<S61>:1:8' if isempty(fParam)
    // '<S61>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S61>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S61>:1:14' j=1;
    j = 1.0;

    // '<S61>:1:15' for i=1:8
    for (b_t = 0; b_t < 8; b_t++) {
      // '<S61>:1:16' if inInts(i) == FaultID
      if (QuadModelv_U.inSILInts[b_t] == QuadModelv_P.FaultID2_Value_m) {
        // '<S61>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S61>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        fParamTmp[static_cast<int32_T>(2.0 * j - 1.0) - 1] = inSILFloats[((b_t +
          1) << 1) - 2];

        // '<S61>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[static_cast<int32_T>(2.0 * j) - 1] = inSILFloats[((b_t + 1) <<
          1) - 1];

        // '<S61>:1:20' j=j+1;
        j++;
      }
    }

    // '<S61>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S61>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_hq = true;

      // '<S61>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = inSILFloats[16];
      fParamTmp[17] = inSILFloats[17];
      fParamTmp[18] = inSILFloats[18];
      fParamTmp[19] = inSILFloats[19];

      // '<S61>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_ft[0], &fParamTmp[0], 20U * sizeof
                  (real_T));
    }

    // '<S61>:1:29' hasFault_isLowVoltage=hFault;
    QuadModelv_B.hasFault_isLowVoltage = QuadModelv_DW.hFault_hq;

    // MATLAB Function: '<S57>/FaultParamsExtract3'
    // '<S61>:1:30' FaultParam=fParam;
    // MATLAB Function 'BatteryFault/FaultParamsExtract3': '<S62>:1'
    // '<S62>:1:5' if isempty(hFault)
    // '<S62>:1:8' if isempty(fParam)
    // '<S62>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S62>:1:13' fParamTmp=zeros(20,1);
    for (i = 0; i < 20; i++) {
      // MATLAB Function: '<S57>/FaultParamsExtract2'
      QuadModelv_B.FaultParam_gu[i] = QuadModelv_DW.fParam_ft[i];

      // MATLAB Function: '<S57>/FaultParamsExtract3'
      fParamTmp[i] = 0.0;
    }

    // MATLAB Function: '<S57>/FaultParamsExtract3' incorporates:
    //   Constant: '<S57>/FaultID3'
    //   Inport: '<Root>/inSILInts'

    // '<S62>:1:14' j=1;
    j = 1.0;

    // '<S62>:1:15' for i=1:8
    for (b_t = 0; b_t < 8; b_t++) {
      // '<S62>:1:16' if inInts(i) == FaultID
      if (QuadModelv_U.inSILInts[b_t] == QuadModelv_P.FaultID3_Value) {
        // '<S62>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S62>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        fParamTmp[static_cast<int32_T>(2.0 * j - 1.0) - 1] = inSILFloats[((b_t +
          1) << 1) - 2];

        // '<S62>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[static_cast<int32_T>(2.0 * j) - 1] = inSILFloats[((b_t + 1) <<
          1) - 1];

        // '<S62>:1:20' j=j+1;
        j++;
      }
    }

    // '<S62>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S62>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_h = true;

      // '<S62>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = inSILFloats[16];
      fParamTmp[17] = inSILFloats[17];
      fParamTmp[18] = inSILFloats[18];
      fParamTmp[19] = inSILFloats[19];

      // '<S62>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_m[0], &fParamTmp[0], 20U * sizeof(real_T));
    }

    // '<S62>:1:29' hasFault_islowCapacity=hFault;
    QuadModelv_B.hasFault_islowCapacity = QuadModelv_DW.hFault_h;

    // '<S62>:1:30' FaultParam=fParam;
    std::memcpy(&QuadModelv_B.FaultParam_h[0], &QuadModelv_DW.fParam_m[0], 20U *
                sizeof(real_T));
  }

  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
    // MATLAB Function: '<S2>/MATLAB Function' incorporates:
    //   Constant: '<S2>/ModelParam.BattHoverMinutes'
    //   Constant: '<S2>/ModelParam.BattHoverThr'
    //   Constant: '<S2>/ModelParam.uavMotNumbs'

    rtb_Add = QuadModelv_P.ModelParam_BattHoverMinutes;
    hFaultTmp = QuadModelv_B.hasFault_isUseCustomHovTime;
    std::memcpy(&fParamTmp[0], &QuadModelv_B.FaultParam_gu[0], 20U * sizeof
                (real_T));

    //  battery model
    // ================================Constants================================= 
    // MATLAB Function 'Battery FaultModel/MATLAB Function': '<S58>:1'
    // '<S58>:1:5' delta_t = 0.01;
    // max_current = 5.500;                % maximum current per motor             (A) 
    // aux_current = 0.100;                % current of auxillary components       (A)         
    // capacity = 1.000;                   % LiPo current capacity                 (Ah)                                  
    // cells = 2;                          % LiPo cell count
    // ========================================================================== 
    // '<S58>:1:14' if isempty(discharge)
    // '<S58>:1:19' if isempty(startTime)
    // '<S58>:1:24' if isempty(reset)
    // '<S58>:1:28' discharge_capacity=0;
    rtb_Add_br = 0.0;

    //  低电量故障
    // '<S58>:1:31' if islowCapacity
    if (QuadModelv_B.hasFault_islowCapacity) {
      // '<S58>:1:32' hover_time = hover_time*remainCapacityRatio(1);
      rtb_Add = QuadModelv_P.ModelParam_BattHoverMinutes *
        QuadModelv_B.FaultParam_h[0];

      // '<S58>:1:33' isUseCustomHovTime=false;
      hFaultTmp = false;
    }

    // '<S58>:1:37' if ~isUseCustomHovTime
    if (!hFaultTmp) {
      // '<S58>:1:38' capacity=1;
      // '<S58>:1:40' if hover_time<0.01
      if (rtb_Add < 0.01) {
        // '<S58>:1:41' hover_time=0.01;
        rtb_Add = 0.01;
      }

      // '<S58>:1:44' hover_current=1/(hover_time*1.4/60);
      rtb_Add_br = 1.0 / (rtb_Add * 1.4 / 60.0);

      // '<S58>:1:45' current=0;
      rtb_Add = 0.0;

      // '<S58>:1:46' for i=1:motorNumbs
      for (b_t = 0; b_t < QuadModelv_P.ModelParam_uavMotNumbs; b_t++) {
        // '<S58>:1:47' current=current+hover_current*(load(i)/hover_thr)^2;
        rtb_Rn = QuadModelv_B.y_mh[b_t] / QuadModelv_P.ModelParam_BattHoverThr;
        rtb_Add += rtb_Rn * rtb_Rn * rtb_Add_br;
      }

      // '<S58>:1:49' aux_current=0.5;
      // current = sum(hover_current .* (load/hover_thr).^2) + aux_current;
      // '<S58>:1:51' current =(current+aux_current)/double(motorNumbs);
      // '<S58>:1:52' discharge = current * (delta_t / 3600) + discharge;
      QuadModelv_DW.discharge += (rtb_Add + 0.5) / static_cast<real_T>
        (QuadModelv_P.ModelParam_uavMotNumbs) * 2.7777777777777779E-6;

      // '<S58>:1:53' discharge_capacity = discharge / capacity;
      rtb_Add_br = QuadModelv_DW.discharge;
    }

    // 如果启用电池模型且使用自定义续航时间
    // '<S58>:1:60' if isUseCustomHovTime
    if (hFaultTmp) {
      // '<S58>:1:61' hover_time=CustomHovTime(1)*1.4*60;
      rtb_Add = QuadModelv_B.FaultParam_e[0] * 1.4 * 60.0;

      // '<S58>:1:62' if reset<0.5
      if (QuadModelv_DW.reset < 0.5) {
        // '<S58>:1:63' startTime=time;
        QuadModelv_DW.startTime = FaultNum;

        // '<S58>:1:64' reset=1;
        QuadModelv_DW.reset = 1.0;
      }

      // '<S58>:1:66' if hover_time<0.01
      if (rtb_Add < 0.01) {
        // '<S58>:1:67' hover_time=0.01;
        rtb_Add = 0.01;
      }

      // '<S58>:1:69' discharge_capacity=(time-startTime)/hover_time;
      rtb_Add_br = (FaultNum - QuadModelv_DW.startTime) / rtb_Add;
    } else {
      // '<S58>:1:70' else
      // 使用默认的续航算法
      // '<S58>:1:72' reset=0;
      QuadModelv_DW.reset = 0.0;
    }

    // '<S58>:1:76' if discharge_capacity<0
    if (rtb_Add_br < 0.0) {
      // '<S58>:1:77' discharge_capacity=0;
      rtb_Add_br = 0.0;
    }

    // '<S58>:1:79' if discharge_capacity>1
    if (rtb_Add_br > 1.0) {
      // '<S58>:1:80' discharge_capacity=1;
      rtb_Add_br = 1.0;
    }

    // 电量与电压的曲线
    // '<S58>:1:85' if (0 <= discharge_capacity) && (discharge_capacity <=0.20)
    if ((rtb_Add_br >= 0.0) && (rtb_Add_br <= 0.2)) {
      // '<S58>:1:86' cell_voltage = -14.029*discharge_capacity^3 + 16.975*discharge_capacity^2 - 5.3339*discharge_capacity + 4.2; 
      rtb_Add = ((rtb_Add_br * rtb_Add_br * 16.975 + -14.029 * rt_powd_snf
                  (rtb_Add_br, 3.0)) - 5.3339 * rtb_Add_br) + 4.2;
    } else if ((rtb_Add_br > 0.2) && (rtb_Add_br < 0.7)) {
      // '<S58>:1:87' elseif (0.20 < discharge_capacity) && (discharge_capacity < 0.70) 
      // '<S58>:1:88' cell_voltage = -0.2*discharge_capacity + 3.74;
      rtb_Add = -0.2 * rtb_Add_br + 3.74;
    } else {
      // '<S58>:1:89' else
      // '<S58>:1:90' cell_voltage = -48*discharge_capacity^3 + 89.6*discharge_capacity^2 - 55.08*discharge_capacity + 14.716; 
      rtb_Add = ((rtb_Add_br * rtb_Add_br * 89.6 + -48.0 * rt_powd_snf
                  (rtb_Add_br, 3.0)) - 55.08 * rtb_Add_br) + 14.716;
    }

    // '<S58>:1:93' if cell_voltage < 0.5
    if (rtb_Add < 0.5) {
      // '<S58>:1:94' cell_voltage = 0;
      rtb_Add = 0.0;
    }

    //  电池失效故障
    // '<S58>:1:98' if isPowOff
    if (QuadModelv_B.hasFault_isPowOff) {
      // '<S58>:1:99' cell_voltage = 0;
      rtb_Add = 0.0;
    }

    //  低电压故障
    // '<S58>:1:103' if isLowVoltage
    if (QuadModelv_B.hasFault_isLowVoltage) {
      // '<S58>:1:104' if remainVoltageRatio(1)>1
      if (QuadModelv_B.FaultParam_gu[0] > 1.0) {
        // '<S58>:1:105' remainVoltageRatio(1)=1;
        fParamTmp[0] = 1.0;
      } else if (QuadModelv_B.FaultParam_gu[0] < 0.0) {
        // '<S58>:1:106' elseif remainVoltageRatio(1)<0
        // '<S58>:1:107' remainVoltageRatio(1)=0;
        fParamTmp[0] = 0.0;
      }

      // '<S58>:1:109' cell_voltage = remainVoltageRatio(1)*4.2;
      rtb_Add = fParamTmp[0] * 4.2;
    }

    // 如果电池模型未启用，续航为无穷久，同时各个全局变量归零
    // '<S58>:1:114' if ~isUseCustomHovTime&&~isPowOff&&~isLowVoltage&&~islowCapacity 
    if ((!hFaultTmp) && (!QuadModelv_B.hasFault_isPowOff) &&
        (!QuadModelv_B.hasFault_isLowVoltage) &&
        (!QuadModelv_B.hasFault_islowCapacity)) {
      // '<S58>:1:115' cell_voltage=4.2;
      rtb_Add = 4.2;

      // '<S58>:1:116' discharge = 0;
      QuadModelv_DW.discharge = 0.0;

      // '<S58>:1:117' startTime = 0;
      QuadModelv_DW.startTime = 0.0;

      // '<S58>:1:118' reset = 0;
      QuadModelv_DW.reset = 0.0;
    }

    // 效率矩阵
    // '<S58>:1:122' EffRatio=cell_voltage/4.2;
    // '<S58>:1:123' remCapacity=1-discharge_capacity;
    QuadModelv_B.remCapacity = 1.0 - rtb_Add_br;

    // ========================================================================== 
    QuadModelv_B.cell_voltage = rtb_Add;

    // End of MATLAB Function: '<S2>/MATLAB Function'

    // Gain: '<S2>/MotorFault3'
    QuadModelv_B.MotorFault3 = QuadModelv_P.MotorFault3_Gain *
      QuadModelv_B.cell_voltage;
  }

  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Product: '<S2>/Product'
    for (i = 0; i < 8; i++) {
      rtb_PWMOut[i] = QuadModelv_B.y_mh[i] * QuadModelv_B.MotorFault3;
    }

    // End of Product: '<S2>/Product'
  }

  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[4] == 0) {
    // Gain: '<S295>/Gain' incorporates:
    //   Gain: '<S297>/Output'
    //   RandomNumber: '<S297>/White Noise'

    QuadModelv_B.Gain = std::sqrt(QuadModelv_P.BandLimitedWhiteNoise_Cov) /
      0.31622776601683794 * QuadModelv_DW.NextOutput *
      QuadModelv_P.FaultParamAPI.FaultInParams[0];
  }

  // RelationalOperator: '<S298>/Gust Start' incorporates:
  //   Constant: '<S298>/Gust start time'

  rtb_Rn = (FaultNum >= QuadModelv_P.FaultParamAPI.FaultInParams[2]);
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // SignalConversion generated from: '<S301>/Enable' incorporates:
    //   Constant: '<S298>/Constant'
    //   Logic: '<S298>/Logical Operator2'

    QuadModelv_B.HiddenBuf_InsertedFor_Distancei = ((rtb_Rn != 0.0) &&
      QuadModelv_P.DiscreteWindGustModel_Gx);
  }

  // Outputs for Enabled SubSystem: '<S298>/Distance into gust (x)'
  // Outputs for Enabled SubSystem: '<S298>/Distance into gust (x)'
  QuadModelv_Distanceintogustx(QuadModelv_B.HiddenBuf_InsertedFor_Distancei,
    QuadModelv_P.Distanceintogustx_d_m, &QuadModelv_B.Distanceintogustx,
    &QuadModelv_DW.Distanceintogustx, &QuadModelv_P.Distanceintogustx,
    &QuadModelv_X.Distanceintogustx);

  // End of Outputs for SubSystem: '<S298>/Distance into gust (x)'
  // End of Outputs for SubSystem: '<S298>/Distance into gust (x)'
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // SignalConversion generated from: '<S302>/Enable' incorporates:
    //   Constant: '<S298>/Constant1'
    //   Logic: '<S298>/Logical Operator1'

    QuadModelv_B.HiddenBuf_InsertedFor_Distanc_h = ((rtb_Rn != 0.0) &&
      QuadModelv_P.DiscreteWindGustModel_Gy);
  }

  // Outputs for Enabled SubSystem: '<S298>/Distance into gust (y)'
  // Outputs for Enabled SubSystem: '<S298>/Distance into gust (y)'
  QuadModelv_Distanceintogusty(QuadModelv_B.HiddenBuf_InsertedFor_Distanc_h,
    QuadModelv_P.Distanceintogusty_d_m, &QuadModelv_B.Distanceintogusty,
    &QuadModelv_DW.Distanceintogusty, &QuadModelv_P.Distanceintogusty,
    &QuadModelv_X.Distanceintogusty);

  // End of Outputs for SubSystem: '<S298>/Distance into gust (y)'
  // End of Outputs for SubSystem: '<S298>/Distance into gust (y)'
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // SignalConversion generated from: '<S303>/Enable' incorporates:
    //   Constant: '<S298>/Constant2'
    //   Logic: '<S298>/Logical Operator3'

    QuadModelv_B.HiddenBuf_InsertedFor_Distanc_b = ((rtb_Rn != 0.0) &&
      QuadModelv_P.DiscreteWindGustModel_Gz);
  }

  // Outputs for Enabled SubSystem: '<S298>/Distance into gust (z)'
  // Outputs for Enabled SubSystem: '<S298>/Distance into gust (z)'
  QuadModelv_Distanceintogusty(QuadModelv_B.HiddenBuf_InsertedFor_Distanc_b,
    QuadModelv_P.Distanceintogustz_d_m, &QuadModelv_B.Distanceintogustz,
    &QuadModelv_DW.Distanceintogustz, &QuadModelv_P.Distanceintogustz,
    &QuadModelv_X.Distanceintogustz);

  // End of Outputs for SubSystem: '<S298>/Distance into gust (z)'
  // End of Outputs for SubSystem: '<S298>/Distance into gust (z)'

  // DataTypeConversion: '<S8>/Data Type Conversion1' incorporates:
  //   Constant: '<S298>/2'
  //   Gain: '<S298>/Gust magnitude//2.0'
  //   Gain: '<S298>/pi//Gust length'
  //   Sum: '<S298>/Sum1'
  //   Trigonometry: '<S298>/cos(pi*x//dm)'

  rtb_DataTypeConversion1[0] = 3.1415926535897931 /
    QuadModelv_P.DiscreteWindGustModel_d_m[0] *
    QuadModelv_B.Distanceintogustx.DistanceintoGustxLimitedtogustl;
  rtb_DataTypeConversion1[1] = 3.1415926535897931 /
    QuadModelv_P.DiscreteWindGustModel_d_m[1] *
    QuadModelv_B.Distanceintogusty.DistanceintoGustxLimitedtogustl;
  rtb_DataTypeConversion1[2] = 3.1415926535897931 /
    QuadModelv_P.DiscreteWindGustModel_d_m[2] *
    QuadModelv_B.Distanceintogustz.DistanceintoGustxLimitedtogustl;
  rtb_DataTypeConversion1[0] = QuadModelv_P.DiscreteWindGustModel_v_m[0] / 2.0 *
    (QuadModelv_P.u_Value - std::cos(rtb_DataTypeConversion1[0]));

  // Sum: '<S295>/Sum'
  rtb_Add_br = QuadModelv_B.Gain + rtb_DataTypeConversion1[0];
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // MATLAB Function: '<S295>/MotorFaultModel' incorporates:
    //   Constant: '<S295>/MotorNum'

    rtb_alt = QuadModelv_P.MotorFault1.MotorNum;

    //  本模块需要用到8维FaultParam的输入
    // 因此故障ID的格式应该是：
    // inSILInts=[FaultID FaultID FaultID FaultID ...]
    // inSILInts=[param1 param2 param3 param4 param5 param6 param7 param8 ...] -> FaultParam 
    //  param的每一位表示本电机的健康系数（0到1），实际上就是pwm_out = pwm_in * param 
    //  如果param1=0，则电机1直接坏掉，始终输出0
    // MATLAB Function 'MotorFault/MotorFaultModel': '<S300>:1'
    // '<S300>:1:9' y=inPWMs;
    std::memcpy(&QuadModelv_B.y_d[0], &rtb_PWMOut[0], sizeof(real_T) << 3U);

    // 默认情况下不注入故障，直接将数据输出
    // '<S300>:1:11' if motNum<1
    if (QuadModelv_P.MotorFault1.MotorNum < 1) {
      // '<S300>:1:12' motNum=int32(1);
      rtb_alt = 1;
    }

    // '<S300>:1:15' if motNum>8
    if (rtb_alt > 8) {
      // '<S300>:1:16' motNum=int32(8);
      rtb_alt = 8;
    }

    // '<S300>:1:19' if hasFault
    if (rtb_hasFault_b) {
      // 如果注入了故障
      // '<S300>:1:21' if count == 1
      if (rtb_count == 1.0) {
        // '<S300>:1:22' y(1) = inPWMs(1) * FaultParams(1)+ noise;
        QuadModelv_B.y_d[0] = rtb_PWMOut[0] * rtb_FaultParam_ak[0] + rtb_Add_br;

        // '<S300>:1:23' y(2) = inPWMs(2) * FaultParams(2)+ noise;
        QuadModelv_B.y_d[1] = rtb_PWMOut[1] * rtb_FaultParam_ak[1] + rtb_Add_br;
      } else if (rtb_count >= 2.0) {
        // '<S300>:1:24' elseif count >= 2
        // '<S300>:1:25' for i=1:motNum
        for (r = 0; r < rtb_alt; r++) {
          // '<S300>:1:26' y(i) = inPWMs(i) * FaultParams(i)+ noise;
          QuadModelv_B.y_d[r] = rtb_PWMOut[r] * rtb_FaultParam_ak[r] +
            rtb_Add_br;
        }
      }
    }

    // End of MATLAB Function: '<S295>/MotorFaultModel'
  }

  // Outputs for Atomic SubSystem: '<S296>/MotorNonlinearDynamic1'
  // Constant: '<S296>/ModelParam.motorWb' incorporates:
  //   Constant: '<S296>/ModelParam.motorCr'
  //   Constant: '<S296>/ModelParam.motorT'

  QuadMode_MotorNonlinearDynamic1(QuadModelv_P.ModelParam_motorWb,
    QuadModelv_P.ModelParam_motorCr, QuadModelv_P.ModelParam_motorT,
    QuadModelv_B.ModelInit_RPM, QuadModelv_B.y_d[0],
    &QuadModelv_B.MotorNonlinearDynamic1, &QuadModelv_DW.MotorNonlinearDynamic1,
    &QuadModelv_P.MotorNonlinearDynamic1, &QuadModelv_P,
    &QuadModelv_X.MotorNonlinearDynamic1);

  // End of Outputs for SubSystem: '<S296>/MotorNonlinearDynamic1'

  // Outputs for Atomic SubSystem: '<S296>/MotorNonlinearDynamic2'
  QuadMode_MotorNonlinearDynamic1(QuadModelv_P.ModelParam_motorWb,
    QuadModelv_P.ModelParam_motorCr, QuadModelv_P.ModelParam_motorT,
    QuadModelv_B.ModelInit_RPM, QuadModelv_B.y_d[1],
    &QuadModelv_B.MotorNonlinearDynamic2, &QuadModelv_DW.MotorNonlinearDynamic2,
    &QuadModelv_P.MotorNonlinearDynamic2, &QuadModelv_P,
    &QuadModelv_X.MotorNonlinearDynamic2);

  // End of Outputs for SubSystem: '<S296>/MotorNonlinearDynamic2'

  // Outputs for Atomic SubSystem: '<S296>/MotorNonlinearDynamic3'
  QuadMode_MotorNonlinearDynamic1(QuadModelv_P.ModelParam_motorWb,
    QuadModelv_P.ModelParam_motorCr, QuadModelv_P.ModelParam_motorT,
    QuadModelv_B.ModelInit_RPM, QuadModelv_B.y_d[2],
    &QuadModelv_B.MotorNonlinearDynamic3, &QuadModelv_DW.MotorNonlinearDynamic3,
    &QuadModelv_P.MotorNonlinearDynamic3, &QuadModelv_P,
    &QuadModelv_X.MotorNonlinearDynamic3);

  // End of Outputs for SubSystem: '<S296>/MotorNonlinearDynamic3'

  // Outputs for Atomic SubSystem: '<S296>/MotorNonlinearDynamic4'
  QuadMode_MotorNonlinearDynamic1(QuadModelv_P.ModelParam_motorWb,
    QuadModelv_P.ModelParam_motorCr, QuadModelv_P.ModelParam_motorT,
    QuadModelv_B.ModelInit_RPM, QuadModelv_B.y_d[3],
    &QuadModelv_B.MotorNonlinearDynamic4, &QuadModelv_DW.MotorNonlinearDynamic4,
    &QuadModelv_P.MotorNonlinearDynamic4, &QuadModelv_P,
    &QuadModelv_X.MotorNonlinearDynamic4);

  // End of Outputs for SubSystem: '<S296>/MotorNonlinearDynamic4'

  // Outputs for Atomic SubSystem: '<S296>/MotorNonlinearDynamic5'
  QuadMode_MotorNonlinearDynamic1(QuadModelv_P.ModelParam_motorWb,
    QuadModelv_P.ModelParam_motorCr, QuadModelv_P.ModelParam_motorT,
    QuadModelv_B.ModelInit_RPM, QuadModelv_B.y_d[4],
    &QuadModelv_B.MotorNonlinearDynamic5, &QuadModelv_DW.MotorNonlinearDynamic5,
    &QuadModelv_P.MotorNonlinearDynamic5, &QuadModelv_P,
    &QuadModelv_X.MotorNonlinearDynamic5);

  // End of Outputs for SubSystem: '<S296>/MotorNonlinearDynamic5'

  // Outputs for Atomic SubSystem: '<S296>/MotorNonlinearDynamic6'
  QuadMode_MotorNonlinearDynamic1(QuadModelv_P.ModelParam_motorWb,
    QuadModelv_P.ModelParam_motorCr, QuadModelv_P.ModelParam_motorT,
    QuadModelv_B.ModelInit_RPM, QuadModelv_B.y_d[5],
    &QuadModelv_B.MotorNonlinearDynamic6, &QuadModelv_DW.MotorNonlinearDynamic6,
    &QuadModelv_P.MotorNonlinearDynamic6, &QuadModelv_P,
    &QuadModelv_X.MotorNonlinearDynamic6);

  // End of Outputs for SubSystem: '<S296>/MotorNonlinearDynamic6'

  // Outputs for Atomic SubSystem: '<S296>/MotorNonlinearDynamic7'
  QuadMode_MotorNonlinearDynamic1(QuadModelv_P.ModelParam_motorWb,
    QuadModelv_P.ModelParam_motorCr, QuadModelv_P.ModelParam_motorT,
    QuadModelv_B.ModelInit_RPM, QuadModelv_B.y_d[6],
    &QuadModelv_B.MotorNonlinearDynamic7, &QuadModelv_DW.MotorNonlinearDynamic7,
    &QuadModelv_P.MotorNonlinearDynamic7, &QuadModelv_P,
    &QuadModelv_X.MotorNonlinearDynamic7);

  // End of Outputs for SubSystem: '<S296>/MotorNonlinearDynamic7'

  // Outputs for Atomic SubSystem: '<S296>/MotorNonlinearDynamic8'
  QuadMode_MotorNonlinearDynamic1(QuadModelv_P.ModelParam_motorWb,
    QuadModelv_P.ModelParam_motorCr, QuadModelv_P.ModelParam_motorT,
    QuadModelv_B.ModelInit_RPM, QuadModelv_B.y_d[7],
    &QuadModelv_B.MotorNonlinearDynamic8, &QuadModelv_DW.MotorNonlinearDynamic8,
    &QuadModelv_P.MotorNonlinearDynamic8, &QuadModelv_P,
    &QuadModelv_X.MotorNonlinearDynamic8);

  // End of Outputs for SubSystem: '<S296>/MotorNonlinearDynamic8'
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[4] == 0) {
    // Gain: '<S522>/Gain' incorporates:
    //   Gain: '<S523>/Output'
    //   RandomNumber: '<S523>/White Noise'

    QuadModelv_B.Gain_e = std::sqrt(QuadModelv_P.BandLimitedWhiteNoise_Cov_a) /
      0.31622776601683794 * QuadModelv_DW.NextOutput_g *
      QuadModelv_P.FaultParamAPI.FaultInParams[0];
  }

  // RelationalOperator: '<S524>/Gust Start' incorporates:
  //   Constant: '<S524>/Gust start time'

  rtb_Rn = (FaultNum >= QuadModelv_P.FaultParamAPI.FaultInParams[2]);
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // SignalConversion generated from: '<S527>/Enable' incorporates:
    //   Constant: '<S524>/Constant'
    //   Logic: '<S524>/Logical Operator2'

    QuadModelv_B.HiddenBuf_InsertedFor_Distanc_p = ((rtb_Rn != 0.0) &&
      QuadModelv_P.DiscreteWindGustModel_Gx_p);
  }

  // Outputs for Enabled SubSystem: '<S524>/Distance into gust (x)'
  // Outputs for Enabled SubSystem: '<S524>/Distance into gust (x)'
  QuadModelv_Distanceintogustx(QuadModelv_B.HiddenBuf_InsertedFor_Distanc_p,
    QuadModelv_P.Distanceintogustx_d_m_i, &QuadModelv_B.Distanceintogustx_g,
    &QuadModelv_DW.Distanceintogustx_g, &QuadModelv_P.Distanceintogustx_g,
    &QuadModelv_X.Distanceintogustx_g);

  // End of Outputs for SubSystem: '<S524>/Distance into gust (x)'
  // End of Outputs for SubSystem: '<S524>/Distance into gust (x)'
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // SignalConversion generated from: '<S528>/Enable' incorporates:
    //   Constant: '<S524>/Constant1'
    //   Logic: '<S524>/Logical Operator1'

    QuadModelv_B.HiddenBuf_InsertedFor_Distanc_c = ((rtb_Rn != 0.0) &&
      QuadModelv_P.DiscreteWindGustModel_Gy_e);
  }

  // Outputs for Enabled SubSystem: '<S524>/Distance into gust (y)'
  // Outputs for Enabled SubSystem: '<S524>/Distance into gust (y)'
  QuadModelv_Distanceintogusty(QuadModelv_B.HiddenBuf_InsertedFor_Distanc_c,
    QuadModelv_P.Distanceintogusty_d_m_g, &QuadModelv_B.Distanceintogusty_f,
    &QuadModelv_DW.Distanceintogusty_f, &QuadModelv_P.Distanceintogusty_f,
    &QuadModelv_X.Distanceintogusty_f);

  // End of Outputs for SubSystem: '<S524>/Distance into gust (y)'
  // End of Outputs for SubSystem: '<S524>/Distance into gust (y)'
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // SignalConversion generated from: '<S529>/Enable' incorporates:
    //   Constant: '<S524>/Constant2'
    //   Logic: '<S524>/Logical Operator3'

    QuadModelv_B.HiddenBuf_InsertedFor_Distanc_m = ((rtb_Rn != 0.0) &&
      QuadModelv_P.DiscreteWindGustModel_Gz_o);
  }

  // Outputs for Enabled SubSystem: '<S524>/Distance into gust (z)'
  // Outputs for Enabled SubSystem: '<S524>/Distance into gust (z)'
  QuadModelv_Distanceintogusty(QuadModelv_B.HiddenBuf_InsertedFor_Distanc_m,
    QuadModelv_P.Distanceintogustz_d_m_n, &QuadModelv_B.Distanceintogustz_b,
    &QuadModelv_DW.Distanceintogustz_b, &QuadModelv_P.Distanceintogustz_b,
    &QuadModelv_X.Distanceintogustz_b);

  // End of Outputs for SubSystem: '<S524>/Distance into gust (z)'
  // End of Outputs for SubSystem: '<S524>/Distance into gust (z)'

  // DataTypeConversion: '<S8>/Data Type Conversion1' incorporates:
  //   Constant: '<S524>/2'
  //   Gain: '<S524>/Gust magnitude//2.0'
  //   Gain: '<S524>/pi//Gust length'
  //   Sum: '<S524>/Sum1'
  //   Trigonometry: '<S524>/cos(pi*x//dm)'

  rtb_DataTypeConversion1[0] = 3.1415926535897931 /
    QuadModelv_P.DiscreteWindGustModel_d_m_p[0] *
    QuadModelv_B.Distanceintogustx_g.DistanceintoGustxLimitedtogustl;
  rtb_DataTypeConversion1[1] = 3.1415926535897931 /
    QuadModelv_P.DiscreteWindGustModel_d_m_p[1] *
    QuadModelv_B.Distanceintogusty_f.DistanceintoGustxLimitedtogustl;
  rtb_DataTypeConversion1[2] = 3.1415926535897931 /
    QuadModelv_P.DiscreteWindGustModel_d_m_p[2] *
    QuadModelv_B.Distanceintogustz_b.DistanceintoGustxLimitedtogustl;
  rtb_DataTypeConversion1[0] = QuadModelv_P.DiscreteWindGustModel_v_m_b[0] / 2.0
    * (QuadModelv_P.u_Value_i - std::cos(rtb_DataTypeConversion1[0]));
  rtb_DataTypeConversion1[1] = QuadModelv_P.DiscreteWindGustModel_v_m_b[1] / 2.0
    * (QuadModelv_P.u_Value_i - std::cos(rtb_DataTypeConversion1[1]));
  rtb_DataTypeConversion1[2] = QuadModelv_P.DiscreteWindGustModel_v_m_b[2] / 2.0
    * (QuadModelv_P.u_Value_i - std::cos(rtb_DataTypeConversion1[2]));

  // Sum: '<S522>/Sum'
  rtb_count = QuadModelv_B.Gain_e + rtb_DataTypeConversion1[0];
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[2] == 0) {
    // UniformRandomNumber: '<S67>/Uniform Random Number'
    QuadModelv_B.UniformRandomNumber[0] =
      QuadModelv_DW.UniformRandomNumber_NextOutput[0];
    QuadModelv_B.UniformRandomNumber[1] =
      QuadModelv_DW.UniformRandomNumber_NextOutput[1];
    QuadModelv_B.UniformRandomNumber[2] =
      QuadModelv_DW.UniformRandomNumber_NextOutput[2];
  }

  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // SignalConversion generated from: '<S526>/ SFunction ' incorporates:
    //   MATLAB Function: '<S522>/PropFaultModel'

    rtb_Gain_ok[0] = QuadModelv_B.MotorNonlinearDynamic1.Motor_Dynamics.x;
    rtb_Gain_ok[1] = QuadModelv_B.MotorNonlinearDynamic2.Motor_Dynamics.x;
    rtb_Gain_ok[2] = QuadModelv_B.MotorNonlinearDynamic3.Motor_Dynamics.x;
    rtb_Gain_ok[3] = QuadModelv_B.MotorNonlinearDynamic4.Motor_Dynamics.x;
    rtb_Gain_ok[4] = QuadModelv_B.MotorNonlinearDynamic5.Motor_Dynamics.x;
    rtb_Gain_ok[5] = QuadModelv_B.MotorNonlinearDynamic6.Motor_Dynamics.x;
    rtb_Gain_ok[6] = QuadModelv_B.MotorNonlinearDynamic7.Motor_Dynamics.x;
    rtb_Gain_ok[7] = QuadModelv_B.MotorNonlinearDynamic8.Motor_Dynamics.x;

    // MATLAB Function: '<S522>/PropFaultModel' incorporates:
    //   Constant: '<S522>/FaultID1'

    rtb_alt = QuadModelv_P.PropFault.PropNum;

    //  本模块需要用到8维FaultParam的输入
    // 因此故障ID的格式应该是：
    // inSILInts=[FaultID FaultID FaultID FaultID ...]
    // inSILInts=[param1 param2 param3 param4 param5 param6 param7 param8 ...] -> FaultParam 
    //  param的每一位表示本电机的健康系数（0到1），实际上就是pwm_out = pwm_in * param 
    //  如果param1=0，则电机1直接坏掉，始终输出0
    // MATLAB Function 'PropFault/PropFaultModel': '<S526>:1'
    // '<S526>:1:9' y=inPWMs;
    std::memcpy(&QuadModelv_B.y[0], &rtb_Gain_ok[0], sizeof(real_T) << 3U);

    // 默认情况下不注入故障，直接将数据输出
    // '<S526>:1:11' if PropNum<1
    if (QuadModelv_P.PropFault.PropNum < 1) {
      // '<S526>:1:12' PropNum=int32(1);
      rtb_alt = 1;
    }

    // '<S526>:1:15' if PropNum>8
    if (rtb_alt > 8) {
      // '<S526>:1:16' PropNum=int32(8);
      rtb_alt = 8;
    }

    // '<S526>:1:19' if hasFault
    if (rtb_hasFault) {
      // 如果注入了故障
      // '<S526>:1:20' for i=1:PropNum
      for (r = 0; r < rtb_alt; r++) {
        // '<S526>:1:21' y(i) = inPWMs(i) * FaultParam(i)+ noise;
        QuadModelv_B.y[r] = rtb_Gain_ok[r] * rtb_FaultParam[r] + rtb_count;
      }
    }

    // MATLAB Function: '<S67>/FaultParamsExtract4' incorporates:
    //   Constant: '<S67>/FaultID4'
    //   Inport: '<Root>/inSILInts'

    // MATLAB Function 'WindFault/FaultParamsExtract4': '<S100>:1'
    // '<S100>:1:5' if isempty(hFault)
    // '<S100>:1:8' if isempty(fParam)
    // '<S100>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S100>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S100>:1:14' j=1;
    j = 1.0;

    // '<S100>:1:15' for i=1:8
    for (b_t = 0; b_t < 8; b_t++) {
      // '<S100>:1:16' if inInts(i) == FaultID
      if (QuadModelv_U.inSILInts[b_t] == QuadModelv_P.FaultID4_Value) {
        // '<S100>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S100>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        fParamTmp[static_cast<int32_T>(2.0 * j - 1.0) - 1] = inSILFloats[((b_t +
          1) << 1) - 2];

        // '<S100>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[static_cast<int32_T>(2.0 * j) - 1] = inSILFloats[((b_t + 1) <<
          1) - 1];

        // '<S100>:1:20' j=j+1;
        j++;
      }
    }

    // '<S100>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S100>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_j0 = true;

      // '<S100>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = inSILFloats[16];
      fParamTmp[17] = inSILFloats[17];
      fParamTmp[18] = inSILFloats[18];
      fParamTmp[19] = inSILFloats[19];

      // '<S100>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_o[0], &fParamTmp[0], 20U * sizeof(real_T));
    }

    // '<S100>:1:29' hasFault_WindNoise=hFault;
    // '<S100>:1:30' FaultParam=fParam;
    std::memcpy(&fParamTmp[0], &QuadModelv_DW.fParam_o[0], 20U * sizeof(real_T));

    // MATLAB Function: '<S67>/noiseUpperWindBodySwitch' incorporates:
    //   MATLAB Function: '<S67>/FaultParamsExtract4'

    // MATLAB Function 'WindFault/noiseUpperWindBodySwitch': '<S106>:1'
    // '<S106>:1:3' y = [0;0;0];
    rtb_sincos_o1_f[0] = 0.0;
    rtb_sincos_o1_f[1] = 0.0;
    rtb_sincos_o1_f[2] = 0.0;

    // '<S106>:1:4' if isNoiseFault
    if (QuadModelv_DW.hFault_j0) {
      // '<S106>:1:5' Param_GlobalNoiseGainSwitch=NoiseParams(1);
      // '<S106>:1:6' y=Noise* Param_GlobalNoiseGainSwitch;
      rtb_sincos_o1_f[0] = QuadModelv_B.UniformRandomNumber[0] * fParamTmp[0];
      rtb_sincos_o1_f[1] = fParamTmp[0] * QuadModelv_B.UniformRandomNumber[1];
      rtb_sincos_o1_f[2] = fParamTmp[0] * QuadModelv_B.UniformRandomNumber[2];
    }

    // End of MATLAB Function: '<S67>/noiseUpperWindBodySwitch'

    // Sum: '<S67>/Sum1' incorporates:
    //   Constant: '<S67>/Constant_[1 1 1]'

    QuadModelv_B.Sum1[0] = QuadModelv_P.Constant_111_Value[0] + rtb_sincos_o1_f
      [0];
    QuadModelv_B.Sum1[1] = QuadModelv_P.Constant_111_Value[1] + rtb_sincos_o1_f
      [1];
    QuadModelv_B.Sum1[2] = QuadModelv_P.Constant_111_Value[2] + rtb_sincos_o1_f
      [2];

    // MATLAB Function: '<S67>/FaultParamsExtract' incorporates:
    //   Constant: '<S67>/FaultID'
    //   Inport: '<Root>/inSILInts'

    // MATLAB Function 'WindFault/FaultParamsExtract': '<S96>:1'
    // '<S96>:1:5' if isempty(hFault)
    // '<S96>:1:8' if isempty(fParam)
    // '<S96>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S96>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S96>:1:14' j=1;
    j = 1.0;

    // '<S96>:1:15' for i=1:8
    for (b_t = 0; b_t < 8; b_t++) {
      // '<S96>:1:16' if inInts(i) == FaultID
      if (QuadModelv_U.inSILInts[b_t] == QuadModelv_P.FaultID_Value_a) {
        // '<S96>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S96>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        fParamTmp[static_cast<int32_T>(2.0 * j - 1.0) - 1] = inSILFloats[((b_t +
          1) << 1) - 2];

        // '<S96>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[static_cast<int32_T>(2.0 * j) - 1] = inSILFloats[((b_t + 1) <<
          1) - 1];

        // '<S96>:1:20' j=j+1;
        j++;
      }
    }

    // '<S96>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S96>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_p = true;

      // '<S96>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = inSILFloats[16];
      fParamTmp[17] = inSILFloats[17];
      fParamTmp[18] = inSILFloats[18];
      fParamTmp[19] = inSILFloats[19];

      // '<S96>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_dl[0], &fParamTmp[0], 20U * sizeof
                  (real_T));
    }

    // '<S96>:1:29' hasFault_ConstWind=hFault;
    QuadModelv_B.hasFault_ConstWind = QuadModelv_DW.hFault_p;

    // MATLAB Function: '<S67>/FaultParamsExtract1'
    // '<S96>:1:30' FaultParam=fParam;
    // MATLAB Function 'WindFault/FaultParamsExtract1': '<S97>:1'
    // '<S97>:1:5' if isempty(hFault)
    // '<S97>:1:8' if isempty(fParam)
    // '<S97>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S97>:1:13' fParamTmp=zeros(20,1);
    for (i = 0; i < 20; i++) {
      // MATLAB Function: '<S67>/FaultParamsExtract'
      QuadModelv_B.FaultParam_g[i] = QuadModelv_DW.fParam_dl[i];

      // MATLAB Function: '<S67>/FaultParamsExtract1'
      fParamTmp[i] = 0.0;
    }

    // MATLAB Function: '<S67>/FaultParamsExtract1' incorporates:
    //   Constant: '<S67>/FaultID1'
    //   Inport: '<Root>/inSILInts'

    // '<S97>:1:14' j=1;
    j = 1.0;

    // '<S97>:1:15' for i=1:8
    for (b_t = 0; b_t < 8; b_t++) {
      // '<S97>:1:16' if inInts(i) == FaultID
      if (QuadModelv_U.inSILInts[b_t] == QuadModelv_P.FaultID1_Value_p) {
        // '<S97>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S97>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        i = (b_t + 1) << 1;
        fParamTmp[static_cast<int32_T>(2.0 * j - 1.0) - 1] = inSILFloats[i - 2];

        // '<S97>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[static_cast<int32_T>(2.0 * j) - 1] = inSILFloats[i - 1];

        // '<S97>:1:20' j=j+1;
        j++;
      }
    }

    // '<S97>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S97>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_o = true;

      // '<S97>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = inSILFloats[16];
      fParamTmp[17] = inSILFloats[17];
      fParamTmp[18] = inSILFloats[18];
      fParamTmp[19] = inSILFloats[19];

      // '<S97>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_h[0], &fParamTmp[0], 20U * sizeof(real_T));
    }

    // '<S97>:1:29' hasFault_GustWind=hFault;
    QuadModelv_B.hasFault_GustWind = QuadModelv_DW.hFault_o;

    // MATLAB Function: '<S67>/FaultParamsExtract2'
    // '<S97>:1:30' FaultParam=fParam;
    // MATLAB Function 'WindFault/FaultParamsExtract2': '<S98>:1'
    // '<S98>:1:5' if isempty(hFault)
    // '<S98>:1:8' if isempty(fParam)
    // '<S98>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S98>:1:13' fParamTmp=zeros(20,1);
    for (i = 0; i < 20; i++) {
      // MATLAB Function: '<S67>/FaultParamsExtract1'
      QuadModelv_B.FaultParam_c[i] = QuadModelv_DW.fParam_h[i];

      // MATLAB Function: '<S67>/FaultParamsExtract2'
      fParamTmp[i] = 0.0;
    }

    // MATLAB Function: '<S67>/FaultParamsExtract2' incorporates:
    //   Constant: '<S67>/FaultID2'
    //   Inport: '<Root>/inSILInts'

    // '<S98>:1:14' j=1;
    j = 1.0;

    // '<S98>:1:15' for i=1:8
    for (b_t = 0; b_t < 8; b_t++) {
      // '<S98>:1:16' if inInts(i) == FaultID
      if (QuadModelv_U.inSILInts[b_t] == QuadModelv_P.FaultID2_Value_j) {
        // '<S98>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S98>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        fParamTmp[static_cast<int32_T>(2.0 * j - 1.0) - 1] = inSILFloats[((b_t +
          1) << 1) - 2];

        // '<S98>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[static_cast<int32_T>(2.0 * j) - 1] = inSILFloats[((b_t + 1) <<
          1) - 1];

        // '<S98>:1:20' j=j+1;
        j++;
      }
    }

    // '<S98>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S98>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_c = true;

      // '<S98>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = inSILFloats[16];
      fParamTmp[17] = inSILFloats[17];
      fParamTmp[18] = inSILFloats[18];
      fParamTmp[19] = inSILFloats[19];

      // '<S98>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_c[0], &fParamTmp[0], 20U * sizeof(real_T));
    }

    // '<S98>:1:29' hasFault_TurbWind=hFault;
    QuadModelv_B.hasFault_TurbWind = QuadModelv_DW.hFault_c;

    // MATLAB Function: '<S67>/FaultParamsExtract3'
    // '<S98>:1:30' FaultParam=fParam;
    // MATLAB Function 'WindFault/FaultParamsExtract3': '<S99>:1'
    // '<S99>:1:5' if isempty(hFault)
    // '<S99>:1:8' if isempty(fParam)
    // '<S99>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S99>:1:13' fParamTmp=zeros(20,1);
    for (i = 0; i < 20; i++) {
      // MATLAB Function: '<S67>/FaultParamsExtract2'
      QuadModelv_B.FaultParam_a[i] = QuadModelv_DW.fParam_c[i];

      // MATLAB Function: '<S67>/FaultParamsExtract3'
      fParamTmp[i] = 0.0;
    }

    // MATLAB Function: '<S67>/FaultParamsExtract3' incorporates:
    //   Constant: '<S67>/FaultID3'
    //   Inport: '<Root>/inSILInts'

    // '<S99>:1:14' j=1;
    j = 1.0;

    // '<S99>:1:15' for i=1:8
    for (b_t = 0; b_t < 8; b_t++) {
      // '<S99>:1:16' if inInts(i) == FaultID
      if (QuadModelv_U.inSILInts[b_t] == QuadModelv_P.FaultID3_Value_d) {
        // '<S99>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S99>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        fParamTmp[static_cast<int32_T>(2.0 * j - 1.0) - 1] = inSILFloats[((b_t +
          1) << 1) - 2];

        // '<S99>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[static_cast<int32_T>(2.0 * j) - 1] = inSILFloats[((b_t + 1) <<
          1) - 1];

        // '<S99>:1:20' j=j+1;
        j++;
      }
    }

    // '<S99>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S99>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_jy = true;

      // '<S99>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = inSILFloats[16];
      fParamTmp[17] = inSILFloats[17];
      fParamTmp[18] = inSILFloats[18];
      fParamTmp[19] = inSILFloats[19];

      // '<S99>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_py[0], &fParamTmp[0], 20U * sizeof
                  (real_T));
    }

    // '<S99>:1:29' hasFault_SheerWind=hFault;
    QuadModelv_B.hasFault_SheerWind = QuadModelv_DW.hFault_jy;

    // '<S99>:1:30' FaultParam=fParam;
    std::memcpy(&QuadModelv_B.FaultParam[0], &QuadModelv_DW.fParam_py[0], 20U *
                sizeof(real_T));
  }

  // Gain: '<S67>/Gain_-1' incorporates:
  //   Integrator: '<S14>/xe,ye,ze'

  rtb_Rn = QuadModelv_P.Gain_1_Gain * QuadModelv_X.xeyeze_CSTATE[2];

  // Saturate: '<S67>/Saturation_2'
  if (rtb_Rn > QuadModelv_P.Saturation_2_UpperSat) {
    rtb_Rn = QuadModelv_P.Saturation_2_UpperSat;
  } else if (rtb_Rn < QuadModelv_P.Saturation_2_LowerSat) {
    rtb_Rn = QuadModelv_P.Saturation_2_LowerSat;
  }

  // End of Saturate: '<S67>/Saturation_2'

  // UnitConversion: '<S149>/Unit Conversion' incorporates:
  //   UnitConversion: '<S110>/Unit Conversion'
  //   UnitConversion: '<S186>/Unit Conversion'

  // Unit Conversion - from: m to: ft
  // Expression: output = (3.28084*input) + (0)
  rtb_count = 3.280839895013123 * rtb_Rn;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // UnitConversion: '<S155>/Unit Conversion' incorporates:
    //   Constant: '<S67>/Constant_V'

    // Unit Conversion - from: m/s to: ft/s
    // Expression: output = (3.28084*input) + (0)
    QuadModelv_B.UnitConversion = 3.280839895013123 *
      QuadModelv_P.Constant_V_Value;
  }

  // Saturate: '<S182>/Limit Function 10ft to 1000ft' incorporates:
  //   UnitConversion: '<S149>/Unit Conversion'

  if (rtb_count > QuadModelv_P.LimitFunction10ftto1000ft_Upper) {
    rtb_Sum1_c = QuadModelv_P.LimitFunction10ftto1000ft_Upper;
  } else if (rtb_count < QuadModelv_P.LimitFunction10ftto1000ft_Lower) {
    rtb_Sum1_c = QuadModelv_P.LimitFunction10ftto1000ft_Lower;
  } else {
    rtb_Sum1_c = rtb_count;
  }

  // End of Saturate: '<S182>/Limit Function 10ft to 1000ft'

  // Fcn: '<S182>/Low Altitude Scale Length'
  rtb_Product3_o = 0.000823 * rtb_Sum1_c + 0.177;
  if (rtb_Product3_o < 0.0) {
    rtb_Product3_o = -rt_powd_snf(-rtb_Product3_o, 1.2);
  } else {
    rtb_Product3_o = rt_powd_snf(rtb_Product3_o, 1.2);
  }

  // Fcn: '<S182>/Low Altitude Scale Length'
  rtb_Add = rtb_Sum1_c / rtb_Product3_o;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // UnitConversion: '<S184>/Unit Conversion' incorporates:
    //   Constant: '<S183>/Medium//High Altitude'

    // Unit Conversion - from: m to: ft
    // Expression: output = (3.28084*input) + (0)
    QuadModelv_B.UnitConversion_g = 3.280839895013123 *
      QuadModelv_P.DrydenWindTurbulenceModelDiscre;
  }

  // Saturate: '<S165>/Limit Height h<1000ft' incorporates:
  //   UnitConversion: '<S149>/Unit Conversion'

  if (rtb_count > QuadModelv_P.LimitHeighth1000ft_UpperSat) {
    rtb_Product3_o = QuadModelv_P.LimitHeighth1000ft_UpperSat;
  } else if (rtb_count < QuadModelv_P.LimitHeighth1000ft_LowerSat) {
    rtb_Product3_o = QuadModelv_P.LimitHeighth1000ft_LowerSat;
  } else {
    rtb_Product3_o = rtb_count;
  }

  // End of Saturate: '<S165>/Limit Height h<1000ft'

  // Fcn: '<S165>/Low Altitude Intensity'
  rtb_Product3_o = 0.000823 * rtb_Product3_o + 0.177;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Gain: '<S165>/sigma_wg ' incorporates:
    //   Constant: '<S95>/Windspeed at 20ft (6m)'
    //   UnitConversion: '<S156>/Unit Conversion'

    // Unit Conversion - from: m/s to: ft/s
    // Expression: output = (3.28084*input) + (0)
    QuadModelv_B.sigma_wg = 3.280839895013123 *
      QuadModelv_P.FaultParamAPI.FaultInParams[7] * QuadModelv_P.sigma_wg_Gain;
  }

  // Fcn: '<S165>/Low Altitude Intensity'
  if (rtb_Product3_o < 0.0) {
    rtb_Product3_o = -rt_powd_snf(-rtb_Product3_o, 0.4);
  } else {
    rtb_Product3_o = rt_powd_snf(rtb_Product3_o, 0.4);
  }

  // Product: '<S165>/sigma_ug, sigma_vg' incorporates:
  //   Fcn: '<S165>/Low Altitude Intensity'

  rtb_TmpSignalConversionAtSFun_2 = 1.0 / rtb_Product3_o * QuadModelv_B.sigma_wg;

  // Interpolation_n-D: '<S164>/Medium//High Altitude Intensity' incorporates:
  //   PreLookup: '<S164>/PreLook-Up Index Search  (altitude)'
  //   UnitConversion: '<S149>/Unit Conversion'

  bpIndex[0] = plook_bincpa(rtb_count,
    QuadModelv_P.PreLookUpIndexSearchaltitude_Br, 11U, &rtb_Product3_o,
    &QuadModelv_DW.PreLookUpIndexSearchaltitude_DW);
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // PreLookup: '<S164>/PreLook-Up Index Search  (prob of exceed)' incorporates:
    //   Constant: '<S164>/Probability of  Exceedance'

    QuadModelv_B.PreLookUpIndexSearchprobofexc_p = plook_bincpa
      (QuadModelv_P.DrydenWindTurbulenceModelDis_n3,
       QuadModelv_P.PreLookUpIndexSearchprobofexcee, 6U,
       &QuadModelv_B.PreLookUpIndexSearchprobofexcee,
       &QuadModelv_DW.PreLookUpIndexSearchprobofexcee);
  }

  // Interpolation_n-D: '<S164>/Medium//High Altitude Intensity'
  frac[0] = rtb_Product3_o;
  frac[1] = QuadModelv_B.PreLookUpIndexSearchprobofexcee;
  bpIndex[1] = QuadModelv_B.PreLookUpIndexSearchprobofexc_p;

  // Interpolation_n-D: '<S164>/Medium//High Altitude Intensity'
  rtb_Add_br = intrp2d_la_pw(bpIndex, frac,
    QuadModelv_P.MediumHighAltitudeIntensity_Tab, 12U,
    QuadModelv_P.MediumHighAltitudeIntensity_max);
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
    // Sqrt: '<S157>/Sqrt1' incorporates:
    //   Constant: '<S157>/Constant1'

    rtb_Product3_o = std::sqrt(QuadModelv_P.WhiteNoise_Ts);

    // Product: '<S157>/Product' incorporates:
    //   Constant: '<S157>/Constant'
    //   Product: '<S157>/Divide'
    //   RandomNumber: '<S157>/White Noise'
    //   Sqrt: '<S157>/Sqrt'

    QuadModelv_B.Product_d[0] = std::sqrt(QuadModelv_P.WhiteNoise_pwr[0]) /
      rtb_Product3_o * QuadModelv_DW.NextOutput_c[0];
    QuadModelv_B.Product_d[1] = std::sqrt(QuadModelv_P.WhiteNoise_pwr[1]) /
      rtb_Product3_o * QuadModelv_DW.NextOutput_c[1];
    QuadModelv_B.Product_d[2] = std::sqrt(QuadModelv_P.WhiteNoise_pwr[2]) /
      rtb_Product3_o * QuadModelv_DW.NextOutput_c[2];
    QuadModelv_B.Product_d[3] = std::sqrt(QuadModelv_P.WhiteNoise_pwr[3]) /
      rtb_Product3_o * QuadModelv_DW.NextOutput_c[3];
  }

  // Outputs for Enabled SubSystem: '<S148>/Hugw(z)'
  // Constant: '<S148>/Constant3'
  QuadModelv_Hugwz(QuadModelv_P.DrydenWindTurbulenceModelDisc_g,
                   QuadModelv_B.UnitConversion, rtb_Add,
                   QuadModelv_B.UnitConversion_g,
                   rtb_TmpSignalConversionAtSFun_2, rtb_Add_br,
                   QuadModelv_B.Product_d[0], &QuadModelv_B.Hugwz_g,
                   &QuadModelv_DW.Hugwz_g, &QuadModelv_P.Hugwz_g);

  // End of Outputs for SubSystem: '<S148>/Hugw(z)'

  // Gain: '<S154>/Lv'
  rtb_Sum_d[0] = QuadModelv_P.Lv_Gain * rtb_Add;
  rtb_Sum_d[1] = QuadModelv_P.Lv_Gain * QuadModelv_B.UnitConversion_g;

  // Outputs for Enabled SubSystem: '<S148>/Hvgw(z)'
  // Constant: '<S148>/Constant3'
  QuadModelv_Hvgwz(QuadModelv_P.DrydenWindTurbulenceModelDisc_g,
                   rtb_TmpSignalConversionAtSFun_2, rtb_Add_br, rtb_Sum_d,
                   QuadModelv_B.UnitConversion, QuadModelv_B.Product_d[1],
                   &QuadModelv_B.Hvgwz_f, &QuadModelv_DW.Hvgwz_f,
                   &QuadModelv_P.Hvgwz_f);

  // End of Outputs for SubSystem: '<S148>/Hvgw(z)'

  // Gain: '<S154>/Lw'
  rtb_Sum_d[0] = QuadModelv_P.Lw_Gain * rtb_Sum1_c;
  rtb_Sum_d[1] = QuadModelv_P.Lw_Gain * QuadModelv_B.UnitConversion_g;

  // Outputs for Enabled SubSystem: '<S148>/Hwgw(z)'
  // Constant: '<S148>/Constant3'
  QuadModelv_Hwgwz(QuadModelv_P.DrydenWindTurbulenceModelDisc_g,
                   QuadModelv_B.UnitConversion, rtb_Sum_d, QuadModelv_B.sigma_wg,
                   rtb_Add_br, QuadModelv_B.Product_d[2], &QuadModelv_B.Hwgwz_a,
                   &QuadModelv_DW.Hwgwz_a, &QuadModelv_P.Hwgwz_a);

  // End of Outputs for SubSystem: '<S148>/Hwgw(z)'
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // UnitConversion: '<S146>/Unit Conversion' incorporates:
    //   Constant: '<S95>/Wind direction'

    // Unit Conversion - from: deg to: rad
    // Expression: output = (0.0174533*input) + (0)
    QuadModelv_B.UnitConversion_p = 0.017453292519943295 *
      QuadModelv_P.FaultParamAPI.FaultInParams[8];
  }

  // If: '<S153>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' incorporates:
  //   Constant: '<S67>/Constant_DCM'
  //   UnitConversion: '<S149>/Unit Conversion'

  if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if (rtb_count <= 1000.0) {
      rtAction = 0;
    } else if (rtb_count >= 2000.0) {
      rtAction = 1;
    } else {
      rtAction = 2;
    }

    QuadModelv_DW.ifHeightMaxlowaltitudeelseifHei = rtAction;
  } else {
    rtAction = QuadModelv_DW.ifHeightMaxlowaltitudeelseifHei;
  }

  switch (rtAction) {
   case 0:
    // Outputs for IfAction SubSystem: '<S153>/Low altitude  velocities' incorporates:
    //   ActionPort: '<S175>/Action Port'

    QuadModel_Lowaltitudevelocities(QuadModelv_P.Constant_DCM_Value,
      QuadModelv_B.Hugwz_g.Sum, QuadModelv_B.Hvgwz_f.Sum,
      QuadModelv_B.Hwgwz_a.Sum, QuadModelv_B.UnitConversion_p,
      rtb_DataTypeConversion1);

    // End of Outputs for SubSystem: '<S153>/Low altitude  velocities'
    break;

   case 1:
    // Outputs for IfAction SubSystem: '<S153>/Medium//High  altitude velocities' incorporates:
    //   ActionPort: '<S176>/Action Port'

    // DataTypeConversion: '<S8>/Data Type Conversion1' incorporates:
    //   Gain: '<S176>/Gain'

    rtb_DataTypeConversion1[0] = QuadModelv_P.Gain_Gain_h *
      QuadModelv_B.Hugwz_g.Sum[1];
    rtb_DataTypeConversion1[1] = QuadModelv_P.Gain_Gain_h *
      QuadModelv_B.Hvgwz_f.Sum[1];
    rtb_DataTypeConversion1[2] = QuadModelv_P.Gain_Gain_h *
      QuadModelv_B.Hwgwz_a.Sum[1];

    // End of Outputs for SubSystem: '<S153>/Medium//High  altitude velocities'
    break;

   case 2:
    // Outputs for IfAction SubSystem: '<S153>/Interpolate  velocities' incorporates:
    //   ActionPort: '<S174>/Action Port'

    QuadModel_Interpolatevelocities(QuadModelv_B.Hugwz_g.Sum,
      QuadModelv_B.Hvgwz_f.Sum, QuadModelv_B.Hwgwz_a.Sum,
      QuadModelv_P.Constant_DCM_Value, QuadModelv_B.UnitConversion_p, rtb_count,
      rtb_DataTypeConversion1, &QuadModelv_P.Interpolatevelocities_i);

    // End of Outputs for SubSystem: '<S153>/Interpolate  velocities'
    break;
  }

  // End of If: '<S153>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' 
  // Unit Conversion - from: ft/s to: m/s
  // Expression: output = (0.3048*input) + (0)
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // MATLAB Function: '<S67>/MATLAB Function' incorporates:
    //   UnitConversion: '<S95>/Unit Conversion'

    std::memcpy(&rtb_FaultParam[0], &QuadModelv_B.FaultParam_c[0], 20U * sizeof
                (real_T));

    // MATLAB Function 'WindFault/MATLAB Function': '<S101>:1'
    // '<S101>:1:3' if(GustWindParams(2)<1)
    if (QuadModelv_B.FaultParam_c[1] < 1.0) {
      // '<S101>:1:4' GustWindParams(2)=1;
      rtb_FaultParam[1] = 1.0;
    }

    // '<S101>:1:6' timeVel=60/GustWindParams(2);
    rtb_Add = 60.0 / rtb_FaultParam[1];

    // '<S101>:1:8' vmax=GustWindParams(1);
    rtb_TmpSignalConversionAtSFun_2 = QuadModelv_B.FaultParam_c[0];

    // '<S101>:1:11' if isempty(t0)
    // '<S101>:1:16' if isempty(isInGust)
    // '<S101>:1:21' if isempty(t1)
    // '<S101>:1:26' if isempty(a)
    // '<S101>:1:31' if isempty(ang)
    // '<S101>:1:36' if isempty(wlast)
    if (!QuadModelv_DW.wlast_not_empty) {
      // '<S101>:1:37' wlast=vmax*0.5;
      QuadModelv_DW.wlast = QuadModelv_B.FaultParam_c[0] * 0.5;
      QuadModelv_DW.wlast_not_empty = true;
    }

    // '<S101>:1:41' if isempty(wNow)
    if (!QuadModelv_DW.wNow_not_empty) {
      // '<S101>:1:42' wNow=1.4*0.8*vmax;
      QuadModelv_DW.wNow = 1.1199999999999999 * QuadModelv_B.FaultParam_c[0];
      QuadModelv_DW.wNow_not_empty = true;
    }

    // '<S101>:1:46' if isGustWind&&isInGust<0.5
    if (QuadModelv_B.hasFault_GustWind && (QuadModelv_DW.isInGust < 0.5)) {
      // '<S101>:1:47' t0=t;
      QuadModelv_DW.t0 = FaultNum;

      // '<S101>:1:48' t1=timeVel*(3/4+1/2*rand(1));
      QuadModelv_DW.t1 = (0.5 * QuadModelv_rand_p() + 0.75) * rtb_Add;

      // '<S101>:1:49' a=1/4+1/2*rand(1);
      QuadModelv_DW.a = 0.5 * QuadModelv_rand_p() + 0.25;

      // '<S101>:1:50' ang=pi*2*rand(1);
      QuadModelv_DW.ang = 6.2831853071795862 * QuadModelv_rand_p();

      // '<S101>:1:51' wlast=wNow;
      QuadModelv_DW.wlast = QuadModelv_DW.wNow;

      // '<S101>:1:52' wNow=1.2*(rand(1)*0.5+0.5)*vmax;
      QuadModelv_DW.wNow = (QuadModelv_rand_p() * 0.5 + 0.5) * 1.2 *
        rtb_TmpSignalConversionAtSFun_2;

      // '<S101>:1:54' isInGust=1;
      QuadModelv_DW.isInGust = 1.0;
    }

    // '<S101>:1:57' if ~isGustWind
    if (!QuadModelv_B.hasFault_GustWind) {
      // '<S101>:1:58' isInGust=0;
      QuadModelv_DW.isInGust = 0.0;
    }

    // '<S101>:1:61' gWind=[0;0;0];
    // '<S101>:1:63' if(t<t0+t1*a)
    rtb_TmpSignalConversionAtSFun_0 = QuadModelv_DW.t1 * QuadModelv_DW.a;
    if (FaultNum < rtb_TmpSignalConversionAtSFun_0 + QuadModelv_DW.t0) {
      // '<S101>:1:64' gWindMag=(wNow-wlast)/2*(1-cos((t-t0)/(t1*a)*pi))+wlast;
      rtb_Product3_o = (1.0 - std::cos((FaultNum - QuadModelv_DW.t0) /
        rtb_TmpSignalConversionAtSFun_0 * 3.1415926535897931)) *
        ((QuadModelv_DW.wNow - QuadModelv_DW.wlast) / 2.0) + QuadModelv_DW.wlast;
    } else {
      // '<S101>:1:65' else
      // '<S101>:1:66' gWindMag=wNow;
      rtb_Product3_o = QuadModelv_DW.wNow;
    }

    // '<S101>:1:69' if abs(t-(t0+t1))<0.05
    if (std::abs(FaultNum - (QuadModelv_DW.t0 + QuadModelv_DW.t1)) < 0.05) {
      uint32_T seed;

      // '<S101>:1:70' rng(t*1000);
      rtb_TmpSignalConversionAtSFun_0 = FaultNum * 1000.0;
      if (rtb_TmpSignalConversionAtSFun_0 < 4.294967296E+9) {
        if (rtb_TmpSignalConversionAtSFun_0 >= 0.0) {
          seed = static_cast<uint32_T>(rtb_TmpSignalConversionAtSFun_0);
        } else {
          seed = 0U;
        }
      } else {
        seed = MAX_uint32_T;
      }

      switch (QuadModelv_DW.method) {
       case 7U:
        if (seed == 0U) {
          seed = 5489U;
        }

        QuadModelv_DW.state_a[0] = seed;
        for (r = 0; r < 623; r++) {
          seed = ((seed >> 30U ^ seed) * 1812433253U + r) + 1U;
          QuadModelv_DW.state_a[r + 1] = seed;
        }

        QuadModelv_DW.state_a[624] = 624U;
        break;

       case 5U:
        QuadModelv_DW.state_i[0] = 362436069U;
        QuadModelv_DW.state_i[1] = seed;
        if (QuadModelv_DW.state_i[1] == 0U) {
          QuadModelv_DW.state_i[1] = 521288629U;
        }
        break;

       default:
        r = static_cast<int32_T>(seed >> 16U);
        b_t = static_cast<int32_T>(seed & 32768U);
        seed = ((((seed - (static_cast<uint32_T>(r) << 16U)) - b_t) << 16U) +
                b_t) + r;
        if (seed < 1U) {
          seed = 1144108930U;
        } else if (seed > 2147483646U) {
          seed = 2147483646U;
        }

        QuadModelv_DW.state = seed;
        break;
      }

      // '<S101>:1:71' t0=t;
      QuadModelv_DW.t0 = FaultNum;

      // '<S101>:1:72' t1=timeVel*(3/4+1/2*rand(1));
      QuadModelv_DW.t1 = (0.5 * QuadModelv_rand_p() + 0.75) * rtb_Add;

      // '<S101>:1:73' a=1/4+1/2*rand(1);
      QuadModelv_DW.a = 0.5 * QuadModelv_rand_p() + 0.25;

      // '<S101>:1:74' ang=ang+0.1*pi*rand(1);
      QuadModelv_DW.ang += 0.31415926535897931 * QuadModelv_rand_p();

      // '<S101>:1:75' wlast=wNow;
      QuadModelv_DW.wlast = QuadModelv_DW.wNow;

      // '<S101>:1:76' wNow=1.2*(rand(1)*0.5+0.5)*vmax;
      QuadModelv_DW.wNow = (QuadModelv_rand_p() * 0.5 + 0.5) * 1.2 *
        rtb_TmpSignalConversionAtSFun_2;
    }

    // '<S101>:1:79' gWind(1)=gWindMag*cos(ang)*0.8+turb(1)*0.4;
    QuadModelv_B.gWind[0] = rtb_Product3_o * std::cos(QuadModelv_DW.ang) * 0.8 +
      0.3048 * rtb_DataTypeConversion1[0] * 0.4;

    // '<S101>:1:80' gWind(2)=gWindMag*sin(ang)*0.8+turb(2)*0.4;
    QuadModelv_B.gWind[1] = rtb_Product3_o * std::sin(QuadModelv_DW.ang) * 0.8 +
      0.3048 * rtb_DataTypeConversion1[1] * 0.4;

    // '<S101>:1:81' gWind(3)=turb(3)*0.4;
    QuadModelv_B.gWind[2] = 0.3048 * rtb_DataTypeConversion1[2] * 0.4;

    // End of MATLAB Function: '<S67>/MATLAB Function'

    // UnitConversion: '<S116>/Unit Conversion' incorporates:
    //   Constant: '<S67>/Constant_V'

    // Unit Conversion - from: m/s to: ft/s
    // Expression: output = (3.28084*input) + (0)
    QuadModelv_B.UnitConversion_a = 3.280839895013123 *
      QuadModelv_P.Constant_V_Value;
  }

  // Saturate: '<S143>/Limit Function 10ft to 1000ft'
  // Unit Conversion - from: m to: ft
  // Expression: output = (3.28084*input) + (0)
  if (rtb_count > QuadModelv_P.LimitFunction10ftto1000ft_Upp_d) {
    rtb_Sum1_c = QuadModelv_P.LimitFunction10ftto1000ft_Upp_d;
  } else if (rtb_count < QuadModelv_P.LimitFunction10ftto1000ft_Low_j) {
    rtb_Sum1_c = QuadModelv_P.LimitFunction10ftto1000ft_Low_j;
  } else {
    rtb_Sum1_c = rtb_count;
  }

  // End of Saturate: '<S143>/Limit Function 10ft to 1000ft'

  // Fcn: '<S143>/Low Altitude Scale Length'
  rtb_Product3_o = 0.000823 * rtb_Sum1_c + 0.177;
  if (rtb_Product3_o < 0.0) {
    rtb_Product3_o = -rt_powd_snf(-rtb_Product3_o, 1.2);
  } else {
    rtb_Product3_o = rt_powd_snf(rtb_Product3_o, 1.2);
  }

  // Fcn: '<S143>/Low Altitude Scale Length'
  rtb_fcn3 = rtb_Sum1_c / rtb_Product3_o;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // UnitConversion: '<S145>/Unit Conversion' incorporates:
    //   Constant: '<S144>/Medium//High Altitude'

    // Unit Conversion - from: m to: ft
    // Expression: output = (3.28084*input) + (0)
    QuadModelv_B.UnitConversion_n = 3.280839895013123 *
      QuadModelv_P.DrydenWindTurbulenceModelDisc_m;
  }

  // Saturate: '<S126>/Limit Height h<1000ft'
  if (rtb_count > QuadModelv_P.LimitHeighth1000ft_UpperSat_e) {
    rtb_Product3_o = QuadModelv_P.LimitHeighth1000ft_UpperSat_e;
  } else if (rtb_count < QuadModelv_P.LimitHeighth1000ft_LowerSat_o) {
    rtb_Product3_o = QuadModelv_P.LimitHeighth1000ft_LowerSat_o;
  } else {
    rtb_Product3_o = rtb_count;
  }

  // End of Saturate: '<S126>/Limit Height h<1000ft'

  // Fcn: '<S126>/Low Altitude Intensity'
  rtb_Product3_o = 0.000823 * rtb_Product3_o + 0.177;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Gain: '<S126>/sigma_wg ' incorporates:
    //   Constant: '<S94>/Windspeed at 20ft (6m)'
    //   UnitConversion: '<S117>/Unit Conversion'

    // Unit Conversion - from: m/s to: ft/s
    // Expression: output = (3.28084*input) + (0)
    QuadModelv_B.sigma_wg_p = 3.280839895013123 *
      QuadModelv_P.FaultParamAPI.FaultInParams[3] * QuadModelv_P.sigma_wg_Gain_o;
  }

  // Fcn: '<S126>/Low Altitude Intensity'
  if (rtb_Product3_o < 0.0) {
    rtb_Product3_o = -rt_powd_snf(-rtb_Product3_o, 0.4);
  } else {
    rtb_Product3_o = rt_powd_snf(rtb_Product3_o, 0.4);
  }

  // Product: '<S126>/sigma_ug, sigma_vg' incorporates:
  //   Fcn: '<S126>/Low Altitude Intensity'

  j = 1.0 / rtb_Product3_o * QuadModelv_B.sigma_wg_p;

  // Interpolation_n-D: '<S125>/Medium//High Altitude Intensity' incorporates:
  //   PreLookup: '<S125>/PreLook-Up Index Search  (altitude)'

  bpIndex_0[0] = plook_bincpa(rtb_count,
    QuadModelv_P.PreLookUpIndexSearchaltitude__g, 11U, &rtb_Product3_o,
    &QuadModelv_DW.PreLookUpIndexSearchaltitude__b);
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // PreLookup: '<S125>/PreLook-Up Index Search  (prob of exceed)' incorporates:
    //   Constant: '<S125>/Probability of  Exceedance'

    QuadModelv_B.PreLookUpIndexSearchprobofexc_n = plook_bincpa
      (QuadModelv_P.DrydenWindTurbulenceModelDis_gr,
       QuadModelv_P.PreLookUpIndexSearchprobofexc_n, 6U,
       &QuadModelv_B.PreLookUpIndexSearchprobofexc_i,
       &QuadModelv_DW.PreLookUpIndexSearchprobofexc_l);
  }

  // Interpolation_n-D: '<S125>/Medium//High Altitude Intensity'
  frac_0[0] = rtb_Product3_o;
  frac_0[1] = QuadModelv_B.PreLookUpIndexSearchprobofexc_i;
  bpIndex_0[1] = QuadModelv_B.PreLookUpIndexSearchprobofexc_n;

  // Interpolation_n-D: '<S125>/Medium//High Altitude Intensity'
  rtb_TmpSignalConversionAtSFun_2 = intrp2d_la_pw(bpIndex_0, frac_0,
    QuadModelv_P.MediumHighAltitudeIntensity_T_b, 12U,
    QuadModelv_P.MediumHighAltitudeIntensity_m_i);
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
    // Sqrt: '<S118>/Sqrt1' incorporates:
    //   Constant: '<S118>/Constant1'

    rtb_Product3_o = std::sqrt(QuadModelv_P.WhiteNoise_Ts_a);

    // Product: '<S118>/Product' incorporates:
    //   Constant: '<S118>/Constant'
    //   Product: '<S118>/Divide'
    //   RandomNumber: '<S118>/White Noise'
    //   Sqrt: '<S118>/Sqrt'

    QuadModelv_B.Product_p[0] = std::sqrt(QuadModelv_P.WhiteNoise_pwr_d[0]) /
      rtb_Product3_o * QuadModelv_DW.NextOutput_p[0];
    QuadModelv_B.Product_p[1] = std::sqrt(QuadModelv_P.WhiteNoise_pwr_d[1]) /
      rtb_Product3_o * QuadModelv_DW.NextOutput_p[1];
    QuadModelv_B.Product_p[2] = std::sqrt(QuadModelv_P.WhiteNoise_pwr_d[2]) /
      rtb_Product3_o * QuadModelv_DW.NextOutput_p[2];
    QuadModelv_B.Product_p[3] = std::sqrt(QuadModelv_P.WhiteNoise_pwr_d[3]) /
      rtb_Product3_o * QuadModelv_DW.NextOutput_p[3];
  }

  // Outputs for Enabled SubSystem: '<S109>/Hugw(z)'
  // Constant: '<S109>/Constant3'
  QuadModelv_Hugwz(QuadModelv_P.DrydenWindTurbulenceModelDisc_k,
                   QuadModelv_B.UnitConversion_a, rtb_fcn3,
                   QuadModelv_B.UnitConversion_n, j,
                   rtb_TmpSignalConversionAtSFun_2, QuadModelv_B.Product_p[0],
                   &QuadModelv_B.Hugwz, &QuadModelv_DW.Hugwz,
                   &QuadModelv_P.Hugwz);

  // End of Outputs for SubSystem: '<S109>/Hugw(z)'

  // Gain: '<S115>/Lv'
  frac_0[0] = QuadModelv_P.Lv_Gain_o * rtb_fcn3;
  frac_0[1] = QuadModelv_P.Lv_Gain_o * QuadModelv_B.UnitConversion_n;

  // Outputs for Enabled SubSystem: '<S109>/Hvgw(z)'
  // Constant: '<S109>/Constant3'
  QuadModelv_Hvgwz(QuadModelv_P.DrydenWindTurbulenceModelDisc_k, j,
                   rtb_TmpSignalConversionAtSFun_2, frac_0,
                   QuadModelv_B.UnitConversion_a, QuadModelv_B.Product_p[1],
                   &QuadModelv_B.Hvgwz, &QuadModelv_DW.Hvgwz,
                   &QuadModelv_P.Hvgwz);

  // End of Outputs for SubSystem: '<S109>/Hvgw(z)'

  // Gain: '<S115>/Lw'
  frac_0[0] = QuadModelv_P.Lw_Gain_c * rtb_Sum1_c;
  frac_0[1] = QuadModelv_P.Lw_Gain_c * QuadModelv_B.UnitConversion_n;

  // Outputs for Enabled SubSystem: '<S109>/Hwgw(z)'
  // Constant: '<S109>/Constant3'
  QuadModelv_Hwgwz(QuadModelv_P.DrydenWindTurbulenceModelDisc_k,
                   QuadModelv_B.UnitConversion_a, frac_0,
                   QuadModelv_B.sigma_wg_p, rtb_TmpSignalConversionAtSFun_2,
                   QuadModelv_B.Product_p[2], &QuadModelv_B.Hwgwz,
                   &QuadModelv_DW.Hwgwz, &QuadModelv_P.Hwgwz);

  // End of Outputs for SubSystem: '<S109>/Hwgw(z)'
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // UnitConversion: '<S107>/Unit Conversion' incorporates:
    //   Constant: '<S94>/Wind direction'

    // Unit Conversion - from: deg to: rad
    // Expression: output = (0.0174533*input) + (0)
    QuadModelv_B.UnitConversion_ab = 0.017453292519943295 *
      QuadModelv_P.FaultParamAPI.FaultInParams[4];
  }

  // If: '<S114>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' incorporates:
  //   Constant: '<S67>/Constant_DCM'

  if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if (rtb_count <= 1000.0) {
      rtAction = 0;
    } else if (rtb_count >= 2000.0) {
      rtAction = 1;
    } else {
      rtAction = 2;
    }

    QuadModelv_DW.ifHeightMaxlowaltitudeelseifH_p = rtAction;
  } else {
    rtAction = QuadModelv_DW.ifHeightMaxlowaltitudeelseifH_p;
  }

  switch (rtAction) {
   case 0:
    // Outputs for IfAction SubSystem: '<S114>/Low altitude  velocities' incorporates:
    //   ActionPort: '<S136>/Action Port'

    QuadModel_Lowaltitudevelocities(QuadModelv_P.Constant_DCM_Value,
      QuadModelv_B.Hugwz.Sum, QuadModelv_B.Hvgwz.Sum, QuadModelv_B.Hwgwz.Sum,
      QuadModelv_B.UnitConversion_ab, rtb_DataTypeConversion1);

    // End of Outputs for SubSystem: '<S114>/Low altitude  velocities'
    break;

   case 1:
    // Outputs for IfAction SubSystem: '<S114>/Medium//High  altitude velocities' incorporates:
    //   ActionPort: '<S137>/Action Port'

    // DataTypeConversion: '<S8>/Data Type Conversion1' incorporates:
    //   Gain: '<S137>/Gain'

    rtb_DataTypeConversion1[0] = QuadModelv_P.Gain_Gain_pq *
      QuadModelv_B.Hugwz.Sum[1];
    rtb_DataTypeConversion1[1] = QuadModelv_P.Gain_Gain_pq *
      QuadModelv_B.Hvgwz.Sum[1];
    rtb_DataTypeConversion1[2] = QuadModelv_P.Gain_Gain_pq *
      QuadModelv_B.Hwgwz.Sum[1];

    // End of Outputs for SubSystem: '<S114>/Medium//High  altitude velocities'
    break;

   case 2:
    // Outputs for IfAction SubSystem: '<S114>/Interpolate  velocities' incorporates:
    //   ActionPort: '<S135>/Action Port'

    QuadModel_Interpolatevelocities(QuadModelv_B.Hugwz.Sum,
      QuadModelv_B.Hvgwz.Sum, QuadModelv_B.Hwgwz.Sum,
      QuadModelv_P.Constant_DCM_Value, QuadModelv_B.UnitConversion_ab, rtb_count,
      rtb_DataTypeConversion1, &QuadModelv_P.Interpolatevelocities);

    // End of Outputs for SubSystem: '<S114>/Interpolate  velocities'
    break;
  }

  // End of If: '<S114>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' 

  // UnitConversion: '<S94>/Unit Conversion'
  // Unit Conversion - from: ft/s to: m/s
  // Expression: output = (0.3048*input) + (0)
  rtb_sincos_o1_f[0] = 0.3048 * rtb_DataTypeConversion1[0];
  rtb_sincos_o1_f[1] = 0.3048 * rtb_DataTypeConversion1[1];
  rtb_sincos_o1_f[2] = 0.3048 * rtb_DataTypeConversion1[2];

  // MATLAB Function 'WindFault/TurbWindStrength_Dec_Switch': '<S104>:1'
  // '<S104>:1:3' TurbWind=vwind*TurbWindParams(1);
  // Unit Conversion - from: m to: ft
  // Expression: output = (3.28084*input) + (0)
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Math: '<S105>/ln(ref_height//z0)' incorporates:
    //   Constant: '<S105>/ref_height//z0'
    //
    //  About '<S105>/ln(ref_height//z0)':
    //   Operator: log

    QuadModelv_B.lnref_heightz0 = std::log(QuadModelv_P.ref_heightz0_Value);
  }

  // Saturate: '<S105>/3ft-->inf'
  if (rtb_count > QuadModelv_P.uftinf_UpperSat) {
    j = QuadModelv_P.uftinf_UpperSat;
  } else if (rtb_count < QuadModelv_P.uftinf_LowerSat) {
    j = QuadModelv_P.uftinf_LowerSat;
  } else {
    j = rtb_count;
  }

  // End of Saturate: '<S105>/3ft-->inf'

  // Product: '<S105>/Product' incorporates:
  //   Gain: '<S105>/h//z0'
  //   Math: '<S105>/ln(h//z0)'
  //
  //  About '<S105>/ln(h//z0)':
  //   Operator: log

  rtb_Product3_o = std::log(QuadModelv_P.hz0_Gain * j) /
    QuadModelv_B.lnref_heightz0;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // UnitConversion: '<S185>/Unit Conversion' incorporates:
    //   Constant: '<S105>/Wind Direction'

    // Unit Conversion - from: deg to: rad
    // Expression: output = (0.0174533*input) + (0)
    Z = 0.017453292519943295 * QuadModelv_P.FaultParamAPI.FaultInParams[6];

    // Gain: '<S105>/Wind speed at reference height' incorporates:
    //   Constant: '<S105>/Wdeg1'
    //   Trigonometry: '<S105>/SinCos'

    QuadModelv_B.Windspeedatreferenceheight[0] =
      -QuadModelv_P.FaultParamAPI.FaultInParams[5] * std::cos(Z);
    QuadModelv_B.Windspeedatreferenceheight[1] =
      -QuadModelv_P.FaultParamAPI.FaultInParams[5] * std::sin(Z);
    QuadModelv_B.Windspeedatreferenceheight[2] =
      -QuadModelv_P.FaultParamAPI.FaultInParams[5] * QuadModelv_P.Wdeg1_Value;
  }

  // DataTypeConversion: '<S8>/Data Type Conversion1' incorporates:
  //   Product: '<S105>/Product1'

  // MATLAB Function 'WindFault/SheerWindStrength_Dec_Switch': '<S103>:1'
  // '<S103>:1:2' SheerWind=vwind*SheerWindParams(1);
  // MATLAB Function 'WindFault/MATLAB Function1': '<S102>:1'
  // '<S102>:1:2' wind=[0;0;0];
  rtb_DataTypeConversion1[0] = rtb_Product3_o *
    QuadModelv_B.Windspeedatreferenceheight[0];

  // MATLAB Function: '<S67>/MATLAB Function1'
  rtb_Rn = 0.0;

  // DataTypeConversion: '<S8>/Data Type Conversion1' incorporates:
  //   Product: '<S105>/Product1'

  rtb_DataTypeConversion1[1] = rtb_Product3_o *
    QuadModelv_B.Windspeedatreferenceheight[1];

  // MATLAB Function: '<S67>/MATLAB Function1'
  rtb_fcn3 = 0.0;

  // DataTypeConversion: '<S8>/Data Type Conversion1' incorporates:
  //   Product: '<S105>/Product1'

  rtb_DataTypeConversion1[2] = rtb_Product3_o *
    QuadModelv_B.Windspeedatreferenceheight[2];

  // MATLAB Function: '<S67>/MATLAB Function1' incorporates:
  //   MATLAB Function: '<S67>/SheerWindStrength_Dec_Switch'
  //   MATLAB Function: '<S67>/TurbWindStrength_Dec_Switch'
  //   Product: '<S105>/Transform from Inertial to Body axes'

  rtb_TmpSignalConversionAtSFun_0 = 0.0;

  // '<S102>:1:3' if isConstWind
  if (QuadModelv_B.hasFault_ConstWind) {
    // '<S102>:1:4' wind=[ConstWind(1);ConstWind(2);ConstWind(3)];
    rtb_Rn = QuadModelv_B.FaultParam_g[0];
    rtb_fcn3 = QuadModelv_B.FaultParam_g[1];
    rtb_TmpSignalConversionAtSFun_0 = QuadModelv_B.FaultParam_g[2];
  } else if (QuadModelv_B.hasFault_GustWind) {
    // '<S102>:1:5' elseif isGustWind
    // '<S102>:1:6' wind=GustWind;
    rtb_Rn = QuadModelv_B.gWind[0];
    rtb_fcn3 = QuadModelv_B.gWind[1];
    rtb_TmpSignalConversionAtSFun_0 = QuadModelv_B.gWind[2];
  } else if (QuadModelv_B.hasFault_TurbWind) {
    // '<S102>:1:7' elseif isTurbWind
    // '<S102>:1:8' wind=TurbWind;
    rtb_Rn = rtb_sincos_o1_f[0] * QuadModelv_B.FaultParam_a[0];
    rtb_fcn3 = QuadModelv_B.FaultParam_a[0] * rtb_sincos_o1_f[1];
    rtb_TmpSignalConversionAtSFun_0 = QuadModelv_B.FaultParam_a[0] *
      rtb_sincos_o1_f[2];
  } else if (QuadModelv_B.hasFault_SheerWind) {
    // Product: '<S105>/Transform from Inertial to Body axes' incorporates:
    //   Constant: '<S67>/Constant_DCM'
    //   DataTypeConversion: '<S8>/Data Type Conversion1'

    // '<S102>:1:9' elseif isSheerWind
    // '<S102>:1:10' wind=SheerWind;
    for (i = 0; i < 3; i++) {
      rtb_Merge[i] = (QuadModelv_P.Constant_DCM_Value[i + 3] *
                      rtb_DataTypeConversion1[1] +
                      QuadModelv_P.Constant_DCM_Value[i] *
                      rtb_DataTypeConversion1[0]) +
        QuadModelv_P.Constant_DCM_Value[i + 6] * rtb_DataTypeConversion1[2];
    }

    rtb_Rn = rtb_Merge[0] * QuadModelv_B.FaultParam[0];
    rtb_fcn3 = QuadModelv_B.FaultParam[0] * rtb_Merge[1];
    rtb_TmpSignalConversionAtSFun_0 = QuadModelv_B.FaultParam[0] * rtb_Merge[2];
  }

  for (i = 0; i < 3; i++) {
    // Product: '<S67>/Product' incorporates:
    //   Concatenate: '<S36>/Vector Concatenate'

    rtb_Product3_o = (VectorConcatenate[i + 3] * rtb_fcn3 + VectorConcatenate[i]
                      * rtb_Rn) + VectorConcatenate[i + 6] *
      rtb_TmpSignalConversionAtSFun_0;

    // Product: '<S67>/Product1' incorporates:
    //   Integrator: '<S14>/ub,vb,wb'
    //   Sum: '<S67>/Sum'
    //   Sum: '<S67>/Sum1'

    rtb_Windb[i] = (QuadModelv_X.ubvbwb_CSTATE[i] - rtb_Product3_o) *
      QuadModelv_B.Sum1[i];

    // Product: '<S67>/Product'
    rtb_sincos_o1_f[i] = rtb_Product3_o;
  }

  // MATLAB Function: '<S7>/Propeller Model old Version' incorporates:
  //   Constant: '<S6>/ModelParam.uavR'
  //   Constant: '<S7>/ModelParam.motorJm'
  //   Constant: '<S7>/ModelParam.rotorCm'
  //   Constant: '<S7>/ModelParam.rotorCt'
  //   Constant: '<S7>/ModelParam.uavType'
  //   Integrator: '<S14>/p,q,r '

  rtAction = QuadModelv_P.ModelParam_uavType;

  //  This is control effectiveness model, where Fb is the force produced by motors, Mb is 
  //  the moment produced by motors, and Fd is the resistance produced by the air. 
  //  The gyroscopic torque is not considered here.
  // MATLAB Function 'Force and Moment Model/Propeller Model old Version': '<S198>:1' 
  //  -1: anticlockwise, 1: clockwise
  // '<S198>:1:7' TypeMotorDirMatrix=[...
  // '<S198>:1:8'     [-1 -1 -1 0 0 0 0 0];... % Tri 3*1 X
  // '<S198>:1:9'     [1 1 1 0 0 0 0 0];...    % Tri 3*1 +
  // '<S198>:1:10'     [-1 -1 1 1 0 0 0 0];... % Quad 4*1 X
  // '<S198>:1:11'     [-1 -1 1 1 0 0 0 0];... % Quad 4*1 +
  // '<S198>:1:12'     [1 -1 1 -1 -1 1 0 0];... % Hex 6*1 X
  // '<S198>:1:13'     [1 -1 1 -1 -1 1 0 0];... % Hex 6*1 +
  // '<S198>:1:14'     [1 -1 1 -1 1 -1 0 0];... % HexCoa 3*2
  // '<S198>:1:15'     [1 1 -1 -1 -1 -1 1 1];... % Oct 8*1 X
  // '<S198>:1:16'     [1 1 -1 -1 -1 -1 1 1];... % Oct 8*1 +
  // '<S198>:1:17'     [1 -1 1 -1 1 -1 1 -1];... % OctCoa 4*2 X
  // '<S198>:1:18' ];
  //  % Tri 3*1 X
  //     % Tri 3*1 +
  //  % Quad 4*1 X
  //  % Quad 4*1 +
  //  % Hex 6*1 X
  //  % Hex 6*1 +
  //  % HexCoa 3*2
  //  % Oct 8*1 X
  //  % Oct 8*1 +
  //  % OctCoa 4*2 X
  // '<S198>:1:20' TypeMotorAngMatrix=[...
  // '<S198>:1:21'     [pi/3 pi -pi/3 0 0 0 0 0];...
  // '<S198>:1:22'     [pi/3 pi -pi/3 0 0 0 0 0];...
  // '<S198>:1:23'     [pi/4 pi/4+pi pi/4+3*pi/2 pi/4+pi/2 0 0 0 0];...
  // '<S198>:1:24'     [pi/2 3*pi/2 0 pi 0 0 0 0];...
  // '<S198>:1:25'     [pi/2 3/2*pi 3/2*pi+pi/3 pi/2+pi/3 pi/2-pi/3 3/2*pi-pi/3 0 0];... 
  // '<S198>:1:26'     [0 pi pi+pi/3 pi/3 -pi/3 pi-pi/3 0 0];...
  // '<S198>:1:27'     [pi/3 pi/3 pi pi -pi/3 -pi/3 0 0];...
  // '<S198>:1:28'     [pi/8 pi+pi/8 pi/8+pi/4 pi-pi/8 -pi/8 3/2*pi-pi/8 3/2*pi+pi/8 pi/2+pi/8];... 
  // '<S198>:1:29'     [0 pi pi/4 3/4*pi -pi/4 pi+pi/4 3/2*pi pi/2];...
  // '<S198>:1:30'     [pi/4 -pi/4 pi+pi/4 pi-pi/4 -pi/4 pi/4 pi-pi/4 pi+pi/4];... 
  // '<S198>:1:31' ];
  // '<S198>:1:33' TypeMotorNonVector=int8([3;3;4;4;6;6;6;8;8;8]);
  // '<S198>:1:35' Mb = [0;0;0];
  // '<S198>:1:36' Fb = [0;0;0];
  rtb_Mb[0] = 0.0;
  rtb_Rn = 0.0;
  rtb_Mb[1] = 0.0;
  rtb_Mb[2] = 0.0;
  rtb_TmpSignalConversionAtSFun_0 = 0.0;

  // '<S198>:1:37' P=wb(1);
  // '<S198>:1:37' Q=wb(2);
  // '<S198>:1:39' if uavType==50
  if (QuadModelv_P.ModelParam_uavType == 50) {
    // '<S198>:1:41' Fb(1)=MotorRads(1)+MotorRads(3);
    rtb_Rn = QuadModelv_B.y[0] + QuadModelv_B.y[2];

    // '<S198>:1:42' Mb(3)=MotorRads(1)*0.1-MotorRads(3)*0.1;
    rtb_Mb[2] = QuadModelv_B.y[0] * 0.1 - QuadModelv_B.y[2] * 0.1;
  } else {
    // '<S198>:1:48' if uavType<0||uavType>length(TypeMotorNonVector)
    if ((QuadModelv_P.ModelParam_uavType < 0) ||
        (QuadModelv_P.ModelParam_uavType > 10)) {
      // '<S198>:1:49' uavType=int8(1);
      rtAction = 1;
    }

    // '<S198>:1:52' TypeMotorDirVector=TypeMotorDirMatrix(uavType,:);
    // '<S198>:1:53' TypeMotorAngVector=TypeMotorAngMatrix(uavType,:);
    // '<S198>:1:55' for i=1:TypeMotorNonVector(uavType)
    for (b_t = 1; b_t - 1 < c[rtAction - 1]; b_t++) {
      // '<S198>:1:56' propAng = TypeMotorAngVector(i);
      // '<S198>:1:57' propdir = TypeMotorDirVector(i);
      // '<S198>:1:58' PropT = Ct*MotorRads(i)*MotorRads(i);
      rtb_Add = QuadModelv_B.y[static_cast<int8_T>(b_t) - 1];
      rtb_Product3_o = rtb_Add * QuadModelv_P.ModelParam_rotorCt * rtb_Add;

      // '<S198>:1:59' PropM = Cm*MotorRads(i)*MotorRads(i);
      // Thrust -> Moment
      // '<S198>:1:62' Mb=Mb+[-R*sin(propAng)*PropT;R*cos(propAng)*PropT;-PropM*propdir]; 
      rtb_TmpSignalConversionAtSFun_1 = rtb_Mb[0];
      rtb_fcn3 = rtb_Mb[1];
      rtb_Sum1_c = rtb_Mb[2];

      // gyroscopic moment
      // '<S198>:1:65' Mb=Mb+[Q*Jm*MotorRads(i)*propdir;-P*Jm*MotorRads(i)*propdir;0]; 
      rtb_alt = ((static_cast<int8_T>(b_t) - 1) * 10 + rtAction) - 1;
      j = d[rtb_alt];
      rtb_DataTypeConversion_idx_0 = b[rtb_alt];
      rtb_Mb[0] = (std::sin(j) * -QuadModelv_P.ModelParam_uavR * rtb_Product3_o
                   + rtb_TmpSignalConversionAtSFun_1) + QuadModelv_X.pqr_CSTATE
        [1] * QuadModelv_P.ModelParam_motorJm * rtb_Add *
        rtb_DataTypeConversion_idx_0;
      rtb_Mb[1] = (std::cos(j) * QuadModelv_P.ModelParam_uavR * rtb_Product3_o +
                   rtb_fcn3) + -QuadModelv_X.pqr_CSTATE[0] *
        QuadModelv_P.ModelParam_motorJm * rtb_Add * rtb_DataTypeConversion_idx_0;
      rtb_Mb[2] = -(rtb_Add * QuadModelv_P.ModelParam_rotorCm * rtb_Add) *
        rtb_DataTypeConversion_idx_0 + rtb_Sum1_c;

      // '<S198>:1:67' Fb=Fb+[0;0;-PropT];
      rtb_TmpSignalConversionAtSFun_0 += -rtb_Product3_o;
    }
  }

  // End of MATLAB Function: '<S7>/Propeller Model old Version'

  // MATLAB Function: '<S7>/Body AeroCenter' incorporates:
  //   Constant: '<S7>/ModelParam.uavCd'
  //   Product: '<S67>/Product1'

  //  This is control effectiveness model, where Fb is the force produced by motors, Mb is 
  //  the moment produced by motors, and Fd is the resistance produced by the air. 
  //  The gyroscopic torque is not considered here.
  //  uavDearo 机身重心位置，向下为正
  // MATLAB Function 'Force and Moment Model/Body AeroCenter': '<S192>:1'
  // '<S192>:1:7' Fd = -Cd*Vb.*abs(Vb);
  //
  // '<S192>:1:9' Mfd=[Fd(2)*uavDearo;-Fd(1)*uavDearo;0];
  // '<S192>:1:11' Md=-CCm.*wb.*abs(wb);
  // '<S192>:1:12' Md=Mfd+Md;
  //   CCm=4.72e-3;
  //   Md=-CCm*wb.*abs(wb)*0.5;%0.3
  //   Md= Mfd+Md;
  rtb_Product3_o = -QuadModelv_P.ModelParam_uavCd * rtb_Windb[0] * std::abs
    (rtb_Windb[0]);

  // Sum: '<S7>/Add2'
  rtb_Add = (rtb_DataTypeConversion2[0] + rtb_Rn) + rtb_Product3_o;

  // DataTypeConversion: '<S8>/Data Type Conversion1' incorporates:
  //   Product: '<S12>/Divide'

  rtb_DataTypeConversion1[0] = rtb_Add / QuadModelv_B.m;

  // MATLAB Function: '<S7>/Body AeroCenter'
  rtb_TmpSignalConversionAtSFun_1 = rtb_Product3_o;

  // Sum: '<S7>/Add2' incorporates:
  //   DataTypeConversion: '<S8>/Data Type Conversion2'

  rtb_DataTypeConversion2[0] = rtb_Add;

  // MATLAB Function: '<S7>/Body AeroCenter' incorporates:
  //   Constant: '<S7>/ModelParam.uavCd'
  //   Product: '<S67>/Product1'

  rtb_Product3_o = -QuadModelv_P.ModelParam_uavCd * rtb_Windb[1] * std::abs
    (rtb_Windb[1]);

  // Sum: '<S7>/Add2'
  rtb_Add = rtb_DataTypeConversion2[1] + rtb_Product3_o;

  // DataTypeConversion: '<S8>/Data Type Conversion1' incorporates:
  //   Product: '<S12>/Divide'

  rtb_DataTypeConversion1[1] = rtb_Add / QuadModelv_B.m;

  // Sum: '<S7>/Add2' incorporates:
  //   Constant: '<S7>/ModelParam.uavCd'
  //   DataTypeConversion: '<S8>/Data Type Conversion2'
  //   MATLAB Function: '<S7>/Body AeroCenter'
  //   Product: '<S67>/Product1'

  rtb_DataTypeConversion2[1] = rtb_Add;
  rtb_Add = -QuadModelv_P.ModelParam_uavCd * rtb_Windb[2] * std::abs(rtb_Windb[2])
    + (rtb_DataTypeConversion2[2] + rtb_TmpSignalConversionAtSFun_0);

  // DataTypeConversion: '<S8>/Data Type Conversion1' incorporates:
  //   Product: '<S12>/Divide'

  rtb_DataTypeConversion1[2] = rtb_Add / QuadModelv_B.m;

  // Gain: '<S427>/Zero-Order Hold' incorporates:
  //   Integrator: '<S14>/p,q,r '

  rtb_DataTypeConversion_idx_0 = QuadModelv_P.ZeroOrderHold_Gain *
    QuadModelv_X.pqr_CSTATE[0];
  rtb_DataTypeConversion_idx_1 = QuadModelv_P.ZeroOrderHold_Gain *
    QuadModelv_X.pqr_CSTATE[1];
  rtb_Sum1_c = QuadModelv_P.ZeroOrderHold_Gain * QuadModelv_X.pqr_CSTATE[2];
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Gain: '<S427>/Gain' incorporates:
    //   Constant: '<S407>/center of gravity'
    //   Constant: '<S427>/wl_ins'
    //   Gain: '<S427>/Zero-Order Hold4'
    //   Sum: '<S427>/Sum7'

    QuadModelv_B.Gain_f[0] = (QuadModelv_P.ZeroOrderHold4_Gain *
      QuadModelv_P.centerofgravity_Value[0] -
      QuadModelv_P.ThreeaxisInertialMeasurementU_e[0]) *
      QuadModelv_P.Gain_Gain_hr[0];
    QuadModelv_B.Gain_f[1] = (QuadModelv_P.ZeroOrderHold4_Gain *
      QuadModelv_P.centerofgravity_Value[1] -
      QuadModelv_P.ThreeaxisInertialMeasurementU_e[1]) *
      QuadModelv_P.Gain_Gain_hr[1];
    QuadModelv_B.Gain_f[2] = (QuadModelv_P.ZeroOrderHold4_Gain *
      QuadModelv_P.centerofgravity_Value[2] -
      QuadModelv_P.ThreeaxisInertialMeasurementU_e[2]) *
      QuadModelv_P.Gain_Gain_hr[2];

    // Concatenate: '<S17>/Matrix Concatenation' incorporates:
    //   Constant: '<S1>/Constant_dI//dt'
    //   Constant: '<S6>/uavJ'
    //   Sum: '<S6>/Add6'

    for (i = 0; i < 3; i++) {
      rtb_MatrixConcatenation[6 * i] = QuadModelv_P.ModelParam_uavJ[3 * i] -
        rtb_uavlostJ[3 * i];

      // Sum: '<S6>/Add6' incorporates:
      //   Concatenate: '<S17>/Matrix Concatenation'
      //   Constant: '<S6>/uavJ'

      b_t = 3 * i + 1;
      rtb_MatrixConcatenation[6 * i + 1] = QuadModelv_P.ModelParam_uavJ[b_t] -
        rtb_uavlostJ[b_t];

      // Sum: '<S6>/Add6' incorporates:
      //   Concatenate: '<S17>/Matrix Concatenation'
      //   Constant: '<S6>/uavJ'

      b_t = 3 * i + 2;
      rtb_MatrixConcatenation[6 * i + 2] = QuadModelv_P.ModelParam_uavJ[b_t] -
        rtb_uavlostJ[b_t];
    }

    for (i = 0; i < 3; i++) {
      rtb_MatrixConcatenation[6 * i + 3] = QuadModelv_P.Constant_dIdt_Value[3 *
        i];
      rtb_MatrixConcatenation[6 * i + 4] = QuadModelv_P.Constant_dIdt_Value[3 *
        i + 1];
      rtb_MatrixConcatenation[6 * i + 5] = QuadModelv_P.Constant_dIdt_Value[3 *
        i + 2];
    }

    // End of Concatenate: '<S17>/Matrix Concatenation'
    for (i = 0; i < 3; i++) {
      // Selector: '<S16>/Selector1' incorporates:
      //   Concatenate: '<S17>/Matrix Concatenation'

      QuadModelv_B.Selector1[3 * i] = rtb_MatrixConcatenation[6 * i + 3];

      // Selector: '<S16>/Selector' incorporates:
      //   Concatenate: '<S17>/Matrix Concatenation'

      QuadModelv_B.Selector[3 * i] = rtb_MatrixConcatenation[6 * i];

      // Selector: '<S16>/Selector1' incorporates:
      //   Selector: '<S16>/Selector'

      b_t = 3 * i + 1;

      // Selector: '<S16>/Selector1' incorporates:
      //   Concatenate: '<S17>/Matrix Concatenation'

      QuadModelv_B.Selector1[b_t] = rtb_MatrixConcatenation[6 * i + 4];

      // Selector: '<S16>/Selector' incorporates:
      //   Concatenate: '<S17>/Matrix Concatenation'

      QuadModelv_B.Selector[b_t] = rtb_MatrixConcatenation[6 * i + 1];

      // Selector: '<S16>/Selector1' incorporates:
      //   Selector: '<S16>/Selector'

      b_t = 3 * i + 2;

      // Selector: '<S16>/Selector1' incorporates:
      //   Concatenate: '<S17>/Matrix Concatenation'

      QuadModelv_B.Selector1[b_t] = rtb_MatrixConcatenation[6 * i + 5];

      // Selector: '<S16>/Selector' incorporates:
      //   Concatenate: '<S17>/Matrix Concatenation'

      QuadModelv_B.Selector[b_t] = rtb_MatrixConcatenation[6 * i + 2];
    }
  }

  // Product: '<S439>/j x i'
  j = QuadModelv_B.Gain_f[0] * rtb_DataTypeConversion_idx_1;

  // Sum: '<S435>/Sum' incorporates:
  //   Product: '<S438>/i x j'
  //   Product: '<S438>/j x k'
  //   Product: '<S438>/k x i'
  //   Product: '<S439>/i x k'
  //   Product: '<S439>/k x j'

  rtb_Rn = rtb_DataTypeConversion_idx_1 * QuadModelv_B.Gain_f[2] -
    QuadModelv_B.Gain_f[1] * rtb_Sum1_c;
  rtb_fcn3 = QuadModelv_B.Gain_f[0] * rtb_Sum1_c - rtb_DataTypeConversion_idx_0 *
    QuadModelv_B.Gain_f[2];
  rtb_TmpSignalConversionAtSFun_0 = rtb_DataTypeConversion_idx_0 *
    QuadModelv_B.Gain_f[1] - j;

  // Product: '<S437>/j x i'
  j = rtb_Rn * rtb_DataTypeConversion_idx_1;

  // Sum: '<S434>/Sum' incorporates:
  //   Product: '<S436>/i x j'
  //   Product: '<S436>/j x k'
  //   Product: '<S436>/k x i'
  //   Product: '<S437>/i x k'
  //   Product: '<S437>/k x j'

  uavlostJzx = rtb_Rn * rtb_Sum1_c;
  rtb_DataTypeConversion_idx_2 = rtb_DataTypeConversion_idx_0 * rtb_fcn3;
  rtb_Rn = rtb_DataTypeConversion_idx_1 * rtb_TmpSignalConversionAtSFun_0 -
    rtb_fcn3 * rtb_Sum1_c;
  rtb_fcn3 = uavlostJzx - rtb_DataTypeConversion_idx_0 *
    rtb_TmpSignalConversionAtSFun_0;
  rtb_TmpSignalConversionAtSFun_0 = rtb_DataTypeConversion_idx_2 - j;

  // Product: '<S49>/Product' incorporates:
  //   Integrator: '<S14>/p,q,r '
  //   Selector: '<S16>/Selector'
  //   Sum: '<S427>/Sum4'

  for (i = 0; i < 3; i++) {
    rtb_Sum4_a[i] = (QuadModelv_B.Selector[i + 3] * QuadModelv_X.pqr_CSTATE[1] +
                     QuadModelv_B.Selector[i] * QuadModelv_X.pqr_CSTATE[0]) +
      QuadModelv_B.Selector[i + 6] * QuadModelv_X.pqr_CSTATE[2];
  }

  // End of Product: '<S49>/Product'

  // Product: '<S52>/j x i' incorporates:
  //   Integrator: '<S14>/p,q,r '

  j = rtb_Sum4_a[0] * QuadModelv_X.pqr_CSTATE[1];
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    for (i = 0; i < 3; i++) {
      // Selector: '<S16>/Selector2' incorporates:
      //   Concatenate: '<S17>/Matrix Concatenation'

      QuadModelv_B.Selector2[3 * i] = rtb_MatrixConcatenation[6 * i];
      QuadModelv_B.Selector2[3 * i + 1] = rtb_MatrixConcatenation[6 * i + 1];
      QuadModelv_B.Selector2[3 * i + 2] = rtb_MatrixConcatenation[6 * i + 2];
    }
  }

  // Product: '<S50>/Product' incorporates:
  //   Integrator: '<S14>/p,q,r '
  //   Selector: '<S16>/Selector1'

  for (i = 0; i < 3; i++) {
    rtb_Merge[i] = (QuadModelv_B.Selector1[i + 3] * QuadModelv_X.pqr_CSTATE[1] +
                    QuadModelv_B.Selector1[i] * QuadModelv_X.pqr_CSTATE[0]) +
      QuadModelv_B.Selector1[i + 6] * QuadModelv_X.pqr_CSTATE[2];
  }

  // Sum: '<S16>/Sum2' incorporates:
  //   Constant: '<S7>/ModelParam.uavCCm'
  //   Constant: '<S7>/ModelParam.uavDearo'
  //   Integrator: '<S14>/p,q,r '
  //   MATLAB Function: '<S7>/Body AeroCenter'
  //   Product: '<S50>/Product'
  //   Product: '<S51>/i x j'
  //   Product: '<S51>/j x k'
  //   Product: '<S51>/k x i'
  //   Product: '<S52>/i x k'
  //   Product: '<S52>/k x j'
  //   Sum: '<S1>/Sum'
  //   Sum: '<S48>/Sum'
  //   Sum: '<S7>/Add3'

  tmp[0] = (((((-QuadModelv_P.ModelParam_uavCCm[0] * QuadModelv_X.pqr_CSTATE[0] *
                std::abs(QuadModelv_X.pqr_CSTATE[0]) + rtb_Product3_o *
                QuadModelv_P.ModelParam_uavDearo) + (QuadModelv_B.M1[0] +
    rtb_Mb[0])) + QuadModelv_B.deltaM[0]) + QuadModelv_B.fm[3]) - rtb_Merge[0])
    - (QuadModelv_X.pqr_CSTATE[1] * rtb_Sum4_a[2] - rtb_Sum4_a[1] *
       QuadModelv_X.pqr_CSTATE[2]);
  tmp[1] = (((((-QuadModelv_P.ModelParam_uavCCm[1] * QuadModelv_X.pqr_CSTATE[1] *
                std::abs(QuadModelv_X.pqr_CSTATE[1]) +
                -rtb_TmpSignalConversionAtSFun_1 *
                QuadModelv_P.ModelParam_uavDearo) + (QuadModelv_B.M1[1] +
    rtb_Mb[1])) + QuadModelv_B.deltaM[1]) + QuadModelv_B.fm[4]) - rtb_Merge[1])
    - (rtb_Sum4_a[0] * QuadModelv_X.pqr_CSTATE[2] - QuadModelv_X.pqr_CSTATE[0] *
       rtb_Sum4_a[2]);
  tmp[2] = ((((-QuadModelv_P.ModelParam_uavCCm[2] * QuadModelv_X.pqr_CSTATE[2] *
               std::abs(QuadModelv_X.pqr_CSTATE[2]) + (QuadModelv_B.M1[2] +
    rtb_Mb[2])) + QuadModelv_B.deltaM[2]) + QuadModelv_B.fm[5]) - rtb_Merge[2])
    - (QuadModelv_X.pqr_CSTATE[0] * rtb_Sum4_a[1] - j);

  // Product: '<S16>/Product2' incorporates:
  //   Selector: '<S16>/Selector2'

  rt_mrdivide_U1d1x3_U2d_9vOrDY9Z(tmp, QuadModelv_B.Selector2,
    QuadModelv_B.Product2);

  // Gain: '<S427>/Zero-Order Hold3'
  rtb_Sum4_a[0] = QuadModelv_P.ZeroOrderHold3_Gain * QuadModelv_B.Product2[0];
  rtb_Sum4_a[1] = QuadModelv_P.ZeroOrderHold3_Gain * QuadModelv_B.Product2[1];
  rtb_Sum4_a[2] = QuadModelv_P.ZeroOrderHold3_Gain * QuadModelv_B.Product2[2];

  // Product: '<S441>/j x i'
  j = QuadModelv_B.Gain_f[0] * rtb_Sum4_a[1];

  // Product: '<S407>/Matrix Multiply1' incorporates:
  //   Concatenate: '<S36>/Vector Concatenate'

  for (i = 0; i < 3; i++) {
    rtb_Mb[i] = (VectorConcatenate[i + 3] * QuadModelv_B.MatrixConcatenate[1] +
                 VectorConcatenate[i] * QuadModelv_B.MatrixConcatenate[0]) +
      VectorConcatenate[i + 6] * QuadModelv_B.MatrixConcatenate[2];
  }

  // End of Product: '<S407>/Matrix Multiply1'

  // Sum: '<S427>/Sum' incorporates:
  //   Gain: '<S427>/Zero-Order Hold1'
  //   Gain: '<S427>/Zero-Order Hold2'
  //   Product: '<S440>/i x j'
  //   Product: '<S440>/j x k'
  //   Product: '<S440>/k x i'
  //   Product: '<S441>/i x k'
  //   Product: '<S441>/k x j'
  //   Sum: '<S431>/Sum'

  rtb_Product3_o = ((QuadModelv_P.ZeroOrderHold1_Gain * rtb_DataTypeConversion1
                     [0] - QuadModelv_P.ZeroOrderHold2_Gain * rtb_Mb[0]) +
                    rtb_Rn) + (rtb_Sum4_a[1] * QuadModelv_B.Gain_f[2] -
    QuadModelv_B.Gain_f[1] * rtb_Sum4_a[2]);
  rtb_TmpSignalConversionAtSFun_1 = ((QuadModelv_P.ZeroOrderHold1_Gain *
    rtb_DataTypeConversion1[1] - QuadModelv_P.ZeroOrderHold2_Gain * rtb_Mb[1]) +
    rtb_fcn3) + (QuadModelv_B.Gain_f[0] * rtb_Sum4_a[2] - rtb_Sum4_a[0] *
                 QuadModelv_B.Gain_f[2]);
  rtb_TmpSignalConversionAtSFun_0 = ((QuadModelv_P.ZeroOrderHold1_Gain *
    rtb_DataTypeConversion1[2] - QuadModelv_P.ZeroOrderHold2_Gain * rtb_Mb[2]) +
    rtb_TmpSignalConversionAtSFun_0) + (rtb_Sum4_a[0] * QuadModelv_B.Gain_f[1] -
    j);

  // Sum: '<S427>/Sum4' incorporates:
  //   Constant: '<S427>/Measurement bias'
  //   Constant: '<S427>/Scale factors & Cross-coupling  errors'
  //   Product: '<S427>/Product'

  for (i = 0; i < 3; i++) {
    rtb_Sum4_a[i] = ((QuadModelv_P.ThreeaxisInertialMeasurementU_h[i + 3] *
                      rtb_TmpSignalConversionAtSFun_1 +
                      QuadModelv_P.ThreeaxisInertialMeasurementU_h[i] *
                      rtb_Product3_o) +
                     QuadModelv_P.ThreeaxisInertialMeasurementU_h[i + 6] *
                     rtb_TmpSignalConversionAtSFun_0) +
      QuadModelv_P.ThreeaxisInertialMeasurementUni[i];
  }

  // End of Sum: '<S427>/Sum4'
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[2] == 0) {
    // Gain: '<S407>/Gain10' incorporates:
    //   UniformRandomNumber: '<S407>/Uniform Random Number5'

    QuadModelv_B.Gain10[0] = QuadModelv_P.Gain10_Gain *
      QuadModelv_DW.UniformRandomNumber5_NextOutput[0];
    QuadModelv_B.Gain10[1] = QuadModelv_P.Gain10_Gain *
      QuadModelv_DW.UniformRandomNumber5_NextOutput[1];
    QuadModelv_B.Gain10[2] = QuadModelv_P.Gain10_Gain *
      QuadModelv_DW.UniformRandomNumber5_NextOutput[2];
  }

  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // MATLAB Function: '<S407>/FaultParamsExtract' incorporates:
    //   Constant: '<S407>/FaultID'
    //   Inport: '<Root>/inSILInts'

    // MATLAB Function 'SensorFault/FaultParamsExtract': '<S410>:1'
    // '<S410>:1:5' if isempty(hFault)
    // '<S410>:1:8' if isempty(fParam)
    // '<S410>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S410>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S410>:1:14' j=1;
    j = 1.0;

    // '<S410>:1:15' for i=1:8
    for (b_t = 0; b_t < 8; b_t++) {
      // '<S410>:1:16' if inInts(i) == FaultID
      if (QuadModelv_U.inSILInts[b_t] == QuadModelv_P.FaultID_Value_f) {
        // '<S410>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S410>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        rtb_alt = (b_t + 1) << 1;
        fParamTmp[static_cast<int32_T>(2.0 * j - 1.0) - 1] = inSILFloats[rtb_alt
          - 2];

        // '<S410>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[static_cast<int32_T>(2.0 * j) - 1] = inSILFloats[rtb_alt - 1];

        // '<S410>:1:20' j=j+1;
        j++;
      }
    }

    // '<S410>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S410>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_n = true;

      // '<S410>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = inSILFloats[16];
      fParamTmp[17] = inSILFloats[17];
      fParamTmp[18] = inSILFloats[18];
      fParamTmp[19] = inSILFloats[19];

      // '<S410>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_ah[0], &fParamTmp[0], 20U * sizeof
                  (real_T));
    }

    // MATLAB Function: '<S408>/Acc NoiseFun' incorporates:
    //   MATLAB Function: '<S407>/FaultParamsExtract'

    // '<S410>:1:29' hasFault_acc=hFault;
    // '<S410>:1:30' FaultParam=fParam;
    QuadModelv_AccNoiseFun(QuadModelv_B.Gain10, QuadModelv_DW.hFault_n,
      QuadModelv_DW.fParam_ah, &QuadModelv_B.sf_AccNoiseFun_i);
  }

  // Switch: '<S429>/Switch' incorporates:
  //   Constant: '<S429>/Constant'
  //   SecondOrderIntegrator: '<S433>/Integrator, Second-Order Limited'

  if (QuadModelv_P.Constant_Value_iy >= QuadModelv_P.Switch_Threshold) {
    rtb_TmpSignalConversionAtSFun_0 =
      QuadModelv_X.IntegratorSecondOrderLimited_CS[0];
  } else {
    rtb_TmpSignalConversionAtSFun_0 = rtb_Sum4_a[0];
  }

  // Saturate: '<S427>/Saturation'
  if (rtb_TmpSignalConversionAtSFun_0 > QuadModelv_P.Saturation_UpperSat[0]) {
    rtb_TmpSignalConversionAtSFun_0 = QuadModelv_P.Saturation_UpperSat[0];
  } else if (rtb_TmpSignalConversionAtSFun_0 < QuadModelv_P.Saturation_LowerSat
             [0]) {
    rtb_TmpSignalConversionAtSFun_0 = QuadModelv_P.Saturation_LowerSat[0];
  }

  // Sum: '<S407>/Sum1'
  rtb_TmpSignalConversionAtSFun_1 = rtb_TmpSignalConversionAtSFun_0 +
    QuadModelv_B.sf_AccNoiseFun_i.y[0];

  // Switch: '<S429>/Switch' incorporates:
  //   Constant: '<S429>/Constant'
  //   SecondOrderIntegrator: '<S433>/Integrator, Second-Order Limited'

  if (QuadModelv_P.Constant_Value_iy >= QuadModelv_P.Switch_Threshold) {
    rtb_TmpSignalConversionAtSFun_0 =
      QuadModelv_X.IntegratorSecondOrderLimited_CS[1];
  } else {
    rtb_TmpSignalConversionAtSFun_0 = rtb_Sum4_a[1];
  }

  // Saturate: '<S427>/Saturation'
  if (rtb_TmpSignalConversionAtSFun_0 > QuadModelv_P.Saturation_UpperSat[1]) {
    rtb_TmpSignalConversionAtSFun_0 = QuadModelv_P.Saturation_UpperSat[1];
  } else if (rtb_TmpSignalConversionAtSFun_0 < QuadModelv_P.Saturation_LowerSat
             [1]) {
    rtb_TmpSignalConversionAtSFun_0 = QuadModelv_P.Saturation_LowerSat[1];
  }

  // Sum: '<S407>/Sum1'
  rtb_DataTypeConversion_idx_0 = rtb_TmpSignalConversionAtSFun_0 +
    QuadModelv_B.sf_AccNoiseFun_i.y[1];

  // Switch: '<S429>/Switch' incorporates:
  //   Constant: '<S429>/Constant'
  //   SecondOrderIntegrator: '<S433>/Integrator, Second-Order Limited'

  if (QuadModelv_P.Constant_Value_iy >= QuadModelv_P.Switch_Threshold) {
    rtb_TmpSignalConversionAtSFun_0 =
      QuadModelv_X.IntegratorSecondOrderLimited_CS[2];
  } else {
    rtb_TmpSignalConversionAtSFun_0 = rtb_Sum4_a[2];
  }

  // Saturate: '<S427>/Saturation'
  if (rtb_TmpSignalConversionAtSFun_0 > QuadModelv_P.Saturation_UpperSat[2]) {
    rtb_TmpSignalConversionAtSFun_0 = QuadModelv_P.Saturation_UpperSat[2];
  } else if (rtb_TmpSignalConversionAtSFun_0 < QuadModelv_P.Saturation_LowerSat
             [2]) {
    rtb_TmpSignalConversionAtSFun_0 = QuadModelv_P.Saturation_LowerSat[2];
  }

  // Sum: '<S407>/Sum1'
  rtb_DataTypeConversion_idx_1 = rtb_TmpSignalConversionAtSFun_0 +
    QuadModelv_B.sf_AccNoiseFun_i.y[2];

  // DataTypeConversion: '<S8>/Data Type Conversion1' incorporates:
  //   UnitConversion: '<S426>/Unit Conversion'

  // Unit Conversion - from: m/s^2 to: gn
  // Expression: output = (0.101972*input) + (0)
  rtb_DataTypeConversion1[0] *= 0.10197162129779282;
  rtb_DataTypeConversion1[1] *= 0.10197162129779282;
  rtb_DataTypeConversion1[2] *= 0.10197162129779282;

  // Sum: '<S428>/Sum4' incorporates:
  //   Constant: '<S428>/Measurement bias'
  //   Constant: '<S428>/Scale factors & Cross-coupling  errors '
  //   Constant: '<S428>/g-sensitive bias'
  //   Gain: '<S428>/Zero-Order Hold'
  //   Gain: '<S428>/Zero-Order Hold1'
  //   Integrator: '<S14>/p,q,r '
  //   Product: '<S428>/Product'
  //   Product: '<S428>/Product1'

  for (i = 0; i < 3; i++) {
    rtb_Mb[i] = (((QuadModelv_P.ZeroOrderHold_Gain_b * QuadModelv_X.pqr_CSTATE[0]
                   * QuadModelv_P.ThreeaxisInertialMeasurementU_m[i] +
                   QuadModelv_P.ThreeaxisInertialMeasurementU_m[i + 3] *
                   (QuadModelv_P.ZeroOrderHold_Gain_b * QuadModelv_X.pqr_CSTATE
                    [1])) + QuadModelv_P.ThreeaxisInertialMeasurementU_m[i + 6] *
                  (QuadModelv_P.ZeroOrderHold_Gain_b * QuadModelv_X.pqr_CSTATE[2]))
                 + QuadModelv_P.ThreeaxisInertialMeasurementU_b[i]) +
      QuadModelv_P.ZeroOrderHold1_Gain_i * rtb_DataTypeConversion1[i] *
      QuadModelv_P.ThreeaxisInertialMeasurementU_c[i];
  }

  // End of Sum: '<S428>/Sum4'
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[2] == 0) {
    // Gain: '<S407>/Gain6' incorporates:
    //   UniformRandomNumber: '<S407>/Uniform Random Number1'

    QuadModelv_B.Gain6[0] = QuadModelv_P.Gain6_Gain *
      QuadModelv_DW.UniformRandomNumber1_NextOutput[0];
    QuadModelv_B.Gain6[1] = QuadModelv_P.Gain6_Gain *
      QuadModelv_DW.UniformRandomNumber1_NextOutput[1];
    QuadModelv_B.Gain6[2] = QuadModelv_P.Gain6_Gain *
      QuadModelv_DW.UniformRandomNumber1_NextOutput[2];
  }

  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // MATLAB Function: '<S407>/FaultParamsExtract1' incorporates:
    //   Constant: '<S407>/FaultID1'
    //   Inport: '<Root>/inSILInts'

    // MATLAB Function 'SensorFault/FaultParamsExtract1': '<S411>:1'
    // '<S411>:1:5' if isempty(hFault)
    // '<S411>:1:8' if isempty(fParam)
    // '<S411>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S411>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S411>:1:14' j=1;
    j = 1.0;

    // '<S411>:1:15' for i=1:8
    for (b_t = 0; b_t < 8; b_t++) {
      // '<S411>:1:16' if inInts(i) == FaultID
      if (QuadModelv_U.inSILInts[b_t] == QuadModelv_P.FaultID1_Value_i) {
        // '<S411>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S411>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        fParamTmp[static_cast<int32_T>(2.0 * j - 1.0) - 1] = inSILFloats[((b_t +
          1) << 1) - 2];

        // '<S411>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[static_cast<int32_T>(2.0 * j) - 1] = inSILFloats[((b_t + 1) <<
          1) - 1];

        // '<S411>:1:20' j=j+1;
        j++;
      }
    }

    // '<S411>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S411>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_kf = true;

      // '<S411>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = inSILFloats[16];
      fParamTmp[17] = inSILFloats[17];
      fParamTmp[18] = inSILFloats[18];
      fParamTmp[19] = inSILFloats[19];

      // '<S411>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_ag[0], &fParamTmp[0], 20U * sizeof
                  (real_T));
    }

    // MATLAB Function: '<S414>/Acc NoiseFun' incorporates:
    //   MATLAB Function: '<S407>/FaultParamsExtract1'

    // '<S411>:1:29' hasFault_gyro=hFault;
    // '<S411>:1:30' FaultParam=fParam;
    QuadModelv_AccNoiseFun(QuadModelv_B.Gain6, QuadModelv_DW.hFault_kf,
      QuadModelv_DW.fParam_ag, &QuadModelv_B.sf_AccNoiseFun_a);
  }

  // Switch: '<S442>/Switch' incorporates:
  //   Constant: '<S442>/Constant'
  //   SecondOrderIntegrator: '<S444>/Integrator, Second-Order Limited'

  if (QuadModelv_P.Constant_Value_e >= QuadModelv_P.Switch_Threshold_p) {
    rtb_TmpSignalConversionAtSFun_0 =
      QuadModelv_X.IntegratorSecondOrderLimited__i[0];
  } else {
    rtb_TmpSignalConversionAtSFun_0 = rtb_Mb[0];
  }

  // Saturate: '<S428>/Saturation'
  if (rtb_TmpSignalConversionAtSFun_0 > QuadModelv_P.Saturation_UpperSat_j[0]) {
    rtb_DataTypeConversion_idx_2 = QuadModelv_P.Saturation_UpperSat_j[0];
  } else if (rtb_TmpSignalConversionAtSFun_0 <
             QuadModelv_P.Saturation_LowerSat_a[0]) {
    rtb_DataTypeConversion_idx_2 = QuadModelv_P.Saturation_LowerSat_a[0];
  } else {
    rtb_DataTypeConversion_idx_2 = rtb_TmpSignalConversionAtSFun_0;
  }

  // Switch: '<S442>/Switch' incorporates:
  //   Constant: '<S442>/Constant'
  //   SecondOrderIntegrator: '<S444>/Integrator, Second-Order Limited'

  if (QuadModelv_P.Constant_Value_e >= QuadModelv_P.Switch_Threshold_p) {
    rtb_TmpSignalConversionAtSFun_0 =
      QuadModelv_X.IntegratorSecondOrderLimited__i[1];
  } else {
    rtb_TmpSignalConversionAtSFun_0 = rtb_Mb[1];
  }

  // Saturate: '<S428>/Saturation'
  if (rtb_TmpSignalConversionAtSFun_0 > QuadModelv_P.Saturation_UpperSat_j[1]) {
    y = QuadModelv_P.Saturation_UpperSat_j[1];
  } else if (rtb_TmpSignalConversionAtSFun_0 <
             QuadModelv_P.Saturation_LowerSat_a[1]) {
    y = QuadModelv_P.Saturation_LowerSat_a[1];
  } else {
    y = rtb_TmpSignalConversionAtSFun_0;
  }

  // Switch: '<S442>/Switch' incorporates:
  //   Constant: '<S442>/Constant'
  //   SecondOrderIntegrator: '<S444>/Integrator, Second-Order Limited'

  if (QuadModelv_P.Constant_Value_e >= QuadModelv_P.Switch_Threshold_p) {
    rtb_TmpSignalConversionAtSFun_0 =
      QuadModelv_X.IntegratorSecondOrderLimited__i[2];
  } else {
    rtb_TmpSignalConversionAtSFun_0 = rtb_Mb[2];
  }

  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Sum: '<S454>/Sum' incorporates:
    //   Constant: '<S420>/Decimal Year'
    //   Constant: '<S454>/epoch'

    rtb_Sum_o1 = QuadModelv_P.DecimalYear_Value - QuadModelv_P.epoch_Value;

    // RelationalOperator: '<S468>/Relational Operator' incorporates:
    //   Constant: '<S420>/Decimal Year'
    //   Memory: '<S468>/otime'

    rtb_RelationalOperator = (QuadModelv_P.DecimalYear_Value !=
      QuadModelv_DW.otime_PreviousInput);
  }

  // Switch: '<S459>/Switch' incorporates:
  //   Abs: '<S459>/Abs'
  //   Bias: '<S459>/Bias'
  //   Bias: '<S459>/Bias1'
  //   Constant: '<S459>/Constant2'
  //   Constant: '<S460>/Constant'
  //   Math: '<S459>/Math Function1'
  //   RelationalOperator: '<S460>/Compare'

  if (std::abs(LoadLeakRatio) > QuadModelv_P.CompareToConstant_const_n) {
    rtb_Product3_o = rt_modd_snf(LoadLeakRatio + QuadModelv_P.Bias_Bias_el,
      QuadModelv_P.Constant2_Value_e) + QuadModelv_P.Bias1_Bias_jm;
  } else {
    rtb_Product3_o = LoadLeakRatio;
  }

  // End of Switch: '<S459>/Switch'

  // Abs: '<S456>/Abs1'
  rtb_fcn3 = std::abs(rtb_Product3_o);

  // RelationalOperator: '<S458>/Compare' incorporates:
  //   Constant: '<S458>/Constant'

  hFaultTmp = (rtb_fcn3 > QuadModelv_P.CompareToConstant_const_a);

  // Switch: '<S450>/Switch1' incorporates:
  //   Constant: '<S450>/Constant'
  //   Constant: '<S450>/Constant1'

  if (hFaultTmp) {
    rtb_Rn = QuadModelv_P.Constant_Value_go;
  } else {
    rtb_Rn = QuadModelv_P.Constant1_Value_a;
  }

  // End of Switch: '<S450>/Switch1'

  // Sum: '<S450>/Sum'
  rtb_Rn += Y;

  // Switch: '<S457>/Switch' incorporates:
  //   Abs: '<S457>/Abs'
  //   Constant: '<S461>/Constant'
  //   RelationalOperator: '<S461>/Compare'

  if (std::abs(rtb_Rn) > QuadModelv_P.CompareToConstant_const_e) {
    // Switch: '<S457>/Switch' incorporates:
    //   Bias: '<S457>/Bias'
    //   Bias: '<S457>/Bias1'
    //   Constant: '<S457>/Constant2'
    //   Math: '<S457>/Math Function1'

    QuadModelv_B.Switch_p = rt_modd_snf(rtb_Rn + QuadModelv_P.Bias_Bias_b,
      QuadModelv_P.Constant2_Value_c) + QuadModelv_P.Bias1_Bias_p;
  } else {
    // Switch: '<S457>/Switch'
    QuadModelv_B.Switch_p = rtb_Rn;
  }

  // End of Switch: '<S457>/Switch'

  // Switch: '<S456>/Switch'
  if (hFaultTmp) {
    // Signum: '<S456>/Sign1'
    if (!rtIsNaN(rtb_Product3_o)) {
      if (rtb_Product3_o < 0.0) {
        rtb_Product3_o = -1.0;
      } else {
        rtb_Product3_o = (rtb_Product3_o > 0.0);
      }
    }

    // End of Signum: '<S456>/Sign1'

    // Switch: '<S456>/Switch' incorporates:
    //   Bias: '<S456>/Bias'
    //   Bias: '<S456>/Bias1'
    //   Gain: '<S456>/Gain'
    //   Product: '<S456>/Divide1'

    QuadModelv_B.Switch_l = ((rtb_fcn3 + QuadModelv_P.Bias_Bias_c0) *
      QuadModelv_P.Gain_Gain_kn + QuadModelv_P.Bias1_Bias_j) * rtb_Product3_o;
  } else {
    // Switch: '<S456>/Switch'
    QuadModelv_B.Switch_l = rtb_Product3_o;
  }

  // End of Switch: '<S456>/Switch'

  // UnitConversion: '<S512>/Unit Conversion'
  // Unit Conversion - from: deg to: rad
  // Expression: output = (0.0174533*input) + (0)
  uavlostJzx = 0.017453292519943295 * QuadModelv_B.Switch_p;
  rtb_fcn3 = 0.017453292519943295 * QuadModelv_B.Switch_l;

  // Trigonometry: '<S465>/sincos'
  j = std::sin(uavlostJzx);
  Z = std::cos(uavlostJzx);
  rtb_q2dot = std::sin(rtb_fcn3);
  rtb_q3dot = std::cos(rtb_fcn3);
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Outputs for Enabled SubSystem: '<S454>/Convert from geodetic to  spherical coordinates ' incorporates:
    //   EnablePort: '<S464>/Enable'

    if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
      // RelationalOperator: '<S467>/Relational Operator' incorporates:
      //   Memory: '<S467>/olon'

      QuadModelv_DW.Convertfromgeodetictosphericalc = (QuadModelv_B.Switch_p !=
        QuadModelv_DW.olon_PreviousInput);
    }

    // End of Outputs for SubSystem: '<S454>/Convert from geodetic to  spherical coordinates ' 

    // Memory: '<S466>/olat'
    QuadModelv_B.olat = QuadModelv_DW.olat_PreviousInput;
  }

  // Outputs for Enabled SubSystem: '<S454>/Convert from geodetic to  spherical coordinates ' incorporates:
  //   EnablePort: '<S464>/Enable'

  if (QuadModelv_DW.Convertfromgeodetictosphericalc) {
    // SignalConversion generated from: '<S464>/sp[2]'
    rtb_Product3_o = j;
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
      // Outputs for Iterator SubSystem: '<S464>/For Iterator Subsystem' incorporates:
      //   ForIterator: '<S511>/For Iterator'

      for (b_t = 1; b_t <= QuadModelv_P.ForIterator_IterationLimit; b_t++) {
        // Switch: '<S511>/cp[m-1] sp[m-1]' incorporates:
        //   SignalConversion generated from: '<S464>/cp[2]'
        //   UnitDelay: '<S511>/Unit Delay1'

        if (b_t > QuadModelv_P.cpm1spm1_Threshold) {
          uavlostJzx = QuadModelv_DW.UnitDelay1_DSTATE[0];
          rtb_fcn3 = QuadModelv_DW.UnitDelay1_DSTATE[1];
        } else {
          uavlostJzx = Z;
          rtb_fcn3 = rtb_Product3_o;
        }

        // End of Switch: '<S511>/cp[m-1] sp[m-1]'

        // Sum: '<S511>/Sum2' incorporates:
        //   Product: '<S511>/Product1'
        //   Product: '<S511>/Product2'
        //   SignalConversion generated from: '<S464>/cp[2]'

        rtb_Rn = uavlostJzx * rtb_Product3_o + rtb_fcn3 * Z;

        // Assignment: '<S511>/Assignment' incorporates:
        //   Constant: '<S511>/Constant'

        if (b_t == 1) {
          std::memcpy(&rtb_Assignment[0], &QuadModelv_P.Constant_Value_lw[0],
                      11U * sizeof(real_T));
        }

        rtb_Assignment[b_t - 1] = rtb_Rn;

        // End of Assignment: '<S511>/Assignment'

        // Sum: '<S511>/Sum1' incorporates:
        //   Product: '<S511>/Product3'
        //   Product: '<S511>/Product8'
        //   SignalConversion generated from: '<S464>/cp[2]'

        rtb_Sum1_c = uavlostJzx * Z - rtb_fcn3 * rtb_Product3_o;

        // Assignment: '<S511>/Assignment1' incorporates:
        //   Constant: '<S511>/Constant1'

        if (b_t == 1) {
          std::memcpy(&rtb_Assignment1[0], &QuadModelv_P.Constant1_Value_n1[0],
                      11U * sizeof(real_T));
        }

        rtb_Assignment1[b_t - 1] = rtb_Sum1_c;

        // End of Assignment: '<S511>/Assignment1'

        // Update for UnitDelay: '<S511>/Unit Delay1'
        QuadModelv_DW.UnitDelay1_DSTATE[0] = rtb_Sum1_c;
        QuadModelv_DW.UnitDelay1_DSTATE[1] = rtb_Rn;
      }

      // End of Outputs for SubSystem: '<S464>/For Iterator Subsystem'
      for (i = 0; i < 11; i++) {
        // Gain: '<S464>/Gain'
        QuadModelv_B.Gain_i[i] = QuadModelv_P.Gain_Gain_a * rtb_Assignment1[i];
      }

      for (i = 0; i < 11; i++) {
        // Gain: '<S464>/Gain1'
        QuadModelv_B.Gain1[i] = QuadModelv_P.Gain1_Gain_n * rtb_Assignment[i];
      }
    }

    // SignalConversion generated from: '<S464>/cp[13]' incorporates:
    //   Constant: '<S464>/cp[1]'
    //   SignalConversion generated from: '<S464>/cp[2]'

    QuadModelv_B.OutportBufferForcp13[0] = QuadModelv_P.cp1_Value;
    QuadModelv_B.OutportBufferForcp13[1] = Z;
    std::memcpy(&QuadModelv_B.OutportBufferForcp13[2], &QuadModelv_B.Gain_i[0],
                11U * sizeof(real_T));

    // SignalConversion generated from: '<S464>/sp[13]' incorporates:
    //   Constant: '<S464>/sp[1]'
    //   SignalConversion generated from: '<S464>/sp[2]'

    QuadModelv_B.OutportBufferForsp13[0] = QuadModelv_P.sp1_Value;
    QuadModelv_B.OutportBufferForsp13[1] = j;
    std::memcpy(&QuadModelv_B.OutportBufferForsp13[2], &QuadModelv_B.Gain1[0],
                11U * sizeof(real_T));
  }

  // End of Outputs for SubSystem: '<S454>/Convert from geodetic to  spherical coordinates ' 

  // Gain: '<S420>/Gain'
  QuadModelv_B.Gain_f0 = QuadModelv_P.Gain_Gain_d0 * rtb_Saturation_1;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Memory: '<S466>/oalt'
    QuadModelv_B.oalt = QuadModelv_DW.oalt_PreviousInput;
  }

  // Logic: '<S466>/Logical Operator' incorporates:
  //   RelationalOperator: '<S466>/Relational Operator'
  //   RelationalOperator: '<S466>/Relational Operator1'

  rtb_hasFault = ((QuadModelv_B.Switch_l != QuadModelv_B.olat) ||
                  (QuadModelv_B.Gain_f0 != QuadModelv_B.oalt));

  // Product: '<S465>/Product'
  Z = rtb_q2dot * rtb_q2dot;

  // Product: '<S465>/Product1'
  uavlostJzx = rtb_q3dot * rtb_q3dot;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Outputs for Enabled SubSystem: '<S454>/Convert from geodetic to  spherical coordinates' incorporates:
    //   EnablePort: '<S463>/Enable'

    if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
      // SignalConversion generated from: '<S463>/Enable'
      QuadModelv_DW.Convertfromgeodetictospherica_g = rtb_hasFault;
    }

    // End of Outputs for SubSystem: '<S454>/Convert from geodetic to  spherical coordinates' 
  }

  // Outputs for Enabled SubSystem: '<S454>/Convert from geodetic to  spherical coordinates' incorporates:
  //   EnablePort: '<S463>/Enable'

  if (QuadModelv_DW.Convertfromgeodetictospherica_g) {
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
      // Product: '<S463>/b2' incorporates:
      //   Constant: '<S463>/b'

      QuadModelv_B.b2 = QuadModelv_P.b_Value * QuadModelv_P.b_Value;

      // Product: '<S463>/a2' incorporates:
      //   Constant: '<S463>/a'

      QuadModelv_B.a2 = QuadModelv_P.a_Value * QuadModelv_P.a_Value;

      // Sum: '<S506>/Sum1'
      QuadModelv_B.c2 = QuadModelv_B.a2 - QuadModelv_B.b2;
    }

    // Sqrt: '<S506>/sqrt' incorporates:
    //   Product: '<S506>/Product'
    //   Sum: '<S506>/Sum'

    rtb_Rn = std::sqrt(QuadModelv_B.a2 - Z * QuadModelv_B.c2);

    // Product: '<S463>/Product1'
    rtb_Sum1_c = rtb_Rn * QuadModelv_B.Gain_f0;

    // Sqrt: '<S505>/sqrt' incorporates:
    //   Product: '<S505>/Product10'
    //   Product: '<S505>/Product9'
    //   Sum: '<S505>/Sum7'

    rtb_Product3_o = std::sqrt(uavlostJzx * QuadModelv_B.a2 + QuadModelv_B.b2 *
      Z);
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
      // Product: '<S508>/a4'
      QuadModelv_B.a4 = QuadModelv_B.a2 * QuadModelv_B.a2;

      // Sum: '<S508>/Sum9' incorporates:
      //   Product: '<S508>/b4'

      QuadModelv_B.c4 = QuadModelv_B.a4 - QuadModelv_B.b2 * QuadModelv_B.b2;
    }

    // Sqrt: '<S463>/sqrt' incorporates:
    //   Gain: '<S508>/Gain'
    //   Product: '<S508>/Product1'
    //   Product: '<S508>/Product6'
    //   Product: '<S508>/Product7'
    //   Product: '<S508>/Product8'
    //   Sum: '<S508>/Sum5'
    //   Sum: '<S508>/Sum6'

    QuadModelv_B.sqrt_o = std::sqrt((QuadModelv_B.a4 - QuadModelv_B.c4 * Z) /
      (rtb_Rn * rtb_Rn) + (QuadModelv_P.Gain_Gain_j * rtb_Sum1_c +
      QuadModelv_B.Gain_f0 * QuadModelv_B.Gain_f0));

    // Product: '<S503>/Product11' incorporates:
    //   Sum: '<S503>/Sum8'

    QuadModelv_B.Product11 = (QuadModelv_B.Gain_f0 + rtb_Product3_o) /
      QuadModelv_B.sqrt_o;

    // Sum: '<S507>/Sum2'
    rtb_Rn = QuadModelv_B.a2 + rtb_Sum1_c;

    // Sum: '<S507>/Sum1'
    rtb_Sum1_c += QuadModelv_B.b2;

    // Product: '<S504>/Product4' incorporates:
    //   Product: '<S504>/Product3'
    //   Product: '<S507>/Product1'
    //   Product: '<S507>/Product2'
    //   Sqrt: '<S504>/sqrt'
    //   Sum: '<S504>/Sum3'

    QuadModelv_B.Product4_p = rtb_q2dot / std::sqrt(rtb_Rn * rtb_Rn /
      (rtb_Sum1_c * rtb_Sum1_c) * uavlostJzx + Z);

    // Product: '<S509>/Product1'
    rtb_Product3_o *= QuadModelv_B.sqrt_o;
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
      // Sum: '<S509>/Sum1'
      QuadModelv_B.c2_h = QuadModelv_B.a2 - QuadModelv_B.b2;
    }

    // Product: '<S509>/Product12'
    QuadModelv_B.Product12 = QuadModelv_B.c2_h / rtb_Product3_o * rtb_q3dot *
      rtb_q2dot;

    // Sqrt: '<S510>/sqrt' incorporates:
    //   Constant: '<S510>/Constant'
    //   Product: '<S510>/Product5'
    //   Sum: '<S510>/Sum4'

    QuadModelv_B.sqrt_d = std::sqrt(QuadModelv_P.Constant_Value_nh -
      QuadModelv_B.Product4_p * QuadModelv_B.Product4_p);
  }

  // End of Outputs for SubSystem: '<S454>/Convert from geodetic to  spherical coordinates' 

  // Product: '<S454>/aor' incorporates:
  //   Constant: '<S454>/re'

  uavlostJzx = QuadModelv_P.re_Value / QuadModelv_B.sqrt_o;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Outputs for Iterator SubSystem: '<S454>/Compute magnetic vector in spherical coordinates' incorporates:
    //   ForIterator: '<S462>/For Iterator'

    // InitializeConditions for UnitDelay: '<S462>/Unit Delay'
    Z = QuadModelv_P.UnitDelay_InitialCondition_pm;

    // InitializeConditions for UnitDelay: '<S462>/Unit Delay2'
    tz[0] = QuadModelv_P.UnitDelay2_InitialCondition_j[0];
    tz[1] = QuadModelv_P.UnitDelay2_InitialCondition_j[1];
    tz[2] = QuadModelv_P.UnitDelay2_InitialCondition_j[2];
    tz[3] = QuadModelv_P.UnitDelay2_InitialCondition_j[3];
    for (r = 1; r <= QuadModelv_P.ForIterator_IterationLimit_h; r++) {
      // Switch: '<S462>/ar(n)' incorporates:
      //   Product: '<S454>/ar'

      if (r <= QuadModelv_P.arn_Threshold) {
        Z = uavlostJzx * uavlostJzx;
      }

      // End of Switch: '<S462>/ar(n)'

      // Product: '<S462>/Product8'
      rtb_Product3_o = Z * uavlostJzx;

      // Sum: '<S462>/Sum' incorporates:
      //   Constant: '<S462>/Constant'

      if ((r < 0) && (QuadModelv_P.Constant_Value_lq < MIN_int32_T - r)) {
        b_t = MIN_int32_T;
      } else if ((r > 0) && (QuadModelv_P.Constant_Value_lq > MAX_int32_T - r))
      {
        b_t = MAX_int32_T;
      } else {
        b_t = r + QuadModelv_P.Constant_Value_lq;
      }

      // Outputs for Iterator SubSystem: '<S462>/For Iterator Subsystem' incorporates:
      //   ForIterator: '<S470>/For Iterator'

      // InitializeConditions for UnitDelay: '<S471>/Unit Delay1'
      rtb_fcn3 = QuadModelv_P.UnitDelay1_InitialCondition_g;

      // InitializeConditions for UnitDelay: '<S471>/Unit Delay3'
      rtb_Rn = QuadModelv_P.UnitDelay3_InitialCondition;

      // InitializeConditions for UnitDelay: '<S471>/Unit Delay2'
      rtb_Sum1_c = QuadModelv_P.UnitDelay2_InitialCondition;

      // InitializeConditions for UnitDelay: '<S471>/Unit Delay4'
      j = QuadModelv_P.UnitDelay4_InitialCondition;
      for (i = 0; i < 6; i++) {
        ForIterator_IterationMarker[i] = 1U;
      }

      // Sum: '<S462>/Sum'
      if (b_t > 2147483646) {
        b_t = 2147483646;
      } else if (b_t < 0) {
        b_t = 0;
      }

      if (b_t >= 1) {
        // Selector: '<S471>/snorm[n+m*13]' incorporates:
        //   Constant: '<S475>/Constant'
        //   Sum: '<S475>/Sum1'

        if ((QuadModelv_P.Constant_Value_pe < 0) && (r < MIN_int32_T
             - QuadModelv_P.Constant_Value_pe)) {
          loop_ub = MIN_int32_T;
        } else if ((QuadModelv_P.Constant_Value_pe > 0) && (r > MAX_int32_T
                    - QuadModelv_P.Constant_Value_pe)) {
          loop_ub = MAX_int32_T;
        } else {
          loop_ub = QuadModelv_P.Constant_Value_pe + r;
        }

        // Sum: '<S471>/Sum' incorporates:
        //   Constant: '<S471>/Constant'

        if ((r < 0) && (QuadModelv_P.Constant_Value_j < MIN_int32_T - r)) {
          r_0 = MIN_int32_T;
        } else if ((r > 0) && (QuadModelv_P.Constant_Value_j > MAX_int32_T - r))
        {
          r_0 = MAX_int32_T;
        } else {
          r_0 = r + QuadModelv_P.Constant_Value_j;
        }

        // Sum: '<S471>/Sum4' incorporates:
        //   Constant: '<S471>/Constant1'

        if ((r < 0) && (QuadModelv_P.Constant1_Value_ja < MIN_int32_T - r)) {
          r_1 = MIN_int32_T;
        } else if ((r > 0) && (QuadModelv_P.Constant1_Value_ja > MAX_int32_T - r))
        {
          r_1 = MAX_int32_T;
        } else {
          r_1 = r + QuadModelv_P.Constant1_Value_ja;
        }
      }

      for (rtb_alt = 1; rtb_alt <= b_t; rtb_alt++) {
        int32_T qY;
        int32_T qY_0;
        int32_T qY_1;
        int32_T rtb_Sum1_mx;

        // Sum: '<S470>/Sum1' incorporates:
        //   Constant: '<S470>/Constant'

        if (QuadModelv_P.Constant_Value_cy < rtb_alt - MAX_int32_T) {
          qY_1 = MAX_int32_T;
        } else {
          qY_1 = rtb_alt - QuadModelv_P.Constant_Value_cy;
        }

        // Outputs for Enabled SubSystem: '<S470>/Time adjust the gauss coefficients' incorporates:
        //   EnablePort: '<S473>/Enable'

        if (rtb_RelationalOperator) {
          // Outputs for Atomic SubSystem: '<S473>/If Action Subsystem'
          // Sum: '<S499>/Sum1' incorporates:
          //   Constant: '<S499>/Constant1'
          //   Sum: '<S470>/Sum1'

          if ((qY_1 < 0) && (QuadModelv_P.Constant1_Value_g < MIN_int32_T - qY_1))
          {
            qY_0 = MIN_int32_T;
          } else if ((qY_1 > 0) && (QuadModelv_P.Constant1_Value_g > MAX_int32_T
                      - qY_1)) {
            qY_0 = MAX_int32_T;
          } else {
            qY_0 = qY_1 + QuadModelv_P.Constant1_Value_g;
          }

          // Sum: '<S499>/Sum2' incorporates:
          //   Constant: '<S499>/Constant'

          if ((r < 0) && (QuadModelv_P.Constant_Value_g0 < MIN_int32_T - r)) {
            qY = MIN_int32_T;
          } else if ((r > 0) && (QuadModelv_P.Constant_Value_g0 > MAX_int32_T
                                 - r)) {
            qY = MAX_int32_T;
          } else {
            qY = r + QuadModelv_P.Constant_Value_g0;
          }

          // End of Outputs for SubSystem: '<S473>/If Action Subsystem'

          // Assignment: '<S473>/Assignment'
          if (ForIterator_IterationMarker[4] < 2) {
            ForIterator_IterationMarker[4] = 2U;

            // Assignment: '<S473>/Assignment' incorporates:
            //   UnitDelay: '<S473>/Unit Delay'

            std::memcpy(&QuadModelv_B.Assignment[0],
                        &QuadModelv_DW.UnitDelay_DSTATE_k[0], 169U * sizeof
                        (real_T));
          }

          // Outputs for Atomic SubSystem: '<S473>/If Action Subsystem'
          // Selector: '<S499>/cd[m][n]' incorporates:
          //   Assignment: '<S473>/Assignment'
          //   Constant: '<S473>/cd[maxdef][maxdef]'
          //   Selector: '<S499>/c[m][n]'
          //   Sum: '<S499>/Sum1'
          //   Sum: '<S499>/Sum2'

          qY_0 = ((qY - 1) * 13 + qY_0) - 1;

          // Assignment: '<S473>/Assignment' incorporates:
          //   Constant: '<S473>/c[maxdef][maxdef]'
          //   Constant: '<S473>/cd[maxdef][maxdef]'
          //   Product: '<S499>/Product'
          //   Selector: '<S499>/c[m][n]'
          //   Selector: '<S499>/cd[m][n]'
          //   Sum: '<S499>/Sum'

          QuadModelv_B.Assignment[qY_0] = QuadModelv_P.cdmaxdefmaxdef_Value[qY_0]
            * rtb_Sum_o1 + QuadModelv_P.cmaxdefmaxdef_Value[qY_0];

          // End of Outputs for SubSystem: '<S473>/If Action Subsystem'

          // Switch: '<S500>/tc_old' incorporates:
          //   Constant: '<S500>/zeros(maxdef+1,maxdef+1)'
          //   UnitDelay: '<S500>/Unit Delay'

          if (r > QuadModelv_P.tc_old_Threshold) {
            std::memcpy(&rtb_tc_old[0], &QuadModelv_DW.UnitDelay_DSTATE_b[0],
                        169U * sizeof(real_T));
          } else {
            std::memcpy(&rtb_tc_old[0], &QuadModelv_P.zerosmaxdef1maxdef1_Value
                        [0], 169U * sizeof(real_T));
          }

          // End of Switch: '<S500>/tc_old'

          // If: '<S500>/If' incorporates:
          //   Sum: '<S470>/Sum1'

          if (qY_1 != 0) {
            // Outputs for IfAction SubSystem: '<S500>/If Action Subsystem1' incorporates:
            //   ActionPort: '<S501>/Action Port'

            // Sum: '<S501>/Sum2' incorporates:
            //   Constant: '<S501>/Constant'

            if ((r < 0) && (QuadModelv_P.Constant_Value_nf < MIN_int32_T - r)) {
              qY_0 = MIN_int32_T;
            } else if ((r > 0) && (QuadModelv_P.Constant_Value_nf > MAX_int32_T
                                   - r)) {
              qY_0 = MAX_int32_T;
            } else {
              qY_0 = r + QuadModelv_P.Constant_Value_nf;
            }

            // Assignment: '<S501>/Assignment2'
            if (ForIterator_IterationMarker[5] < 2) {
              ForIterator_IterationMarker[5] = 2U;

              // Assignment: '<S501>/Assignment2' incorporates:
              //   Switch: '<S500>/tc_old'

              std::memcpy(&QuadModelv_B.Assignment2[0], &rtb_tc_old[0], 169U *
                          sizeof(real_T));
            }

            // Selector: '<S501>/c[m][n]' incorporates:
            //   Assignment: '<S501>/Assignment2'
            //   Constant: '<S473>/cd[maxdef][maxdef]'
            //   Selector: '<S501>/cd[m][n]'
            //   Sum: '<S501>/Sum2'

            qY_0 = ((qY_1 - 1) * 13 + qY_0) - 1;

            // Assignment: '<S501>/Assignment2' incorporates:
            //   Constant: '<S473>/c[maxdef][maxdef]'
            //   Constant: '<S473>/cd[maxdef][maxdef]'
            //   Product: '<S501>/Product'
            //   Selector: '<S501>/c[m][n]'
            //   Selector: '<S501>/cd[m][n]'
            //   Sum: '<S501>/Sum'

            QuadModelv_B.Assignment2[qY_0] =
              QuadModelv_P.cdmaxdefmaxdef_Value[qY_0] * rtb_Sum_o1 +
              QuadModelv_P.cmaxdefmaxdef_Value[qY_0];

            // Gain: '<S501>/Gain' incorporates:
            //   Assignment: '<S501>/Assignment2'
            //   Merge: '<S500>/Merge'

            for (i = 0; i < 169; i++) {
              rtb_tc_old[i] = QuadModelv_P.Gain_Gain_ps *
                QuadModelv_B.Assignment2[i];
            }

            // End of Gain: '<S501>/Gain'
            // End of Outputs for SubSystem: '<S500>/If Action Subsystem1'
          }

          // End of If: '<S500>/If'
          for (i = 0; i < 169; i++) {
            // Sum: '<S473>/Sum2' incorporates:
            //   Merge: '<S500>/Merge'

            Z = rtb_tc_old[i];

            // Sum: '<S473>/Sum2' incorporates:
            //   Assignment: '<S473>/Assignment'

            QuadModelv_B.Sum2_h[i] = QuadModelv_B.Assignment[i] + Z;

            // Update for UnitDelay: '<S473>/Unit Delay' incorporates:
            //   Assignment: '<S473>/Assignment'

            QuadModelv_DW.UnitDelay_DSTATE_k[i] = QuadModelv_B.Assignment[i];

            // Update for UnitDelay: '<S500>/Unit Delay'
            QuadModelv_DW.UnitDelay_DSTATE_b[i] = Z;
          }
        }

        // End of Outputs for SubSystem: '<S470>/Time adjust the gauss coefficients' 

        // Sum: '<S476>/Sum4' incorporates:
        //   Constant: '<S476>/Constant1'
        //   Sum: '<S470>/Sum1'

        Z = static_cast<real_T>(qY_1) + QuadModelv_P.Constant1_Value_n;

        // If: '<S476>/If' incorporates:
        //   Sum: '<S470>/Sum1'

        if (qY_1 == 0) {
          // Outputs for IfAction SubSystem: '<S476>/If Action Subsystem' incorporates:
          //   ActionPort: '<S482>/Action Port'

          // Product: '<S482>/Product' incorporates:
          //   Constant: '<S482>/Constant'
          //   Selector: '<S482>/Selector'
          //   Sum: '<S473>/Sum2'
          //   Sum: '<S482>/Sum'

          rtb_q2dot_tmp = QuadModelv_B.Sum2_h[((static_cast<int32_T>(
            static_cast<real_T>(r) + QuadModelv_P.Constant_Value_g) - 1) * 13 +
            static_cast<int32_T>(QuadModelv_P.Constant_Value_g)) - 1];

          // Merge: '<S476>/Merge' incorporates:
          //   Gain: '<S482>/Gain1'
          //   Product: '<S482>/Product'
          //   Selector: '<S476>/cp[m+1]'
          //   Selector: '<S482>/Selector'

          rtb_q2dot = QuadModelv_B.OutportBufferForcp13[static_cast<int32_T>(Z)
            - 1] * rtb_q2dot_tmp * QuadModelv_P.Gain1_Gain;

          // Merge: '<S476>/Merge1' incorporates:
          //   Gain: '<S482>/Gain2'
          //   Product: '<S482>/Product'
          //   Selector: '<S476>/cp[m+1]'
          //   Selector: '<S476>/sp[m+1]'

          rtb_q3dot = QuadModelv_B.OutportBufferForsp13[static_cast<int32_T>(Z)
            - 1] * rtb_q2dot_tmp * QuadModelv_P.Gain2_Gain;

          // End of Outputs for SubSystem: '<S476>/If Action Subsystem'
        } else {
          // Outputs for IfAction SubSystem: '<S476>/If Action Subsystem1' incorporates:
          //   ActionPort: '<S483>/Action Port'

          // Product: '<S483>/Product1' incorporates:
          //   Constant: '<S485>/Constant'
          //   Product: '<S483>/Product'
          //   Selector: '<S476>/cp[m+1]'
          //   Selector: '<S483>/Selector1'
          //   Sum: '<S473>/Sum2'
          //   Sum: '<S485>/Sum'

          rtb_q3dot = QuadModelv_B.Sum2_h[((qY_1 - 1) * 13 + static_cast<int32_T>
            (static_cast<real_T>(r) + QuadModelv_P.Constant_Value_k4)) - 1];
          rtb_sincos_o2_idx_0_tmp = QuadModelv_B.OutportBufferForcp13[
            static_cast<int32_T>(Z) - 1];

          // Product: '<S483>/Product' incorporates:
          //   Constant: '<S484>/Constant'
          //   Selector: '<S483>/Selector'
          //   Sum: '<S473>/Sum2'
          //   Sum: '<S484>/Sum'

          rtb_q2dot_tmp = QuadModelv_B.Sum2_h[((static_cast<int32_T>(
            static_cast<real_T>(r) + QuadModelv_P.Constant_Value_i) - 1) * 13 +
            static_cast<int32_T>(static_cast<real_T>(qY_1) +
            QuadModelv_P.Constant_Value_i)) - 1];

          // Product: '<S483>/Product1' incorporates:
          //   Product: '<S483>/Product'
          //   Selector: '<S476>/cp[m+1]'
          //   Selector: '<S476>/sp[m+1]'

          Z = QuadModelv_B.OutportBufferForsp13[static_cast<int32_T>(Z) - 1];

          // Merge: '<S476>/Merge' incorporates:
          //   Product: '<S483>/Product'
          //   Product: '<S483>/Product1'
          //   Selector: '<S476>/sp[m+1]'
          //   Selector: '<S483>/Selector'
          //   Sum: '<S483>/Sum'

          rtb_q2dot = rtb_q3dot * Z + rtb_q2dot_tmp * rtb_sincos_o2_idx_0_tmp;

          // Merge: '<S476>/Merge1' incorporates:
          //   Product: '<S483>/Product'
          //   Product: '<S483>/Product1'
          //   Selector: '<S476>/cp[m+1]'
          //   Selector: '<S483>/Selector1'
          //   Sum: '<S483>/Sum1'

          rtb_q3dot = rtb_q2dot_tmp * Z - rtb_q3dot * rtb_sincos_o2_idx_0_tmp;

          // End of Outputs for SubSystem: '<S476>/If Action Subsystem1'
        }

        // End of If: '<S476>/If'

        // Outputs for Enabled SubSystem: '<S470>/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations' incorporates:
        //   EnablePort: '<S472>/Enable'

        // SignalConversion generated from: '<S472>/Enable'
        if (rtb_hasFault) {
          // If: '<S472>/if n == m elseif (n==1&m==0) elseif (n>1&m~=n)' incorporates:
          //   Sum: '<S470>/Sum1'

          if (r == qY_1) {
            // Outputs for IfAction SubSystem: '<S472>/If Action Subsystem' incorporates:
            //   ActionPort: '<S486>/Action Port'

            // Selector: '<S486>/Selector' incorporates:
            //   Constant: '<S490>/Constant'
            //   Gain: '<S490>/Gain'
            //   Sum: '<S490>/Sum1'
            //   Sum: '<S490>/Sum2'

            if ((qY_1 >= 0) && (QuadModelv_P.Constant_Value_os < qY_1 -
                                MAX_int32_T)) {
              qY_0 = MAX_int32_T;
            } else if ((qY_1 < 0) && (QuadModelv_P.Constant_Value_os > qY_1 -
                        MIN_int32_T)) {
              qY_0 = MIN_int32_T;
            } else {
              qY_0 = qY_1 - QuadModelv_P.Constant_Value_os;
            }

            i = mul_s32_sat(QuadModelv_P.Gain_Gain_bd, qY_0);
            if ((r < 0) && (i < MIN_int32_T - r)) {
              i = MIN_int32_T;
            } else if ((r > 0) && (i > MAX_int32_T - r)) {
              i = MAX_int32_T;
            } else {
              i += r;
            }

            // Merge: '<S472>/Merge' incorporates:
            //   Product: '<S486>/Product1'
            //   Selector: '<S486>/Selector'
            //   UnitDelay: '<S472>/Unit Delay1'

            QuadModelv_B.Merge_i = QuadModelv_DW.UnitDelay1_DSTATE_j[i - 1] *
              QuadModelv_B.sqrt_d;

            // Selector: '<S486>/Selector' incorporates:
            //   Constant: '<S490>/Constant'
            //   Gain: '<S490>/Gain'
            //   Sum: '<S490>/Sum1'
            //   Sum: '<S490>/Sum2'

            if ((qY_1 >= 0) && (QuadModelv_P.Constant_Value_os < qY_1 -
                                MAX_int32_T)) {
              qY_0 = MAX_int32_T;
            } else if ((qY_1 < 0) && (QuadModelv_P.Constant_Value_os > qY_1 -
                        MIN_int32_T)) {
              qY_0 = MIN_int32_T;
            } else {
              qY_0 = qY_1 - QuadModelv_P.Constant_Value_os;
            }

            i = mul_s32_sat(QuadModelv_P.Gain_Gain_bd, qY_0);
            if ((r < 0) && (i < MIN_int32_T - r)) {
              i = MIN_int32_T;
            } else if ((r > 0) && (i > MAX_int32_T - r)) {
              i = MAX_int32_T;
            } else {
              i += r;
            }

            // Merge: '<S472>/Merge1' incorporates:
            //   Product: '<S486>/Product'
            //   Product: '<S486>/Product2'
            //   Selector: '<S486>/Selector'
            //   Selector: '<S486>/Selector1'
            //   Sum: '<S486>/Sum'
            //   UnitDelay: '<S472>/Unit Delay'
            //   UnitDelay: '<S472>/Unit Delay1'

            QuadModelv_B.Merge1_a = QuadModelv_DW.UnitDelay_DSTATE_i[((r - 1) *
              13 + qY_1) - 1] * QuadModelv_B.sqrt_d +
              QuadModelv_DW.UnitDelay1_DSTATE_j[i - 1] * QuadModelv_B.Product4_p;

            // End of Outputs for SubSystem: '<S472>/If Action Subsystem'
          } else if ((r == 1) && (qY_1 == 0)) {
            // Outputs for IfAction SubSystem: '<S472>/If Action Subsystem1' incorporates:
            //   ActionPort: '<S487>/Action Port'

            // Selector: '<S487>/Selector' incorporates:
            //   Gain: '<S492>/Gain'
            //   Sum: '<S492>/Sum1'

            qY_0 = mul_s32_sat(QuadModelv_P.Gain_Gain_hz, 0);
            if (qY_0 > 2147483646) {
              i = MAX_int32_T;
            } else {
              i = qY_0 + 1;
            }

            // Merge: '<S472>/Merge' incorporates:
            //   Product: '<S487>/Product3'
            //   Selector: '<S487>/Selector'
            //   UnitDelay: '<S472>/Unit Delay1'

            QuadModelv_B.Merge_i = QuadModelv_DW.UnitDelay1_DSTATE_j[i - 1] *
              QuadModelv_B.Product4_p;

            // Selector: '<S487>/Selector' incorporates:
            //   Sum: '<S492>/Sum1'

            if (qY_0 > 2147483646) {
              i = MAX_int32_T;
            } else {
              i = qY_0 + 1;
            }

            // Merge: '<S472>/Merge1' incorporates:
            //   Constant: '<S493>/Constant'
            //   Product: '<S487>/Product'
            //   Product: '<S487>/Product2'
            //   Selector: '<S487>/Selector'
            //   Selector: '<S487>/Selector1'
            //   Sum: '<S487>/Sum'
            //   UnitDelay: '<S472>/Unit Delay'
            //   UnitDelay: '<S472>/Unit Delay1'

            QuadModelv_B.Merge1_a =
              QuadModelv_DW.UnitDelay_DSTATE_i[QuadModelv_P.Constant_Value_es -
              1] * QuadModelv_B.Product4_p - QuadModelv_DW.UnitDelay1_DSTATE_j[i
              - 1] * QuadModelv_B.sqrt_d;

            // End of Outputs for SubSystem: '<S472>/If Action Subsystem1'
          } else if ((r > 1) && (r != qY_1)) {
            int32_T qY_2;

            // Outputs for IfAction SubSystem: '<S472>/If Action Subsystem2' incorporates:
            //   ActionPort: '<S488>/Action Port'

            // Sum: '<S495>/Sum' incorporates:
            //   Constant: '<S495>/Constant'

            if (QuadModelv_P.Constant_Value_g2 > MAX_int32_T - r) {
              qY_0 = MAX_int32_T;
            } else {
              qY_0 = r + QuadModelv_P.Constant_Value_g2;
            }

            if ((qY_1 < 0) && (QuadModelv_P.Constant_Value_g2 < MIN_int32_T
                               - qY_1)) {
              qY = MIN_int32_T;
            } else if ((qY_1 > 0) && (QuadModelv_P.Constant_Value_g2 >
                        MAX_int32_T - qY_1)) {
              qY = MAX_int32_T;
            } else {
              qY = qY_1 + QuadModelv_P.Constant_Value_g2;
            }

            // Gain: '<S494>/Gain'
            rtb_Sum1_mx = mul_s32_sat(QuadModelv_P.Gain_Gain_aa, qY_1);

            // Sum: '<S497>/Sum2' incorporates:
            //   Constant: '<S497>/Constant1'

            if (QuadModelv_P.Constant1_Value_fy < r - MAX_int32_T) {
              i = MAX_int32_T;
            } else {
              i = r - QuadModelv_P.Constant1_Value_fy;
            }

            // End of Sum: '<S497>/Sum2'

            // Switch: '<S488>/Switch' incorporates:
            //   Constant: '<S488>/Constant'
            //   RelationalOperator: '<S497>/Relational Operator'
            //   Selector: '<S488>/Selector1'
            //   UnitDelay: '<S472>/Unit Delay'

            if (i >= qY_1) {
              // Sum: '<S496>/Sum' incorporates:
              //   Constant: '<S496>/Constant'

              if ((qY_1 < 0) && (QuadModelv_P.Constant_Value_py < MIN_int32_T
                                 - qY_1)) {
                qY_2 = MIN_int32_T;
              } else if ((qY_1 > 0) && (QuadModelv_P.Constant_Value_py >
                          MAX_int32_T - qY_1)) {
                qY_2 = MAX_int32_T;
              } else {
                qY_2 = qY_1 + QuadModelv_P.Constant_Value_py;
              }

              // Selector: '<S488>/Selector1' incorporates:
              //   Constant: '<S496>/Constant1'
              //   Sum: '<S496>/Sum2'

              if (QuadModelv_P.Constant1_Value_c < r - MAX_int32_T) {
                i = MAX_int32_T;
              } else {
                i = r - QuadModelv_P.Constant1_Value_c;
              }

              Z = QuadModelv_DW.UnitDelay_DSTATE_i[((i - 1) * 13 + qY_2) - 1];
            } else {
              Z = QuadModelv_P.Constant_Value_m;
            }

            // End of Switch: '<S488>/Switch'

            // Sum: '<S496>/Sum' incorporates:
            //   Constant: '<S496>/Constant'

            if ((qY_1 < 0) && (QuadModelv_P.Constant_Value_py < MIN_int32_T
                               - qY_1)) {
              qY_2 = MIN_int32_T;
            } else if ((qY_1 > 0) && (QuadModelv_P.Constant_Value_py >
                        MAX_int32_T - qY_1)) {
              qY_2 = MAX_int32_T;
            } else {
              qY_2 = qY_1 + QuadModelv_P.Constant_Value_py;
            }

            // Selector: '<S488>/Selector' incorporates:
            //   Sum: '<S494>/Sum1'

            if (rtb_Sum1_mx > MAX_int32_T - r) {
              i = MAX_int32_T;
            } else {
              i = r + rtb_Sum1_mx;
            }

            // Selector: '<S488>/Selector2' incorporates:
            //   Constant: '<S488>/k[13][13]'
            //   Sum: '<S495>/Sum'

            rtb_q2dot_tmp = QuadModelv_P.k1313_Value_c[((qY_0 - 1) * 13 + qY) -
              1];

            // Merge: '<S472>/Merge1' incorporates:
            //   Constant: '<S488>/k[13][13]'
            //   Product: '<S488>/Product'
            //   Product: '<S488>/Product1'
            //   Product: '<S488>/Product4'
            //   Selector: '<S488>/Selector'
            //   Selector: '<S488>/Selector1'
            //   Selector: '<S488>/Selector2'
            //   Sum: '<S488>/Sum'
            //   UnitDelay: '<S472>/Unit Delay'
            //   UnitDelay: '<S472>/Unit Delay1'

            QuadModelv_B.Merge1_a = (QuadModelv_DW.UnitDelay_DSTATE_i[((r - 1) *
              13 + qY_2) - 1] * QuadModelv_B.Product4_p -
              QuadModelv_DW.UnitDelay1_DSTATE_j[i - 1] * QuadModelv_B.sqrt_d) -
              rtb_q2dot_tmp * Z;

            // Sum: '<S498>/Sum2' incorporates:
            //   Constant: '<S498>/Constant1'

            if (QuadModelv_P.Constant1_Value_hs < r - MAX_int32_T) {
              i = MAX_int32_T;
            } else {
              i = r - QuadModelv_P.Constant1_Value_hs;
            }

            // End of Sum: '<S498>/Sum2'

            // Switch: '<S488>/Switch1' incorporates:
            //   Constant: '<S488>/Constant1'
            //   RelationalOperator: '<S498>/Relational Operator'
            //   Selector: '<S488>/Selector'
            //   UnitDelay: '<S472>/Unit Delay1'

            if (i >= qY_1) {
              // Selector: '<S488>/Selector' incorporates:
              //   Constant: '<S494>/Constant1'
              //   Sum: '<S494>/Sum1'
              //   Sum: '<S494>/Sum2'

              if (QuadModelv_P.Constant1_Value_m < r - MAX_int32_T) {
                qY_0 = MAX_int32_T;
              } else {
                qY_0 = r - QuadModelv_P.Constant1_Value_m;
              }

              if ((qY_0 < 0) && (rtb_Sum1_mx < MIN_int32_T - qY_0)) {
                qY_0 = MIN_int32_T;
              } else if ((qY_0 > 0) && (rtb_Sum1_mx > MAX_int32_T - qY_0)) {
                qY_0 = MAX_int32_T;
              } else {
                qY_0 += rtb_Sum1_mx;
              }

              Z = QuadModelv_DW.UnitDelay1_DSTATE_j[qY_0 - 1];
            } else {
              Z = QuadModelv_P.Constant1_Value_o;
            }

            // End of Switch: '<S488>/Switch1'

            // Selector: '<S488>/Selector' incorporates:
            //   Sum: '<S494>/Sum1'

            if (rtb_Sum1_mx > MAX_int32_T - r) {
              rtb_Sum1_mx = MAX_int32_T;
            } else {
              rtb_Sum1_mx += r;
            }

            // Merge: '<S472>/Merge' incorporates:
            //   Product: '<S488>/Product2'
            //   Product: '<S488>/Product3'
            //   Selector: '<S488>/Selector'
            //   Sum: '<S488>/Sum1'
            //   UnitDelay: '<S472>/Unit Delay1'

            QuadModelv_B.Merge_i = QuadModelv_DW.UnitDelay1_DSTATE_j[rtb_Sum1_mx
              - 1] * QuadModelv_B.Product4_p - rtb_q2dot_tmp * Z;

            // End of Outputs for SubSystem: '<S472>/If Action Subsystem2'
          }

          // End of If: '<S472>/if n == m elseif (n==1&m==0) elseif (n>1&m~=n)'

          // Sum: '<S472>/Sum' incorporates:
          //   Constant: '<S472>/Constant'
          //   Sum: '<S470>/Sum1'

          if ((r < 0) && (QuadModelv_P.Constant_Value_f < MIN_int32_T - r)) {
            qY_0 = MIN_int32_T;
          } else if ((r > 0) && (QuadModelv_P.Constant_Value_f > MAX_int32_T - r))
          {
            qY_0 = MAX_int32_T;
          } else {
            qY_0 = r + QuadModelv_P.Constant_Value_f;
          }

          if ((qY_1 < 0) && (QuadModelv_P.Constant_Value_f < MIN_int32_T - qY_1))
          {
            qY = MIN_int32_T;
          } else if ((qY_1 > 0) && (QuadModelv_P.Constant_Value_f > MAX_int32_T
                      - qY_1)) {
            qY = MAX_int32_T;
          } else {
            qY = qY_1 + QuadModelv_P.Constant_Value_f;
          }

          // Assignment: '<S472>/Assignment' incorporates:
          //   Sum: '<S472>/Sum'
          //   UnitDelay: '<S472>/Unit Delay'

          if (ForIterator_IterationMarker[2] < 2) {
            ForIterator_IterationMarker[2] = 2U;
            std::memcpy(&QuadModelv_B.Assignment_g[0],
                        &QuadModelv_DW.UnitDelay_DSTATE_i[0], 169U * sizeof
                        (real_T));
          }

          QuadModelv_B.Assignment_g[(qY + 13 * (qY_0 - 1)) - 1] =
            QuadModelv_B.Merge1_a;

          // End of Assignment: '<S472>/Assignment'

          // Assignment: '<S472>/Assignment_snorm'
          if (ForIterator_IterationMarker[3] < 2) {
            ForIterator_IterationMarker[3] = 2U;

            // Assignment: '<S472>/Assignment_snorm' incorporates:
            //   UnitDelay: '<S472>/Unit Delay1'

            std::memcpy(&QuadModelv_B.Assignment_snorm[0],
                        &QuadModelv_DW.UnitDelay1_DSTATE_j[0], 169U * sizeof
                        (real_T));
          }

          // Sum: '<S489>/Sum2' incorporates:
          //   Constant: '<S489>/Constant'
          //   Sum: '<S472>/Sum'

          if ((qY >= 0) && (QuadModelv_P.Constant_Value_lp < qY - MAX_int32_T))
          {
            qY = MAX_int32_T;
          } else if ((qY < 0) && (QuadModelv_P.Constant_Value_lp > qY -
                                  MIN_int32_T)) {
            qY = MIN_int32_T;
          } else {
            qY -= QuadModelv_P.Constant_Value_lp;
          }

          // End of Sum: '<S489>/Sum2'

          // Sum: '<S489>/Sum1' incorporates:
          //   Gain: '<S489>/Gain'
          //   Sum: '<S472>/Sum'

          i = mul_s32_sat(QuadModelv_P.Gain_Gain_ob, qY);
          if ((qY_0 < 0) && (i < MIN_int32_T - qY_0)) {
            qY_0 = MIN_int32_T;
          } else if ((qY_0 > 0) && (i > MAX_int32_T - qY_0)) {
            qY_0 = MAX_int32_T;
          } else {
            qY_0 += i;
          }

          // End of Sum: '<S489>/Sum1'

          // Assignment: '<S472>/Assignment_snorm'
          QuadModelv_B.Assignment_snorm[qY_0 - 1] = QuadModelv_B.Merge_i;

          // Update for UnitDelay: '<S472>/Unit Delay' incorporates:
          //   Assignment: '<S472>/Assignment'

          std::memcpy(&QuadModelv_DW.UnitDelay_DSTATE_i[0],
                      &QuadModelv_B.Assignment_g[0], 169U * sizeof(real_T));

          // Update for UnitDelay: '<S472>/Unit Delay1'
          std::memcpy(&QuadModelv_DW.UnitDelay1_DSTATE_j[0],
                      &QuadModelv_B.Assignment_snorm[0], 169U * sizeof(real_T));
        }

        // End of SignalConversion generated from: '<S472>/Enable'
        // End of Outputs for SubSystem: '<S470>/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations' 

        // Selector: '<S471>/snorm[n+m*13]' incorporates:
        //   Gain: '<S475>/Gain'
        //   Sum: '<S470>/Sum1'
        //   Sum: '<S475>/Sum1'

        i = mul_s32_sat(QuadModelv_P.Gain_Gain_ij, qY_1);
        if ((loop_ub < 0) && (i < MIN_int32_T - loop_ub)) {
          qY_0 = MIN_int32_T;
        } else if ((loop_ub > 0) && (i > MAX_int32_T - loop_ub)) {
          qY_0 = MAX_int32_T;
        } else {
          qY_0 = loop_ub + i;
        }

        // Product: '<S471>/par' incorporates:
        //   Selector: '<S471>/snorm[n+m*13]'

        Z = QuadModelv_B.Assignment_snorm[qY_0 - 1] * rtb_Product3_o;

        // Outputs for Enabled SubSystem: '<S471>/Special case - North//South Geographic Pole' incorporates:
        //   EnablePort: '<S474>/Enable'

        // Logic: '<S477>/Logical Operator' incorporates:
        //   Constant: '<S477>/Constant'
        //   Constant: '<S477>/Constant1'
        //   RelationalOperator: '<S477>/Relational Operator'
        //   RelationalOperator: '<S477>/Relational Operator1'
        //   Sum: '<S470>/Sum1'

        if ((QuadModelv_B.sqrt_d == QuadModelv_P.Constant1_Value_h) &&
            (QuadModelv_P.Constant_Value_a == qY_1)) {
          QuadModelv_DW.SpecialcaseNorthSouthGeographic = true;

          // If: '<S474>/n ==1' incorporates:
          //   Assignment: '<S479>/Assignment2'

          if (r == 1) {
            // Outputs for IfAction SubSystem: '<S474>/If Action Subsystem1' incorporates:
            //   ActionPort: '<S478>/Action Port'

            // Assignment: '<S478>/Assignment2' incorporates:
            //   Constant: '<S478>/Constant'
            //   Selector: '<S478>/pp[n-1]'
            //   Sum: '<S478>/Sum2'
            //   UnitDelay: '<S474>/Unit Delay1'

            if (ForIterator_IterationMarker[0] < 2) {
              ForIterator_IterationMarker[0] = 2U;
              std::memcpy(&QuadModelv_B.Assignment2_a[0],
                          &QuadModelv_DW.UnitDelay1_DSTATE_a[0], 13U * sizeof
                          (real_T));
            }

            QuadModelv_B.Assignment2_a[static_cast<int32_T>
              (QuadModelv_P.Constant_Value_k + 1.0) - 1] =
              QuadModelv_DW.UnitDelay1_DSTATE_a[0];

            // End of Assignment: '<S478>/Assignment2'
            // End of Outputs for SubSystem: '<S474>/If Action Subsystem1'
          } else {
            // Outputs for IfAction SubSystem: '<S474>/If Action Subsystem2' incorporates:
            //   ActionPort: '<S479>/Action Port'

            if (ForIterator_IterationMarker[1] < 2) {
              // Assignment: '<S479>/Assignment2'
              ForIterator_IterationMarker[1] = 2U;

              // Assignment: '<S479>/Assignment2' incorporates:
              //   UnitDelay: '<S474>/Unit Delay1'

              std::memcpy(&QuadModelv_B.Assignment2_m[0],
                          &QuadModelv_DW.UnitDelay1_DSTATE_a[0], 13U * sizeof
                          (real_T));
            }

            // Sum: '<S479>/Sum2' incorporates:
            //   Constant: '<S479>/Constant'

            if ((r < 0) && (QuadModelv_P.Constant_Value_mh < MIN_int32_T - r)) {
              i = MIN_int32_T;
            } else if ((r > 0) && (QuadModelv_P.Constant_Value_mh > MAX_int32_T
                                   - r)) {
              i = MAX_int32_T;
            } else {
              i = r + QuadModelv_P.Constant_Value_mh;
            }

            // End of Sum: '<S479>/Sum2'

            // Sum: '<S481>/Sum' incorporates:
            //   Constant: '<S481>/Constant'

            if ((qY_1 < 0) && (QuadModelv_P.Constant_Value_op < MIN_int32_T
                               - qY_1)) {
              qY_0 = MIN_int32_T;
            } else if ((qY_1 > 0) && (QuadModelv_P.Constant_Value_op >
                        MAX_int32_T - qY_1)) {
              qY_0 = MAX_int32_T;
            } else {
              qY_0 = qY_1 + QuadModelv_P.Constant_Value_op;
            }

            if ((r < 0) && (QuadModelv_P.Constant_Value_op < MIN_int32_T - r)) {
              qY = MIN_int32_T;
            } else if ((r > 0) && (QuadModelv_P.Constant_Value_op > MAX_int32_T
                                   - r)) {
              qY = MAX_int32_T;
            } else {
              qY = r + QuadModelv_P.Constant_Value_op;
            }

            // End of Sum: '<S481>/Sum'

            // Selector: '<S479>/pp[n-2] pp[n-1]' incorporates:
            //   Constant: '<S480>/Constant1'
            //   Sum: '<S480>/Sum2'

            if ((r >= 0) && (QuadModelv_P.Constant1_Value_l < r - MAX_int32_T))
            {
              rtb_Sum1_mx = MAX_int32_T;
            } else if ((r < 0) && (QuadModelv_P.Constant1_Value_l > r -
                                   MIN_int32_T)) {
              rtb_Sum1_mx = MIN_int32_T;
            } else {
              rtb_Sum1_mx = r - QuadModelv_P.Constant1_Value_l;
            }

            // Assignment: '<S479>/Assignment2' incorporates:
            //   Constant: '<S479>/k[13][13]'
            //   Product: '<S479>/Product1'
            //   Product: '<S479>/Product2'
            //   Selector: '<S479>/Selector2'
            //   Selector: '<S479>/pp[n-2] pp[n-1]'
            //   Sum: '<S479>/Sum1'
            //   UnitDelay: '<S474>/Unit Delay1'

            QuadModelv_B.Assignment2_m[i - 1] =
              QuadModelv_DW.UnitDelay1_DSTATE_a[r - 1] * QuadModelv_B.Product4_p
              - QuadModelv_P.k1313_Value[((qY - 1) * 13 + qY_0) - 1] *
              QuadModelv_DW.UnitDelay1_DSTATE_a[rtb_Sum1_mx - 1];

            // End of Outputs for SubSystem: '<S474>/If Action Subsystem2'
          }

          // End of If: '<S474>/n ==1'

          // SignalConversion generated from: '<S474>/pp[n]' incorporates:
          //   UnitDelay: '<S474>/Unit Delay1'

          rtb_TmpSignalConversionAtppnInp[0] =
            QuadModelv_DW.UnitDelay1_DSTATE_a[0];
          rtb_TmpSignalConversionAtppnInp[1] = QuadModelv_B.Assignment2_a[1];
          std::memcpy(&rtb_TmpSignalConversionAtppnInp[2],
                      &QuadModelv_B.Assignment2_m[2], 11U * sizeof(real_T));

          // Product: '<S474>/Product2' incorporates:
          //   Constant: '<S474>/Constant'
          //   Constant: '<S474>/Constant1'
          //   Selector: '<S474>/pp[n]'
          //   Sum: '<S474>/Sum2'

          QuadModelv_B.Product2_b = rtb_TmpSignalConversionAtppnInp
            [static_cast<int32_T>(static_cast<real_T>(r) +
            QuadModelv_P.Constant1_Value_i) - 1] * rtb_Product3_o *
            QuadModelv_P.Constant_Value_p * rtb_q3dot;

          // Update for UnitDelay: '<S474>/Unit Delay1'
          std::memcpy(&QuadModelv_DW.UnitDelay1_DSTATE_a[0],
                      &rtb_TmpSignalConversionAtppnInp[0], 13U * sizeof(real_T));
        } else if (QuadModelv_DW.SpecialcaseNorthSouthGeographic) {
          // Disable for Product: '<S474>/Product2' incorporates:
          //   Outport: '<S474>/bpp'

          QuadModelv_B.Product2_b = QuadModelv_P.bpp_Y0;
          QuadModelv_DW.SpecialcaseNorthSouthGeographic = false;
        }

        // End of Logic: '<S477>/Logical Operator'
        // End of Outputs for SubSystem: '<S471>/Special case - North//South Geographic Pole' 

        // Sum: '<S471>/Sum' incorporates:
        //   Constant: '<S471>/Constant'
        //   Sum: '<S470>/Sum1'

        if ((qY_1 < 0) && (QuadModelv_P.Constant_Value_j < MIN_int32_T - qY_1))
        {
          qY_0 = MIN_int32_T;
        } else if ((qY_1 > 0) && (QuadModelv_P.Constant_Value_j > MAX_int32_T
                    - qY_1)) {
          qY_0 = MAX_int32_T;
        } else {
          qY_0 = qY_1 + QuadModelv_P.Constant_Value_j;
        }

        // Sum: '<S471>/Sum1' incorporates:
        //   Assignment: '<S472>/Assignment'
        //   Product: '<S471>/Product'
        //   Selector: '<S471>/dp[n][m]'
        //   UnitDelay: '<S471>/Unit Delay1'

        QuadModelv_B.Sum1_f = rtb_fcn3 - QuadModelv_B.Assignment_g[((r_0 - 1) *
          13 + qY_0) - 1] * rtb_q2dot * rtb_Product3_o;

        // Sum: '<S471>/Sum4' incorporates:
        //   Constant: '<S471>/Constant1'
        //   Sum: '<S470>/Sum1'

        if ((qY_1 < 0) && (QuadModelv_P.Constant1_Value_ja < MIN_int32_T - qY_1))
        {
          qY_1 = MIN_int32_T;
        } else if ((qY_1 > 0) && (QuadModelv_P.Constant1_Value_ja > MAX_int32_T
                    - qY_1)) {
          qY_1 = MAX_int32_T;
        } else {
          qY_1 += QuadModelv_P.Constant1_Value_ja;
        }

        // Sum: '<S471>/Sum2' incorporates:
        //   Constant: '<S471>/fm'
        //   Product: '<S471>/Product1'
        //   Selector: '<S471>/fm[m]'
        //   UnitDelay: '<S471>/Unit Delay3'

        QuadModelv_B.Sum2_k = QuadModelv_P.fm_Value[qY_1 - 1] * Z * rtb_q3dot +
          rtb_Rn;

        // Sum: '<S471>/Sum3' incorporates:
        //   Constant: '<S471>/fn'
        //   Product: '<S471>/Product2'
        //   Selector: '<S471>/fn[m]'
        //   UnitDelay: '<S471>/Unit Delay2'

        QuadModelv_B.Sum3 = QuadModelv_P.fn_Value[r_1 - 1] * Z * rtb_q2dot +
          rtb_Sum1_c;

        // Sum: '<S471>/Sum5' incorporates:
        //   UnitDelay: '<S471>/Unit Delay4'

        QuadModelv_B.Sum5 = j + QuadModelv_B.Product2_b;

        // Update for UnitDelay: '<S471>/Unit Delay1'
        rtb_fcn3 = QuadModelv_B.Sum1_f;

        // Update for UnitDelay: '<S471>/Unit Delay3'
        rtb_Rn = QuadModelv_B.Sum2_k;

        // Update for UnitDelay: '<S471>/Unit Delay2'
        rtb_Sum1_c = QuadModelv_B.Sum3;

        // Update for UnitDelay: '<S471>/Unit Delay4'
        j = QuadModelv_B.Sum5;
      }

      // End of Outputs for SubSystem: '<S462>/For Iterator Subsystem'

      // Sum: '<S462>/Sum1' incorporates:
      //   UnitDelay: '<S462>/Unit Delay2'

      QuadModelv_B.Sum1_l[0] = tz[0] + QuadModelv_B.Sum1_f;
      QuadModelv_B.Sum1_l[1] = tz[1] + QuadModelv_B.Sum2_k;
      QuadModelv_B.Sum1_l[2] = tz[2] + QuadModelv_B.Sum3;
      QuadModelv_B.Sum1_l[3] = tz[3] + QuadModelv_B.Sum5;

      // Update for UnitDelay: '<S462>/Unit Delay'
      Z = rtb_Product3_o;

      // Update for UnitDelay: '<S462>/Unit Delay2'
      tz[0] = QuadModelv_B.Sum1_l[0];
      tz[1] = QuadModelv_B.Sum1_l[1];
      tz[2] = QuadModelv_B.Sum1_l[2];
      tz[3] = QuadModelv_B.Sum1_l[3];
    }

    // End of Outputs for SubSystem: '<S454>/Compute magnetic vector in spherical coordinates' 
  }

  // Switch: '<S514>/Switch' incorporates:
  //   Product: '<S514>/Product'

  if (QuadModelv_B.sqrt_d != 0.0) {
    rtb_Rn = QuadModelv_B.Sum1_l[1] / QuadModelv_B.sqrt_d;
  } else {
    rtb_Rn = QuadModelv_B.Sum1_l[3];
  }

  // End of Switch: '<S514>/Switch'

  // Sum: '<S513>/Sum1' incorporates:
  //   Product: '<S513>/Product1'
  //   Product: '<S513>/Product4'

  rtb_Sum1_c = (0.0 - QuadModelv_B.Product11 * QuadModelv_B.Sum1_l[0]) -
    QuadModelv_B.Sum1_l[2] * QuadModelv_B.Product12;

  // UnitConversion: '<S518>/Unit Conversion' incorporates:
  //   Trigonometry: '<S516>/Trigonometric Function1'

  // Unit Conversion - from: rad to: deg
  // Expression: output = (57.2958*input) + (0)
  rtb_Product3_o = 57.295779513082323 * rt_atan2d_snf(rtb_Rn, rtb_Sum1_c);

  // Sum: '<S515>/Sum1' incorporates:
  //   Product: '<S515>/Product1'
  //   Product: '<S515>/Product4'

  rtb_q2dot = QuadModelv_B.Product12 * QuadModelv_B.Sum1_l[0] -
    QuadModelv_B.Sum1_l[2] * QuadModelv_B.Product11;

  // Sum: '<S516>/Sum' incorporates:
  //   Product: '<S516>/Product'
  //   Product: '<S516>/Product1'

  rtb_Rn = rtb_Rn * rtb_Rn + rtb_Sum1_c * rtb_Sum1_c;

  // UnitConversion: '<S455>/Unit Conversion' incorporates:
  //   Sqrt: '<S516>/sqrt1'
  //   Trigonometry: '<S516>/Trigonometric Function'
  //   UnitConversion: '<S517>/Unit Conversion'

  // Unit Conversion - from: rad to: deg
  // Expression: output = (57.2958*input) + (0)
  // Unit Conversion - from: deg to: rad
  // Expression: output = (0.0174533*input) + (0)
  uavlostJzx = 0.017453292519943295 * rtb_Product3_o;
  rtb_fcn3 = 57.295779513082323 * rt_atan2d_snf(rtb_q2dot, std::sqrt(rtb_Rn)) *
    0.017453292519943295;

  // Sqrt: '<S516>/sqrt' incorporates:
  //   Product: '<S516>/Product2'
  //   Sum: '<S516>/Sum1'

  rtb_Rn = std::sqrt(rtb_q2dot * rtb_q2dot + rtb_Rn);

  // Product: '<S448>/h1' incorporates:
  //   Trigonometry: '<S448>/sincos'

  rtb_Sum1_c = std::cos(rtb_fcn3) * rtb_Rn;

  // SignalConversion generated from: '<S407>/Matrix Multiply2' incorporates:
  //   Product: '<S448>/x1'
  //   Product: '<S448>/y1'
  //   Product: '<S448>/z1'
  //   Trigonometry: '<S448>/sincos'

  j = std::cos(uavlostJzx) * rtb_Sum1_c;
  rtb_Sum1_c *= std::sin(uavlostJzx);
  rtb_Rn *= std::sin(rtb_fcn3);
  for (i = 0; i < 3; i++) {
    // DataTypeConversion: '<S8>/Data Type Conversion1' incorporates:
    //   Concatenate: '<S36>/Vector Concatenate'
    //   Product: '<S407>/Matrix Multiply2'

    rtb_DataTypeConversion1[i] = 0.0;
    rtb_DataTypeConversion1[i] += VectorConcatenate[i] * j;
    rtb_DataTypeConversion1[i] += VectorConcatenate[i + 3] * rtb_Sum1_c;
    rtb_DataTypeConversion1[i] += VectorConcatenate[i + 6] * rtb_Rn;
  }

  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[2] == 0) {
    // Gain: '<S407>/Gain11' incorporates:
    //   UniformRandomNumber: '<S407>/Uniform Random Number7'

    QuadModelv_B.Gain11[0] = QuadModelv_P.Gain11_Gain *
      QuadModelv_DW.UniformRandomNumber7_NextOutput[0];
    QuadModelv_B.Gain11[1] = QuadModelv_P.Gain11_Gain *
      QuadModelv_DW.UniformRandomNumber7_NextOutput[1];
    QuadModelv_B.Gain11[2] = QuadModelv_P.Gain11_Gain *
      QuadModelv_DW.UniformRandomNumber7_NextOutput[2];
  }

  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // MATLAB Function: '<S407>/FaultParamsExtract2' incorporates:
    //   Constant: '<S407>/FaultID2'
    //   Inport: '<Root>/inSILInts'

    // MATLAB Function 'SensorFault/FaultParamsExtract2': '<S412>:1'
    // '<S412>:1:5' if isempty(hFault)
    // '<S412>:1:8' if isempty(fParam)
    // '<S412>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S412>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S412>:1:14' j=1;
    j = 1.0;

    // '<S412>:1:15' for i=1:8
    for (b_t = 0; b_t < 8; b_t++) {
      // '<S412>:1:16' if inInts(i) == FaultID
      if (QuadModelv_U.inSILInts[b_t] == QuadModelv_P.FaultID2_Value_d) {
        // '<S412>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S412>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        rtb_alt = (b_t + 1) << 1;
        fParamTmp[static_cast<int32_T>(2.0 * j - 1.0) - 1] = inSILFloats[rtb_alt
          - 2];

        // '<S412>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[static_cast<int32_T>(2.0 * j) - 1] = inSILFloats[rtb_alt - 1];

        // '<S412>:1:20' j=j+1;
        j++;
      }
    }

    // '<S412>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S412>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_k = true;

      // '<S412>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = inSILFloats[16];
      fParamTmp[17] = inSILFloats[17];
      fParamTmp[18] = inSILFloats[18];
      fParamTmp[19] = inSILFloats[19];

      // '<S412>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_d[0], &fParamTmp[0], 20U * sizeof(real_T));
    }

    // MATLAB Function: '<S418>/Acc NoiseFun' incorporates:
    //   MATLAB Function: '<S407>/FaultParamsExtract2'

    // '<S412>:1:29' hasFault_mag=hFault;
    // '<S412>:1:30' FaultParam=fParam;
    QuadModelv_AccNoiseFun(QuadModelv_B.Gain11, QuadModelv_DW.hFault_k,
      QuadModelv_DW.fParam_d, &QuadModelv_B.sf_AccNoiseFun_p);
  }

  // Saturate: '<S64>/Limit  altitude  to troposhere'
  if (rtb_Saturation_1 > QuadModelv_P.Limitaltitudetotroposhere_Upper) {
    rtb_Rn = QuadModelv_P.Limitaltitudetotroposhere_Upper;
  } else if (rtb_Saturation_1 < QuadModelv_P.Limitaltitudetotroposhere_Lower) {
    rtb_Rn = QuadModelv_P.Limitaltitudetotroposhere_Lower;
  } else {
    rtb_Rn = rtb_Saturation_1;
  }

  // End of Saturate: '<S64>/Limit  altitude  to troposhere'

  // Sum: '<S64>/Sum1' incorporates:
  //   Constant: '<S64>/Sea Level  Temperature'
  //   Gain: '<S64>/Lapse Rate'

  rtb_Rn = QuadModelv_P.SeaLevelTemperature_Value_i -
    QuadModelv_P.LapseRate_Gain * rtb_Rn;

  // Gain: '<S64>/1//T0'
  rtb_Sum1_c = QuadModelv_P.uT0_Gain * rtb_Rn;

  // Sum: '<S64>/Sum' incorporates:
  //   Constant: '<S64>/Altitude of Troposphere'

  j = QuadModelv_P.AltitudeofTroposphere_Value - rtb_Saturation_1;

  // Math: '<S64>/(T//T0)^(g//LR) ' incorporates:
  //   Constant: '<S64>/Constant'

  if ((rtb_Sum1_c < 0.0) && (QuadModelv_P.Constant_Value_mq > std::floor
       (QuadModelv_P.Constant_Value_mq))) {
    rtb_fcn3 = -rt_powd_snf(-rtb_Sum1_c, QuadModelv_P.Constant_Value_mq);
  } else {
    rtb_fcn3 = rt_powd_snf(rtb_Sum1_c, QuadModelv_P.Constant_Value_mq);
  }

  // End of Math: '<S64>/(T//T0)^(g//LR) '

  // Saturate: '<S64>/Limit  altitude  to Stratosphere'
  if (j > QuadModelv_P.LimitaltitudetoStratosphere_Upp) {
    j = QuadModelv_P.LimitaltitudetoStratosphere_Upp;
  } else if (j < QuadModelv_P.LimitaltitudetoStratosphere_Low) {
    j = QuadModelv_P.LimitaltitudetoStratosphere_Low;
  }

  // End of Saturate: '<S64>/Limit  altitude  to Stratosphere'

  // Product: '<S64>/Product3' incorporates:
  //   Gain: '<S64>/g//R'
  //   Gain: '<S64>/rho0'
  //   Math: '<S64>/Stratosphere Model'
  //   Product: '<S64>/Product'
  //   Product: '<S64>/Product1'
  //
  //  About '<S64>/Stratosphere Model':
  //   Operator: exp

  rtb_Sum1_c = rtb_fcn3 / rtb_Sum1_c * QuadModelv_P.rho0_Gain * std::exp(1.0 /
    rtb_Rn * (QuadModelv_P.gR_Gain * j));

  // Sum: '<S423>/Sum' incorporates:
  //   Product: '<S423>/Product'
  //   Product: '<S423>/Product1'
  //   Product: '<S423>/Product2'
  //   Sum: '<S520>/Sum'
  //   Sum: '<S521>/Sum'

  rtb_q2dot_tmp = (rtb_Windb[0] * rtb_Windb[0] + rtb_Windb[1] * rtb_Windb[1]) +
    rtb_Windb[2] * rtb_Windb[2];

  // Gain: '<S409>/1//2rhoV^2' incorporates:
  //   Product: '<S409>/Product2'
  //   Sum: '<S423>/Sum'

  rtb_q2dot = rtb_q2dot_tmp * rtb_Sum1_c * QuadModelv_P.u2rhoV2_Gain;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // MATLAB Function: '<S407>/FaultParamsExtract3' incorporates:
    //   Constant: '<S407>/FaultID3'
    //   Inport: '<Root>/inSILInts'

    // MATLAB Function 'SensorFault/FaultParamsExtract3': '<S413>:1'
    // '<S413>:1:5' if isempty(hFault)
    // '<S413>:1:8' if isempty(fParam)
    // '<S413>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S413>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S413>:1:14' j=1;
    j = 1.0;

    // '<S413>:1:15' for i=1:8
    for (b_t = 0; b_t < 8; b_t++) {
      // '<S413>:1:16' if inInts(i) == FaultID
      if (QuadModelv_U.inSILInts[b_t] == QuadModelv_P.FaultID3_Value_l) {
        // '<S413>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S413>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        fParamTmp[static_cast<int32_T>(2.0 * j - 1.0) - 1] = inSILFloats[((b_t +
          1) << 1) - 2];

        // '<S413>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[static_cast<int32_T>(2.0 * j) - 1] = inSILFloats[((b_t + 1) <<
          1) - 1];

        // '<S413>:1:20' j=j+1;
        j++;
      }
    }

    // '<S413>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S413>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_g = true;

      // '<S413>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = inSILFloats[16];
      fParamTmp[17] = inSILFloats[17];
      fParamTmp[18] = inSILFloats[18];
      fParamTmp[19] = inSILFloats[19];

      // '<S413>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_a[0], &fParamTmp[0], 20U * sizeof(real_T));
    }

    // MATLAB Function: '<S407>/baro NoiseFun' incorporates:
    //   MATLAB Function: '<S407>/FaultParamsExtract3'

    // '<S413>:1:29' hasFault_baro=hFault;
    // '<S413>:1:30' FaultParam=fParam;
    // MATLAB Function 'SensorFault/baro NoiseFun': '<S421>:1'
    // '<S421>:1:3' y = 0;
    QuadModelv_B.y_o = 0.0;

    // '<S421>:1:4' if isbyroFault
    if (QuadModelv_DW.hFault_g) {
      // '<S421>:1:5' y = baroFaultParam(1);
      QuadModelv_B.y_o = QuadModelv_DW.fParam_a[0];
    }

    // End of MATLAB Function: '<S407>/baro NoiseFun'
  }

  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
    // Product: '<S415>/Product4' incorporates:
    //   Constant: '<S415>/Constant2'
    //   Gain: '<S407>/Gain5'
    //   UniformRandomNumber: '<S407>/Uniform Random Number'

    QuadModelv_B.Product4 = QuadModelv_P.Gain5_Gain *
      QuadModelv_DW.UniformRandomNumber_NextOutpu_l *
      QuadModelv_P.Constant2_Value_oq;
  }

  // Sum: '<S407>/Add2' incorporates:
  //   Product: '<S407>/Product'

  rtb_fcn3 = (rtb_q2dot * QuadModelv_B.y_o + rtb_Saturation_1) +
    QuadModelv_B.Product4;

  // Saturate: '<S417>/Limit  altitude  to troposhere'
  if (rtb_fcn3 > QuadModelv_P.Limitaltitudetotroposhere_Upp_h) {
    j = QuadModelv_P.Limitaltitudetotroposhere_Upp_h;
  } else if (rtb_fcn3 < QuadModelv_P.Limitaltitudetotroposhere_Low_g) {
    j = QuadModelv_P.Limitaltitudetotroposhere_Low_g;
  } else {
    j = rtb_fcn3;
  }

  // End of Saturate: '<S417>/Limit  altitude  to troposhere'

  // Sum: '<S417>/Sum1' incorporates:
  //   Constant: '<S417>/Sea Level  Temperature'
  //   Gain: '<S417>/Lapse Rate'

  rtb_q3dot = QuadModelv_P.SeaLevelTemperature_Value -
    QuadModelv_P.LapseRate_Gain_g * j;

  // Gain: '<S417>/1//T0'
  j = QuadModelv_P.uT0_Gain_d * rtb_q3dot;

  // Math: '<S417>/(T//T0)^(g//LR) ' incorporates:
  //   Constant: '<S417>/Constant'

  if ((j < 0.0) && (QuadModelv_P.Constant_Value_o > std::floor
                    (QuadModelv_P.Constant_Value_o))) {
    Z = -rt_powd_snf(-j, QuadModelv_P.Constant_Value_o);
  } else {
    Z = rt_powd_snf(j, QuadModelv_P.Constant_Value_o);
  }

  // End of Math: '<S417>/(T//T0)^(g//LR) '

  // Sum: '<S417>/Sum' incorporates:
  //   Constant: '<S417>/Altitude of Troposphere'

  j = QuadModelv_P.AltitudeofTroposphere_Value_f - rtb_fcn3;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
    // Product: '<S416>/Product4' incorporates:
    //   Constant: '<S416>/Constant'
    //   Constant: '<S416>/Constant2'
    //   Gain: '<S407>/Gain9'
    //   Gain: '<S416>/Gain2'
    //   Sum: '<S416>/Sum'
    //   UniformRandomNumber: '<S407>/Uniform Random Number4'

    QuadModelv_B.Product4_h = (QuadModelv_P.Gain2_Gain_a *
      QuadModelv_P.Constant_Value_ly + QuadModelv_P.Constant2_Value_i) *
      (QuadModelv_P.Gain9_Gain * QuadModelv_DW.UniformRandomNumber4_NextOutput);
  }

  // Unit Conversion - from: K to: degC
  // Expression: output = (1*input) + (-273.15)
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // DataTypeConversion: '<S403>/Data Type Conversion6' incorporates:
    //   Constant: '<S403>/Constant'

    rtb_sincos_o2_idx_0_tmp = std::floor(QuadModelv_P.Constant_Value_gb);
    if (rtIsNaN(rtb_sincos_o2_idx_0_tmp) || rtIsInf(rtb_sincos_o2_idx_0_tmp)) {
      rtb_sincos_o2_idx_0_tmp = 0.0;
    } else {
      rtb_sincos_o2_idx_0_tmp = std::fmod(rtb_sincos_o2_idx_0_tmp,
        4.294967296E+9);
    }

    // DataTypeConversion: '<S403>/Data Type Conversion6'
    QuadModelv_B.fields_updated = rtb_sincos_o2_idx_0_tmp < 0.0 ?
      static_cast<uint32_T>(-static_cast<int32_T>(static_cast<uint32_T>
      (-rtb_sincos_o2_idx_0_tmp))) : static_cast<uint32_T>
      (rtb_sincos_o2_idx_0_tmp);
  }

  // Outport: '<Root>/MavHILSensor' incorporates:
  //   BusCreator: '<S403>/Bus Creator'
  //   DataTypeConversion: '<S407>/Data Type Conversion2'
  //   DataTypeConversion: '<S407>/Data Type Conversion3'
  //   Sum: '<S407>/Sum2'

  QuadModelv_Y.MavHILSensor.time_usec = rtb_time_usec;
  QuadModelv_Y.MavHILSensor.xacc = static_cast<real32_T>
    (rtb_TmpSignalConversionAtSFun_1);
  QuadModelv_Y.MavHILSensor.yacc = static_cast<real32_T>
    (rtb_DataTypeConversion_idx_0);
  QuadModelv_Y.MavHILSensor.zacc = static_cast<real32_T>
    (rtb_DataTypeConversion_idx_1);
  QuadModelv_Y.MavHILSensor.xgyro = static_cast<real32_T>
    (rtb_DataTypeConversion_idx_2 + QuadModelv_B.sf_AccNoiseFun_a.y[0]);
  QuadModelv_Y.MavHILSensor.ygyro = static_cast<real32_T>(y +
    QuadModelv_B.sf_AccNoiseFun_a.y[1]);

  // Saturate: '<S428>/Saturation'
  if (rtb_TmpSignalConversionAtSFun_0 > QuadModelv_P.Saturation_UpperSat_j[2]) {
    rtb_TmpSignalConversionAtSFun_0 = QuadModelv_P.Saturation_UpperSat_j[2];
  } else if (rtb_TmpSignalConversionAtSFun_0 <
             QuadModelv_P.Saturation_LowerSat_a[2]) {
    rtb_TmpSignalConversionAtSFun_0 = QuadModelv_P.Saturation_LowerSat_a[2];
  }

  // Outport: '<Root>/MavHILSensor' incorporates:
  //   DataTypeConversion: '<S407>/Data Type Conversion3'
  //   DataTypeConversion: '<S407>/Data Type Conversion4'
  //   Gain: '<S407>/Gain_Mag'
  //   Gain: '<S407>/nT2Gauss'
  //   Sum: '<S407>/Sum2'
  //   Sum: '<S407>/Sum3'

  QuadModelv_Y.MavHILSensor.zgyro = static_cast<real32_T>
    (rtb_TmpSignalConversionAtSFun_0 + QuadModelv_B.sf_AccNoiseFun_a.y[2]);
  QuadModelv_Y.MavHILSensor.xmag = static_cast<real32_T>
    (QuadModelv_P.Gain_Mag_Gain * rtb_DataTypeConversion1[0] *
     QuadModelv_P.nT2Gauss_Gain + QuadModelv_B.sf_AccNoiseFun_p.y[0]);
  QuadModelv_Y.MavHILSensor.ymag = static_cast<real32_T>
    (QuadModelv_P.Gain_Mag_Gain * rtb_DataTypeConversion1[1] *
     QuadModelv_P.nT2Gauss_Gain + QuadModelv_B.sf_AccNoiseFun_p.y[1]);
  QuadModelv_Y.MavHILSensor.zmag = static_cast<real32_T>
    (QuadModelv_P.Gain_Mag_Gain * rtb_DataTypeConversion1[2] *
     QuadModelv_P.nT2Gauss_Gain + QuadModelv_B.sf_AccNoiseFun_p.y[2]);

  // Saturate: '<S417>/Limit  altitude  to Stratosphere'
  if (j > QuadModelv_P.LimitaltitudetoStratosphere_U_p) {
    j = QuadModelv_P.LimitaltitudetoStratosphere_U_p;
  } else if (j < QuadModelv_P.LimitaltitudetoStratosphere_L_f) {
    j = QuadModelv_P.LimitaltitudetoStratosphere_L_f;
  }

  // End of Saturate: '<S417>/Limit  altitude  to Stratosphere'

  // Outport: '<Root>/MavHILSensor' incorporates:
  //   BusCreator: '<S403>/Bus Creator'
  //   Constant: '<S5>/Constant1'
  //   DataTypeConversion: '<S407>/Data Type Conversion5'
  //   Gain: '<S407>/Gain'
  //   Gain: '<S407>/Gain1'
  //   Gain: '<S417>/P0'
  //   Gain: '<S417>/g//R'
  //   Math: '<S417>/Stratosphere Model'
  //   Product: '<S417>/Product1'
  //   Product: '<S417>/Product2'
  //   Sum: '<S407>/Sum'
  //   Sum: '<S5>/Sum2'
  //   UnitConversion: '<S65>/Unit Conversion'
  //
  //  About '<S417>/Stratosphere Model':
  //   Operator: exp

  QuadModelv_Y.MavHILSensor.abs_pressure = static_cast<real32_T>(std::exp(1.0 /
    rtb_q3dot * (QuadModelv_P.gR_Gain_d * j)) * (QuadModelv_P.P0_Gain * Z) *
    QuadModelv_P.Gain_Gain_io);
  QuadModelv_Y.MavHILSensor.diff_pressure = static_cast<real32_T>
    (QuadModelv_P.Gain1_Gain_g * rtb_q2dot + QuadModelv_B.Product4_h);
  QuadModelv_Y.MavHILSensor.pressure_alt = static_cast<real32_T>(rtb_fcn3);
  QuadModelv_Y.MavHILSensor.temperature = static_cast<real32_T>((rtb_Rn +
    -273.15000000000003) + QuadModelv_P.Constant1_Value_j);
  QuadModelv_Y.MavHILSensor.fields_updated = QuadModelv_B.fields_updated;

  // If: '<S329>/If' incorporates:
  //   Merge: '<S329>/Merge'

  if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    rtAction = static_cast<int8_T>(!(rtb_UniformRandomNumber7_idx_2 > 0.0));
    QuadModelv_DW.If_ActiveSubsystem_m = rtAction;
  } else {
    rtAction = QuadModelv_DW.If_ActiveSubsystem_m;
  }

  switch (rtAction) {
   case 0:
    // Outputs for IfAction SubSystem: '<S329>/Positive Trace' incorporates:
    //   ActionPort: '<S336>/Action Port'

    QuadModelv_PositiveTrace(rtb_UniformRandomNumber7_idx_2, VectorConcatenate,
      &QuadModelv_B.Merge_f[0], &QuadModelv_B.Merge_f[1],
      &QuadModelv_P.PositiveTrace_b);

    // End of Outputs for SubSystem: '<S329>/Positive Trace'
    break;

   case 1:
    // Outputs for IfAction SubSystem: '<S329>/Negative Trace' incorporates:
    //   ActionPort: '<S335>/Action Port'

    QuadModelv_NegativeTrace(VectorConcatenate, QuadModelv_B.Merge_f,
      &QuadModelv_DW.NegativeTrace_l, &QuadModelv_P.NegativeTrace_l);

    // End of Outputs for SubSystem: '<S329>/Negative Trace'
    break;
  }

  // End of If: '<S329>/If'

  // DataTypeConversion: '<S332>/Data Type Conversion7' incorporates:
  //   Gain: '<S332>/Gain4'

  rtb_TmpSignalConversionAtSFun_0 = rt_roundd_snf(QuadModelv_P.Gain4_Gain *
    QuadModelv_B.Product[0]);
  if (rtIsNaN(rtb_TmpSignalConversionAtSFun_0) || rtIsInf
      (rtb_TmpSignalConversionAtSFun_0)) {
    rtb_TmpSignalConversionAtSFun_0 = 0.0;
  } else {
    rtb_TmpSignalConversionAtSFun_0 = std::fmod(rtb_TmpSignalConversionAtSFun_0,
      65536.0);
  }

  rtb_sincos_o2_idx_0_tmp = rt_roundd_snf(QuadModelv_P.Gain4_Gain *
    QuadModelv_B.Product[1]);
  if (rtIsNaN(rtb_sincos_o2_idx_0_tmp) || rtIsInf(rtb_sincos_o2_idx_0_tmp)) {
    rtb_sincos_o2_idx_0_tmp = 0.0;
  } else {
    rtb_sincos_o2_idx_0_tmp = std::fmod(rtb_sincos_o2_idx_0_tmp, 65536.0);
  }

  rtb_Rn = rt_roundd_snf(QuadModelv_P.Gain4_Gain * QuadModelv_B.Product[2]);
  if (rtIsNaN(rtb_Rn) || rtIsInf(rtb_Rn)) {
    rtb_Rn = 0.0;
  } else {
    rtb_Rn = std::fmod(rtb_Rn, 65536.0);
  }

  // DataTypeConversion: '<S332>/Data Type Conversion9' incorporates:
  //   Gain: '<S332>/Gain2'
  //   Gain: '<S332>/Gain7'
  //   Gain: '<S519>/1//2rhoV^2'
  //   Product: '<S519>/Product2'
  //   Sqrt: '<S332>/Airspeed1'
  //   Sqrt: '<S332>/Sqrt1'

  rtb_Sum1_c = rt_roundd_snf(std::sqrt(rtb_q2dot_tmp * rtb_Sum1_c *
    QuadModelv_P.u2rhoV2_Gain_a * QuadModelv_P.Gain2_Gain_j) *
    QuadModelv_P.Gain7_Gain);
  if (rtIsNaN(rtb_Sum1_c) || rtIsInf(rtb_Sum1_c)) {
    rtb_Sum1_c = 0.0;
  } else {
    rtb_Sum1_c = std::fmod(rtb_Sum1_c, 65536.0);
  }

  rtb_UniformRandomNumber7_idx_2 = rt_roundd_snf(std::sqrt(rtb_q2dot_tmp) *
    QuadModelv_P.Gain7_Gain);
  if (rtIsNaN(rtb_UniformRandomNumber7_idx_2) || rtIsInf
      (rtb_UniformRandomNumber7_idx_2)) {
    rtb_UniformRandomNumber7_idx_2 = 0.0;
  } else {
    rtb_UniformRandomNumber7_idx_2 = std::fmod(rtb_UniformRandomNumber7_idx_2,
      65536.0);
  }

  // DataTypeConversion: '<S332>/Data Type Conversion8' incorporates:
  //   DataTypeConversion: '<S407>/Data Type Conversion2'
  //   Gain: '<S332>/Gain5'

  tmp_2 = rt_roundf_snf(QuadModelv_P.Gain5_Gain_p * static_cast<real32_T>
                        (rtb_TmpSignalConversionAtSFun_1));
  if (rtIsNaNF(tmp_2) || rtIsInfF(tmp_2)) {
    tmp_2 = 0.0F;
  } else {
    tmp_2 = std::fmod(tmp_2, 65536.0F);
  }

  tmp_1 = rt_roundf_snf(QuadModelv_P.Gain5_Gain_p * static_cast<real32_T>
                        (rtb_DataTypeConversion_idx_0));
  if (rtIsNaNF(tmp_1) || rtIsInfF(tmp_1)) {
    tmp_1 = 0.0F;
  } else {
    tmp_1 = std::fmod(tmp_1, 65536.0F);
  }

  tmp_0 = rt_roundf_snf(QuadModelv_P.Gain5_Gain_p * static_cast<real32_T>
                        (rtb_DataTypeConversion_idx_1));
  if (rtIsNaNF(tmp_0) || rtIsInfF(tmp_0)) {
    tmp_0 = 0.0F;
  } else {
    tmp_0 = std::fmod(tmp_0, 65536.0F);
  }

  // Outport: '<Root>/MavHILStateQuat' incorporates:
  //   BusCreator: '<S332>/Bus Creator'
  //   DataTypeConversion: '<S332>/Data Type Conversion2'
  //   DataTypeConversion: '<S332>/Data Type Conversion3'
  //   Integrator: '<S14>/p,q,r '

  QuadModelv_Y.MavHILStateQuat.time_usec = rtb_time_usec;
  QuadModelv_Y.MavHILStateQuat.attitude_quaternion[0] = static_cast<real32_T>
    (QuadModelv_B.Merge_f[0]);
  QuadModelv_Y.MavHILStateQuat.attitude_quaternion[1] = static_cast<real32_T>
    (QuadModelv_B.Merge_f[1]);
  QuadModelv_Y.MavHILStateQuat.attitude_quaternion[2] = static_cast<real32_T>
    (QuadModelv_B.Merge_f[2]);
  QuadModelv_Y.MavHILStateQuat.attitude_quaternion[3] = static_cast<real32_T>
    (QuadModelv_B.Merge_f[3]);
  QuadModelv_Y.MavHILStateQuat.rollspeed = static_cast<real32_T>
    (QuadModelv_X.pqr_CSTATE[0]);
  QuadModelv_Y.MavHILStateQuat.pitchspeed = static_cast<real32_T>
    (QuadModelv_X.pqr_CSTATE[1]);
  QuadModelv_Y.MavHILStateQuat.yawspeed = static_cast<real32_T>
    (QuadModelv_X.pqr_CSTATE[2]);

  // DataTypeConversion: '<S332>/Data Type Conversion4' incorporates:
  //   Gain: '<S332>/Gain'

  rtb_TmpSignalConversionAtSFun_1 = rt_roundd_snf(QuadModelv_P.Gain_Gain_b *
    LoadLeakRatio);
  if (rtIsNaN(rtb_TmpSignalConversionAtSFun_1) || rtIsInf
      (rtb_TmpSignalConversionAtSFun_1)) {
    rtb_TmpSignalConversionAtSFun_1 = 0.0;
  } else {
    rtb_TmpSignalConversionAtSFun_1 = std::fmod(rtb_TmpSignalConversionAtSFun_1,
      4.294967296E+9);
  }

  // Outport: '<Root>/MavHILStateQuat' incorporates:
  //   DataTypeConversion: '<S332>/Data Type Conversion4'

  QuadModelv_Y.MavHILStateQuat.lat = rtb_TmpSignalConversionAtSFun_1 < 0.0 ? -
    static_cast<int32_T>(static_cast<uint32_T>(-rtb_TmpSignalConversionAtSFun_1))
    : static_cast<int32_T>(static_cast<uint32_T>(rtb_TmpSignalConversionAtSFun_1));

  // DataTypeConversion: '<S332>/Data Type Conversion1' incorporates:
  //   Gain: '<S332>/Gain1'

  rtb_TmpSignalConversionAtSFun_1 = rt_roundd_snf(QuadModelv_P.Gain1_Gain_k * Y);
  if (rtIsNaN(rtb_TmpSignalConversionAtSFun_1) || rtIsInf
      (rtb_TmpSignalConversionAtSFun_1)) {
    rtb_TmpSignalConversionAtSFun_1 = 0.0;
  } else {
    rtb_TmpSignalConversionAtSFun_1 = std::fmod(rtb_TmpSignalConversionAtSFun_1,
      4.294967296E+9);
  }

  // Outport: '<Root>/MavHILStateQuat' incorporates:
  //   DataTypeConversion: '<S332>/Data Type Conversion1'

  QuadModelv_Y.MavHILStateQuat.lon = rtb_TmpSignalConversionAtSFun_1 < 0.0 ? -
    static_cast<int32_T>(static_cast<uint32_T>(-rtb_TmpSignalConversionAtSFun_1))
    : static_cast<int32_T>(static_cast<uint32_T>(rtb_TmpSignalConversionAtSFun_1));

  // DataTypeConversion: '<S332>/Data Type Conversion5' incorporates:
  //   Gain: '<S332>/Gain3'

  rtb_TmpSignalConversionAtSFun_1 = rt_roundd_snf(QuadModelv_P.Gain3_Gain *
    rtb_Saturation_1);
  if (rtIsNaN(rtb_TmpSignalConversionAtSFun_1) || rtIsInf
      (rtb_TmpSignalConversionAtSFun_1)) {
    rtb_TmpSignalConversionAtSFun_1 = 0.0;
  } else {
    rtb_TmpSignalConversionAtSFun_1 = std::fmod(rtb_TmpSignalConversionAtSFun_1,
      4.294967296E+9);
  }

  // Outport: '<Root>/MavHILStateQuat' incorporates:
  //   DataTypeConversion: '<S332>/Data Type Conversion5'
  //   DataTypeConversion: '<S332>/Data Type Conversion7'
  //   DataTypeConversion: '<S332>/Data Type Conversion8'
  //   DataTypeConversion: '<S332>/Data Type Conversion9'

  QuadModelv_Y.MavHILStateQuat.alt = rtb_TmpSignalConversionAtSFun_1 < 0.0 ? -
    static_cast<int32_T>(static_cast<uint32_T>(-rtb_TmpSignalConversionAtSFun_1))
    : static_cast<int32_T>(static_cast<uint32_T>(rtb_TmpSignalConversionAtSFun_1));
  QuadModelv_Y.MavHILStateQuat.vx = static_cast<int16_T>
    (rtb_TmpSignalConversionAtSFun_0 < 0.0 ? static_cast<int32_T>
     (static_cast<int16_T>(-static_cast<int16_T>(static_cast<uint16_T>
        (-rtb_TmpSignalConversionAtSFun_0)))) : static_cast<int32_T>(
      static_cast<int16_T>(static_cast<uint16_T>(rtb_TmpSignalConversionAtSFun_0))));
  QuadModelv_Y.MavHILStateQuat.vy = static_cast<int16_T>(rtb_sincos_o2_idx_0_tmp
    < 0.0 ? static_cast<int32_T>(static_cast<int16_T>(-static_cast<int16_T>(
    static_cast<uint16_T>(-rtb_sincos_o2_idx_0_tmp)))) : static_cast<int32_T>(
    static_cast<int16_T>(static_cast<uint16_T>(rtb_sincos_o2_idx_0_tmp))));
  QuadModelv_Y.MavHILStateQuat.vz = static_cast<int16_T>(rtb_Rn < 0.0 ?
    static_cast<int32_T>(static_cast<int16_T>(-static_cast<int16_T>
    (static_cast<uint16_T>(-rtb_Rn)))) : static_cast<int32_T>
    (static_cast<int16_T>(static_cast<uint16_T>(rtb_Rn))));
  QuadModelv_Y.MavHILStateQuat.ind_airspeed = static_cast<uint16_T>(rtb_Sum1_c <
    0.0 ? static_cast<int32_T>(static_cast<uint16_T>(-static_cast<int16_T>(
    static_cast<uint16_T>(-rtb_Sum1_c)))) : static_cast<int32_T>
    (static_cast<uint16_T>(rtb_Sum1_c)));
  QuadModelv_Y.MavHILStateQuat.true_airspeed = static_cast<uint16_T>
    (rtb_UniformRandomNumber7_idx_2 < 0.0 ? static_cast<int32_T>
     (static_cast<uint16_T>(-static_cast<int16_T>(static_cast<uint16_T>
        (-rtb_UniformRandomNumber7_idx_2)))) : static_cast<int32_T>
     (static_cast<uint16_T>(rtb_UniformRandomNumber7_idx_2)));
  QuadModelv_Y.MavHILStateQuat.xacc = static_cast<int16_T>(tmp_2 < 0.0F ?
    static_cast<int32_T>(static_cast<int16_T>(-static_cast<int16_T>
    (static_cast<uint16_T>(-tmp_2)))) : static_cast<int32_T>(static_cast<int16_T>
    (static_cast<uint16_T>(tmp_2))));
  QuadModelv_Y.MavHILStateQuat.yacc = static_cast<int16_T>(tmp_1 < 0.0F ?
    static_cast<int32_T>(static_cast<int16_T>(-static_cast<int16_T>(static_cast<
    uint16_T>(-tmp_1)))) : static_cast<int32_T>(static_cast<int16_T>(
    static_cast<uint16_T>(tmp_1))));
  QuadModelv_Y.MavHILStateQuat.zacc = static_cast<int16_T>(tmp_0 < 0.0F ?
    static_cast<int32_T>(static_cast<int16_T>(-static_cast<int16_T>(static_cast<
    uint16_T>(-tmp_0)))) : static_cast<int32_T>(static_cast<int16_T>(
    static_cast<uint16_T>(tmp_0))));

  // MATLAB Function: '<S12>/ZLimit' incorporates:
  //   Integrator: '<S14>/xe,ye,ze'

  // MATLAB Function 'OutputPort1/ZLimit': '<S334>:1'
  // '<S334>:1:2' Xee=xe;
  rtb_UniformRandomNumber7_idx_2 = QuadModelv_X.xeyeze_CSTATE[2];

  // '<S334>:1:3' if Xee(3)>zt
  if (QuadModelv_X.xeyeze_CSTATE[2] > QuadModelv_B.y_k) {
    // '<S334>:1:4' Xee(3)=zt;
    rtb_UniformRandomNumber7_idx_2 = QuadModelv_B.y_k;
  }

  // End of MATLAB Function: '<S12>/ZLimit'
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[5] == 0) {
    // Gain: '<S374>/BiasGain2' incorporates:
    //   UniformRandomNumber: '<S374>/Uniform Random Number5'

    QuadModelv_B.BiasGain2[0] = QuadModelv_P.BiasGain2_Gain *
      QuadModelv_DW.UniformRandomNumber5_NextOutp_m[0];
    QuadModelv_B.BiasGain2[1] = QuadModelv_P.BiasGain2_Gain *
      QuadModelv_DW.UniformRandomNumber5_NextOutp_m[1];
    QuadModelv_B.BiasGain2[2] = QuadModelv_P.BiasGain2_Gain *
      QuadModelv_DW.UniformRandomNumber5_NextOutp_m[2];
  }

  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // MATLAB Function: '<S374>/FaultParamsExtract' incorporates:
    //   Constant: '<S374>/FaultID'
    //   Inport: '<Root>/inSILInts'

    // MATLAB Function 'GPSFault/FaultParamsExtract': '<S378>:1'
    // '<S378>:1:5' if isempty(hFault)
    // '<S378>:1:8' if isempty(fParam)
    // '<S378>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S378>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S378>:1:14' j=1;
    j = 1.0;

    // '<S378>:1:15' for i=1:8
    for (b_t = 0; b_t < 8; b_t++) {
      // '<S378>:1:16' if inInts(i) == FaultID
      if (QuadModelv_U.inSILInts[b_t] == QuadModelv_P.FaultID_Value_p) {
        // '<S378>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S378>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        fParamTmp[static_cast<int32_T>(2.0 * j - 1.0) - 1] = inSILFloats[((b_t +
          1) << 1) - 2];

        // '<S378>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[static_cast<int32_T>(2.0 * j) - 1] = inSILFloats[((b_t + 1) <<
          1) - 1];

        // '<S378>:1:20' j=j+1;
        j++;
      }
    }

    // '<S378>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S378>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_d = true;

      // '<S378>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = inSILFloats[16];
      fParamTmp[17] = inSILFloats[17];
      fParamTmp[18] = inSILFloats[18];
      fParamTmp[19] = inSILFloats[19];

      // '<S378>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_dy[0], &fParamTmp[0], 20U * sizeof
                  (real_T));
    }

    // MATLAB Function: '<S377>/Acc NoiseFun' incorporates:
    //   MATLAB Function: '<S374>/FaultParamsExtract'

    // '<S378>:1:29' hasFault_GPS=hFault;
    // '<S378>:1:30' FaultParam=fParam;
    // MATLAB Function 'GPSFault/AccNoiseSwitch1/Acc NoiseFun': '<S380>:1'
    // '<S380>:1:3' y = 0.3*u;
    QuadModelv_B.y_m[0] = 0.3 * QuadModelv_B.BiasGain2[0];
    QuadModelv_B.y_m[1] = 0.3 * QuadModelv_B.BiasGain2[1];
    QuadModelv_B.y_m[2] = 0.3 * QuadModelv_B.BiasGain2[2];

    // '<S380>:1:5' if isAccFault
    if (QuadModelv_DW.hFault_d) {
      // '<S380>:1:6' y = (0.3+0.7*AccFaultParams(1))*u;
      rtb_Rn = 0.7 * QuadModelv_DW.fParam_dy[0] + 0.3;
      QuadModelv_B.y_m[0] = rtb_Rn * QuadModelv_B.BiasGain2[0];
      QuadModelv_B.y_m[1] = rtb_Rn * QuadModelv_B.BiasGain2[1];
      QuadModelv_B.y_m[2] = rtb_Rn * QuadModelv_B.BiasGain2[2];
    }

    // End of MATLAB Function: '<S377>/Acc NoiseFun'

    // UnitConversion: '<S398>/Unit Conversion' incorporates:
    //   Constant: '<S379>/ref_rotation'

    // Unit Conversion - from: deg to: rad
    // Expression: output = (0.0174533*input) + (0)
    Z = 0.017453292519943295 * QuadModelv_P.FlatEarthtoLLA_psi_e;

    // Trigonometry: '<S383>/SinCos'
    QuadModelv_B.SinCos_o1_m = std::sin(Z);

    // Trigonometry: '<S383>/SinCos'
    QuadModelv_B.SinCos_o2_i = std::cos(Z);

    // Sum: '<S401>/Sum' incorporates:
    //   Constant: '<S401>/Constant'
    //   Constant: '<S401>/f'

    Z = QuadModelv_P.f_Value_l - QuadModelv_P.Constant_Value_na;

    // Sqrt: '<S402>/sqrt' incorporates:
    //   Constant: '<S402>/Constant'
    //   Product: '<S402>/Product1'
    //   Sum: '<S402>/Sum1'

    Z = std::sqrt(QuadModelv_P.Constant_Value_ol - Z * Z);

    // Switch: '<S394>/Switch' incorporates:
    //   Abs: '<S394>/Abs'
    //   Bias: '<S394>/Bias'
    //   Bias: '<S394>/Bias1'
    //   Constant: '<S379>/ref_position'
    //   Constant: '<S394>/Constant2'
    //   Constant: '<S395>/Constant'
    //   Math: '<S394>/Math Function1'
    //   RelationalOperator: '<S395>/Compare'

    if (std::abs(QuadModelv_P.ModelParam_GPSLatLong[0]) >
        QuadModelv_P.CompareToConstant_const_b) {
      j = rt_modd_snf(QuadModelv_P.ModelParam_GPSLatLong[0] +
                      QuadModelv_P.Bias_Bias_j, QuadModelv_P.Constant2_Value_m)
        + QuadModelv_P.Bias1_Bias_o4;
    } else {
      j = QuadModelv_P.ModelParam_GPSLatLong[0];
    }

    // End of Switch: '<S394>/Switch'

    // Abs: '<S391>/Abs1'
    uavlostJzx = std::abs(j);

    // RelationalOperator: '<S393>/Compare' incorporates:
    //   Constant: '<S393>/Constant'

    rtb_Compare_b5 = (uavlostJzx > QuadModelv_P.CompareToConstant_const_g);

    // Switch: '<S391>/Switch'
    if (rtb_Compare_b5) {
      // Signum: '<S391>/Sign1'
      if (!rtIsNaN(j)) {
        if (j < 0.0) {
          j = -1.0;
        } else {
          j = (j > 0.0);
        }
      }

      // End of Signum: '<S391>/Sign1'

      // Switch: '<S391>/Switch' incorporates:
      //   Bias: '<S391>/Bias'
      //   Bias: '<S391>/Bias1'
      //   Gain: '<S391>/Gain'
      //   Product: '<S391>/Divide1'

      QuadModelv_B.Switch_h = ((uavlostJzx + QuadModelv_P.Bias_Bias_d) *
        QuadModelv_P.Gain_Gain_i + QuadModelv_P.Bias1_Bias_at) * j;
    } else {
      // Switch: '<S391>/Switch'
      QuadModelv_B.Switch_h = j;
    }

    // End of Switch: '<S391>/Switch'

    // UnitConversion: '<S399>/Unit Conversion'
    // Unit Conversion - from: deg to: rad
    // Expression: output = (0.0174533*input) + (0)
    uavlostJzx = 0.017453292519943295 * QuadModelv_B.Switch_h;

    // Trigonometry: '<S400>/Trigonometric Function1'
    rtb_Sum1_c = std::sin(uavlostJzx);

    // Product: '<S400>/Product1' incorporates:
    //   Product: '<S397>/Product2'

    rtb_TmpSignalConversionAtSFun_0 = Z * Z;

    // Sum: '<S400>/Sum1' incorporates:
    //   Constant: '<S400>/Constant'
    //   Product: '<S400>/Product1'

    rtb_Sum1_c = QuadModelv_P.Constant_Value_cc -
      rtb_TmpSignalConversionAtSFun_0 * rtb_Sum1_c * rtb_Sum1_c;

    // Product: '<S397>/Product1' incorporates:
    //   Constant: '<S397>/Constant1'
    //   Sqrt: '<S397>/sqrt'

    rtb_Rn = QuadModelv_P.Constant1_Value_d0 / std::sqrt(rtb_Sum1_c);

    // Trigonometry: '<S397>/Trigonometric Function1' incorporates:
    //   Constant: '<S397>/Constant'
    //   Constant: '<S397>/Constant2'
    //   Product: '<S397>/Product3'
    //   Sum: '<S397>/Sum1'

    QuadModelv_B.TrigonometricFunction1_d = rt_atan2d_snf
      (QuadModelv_P.Constant2_Value_k, (QuadModelv_P.Constant_Value_b -
        rtb_TmpSignalConversionAtSFun_0) * rtb_Rn / rtb_Sum1_c);

    // Trigonometry: '<S397>/Trigonometric Function2' incorporates:
    //   Constant: '<S397>/Constant3'
    //   Product: '<S397>/Product4'
    //   Trigonometry: '<S397>/Trigonometric Function'

    QuadModelv_B.TrigonometricFunction2_i = rt_atan2d_snf
      (QuadModelv_P.Constant3_Value_l, rtb_Rn * std::cos(uavlostJzx));
  }

  // Sum: '<S374>/Sum' incorporates:
  //   Integrator: '<S14>/xe,ye,ze'

  rtb_DataTypeConversion_idx_0 = QuadModelv_X.xeyeze_CSTATE[0] +
    QuadModelv_B.y_m[0];
  rtb_DataTypeConversion_idx_1 = QuadModelv_X.xeyeze_CSTATE[1] +
    QuadModelv_B.y_m[1];

  // Unit Conversion - from: rad to: deg
  // Expression: output = (57.2958*input) + (0)
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Switch: '<S382>/Switch1' incorporates:
    //   Constant: '<S382>/Constant'
    //   Constant: '<S382>/Constant1'

    if (rtb_Compare_b5) {
      rtb_TmpSignalConversionAtSFun_0 = QuadModelv_P.Constant_Value_d;
    } else {
      rtb_TmpSignalConversionAtSFun_0 = QuadModelv_P.Constant1_Value_b;
    }

    // End of Switch: '<S382>/Switch1'

    // Sum: '<S382>/Sum' incorporates:
    //   Constant: '<S379>/ref_position'

    j = rtb_TmpSignalConversionAtSFun_0 + QuadModelv_P.ModelParam_GPSLatLong[1];

    // Switch: '<S392>/Switch' incorporates:
    //   Abs: '<S392>/Abs'
    //   Constant: '<S396>/Constant'
    //   RelationalOperator: '<S396>/Compare'

    if (std::abs(j) > QuadModelv_P.CompareToConstant_const_iz) {
      // Switch: '<S392>/Switch' incorporates:
      //   Bias: '<S392>/Bias'
      //   Bias: '<S392>/Bias1'
      //   Constant: '<S392>/Constant2'
      //   Math: '<S392>/Math Function1'

      QuadModelv_B.Switch_o = rt_modd_snf(j + QuadModelv_P.Bias_Bias_f,
        QuadModelv_P.Constant2_Value_h) + QuadModelv_P.Bias1_Bias_gl;
    } else {
      // Switch: '<S392>/Switch'
      QuadModelv_B.Switch_o = j;
    }

    // End of Switch: '<S392>/Switch'
  }

  // Sum: '<S379>/Sum' incorporates:
  //   Product: '<S383>/rad lat'
  //   Product: '<S383>/x*cos'
  //   Product: '<S383>/y*sin'
  //   Sum: '<S383>/Sum'
  //   UnitConversion: '<S384>/Unit Conversion'

  j = (rtb_DataTypeConversion_idx_0 * QuadModelv_B.SinCos_o2_i -
       rtb_DataTypeConversion_idx_1 * QuadModelv_B.SinCos_o1_m) *
    QuadModelv_B.TrigonometricFunction1_d * 57.295779513082323 +
    QuadModelv_B.Switch_h;

  // Switch: '<S388>/Switch' incorporates:
  //   Abs: '<S388>/Abs'
  //   Bias: '<S388>/Bias'
  //   Bias: '<S388>/Bias1'
  //   Constant: '<S388>/Constant2'
  //   Constant: '<S389>/Constant'
  //   Math: '<S388>/Math Function1'
  //   RelationalOperator: '<S389>/Compare'

  if (std::abs(j) > QuadModelv_P.CompareToConstant_const_bz) {
    rtb_Rn = rt_modd_snf(j + QuadModelv_P.Bias_Bias_ee,
                         QuadModelv_P.Constant2_Value_nr) +
      QuadModelv_P.Bias1_Bias_k;
  } else {
    rtb_Rn = j;
  }

  // End of Switch: '<S388>/Switch'

  // Abs: '<S385>/Abs1'
  j = std::abs(rtb_Rn);

  // RelationalOperator: '<S387>/Compare' incorporates:
  //   Constant: '<S387>/Constant'

  hFaultTmp = (j > QuadModelv_P.CompareToConstant_const_d);

  // Switch: '<S385>/Switch' incorporates:
  //   Bias: '<S385>/Bias'
  //   Bias: '<S385>/Bias1'
  //   Gain: '<S385>/Gain'
  //   Product: '<S385>/Divide1'

  if (hFaultTmp) {
    // Signum: '<S385>/Sign1'
    if (!rtIsNaN(rtb_Rn)) {
      if (rtb_Rn < 0.0) {
        rtb_Rn = -1.0;
      } else {
        rtb_Rn = (rtb_Rn > 0.0);
      }
    }

    // End of Signum: '<S385>/Sign1'
    rtb_Rn *= (j + QuadModelv_P.Bias_Bias_e) * QuadModelv_P.Gain_Gain_d +
      QuadModelv_P.Bias1_Bias_g2;
  }

  // End of Switch: '<S385>/Switch'

  // DataTypeConversion: '<S330>/Data Type Conversion4' incorporates:
  //   Gain: '<S330>/latScale'

  rtb_TmpSignalConversionAtSFun_0 = rt_roundd_snf(QuadModelv_P.latScale_Gain *
    rtb_Rn);
  if (rtIsNaN(rtb_TmpSignalConversionAtSFun_0) || rtIsInf
      (rtb_TmpSignalConversionAtSFun_0)) {
    rtb_TmpSignalConversionAtSFun_0 = 0.0;
  } else {
    rtb_TmpSignalConversionAtSFun_0 = std::fmod(rtb_TmpSignalConversionAtSFun_0,
      4.294967296E+9);
  }

  r = rtb_TmpSignalConversionAtSFun_0 < 0.0 ? -static_cast<int32_T>
    (static_cast<uint32_T>(-rtb_TmpSignalConversionAtSFun_0)) :
    static_cast<int32_T>(static_cast<uint32_T>(rtb_TmpSignalConversionAtSFun_0));

  // End of DataTypeConversion: '<S330>/Data Type Conversion4'

  // Switch: '<S381>/Switch1' incorporates:
  //   Constant: '<S381>/Constant'
  //   Constant: '<S381>/Constant1'

  if (hFaultTmp) {
    rtb_Rn = QuadModelv_P.Constant_Value_bd;
  } else {
    rtb_Rn = QuadModelv_P.Constant1_Value_f;
  }

  // End of Switch: '<S381>/Switch1'

  // Sum: '<S381>/Sum' incorporates:
  //   Product: '<S383>/rad long '
  //   Product: '<S383>/x*sin'
  //   Product: '<S383>/y*cos'
  //   Sum: '<S379>/Sum'
  //   Sum: '<S383>/Sum1'
  //   UnitConversion: '<S384>/Unit Conversion'

  rtb_Rn += (rtb_DataTypeConversion_idx_0 * QuadModelv_B.SinCos_o1_m +
             rtb_DataTypeConversion_idx_1 * QuadModelv_B.SinCos_o2_i) *
    QuadModelv_B.TrigonometricFunction2_i * 57.295779513082323 +
    QuadModelv_B.Switch_o;

  // Switch: '<S386>/Switch' incorporates:
  //   Abs: '<S386>/Abs'
  //   Bias: '<S386>/Bias'
  //   Bias: '<S386>/Bias1'
  //   Constant: '<S386>/Constant2'
  //   Constant: '<S390>/Constant'
  //   Math: '<S386>/Math Function1'
  //   RelationalOperator: '<S390>/Compare'

  if (std::abs(rtb_Rn) > QuadModelv_P.CompareToConstant_const_ex) {
    rtb_Rn = rt_modd_snf(rtb_Rn + QuadModelv_P.Bias_Bias_gf,
                         QuadModelv_P.Constant2_Value_g) +
      QuadModelv_P.Bias1_Bias_a;
  }

  // End of Switch: '<S386>/Switch'

  // DataTypeConversion: '<S330>/Data Type Conversion5' incorporates:
  //   Gain: '<S330>/lonScale'

  rtb_TmpSignalConversionAtSFun_0 = rt_roundd_snf(QuadModelv_P.lonScale_Gain *
    rtb_Rn);
  if (rtIsNaN(rtb_TmpSignalConversionAtSFun_0) || rtIsInf
      (rtb_TmpSignalConversionAtSFun_0)) {
    rtb_TmpSignalConversionAtSFun_0 = 0.0;
  } else {
    rtb_TmpSignalConversionAtSFun_0 = std::fmod(rtb_TmpSignalConversionAtSFun_0,
      4.294967296E+9);
  }

  b_t = rtb_TmpSignalConversionAtSFun_0 < 0.0 ? -static_cast<int32_T>(
    static_cast<uint32_T>(-rtb_TmpSignalConversionAtSFun_0)) :
    static_cast<int32_T>(static_cast<uint32_T>(rtb_TmpSignalConversionAtSFun_0));

  // End of DataTypeConversion: '<S330>/Data Type Conversion5'

  // Sum: '<S379>/Sum1' incorporates:
  //   Constant: '<S374>/ModelParam.envAltitude'
  //   Sum: '<S374>/Sum'
  //   UnaryMinus: '<S379>/Ze2height'

  j = -(rtb_UniformRandomNumber7_idx_2 + QuadModelv_B.y_m[2]) -
    QuadModelv_P.ModelParam_envAltitude;

  // Saturate: '<S374>/Saturation'
  if (j > QuadModelv_P.Saturation_UpperSat_h) {
    j = QuadModelv_P.Saturation_UpperSat_h;
  } else if (j < QuadModelv_P.Saturation_LowerSat_au) {
    j = QuadModelv_P.Saturation_LowerSat_au;
  }

  // End of Saturate: '<S374>/Saturation'

  // DataTypeConversion: '<S330>/Data Type Conversion6' incorporates:
  //   Gain: '<S330>/altScale'

  rtb_TmpSignalConversionAtSFun_0 = rt_roundd_snf(QuadModelv_P.altScale_Gain * j);
  if (rtIsNaN(rtb_TmpSignalConversionAtSFun_0) || rtIsInf
      (rtb_TmpSignalConversionAtSFun_0)) {
    rtb_TmpSignalConversionAtSFun_0 = 0.0;
  } else {
    rtb_TmpSignalConversionAtSFun_0 = std::fmod(rtb_TmpSignalConversionAtSFun_0,
      4.294967296E+9);
  }

  rtb_alt = rtb_TmpSignalConversionAtSFun_0 < 0.0 ? -static_cast<int32_T>(
    static_cast<uint32_T>(-rtb_TmpSignalConversionAtSFun_0)) : static_cast<
    int32_T>(static_cast<uint32_T>(rtb_TmpSignalConversionAtSFun_0));

  // End of DataTypeConversion: '<S330>/Data Type Conversion6'
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // DataTypeConversion: '<S330>/Data Type Conversion8' incorporates:
    //   Constant: '<S330>/ModelParam.GPSEphFinal'
    //   Gain: '<S330>/Gain6'

    rtb_TmpSignalConversionAtSFun_0 = rt_roundd_snf(QuadModelv_P.Gain6_Gain_i *
      QuadModelv_P.ModelParam_GPSEphFinal);
    if (rtIsNaN(rtb_TmpSignalConversionAtSFun_0) || rtIsInf
        (rtb_TmpSignalConversionAtSFun_0)) {
      rtb_TmpSignalConversionAtSFun_0 = 0.0;
    } else {
      rtb_TmpSignalConversionAtSFun_0 = std::fmod
        (rtb_TmpSignalConversionAtSFun_0, 65536.0);
    }

    // DataTypeConversion: '<S330>/Data Type Conversion8'
    QuadModelv_B.eph = static_cast<uint16_T>(rtb_TmpSignalConversionAtSFun_0 <
      0.0 ? static_cast<int32_T>(static_cast<uint16_T>(-static_cast<int16_T>(
      static_cast<uint16_T>(-rtb_TmpSignalConversionAtSFun_0)))) :
      static_cast<int32_T>(static_cast<uint16_T>(rtb_TmpSignalConversionAtSFun_0)));

    // DataTypeConversion: '<S330>/Data Type Conversion9' incorporates:
    //   Constant: '<S330>/ModelParam.GPSEpvFinal'
    //   Gain: '<S330>/Gain8'

    rtb_TmpSignalConversionAtSFun_0 = rt_roundd_snf(QuadModelv_P.Gain8_Gain *
      QuadModelv_P.ModelParam_GPSEpvFinal);
    if (rtIsNaN(rtb_TmpSignalConversionAtSFun_0) || rtIsInf
        (rtb_TmpSignalConversionAtSFun_0)) {
      rtb_TmpSignalConversionAtSFun_0 = 0.0;
    } else {
      rtb_TmpSignalConversionAtSFun_0 = std::fmod
        (rtb_TmpSignalConversionAtSFun_0, 65536.0);
    }

    // DataTypeConversion: '<S330>/Data Type Conversion9'
    QuadModelv_B.epv = static_cast<uint16_T>(rtb_TmpSignalConversionAtSFun_0 <
      0.0 ? static_cast<int32_T>(static_cast<uint16_T>(-static_cast<int16_T>(
      static_cast<uint16_T>(-rtb_TmpSignalConversionAtSFun_0)))) :
      static_cast<int32_T>(static_cast<uint16_T>(rtb_TmpSignalConversionAtSFun_0)));
  }

  // Sum: '<S330>/Sum3' incorporates:
  //   TransferFcn: '<S376>/Transfer Fcn1'
  //   TransferFcn: '<S376>/Transfer Fcn4'

  rtb_TmpSignalConversionAtSFun_0 = QuadModelv_P.TransferFcn4_C *
    QuadModelv_X.TransferFcn4_CSTATE + QuadModelv_B.Product[0];
  rtb_TmpSignalConversionAtSFun_1 = QuadModelv_P.TransferFcn1_C *
    QuadModelv_X.TransferFcn1_CSTATE + QuadModelv_B.Product[1];

  // MATLAB Function: '<S330>/GenCogVel'
  // MATLAB Function 'OutputPort1/HILGPSModel/GenCogVel': '<S375>:1'
  // '<S375>:1:2' v=norm(u(1:2));
  Z = 3.3121686421112381E-170;
  uavlostJzx = std::abs(rtb_TmpSignalConversionAtSFun_0);
  if (uavlostJzx > 3.3121686421112381E-170) {
    j = 1.0;
    Z = uavlostJzx;
  } else {
    rtb_Rn = uavlostJzx / 3.3121686421112381E-170;
    j = rtb_Rn * rtb_Rn;
  }

  uavlostJzx = std::abs(rtb_TmpSignalConversionAtSFun_1);
  if (uavlostJzx > Z) {
    rtb_Rn = Z / uavlostJzx;
    j = j * rtb_Rn * rtb_Rn + 1.0;
    Z = uavlostJzx;
  } else {
    rtb_Rn = uavlostJzx / Z;
    j += rtb_Rn * rtb_Rn;
  }

  j = Z * std::sqrt(j);

  // '<S375>:1:4' if v < 1
  if (j < 1.0) {
    // '<S375>:1:5' cot = 0;
    Z = 0.0;
  } else {
    // '<S375>:1:6' else
    // '<S375>:1:7' cot=atan2d(u(2),u(1));
    Z = 57.295779513082323 * rt_atan2d_snf(rtb_TmpSignalConversionAtSFun_1,
      rtb_TmpSignalConversionAtSFun_0);
  }

  // '<S375>:1:10' if cot<0
  if (Z < 0.0) {
    // '<S375>:1:11' cot=cot+360;
    Z += 360.0;
  }

  // DataTypeConversion: '<S330>/Data Type Conversion3' incorporates:
  //   Gain: '<S330>/VeScale'
  //   Sum: '<S330>/Sum3'
  //   TransferFcn: '<S376>/Transfer Fcn2'

  rtb_TmpSignalConversionAtSFun_0 = rt_roundd_snf(QuadModelv_P.VeScale_Gain *
    rtb_TmpSignalConversionAtSFun_0);
  if (rtIsNaN(rtb_TmpSignalConversionAtSFun_0) || rtIsInf
      (rtb_TmpSignalConversionAtSFun_0)) {
    rtb_TmpSignalConversionAtSFun_0 = 0.0;
  } else {
    rtb_TmpSignalConversionAtSFun_0 = std::fmod(rtb_TmpSignalConversionAtSFun_0,
      65536.0);
  }

  rtb_sincos_o2_idx_0_tmp = rt_roundd_snf(QuadModelv_P.VeScale_Gain *
    rtb_TmpSignalConversionAtSFun_1);
  if (rtIsNaN(rtb_sincos_o2_idx_0_tmp) || rtIsInf(rtb_sincos_o2_idx_0_tmp)) {
    rtb_sincos_o2_idx_0_tmp = 0.0;
  } else {
    rtb_sincos_o2_idx_0_tmp = std::fmod(rtb_sincos_o2_idx_0_tmp, 65536.0);
  }

  rtb_Rn = rt_roundd_snf((QuadModelv_P.TransferFcn2_C *
    QuadModelv_X.TransferFcn2_CSTATE + QuadModelv_B.Product[2]) *
    QuadModelv_P.VeScale_Gain);
  if (rtIsNaN(rtb_Rn) || rtIsInf(rtb_Rn)) {
    rtb_Rn = 0.0;
  } else {
    rtb_Rn = std::fmod(rtb_Rn, 65536.0);
  }

  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // DataTypeConversion: '<S330>/Data Type Conversion10' incorporates:
    //   Constant: '<S330>/ModelParam_GPSFix3DFix'

    rtb_Sum1_c = rt_roundd_snf(QuadModelv_P.ModelParam_GPSFix3DFix);
    if (rtIsNaN(rtb_Sum1_c) || rtIsInf(rtb_Sum1_c)) {
      rtb_Sum1_c = 0.0;
    } else {
      rtb_Sum1_c = std::fmod(rtb_Sum1_c, 256.0);
    }

    // DataTypeConversion: '<S330>/Data Type Conversion10'
    QuadModelv_B.fix_type = static_cast<uint8_T>(rtb_Sum1_c < 0.0 ?
      static_cast<int32_T>(static_cast<uint8_T>(-static_cast<int8_T>(
      static_cast<uint8_T>(-rtb_Sum1_c)))) : static_cast<int32_T>
      (static_cast<uint8_T>(rtb_Sum1_c)));

    // DataTypeConversion: '<S330>/Data Type Conversion11' incorporates:
    //   Constant: '<S330>/ModelParam.GPSSatsVisible'

    rtb_Sum1_c = rt_roundd_snf(QuadModelv_P.ModelParam_GPSSatsVisible);
    if (rtIsNaN(rtb_Sum1_c) || rtIsInf(rtb_Sum1_c)) {
      rtb_Sum1_c = 0.0;
    } else {
      rtb_Sum1_c = std::fmod(rtb_Sum1_c, 256.0);
    }

    // DataTypeConversion: '<S330>/Data Type Conversion11'
    QuadModelv_B.satellites_visible = static_cast<uint8_T>(rtb_Sum1_c < 0.0 ?
      static_cast<int32_T>(static_cast<uint8_T>(-static_cast<int8_T>(
      static_cast<uint8_T>(-rtb_Sum1_c)))) : static_cast<int32_T>(static_cast<
      uint8_T>(rtb_Sum1_c)));
  }

  // Outport: '<Root>/MavHILGPS' incorporates:
  //   BusCreator: '<S330>/Bus Creator'

  QuadModelv_Y.MavHILGPS.time_usec = rtb_time_usec;
  QuadModelv_Y.MavHILGPS.lat = r;
  QuadModelv_Y.MavHILGPS.lon = b_t;
  QuadModelv_Y.MavHILGPS.alt = rtb_alt;
  QuadModelv_Y.MavHILGPS.eph = QuadModelv_B.eph;
  QuadModelv_Y.MavHILGPS.epv = QuadModelv_B.epv;

  // DataTypeConversion: '<S330>/Data Type Conversion2' incorporates:
  //   Gain: '<S330>/VelScale'
  //   MATLAB Function: '<S330>/GenCogVel'

  rtb_Sum1_c = rt_roundd_snf(QuadModelv_P.VelScale_Gain * j);
  if (rtIsNaN(rtb_Sum1_c) || rtIsInf(rtb_Sum1_c)) {
    rtb_Sum1_c = 0.0;
  } else {
    rtb_Sum1_c = std::fmod(rtb_Sum1_c, 65536.0);
  }

  // Outport: '<Root>/MavHILGPS' incorporates:
  //   DataTypeConversion: '<S330>/Data Type Conversion2'
  //   DataTypeConversion: '<S330>/Data Type Conversion3'

  QuadModelv_Y.MavHILGPS.vel = static_cast<uint16_T>(rtb_Sum1_c < 0.0 ?
    static_cast<int32_T>(static_cast<uint16_T>(-static_cast<int16_T>(
    static_cast<uint16_T>(-rtb_Sum1_c)))) : static_cast<int32_T>
    (static_cast<uint16_T>(rtb_Sum1_c)));
  QuadModelv_Y.MavHILGPS.vn = static_cast<int16_T>
    (rtb_TmpSignalConversionAtSFun_0 < 0.0 ? static_cast<int32_T>(static_cast<
      int16_T>(-static_cast<int16_T>(static_cast<uint16_T>
        (-rtb_TmpSignalConversionAtSFun_0)))) : static_cast<int32_T>(
      static_cast<int16_T>(static_cast<uint16_T>(rtb_TmpSignalConversionAtSFun_0))));
  QuadModelv_Y.MavHILGPS.ve = static_cast<int16_T>(rtb_sincos_o2_idx_0_tmp < 0.0
    ? static_cast<int32_T>(static_cast<int16_T>(-static_cast<int16_T>(
    static_cast<uint16_T>(-rtb_sincos_o2_idx_0_tmp)))) : static_cast<int32_T>(
    static_cast<int16_T>(static_cast<uint16_T>(rtb_sincos_o2_idx_0_tmp))));
  QuadModelv_Y.MavHILGPS.vd = static_cast<int16_T>(rtb_Rn < 0.0 ?
    static_cast<int32_T>(static_cast<int16_T>(-static_cast<int16_T>
    (static_cast<uint16_T>(-rtb_Rn)))) : static_cast<int32_T>
    (static_cast<int16_T>(static_cast<uint16_T>(rtb_Rn))));

  // DataTypeConversion: '<S330>/Data Type Conversion7' incorporates:
  //   Gain: '<S330>/AngleScale'
  //   MATLAB Function: '<S330>/GenCogVel'

  rtb_TmpSignalConversionAtSFun_0 = rt_roundd_snf(QuadModelv_P.AngleScale_Gain *
    Z);
  if (rtIsNaN(rtb_TmpSignalConversionAtSFun_0) || rtIsInf
      (rtb_TmpSignalConversionAtSFun_0)) {
    rtb_TmpSignalConversionAtSFun_0 = 0.0;
  } else {
    rtb_TmpSignalConversionAtSFun_0 = std::fmod(rtb_TmpSignalConversionAtSFun_0,
      65536.0);
  }

  // Outport: '<Root>/MavHILGPS' incorporates:
  //   BusCreator: '<S330>/Bus Creator'
  //   DataTypeConversion: '<S330>/Data Type Conversion7'

  QuadModelv_Y.MavHILGPS.cog = static_cast<uint16_T>
    (rtb_TmpSignalConversionAtSFun_0 < 0.0 ? static_cast<int32_T>
     (static_cast<uint16_T>(-static_cast<int16_T>(static_cast<uint16_T>
        (-rtb_TmpSignalConversionAtSFun_0)))) : static_cast<int32_T>(
      static_cast<uint16_T>(rtb_TmpSignalConversionAtSFun_0)));
  QuadModelv_Y.MavHILGPS.fix_type = QuadModelv_B.fix_type;
  QuadModelv_Y.MavHILGPS.satellites_visible = QuadModelv_B.satellites_visible;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // DataTypeConversion: '<S330>/Data Type Conversion14' incorporates:
    //   Constant: '<S330>/ModelParam.GPSEphFinal'
    //   Gain: '<S330>/Gain1'

    rtb_TmpSignalConversionAtSFun_0 = rt_roundd_snf(QuadModelv_P.Gain1_Gain_c *
      QuadModelv_P.ModelParam_GPSEphFinal);
    if (rtIsNaN(rtb_TmpSignalConversionAtSFun_0) || rtIsInf
        (rtb_TmpSignalConversionAtSFun_0)) {
      rtb_TmpSignalConversionAtSFun_0 = 0.0;
    } else {
      rtb_TmpSignalConversionAtSFun_0 = std::fmod
        (rtb_TmpSignalConversionAtSFun_0, 4.294967296E+9);
    }

    // DataTypeConversion: '<S330>/Data Type Conversion14'
    QuadModelv_B.hAcc = rtb_TmpSignalConversionAtSFun_0 < 0.0 ?
      static_cast<uint32_T>(-static_cast<int32_T>(static_cast<uint32_T>
      (-rtb_TmpSignalConversionAtSFun_0))) : static_cast<uint32_T>
      (rtb_TmpSignalConversionAtSFun_0);

    // DataTypeConversion: '<S330>/Data Type Conversion15' incorporates:
    //   Constant: '<S330>/ModelParam.GPSEpvFinal'
    //   Gain: '<S330>/Gain3'

    rtb_TmpSignalConversionAtSFun_0 = rt_roundd_snf(QuadModelv_P.Gain3_Gain_l *
      QuadModelv_P.ModelParam_GPSEpvFinal);
    if (rtIsNaN(rtb_TmpSignalConversionAtSFun_0) || rtIsInf
        (rtb_TmpSignalConversionAtSFun_0)) {
      rtb_TmpSignalConversionAtSFun_0 = 0.0;
    } else {
      rtb_TmpSignalConversionAtSFun_0 = std::fmod
        (rtb_TmpSignalConversionAtSFun_0, 4.294967296E+9);
    }

    // DataTypeConversion: '<S330>/Data Type Conversion15'
    QuadModelv_B.vAcc = rtb_TmpSignalConversionAtSFun_0 < 0.0 ?
      static_cast<uint32_T>(-static_cast<int32_T>(static_cast<uint32_T>
      (-rtb_TmpSignalConversionAtSFun_0))) : static_cast<uint32_T>
      (rtb_TmpSignalConversionAtSFun_0);
  }

  // DataTypeConversion: '<S330>/Data Type Conversion1' incorporates:
  //   Gain: '<S330>/VeScale1'

  rtb_TmpSignalConversionAtSFun_0 = rt_roundd_snf(QuadModelv_P.VeScale1_Gain *
    QuadModelv_B.Product[0]);
  if (rtIsNaN(rtb_TmpSignalConversionAtSFun_0) || rtIsInf
      (rtb_TmpSignalConversionAtSFun_0)) {
    rtb_TmpSignalConversionAtSFun_0 = 0.0;
  } else {
    rtb_TmpSignalConversionAtSFun_0 = std::fmod(rtb_TmpSignalConversionAtSFun_0,
      4.294967296E+9);
  }

  rtb_sincos_o2_idx_0_tmp = rt_roundd_snf(QuadModelv_P.VeScale1_Gain *
    QuadModelv_B.Product[1]);
  if (rtIsNaN(rtb_sincos_o2_idx_0_tmp) || rtIsInf(rtb_sincos_o2_idx_0_tmp)) {
    rtb_sincos_o2_idx_0_tmp = 0.0;
  } else {
    rtb_sincos_o2_idx_0_tmp = std::fmod(rtb_sincos_o2_idx_0_tmp, 4.294967296E+9);
  }

  rtb_Rn = rt_roundd_snf(QuadModelv_P.VeScale1_Gain * QuadModelv_B.Product[2]);
  if (rtIsNaN(rtb_Rn) || rtIsInf(rtb_Rn)) {
    rtb_Rn = 0.0;
  } else {
    rtb_Rn = std::fmod(rtb_Rn, 4.294967296E+9);
  }

  // MATLAB Function: '<S330>/-pi-pi---->0-2pi'
  // //rad
  // MATLAB Function 'OutputPort1/HILGPSModel/-pi-pi---->0-2pi': '<S373>:1'
  // '<S373>:1:3' if yaw180>0
  if (VectorConcatenate_n_idx_0 > 0.0) {
    // '<S373>:1:4' yaw360 = yaw180;
    uavlostJzx = VectorConcatenate_n_idx_0;
  } else {
    // '<S373>:1:5' else
    // '<S373>:1:6' yaw360 = yaw180 +2*pi;
    uavlostJzx = VectorConcatenate_n_idx_0 + 6.2831853071795862;
  }

  // End of MATLAB Function: '<S330>/-pi-pi---->0-2pi'

  // Outport: '<Root>/PXIHILGPSData' incorporates:
  //   BusCreator: '<S330>/Bus Creator1'
  //   DataTypeConversion: '<S330>/Data Type Conversion1'

  QuadModelv_Y.PXIHILGPSData.fix_type = QuadModelv_B.fix_type;
  QuadModelv_Y.PXIHILGPSData.satellites_visible =
    QuadModelv_B.satellites_visible;
  QuadModelv_Y.PXIHILGPSData.lat = r;
  QuadModelv_Y.PXIHILGPSData.lon = b_t;
  QuadModelv_Y.PXIHILGPSData.alt = rtb_alt;
  QuadModelv_Y.PXIHILGPSData.hAcc = QuadModelv_B.hAcc;
  QuadModelv_Y.PXIHILGPSData.vAcc = QuadModelv_B.vAcc;
  QuadModelv_Y.PXIHILGPSData.velN = rtb_TmpSignalConversionAtSFun_0 < 0.0 ? -
    static_cast<int32_T>(static_cast<uint32_T>(-rtb_TmpSignalConversionAtSFun_0))
    : static_cast<int32_T>(static_cast<uint32_T>(rtb_TmpSignalConversionAtSFun_0));
  QuadModelv_Y.PXIHILGPSData.velE = rtb_sincos_o2_idx_0_tmp < 0.0 ? -
    static_cast<int32_T>(static_cast<uint32_T>(-rtb_sincos_o2_idx_0_tmp)) :
    static_cast<int32_T>(static_cast<uint32_T>(rtb_sincos_o2_idx_0_tmp));
  QuadModelv_Y.PXIHILGPSData.velD = rtb_Rn < 0.0 ? -static_cast<int32_T>(
    static_cast<uint32_T>(-rtb_Rn)) : static_cast<int32_T>(static_cast<uint32_T>
    (rtb_Rn));

  // DataTypeConversion: '<S330>/Data Type Conversion12' incorporates:
  //   Gain: '<S330>/VelScale1'
  //   MATLAB Function: '<S330>/GenCogVel'

  rtb_TmpSignalConversionAtSFun_0 = rt_roundd_snf(QuadModelv_P.VelScale1_Gain *
    j);
  if (rtIsNaN(rtb_TmpSignalConversionAtSFun_0) || rtIsInf
      (rtb_TmpSignalConversionAtSFun_0)) {
    rtb_TmpSignalConversionAtSFun_0 = 0.0;
  } else {
    rtb_TmpSignalConversionAtSFun_0 = std::fmod(rtb_TmpSignalConversionAtSFun_0,
      4.294967296E+9);
  }

  // Outport: '<Root>/PXIHILGPSData' incorporates:
  //   DataTypeConversion: '<S330>/Data Type Conversion12'

  QuadModelv_Y.PXIHILGPSData.gSpeed = rtb_TmpSignalConversionAtSFun_0 < 0.0 ? -
    static_cast<int32_T>(static_cast<uint32_T>(-rtb_TmpSignalConversionAtSFun_0))
    : static_cast<int32_T>(static_cast<uint32_T>(rtb_TmpSignalConversionAtSFun_0));

  // DataTypeConversion: '<S330>/Data Type Conversion13' incorporates:
  //   Gain: '<S330>/headMotAngle'
  //   MATLAB Function: '<S330>/GenCogVel'

  rtb_TmpSignalConversionAtSFun_0 = rt_roundd_snf(QuadModelv_P.headMotAngle_Gain
    * Z);
  if (rtIsNaN(rtb_TmpSignalConversionAtSFun_0) || rtIsInf
      (rtb_TmpSignalConversionAtSFun_0)) {
    rtb_TmpSignalConversionAtSFun_0 = 0.0;
  } else {
    rtb_TmpSignalConversionAtSFun_0 = std::fmod(rtb_TmpSignalConversionAtSFun_0,
      4.294967296E+9);
  }

  // Outport: '<Root>/PXIHILGPSData' incorporates:
  //   DataTypeConversion: '<S330>/Data Type Conversion13'

  QuadModelv_Y.PXIHILGPSData.headMot = rtb_TmpSignalConversionAtSFun_0 < 0.0 ? -
    static_cast<int32_T>(static_cast<uint32_T>(-rtb_TmpSignalConversionAtSFun_0))
    : static_cast<int32_T>(static_cast<uint32_T>(rtb_TmpSignalConversionAtSFun_0));

  // DataTypeConversion: '<S330>/Data Type Conversion16' incorporates:
  //   Gain: '<S330>/headVehAngleScale'

  rtb_TmpSignalConversionAtSFun_0 = rt_roundd_snf
    (QuadModelv_P.headVehAngleScale_Gain * uavlostJzx);
  if (rtIsNaN(rtb_TmpSignalConversionAtSFun_0) || rtIsInf
      (rtb_TmpSignalConversionAtSFun_0)) {
    rtb_TmpSignalConversionAtSFun_0 = 0.0;
  } else {
    rtb_TmpSignalConversionAtSFun_0 = std::fmod(rtb_TmpSignalConversionAtSFun_0,
      4.294967296E+9);
  }

  // Outport: '<Root>/PXIHILGPSData' incorporates:
  //   DataTypeConversion: '<S330>/Data Type Conversion16'

  QuadModelv_Y.PXIHILGPSData.headVeh = rtb_TmpSignalConversionAtSFun_0 < 0.0 ? -
    static_cast<int32_T>(static_cast<uint32_T>(-rtb_TmpSignalConversionAtSFun_0))
    : static_cast<int32_T>(static_cast<uint32_T>(rtb_TmpSignalConversionAtSFun_0));

  // DataTypeConversion: '<S407>/Data Type Conversion1' incorporates:
  //   Gain: '<S407>/magDecGain'

  rtb_TmpSignalConversionAtSFun_0 = std::floor(QuadModelv_P.magDecGain_Gain *
    rtb_Product3_o);
  if (rtIsNaN(rtb_TmpSignalConversionAtSFun_0) || rtIsInf
      (rtb_TmpSignalConversionAtSFun_0)) {
    rtb_TmpSignalConversionAtSFun_0 = 0.0;
  } else {
    rtb_TmpSignalConversionAtSFun_0 = std::fmod(rtb_TmpSignalConversionAtSFun_0,
      65536.0);
  }

  // Outport: '<Root>/magDec' incorporates:
  //   DataTypeConversion: '<S407>/Data Type Conversion1'

  QuadModelv_Y.magDec = static_cast<int16_T>(rtb_TmpSignalConversionAtSFun_0 <
    0.0 ? static_cast<int32_T>(static_cast<int16_T>(-static_cast<int16_T>(
    static_cast<uint16_T>(-rtb_TmpSignalConversionAtSFun_0)))) :
    static_cast<int32_T>(static_cast<int16_T>(static_cast<uint16_T>
    (rtb_TmpSignalConversionAtSFun_0))));

  // Outport: '<Root>/SysTime'
  QuadModelv_Y.SysTime = rtb_time_usec;

  // Outport: '<Root>/VelE'
  QuadModelv_Y.VelE[0] = QuadModelv_B.Product[0];

  // Outport: '<Root>/PosE' incorporates:
  //   Integrator: '<S14>/xe,ye,ze'

  QuadModelv_Y.PosE[0] = QuadModelv_X.xeyeze_CSTATE[0];

  // Outport: '<Root>/VelE'
  QuadModelv_Y.VelE[1] = QuadModelv_B.Product[1];

  // Outport: '<Root>/PosE' incorporates:
  //   Integrator: '<S14>/xe,ye,ze'

  QuadModelv_Y.PosE[1] = QuadModelv_X.xeyeze_CSTATE[1];

  // Outport: '<Root>/VelE'
  QuadModelv_Y.VelE[2] = QuadModelv_B.Product[2];

  // Outport: '<Root>/PosE'
  QuadModelv_Y.PosE[2] = rtb_UniformRandomNumber7_idx_2;

  // Outport: '<Root>/AngEuler'
  QuadModelv_Y.AngEuler[0] = VectorConcatenate_n_idx_2;
  QuadModelv_Y.AngEuler[1] = VectorConcatenate_n_idx_1;
  QuadModelv_Y.AngEuler[2] = VectorConcatenate_n_idx_0;

  // Outport: '<Root>/AngQuatern'
  QuadModelv_Y.AngQuatern[0] = QuadModelv_B.Merge_f[0];
  QuadModelv_Y.AngQuatern[1] = QuadModelv_B.Merge_f[1];
  QuadModelv_Y.AngQuatern[2] = QuadModelv_B.Merge_f[2];
  QuadModelv_Y.AngQuatern[3] = QuadModelv_B.Merge_f[3];

  // Outport: '<Root>/RateB' incorporates:
  //   Integrator: '<S14>/p,q,r '

  QuadModelv_Y.RateB[0] = QuadModelv_X.pqr_CSTATE[0];
  QuadModelv_Y.RateB[1] = QuadModelv_X.pqr_CSTATE[1];
  QuadModelv_Y.RateB[2] = QuadModelv_X.pqr_CSTATE[2];
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Outputs for Iterator SubSystem: '<S53>/For Each Subsystem' incorporates:
    //   ForEach: '<S54>/For Each'

    for (ForEach_itr = 0; ForEach_itr < 1; ForEach_itr++) {
      // ForEachSliceAssignment generated from: '<S54>/F' incorporates:
      //   ForEachSliceSelector generated from: '<S54>/mdot'
      //   Product: '<S54>/Product'
      //   SignalConversion generated from: '<S53>/For Each Subsystem'

      rtb_Merge[0] = 0.0;
      rtb_Merge[1] = 0.0;
      rtb_Merge[2] = 0.0;
    }

    // End of Outputs for SubSystem: '<S53>/For Each Subsystem'

    // Sum: '<S53>/Sum of Elements'
    QuadModelv_B.SumofElements[0] = 0.0;
    QuadModelv_B.SumofElements[1] = 0.0;
    QuadModelv_B.SumofElements[2] = 0.0;
  }

  // Product: '<S56>/j x i' incorporates:
  //   Integrator: '<S14>/p,q,r '
  //   Integrator: '<S14>/ub,vb,wb'

  j = QuadModelv_X.pqr_CSTATE[0] * QuadModelv_X.ubvbwb_CSTATE[1];

  // Sum: '<S14>/Sum' incorporates:
  //   DataTypeConversion: '<S8>/Data Type Conversion2'
  //   Integrator: '<S14>/p,q,r '
  //   Integrator: '<S14>/ub,vb,wb'
  //   Product: '<S14>/Product'
  //   Product: '<S55>/i x j'
  //   Product: '<S55>/j x k'
  //   Product: '<S55>/k x i'
  //   Product: '<S56>/i x k'
  //   Product: '<S56>/k x j'
  //   Sum: '<S17>/Sum'
  //   Sum: '<S18>/Sum'
  //   Sum: '<S1>/Sum1'

  QuadModelv_B.Sum_p[0] = ((QuadModelv_B.fm[0] + rtb_DataTypeConversion2[0]) +
    QuadModelv_B.SumofElements[0]) / QuadModelv_B.m +
    (QuadModelv_X.ubvbwb_CSTATE[1] * QuadModelv_X.pqr_CSTATE[2] -
     QuadModelv_X.pqr_CSTATE[1] * QuadModelv_X.ubvbwb_CSTATE[2]);
  QuadModelv_B.Sum_p[1] = ((QuadModelv_B.fm[1] + rtb_DataTypeConversion2[1]) +
    QuadModelv_B.SumofElements[1]) / QuadModelv_B.m + (QuadModelv_X.pqr_CSTATE[0]
    * QuadModelv_X.ubvbwb_CSTATE[2] - QuadModelv_X.ubvbwb_CSTATE[0] *
    QuadModelv_X.pqr_CSTATE[2]);
  QuadModelv_B.Sum_p[2] = ((QuadModelv_B.fm[2] + rtb_Add) +
    QuadModelv_B.SumofElements[2]) / QuadModelv_B.m +
    (QuadModelv_X.ubvbwb_CSTATE[0] * QuadModelv_X.pqr_CSTATE[1] - j);

  // Outport: '<Root>/AccB'
  QuadModelv_Y.AccB[0] = QuadModelv_B.Sum_p[0];
  QuadModelv_Y.AccB[1] = QuadModelv_B.Sum_p[1];
  QuadModelv_Y.AccB[2] = QuadModelv_B.Sum_p[2];
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // MATLAB Function: '<S12>/Model Fail Assessment' incorporates:
    //   Constant: '<S12>/ModelFailEnable'
    //   Integrator: '<S14>/p,q,r '
    //   Integrator: '<S14>/xe,ye,ze'
    //   SignalConversion generated from: '<S333>/ SFunction '

    // MATLAB Function 'OutputPort1/Model Fail Assessment': '<S333>:1'
    // '<S333>:1:4' if isempty(xTemp)
    // '<S333>:1:9' if isempty(kk)
    // '<S333>:1:14' if isempty(ss)
    // '<S333>:1:18' if ~ModelFailEnable&&(kk<0.5)
    if ((!(QuadModelv_P.ModelFailEnable != 0.0)) && (QuadModelv_DW.kk < 0.5)) {
      // '<S333>:1:19' xTemp=zeros(8,1);
      std::memset(&QuadModelv_DW.xTemp[0], 0, sizeof(real_T) << 3U);

      // '<S333>:1:20' kk=0;
      QuadModelv_DW.kk = 0.0;

      // '<S333>:1:21' ss=[xe;ang];
      QuadModelv_DW.ss[0] = QuadModelv_X.xeyeze_CSTATE[0];
      QuadModelv_DW.ss[3] = VectorConcatenate_n_idx_2;
      QuadModelv_DW.ss[1] = QuadModelv_X.xeyeze_CSTATE[1];
      QuadModelv_DW.ss[4] = VectorConcatenate_n_idx_1;
      QuadModelv_DW.ss[2] = rtb_UniformRandomNumber7_idx_2;
      QuadModelv_DW.ss[5] = VectorConcatenate_n_idx_0;
    }

    // '<S333>:1:24' if ModelFailEnable&&~landed
    if ((QuadModelv_P.ModelFailEnable != 0.0) && (rtb_landed == 0)) {
      // '<S333>:1:25' xTemp(1:4,1)=[norm(xe-ss(1:3,1));norm(ve);norm(ang-ss(4:6,1));norm(wb)]; 
      rtb_DataTypeConversion2[0] = QuadModelv_X.xeyeze_CSTATE[0] -
        QuadModelv_DW.ss[0];
      rtb_Merge[0] = VectorConcatenate_n_idx_2 - QuadModelv_DW.ss[3];
      rtb_DataTypeConversion2[1] = QuadModelv_X.xeyeze_CSTATE[1] -
        QuadModelv_DW.ss[1];
      rtb_Merge[1] = VectorConcatenate_n_idx_1 - QuadModelv_DW.ss[4];
      rtb_DataTypeConversion2[2] = rtb_UniformRandomNumber7_idx_2 -
        QuadModelv_DW.ss[2];
      rtb_Merge[2] = VectorConcatenate_n_idx_0 - QuadModelv_DW.ss[5];
      QuadModelv_DW.xTemp[0] = QuadModelv_norm(rtb_DataTypeConversion2);
      QuadModelv_DW.xTemp[1] = QuadModelv_norm(QuadModelv_B.Product);
      QuadModelv_DW.xTemp[2] = QuadModelv_norm(rtb_Merge);
      QuadModelv_DW.xTemp[3] = QuadModelv_norm(QuadModelv_X.pqr_CSTATE);

      // '<S333>:1:26' xTemp(5:8,1)= (xTemp(5:8,1)*kk+xTemp(1:4,1))/(kk+1);
      QuadModelv_DW.xTemp[4] = (QuadModelv_DW.xTemp[4] * QuadModelv_DW.kk +
        QuadModelv_DW.xTemp[0]) / (QuadModelv_DW.kk + 1.0);
      QuadModelv_DW.xTemp[5] = (QuadModelv_DW.xTemp[5] * QuadModelv_DW.kk +
        QuadModelv_DW.xTemp[1]) / (QuadModelv_DW.kk + 1.0);
      QuadModelv_DW.xTemp[6] = (QuadModelv_DW.xTemp[6] * QuadModelv_DW.kk +
        QuadModelv_DW.xTemp[2]) / (QuadModelv_DW.kk + 1.0);
      QuadModelv_DW.xTemp[7] = (QuadModelv_DW.xTemp[7] * QuadModelv_DW.kk +
        QuadModelv_DW.xTemp[3]) / (QuadModelv_DW.kk + 1.0);

      // '<S333>:1:28' kk=kk+1;
      QuadModelv_DW.kk++;
    }

    // Outport: '<Root>/FailInfo' incorporates:
    //   MATLAB Function: '<S12>/Model Fail Assessment'

    // landed = int8(0);
    // '<S333>:1:32' FailInfo=xTemp;
    std::memcpy(&QuadModelv_Y.FailInfo[0], &QuadModelv_DW.xTemp[0], sizeof
                (real_T) << 3U);

    // Outport: '<Root>/landed'
    QuadModelv_Y.landed = rtb_landed;

    // Gain: '<S12>/Gain'
    for (i = 0; i < 8; i++) {
      rtb_Gain_ok[i] = QuadModelv_P.Gain_Gain_ja * QuadModelv_B.y[i];
    }

    // End of Gain: '<S12>/Gain'

    // Outport: '<Root>/MotorRPMS'
    std::memcpy(&QuadModelv_Y.MotorRPMS[0], &rtb_Gain_ok[0], sizeof(real_T) <<
                3U);
  }

  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
    // Outport: '<Root>/BattCellVoltage'
    QuadModelv_Y.BattCellVoltage = QuadModelv_B.cell_voltage;

    // Outport: '<Root>/BattRemCapacity'
    QuadModelv_Y.BattRemCapacity = QuadModelv_B.remCapacity;
  }

  // Outport: '<Root>/wind'
  QuadModelv_Y.wind[0] = rtb_sincos_o1_f[0];
  QuadModelv_Y.wind[1] = rtb_sincos_o1_f[1];
  QuadModelv_Y.wind[2] = rtb_sincos_o1_f[2];
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // DataTypeConversion: '<S12>/Data Type Conversion' incorporates:
    //   Constant: '<S12>/CopterID'

    rtb_TmpSignalConversionAtSFun_0 = std::floor(QuadModelv_P.CopterID_Value);
    if (rtIsNaN(rtb_TmpSignalConversionAtSFun_0) || rtIsInf
        (rtb_TmpSignalConversionAtSFun_0)) {
      rtb_TmpSignalConversionAtSFun_0 = 0.0;
    } else {
      rtb_TmpSignalConversionAtSFun_0 = std::fmod
        (rtb_TmpSignalConversionAtSFun_0, 4.294967296E+9);
    }

    // DataTypeConversion: '<S12>/Data Type Conversion'
    QuadModelv_B.copterID = rtb_TmpSignalConversionAtSFun_0 < 0.0 ? -
      static_cast<int32_T>(static_cast<uint32_T>
      (-rtb_TmpSignalConversionAtSFun_0)) : static_cast<int32_T>
      (static_cast<uint32_T>(rtb_TmpSignalConversionAtSFun_0));

    // DataTypeConversion: '<S12>/Data Type Conversion2' incorporates:
    //   Constant: '<S12>/UAVType'

    QuadModelv_B.vehicleType = QuadModelv_P.ModelParam_uavType;
    for (i = 0; i < 8; i++) {
      // DataTypeConversion: '<S12>/Data Type Conversion8'
      QuadModelv_B.MotorRPMS[i] = static_cast<real32_T>(rtb_Gain_ok[i]);
    }
  }

  // Outport: '<Root>/MavVehile3DInfo' incorporates:
  //   BusCreator generated from: '<Root>/MavVehile3DInfo'
  //   Clock: '<S12>/Clock1'
  //   DataTypeConversion: '<S12>/Data Type Conversion4'
  //   DataTypeConversion: '<S12>/Data Type Conversion5'
  //   DataTypeConversion: '<S12>/Data Type Conversion6'
  //   DataTypeConversion: '<S12>/Data Type Conversion7'
  //   DataTypeConversion: '<S12>/Data Type Conversion9'
  //   DataTypeConversion: '<S332>/Data Type Conversion3'
  //   Integrator: '<S14>/p,q,r '
  //   Integrator: '<S14>/xe,ye,ze'

  QuadModelv_Y.MavVehile3DInfo.copterID = QuadModelv_B.copterID;
  QuadModelv_Y.MavVehile3DInfo.vehicleType = QuadModelv_B.vehicleType;
  QuadModelv_Y.MavVehile3DInfo.runnedTime = FaultNum;
  QuadModelv_Y.MavVehile3DInfo.VelE[0] = static_cast<real32_T>
    (QuadModelv_B.Product[0]);
  QuadModelv_Y.MavVehile3DInfo.PosE[0] = static_cast<real32_T>
    (QuadModelv_X.xeyeze_CSTATE[0]);
  QuadModelv_Y.MavVehile3DInfo.VelE[1] = static_cast<real32_T>
    (QuadModelv_B.Product[1]);
  QuadModelv_Y.MavVehile3DInfo.PosE[1] = static_cast<real32_T>
    (QuadModelv_X.xeyeze_CSTATE[1]);
  QuadModelv_Y.MavVehile3DInfo.VelE[2] = static_cast<real32_T>
    (QuadModelv_B.Product[2]);
  QuadModelv_Y.MavVehile3DInfo.PosE[2] = static_cast<real32_T>
    (rtb_UniformRandomNumber7_idx_2);
  QuadModelv_Y.MavVehile3DInfo.AngEuler[0] = static_cast<real32_T>
    (VectorConcatenate_n_idx_2);
  QuadModelv_Y.MavVehile3DInfo.AngEuler[1] = static_cast<real32_T>
    (VectorConcatenate_n_idx_1);
  QuadModelv_Y.MavVehile3DInfo.AngEuler[2] = static_cast<real32_T>
    (VectorConcatenate_n_idx_0);
  QuadModelv_Y.MavVehile3DInfo.AngQuatern[0] = static_cast<real32_T>
    (QuadModelv_B.Merge_f[0]);
  QuadModelv_Y.MavVehile3DInfo.AngQuatern[1] = static_cast<real32_T>
    (QuadModelv_B.Merge_f[1]);
  QuadModelv_Y.MavVehile3DInfo.AngQuatern[2] = static_cast<real32_T>
    (QuadModelv_B.Merge_f[2]);
  QuadModelv_Y.MavVehile3DInfo.AngQuatern[3] = static_cast<real32_T>
    (QuadModelv_B.Merge_f[3]);
  QuadModelv_Y.MavVehile3DInfo.AccB[0] = static_cast<real32_T>
    (QuadModelv_B.Sum_p[0]);
  QuadModelv_Y.MavVehile3DInfo.RateB[0] = static_cast<real32_T>
    (QuadModelv_X.pqr_CSTATE[0]);
  QuadModelv_Y.MavVehile3DInfo.PosGPS[0] = LoadLeakRatio;

  // DataTypeConversion: '<S8>/Data Type Conversion1' incorporates:
  //   DataTypeConversion: '<S332>/Data Type Conversion3'
  //   Integrator: '<S14>/p,q,r '

  rtb_DataTypeConversion1[0] = static_cast<real32_T>(QuadModelv_X.pqr_CSTATE[0]);

  // Outport: '<Root>/MavVehile3DInfo' incorporates:
  //   DataTypeConversion: '<S12>/Data Type Conversion9'
  //   DataTypeConversion: '<S332>/Data Type Conversion3'
  //   Integrator: '<S14>/p,q,r '
  //   SignalConversion generated from: '<Root>/MavVehile3DInfo'

  QuadModelv_Y.MavVehile3DInfo.AccB[1] = static_cast<real32_T>
    (QuadModelv_B.Sum_p[1]);
  QuadModelv_Y.MavVehile3DInfo.RateB[1] = static_cast<real32_T>
    (QuadModelv_X.pqr_CSTATE[1]);
  QuadModelv_Y.MavVehile3DInfo.PosGPS[1] = Y;

  // DataTypeConversion: '<S8>/Data Type Conversion1' incorporates:
  //   DataTypeConversion: '<S332>/Data Type Conversion3'
  //   Integrator: '<S14>/p,q,r '

  rtb_DataTypeConversion1[1] = static_cast<real32_T>(QuadModelv_X.pqr_CSTATE[1]);

  // Outport: '<Root>/MavVehile3DInfo' incorporates:
  //   DataTypeConversion: '<S12>/Data Type Conversion9'
  //   DataTypeConversion: '<S332>/Data Type Conversion3'
  //   Integrator: '<S14>/p,q,r '
  //   SignalConversion generated from: '<Root>/MavVehile3DInfo'

  QuadModelv_Y.MavVehile3DInfo.AccB[2] = static_cast<real32_T>
    (QuadModelv_B.Sum_p[2]);
  QuadModelv_Y.MavVehile3DInfo.RateB[2] = static_cast<real32_T>
    (QuadModelv_X.pqr_CSTATE[2]);
  QuadModelv_Y.MavVehile3DInfo.PosGPS[2] = rtb_Saturation_1;

  // DataTypeConversion: '<S8>/Data Type Conversion1' incorporates:
  //   DataTypeConversion: '<S332>/Data Type Conversion3'
  //   Integrator: '<S14>/p,q,r '

  rtb_DataTypeConversion1[2] = static_cast<real32_T>(QuadModelv_X.pqr_CSTATE[2]);

  // MATLAB Function 'LogSelectModel/MATLAB Function1': '<S294>:1'
  // '<S294>:1:2' FaultID = zeros(8,1);
  for (i = 0; i < 8; i++) {
    // Outport: '<Root>/MavVehile3DInfo' incorporates:
    //   BusCreator generated from: '<Root>/MavVehile3DInfo'

    QuadModelv_Y.MavVehile3DInfo.MotorRPMS[i] = QuadModelv_B.MotorRPMS[i];

    // MATLAB Function: '<S8>/MATLAB Function1'
    Memory[i] = 0.0;
  }

  // MATLAB Function: '<S8>/MATLAB Function1' incorporates:
  //   Inport: '<Root>/inSILInts'

  // '<S294>:1:3' j = 1;
  j = 1.0;

  // '<S294>:1:4' for i=1:8
  for (b_t = 0; b_t < 8; b_t++) {
    // '<S294>:1:5' if inInts(i)~= 0
    if (QuadModelv_U.inSILInts[b_t] != 0) {
      // '<S294>:1:6' FaultID(j) = inInts(i);
      Memory[static_cast<int32_T>(j) - 1] = QuadModelv_U.inSILInts[b_t];

      // '<S294>:1:7' j = j + 1;
      j++;
    }
  }

  //  去除相同的ID
  // '<S294>:1:12' for i = 1:7
  for (r = 0; r < 7; r++) {
    // '<S294>:1:13' for j = i+1:8
    for (b_t = 0; b_t <= 6 - r; b_t++) {
      j = static_cast<real_T>(r + b_t) + 2.0;

      // '<S294>:1:14' if FaultID(i) == FaultID(j)
      i = static_cast<int32_T>(j) - 1;
      if (Memory[i] == Memory[r]) {
        // '<S294>:1:15' FaultID(j) = 0;
        Memory[i] = 0.0;
      }
    }
  }

  //
  // '<S294>:1:21' for i = 1:8
  for (r = 0; r < 8; r++) {
    // '<S294>:1:22' if FaultID(i) == 0
    if (Memory[r] == 0.0) {
      boolean_T exitg1;

      // '<S294>:1:23' for j = i+1:8
      b_t = 0;
      exitg1 = false;
      while ((!exitg1) && (b_t <= 6 - r)) {
        j = static_cast<real_T>(r + b_t) + 2.0;

        // '<S294>:1:24' if FaultID(j)~=0
        i = static_cast<int32_T>(j) - 1;
        rtb_TmpSignalConversionAtSFun_0 = Memory[i];
        if (rtb_TmpSignalConversionAtSFun_0 != 0.0) {
          // '<S294>:1:25' FaultID(i) =  FaultID(j);
          Memory[r] = rtb_TmpSignalConversionAtSFun_0;

          // '<S294>:1:26' FaultID(j) = 0;
          Memory[i] = 0.0;
          exitg1 = true;
        } else {
          b_t++;
        }
      }
    }
  }

  // SignalConversion generated from: '<S293>/ SFunction ' incorporates:
  //   MATLAB Function: '<S8>/MATLAB Function'

  // '<S294>:1:33' y = FaultID;
  tz[0] = QuadModelv_B.uavlostMass;
  tz[1] = QuadModelv_B.deltaM[0];

  // SignalConversion generated from: '<S293>/ SFunction ' incorporates:
  //   MATLAB Function: '<S8>/MATLAB Function'

  ddm[0] = rtb_sincos_o1_f[0];
  ddm[3] = rtb_Windb[0];

  // SignalConversion generated from: '<S293>/ SFunction ' incorporates:
  //   MATLAB Function: '<S8>/MATLAB Function'

  tz[2] = QuadModelv_B.deltaM[1];

  // SignalConversion generated from: '<S293>/ SFunction ' incorporates:
  //   MATLAB Function: '<S8>/MATLAB Function'

  ddm[1] = rtb_sincos_o1_f[1];
  ddm[4] = rtb_Windb[1];

  // SignalConversion generated from: '<S293>/ SFunction ' incorporates:
  //   MATLAB Function: '<S8>/MATLAB Function'

  tz[3] = QuadModelv_B.deltaM[2];

  // SignalConversion generated from: '<S293>/ SFunction ' incorporates:
  //   MATLAB Function: '<S8>/MATLAB Function'

  ddm[2] = rtb_sincos_o1_f[2];
  ddm[5] = rtb_Windb[2];

  // MATLAB Function: '<S8>/MATLAB Function' incorporates:
  //   SignalConversion generated from: '<S293>/ SFunction '
  //
  // MATLAB Function 'LogSelectModel/MATLAB Function': '<S293>:1'
  // '<S293>:1:3' y = zeros(23,1);
  std::memset(&rtb_y_i[0], 0, 23U * sizeof(real_T));

  // '<S293>:1:5' FaultNum = 0;
  FaultNum = 0.0;

  // '<S293>:1:6' for i = 1:8
  for (b_t = 0; b_t < 8; b_t++) {
    // '<S293>:1:7' if FaultIDs(i)~=0
    if (Memory[b_t] != 0.0) {
      // '<S293>:1:8' FaultNum = FaultNum + 1;
      FaultNum++;
    }
  }

  // '<S293>:1:13' if FaultNum == 1
  if (FaultNum == 1.0) {
    // '<S293>:1:14' if FaultIDs(1) == 123450|| FaultIDs(1) == 123549
    if ((Memory[0] == 123450.0) || (Memory[0] == 123549.0)) {
      // '<S293>:1:15' for i = 1:8
      // '<S293>:1:16' y(i) = D123450(i);
      std::memcpy(&rtb_y_i[0], &QuadModelv_B.y_d[0], sizeof(real_T) << 3U);
    } else if (Memory[0] == 123451.0) {
      // '<S293>:1:18' elseif FaultIDs(1) == 123451
      // '<S293>:1:19' for i = 1:8
      // '<S293>:1:20' y(i) = D123451(i);
      std::memcpy(&rtb_y_i[0], &QuadModelv_B.y[0], sizeof(real_T) << 3U);
    } else if ((Memory[0] == 123452.0) || (Memory[0] == 123453.0) || (Memory[0] ==
                123454.0) || (Memory[0] == 123455.0)) {
      // '<S293>:1:22' elseif FaultIDs(1) == 123452 || FaultIDs(1) == 123453 || FaultIDs(1) == 123454 || FaultIDs(1) == 123455 
      // '<S293>:1:23' for i = 1:2
      // '<S293>:1:24' y(i) = D123452(i);
      rtb_y_i[0] = QuadModelv_B.cell_voltage;

      // '<S293>:1:24' y(i) = D123452(i);
      rtb_y_i[1] = QuadModelv_B.remCapacity;
    } else if ((Memory[0] == 123456.0) || (Memory[0] == 123457.0) || (Memory[0] ==
                123458.0)) {
      // '<S293>:1:26' elseif FaultIDs(1) == 123456 || FaultIDs(1) == 123457 || FaultIDs(1) == 123458 
      // '<S293>:1:27' for i = 1:4
      // '<S293>:1:28' y(i) = D123456(i);
      rtb_y_i[0] = QuadModelv_B.uavlostMass;

      // '<S293>:1:28' y(i) = D123456(i);
      rtb_y_i[1] = QuadModelv_B.deltaM[0];

      // '<S293>:1:28' y(i) = D123456(i);
      rtb_y_i[2] = QuadModelv_B.deltaM[1];

      // '<S293>:1:28' y(i) = D123456(i);
      rtb_y_i[3] = QuadModelv_B.deltaM[2];
    } else if ((Memory[0] == 123459.0) || (Memory[0] == 123540.0) || (Memory[0] ==
                123541.0) || (Memory[0] == 123542.0) || (Memory[0] == 123543.0))
    {
      // '<S293>:1:30' elseif FaultIDs(1) == 123459 || FaultIDs(1) == 123540 || FaultIDs(1) == 123541 || FaultIDs(1) == 123542 || FaultIDs(1) == 123543 
      // '<S293>:1:31' for i = 1:6
      for (r = 0; r < 6; r++) {
        // '<S293>:1:32' y(i) = D123459(i);
        rtb_y_i[r] = ddm[r];
      }
    } else if (Memory[0] == 123544.0) {
      // '<S293>:1:34' elseif FaultIDs(1) == 123544
      // '<S293>:1:35' for i = 1:3
      // '<S293>:1:36' y(i) = D123544(i);
      rtb_y_i[0] = QuadModelv_B.sf_AccNoiseFun_i.y[0];

      // '<S293>:1:36' y(i) = D123544(i);
      rtb_y_i[1] = QuadModelv_B.sf_AccNoiseFun_i.y[1];

      // '<S293>:1:36' y(i) = D123544(i);
      rtb_y_i[2] = QuadModelv_B.sf_AccNoiseFun_i.y[2];
    } else if (Memory[0] == 123545.0) {
      // '<S293>:1:38' elseif FaultIDs(1) == 123545
      // '<S293>:1:39' for i = 1:3
      // '<S293>:1:40' y(i) = D123545(i);
      rtb_y_i[0] = QuadModelv_B.sf_AccNoiseFun_a.y[0];

      // '<S293>:1:40' y(i) = D123545(i);
      rtb_y_i[1] = QuadModelv_B.sf_AccNoiseFun_a.y[1];

      // '<S293>:1:40' y(i) = D123545(i);
      rtb_y_i[2] = QuadModelv_B.sf_AccNoiseFun_a.y[2];
    } else if (Memory[0] == 123546.0) {
      // '<S293>:1:42' elseif FaultIDs(1) == 123546
      // '<S293>:1:43' for i = 1:3
      // '<S293>:1:44' y(i) = D123546(i);
      rtb_y_i[0] = QuadModelv_B.sf_AccNoiseFun_p.y[0];

      // '<S293>:1:44' y(i) = D123546(i);
      rtb_y_i[1] = QuadModelv_B.sf_AccNoiseFun_p.y[1];

      // '<S293>:1:44' y(i) = D123546(i);
      rtb_y_i[2] = QuadModelv_B.sf_AccNoiseFun_p.y[2];
    } else if (Memory[0] == 123547.0) {
      // '<S293>:1:46' elseif FaultIDs(1) == 123547
      // '<S293>:1:47' y(1) = D123547(1);
      rtb_y_i[0] = rtb_fcn3;
    } else if (Memory[0] == 123548.0) {
      // '<S293>:1:48' elseif FaultIDs(1) == 123548
      // '<S293>:1:49' for i = 1:3
      // '<S293>:1:50' y(i) = D123548(i);
      rtb_y_i[0] = QuadModelv_B.y_m[0];

      // '<S293>:1:50' y(i) = D123548(i);
      rtb_y_i[1] = QuadModelv_B.y_m[1];

      // '<S293>:1:50' y(i) = D123548(i);
      rtb_y_i[2] = QuadModelv_B.y_m[2];
    }
  }

  // '<S293>:1:55' if FaultNum == 2
  if (FaultNum == 2.0) {
    // '<S293>:1:56' if FaultIDs(1) == 123450|| FaultIDs(1) == 123549
    if ((Memory[0] == 123450.0) || (Memory[0] == 123549.0)) {
      // '<S293>:1:57' for i = 1:8
      // '<S293>:1:58' y(i) = D123450(i);
      std::memcpy(&rtb_y_i[0], &QuadModelv_B.y_d[0], sizeof(real_T) << 3U);

      // '<S293>:1:60' if FaultIDs(2) == 123451
      if (Memory[1] == 123451.0) {
        // '<S293>:1:61' j = 1;
        // '<S293>:1:62' for i = 9:16
        // '<S293>:1:63' y(i) = D123451(j);
        std::memcpy(&rtb_y_i[8], &QuadModelv_B.y[0], sizeof(real_T) << 3U);
      } else if ((Memory[1] == 123452.0) || (Memory[1] == 123453.0) || (Memory[1]
                  == 123454.0) || (Memory[0] == 123455.0)) {
        // '<S293>:1:66' elseif FaultIDs(2) == 123452 || FaultIDs(2) == 123453 || FaultIDs(2) == 123454 || FaultIDs(1) == 123455 
        // '<S293>:1:67' j = 1;
        // '<S293>:1:68' for i = 9:10
        // '<S293>:1:69' y(i) = D123452(j);
        rtb_y_i[8] = QuadModelv_B.cell_voltage;

        // '<S293>:1:70' j = j + 1;
        // '<S293>:1:69' y(i) = D123452(j);
        rtb_y_i[9] = QuadModelv_B.remCapacity;

        // '<S293>:1:70' j = j + 1;
      } else if ((Memory[1] == 123456.0) || (Memory[1] == 123457.0) || (Memory[1]
                  == 123458.0)) {
        // '<S293>:1:72' elseif FaultIDs(2) == 123456 || FaultIDs(2) == 123457 || FaultIDs(2) == 123458 
        // '<S293>:1:73' j = 1;
        // '<S293>:1:74' for i = 9:12
        // '<S293>:1:75' y(i) = D123456(j);
        rtb_y_i[8] = QuadModelv_B.uavlostMass;

        // '<S293>:1:76' j = j + 1;
        // '<S293>:1:75' y(i) = D123456(j);
        rtb_y_i[9] = QuadModelv_B.deltaM[0];

        // '<S293>:1:76' j = j + 1;
        // '<S293>:1:75' y(i) = D123456(j);
        rtb_y_i[10] = QuadModelv_B.deltaM[1];

        // '<S293>:1:76' j = j + 1;
        // '<S293>:1:75' y(i) = D123456(j);
        rtb_y_i[11] = QuadModelv_B.deltaM[2];

        // '<S293>:1:76' j = j + 1;
      } else if ((Memory[1] == 123459.0) || (Memory[1] == 123540.0) || (Memory[1]
                  == 123541.0) || (Memory[1] == 123542.0) || (Memory[1] ==
                  123543.0)) {
        // '<S293>:1:78' elseif FaultIDs(2) == 123459 || FaultIDs(2) == 123540 || FaultIDs(2) == 123541 || FaultIDs(2) == 123542 || FaultIDs(2) == 123543 
        // '<S293>:1:79' j = 1;
        // '<S293>:1:80' for i = 9:14
        for (r = 0; r < 6; r++) {
          // '<S293>:1:81' y(i) = D123459(j);
          rtb_y_i[r + 8] = ddm[r];

          // '<S293>:1:82' j = j + 1;
        }
      } else if (Memory[1] == 123544.0) {
        // '<S293>:1:84' elseif FaultIDs(2) == 123544
        // '<S293>:1:85' j = 1;
        // '<S293>:1:86' for i = 9:11
        // '<S293>:1:87' y(i) = D123544(j);
        rtb_y_i[8] = QuadModelv_B.sf_AccNoiseFun_i.y[0];

        // '<S293>:1:88' j = j + 1;
        // '<S293>:1:87' y(i) = D123544(j);
        rtb_y_i[9] = QuadModelv_B.sf_AccNoiseFun_i.y[1];

        // '<S293>:1:88' j = j + 1;
        // '<S293>:1:87' y(i) = D123544(j);
        rtb_y_i[10] = QuadModelv_B.sf_AccNoiseFun_i.y[2];

        // '<S293>:1:88' j = j + 1;
      } else if (Memory[1] == 123545.0) {
        // '<S293>:1:90' elseif FaultIDs(2) == 123545
        // '<S293>:1:91' j = 1;
        // '<S293>:1:92' for i = 9:11
        // '<S293>:1:93' y(i) = D123545(j);
        rtb_y_i[8] = QuadModelv_B.sf_AccNoiseFun_a.y[0];

        // '<S293>:1:94' j = j + 1;
        // '<S293>:1:93' y(i) = D123545(j);
        rtb_y_i[9] = QuadModelv_B.sf_AccNoiseFun_a.y[1];

        // '<S293>:1:94' j = j + 1;
        // '<S293>:1:93' y(i) = D123545(j);
        rtb_y_i[10] = QuadModelv_B.sf_AccNoiseFun_a.y[2];

        // '<S293>:1:94' j = j + 1;
      } else if (Memory[1] == 123546.0) {
        // '<S293>:1:96' elseif FaultIDs(2) == 123546
        // '<S293>:1:97' j = 1;
        // '<S293>:1:98' for i = 9:11
        // '<S293>:1:99' y(i) = D123546(j);
        rtb_y_i[8] = QuadModelv_B.sf_AccNoiseFun_p.y[0];

        // '<S293>:1:100' j = j + 1;
        // '<S293>:1:99' y(i) = D123546(j);
        rtb_y_i[9] = QuadModelv_B.sf_AccNoiseFun_p.y[1];

        // '<S293>:1:100' j = j + 1;
        // '<S293>:1:99' y(i) = D123546(j);
        rtb_y_i[10] = QuadModelv_B.sf_AccNoiseFun_p.y[2];

        // '<S293>:1:100' j = j + 1;
      } else if (Memory[1] == 123547.0) {
        // '<S293>:1:102' elseif FaultIDs(2) == 123547
        // '<S293>:1:103' y(9) = D123547;
        rtb_y_i[8] = rtb_fcn3;
      } else if (Memory[1] == 123548.0) {
        // '<S293>:1:104' elseif FaultIDs(2) == 123548
        // '<S293>:1:105' j = 1;
        // '<S293>:1:106' for i = 9:11
        // '<S293>:1:107' y(i) = D123548(j);
        rtb_y_i[8] = QuadModelv_B.y_m[0];

        // '<S293>:1:108' j = j + 1;
        // '<S293>:1:107' y(i) = D123548(j);
        rtb_y_i[9] = QuadModelv_B.y_m[1];

        // '<S293>:1:108' j = j + 1;
        // '<S293>:1:107' y(i) = D123548(j);
        rtb_y_i[10] = QuadModelv_B.y_m[2];

        // '<S293>:1:108' j = j + 1;
      }
    } else if (Memory[0] == 123451.0) {
      // '<S293>:1:111' elseif FaultIDs(1) == 123451
      // '<S293>:1:112' for i = 1:8
      // '<S293>:1:113' y(i) = D123451(i);
      std::memcpy(&rtb_y_i[0], &QuadModelv_B.y[0], sizeof(real_T) << 3U);

      // '<S293>:1:115' if FaultIDs(2) == 123450|| FaultIDs(2) == 123549
      if ((Memory[1] == 123450.0) || (Memory[1] == 123549.0)) {
        // '<S293>:1:116' j = 1;
        // '<S293>:1:117' for i = 9:16
        // '<S293>:1:118' y(i) = D123450(j);
        std::memcpy(&rtb_y_i[8], &QuadModelv_B.y_d[0], sizeof(real_T) << 3U);
      } else if ((Memory[1] == 123452.0) || (Memory[1] == 123453.0) || (Memory[1]
                  == 123454.0)) {
        // '<S293>:1:121' elseif FaultIDs(2) == 123452 || FaultIDs(2) == 123453 || FaultIDs(2) == 123454 || FaultIDs(1) == 123455 
        // '<S293>:1:122' j = 1;
        // '<S293>:1:123' for i = 9:10
        // '<S293>:1:124' y(i) = D123452(j);
        rtb_y_i[8] = QuadModelv_B.cell_voltage;

        // '<S293>:1:125' j = j + 1;
        // '<S293>:1:124' y(i) = D123452(j);
        rtb_y_i[9] = QuadModelv_B.remCapacity;

        // '<S293>:1:125' j = j + 1;
      } else if ((Memory[1] == 123456.0) || (Memory[1] == 123457.0) || (Memory[1]
                  == 123458.0)) {
        // '<S293>:1:127' elseif FaultIDs(2) == 123456 || FaultIDs(2) == 123457 || FaultIDs(2) == 123458 
        // '<S293>:1:128' j = 1;
        // '<S293>:1:129' for i = 9:12
        // '<S293>:1:130' y(i) = D123456(j);
        rtb_y_i[8] = QuadModelv_B.uavlostMass;

        // '<S293>:1:131' j = j + 1;
        // '<S293>:1:130' y(i) = D123456(j);
        rtb_y_i[9] = QuadModelv_B.deltaM[0];

        // '<S293>:1:131' j = j + 1;
        // '<S293>:1:130' y(i) = D123456(j);
        rtb_y_i[10] = QuadModelv_B.deltaM[1];

        // '<S293>:1:131' j = j + 1;
        // '<S293>:1:130' y(i) = D123456(j);
        rtb_y_i[11] = QuadModelv_B.deltaM[2];

        // '<S293>:1:131' j = j + 1;
      } else if ((Memory[1] == 123459.0) || (Memory[1] == 123540.0) || (Memory[1]
                  == 123541.0) || (Memory[1] == 123542.0) || (Memory[1] ==
                  123543.0)) {
        // '<S293>:1:133' elseif FaultIDs(2) == 123459 || FaultIDs(2) == 123540 || FaultIDs(2) == 123541 || FaultIDs(2) == 123542 || FaultIDs(2) == 123543 
        // '<S293>:1:134' j = 1;
        // '<S293>:1:135' for i = 9:14
        for (r = 0; r < 6; r++) {
          // '<S293>:1:136' y(i) = D123459(j);
          rtb_y_i[r + 8] = ddm[r];

          // '<S293>:1:137' j = j + 1;
        }
      } else if (Memory[1] == 123544.0) {
        // '<S293>:1:139' elseif FaultIDs(2) == 123544
        // '<S293>:1:140' j = 1;
        // '<S293>:1:141' for i = 9:11
        // '<S293>:1:142' y(i) = D123544(j);
        rtb_y_i[8] = QuadModelv_B.sf_AccNoiseFun_i.y[0];

        // '<S293>:1:143' j = j + 1;
        // '<S293>:1:142' y(i) = D123544(j);
        rtb_y_i[9] = QuadModelv_B.sf_AccNoiseFun_i.y[1];

        // '<S293>:1:143' j = j + 1;
        // '<S293>:1:142' y(i) = D123544(j);
        rtb_y_i[10] = QuadModelv_B.sf_AccNoiseFun_i.y[2];

        // '<S293>:1:143' j = j + 1;
      } else if (Memory[1] == 123545.0) {
        // '<S293>:1:145' elseif FaultIDs(2) == 123545
        // '<S293>:1:146' j = 1;
        // '<S293>:1:147' for i = 9:11
        // '<S293>:1:148' y(i) = D123545(j);
        rtb_y_i[8] = QuadModelv_B.sf_AccNoiseFun_a.y[0];

        // '<S293>:1:149' j = j + 1;
        // '<S293>:1:148' y(i) = D123545(j);
        rtb_y_i[9] = QuadModelv_B.sf_AccNoiseFun_a.y[1];

        // '<S293>:1:149' j = j + 1;
        // '<S293>:1:148' y(i) = D123545(j);
        rtb_y_i[10] = QuadModelv_B.sf_AccNoiseFun_a.y[2];

        // '<S293>:1:149' j = j + 1;
      } else if (Memory[1] == 123546.0) {
        // '<S293>:1:151' elseif FaultIDs(2) == 123546
        // '<S293>:1:152' j = 1;
        // '<S293>:1:153' for i = 9:11
        // '<S293>:1:154' y(i) = D123546(j);
        rtb_y_i[8] = QuadModelv_B.sf_AccNoiseFun_p.y[0];

        // '<S293>:1:155' j = j + 1;
        // '<S293>:1:154' y(i) = D123546(j);
        rtb_y_i[9] = QuadModelv_B.sf_AccNoiseFun_p.y[1];

        // '<S293>:1:155' j = j + 1;
        // '<S293>:1:154' y(i) = D123546(j);
        rtb_y_i[10] = QuadModelv_B.sf_AccNoiseFun_p.y[2];

        // '<S293>:1:155' j = j + 1;
      } else if (Memory[1] == 123547.0) {
        // '<S293>:1:157' elseif FaultIDs(2) == 123547
        // '<S293>:1:158' y(9) = D123547;
        rtb_y_i[8] = rtb_fcn3;
      } else if (Memory[1] == 123548.0) {
        // '<S293>:1:159' elseif FaultIDs(2) == 123548
        // '<S293>:1:160' j = 1;
        // '<S293>:1:161' for i = 9:11
        // '<S293>:1:162' y(i) = D123548(j);
        rtb_y_i[8] = QuadModelv_B.y_m[0];

        // '<S293>:1:163' j = j + 1;
        // '<S293>:1:162' y(i) = D123548(j);
        rtb_y_i[9] = QuadModelv_B.y_m[1];

        // '<S293>:1:163' j = j + 1;
        // '<S293>:1:162' y(i) = D123548(j);
        rtb_y_i[10] = QuadModelv_B.y_m[2];

        // '<S293>:1:163' j = j + 1;
      }
    } else if ((Memory[0] == 123452.0) || (Memory[0] == 123453.0) || (Memory[0] ==
                123454.0) || (Memory[0] == 123455.0)) {
      // '<S293>:1:166' elseif FaultIDs(1) == 123452 || FaultIDs(1) == 123453 || FaultIDs(1) == 123454 || FaultIDs(1) == 123455 
      // '<S293>:1:167' for i = 1:2
      // '<S293>:1:168' y(i) = D123452(i);
      rtb_y_i[0] = QuadModelv_B.cell_voltage;

      // '<S293>:1:168' y(i) = D123452(i);
      rtb_y_i[1] = QuadModelv_B.remCapacity;

      // '<S293>:1:170' if FaultIDs(2) == 123450|| FaultIDs(2) == 123549
      if ((Memory[1] == 123450.0) || (Memory[1] == 123549.0)) {
        // '<S293>:1:171' j = 1;
        // '<S293>:1:172' for i = 3:10
        // '<S293>:1:173' y(i) = D123450(j);
        std::memcpy(&rtb_y_i[2], &QuadModelv_B.y_d[0], sizeof(real_T) << 3U);
      } else if (Memory[1] == 123451.0) {
        // '<S293>:1:176' elseif FaultIDs(2) == 123451
        // '<S293>:1:177' j = 1;
        // '<S293>:1:178' for i = 3:10
        // '<S293>:1:179' y(i) = D123451(j);
        std::memcpy(&rtb_y_i[2], &QuadModelv_B.y[0], sizeof(real_T) << 3U);
      } else if ((Memory[1] == 123456.0) || (Memory[1] == 123457.0) || (Memory[1]
                  == 123458.0)) {
        // '<S293>:1:182' elseif FaultIDs(2) == 123456 || FaultIDs(2) == 123457 || FaultIDs(2) == 123458 
        // '<S293>:1:183' j = 1;
        // '<S293>:1:184' for i = 3:6
        // '<S293>:1:185' y(i) = D123456(j);
        rtb_y_i[2] = QuadModelv_B.uavlostMass;

        // '<S293>:1:186' j = j + 1;
        // '<S293>:1:185' y(i) = D123456(j);
        rtb_y_i[3] = QuadModelv_B.deltaM[0];

        // '<S293>:1:186' j = j + 1;
        // '<S293>:1:185' y(i) = D123456(j);
        rtb_y_i[4] = QuadModelv_B.deltaM[1];

        // '<S293>:1:186' j = j + 1;
        // '<S293>:1:185' y(i) = D123456(j);
        rtb_y_i[5] = QuadModelv_B.deltaM[2];

        // '<S293>:1:186' j = j + 1;
      } else if ((Memory[1] == 123459.0) || (Memory[1] == 123540.0) || (Memory[1]
                  == 123541.0) || (Memory[1] == 123542.0) || (Memory[1] ==
                  123543.0)) {
        // '<S293>:1:188' elseif FaultIDs(2) == 123459 || FaultIDs(2) == 123540 || FaultIDs(2) == 123541 || FaultIDs(2) == 123542 || FaultIDs(2) == 123543 
        // '<S293>:1:189' j = 1;
        // '<S293>:1:190' for i = 3:8
        for (r = 0; r < 6; r++) {
          // '<S293>:1:191' y(i) = D123459(j);
          rtb_y_i[r + 2] = ddm[r];

          // '<S293>:1:192' j = j + 1;
        }
      } else if (Memory[1] == 123544.0) {
        // '<S293>:1:194' elseif FaultIDs(2) == 123544
        // '<S293>:1:195' j = 1;
        // '<S293>:1:196' for i = 3:5
        // '<S293>:1:197' y(i) = D123544(j);
        rtb_y_i[2] = QuadModelv_B.sf_AccNoiseFun_i.y[0];

        // '<S293>:1:198' j = j + 1;
        // '<S293>:1:197' y(i) = D123544(j);
        rtb_y_i[3] = QuadModelv_B.sf_AccNoiseFun_i.y[1];

        // '<S293>:1:198' j = j + 1;
        // '<S293>:1:197' y(i) = D123544(j);
        rtb_y_i[4] = QuadModelv_B.sf_AccNoiseFun_i.y[2];

        // '<S293>:1:198' j = j + 1;
      } else if (Memory[1] == 123545.0) {
        // '<S293>:1:200' elseif FaultIDs(2) == 123545
        // '<S293>:1:201' j = 1;
        // '<S293>:1:202' for i = 3:5
        // '<S293>:1:203' y(i) = D123545(j);
        rtb_y_i[2] = QuadModelv_B.sf_AccNoiseFun_a.y[0];

        // '<S293>:1:204' j = j + 1;
        // '<S293>:1:203' y(i) = D123545(j);
        rtb_y_i[3] = QuadModelv_B.sf_AccNoiseFun_a.y[1];

        // '<S293>:1:204' j = j + 1;
        // '<S293>:1:203' y(i) = D123545(j);
        rtb_y_i[4] = QuadModelv_B.sf_AccNoiseFun_a.y[2];

        // '<S293>:1:204' j = j + 1;
      } else if (Memory[1] == 123546.0) {
        // '<S293>:1:206' elseif FaultIDs(2) == 123546
        // '<S293>:1:207' j  = 1;
        // '<S293>:1:208' for i = 3:5
        // '<S293>:1:209' y(i) = D123546(j);
        rtb_y_i[2] = QuadModelv_B.sf_AccNoiseFun_p.y[0];

        // '<S293>:1:210' j = j + 1;
        // '<S293>:1:209' y(i) = D123546(j);
        rtb_y_i[3] = QuadModelv_B.sf_AccNoiseFun_p.y[1];

        // '<S293>:1:210' j = j + 1;
        // '<S293>:1:209' y(i) = D123546(j);
        rtb_y_i[4] = QuadModelv_B.sf_AccNoiseFun_p.y[2];

        // '<S293>:1:210' j = j + 1;
      } else if (Memory[1] == 123547.0) {
        // '<S293>:1:212' elseif FaultIDs(2) == 123547
        // '<S293>:1:213' y(3) = D123547;
        rtb_y_i[2] = rtb_fcn3;
      } else if (Memory[1] == 123548.0) {
        // '<S293>:1:214' elseif FaultIDs(2) == 123548
        // '<S293>:1:215' j = 1;
        // '<S293>:1:216' for i = 3:5
        // '<S293>:1:217' y(i) = D123548(j);
        rtb_y_i[2] = QuadModelv_B.y_m[0];

        // '<S293>:1:218' j = j + 1;
        // '<S293>:1:217' y(i) = D123548(j);
        rtb_y_i[3] = QuadModelv_B.y_m[1];

        // '<S293>:1:218' j = j + 1;
        // '<S293>:1:217' y(i) = D123548(j);
        rtb_y_i[4] = QuadModelv_B.y_m[2];

        // '<S293>:1:218' j = j + 1;
      }
    } else if ((Memory[0] == 123456.0) || (Memory[0] == 123457.0) || (Memory[0] ==
                123458.0)) {
      // '<S293>:1:221' elseif FaultIDs(1) == 123456 || FaultIDs(1) == 123457 || FaultIDs(1) == 123458 
      // '<S293>:1:222' for i = 1:4
      // '<S293>:1:223' y(i) = D123456(i);
      rtb_y_i[0] = QuadModelv_B.uavlostMass;

      // '<S293>:1:223' y(i) = D123456(i);
      rtb_y_i[1] = QuadModelv_B.deltaM[0];

      // '<S293>:1:223' y(i) = D123456(i);
      rtb_y_i[2] = QuadModelv_B.deltaM[1];

      // '<S293>:1:223' y(i) = D123456(i);
      rtb_y_i[3] = QuadModelv_B.deltaM[2];

      // '<S293>:1:225' if FaultIDs(2) == 123450|| FaultIDs(2) == 123549
      if ((Memory[1] == 123450.0) || (Memory[1] == 123549.0)) {
        // '<S293>:1:226' j = 1;
        // '<S293>:1:227' for i = 5:12
        // '<S293>:1:228' y(i) = D123450(j);
        std::memcpy(&rtb_y_i[4], &QuadModelv_B.y_d[0], sizeof(real_T) << 3U);
      } else if (Memory[1] == 123451.0) {
        // '<S293>:1:231' elseif FaultIDs(2) == 123451
        // '<S293>:1:232' j = 1;
        // '<S293>:1:233' for i = 5:12
        // '<S293>:1:234' y(i) = D123451(j);
        std::memcpy(&rtb_y_i[4], &QuadModelv_B.y[0], sizeof(real_T) << 3U);
      } else if ((Memory[1] == 123452.0) || (Memory[1] == 123453.0) || (Memory[1]
                  == 123454.0) || (Memory[1] == 123455.0)) {
        // '<S293>:1:237' elseif FaultIDs(2) == 123452 || FaultIDs(2) == 123453 || FaultIDs(2) == 123454 || FaultIDs(2) == 123455 
        // '<S293>:1:238' j = 1;
        // '<S293>:1:239' for i = 5:6
        // '<S293>:1:240' y(i) = D123452(j);
        rtb_y_i[4] = QuadModelv_B.cell_voltage;

        // '<S293>:1:241' j = j + 1;
        // '<S293>:1:240' y(i) = D123452(j);
        rtb_y_i[5] = QuadModelv_B.remCapacity;

        // '<S293>:1:241' j = j + 1;
      } else if ((Memory[1] == 123459.0) || (Memory[1] == 123540.0) || (Memory[1]
                  == 123541.0) || (Memory[1] == 123542.0) || (Memory[1] ==
                  123543.0)) {
        // '<S293>:1:243' elseif FaultIDs(2) == 123459 || FaultIDs(2) == 123540 || FaultIDs(2) == 123541 || FaultIDs(2) == 123542 || FaultIDs(2) == 123543 
        // '<S293>:1:244' j = 1;
        // '<S293>:1:245' for i = 5:10
        for (r = 0; r < 6; r++) {
          // '<S293>:1:246' y(i) = D123459(j);
          rtb_y_i[r + 4] = ddm[r];

          // '<S293>:1:247' j = j + 1;
        }
      } else if (Memory[1] == 123544.0) {
        // '<S293>:1:249' elseif FaultIDs(2) == 123544
        // '<S293>:1:250' j = 1;
        // '<S293>:1:251' for i = 5:7
        // '<S293>:1:252' y(i) = D123544(j);
        rtb_y_i[4] = QuadModelv_B.sf_AccNoiseFun_i.y[0];

        // '<S293>:1:253' j = j + 1;
        // '<S293>:1:252' y(i) = D123544(j);
        rtb_y_i[5] = QuadModelv_B.sf_AccNoiseFun_i.y[1];

        // '<S293>:1:253' j = j + 1;
        // '<S293>:1:252' y(i) = D123544(j);
        rtb_y_i[6] = QuadModelv_B.sf_AccNoiseFun_i.y[2];

        // '<S293>:1:253' j = j + 1;
      } else if (Memory[1] == 123545.0) {
        // '<S293>:1:255' elseif FaultIDs(2) == 123545
        // '<S293>:1:256' j = 1;
        // '<S293>:1:257' for i = 5:7
        // '<S293>:1:258' y(i) = D123545(j);
        rtb_y_i[4] = QuadModelv_B.sf_AccNoiseFun_a.y[0];

        // '<S293>:1:259' j = j + 1;
        // '<S293>:1:258' y(i) = D123545(j);
        rtb_y_i[5] = QuadModelv_B.sf_AccNoiseFun_a.y[1];

        // '<S293>:1:259' j = j + 1;
        // '<S293>:1:258' y(i) = D123545(j);
        rtb_y_i[6] = QuadModelv_B.sf_AccNoiseFun_a.y[2];

        // '<S293>:1:259' j = j + 1;
      } else if (Memory[1] == 123546.0) {
        // '<S293>:1:261' elseif FaultIDs(2) == 123546
        // '<S293>:1:262' j = 1;
        // '<S293>:1:263' for i = 5:7
        // '<S293>:1:264' y(i) = D123546(j);
        rtb_y_i[4] = QuadModelv_B.sf_AccNoiseFun_p.y[0];

        // '<S293>:1:265' j = j + 1;
        // '<S293>:1:264' y(i) = D123546(j);
        rtb_y_i[5] = QuadModelv_B.sf_AccNoiseFun_p.y[1];

        // '<S293>:1:265' j = j + 1;
        // '<S293>:1:264' y(i) = D123546(j);
        rtb_y_i[6] = QuadModelv_B.sf_AccNoiseFun_p.y[2];

        // '<S293>:1:265' j = j + 1;
      } else if (Memory[1] == 123547.0) {
        // '<S293>:1:267' elseif FaultIDs(2) == 123547
        // '<S293>:1:268' y(5) = D123547;
        rtb_y_i[4] = rtb_fcn3;
      } else if (Memory[1] == 123548.0) {
        // '<S293>:1:269' elseif FaultIDs(2) == 123548
        // '<S293>:1:270' j = 1;
        // '<S293>:1:271' for i = 5:7
        // '<S293>:1:272' y(i) = D123548(j);
        rtb_y_i[4] = QuadModelv_B.y_m[0];

        // '<S293>:1:273' j = j + 1;
        // '<S293>:1:272' y(i) = D123548(j);
        rtb_y_i[5] = QuadModelv_B.y_m[1];

        // '<S293>:1:273' j = j + 1;
        // '<S293>:1:272' y(i) = D123548(j);
        rtb_y_i[6] = QuadModelv_B.y_m[2];

        // '<S293>:1:273' j = j + 1;
      }
    } else if ((Memory[0] == 123459.0) || (Memory[0] == 123540.0) || (Memory[0] ==
                123541.0) || (Memory[0] == 123542.0) || (Memory[0] == 123543.0))
    {
      // '<S293>:1:276' elseif FaultIDs(1) == 123459 || FaultIDs(1) == 123540 || FaultIDs(1) == 123541 || FaultIDs(1) == 123542 || FaultIDs(1) == 123543 
      // '<S293>:1:277' for i = 1:6
      for (r = 0; r < 6; r++) {
        // '<S293>:1:278' y(i) = D123459(i);
        rtb_y_i[r] = ddm[r];
      }

      // '<S293>:1:280' if FaultIDs(2) == 123450|| FaultIDs(2) == 123549
      if ((Memory[1] == 123450.0) || (Memory[1] == 123549.0)) {
        // '<S293>:1:281' j = 1;
        // '<S293>:1:282' for i = 7:14
        // '<S293>:1:283' y(i) = D123450(j);
        std::memcpy(&rtb_y_i[6], &QuadModelv_B.y_d[0], sizeof(real_T) << 3U);
      } else if (Memory[1] == 123451.0) {
        // '<S293>:1:286' elseif FaultIDs(2) == 123451
        // '<S293>:1:287' j = 1;
        // '<S293>:1:288' for i = 7:14
        // '<S293>:1:289' y(i) = D123451(j);
        std::memcpy(&rtb_y_i[6], &QuadModelv_B.y[0], sizeof(real_T) << 3U);
      } else if ((Memory[1] == 123452.0) || (Memory[1] == 123453.0) || (Memory[1]
                  == 123454.0) || (Memory[1] == 123455.0)) {
        // '<S293>:1:292' elseif FaultIDs(2) == 123452 || FaultIDs(2) == 123453 || FaultIDs(2) == 123454 || FaultIDs(2) == 123455 
        // '<S293>:1:293' j = 1;
        // '<S293>:1:294' for i = 7:8
        // '<S293>:1:295' y(i) = D123452(j);
        rtb_y_i[6] = QuadModelv_B.cell_voltage;

        // '<S293>:1:296' j = j + 1;
        // '<S293>:1:295' y(i) = D123452(j);
        rtb_y_i[7] = QuadModelv_B.remCapacity;

        // '<S293>:1:296' j = j + 1;
      } else if ((Memory[1] == 123456.0) || (Memory[1] == 123457.0) || (Memory[1]
                  == 123458.0)) {
        // '<S293>:1:298' elseif FaultIDs(2) == 123456 || FaultIDs(2) == 123457 || FaultIDs(2) == 123458 
        // '<S293>:1:299' j = 1;
        // '<S293>:1:300' for i = 7:12
        for (r = 0; r < 6; r++) {
          // '<S293>:1:301' y(i) = D123456(j);
          rtb_y_i[r + 6] = tz[r];

          // '<S293>:1:302' j = j + 1;
        }
      } else if (Memory[1] == 123544.0) {
        // '<S293>:1:304' elseif FaultIDs(2) == 123544
        // '<S293>:1:305' j = 1;
        // '<S293>:1:306' for i = 7:9
        // '<S293>:1:307' y(i) = D123544(j);
        rtb_y_i[6] = QuadModelv_B.sf_AccNoiseFun_i.y[0];

        // '<S293>:1:308' j = j + 1;
        // '<S293>:1:307' y(i) = D123544(j);
        rtb_y_i[7] = QuadModelv_B.sf_AccNoiseFun_i.y[1];

        // '<S293>:1:308' j = j + 1;
        // '<S293>:1:307' y(i) = D123544(j);
        rtb_y_i[8] = QuadModelv_B.sf_AccNoiseFun_i.y[2];

        // '<S293>:1:308' j = j + 1;
      } else if (Memory[1] == 123545.0) {
        // '<S293>:1:310' elseif FaultIDs(2) == 123545
        // '<S293>:1:311' j = 1;
        // '<S293>:1:312' for i = 7:9
        // '<S293>:1:313' y(i) = D123545(j);
        rtb_y_i[6] = QuadModelv_B.sf_AccNoiseFun_a.y[0];

        // '<S293>:1:314' j = j + 1;
        // '<S293>:1:313' y(i) = D123545(j);
        rtb_y_i[7] = QuadModelv_B.sf_AccNoiseFun_a.y[1];

        // '<S293>:1:314' j = j + 1;
        // '<S293>:1:313' y(i) = D123545(j);
        rtb_y_i[8] = QuadModelv_B.sf_AccNoiseFun_a.y[2];

        // '<S293>:1:314' j = j + 1;
      } else if (Memory[1] == 123546.0) {
        // '<S293>:1:316' elseif FaultIDs(2) == 123546
        // '<S293>:1:317' j = 1;
        // '<S293>:1:318' for i = 7:9
        // '<S293>:1:319' y(i) = D123546(j);
        rtb_y_i[6] = QuadModelv_B.sf_AccNoiseFun_p.y[0];

        // '<S293>:1:320' j = j + 1;
        // '<S293>:1:319' y(i) = D123546(j);
        rtb_y_i[7] = QuadModelv_B.sf_AccNoiseFun_p.y[1];

        // '<S293>:1:320' j = j + 1;
        // '<S293>:1:319' y(i) = D123546(j);
        rtb_y_i[8] = QuadModelv_B.sf_AccNoiseFun_p.y[2];

        // '<S293>:1:320' j = j + 1;
      } else if (Memory[1] == 123547.0) {
        // '<S293>:1:322' elseif FaultIDs(2) == 123547
        // '<S293>:1:323' y(7) = D123547;
        rtb_y_i[6] = rtb_fcn3;
      } else if (Memory[1] == 123548.0) {
        // '<S293>:1:324' elseif FaultIDs(2) == 123548
        // '<S293>:1:325' j = 1;
        // '<S293>:1:326' for i = 7:9
        // '<S293>:1:327' y(i) = D123548(j);
        rtb_y_i[6] = QuadModelv_B.y_m[0];

        // '<S293>:1:328' j = j + 1;
        // '<S293>:1:327' y(i) = D123548(j);
        rtb_y_i[7] = QuadModelv_B.y_m[1];

        // '<S293>:1:328' j = j + 1;
        // '<S293>:1:327' y(i) = D123548(j);
        rtb_y_i[8] = QuadModelv_B.y_m[2];

        // '<S293>:1:328' j = j + 1;
      }
    } else if (Memory[0] == 123544.0) {
      // '<S293>:1:331' elseif FaultIDs(1) == 123544
      // '<S293>:1:332' for i = 1:3
      // '<S293>:1:333' y(i) = D123544(i);
      rtb_y_i[0] = QuadModelv_B.sf_AccNoiseFun_i.y[0];

      // '<S293>:1:333' y(i) = D123544(i);
      rtb_y_i[1] = QuadModelv_B.sf_AccNoiseFun_i.y[1];

      // '<S293>:1:333' y(i) = D123544(i);
      rtb_y_i[2] = QuadModelv_B.sf_AccNoiseFun_i.y[2];

      // '<S293>:1:335' if FaultIDs(2) == 123450|| FaultIDs(2) == 123549
      if ((Memory[1] == 123450.0) || (Memory[1] == 123549.0)) {
        // '<S293>:1:336' j = 1;
        // '<S293>:1:337' for i = 4:11
        // '<S293>:1:338' y(i) = D123450(j);
        std::memcpy(&rtb_y_i[3], &QuadModelv_B.y_d[0], sizeof(real_T) << 3U);
      } else if (Memory[1] == 123451.0) {
        // '<S293>:1:341' elseif FaultIDs(2) == 123451
        // '<S293>:1:342' j = 1;
        // '<S293>:1:343' for i = 4:11
        // '<S293>:1:344' y(i) = D123451(j);
        std::memcpy(&rtb_y_i[3], &QuadModelv_B.y[0], sizeof(real_T) << 3U);
      } else if ((Memory[1] == 123452.0) || (Memory[1] == 123453.0) || (Memory[1]
                  == 123454.0) || (Memory[1] == 123455.0)) {
        // '<S293>:1:347' elseif FaultIDs(2) == 123452 || FaultIDs(2) == 123453 || FaultIDs(2) == 123454 || FaultIDs(2) == 123455 
        // '<S293>:1:348' j = 1;
        // '<S293>:1:349' for i = 4:5
        // '<S293>:1:350' y(i) = D123452(j);
        rtb_y_i[3] = QuadModelv_B.cell_voltage;

        // '<S293>:1:351' j = j + 1;
        // '<S293>:1:350' y(i) = D123452(j);
        rtb_y_i[4] = QuadModelv_B.remCapacity;

        // '<S293>:1:351' j = j + 1;
      } else if ((Memory[1] == 123456.0) || (Memory[1] == 123457.0) || (Memory[1]
                  == 123458.0)) {
        // '<S293>:1:353' elseif FaultIDs(2) == 123456 || FaultIDs(2) == 123457 || FaultIDs(2) == 123458 
        // '<S293>:1:354' j = 1;
        // '<S293>:1:355' for i = 4:9
        for (r = 0; r < 6; r++) {
          // '<S293>:1:356' y(i) = D123456(j);
          rtb_y_i[r + 3] = tz[r];

          // '<S293>:1:357' j = j + 1;
        }
      } else if ((Memory[1] == 123459.0) || (Memory[1] == 123540.0) || (Memory[1]
                  == 123541.0) || (Memory[1] == 123542.0) || (Memory[1] ==
                  123543.0)) {
        // '<S293>:1:359' elseif FaultIDs(2) == 123459 || FaultIDs(2) == 123540 || FaultIDs(2) == 123541 || FaultIDs(2) == 123542 || FaultIDs(2) == 123543 
        // '<S293>:1:360' j = 1;
        // '<S293>:1:361' for i = 4:9
        for (r = 0; r < 6; r++) {
          // '<S293>:1:362' y(i) = D123459(j);
          rtb_y_i[r + 3] = ddm[r];

          // '<S293>:1:363' j = j + 1;
        }
      } else if (Memory[1] == 123545.0) {
        // '<S293>:1:365' elseif FaultIDs(2) == 123545
        // '<S293>:1:366' j = 1;
        // '<S293>:1:367' for i = 4:6
        // '<S293>:1:368' y(i) = D123545(j);
        rtb_y_i[3] = QuadModelv_B.sf_AccNoiseFun_a.y[0];

        // '<S293>:1:369' j = j + 1;
        // '<S293>:1:368' y(i) = D123545(j);
        rtb_y_i[4] = QuadModelv_B.sf_AccNoiseFun_a.y[1];

        // '<S293>:1:369' j = j + 1;
        // '<S293>:1:368' y(i) = D123545(j);
        rtb_y_i[5] = QuadModelv_B.sf_AccNoiseFun_a.y[2];

        // '<S293>:1:369' j = j + 1;
      } else if (Memory[1] == 123546.0) {
        // '<S293>:1:371' elseif FaultIDs(2) == 123546
        // '<S293>:1:372' j = 1;
        // '<S293>:1:373' for i = 4:6
        // '<S293>:1:374' y(i) = D123546(j);
        rtb_y_i[3] = QuadModelv_B.sf_AccNoiseFun_p.y[0];

        // '<S293>:1:375' j = j + 1;
        // '<S293>:1:374' y(i) = D123546(j);
        rtb_y_i[4] = QuadModelv_B.sf_AccNoiseFun_p.y[1];

        // '<S293>:1:375' j = j + 1;
        // '<S293>:1:374' y(i) = D123546(j);
        rtb_y_i[5] = QuadModelv_B.sf_AccNoiseFun_p.y[2];

        // '<S293>:1:375' j = j + 1;
      } else if (Memory[1] == 123547.0) {
        // '<S293>:1:377' elseif FaultIDs(2) == 123547
        // '<S293>:1:378' y(4) = D123547;
        rtb_y_i[3] = rtb_fcn3;
      } else if (Memory[1] == 123548.0) {
        // '<S293>:1:379' elseif FaultIDs(2) == 123548
        // '<S293>:1:380' j = 1;
        // '<S293>:1:381' for i = 4:6
        // '<S293>:1:382' y(i) = D123548(j);
        rtb_y_i[3] = QuadModelv_B.y_m[0];

        // '<S293>:1:383' j = j + 1;
        // '<S293>:1:382' y(i) = D123548(j);
        rtb_y_i[4] = QuadModelv_B.y_m[1];

        // '<S293>:1:383' j = j + 1;
        // '<S293>:1:382' y(i) = D123548(j);
        rtb_y_i[5] = QuadModelv_B.y_m[2];

        // '<S293>:1:383' j = j + 1;
      }
    } else if (Memory[0] == 123545.0) {
      // '<S293>:1:386' elseif FaultIDs(1) == 123545
      // '<S293>:1:387' for i = 1:3
      // '<S293>:1:388' y(i) = D123545(i);
      rtb_y_i[0] = QuadModelv_B.sf_AccNoiseFun_a.y[0];

      // '<S293>:1:388' y(i) = D123545(i);
      rtb_y_i[1] = QuadModelv_B.sf_AccNoiseFun_a.y[1];

      // '<S293>:1:388' y(i) = D123545(i);
      rtb_y_i[2] = QuadModelv_B.sf_AccNoiseFun_a.y[2];

      // '<S293>:1:390' if FaultIDs(2) == 123450|| FaultIDs(2) == 123549
      if ((Memory[1] == 123450.0) || (Memory[1] == 123549.0)) {
        // '<S293>:1:391' j = 1;
        // '<S293>:1:392' for i = 4:11
        // '<S293>:1:393' y(i) = D123450(j);
        std::memcpy(&rtb_y_i[3], &QuadModelv_B.y_d[0], sizeof(real_T) << 3U);
      } else if (Memory[1] == 123451.0) {
        // '<S293>:1:396' elseif FaultIDs(2) == 123451
        // '<S293>:1:397' j = 1;
        // '<S293>:1:398' for i = 4:11
        // '<S293>:1:399' y(i) = D123451(j);
        std::memcpy(&rtb_y_i[3], &QuadModelv_B.y[0], sizeof(real_T) << 3U);
      } else if ((Memory[1] == 123452.0) || (Memory[1] == 123453.0) || (Memory[1]
                  == 123454.0) || (Memory[1] == 123455.0)) {
        // '<S293>:1:402' elseif FaultIDs(2) == 123452 || FaultIDs(2) == 123453 || FaultIDs(2) == 123454 || FaultIDs(2) == 123455 
        // '<S293>:1:403' j = 1;
        // '<S293>:1:404' for i = 4:5
        // '<S293>:1:405' y(i) = D123452(j);
        rtb_y_i[3] = QuadModelv_B.cell_voltage;

        // '<S293>:1:406' j = j + 1;
        // '<S293>:1:405' y(i) = D123452(j);
        rtb_y_i[4] = QuadModelv_B.remCapacity;

        // '<S293>:1:406' j = j + 1;
      } else if ((Memory[1] == 123456.0) || (Memory[1] == 123457.0) || (Memory[1]
                  == 123458.0)) {
        // '<S293>:1:408' elseif FaultIDs(2) == 123456 || FaultIDs(2) == 123457 || FaultIDs(2) == 123458 
        // '<S293>:1:409' j = 1;
        // '<S293>:1:410' for i = 4:9
        for (r = 0; r < 6; r++) {
          // '<S293>:1:411' y(i) = D123456(j);
          rtb_y_i[r + 3] = tz[r];

          // '<S293>:1:412' j = j + 1;
        }
      } else if ((Memory[1] == 123459.0) || (Memory[1] == 123540.0) || (Memory[1]
                  == 123541.0) || (Memory[1] == 123542.0) || (Memory[1] ==
                  123543.0)) {
        // '<S293>:1:414' elseif FaultIDs(2) == 123459 || FaultIDs(2) == 123540 || FaultIDs(2) == 123541 || FaultIDs(2) == 123542 || FaultIDs(2) == 123543 
        // '<S293>:1:415' j = 1;
        // '<S293>:1:416' for i = 4:9
        for (r = 0; r < 6; r++) {
          // '<S293>:1:417' y(i) = D123459(j);
          rtb_y_i[r + 3] = ddm[r];

          // '<S293>:1:418' j = j + 1;
        }
      } else if (Memory[1] == 123544.0) {
        // '<S293>:1:420' elseif FaultIDs(2) == 123544
        // '<S293>:1:421' j = 1;
        // '<S293>:1:422' for i = 4:6
        // '<S293>:1:423' y(i) = D123544(j);
        rtb_y_i[3] = QuadModelv_B.sf_AccNoiseFun_i.y[0];

        // '<S293>:1:424' j = j + 1;
        // '<S293>:1:423' y(i) = D123544(j);
        rtb_y_i[4] = QuadModelv_B.sf_AccNoiseFun_i.y[1];

        // '<S293>:1:424' j = j + 1;
        // '<S293>:1:423' y(i) = D123544(j);
        rtb_y_i[5] = QuadModelv_B.sf_AccNoiseFun_i.y[2];

        // '<S293>:1:424' j = j + 1;
      } else if (Memory[1] == 123546.0) {
        // '<S293>:1:426' elseif FaultIDs(2) == 123546
        // '<S293>:1:427' j = 1;
        // '<S293>:1:428' for i = 4:6
        // '<S293>:1:429' y(i) = D123546(j);
        rtb_y_i[3] = QuadModelv_B.sf_AccNoiseFun_p.y[0];

        // '<S293>:1:430' j = j + 1;
        // '<S293>:1:429' y(i) = D123546(j);
        rtb_y_i[4] = QuadModelv_B.sf_AccNoiseFun_p.y[1];

        // '<S293>:1:430' j = j + 1;
        // '<S293>:1:429' y(i) = D123546(j);
        rtb_y_i[5] = QuadModelv_B.sf_AccNoiseFun_p.y[2];

        // '<S293>:1:430' j = j + 1;
      } else if (Memory[1] == 123547.0) {
        // '<S293>:1:432' elseif FaultIDs(2) == 123547
        // '<S293>:1:433' y(4) = D123547;
        rtb_y_i[3] = rtb_fcn3;
      } else if (Memory[1] == 123548.0) {
        // '<S293>:1:434' elseif FaultIDs(2) == 123548
        // '<S293>:1:435' j = 1;
        // '<S293>:1:436' for i = 4:6
        // '<S293>:1:437' y(i) = D123548(j);
        rtb_y_i[3] = QuadModelv_B.y_m[0];

        // '<S293>:1:438' j = j + 1;
        // '<S293>:1:437' y(i) = D123548(j);
        rtb_y_i[4] = QuadModelv_B.y_m[1];

        // '<S293>:1:438' j = j + 1;
        // '<S293>:1:437' y(i) = D123548(j);
        rtb_y_i[5] = QuadModelv_B.y_m[2];

        // '<S293>:1:438' j = j + 1;
      }
    } else if (Memory[0] == 123546.0) {
      // '<S293>:1:441' elseif FaultIDs(1) == 123546
      // '<S293>:1:442' for i = 1:3
      // '<S293>:1:443' y(i) = D123545(i);
      rtb_y_i[0] = QuadModelv_B.sf_AccNoiseFun_a.y[0];

      // '<S293>:1:443' y(i) = D123545(i);
      rtb_y_i[1] = QuadModelv_B.sf_AccNoiseFun_a.y[1];

      // '<S293>:1:443' y(i) = D123545(i);
      rtb_y_i[2] = QuadModelv_B.sf_AccNoiseFun_a.y[2];

      // '<S293>:1:445' if FaultIDs(2) == 123450|| FaultIDs(2) == 123549
      if ((Memory[1] == 123450.0) || (Memory[1] == 123549.0)) {
        // '<S293>:1:446' j = 1;
        // '<S293>:1:447' for i = 4:11
        // '<S293>:1:448' y(i) = D123450(j);
        std::memcpy(&rtb_y_i[3], &QuadModelv_B.y_d[0], sizeof(real_T) << 3U);
      } else if (Memory[1] == 123451.0) {
        // '<S293>:1:451' elseif FaultIDs(2) == 123451
        // '<S293>:1:452' j = 1;
        // '<S293>:1:453' for i = 4:11
        // '<S293>:1:454' y(i) = D123451(j);
        std::memcpy(&rtb_y_i[3], &QuadModelv_B.y[0], sizeof(real_T) << 3U);
      } else if ((Memory[1] == 123452.0) || (Memory[1] == 123453.0) || (Memory[1]
                  == 123454.0) || (Memory[1] == 123455.0)) {
        // '<S293>:1:457' elseif FaultIDs(2) == 123452 || FaultIDs(2) == 123453 || FaultIDs(2) == 123454 || FaultIDs(2) == 123455 
        // '<S293>:1:458' j = 1;
        // '<S293>:1:459' for i = 4:5
        // '<S293>:1:460' y(i) = D123452(j);
        rtb_y_i[3] = QuadModelv_B.cell_voltage;

        // '<S293>:1:461' j = j + 1;
        // '<S293>:1:460' y(i) = D123452(j);
        rtb_y_i[4] = QuadModelv_B.remCapacity;

        // '<S293>:1:461' j = j + 1;
      } else if ((Memory[1] == 123456.0) || (Memory[1] == 123457.0) || (Memory[1]
                  == 123458.0)) {
        // '<S293>:1:463' elseif FaultIDs(2) == 123456 || FaultIDs(2) == 123457 || FaultIDs(2) == 123458 
        // '<S293>:1:464' j = 1;
        // '<S293>:1:465' for i = 4:9
        for (r = 0; r < 6; r++) {
          // '<S293>:1:466' y(i) = D123456(j);
          rtb_y_i[r + 3] = tz[r];

          // '<S293>:1:467' j = j + 1;
        }
      } else if ((Memory[1] == 123459.0) || (Memory[1] == 123540.0) || (Memory[1]
                  == 123541.0) || (Memory[1] == 123542.0) || (Memory[1] ==
                  123543.0)) {
        // '<S293>:1:469' elseif FaultIDs(2) == 123459 || FaultIDs(2) == 123540 || FaultIDs(2) == 123541 || FaultIDs(2) == 123542 || FaultIDs(2) == 123543 
        // '<S293>:1:470' j = 1;
        // '<S293>:1:471' for i = 4:9
        for (r = 0; r < 6; r++) {
          // '<S293>:1:472' y(i) = D123459(j);
          rtb_y_i[r + 3] = ddm[r];

          // '<S293>:1:473' j = j + 1;
        }
      } else if (Memory[1] == 123544.0) {
        // '<S293>:1:475' elseif FaultIDs(2) == 123544
        // '<S293>:1:476' j = 1;
        // '<S293>:1:477' for i = 4:6
        // '<S293>:1:478' y(i) = D123544(j);
        rtb_y_i[3] = QuadModelv_B.sf_AccNoiseFun_i.y[0];

        // '<S293>:1:479' j = j + 1;
        // '<S293>:1:478' y(i) = D123544(j);
        rtb_y_i[4] = QuadModelv_B.sf_AccNoiseFun_i.y[1];

        // '<S293>:1:479' j = j + 1;
        // '<S293>:1:478' y(i) = D123544(j);
        rtb_y_i[5] = QuadModelv_B.sf_AccNoiseFun_i.y[2];

        // '<S293>:1:479' j = j + 1;
      } else if (Memory[1] == 123545.0) {
        // '<S293>:1:481' elseif FaultIDs(2) == 123545
        // '<S293>:1:482' j = 1;
        // '<S293>:1:483' for i = 4:6
        // '<S293>:1:484' y(i) = D123545(j);
        rtb_y_i[3] = QuadModelv_B.sf_AccNoiseFun_a.y[0];

        // '<S293>:1:485' j = j + 1;
        // '<S293>:1:484' y(i) = D123545(j);
        rtb_y_i[4] = QuadModelv_B.sf_AccNoiseFun_a.y[1];

        // '<S293>:1:485' j = j + 1;
        // '<S293>:1:484' y(i) = D123545(j);
        rtb_y_i[5] = QuadModelv_B.sf_AccNoiseFun_a.y[2];

        // '<S293>:1:485' j = j + 1;
      } else if (Memory[1] == 123547.0) {
        // '<S293>:1:487' elseif FaultIDs(2) == 123547
        // '<S293>:1:488' y(4) = D123547;
        rtb_y_i[3] = rtb_fcn3;
      } else if (Memory[1] == 123548.0) {
        // '<S293>:1:489' elseif FaultIDs(2) == 123548
        // '<S293>:1:490' j = 1;
        // '<S293>:1:491' for i = 4:6
        // '<S293>:1:492' y(i) = D123548(j);
        rtb_y_i[3] = QuadModelv_B.y_m[0];

        // '<S293>:1:493' j = j + 1;
        // '<S293>:1:492' y(i) = D123548(j);
        rtb_y_i[4] = QuadModelv_B.y_m[1];

        // '<S293>:1:493' j = j + 1;
        // '<S293>:1:492' y(i) = D123548(j);
        rtb_y_i[5] = QuadModelv_B.y_m[2];

        // '<S293>:1:493' j = j + 1;
      }
    } else if (Memory[0] == 123547.0) {
      // '<S293>:1:496' elseif FaultIDs(1) == 123547
      // '<S293>:1:497' y(1) = D123547(1);
      rtb_y_i[0] = rtb_fcn3;

      // '<S293>:1:498' if FaultIDs(2) == 123450|| FaultIDs(2) == 123549
      if ((Memory[1] == 123450.0) || (Memory[1] == 123549.0)) {
        // '<S293>:1:499' j = 1;
        // '<S293>:1:500' for i = 2:9
        // '<S293>:1:501' y(i) = D123450(j);
        std::memcpy(&rtb_y_i[1], &QuadModelv_B.y_d[0], sizeof(real_T) << 3U);
      } else if (Memory[1] == 123451.0) {
        // '<S293>:1:504' elseif FaultIDs(2) == 123451
        // '<S293>:1:505' j = 1;
        // '<S293>:1:506' for i = 2:9
        // '<S293>:1:507' y(i) = D123451(j);
        std::memcpy(&rtb_y_i[1], &QuadModelv_B.y[0], sizeof(real_T) << 3U);
      } else if ((Memory[1] == 123452.0) || (Memory[1] == 123453.0) || (Memory[1]
                  == 123454.0) || (Memory[1] == 123455.0)) {
        // '<S293>:1:510' elseif FaultIDs(2) == 123452 || FaultIDs(2) == 123453 || FaultIDs(2) == 123454 || FaultIDs(2) == 123455 
        // '<S293>:1:511' j = 1;
        // '<S293>:1:512' for i = 2:3
        // '<S293>:1:513' y(i) = D123452(j);
        rtb_y_i[1] = QuadModelv_B.cell_voltage;

        // '<S293>:1:514' j = j + 1;
        // '<S293>:1:513' y(i) = D123452(j);
        rtb_y_i[2] = QuadModelv_B.remCapacity;

        // '<S293>:1:514' j = j + 1;
      } else if ((Memory[1] == 123456.0) || (Memory[1] == 123457.0) || (Memory[1]
                  == 123458.0)) {
        // '<S293>:1:516' elseif FaultIDs(2) == 123456 || FaultIDs(2) == 123457 || FaultIDs(2) == 123458 
        // '<S293>:1:517' j = 1;
        // '<S293>:1:518' for i = 2:7
        for (r = 0; r < 6; r++) {
          // '<S293>:1:519' y(i) = D123456(j);
          rtb_y_i[r + 1] = tz[r];

          // '<S293>:1:520' j = j + 1;
        }
      } else if ((Memory[1] == 123459.0) || (Memory[1] == 123540.0) || (Memory[1]
                  == 123541.0) || (Memory[1] == 123542.0) || (Memory[1] ==
                  123543.0)) {
        // '<S293>:1:522' elseif FaultIDs(2) == 123459 || FaultIDs(2) == 123540 || FaultIDs(2) == 123541 || FaultIDs(2) == 123542 || FaultIDs(2) == 123543 
        // '<S293>:1:523' j = 1;
        // '<S293>:1:524' for i = 2:7
        for (r = 0; r < 6; r++) {
          // '<S293>:1:525' y(i) = D123459(j);
          rtb_y_i[r + 1] = ddm[r];

          // '<S293>:1:526' j = j + 1;
        }
      } else if (Memory[1] == 123544.0) {
        // '<S293>:1:528' elseif FaultIDs(2) == 123544
        // '<S293>:1:529' j = 1;
        // '<S293>:1:530' for i = 2:4
        // '<S293>:1:531' y(i) = D123544(j);
        rtb_y_i[1] = QuadModelv_B.sf_AccNoiseFun_i.y[0];

        // '<S293>:1:532' j = j + 1;
        // '<S293>:1:531' y(i) = D123544(j);
        rtb_y_i[2] = QuadModelv_B.sf_AccNoiseFun_i.y[1];

        // '<S293>:1:532' j = j + 1;
        // '<S293>:1:531' y(i) = D123544(j);
        rtb_y_i[3] = QuadModelv_B.sf_AccNoiseFun_i.y[2];

        // '<S293>:1:532' j = j + 1;
      } else if (Memory[1] == 123545.0) {
        // '<S293>:1:534' elseif FaultIDs(2) == 123545
        // '<S293>:1:535' j = 1;
        // '<S293>:1:536' for i = 2:4
        // '<S293>:1:537' y(i) = D123545(j);
        rtb_y_i[1] = QuadModelv_B.sf_AccNoiseFun_a.y[0];

        // '<S293>:1:538' j = j + 1;
        // '<S293>:1:537' y(i) = D123545(j);
        rtb_y_i[2] = QuadModelv_B.sf_AccNoiseFun_a.y[1];

        // '<S293>:1:538' j = j + 1;
        // '<S293>:1:537' y(i) = D123545(j);
        rtb_y_i[3] = QuadModelv_B.sf_AccNoiseFun_a.y[2];

        // '<S293>:1:538' j = j + 1;
      } else if (Memory[1] == 123546.0) {
        // '<S293>:1:540' elseif FaultIDs(2) == 123546
        // '<S293>:1:541' j = 1;
        // '<S293>:1:542' for i = 2:4
        // '<S293>:1:543' y(i) = D123546(j);
        rtb_y_i[1] = QuadModelv_B.sf_AccNoiseFun_p.y[0];

        // '<S293>:1:544' j = j + 1;
        // '<S293>:1:543' y(i) = D123546(j);
        rtb_y_i[2] = QuadModelv_B.sf_AccNoiseFun_p.y[1];

        // '<S293>:1:544' j = j + 1;
        // '<S293>:1:543' y(i) = D123546(j);
        rtb_y_i[3] = QuadModelv_B.sf_AccNoiseFun_p.y[2];

        // '<S293>:1:544' j = j + 1;
      } else if (Memory[1] == 123548.0) {
        // '<S293>:1:546' elseif FaultIDs(2) == 123548
        // '<S293>:1:547' j = 1;
        // '<S293>:1:548' for i = 2:4
        // '<S293>:1:549' y(i) = D123548(j);
        rtb_y_i[1] = QuadModelv_B.y_m[0];

        // '<S293>:1:550' j = j + 1;
        // '<S293>:1:549' y(i) = D123548(j);
        rtb_y_i[2] = QuadModelv_B.y_m[1];

        // '<S293>:1:550' j = j + 1;
        // '<S293>:1:549' y(i) = D123548(j);
        rtb_y_i[3] = QuadModelv_B.y_m[2];

        // '<S293>:1:550' j = j + 1;
      }
    } else if (Memory[0] == 123548.0) {
      // '<S293>:1:553' elseif FaultIDs(1) == 123548
      // '<S293>:1:554' for i = 1:3
      // '<S293>:1:555' y(i) = D123548(i);
      rtb_y_i[0] = QuadModelv_B.y_m[0];

      // '<S293>:1:555' y(i) = D123548(i);
      rtb_y_i[1] = QuadModelv_B.y_m[1];

      // '<S293>:1:555' y(i) = D123548(i);
      rtb_y_i[2] = QuadModelv_B.y_m[2];

      // '<S293>:1:557' if FaultIDs(2) == 123450|| FaultIDs(2) == 123549
      if ((Memory[1] == 123450.0) || (Memory[1] == 123549.0)) {
        // '<S293>:1:558' j = 1;
        // '<S293>:1:559' for i = 4:11
        // '<S293>:1:560' y(i) = D123450(j);
        std::memcpy(&rtb_y_i[3], &QuadModelv_B.y_d[0], sizeof(real_T) << 3U);
      } else if (Memory[1] == 123451.0) {
        // '<S293>:1:563' elseif FaultIDs(2) == 123451
        // '<S293>:1:564' j = 1;
        // '<S293>:1:565' for i = 4:11
        // '<S293>:1:566' y(i) = D123451(j);
        std::memcpy(&rtb_y_i[3], &QuadModelv_B.y[0], sizeof(real_T) << 3U);
      } else if ((Memory[1] == 123452.0) || (Memory[1] == 123453.0) || (Memory[1]
                  == 123454.0) || (Memory[1] == 123455.0)) {
        // '<S293>:1:569' elseif FaultIDs(2) == 123452 || FaultIDs(2) == 123453 || FaultIDs(2) == 123454 || FaultIDs(2) == 123455 
        // '<S293>:1:570' j = 1;
        // '<S293>:1:571' for i = 4:5
        // '<S293>:1:572' y(i) = D123452(j);
        rtb_y_i[3] = QuadModelv_B.cell_voltage;

        // '<S293>:1:573' j = j + 1;
        // '<S293>:1:572' y(i) = D123452(j);
        rtb_y_i[4] = QuadModelv_B.remCapacity;

        // '<S293>:1:573' j = j + 1;
      } else if ((Memory[1] == 123456.0) || (Memory[1] == 123457.0) || (Memory[1]
                  == 123458.0)) {
        // '<S293>:1:575' elseif FaultIDs(2) == 123456 || FaultIDs(2) == 123457 || FaultIDs(2) == 123458 
        // '<S293>:1:576' j = 1;
        // '<S293>:1:577' for i = 4:9
        for (r = 0; r < 6; r++) {
          // '<S293>:1:578' y(i) = D123456(j);
          rtb_y_i[r + 3] = tz[r];

          // '<S293>:1:579' j = j + 1;
        }
      } else if ((Memory[1] == 123459.0) || (Memory[1] == 123540.0) || (Memory[1]
                  == 123541.0) || (Memory[1] == 123542.0) || (Memory[1] ==
                  123543.0)) {
        // '<S293>:1:581' elseif FaultIDs(2) == 123459 || FaultIDs(2) == 123540 || FaultIDs(2) == 123541 || FaultIDs(2) == 123542 || FaultIDs(2) == 123543 
        // '<S293>:1:582' j = 1;
        // '<S293>:1:583' for i = 4:9
        for (r = 0; r < 6; r++) {
          // '<S293>:1:584' y(i) = D123459(j);
          rtb_y_i[r + 3] = ddm[r];

          // '<S293>:1:585' j = j + 1;
        }
      } else if (Memory[1] == 123544.0) {
        // '<S293>:1:587' elseif FaultIDs(2) == 123544
        // '<S293>:1:588' j = 1;
        // '<S293>:1:589' for i = 4:6
        // '<S293>:1:590' y(i) = D123544(j);
        rtb_y_i[3] = QuadModelv_B.sf_AccNoiseFun_i.y[0];

        // '<S293>:1:591' j = j + 1;
        // '<S293>:1:590' y(i) = D123544(j);
        rtb_y_i[4] = QuadModelv_B.sf_AccNoiseFun_i.y[1];

        // '<S293>:1:591' j = j + 1;
        // '<S293>:1:590' y(i) = D123544(j);
        rtb_y_i[5] = QuadModelv_B.sf_AccNoiseFun_i.y[2];

        // '<S293>:1:591' j = j + 1;
      } else if (Memory[1] == 123545.0) {
        // '<S293>:1:593' elseif FaultIDs(2) == 123545
        // '<S293>:1:594' j = 1;
        // '<S293>:1:595' for i = 4:6
        // '<S293>:1:596' y(i) = D123545(j);
        rtb_y_i[3] = QuadModelv_B.sf_AccNoiseFun_a.y[0];

        // '<S293>:1:597' j = j + 1;
        // '<S293>:1:596' y(i) = D123545(j);
        rtb_y_i[4] = QuadModelv_B.sf_AccNoiseFun_a.y[1];

        // '<S293>:1:597' j = j + 1;
        // '<S293>:1:596' y(i) = D123545(j);
        rtb_y_i[5] = QuadModelv_B.sf_AccNoiseFun_a.y[2];

        // '<S293>:1:597' j = j + 1;
      } else if (Memory[1] == 123547.0) {
        // '<S293>:1:599' elseif FaultIDs(2) == 123547
        // '<S293>:1:600' y(4) = D123547;
        rtb_y_i[3] = rtb_fcn3;
      } else if (Memory[1] == 123546.0) {
        // '<S293>:1:601' elseif FaultIDs(2) == 123546
        // '<S293>:1:602' j = 1;
        // '<S293>:1:603' for i = 4:6
        // '<S293>:1:604' y(i) = D123546(j);
        rtb_y_i[3] = QuadModelv_B.sf_AccNoiseFun_p.y[0];

        // '<S293>:1:605' j = j + 1;
        // '<S293>:1:604' y(i) = D123546(j);
        rtb_y_i[4] = QuadModelv_B.sf_AccNoiseFun_p.y[1];

        // '<S293>:1:605' j = j + 1;
        // '<S293>:1:604' y(i) = D123546(j);
        rtb_y_i[5] = QuadModelv_B.sf_AccNoiseFun_p.y[2];

        // '<S293>:1:605' j = j + 1;
      }
    }
  }

  // '<S293>:1:611' if FaultNum >= 3
  if (FaultNum >= 3.0) {
    boolean_T guard1 = false;
    boolean_T guard10 = false;
    boolean_T guard11 = false;
    boolean_T guard12 = false;
    boolean_T guard13 = false;
    boolean_T guard14 = false;
    boolean_T guard15 = false;
    boolean_T guard16 = false;
    boolean_T guard2 = false;
    boolean_T guard3 = false;
    boolean_T guard4 = false;
    boolean_T guard5 = false;
    boolean_T guard6 = false;
    boolean_T guard7 = false;
    boolean_T guard8 = false;
    boolean_T guard9 = false;

    // '<S293>:1:612' if any(find(FaultIDs==123450|FaultIDs==123549))&&any(find(FaultIDs==123452|FaultIDs==123453|FaultIDs==123454|FaultIDs==123455))&&any(find(FaultIDs==123456|FaultIDs==123457|FaultIDs==123458)) 
    for (i = 0; i < 8; i++) {
      FaultNum = Memory[i];
      Memory_0[i] = ((FaultNum == 123450.0) || (FaultNum == 123549.0));
    }

    QuadModelv_eml_find(Memory_0, tmp_data, &i);
    guard1 = false;
    guard2 = false;
    guard3 = false;
    guard4 = false;
    guard5 = false;
    guard6 = false;
    guard7 = false;
    guard8 = false;
    guard9 = false;
    guard10 = false;
    guard11 = false;
    guard12 = false;
    guard13 = false;
    guard14 = false;
    guard15 = false;
    guard16 = false;
    if (QuadModelv_any(&i)) {
      for (i = 0; i < 8; i++) {
        FaultNum = Memory[i];
        Memory_0[i] = ((FaultNum == 123452.0) || (FaultNum == 123453.0) ||
                       (FaultNum == 123454.0) || (FaultNum == 123455.0));
      }

      QuadModelv_eml_find(Memory_0, tmp_data, &i);
      if (QuadModelv_any(&i)) {
        for (i = 0; i < 8; i++) {
          FaultNum = Memory[i];
          Memory_0[i] = ((FaultNum == 123456.0) || (FaultNum == 123457.0) ||
                         (FaultNum == 123458.0));
        }

        QuadModelv_eml_find(Memory_0, tmp_data, &i);
        if (QuadModelv_any(&i)) {
          // '<S293>:1:613' for i = 1:8
          // '<S293>:1:614' y(i) = D123450(i);
          std::memcpy(&rtb_y_i[0], &QuadModelv_B.y_d[0], sizeof(real_T) << 3U);

          // '<S293>:1:616' j = 1;
          // '<S293>:1:617' for i = 9:10
          // '<S293>:1:618' y(i) = D123452(j);
          rtb_y_i[8] = QuadModelv_B.cell_voltage;

          // '<S293>:1:619' j = j + 1;
          // '<S293>:1:618' y(i) = D123452(j);
          rtb_y_i[9] = QuadModelv_B.remCapacity;

          // '<S293>:1:619' j = j + 1;
          // '<S293>:1:621' k = 1;
          // '<S293>:1:622' for i = 11:14
          // '<S293>:1:623' y(i) = D123456(k);
          rtb_y_i[10] = QuadModelv_B.uavlostMass;

          // '<S293>:1:624' k = k + 1;
          // '<S293>:1:623' y(i) = D123456(k);
          rtb_y_i[11] = QuadModelv_B.deltaM[0];

          // '<S293>:1:624' k = k + 1;
          // '<S293>:1:623' y(i) = D123456(k);
          rtb_y_i[12] = QuadModelv_B.deltaM[1];

          // '<S293>:1:624' k = k + 1;
          // '<S293>:1:623' y(i) = D123456(k);
          rtb_y_i[13] = QuadModelv_B.deltaM[2];

          // '<S293>:1:624' k = k + 1;
        } else {
          guard16 = true;
        }
      } else {
        guard16 = true;
      }
    } else {
      guard16 = true;
    }

    if (guard16) {
      for (i = 0; i < 8; i++) {
        FaultNum = Memory[i];
        Memory_0[i] = ((FaultNum == 123450.0) || (FaultNum == 123549.0));
      }

      QuadModelv_eml_find(Memory_0, tmp_data, &i);
      if (QuadModelv_any(&i)) {
        for (i = 0; i < 8; i++) {
          FaultNum = Memory[i];
          Memory_0[i] = ((FaultNum == 123452.0) || (FaultNum == 123453.0) ||
                         (FaultNum == 123454.0) || (FaultNum == 123455.0));
        }

        QuadModelv_eml_find(Memory_0, tmp_data, &i);
        if (QuadModelv_any(&i)) {
          for (i = 0; i < 8; i++) {
            FaultNum = Memory[i];
            Memory_0[i] = ((FaultNum == 123459.0) || (FaultNum == 123540.0) ||
                           (FaultNum == 123541.0) || (FaultNum == 123542.0) ||
                           (FaultNum == 123543.0));
          }

          QuadModelv_eml_find(Memory_0, tmp_data, &i);
          if (QuadModelv_any(&i)) {
            // '<S293>:1:626' elseif any(find(FaultIDs==123450|FaultIDs==123549))&&any(find(FaultIDs==123452|FaultIDs==123453|FaultIDs==123454|FaultIDs==123455))&&any(find(FaultIDs==123459|FaultIDs==123540|FaultIDs==123541|FaultIDs==123542|FaultIDs==123543)) 
            // '<S293>:1:627' for i = 1:8
            // '<S293>:1:628' y(i) = D123450(i);
            std::memcpy(&rtb_y_i[0], &QuadModelv_B.y_d[0], sizeof(real_T) << 3U);

            // '<S293>:1:630' j = 1;
            // '<S293>:1:631' for i = 9:10
            // '<S293>:1:632' y(i) = D123452(j);
            rtb_y_i[8] = QuadModelv_B.cell_voltage;

            // '<S293>:1:633' j = j + 1;
            // '<S293>:1:632' y(i) = D123452(j);
            rtb_y_i[9] = QuadModelv_B.remCapacity;

            // '<S293>:1:633' j = j + 1;
            // '<S293>:1:635' k = 1;
            // '<S293>:1:636' for i = 11:16
            for (r = 0; r < 6; r++) {
              // '<S293>:1:637' y(i) = D123459(k);
              rtb_y_i[r + 10] = ddm[r];

              // '<S293>:1:638' k = k + 1;
            }
          } else {
            guard15 = true;
          }
        } else {
          guard15 = true;
        }
      } else {
        guard15 = true;
      }
    }

    if (guard15) {
      for (i = 0; i < 8; i++) {
        FaultNum = Memory[i];
        Memory_0[i] = ((FaultNum == 123450.0) || (FaultNum == 123549.0));
      }

      QuadModelv_eml_find(Memory_0, tmp_data, &i);
      if (QuadModelv_any(&i)) {
        for (i = 0; i < 8; i++) {
          FaultNum = Memory[i];
          Memory_0[i] = ((FaultNum == 123452.0) || (FaultNum == 123453.0) ||
                         (FaultNum == 123454.0) || (FaultNum == 123455.0));
        }

        QuadModelv_eml_find(Memory_0, tmp_data, &i);
        if (QuadModelv_any(&i)) {
          for (i = 0; i < 8; i++) {
            FaultNum = Memory[i];
            Memory_0[i] = ((FaultNum == 123544.0) || (FaultNum == 123545.0) ||
                           (FaultNum == 123546.0) || (FaultNum == 123548.0) ||
                           (FaultNum == 123547.0));
          }

          QuadModelv_eml_find(Memory_0, tmp_data, &i);
          if (QuadModelv_any(&i)) {
            // '<S293>:1:640' elseif any(find(FaultIDs==123450|FaultIDs==123549))&&any(find(FaultIDs==123452|FaultIDs==123453|FaultIDs==123454|FaultIDs==123455))&&any(find(FaultIDs==123544|FaultIDs==123545|FaultIDs==123546|FaultIDs==123548|FaultIDs==123547)) 
            // '<S293>:1:641' for i = 1:8
            // '<S293>:1:644' j = 1;
            // '<S293>:1:645' for i = 9:10
            // '<S293>:1:646' y(i) = D123452(j);
            rtb_y_i[8] = QuadModelv_B.cell_voltage;

            // '<S293>:1:647' j = j + 1;
            // '<S293>:1:646' y(i) = D123452(j);
            rtb_y_i[9] = QuadModelv_B.remCapacity;

            // '<S293>:1:647' j = j + 1;
            // '<S293>:1:649' k = 1;
            // '<S293>:1:650' if any(find(FaultIDs==123544))
            for (r = 0; r < 8; r++) {
              // '<S293>:1:642' y(i) = D123450(i);
              rtb_y_i[r] = QuadModelv_B.y_d[r];
              Memory_0[r] = (Memory[r] == 123544.0);
            }

            QuadModelv_eml_find(Memory_0, tmp_data, &i);
            if (QuadModelv_any(&i)) {
              // '<S293>:1:651' for i = 11:13
              // '<S293>:1:652' y(i) = D123544(k);
              rtb_y_i[10] = QuadModelv_B.sf_AccNoiseFun_i.y[0];

              // '<S293>:1:653' k = k + 1;
              // '<S293>:1:652' y(i) = D123544(k);
              rtb_y_i[11] = QuadModelv_B.sf_AccNoiseFun_i.y[1];

              // '<S293>:1:653' k = k + 1;
              // '<S293>:1:652' y(i) = D123544(k);
              rtb_y_i[12] = QuadModelv_B.sf_AccNoiseFun_i.y[2];

              // '<S293>:1:653' k = k + 1;
            } else {
              for (i = 0; i < 8; i++) {
                Memory_0[i] = (Memory[i] == 123545.0);
              }

              QuadModelv_eml_find(Memory_0, tmp_data, &i);
              if (QuadModelv_any(&i)) {
                // '<S293>:1:655' elseif any(find(FaultIDs==123545))
                // '<S293>:1:656' for i = 11:13
                // '<S293>:1:657' y(i) = D123545(k);
                rtb_y_i[10] = QuadModelv_B.sf_AccNoiseFun_a.y[0];

                // '<S293>:1:658' k = k + 1;
                // '<S293>:1:657' y(i) = D123545(k);
                rtb_y_i[11] = QuadModelv_B.sf_AccNoiseFun_a.y[1];

                // '<S293>:1:658' k = k + 1;
                // '<S293>:1:657' y(i) = D123545(k);
                rtb_y_i[12] = QuadModelv_B.sf_AccNoiseFun_a.y[2];

                // '<S293>:1:658' k = k + 1;
              } else {
                for (i = 0; i < 8; i++) {
                  Memory_0[i] = (Memory[i] == 123546.0);
                }

                QuadModelv_eml_find(Memory_0, tmp_data, &i);
                if (QuadModelv_any(&i)) {
                  // '<S293>:1:660' elseif any(find(FaultIDs==123546))
                  // '<S293>:1:661' for i = 11:13
                  // '<S293>:1:662' y(i) = D123546(k);
                  rtb_y_i[10] = QuadModelv_B.sf_AccNoiseFun_p.y[0];

                  // '<S293>:1:663' k = k + 1;
                  // '<S293>:1:662' y(i) = D123546(k);
                  rtb_y_i[11] = QuadModelv_B.sf_AccNoiseFun_p.y[1];

                  // '<S293>:1:663' k = k + 1;
                  // '<S293>:1:662' y(i) = D123546(k);
                  rtb_y_i[12] = QuadModelv_B.sf_AccNoiseFun_p.y[2];

                  // '<S293>:1:663' k = k + 1;
                } else {
                  for (i = 0; i < 8; i++) {
                    Memory_0[i] = (Memory[i] == 123548.0);
                  }

                  QuadModelv_eml_find(Memory_0, tmp_data, &i);
                  if (QuadModelv_any(&i)) {
                    // '<S293>:1:665' elseif any(find(FaultIDs==123548))
                    // '<S293>:1:666' for i = 11:13
                    // '<S293>:1:667' y(i) = D123548(k);
                    rtb_y_i[10] = QuadModelv_B.y_m[0];

                    // '<S293>:1:668' k = k + 1;
                    // '<S293>:1:667' y(i) = D123548(k);
                    rtb_y_i[11] = QuadModelv_B.y_m[1];

                    // '<S293>:1:668' k = k + 1;
                    // '<S293>:1:667' y(i) = D123548(k);
                    rtb_y_i[12] = QuadModelv_B.y_m[2];

                    // '<S293>:1:668' k = k + 1;
                  } else {
                    for (i = 0; i < 8; i++) {
                      Memory_0[i] = (Memory[i] == 123547.0);
                    }

                    QuadModelv_eml_find(Memory_0, tmp_data, &i);
                    if (QuadModelv_any(&i)) {
                      // '<S293>:1:670' elseif any(find(FaultIDs==123547))
                      // '<S293>:1:671' y(11) = D123547;
                      rtb_y_i[10] = rtb_fcn3;
                    }
                  }
                }
              }
            }
          } else {
            guard14 = true;
          }
        } else {
          guard14 = true;
        }
      } else {
        guard14 = true;
      }
    }

    if (guard14) {
      for (i = 0; i < 8; i++) {
        FaultNum = Memory[i];
        Memory_0[i] = ((FaultNum == 123452.0) || (FaultNum == 123453.0) ||
                       (FaultNum == 123454.0) || (FaultNum == 123455.0));
      }

      QuadModelv_eml_find(Memory_0, tmp_data, &i);
      if (QuadModelv_any(&i)) {
        for (i = 0; i < 8; i++) {
          FaultNum = Memory[i];
          Memory_0[i] = ((FaultNum == 123456.0) || (FaultNum == 123457.0) ||
                         (FaultNum == 123458.0));
        }

        QuadModelv_eml_find(Memory_0, tmp_data, &i);
        if (QuadModelv_any(&i)) {
          for (i = 0; i < 8; i++) {
            FaultNum = Memory[i];
            Memory_0[i] = ((FaultNum == 123459.0) || (FaultNum == 123540.0) ||
                           (FaultNum == 123541.0) || (FaultNum == 123542.0) ||
                           (FaultNum == 123543.0));
          }

          QuadModelv_eml_find(Memory_0, tmp_data, &i);
          if (QuadModelv_any(&i)) {
            // '<S293>:1:673' elseif any(find(FaultIDs==123452|FaultIDs==123453|FaultIDs==123454|FaultIDs==123455))&&any(find(FaultIDs==123456|FaultIDs==123457|FaultIDs==123458))&&any(find(FaultIDs==123459|FaultIDs==123540|FaultIDs==123541|FaultIDs==123542|FaultIDs==123543)) 
            // '<S293>:1:674' for i = 1:2
            // '<S293>:1:675' y(i) = D123452(i);
            rtb_y_i[0] = QuadModelv_B.cell_voltage;

            // '<S293>:1:675' y(i) = D123452(i);
            rtb_y_i[1] = QuadModelv_B.remCapacity;

            // '<S293>:1:677' j = 1;
            // '<S293>:1:678' for i = 3:6
            // '<S293>:1:679' y(i) = D123456(j);
            rtb_y_i[2] = QuadModelv_B.uavlostMass;

            // '<S293>:1:680' j = j + 1;
            // '<S293>:1:679' y(i) = D123456(j);
            rtb_y_i[3] = QuadModelv_B.deltaM[0];

            // '<S293>:1:680' j = j + 1;
            // '<S293>:1:679' y(i) = D123456(j);
            rtb_y_i[4] = QuadModelv_B.deltaM[1];

            // '<S293>:1:680' j = j + 1;
            // '<S293>:1:679' y(i) = D123456(j);
            rtb_y_i[5] = QuadModelv_B.deltaM[2];

            // '<S293>:1:680' j = j + 1;
            // '<S293>:1:682' k = 1;
            // '<S293>:1:683' for i = 7:12
            for (r = 0; r < 6; r++) {
              // '<S293>:1:684' y(i) = D123459(k);
              rtb_y_i[r + 6] = ddm[r];

              // '<S293>:1:685' k = k + 1;
            }
          } else {
            guard13 = true;
          }
        } else {
          guard13 = true;
        }
      } else {
        guard13 = true;
      }
    }

    if (guard13) {
      for (i = 0; i < 8; i++) {
        FaultNum = Memory[i];
        Memory_0[i] = ((FaultNum == 123452.0) || (FaultNum == 123453.0) ||
                       (FaultNum == 123454.0) || (FaultNum == 123455.0));
      }

      QuadModelv_eml_find(Memory_0, tmp_data, &i);
      if (QuadModelv_any(&i)) {
        for (i = 0; i < 8; i++) {
          FaultNum = Memory[i];
          Memory_0[i] = ((FaultNum == 123456.0) || (FaultNum == 123457.0) ||
                         (FaultNum == 123458.0));
        }

        QuadModelv_eml_find(Memory_0, tmp_data, &i);
        if (QuadModelv_any(&i)) {
          for (i = 0; i < 8; i++) {
            FaultNum = Memory[i];
            Memory_0[i] = ((FaultNum == 123544.0) || (FaultNum == 123545.0) ||
                           (FaultNum == 123546.0) || (FaultNum == 123548.0) ||
                           (FaultNum == 123547.0));
          }

          QuadModelv_eml_find(Memory_0, tmp_data, &i);
          if (QuadModelv_any(&i)) {
            // '<S293>:1:687' elseif any(find(FaultIDs==123452|FaultIDs==123453|FaultIDs==123454|FaultIDs==123455))&&any(find(FaultIDs==123456|FaultIDs==123457|FaultIDs==123458))&&any(find(FaultIDs==123544|FaultIDs==123545|FaultIDs==123546|FaultIDs==123548|FaultIDs==123547)) 
            // '<S293>:1:688' for i = 1:2
            // '<S293>:1:689' y(i) = D123452(i);
            rtb_y_i[0] = QuadModelv_B.cell_voltage;

            // '<S293>:1:689' y(i) = D123452(i);
            rtb_y_i[1] = QuadModelv_B.remCapacity;

            // '<S293>:1:691' j = 1;
            // '<S293>:1:692' for i = 3:6
            // '<S293>:1:693' y(i) = D123456(j);
            rtb_y_i[2] = QuadModelv_B.uavlostMass;

            // '<S293>:1:694' j = j + 1;
            // '<S293>:1:693' y(i) = D123456(j);
            rtb_y_i[3] = QuadModelv_B.deltaM[0];

            // '<S293>:1:694' j = j + 1;
            // '<S293>:1:693' y(i) = D123456(j);
            rtb_y_i[4] = QuadModelv_B.deltaM[1];

            // '<S293>:1:694' j = j + 1;
            // '<S293>:1:693' y(i) = D123456(j);
            rtb_y_i[5] = QuadModelv_B.deltaM[2];

            // '<S293>:1:694' j = j + 1;
            // '<S293>:1:696' k = 1;
            // '<S293>:1:697' if any(find(FaultIDs==123544))
            for (i = 0; i < 8; i++) {
              Memory_0[i] = (Memory[i] == 123544.0);
            }

            QuadModelv_eml_find(Memory_0, tmp_data, &i);
            if (QuadModelv_any(&i)) {
              // '<S293>:1:698' for i = 7:9
              // '<S293>:1:699' y(i) = D123544(k);
              rtb_y_i[6] = QuadModelv_B.sf_AccNoiseFun_i.y[0];

              // '<S293>:1:700' k = k + 1;
              // '<S293>:1:699' y(i) = D123544(k);
              rtb_y_i[7] = QuadModelv_B.sf_AccNoiseFun_i.y[1];

              // '<S293>:1:700' k = k + 1;
              // '<S293>:1:699' y(i) = D123544(k);
              rtb_y_i[8] = QuadModelv_B.sf_AccNoiseFun_i.y[2];

              // '<S293>:1:700' k = k + 1;
            } else {
              for (i = 0; i < 8; i++) {
                Memory_0[i] = (Memory[i] == 123545.0);
              }

              QuadModelv_eml_find(Memory_0, tmp_data, &i);
              if (QuadModelv_any(&i)) {
                // '<S293>:1:702' elseif any(find(FaultIDs==123545))
                // '<S293>:1:703' for i = 7:9
                // '<S293>:1:704' y(i) = D123545(k);
                rtb_y_i[6] = QuadModelv_B.sf_AccNoiseFun_a.y[0];

                // '<S293>:1:705' k = k + 1;
                // '<S293>:1:704' y(i) = D123545(k);
                rtb_y_i[7] = QuadModelv_B.sf_AccNoiseFun_a.y[1];

                // '<S293>:1:705' k = k + 1;
                // '<S293>:1:704' y(i) = D123545(k);
                rtb_y_i[8] = QuadModelv_B.sf_AccNoiseFun_a.y[2];

                // '<S293>:1:705' k = k + 1;
              } else {
                for (i = 0; i < 8; i++) {
                  Memory_0[i] = (Memory[i] == 123546.0);
                }

                QuadModelv_eml_find(Memory_0, tmp_data, &i);
                if (QuadModelv_any(&i)) {
                  // '<S293>:1:707' elseif any(find(FaultIDs==123546))
                  // '<S293>:1:708' for i = 7:9
                  // '<S293>:1:709' y(i) = D123546(k);
                  rtb_y_i[6] = QuadModelv_B.sf_AccNoiseFun_p.y[0];

                  // '<S293>:1:710' k = k + 1;
                  // '<S293>:1:709' y(i) = D123546(k);
                  rtb_y_i[7] = QuadModelv_B.sf_AccNoiseFun_p.y[1];

                  // '<S293>:1:710' k = k + 1;
                  // '<S293>:1:709' y(i) = D123546(k);
                  rtb_y_i[8] = QuadModelv_B.sf_AccNoiseFun_p.y[2];

                  // '<S293>:1:710' k = k + 1;
                } else {
                  for (i = 0; i < 8; i++) {
                    Memory_0[i] = (Memory[i] == 123548.0);
                  }

                  QuadModelv_eml_find(Memory_0, tmp_data, &i);
                  if (QuadModelv_any(&i)) {
                    // '<S293>:1:712' elseif any(find(FaultIDs==123548))
                    // '<S293>:1:713' for i = 7:9
                    // '<S293>:1:714' y(i) = D123548(k);
                    rtb_y_i[6] = QuadModelv_B.y_m[0];

                    // '<S293>:1:715' k = k + 1;
                    // '<S293>:1:714' y(i) = D123548(k);
                    rtb_y_i[7] = QuadModelv_B.y_m[1];

                    // '<S293>:1:715' k = k + 1;
                    // '<S293>:1:714' y(i) = D123548(k);
                    rtb_y_i[8] = QuadModelv_B.y_m[2];

                    // '<S293>:1:715' k = k + 1;
                  } else {
                    for (i = 0; i < 8; i++) {
                      Memory_0[i] = (Memory[i] == 123547.0);
                    }

                    QuadModelv_eml_find(Memory_0, tmp_data, &i);
                    if (QuadModelv_any(&i)) {
                      // '<S293>:1:717' elseif any(find(FaultIDs==123547))
                      // '<S293>:1:718' y(7) = D123547;
                      rtb_y_i[6] = rtb_fcn3;
                    }
                  }
                }
              }
            }
          } else {
            guard12 = true;
          }
        } else {
          guard12 = true;
        }
      } else {
        guard12 = true;
      }
    }

    if (guard12) {
      for (i = 0; i < 8; i++) {
        FaultNum = Memory[i];
        Memory_0[i] = ((FaultNum == 123456.0) || (FaultNum == 123457.0) ||
                       (FaultNum == 123458.0));
      }

      QuadModelv_eml_find(Memory_0, tmp_data, &i);
      if (QuadModelv_any(&i)) {
        for (i = 0; i < 8; i++) {
          FaultNum = Memory[i];
          Memory_0[i] = ((FaultNum == 123544.0) || (FaultNum == 123545.0) ||
                         (FaultNum == 123546.0) || (FaultNum == 123548.0) ||
                         (FaultNum == 123547.0));
        }

        QuadModelv_eml_find(Memory_0, tmp_data, &i);
        if (QuadModelv_any(&i)) {
          for (i = 0; i < 8; i++) {
            FaultNum = Memory[i];
            Memory_0[i] = ((FaultNum == 123459.0) || (FaultNum == 123540.0) ||
                           (FaultNum == 123541.0) || (FaultNum == 123542.0) ||
                           (FaultNum == 123543.0));
          }

          QuadModelv_eml_find(Memory_0, tmp_data, &i);
          if (QuadModelv_any(&i)) {
            // '<S293>:1:720' elseif any(find(FaultIDs==123456|FaultIDs==123457|FaultIDs==123458))&&any(find(FaultIDs==123544|FaultIDs==123545|FaultIDs==123546|FaultIDs==123548|FaultIDs==123547))&&any(find(FaultIDs==123459|FaultIDs==123540|FaultIDs==123541|FaultIDs==123542|FaultIDs==123543)) 
            // '<S293>:1:721' for i = 1:4
            // '<S293>:1:722' y(i) = D123456(i);
            rtb_y_i[0] = QuadModelv_B.uavlostMass;

            // '<S293>:1:722' y(i) = D123456(i);
            rtb_y_i[1] = QuadModelv_B.deltaM[0];

            // '<S293>:1:722' y(i) = D123456(i);
            rtb_y_i[2] = QuadModelv_B.deltaM[1];

            // '<S293>:1:722' y(i) = D123456(i);
            rtb_y_i[3] = QuadModelv_B.deltaM[2];

            // '<S293>:1:724' j = 1;
            // '<S293>:1:725' for i = 5:10
            for (r = 0; r < 6; r++) {
              // '<S293>:1:726' y(i) = D123459(j);
              rtb_y_i[r + 4] = ddm[r];

              // '<S293>:1:727' j = j + 1;
            }

            // '<S293>:1:729' k = 1;
            // '<S293>:1:730' if any(find(FaultIDs==123544))
            for (i = 0; i < 8; i++) {
              Memory_0[i] = (Memory[i] == 123544.0);
            }

            QuadModelv_eml_find(Memory_0, tmp_data, &i);
            if (QuadModelv_any(&i)) {
              // '<S293>:1:731' for i = 11:13
              // '<S293>:1:732' y(i) = D123544(k);
              rtb_y_i[10] = QuadModelv_B.sf_AccNoiseFun_i.y[0];

              // '<S293>:1:733' k = k + 1;
              // '<S293>:1:732' y(i) = D123544(k);
              rtb_y_i[11] = QuadModelv_B.sf_AccNoiseFun_i.y[1];

              // '<S293>:1:733' k = k + 1;
              // '<S293>:1:732' y(i) = D123544(k);
              rtb_y_i[12] = QuadModelv_B.sf_AccNoiseFun_i.y[2];

              // '<S293>:1:733' k = k + 1;
            } else {
              for (i = 0; i < 8; i++) {
                Memory_0[i] = (Memory[i] == 123545.0);
              }

              QuadModelv_eml_find(Memory_0, tmp_data, &i);
              if (QuadModelv_any(&i)) {
                // '<S293>:1:735' elseif any(find(FaultIDs==123545))
                // '<S293>:1:736' for i = 11:13
                // '<S293>:1:737' y(i) = D123545(k);
                rtb_y_i[10] = QuadModelv_B.sf_AccNoiseFun_a.y[0];

                // '<S293>:1:738' k = k + 1;
                // '<S293>:1:737' y(i) = D123545(k);
                rtb_y_i[11] = QuadModelv_B.sf_AccNoiseFun_a.y[1];

                // '<S293>:1:738' k = k + 1;
                // '<S293>:1:737' y(i) = D123545(k);
                rtb_y_i[12] = QuadModelv_B.sf_AccNoiseFun_a.y[2];

                // '<S293>:1:738' k = k + 1;
              } else {
                for (i = 0; i < 8; i++) {
                  Memory_0[i] = (Memory[i] == 123546.0);
                }

                QuadModelv_eml_find(Memory_0, tmp_data, &i);
                if (QuadModelv_any(&i)) {
                  // '<S293>:1:740' elseif any(find(FaultIDs==123546))
                  // '<S293>:1:741' for i = 11:13
                  // '<S293>:1:742' y(i) = D123546(k);
                  rtb_y_i[10] = QuadModelv_B.sf_AccNoiseFun_p.y[0];

                  // '<S293>:1:743' k = k + 1;
                  // '<S293>:1:742' y(i) = D123546(k);
                  rtb_y_i[11] = QuadModelv_B.sf_AccNoiseFun_p.y[1];

                  // '<S293>:1:743' k = k + 1;
                  // '<S293>:1:742' y(i) = D123546(k);
                  rtb_y_i[12] = QuadModelv_B.sf_AccNoiseFun_p.y[2];

                  // '<S293>:1:743' k = k + 1;
                } else {
                  for (i = 0; i < 8; i++) {
                    Memory_0[i] = (Memory[i] == 123548.0);
                  }

                  QuadModelv_eml_find(Memory_0, tmp_data, &i);
                  if (QuadModelv_any(&i)) {
                    // '<S293>:1:745' elseif any(find(FaultIDs==123548))
                    // '<S293>:1:746' for i = 11:13
                    // '<S293>:1:747' y(i) = D123548(k);
                    rtb_y_i[10] = QuadModelv_B.y_m[0];

                    // '<S293>:1:748' k = k + 1;
                    // '<S293>:1:747' y(i) = D123548(k);
                    rtb_y_i[11] = QuadModelv_B.y_m[1];

                    // '<S293>:1:748' k = k + 1;
                    // '<S293>:1:747' y(i) = D123548(k);
                    rtb_y_i[12] = QuadModelv_B.y_m[2];

                    // '<S293>:1:748' k = k + 1;
                  } else {
                    for (i = 0; i < 8; i++) {
                      Memory_0[i] = (Memory[i] == 123547.0);
                    }

                    QuadModelv_eml_find(Memory_0, tmp_data, &i);
                    if (QuadModelv_any(&i)) {
                      // '<S293>:1:750' elseif any(find(FaultIDs==123547))
                      // '<S293>:1:751' y(11) = D123547;
                      rtb_y_i[10] = rtb_fcn3;
                    }
                  }
                }
              }
            }
          } else {
            guard11 = true;
          }
        } else {
          guard11 = true;
        }
      } else {
        guard11 = true;
      }
    }

    if (guard11) {
      for (i = 0; i < 8; i++) {
        FaultNum = Memory[i];
        Memory_0[i] = ((FaultNum == 123450.0) || (FaultNum == 123549.0));
      }

      QuadModelv_eml_find(Memory_0, tmp_data, &i);
      if (QuadModelv_any(&i)) {
        for (i = 0; i < 8; i++) {
          FaultNum = Memory[i];
          Memory_0[i] = ((FaultNum == 123456.0) || (FaultNum == 123457.0) ||
                         (FaultNum == 123458.0));
        }

        QuadModelv_eml_find(Memory_0, tmp_data, &i);
        if (QuadModelv_any(&i)) {
          for (i = 0; i < 8; i++) {
            FaultNum = Memory[i];
            Memory_0[i] = ((FaultNum == 123459.0) || (FaultNum == 123540.0) ||
                           (FaultNum == 123541.0) || (FaultNum == 123542.0) ||
                           (FaultNum == 123543.0));
          }

          QuadModelv_eml_find(Memory_0, tmp_data, &i);
          if (QuadModelv_any(&i)) {
            // '<S293>:1:753' elseif any(find(FaultIDs==123450|FaultIDs==123549))&&any(find(FaultIDs==123456|FaultIDs==123457|FaultIDs==123458))&&any(find(FaultIDs==123459|FaultIDs==123540|FaultIDs==123541|FaultIDs==123542|FaultIDs==123543)) 
            // '<S293>:1:754' for i = 1:8
            // '<S293>:1:755' y(i) = D123450(i);
            std::memcpy(&rtb_y_i[0], &QuadModelv_B.y_d[0], sizeof(real_T) << 3U);

            // '<S293>:1:757' j = 1;
            // '<S293>:1:758' for i = 9:12
            // '<S293>:1:759' y(i) = D123456(j);
            rtb_y_i[8] = QuadModelv_B.uavlostMass;

            // '<S293>:1:760' j = j + 1;
            // '<S293>:1:759' y(i) = D123456(j);
            rtb_y_i[9] = QuadModelv_B.deltaM[0];

            // '<S293>:1:760' j = j + 1;
            // '<S293>:1:759' y(i) = D123456(j);
            rtb_y_i[10] = QuadModelv_B.deltaM[1];

            // '<S293>:1:760' j = j + 1;
            // '<S293>:1:759' y(i) = D123456(j);
            rtb_y_i[11] = QuadModelv_B.deltaM[2];

            // '<S293>:1:760' j = j + 1;
            // '<S293>:1:762' k = 1;
            // '<S293>:1:763' for i = 13:18
            for (r = 0; r < 6; r++) {
              // '<S293>:1:764' y(i) = D123459(k);
              rtb_y_i[r + 12] = ddm[r];

              // '<S293>:1:765' k = k + 1;
            }
          } else {
            guard10 = true;
          }
        } else {
          guard10 = true;
        }
      } else {
        guard10 = true;
      }
    }

    if (guard10) {
      for (i = 0; i < 8; i++) {
        FaultNum = Memory[i];
        Memory_0[i] = ((FaultNum == 123450.0) || (FaultNum == 123549.0));
      }

      QuadModelv_eml_find(Memory_0, tmp_data, &i);
      if (QuadModelv_any(&i)) {
        for (i = 0; i < 8; i++) {
          FaultNum = Memory[i];
          Memory_0[i] = ((FaultNum == 123456.0) || (FaultNum == 123457.0) ||
                         (FaultNum == 123458.0));
        }

        QuadModelv_eml_find(Memory_0, tmp_data, &i);
        if (QuadModelv_any(&i)) {
          for (i = 0; i < 8; i++) {
            FaultNum = Memory[i];
            Memory_0[i] = ((FaultNum == 123544.0) || (FaultNum == 123545.0) ||
                           (FaultNum == 123546.0) || (FaultNum == 123548.0) ||
                           (FaultNum == 123547.0));
          }

          QuadModelv_eml_find(Memory_0, tmp_data, &i);
          if (QuadModelv_any(&i)) {
            // '<S293>:1:767' elseif any(find(FaultIDs==123450|FaultIDs==123549))&&any(find(FaultIDs==123456|FaultIDs==123457|FaultIDs==123458))&&any(find(FaultIDs==123544|FaultIDs==123545|FaultIDs==123546|FaultIDs==123548|FaultIDs==123547)) 
            // '<S293>:1:768' for i = 1:8
            // '<S293>:1:771' j = 1;
            // '<S293>:1:772' for i = 9:12
            // '<S293>:1:773' y(i) = D123456(j);
            rtb_y_i[8] = QuadModelv_B.uavlostMass;

            // '<S293>:1:774' j = j + 1;
            // '<S293>:1:773' y(i) = D123456(j);
            rtb_y_i[9] = QuadModelv_B.deltaM[0];

            // '<S293>:1:774' j = j + 1;
            // '<S293>:1:773' y(i) = D123456(j);
            rtb_y_i[10] = QuadModelv_B.deltaM[1];

            // '<S293>:1:774' j = j + 1;
            // '<S293>:1:773' y(i) = D123456(j);
            rtb_y_i[11] = QuadModelv_B.deltaM[2];

            // '<S293>:1:774' j = j + 1;
            // '<S293>:1:776' k = 1;
            // '<S293>:1:777' if any(find(FaultIDs==123544))
            for (r = 0; r < 8; r++) {
              // '<S293>:1:769' y(i) = D123450(i);
              rtb_y_i[r] = QuadModelv_B.y_d[r];
              Memory_0[r] = (Memory[r] == 123544.0);
            }

            QuadModelv_eml_find(Memory_0, tmp_data, &i);
            if (QuadModelv_any(&i)) {
              // '<S293>:1:778' for i = 13:15
              // '<S293>:1:779' y(i) = D123544(k);
              rtb_y_i[12] = QuadModelv_B.sf_AccNoiseFun_i.y[0];

              // '<S293>:1:780' k = k + 1;
              // '<S293>:1:779' y(i) = D123544(k);
              rtb_y_i[13] = QuadModelv_B.sf_AccNoiseFun_i.y[1];

              // '<S293>:1:780' k = k + 1;
              // '<S293>:1:779' y(i) = D123544(k);
              rtb_y_i[14] = QuadModelv_B.sf_AccNoiseFun_i.y[2];

              // '<S293>:1:780' k = k + 1;
            } else {
              for (i = 0; i < 8; i++) {
                Memory_0[i] = (Memory[i] == 123545.0);
              }

              QuadModelv_eml_find(Memory_0, tmp_data, &i);
              if (QuadModelv_any(&i)) {
                // '<S293>:1:782' elseif any(find(FaultIDs==123545))
                // '<S293>:1:783' for i = 13:15
                // '<S293>:1:784' y(i) = D123545(k);
                rtb_y_i[12] = QuadModelv_B.sf_AccNoiseFun_a.y[0];

                // '<S293>:1:785' k = k + 1;
                // '<S293>:1:784' y(i) = D123545(k);
                rtb_y_i[13] = QuadModelv_B.sf_AccNoiseFun_a.y[1];

                // '<S293>:1:785' k = k + 1;
                // '<S293>:1:784' y(i) = D123545(k);
                rtb_y_i[14] = QuadModelv_B.sf_AccNoiseFun_a.y[2];

                // '<S293>:1:785' k = k + 1;
              } else {
                for (i = 0; i < 8; i++) {
                  Memory_0[i] = (Memory[i] == 123546.0);
                }

                QuadModelv_eml_find(Memory_0, tmp_data, &i);
                if (QuadModelv_any(&i)) {
                  // '<S293>:1:787' elseif any(find(FaultIDs==123546))
                  // '<S293>:1:788' for i = 13:15
                  // '<S293>:1:789' y(i) = D123546(k);
                  rtb_y_i[12] = QuadModelv_B.sf_AccNoiseFun_p.y[0];

                  // '<S293>:1:790' k = k + 1;
                  // '<S293>:1:789' y(i) = D123546(k);
                  rtb_y_i[13] = QuadModelv_B.sf_AccNoiseFun_p.y[1];

                  // '<S293>:1:790' k = k + 1;
                  // '<S293>:1:789' y(i) = D123546(k);
                  rtb_y_i[14] = QuadModelv_B.sf_AccNoiseFun_p.y[2];

                  // '<S293>:1:790' k = k + 1;
                } else {
                  for (i = 0; i < 8; i++) {
                    Memory_0[i] = (Memory[i] == 123548.0);
                  }

                  QuadModelv_eml_find(Memory_0, tmp_data, &i);
                  if (QuadModelv_any(&i)) {
                    // '<S293>:1:792' elseif any(find(FaultIDs==123548))
                    // '<S293>:1:793' for i = 13:15
                    // '<S293>:1:794' y(i) = D123548(k);
                    rtb_y_i[12] = QuadModelv_B.y_m[0];

                    // '<S293>:1:795' k = k + 1;
                    // '<S293>:1:794' y(i) = D123548(k);
                    rtb_y_i[13] = QuadModelv_B.y_m[1];

                    // '<S293>:1:795' k = k + 1;
                    // '<S293>:1:794' y(i) = D123548(k);
                    rtb_y_i[14] = QuadModelv_B.y_m[2];

                    // '<S293>:1:795' k = k + 1;
                  } else {
                    for (i = 0; i < 8; i++) {
                      Memory_0[i] = (Memory[i] == 123547.0);
                    }

                    QuadModelv_eml_find(Memory_0, tmp_data, &i);
                    if (QuadModelv_any(&i)) {
                      // '<S293>:1:797' elseif any(find(FaultIDs==123547))
                      // '<S293>:1:798' y(13) = D123547;
                      rtb_y_i[12] = rtb_fcn3;
                    }
                  }
                }
              }
            }
          } else {
            guard9 = true;
          }
        } else {
          guard9 = true;
        }
      } else {
        guard9 = true;
      }
    }

    if (guard9) {
      for (i = 0; i < 8; i++) {
        FaultNum = Memory[i];
        Memory_0[i] = ((FaultNum == 123450.0) || (FaultNum == 123549.0));
      }

      QuadModelv_eml_find(Memory_0, tmp_data, &i);
      if (QuadModelv_any(&i)) {
        for (i = 0; i < 8; i++) {
          FaultNum = Memory[i];
          Memory_0[i] = ((FaultNum == 123459.0) || (FaultNum == 123540.0) ||
                         (FaultNum == 123541.0) || (FaultNum == 123542.0) ||
                         (FaultNum == 123543.0));
        }

        QuadModelv_eml_find(Memory_0, tmp_data, &i);
        if (QuadModelv_any(&i)) {
          for (i = 0; i < 8; i++) {
            FaultNum = Memory[i];
            Memory_0[i] = ((FaultNum == 123544.0) || (FaultNum == 123545.0) ||
                           (FaultNum == 123546.0) || (FaultNum == 123548.0) ||
                           (FaultNum == 123547.0));
          }

          QuadModelv_eml_find(Memory_0, tmp_data, &i);
          if (QuadModelv_any(&i)) {
            // '<S293>:1:800' elseif any(find(FaultIDs==123450|FaultIDs==123549))&&any(find(FaultIDs==123459|FaultIDs==123540|FaultIDs==123541|FaultIDs==123542|FaultIDs==123543))&&any(find(FaultIDs==123544|FaultIDs==123545|FaultIDs==123546|FaultIDs==123548|FaultIDs==123547)) 
            // '<S293>:1:801' for i = 1:8
            // '<S293>:1:802' y(i) = D123450(i);
            std::memcpy(&rtb_y_i[0], &QuadModelv_B.y_d[0], sizeof(real_T) << 3U);

            // '<S293>:1:804' j = 1;
            // '<S293>:1:805' for i = 9:14
            for (r = 0; r < 6; r++) {
              // '<S293>:1:806' y(i) = D123459(j);
              rtb_y_i[r + 8] = ddm[r];

              // '<S293>:1:807' j = j + 1;
            }

            // '<S293>:1:809' k = 1;
            // '<S293>:1:810' if any(find(FaultIDs==123544))
            for (i = 0; i < 8; i++) {
              Memory_0[i] = (Memory[i] == 123544.0);
            }

            QuadModelv_eml_find(Memory_0, tmp_data, &i);
            if (QuadModelv_any(&i)) {
              // '<S293>:1:811' for i = 15:17
              // '<S293>:1:812' y(i) = D123544(k);
              rtb_y_i[14] = QuadModelv_B.sf_AccNoiseFun_i.y[0];

              // '<S293>:1:813' k = k + 1;
              // '<S293>:1:812' y(i) = D123544(k);
              rtb_y_i[15] = QuadModelv_B.sf_AccNoiseFun_i.y[1];

              // '<S293>:1:813' k = k + 1;
              // '<S293>:1:812' y(i) = D123544(k);
              rtb_y_i[16] = QuadModelv_B.sf_AccNoiseFun_i.y[2];

              // '<S293>:1:813' k = k + 1;
            } else {
              for (i = 0; i < 8; i++) {
                Memory_0[i] = (Memory[i] == 123545.0);
              }

              QuadModelv_eml_find(Memory_0, tmp_data, &i);
              if (QuadModelv_any(&i)) {
                // '<S293>:1:815' elseif any(find(FaultIDs==123545))
                // '<S293>:1:816' for i = 15:17
                // '<S293>:1:817' y(i) = D123545(k);
                rtb_y_i[14] = QuadModelv_B.sf_AccNoiseFun_a.y[0];

                // '<S293>:1:818' k = k + 1;
                // '<S293>:1:817' y(i) = D123545(k);
                rtb_y_i[15] = QuadModelv_B.sf_AccNoiseFun_a.y[1];

                // '<S293>:1:818' k = k + 1;
                // '<S293>:1:817' y(i) = D123545(k);
                rtb_y_i[16] = QuadModelv_B.sf_AccNoiseFun_a.y[2];

                // '<S293>:1:818' k = k + 1;
              } else {
                for (i = 0; i < 8; i++) {
                  Memory_0[i] = (Memory[i] == 123546.0);
                }

                QuadModelv_eml_find(Memory_0, tmp_data, &i);
                if (QuadModelv_any(&i)) {
                  // '<S293>:1:820' elseif any(find(FaultIDs==123546))
                  // '<S293>:1:821' for i = 15:17
                  // '<S293>:1:822' y(i) = D123546(k);
                  rtb_y_i[14] = QuadModelv_B.sf_AccNoiseFun_p.y[0];

                  // '<S293>:1:823' k = k + 1;
                  // '<S293>:1:822' y(i) = D123546(k);
                  rtb_y_i[15] = QuadModelv_B.sf_AccNoiseFun_p.y[1];

                  // '<S293>:1:823' k = k + 1;
                  // '<S293>:1:822' y(i) = D123546(k);
                  rtb_y_i[16] = QuadModelv_B.sf_AccNoiseFun_p.y[2];

                  // '<S293>:1:823' k = k + 1;
                } else {
                  for (i = 0; i < 8; i++) {
                    Memory_0[i] = (Memory[i] == 123548.0);
                  }

                  QuadModelv_eml_find(Memory_0, tmp_data, &i);
                  if (QuadModelv_any(&i)) {
                    // '<S293>:1:825' elseif any(find(FaultIDs==123548))
                    // '<S293>:1:826' for i = 15:17
                    // '<S293>:1:827' y(i) = D123548(k);
                    rtb_y_i[14] = QuadModelv_B.y_m[0];

                    // '<S293>:1:828' k = k + 1;
                    // '<S293>:1:827' y(i) = D123548(k);
                    rtb_y_i[15] = QuadModelv_B.y_m[1];

                    // '<S293>:1:828' k = k + 1;
                    // '<S293>:1:827' y(i) = D123548(k);
                    rtb_y_i[16] = QuadModelv_B.y_m[2];

                    // '<S293>:1:828' k = k + 1;
                  } else {
                    for (i = 0; i < 8; i++) {
                      Memory_0[i] = (Memory[i] == 123547.0);
                    }

                    QuadModelv_eml_find(Memory_0, tmp_data, &i);
                    if (QuadModelv_any(&i)) {
                      // '<S293>:1:830' elseif any(find(FaultIDs==123547))
                      // '<S293>:1:831' y(15) = D123547;
                      rtb_y_i[14] = rtb_fcn3;
                    }
                  }
                }
              }
            }
          } else {
            guard8 = true;
          }
        } else {
          guard8 = true;
        }
      } else {
        guard8 = true;
      }
    }

    if (guard8) {
      for (i = 0; i < 8; i++) {
        FaultNum = Memory[i];
        Memory_0[i] = ((FaultNum == 123452.0) || (FaultNum == 123453.0) ||
                       (FaultNum == 123454.0) || (FaultNum == 123455.0));
      }

      QuadModelv_eml_find(Memory_0, tmp_data, &i);
      if (QuadModelv_any(&i)) {
        for (i = 0; i < 8; i++) {
          FaultNum = Memory[i];
          Memory_0[i] = ((FaultNum == 123459.0) || (FaultNum == 123540.0) ||
                         (FaultNum == 123541.0) || (FaultNum == 123542.0) ||
                         (FaultNum == 123543.0));
        }

        QuadModelv_eml_find(Memory_0, tmp_data, &i);
        if (QuadModelv_any(&i)) {
          for (i = 0; i < 8; i++) {
            FaultNum = Memory[i];
            Memory_0[i] = ((FaultNum == 123544.0) || (FaultNum == 123545.0) ||
                           (FaultNum == 123546.0) || (FaultNum == 123548.0) ||
                           (FaultNum == 123547.0));
          }

          QuadModelv_eml_find(Memory_0, tmp_data, &i);
          if (QuadModelv_any(&i)) {
            // '<S293>:1:833' elseif any(find(FaultIDs==123452|FaultIDs==123453|FaultIDs==123454|FaultIDs==123455))&&any(find(FaultIDs==123459|FaultIDs==123540|FaultIDs==123541|FaultIDs==123542|FaultIDs==123543))&&any(find(FaultIDs==123544|FaultIDs==123545|FaultIDs==123546|FaultIDs==123548|FaultIDs==123547)) 
            // '<S293>:1:834' for i = 1:2
            // '<S293>:1:835' y(i) = D123452(i);
            rtb_y_i[0] = QuadModelv_B.cell_voltage;

            // '<S293>:1:835' y(i) = D123452(i);
            rtb_y_i[1] = QuadModelv_B.remCapacity;

            // '<S293>:1:837' j = 1;
            // '<S293>:1:838' for i = 3:8
            for (r = 0; r < 6; r++) {
              // '<S293>:1:839' y(i) = D123459(j);
              rtb_y_i[r + 2] = ddm[r];

              // '<S293>:1:840' j = j + 1;
            }

            // '<S293>:1:842' k = 1;
            // '<S293>:1:843' if any(find(FaultIDs==123544))
            for (i = 0; i < 8; i++) {
              Memory_0[i] = (Memory[i] == 123544.0);
            }

            QuadModelv_eml_find(Memory_0, tmp_data, &i);
            if (QuadModelv_any(&i)) {
              // '<S293>:1:844' for i = 8:10
              // '<S293>:1:845' y(i) = D123544(k);
              rtb_y_i[7] = QuadModelv_B.sf_AccNoiseFun_i.y[0];

              // '<S293>:1:846' k = k + 1;
              // '<S293>:1:845' y(i) = D123544(k);
              rtb_y_i[8] = QuadModelv_B.sf_AccNoiseFun_i.y[1];

              // '<S293>:1:846' k = k + 1;
              // '<S293>:1:845' y(i) = D123544(k);
              rtb_y_i[9] = QuadModelv_B.sf_AccNoiseFun_i.y[2];

              // '<S293>:1:846' k = k + 1;
            } else {
              for (i = 0; i < 8; i++) {
                Memory_0[i] = (Memory[i] == 123545.0);
              }

              QuadModelv_eml_find(Memory_0, tmp_data, &i);
              if (QuadModelv_any(&i)) {
                // '<S293>:1:848' elseif any(find(FaultIDs==123545))
                // '<S293>:1:849' for i = 8:10
                // '<S293>:1:850' y(i) = D123545(k);
                rtb_y_i[7] = QuadModelv_B.sf_AccNoiseFun_a.y[0];

                // '<S293>:1:851' k = k + 1;
                // '<S293>:1:850' y(i) = D123545(k);
                rtb_y_i[8] = QuadModelv_B.sf_AccNoiseFun_a.y[1];

                // '<S293>:1:851' k = k + 1;
                // '<S293>:1:850' y(i) = D123545(k);
                rtb_y_i[9] = QuadModelv_B.sf_AccNoiseFun_a.y[2];

                // '<S293>:1:851' k = k + 1;
              } else {
                for (i = 0; i < 8; i++) {
                  Memory_0[i] = (Memory[i] == 123546.0);
                }

                QuadModelv_eml_find(Memory_0, tmp_data, &i);
                if (QuadModelv_any(&i)) {
                  // '<S293>:1:853' elseif any(find(FaultIDs==123546))
                  // '<S293>:1:854' for i = 8:10
                  // '<S293>:1:855' y(i) = D123546(k);
                  rtb_y_i[7] = QuadModelv_B.sf_AccNoiseFun_p.y[0];

                  // '<S293>:1:856' k = k + 1;
                  // '<S293>:1:855' y(i) = D123546(k);
                  rtb_y_i[8] = QuadModelv_B.sf_AccNoiseFun_p.y[1];

                  // '<S293>:1:856' k = k + 1;
                  // '<S293>:1:855' y(i) = D123546(k);
                  rtb_y_i[9] = QuadModelv_B.sf_AccNoiseFun_p.y[2];

                  // '<S293>:1:856' k = k + 1;
                } else {
                  for (i = 0; i < 8; i++) {
                    Memory_0[i] = (Memory[i] == 123548.0);
                  }

                  QuadModelv_eml_find(Memory_0, tmp_data, &i);
                  if (QuadModelv_any(&i)) {
                    // '<S293>:1:858' elseif any(find(FaultIDs==123548))
                    // '<S293>:1:859' for i = 8:10
                    // '<S293>:1:860' y(i) = D123548(k);
                    rtb_y_i[7] = QuadModelv_B.y_m[0];

                    // '<S293>:1:861' k = k + 1;
                    // '<S293>:1:860' y(i) = D123548(k);
                    rtb_y_i[8] = QuadModelv_B.y_m[1];

                    // '<S293>:1:861' k = k + 1;
                    // '<S293>:1:860' y(i) = D123548(k);
                    rtb_y_i[9] = QuadModelv_B.y_m[2];

                    // '<S293>:1:861' k = k + 1;
                  } else {
                    for (i = 0; i < 8; i++) {
                      Memory_0[i] = (Memory[i] == 123547.0);
                    }

                    QuadModelv_eml_find(Memory_0, tmp_data, &i);
                    if (QuadModelv_any(&i)) {
                      // '<S293>:1:863' elseif any(find(FaultIDs==123547))
                      // '<S293>:1:864' y(8) = D123547;
                      rtb_y_i[7] = rtb_fcn3;
                    }
                  }
                }
              }
            }
          } else {
            guard7 = true;
          }
        } else {
          guard7 = true;
        }
      } else {
        guard7 = true;
      }
    }

    if (guard7) {
      for (i = 0; i < 8; i++) {
        FaultNum = Memory[i];
        Memory_0[i] = ((FaultNum == 123450.0) || (FaultNum == 123549.0));
      }

      QuadModelv_eml_find(Memory_0, tmp_data, &i);
      if (QuadModelv_any(&i)) {
        for (i = 0; i < 8; i++) {
          FaultNum = Memory[i];
          Memory_0[i] = ((FaultNum == 123452.0) || (FaultNum == 123453.0) ||
                         (FaultNum == 123454.0) || (FaultNum == 123455.0));
        }

        QuadModelv_eml_find(Memory_0, tmp_data, &i);
        if (QuadModelv_any(&i)) {
          // '<S293>:1:866' elseif any(find(FaultIDs==123450|FaultIDs==123549))&&any(find(FaultIDs==123452|FaultIDs==123453|FaultIDs==123454|FaultIDs==123455)) 
          // '<S293>:1:867' for i = 1:8
          // '<S293>:1:868' y(i) = D123450(i);
          std::memcpy(&rtb_y_i[0], &QuadModelv_B.y_d[0], sizeof(real_T) << 3U);

          // '<S293>:1:870' j = 1;
          // '<S293>:1:871' for i = 9:10
          // '<S293>:1:872' y(i) = D123452(j);
          rtb_y_i[8] = QuadModelv_B.cell_voltage;

          // '<S293>:1:873' j = j + 1;
          // '<S293>:1:872' y(i) = D123452(j);
          rtb_y_i[9] = QuadModelv_B.remCapacity;

          // '<S293>:1:873' j = j + 1;
        } else {
          guard6 = true;
        }
      } else {
        guard6 = true;
      }
    }

    if (guard6) {
      for (i = 0; i < 8; i++) {
        FaultNum = Memory[i];
        Memory_0[i] = ((FaultNum == 123450.0) || (FaultNum == 123549.0));
      }

      QuadModelv_eml_find(Memory_0, tmp_data, &i);
      if (QuadModelv_any(&i)) {
        for (i = 0; i < 8; i++) {
          FaultNum = Memory[i];
          Memory_0[i] = ((FaultNum == 123456.0) || (FaultNum == 123457.0) ||
                         (FaultNum == 123458.0));
        }

        QuadModelv_eml_find(Memory_0, tmp_data, &i);
        if (QuadModelv_any(&i)) {
          // '<S293>:1:875' elseif any(find(FaultIDs==123450|FaultIDs==123549))&&any(find(FaultIDs==123456|FaultIDs==123457|FaultIDs==123458)) 
          // '<S293>:1:876' for i = 1:8
          // '<S293>:1:877' y(i) = D123450(i);
          std::memcpy(&rtb_y_i[0], &QuadModelv_B.y_d[0], sizeof(real_T) << 3U);

          // '<S293>:1:879' j = 1;
          // '<S293>:1:880' for i = 9:12
          // '<S293>:1:881' y(i) = D123456(j);
          rtb_y_i[8] = QuadModelv_B.uavlostMass;

          // '<S293>:1:882' j = j + 1;
          // '<S293>:1:881' y(i) = D123456(j);
          rtb_y_i[9] = QuadModelv_B.deltaM[0];

          // '<S293>:1:882' j = j + 1;
          // '<S293>:1:881' y(i) = D123456(j);
          rtb_y_i[10] = QuadModelv_B.deltaM[1];

          // '<S293>:1:882' j = j + 1;
          // '<S293>:1:881' y(i) = D123456(j);
          rtb_y_i[11] = QuadModelv_B.deltaM[2];

          // '<S293>:1:882' j = j + 1;
        } else {
          guard5 = true;
        }
      } else {
        guard5 = true;
      }
    }

    if (guard5) {
      for (i = 0; i < 8; i++) {
        FaultNum = Memory[i];
        Memory_0[i] = ((FaultNum == 123450.0) || (FaultNum == 123549.0));
      }

      QuadModelv_eml_find(Memory_0, tmp_data, &i);
      if (QuadModelv_any(&i)) {
        for (i = 0; i < 8; i++) {
          FaultNum = Memory[i];
          Memory_0[i] = ((FaultNum == 123459.0) || (FaultNum == 123540.0) ||
                         (FaultNum == 123541.0) || (FaultNum == 123542.0) ||
                         (FaultNum == 123543.0));
        }

        QuadModelv_eml_find(Memory_0, tmp_data, &i);
        if (QuadModelv_any(&i)) {
          // '<S293>:1:884' elseif any(find(FaultIDs==123450|FaultIDs==123549))&&any(find(FaultIDs==123459|FaultIDs==123540|FaultIDs==123541|FaultIDs==123542|FaultIDs==123543)) 
          // '<S293>:1:885' for i = 1:8
          // '<S293>:1:886' y(i) = D123450(i);
          std::memcpy(&rtb_y_i[0], &QuadModelv_B.y_d[0], sizeof(real_T) << 3U);

          // '<S293>:1:888' j = 1;
          // '<S293>:1:889' for i = 9:14
          for (r = 0; r < 6; r++) {
            // '<S293>:1:890' y(i) = D123459(j);
            rtb_y_i[r + 8] = ddm[r];

            // '<S293>:1:891' j = j + 1;
          }
        } else {
          guard4 = true;
        }
      } else {
        guard4 = true;
      }
    }

    if (guard4) {
      for (i = 0; i < 8; i++) {
        FaultNum = Memory[i];
        Memory_0[i] = ((FaultNum == 123452.0) || (FaultNum == 123453.0) ||
                       (FaultNum == 123454.0) || (FaultNum == 123455.0));
      }

      QuadModelv_eml_find(Memory_0, tmp_data, &i);
      if (QuadModelv_any(&i)) {
        for (i = 0; i < 8; i++) {
          FaultNum = Memory[i];
          Memory_0[i] = ((FaultNum == 123456.0) || (FaultNum == 123457.0) ||
                         (FaultNum == 123458.0));
        }

        QuadModelv_eml_find(Memory_0, tmp_data, &i);
        if (QuadModelv_any(&i)) {
          // '<S293>:1:893' elseif any(find(FaultIDs==123452|FaultIDs==123453|FaultIDs==123454|FaultIDs==123455))&&any(find(FaultIDs==123456|FaultIDs==123457|FaultIDs==123458)) 
          // '<S293>:1:894' for i = 1:2
          // '<S293>:1:895' y(i) = D123452(i);
          rtb_y_i[0] = QuadModelv_B.cell_voltage;

          // '<S293>:1:895' y(i) = D123452(i);
          rtb_y_i[1] = QuadModelv_B.remCapacity;

          // '<S293>:1:897' j = 1;
          // '<S293>:1:898' for i = 3:6
          // '<S293>:1:899' y(i) = D123456(j);
          rtb_y_i[2] = QuadModelv_B.uavlostMass;

          // '<S293>:1:900' j = j + 1;
          // '<S293>:1:899' y(i) = D123456(j);
          rtb_y_i[3] = QuadModelv_B.deltaM[0];

          // '<S293>:1:900' j = j + 1;
          // '<S293>:1:899' y(i) = D123456(j);
          rtb_y_i[4] = QuadModelv_B.deltaM[1];

          // '<S293>:1:900' j = j + 1;
          // '<S293>:1:899' y(i) = D123456(j);
          rtb_y_i[5] = QuadModelv_B.deltaM[2];

          // '<S293>:1:900' j = j + 1;
        } else {
          guard3 = true;
        }
      } else {
        guard3 = true;
      }
    }

    if (guard3) {
      for (i = 0; i < 8; i++) {
        FaultNum = Memory[i];
        Memory_0[i] = ((FaultNum == 123452.0) || (FaultNum == 123453.0) ||
                       (FaultNum == 123454.0) || (FaultNum == 123455.0));
      }

      QuadModelv_eml_find(Memory_0, tmp_data, &i);
      if (QuadModelv_any(&i)) {
        for (i = 0; i < 8; i++) {
          FaultNum = Memory[i];
          Memory_0[i] = ((FaultNum == 123459.0) || (FaultNum == 123540.0) ||
                         (FaultNum == 123541.0) || (FaultNum == 123542.0) ||
                         (FaultNum == 123543.0));
        }

        QuadModelv_eml_find(Memory_0, tmp_data, &i);
        if (QuadModelv_any(&i)) {
          // '<S293>:1:902' elseif any(find(FaultIDs==123452|FaultIDs==123453|FaultIDs==123454|FaultIDs==123455))&&any(find(FaultIDs==123459|FaultIDs==123540|FaultIDs==123541|FaultIDs==123542|FaultIDs==123543)) 
          // '<S293>:1:903' for i = 1:2
          // '<S293>:1:904' y(i) = D123452(i);
          rtb_y_i[0] = QuadModelv_B.cell_voltage;

          // '<S293>:1:904' y(i) = D123452(i);
          rtb_y_i[1] = QuadModelv_B.remCapacity;

          // '<S293>:1:906' j = 1;
          // '<S293>:1:907' for i = 3:8
          for (r = 0; r < 6; r++) {
            // '<S293>:1:908' y(i) = D123459(j);
            rtb_y_i[r + 2] = ddm[r];

            // '<S293>:1:909' j = j + 1;
          }
        } else {
          guard2 = true;
        }
      } else {
        guard2 = true;
      }
    }

    if (guard2) {
      for (i = 0; i < 8; i++) {
        FaultNum = Memory[i];
        Memory_0[i] = ((FaultNum == 123456.0) || (FaultNum == 123457.0) ||
                       (FaultNum == 123458.0));
      }

      QuadModelv_eml_find(Memory_0, tmp_data, &i);
      if (QuadModelv_any(&i)) {
        for (i = 0; i < 8; i++) {
          FaultNum = Memory[i];
          Memory_0[i] = ((FaultNum == 123459.0) || (FaultNum == 123540.0) ||
                         (FaultNum == 123541.0) || (FaultNum == 123542.0) ||
                         (FaultNum == 123543.0));
        }

        QuadModelv_eml_find(Memory_0, tmp_data, &i);
        if (QuadModelv_any(&i)) {
          // '<S293>:1:911' elseif any(find(FaultIDs==123456|FaultIDs==123457|FaultIDs==123458))&&any(find(FaultIDs==123459|FaultIDs==123540|FaultIDs==123541|FaultIDs==123542|FaultIDs==123543)) 
          // '<S293>:1:912' for i = 1:4
          // '<S293>:1:913' y(i) = D123456(i);
          rtb_y_i[0] = QuadModelv_B.uavlostMass;

          // '<S293>:1:913' y(i) = D123456(i);
          rtb_y_i[1] = QuadModelv_B.deltaM[0];

          // '<S293>:1:913' y(i) = D123456(i);
          rtb_y_i[2] = QuadModelv_B.deltaM[1];

          // '<S293>:1:913' y(i) = D123456(i);
          rtb_y_i[3] = QuadModelv_B.deltaM[2];

          // '<S293>:1:915' j = 1;
          // '<S293>:1:916' for i = 5:10
          for (r = 0; r < 6; r++) {
            // '<S293>:1:917' y(i) = D123459(j);
            rtb_y_i[r + 4] = ddm[r];

            // '<S293>:1:918' j = j + 1;
          }
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
    }

    if (guard1) {
      // '<S293>:1:920' else
      // '<S293>:1:921' for i = 1:8
      // '<S293>:1:922' y(i) = D123450(i);
      std::memcpy(&rtb_y_i[0], &QuadModelv_B.y_d[0], sizeof(real_T) << 3U);

      // '<S293>:1:924' j = 1;
      // '<S293>:1:925' for i = 9:10
      // '<S293>:1:926' y(i) = D123452(j);
      rtb_y_i[8] = QuadModelv_B.cell_voltage;

      // '<S293>:1:927' j = j + 1;
      // '<S293>:1:926' y(i) = D123452(j);
      // '<S293>:1:927' j = j + 1;
      // '<S293>:1:929' c = 1;
      // '<S293>:1:930' for i = 10:13
      // '<S293>:1:931' y(i) = D123456(c);
      rtb_y_i[9] = QuadModelv_B.uavlostMass;

      // '<S293>:1:932' c = c + 1;
      // '<S293>:1:931' y(i) = D123456(c);
      rtb_y_i[10] = QuadModelv_B.deltaM[0];

      // '<S293>:1:932' c = c + 1;
      // '<S293>:1:931' y(i) = D123456(c);
      rtb_y_i[11] = QuadModelv_B.deltaM[1];

      // '<S293>:1:932' c = c + 1;
      // '<S293>:1:931' y(i) = D123456(c);
      rtb_y_i[12] = QuadModelv_B.deltaM[2];

      // '<S293>:1:932' c = c + 1;
      // '<S293>:1:934' q = 1;
      // '<S293>:1:935' for i = 14:19
      for (r = 0; r < 6; r++) {
        // '<S293>:1:936' y(i) = D123459(q);
        rtb_y_i[r + 13] = ddm[r];

        // '<S293>:1:937' q = q + 1;
      }

      // '<S293>:1:939' k = 1;
      // '<S293>:1:940' if any(find(FaultIDs==123544))
      for (i = 0; i < 8; i++) {
        Memory_0[i] = (Memory[i] == 123544.0);
      }

      QuadModelv_eml_find(Memory_0, tmp_data, &i);
      if (QuadModelv_any(&i)) {
        // '<S293>:1:941' for i = 20:22
        // '<S293>:1:942' y(i) = D123544(k);
        rtb_y_i[19] = QuadModelv_B.sf_AccNoiseFun_i.y[0];

        // '<S293>:1:943' k = k + 1;
        // '<S293>:1:942' y(i) = D123544(k);
        rtb_y_i[20] = QuadModelv_B.sf_AccNoiseFun_i.y[1];

        // '<S293>:1:943' k = k + 1;
        // '<S293>:1:942' y(i) = D123544(k);
        rtb_y_i[21] = QuadModelv_B.sf_AccNoiseFun_i.y[2];

        // '<S293>:1:943' k = k + 1;
      } else {
        for (i = 0; i < 8; i++) {
          Memory_0[i] = (Memory[i] == 123545.0);
        }

        QuadModelv_eml_find(Memory_0, tmp_data, &i);
        if (QuadModelv_any(&i)) {
          // '<S293>:1:945' elseif any(find(FaultIDs==123545))
          // '<S293>:1:946' for i = 20:22
          // '<S293>:1:947' y(i) = D123545(k);
          rtb_y_i[19] = QuadModelv_B.sf_AccNoiseFun_a.y[0];

          // '<S293>:1:948' k = k + 1;
          // '<S293>:1:947' y(i) = D123545(k);
          rtb_y_i[20] = QuadModelv_B.sf_AccNoiseFun_a.y[1];

          // '<S293>:1:948' k = k + 1;
          // '<S293>:1:947' y(i) = D123545(k);
          rtb_y_i[21] = QuadModelv_B.sf_AccNoiseFun_a.y[2];

          // '<S293>:1:948' k = k + 1;
        } else {
          for (i = 0; i < 8; i++) {
            Memory_0[i] = (Memory[i] == 123546.0);
          }

          QuadModelv_eml_find(Memory_0, tmp_data, &i);
          if (QuadModelv_any(&i)) {
            // '<S293>:1:950' elseif any(find(FaultIDs==123546))
            // '<S293>:1:951' for i = 20:22
            // '<S293>:1:952' y(i) = D123546(k);
            rtb_y_i[19] = QuadModelv_B.sf_AccNoiseFun_p.y[0];

            // '<S293>:1:953' k = k + 1;
            // '<S293>:1:952' y(i) = D123546(k);
            rtb_y_i[20] = QuadModelv_B.sf_AccNoiseFun_p.y[1];

            // '<S293>:1:953' k = k + 1;
            // '<S293>:1:952' y(i) = D123546(k);
            rtb_y_i[21] = QuadModelv_B.sf_AccNoiseFun_p.y[2];

            // '<S293>:1:953' k = k + 1;
          } else {
            for (i = 0; i < 8; i++) {
              Memory_0[i] = (Memory[i] == 123548.0);
            }

            QuadModelv_eml_find(Memory_0, tmp_data, &i);
            if (QuadModelv_any(&i)) {
              // '<S293>:1:955' elseif any(find(FaultIDs==123548))
              // '<S293>:1:956' for i = 20:22
              // '<S293>:1:957' y(i) = D123548(k);
              rtb_y_i[19] = QuadModelv_B.y_m[0];

              // '<S293>:1:958' k = k + 1;
              // '<S293>:1:957' y(i) = D123548(k);
              rtb_y_i[20] = QuadModelv_B.y_m[1];

              // '<S293>:1:958' k = k + 1;
              // '<S293>:1:957' y(i) = D123548(k);
              rtb_y_i[21] = QuadModelv_B.y_m[2];

              // '<S293>:1:958' k = k + 1;
            } else {
              for (i = 0; i < 8; i++) {
                Memory_0[i] = (Memory[i] == 123547.0);
              }

              QuadModelv_eml_find(Memory_0, tmp_data, &i);
              if (QuadModelv_any(&i)) {
                // '<S293>:1:960' elseif any(find(FaultIDs==123547))
                // '<S293>:1:961' y(20) = D123547;
                rtb_y_i[19] = rtb_fcn3;
              }
            }
          }
        }
      }
    }
  }

  // Outport: '<Root>/outCopterData' incorporates:
  //   DataTypeConversion: '<S12>/Data Type Conversion4'
  //   DataTypeConversion: '<S12>/Data Type Conversion9'

  QuadModelv_Y.outCopterData[0] = static_cast<real32_T>(QuadModelv_B.Sum_p[0]);
  QuadModelv_Y.outCopterData[3] = rtb_DataTypeConversion1[0];
  QuadModelv_Y.outCopterData[6] = static_cast<real32_T>(QuadModelv_B.Product[0]);
  QuadModelv_Y.outCopterData[1] = static_cast<real32_T>(QuadModelv_B.Sum_p[1]);
  QuadModelv_Y.outCopterData[4] = rtb_DataTypeConversion1[1];
  QuadModelv_Y.outCopterData[7] = static_cast<real32_T>(QuadModelv_B.Product[1]);
  QuadModelv_Y.outCopterData[2] = static_cast<real32_T>(QuadModelv_B.Sum_p[2]);
  QuadModelv_Y.outCopterData[5] = rtb_DataTypeConversion1[2];
  QuadModelv_Y.outCopterData[8] = static_cast<real32_T>(QuadModelv_B.Product[2]);
  std::memcpy(&QuadModelv_Y.outCopterData[9], &rtb_y_i[0], 23U * sizeof(real_T));

  // Gain: '<S26>/High Gain Quaternion Normalization' incorporates:
  //   Constant: '<S26>/Constant'
  //   DotProduct: '<S26>/Dot Product'
  //   Sum: '<S26>/Sum'

  rtb_Rn = (QuadModelv_P.Constant_Value_h - (((q0_tmp + VectorConcatenate_tmp_0)
              + VectorConcatenate_tmp_1) + VectorConcatenate_tmp)) *
    QuadModelv_P.CustomVariableMass6DOFQuaternio;

  // SignalConversion generated from: '<S15>/q0 q1 q2 q3' incorporates:
  //   Fcn: '<S26>/q0dot'
  //   Fcn: '<S26>/q1dot'
  //   Fcn: '<S26>/q2dot'
  //   Fcn: '<S26>/q3dot'
  //   Integrator: '<S14>/p,q,r '
  //   Integrator: '<S15>/q0 q1 q2 q3'

  QuadModelv_B.TmpSignalConversionAtq0q1q2q3_c[0] = ((QuadModelv_X.pqr_CSTATE[0]
    * QuadModelv_X.q0q1q2q3_CSTATE[1] + QuadModelv_X.pqr_CSTATE[1] *
    QuadModelv_X.q0q1q2q3_CSTATE[2]) + QuadModelv_X.pqr_CSTATE[2] *
    QuadModelv_X.q0q1q2q3_CSTATE[3]) * -0.5 + rtb_Rn *
    QuadModelv_X.q0q1q2q3_CSTATE[0];
  QuadModelv_B.TmpSignalConversionAtq0q1q2q3_c[1] =
    ((QuadModelv_X.q0q1q2q3_CSTATE[0] * QuadModelv_X.pqr_CSTATE[0] +
      QuadModelv_X.q0q1q2q3_CSTATE[2] * QuadModelv_X.pqr_CSTATE[2]) -
     QuadModelv_X.pqr_CSTATE[1] * QuadModelv_X.q0q1q2q3_CSTATE[3]) * 0.5 +
    rtb_Rn * QuadModelv_X.q0q1q2q3_CSTATE[1];
  QuadModelv_B.TmpSignalConversionAtq0q1q2q3_c[2] =
    ((QuadModelv_X.q0q1q2q3_CSTATE[0] * QuadModelv_X.pqr_CSTATE[1] +
      QuadModelv_X.pqr_CSTATE[0] * QuadModelv_X.q0q1q2q3_CSTATE[3]) -
     QuadModelv_X.q0q1q2q3_CSTATE[1] * QuadModelv_X.pqr_CSTATE[2]) * 0.5 +
    rtb_Rn * QuadModelv_X.q0q1q2q3_CSTATE[2];
  QuadModelv_B.TmpSignalConversionAtq0q1q2q3_c[3] =
    ((QuadModelv_X.q0q1q2q3_CSTATE[0] * QuadModelv_X.pqr_CSTATE[2] +
      QuadModelv_X.q0q1q2q3_CSTATE[1] * QuadModelv_X.pqr_CSTATE[1]) -
     QuadModelv_X.pqr_CSTATE[0] * QuadModelv_X.q0q1q2q3_CSTATE[2]) * 0.5 +
    rtb_Rn * QuadModelv_X.q0q1q2q3_CSTATE[3];
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // UnitConversion: '<S111>/Unit Conversion' incorporates:
    //   Constant: '<S94>/Wingspan'

    // Unit Conversion - from: m to: ft
    // Expression: output = (3.28084*input) + (0)
    QuadModelv_B.UnitConversion_e = 3.280839895013123 *
      QuadModelv_P.DrydenWindTurbulenceModelDis_np;
  }

  // Outputs for Enabled SubSystem: '<S108>/Hpgw'
  // Constant: '<S108>/Constant1'
  QuadModelv_Hpgw(QuadModelv_P.DrydenWindTurbulenceModelDisc_k, frac_0,
                  QuadModelv_B.sigma_wg_p, rtb_TmpSignalConversionAtSFun_2,
                  QuadModelv_B.Product_p[3], QuadModelv_B.UnitConversion_e,
                  &QuadModelv_B.Hpgw, &QuadModelv_DW.Hpgw, &QuadModelv_P.Hpgw);

  // End of Outputs for SubSystem: '<S108>/Hpgw'

  // Outputs for Enabled SubSystem: '<S108>/Hqgw'
  // Constant: '<S108>/Constant2'
  QuadModelv_Hqgw(QuadModelv_P.DrydenWindTurbulenceModelDisc_k,
                  QuadModelv_B.UnitConversion_a, QuadModelv_B.Hwgwz.Sum,
                  QuadModelv_B.UnitConversion_e, &QuadModelv_B.Hqgw,
                  &QuadModelv_DW.Hqgw, &QuadModelv_P.Hqgw);

  // End of Outputs for SubSystem: '<S108>/Hqgw'

  // Outputs for Enabled SubSystem: '<S108>/Hrgw'
  // Constant: '<S108>/Constant3'
  QuadModelv_Hrgw(QuadModelv_P.DrydenWindTurbulenceModelDisc_k,
                  QuadModelv_B.UnitConversion_a, QuadModelv_B.Hvgwz.Sum,
                  QuadModelv_B.UnitConversion_e, &QuadModelv_B.Hrgw,
                  &QuadModelv_DW.Hrgw, &QuadModelv_P.Hrgw);

  // End of Outputs for SubSystem: '<S108>/Hrgw'

  // If: '<S113>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' incorporates:
  //   Constant: '<S67>/Constant_DCM'

  if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if (rtb_count <= 1000.0) {
      rtAction = 0;
    } else if (rtb_count >= 2000.0) {
      rtAction = 1;
    } else {
      rtAction = 2;
    }

    QuadModelv_DW.ifHeightMaxlowaltitudeelseifH_j = rtAction;
  } else {
    rtAction = QuadModelv_DW.ifHeightMaxlowaltitudeelseifH_j;
  }

  switch (rtAction) {
   case 0:
    // Outputs for IfAction SubSystem: '<S113>/Low altitude  rates' incorporates:
    //   ActionPort: '<S128>/Action Port'

    QuadModelv_Lowaltituderates(QuadModelv_P.Constant_DCM_Value,
      QuadModelv_B.Hpgw.Sum, QuadModelv_B.Hqgw.Sum1, QuadModelv_B.Hrgw.Sum1,
      QuadModelv_B.UnitConversion_ab, rtb_Merge);

    // End of Outputs for SubSystem: '<S113>/Low altitude  rates'
    break;

   case 1:
    break;

   case 2:
    // Outputs for IfAction SubSystem: '<S113>/Interpolate  rates' incorporates:
    //   ActionPort: '<S127>/Action Port'

    QuadModelv_Interpolaterates(QuadModelv_B.Hpgw.Sum, QuadModelv_B.Hqgw.Sum1,
      QuadModelv_B.Hrgw.Sum1, QuadModelv_P.Constant_DCM_Value,
      QuadModelv_B.UnitConversion_ab, rtb_count, rtb_Merge,
      &QuadModelv_P.Interpolaterates);

    // End of Outputs for SubSystem: '<S113>/Interpolate  rates'
    break;
  }

  // End of If: '<S113>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' 
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // UnitConversion: '<S150>/Unit Conversion' incorporates:
    //   Constant: '<S95>/Wingspan'

    // Unit Conversion - from: m to: ft
    // Expression: output = (3.28084*input) + (0)
    QuadModelv_B.UnitConversion_f = 3.280839895013123 *
      QuadModelv_P.DrydenWindTurbulenceModelDisc_d;
  }

  // Outputs for Enabled SubSystem: '<S147>/Hpgw'
  // Constant: '<S147>/Constant1'
  QuadModelv_Hpgw(QuadModelv_P.DrydenWindTurbulenceModelDisc_g, rtb_Sum_d,
                  QuadModelv_B.sigma_wg, rtb_Add_br, QuadModelv_B.Product_d[3],
                  QuadModelv_B.UnitConversion_f, &QuadModelv_B.Hpgw_h,
                  &QuadModelv_DW.Hpgw_h, &QuadModelv_P.Hpgw_h);

  // End of Outputs for SubSystem: '<S147>/Hpgw'

  // Outputs for Enabled SubSystem: '<S147>/Hqgw'
  // Constant: '<S147>/Constant2'
  QuadModelv_Hqgw(QuadModelv_P.DrydenWindTurbulenceModelDisc_g,
                  QuadModelv_B.UnitConversion, QuadModelv_B.Hwgwz_a.Sum,
                  QuadModelv_B.UnitConversion_f, &QuadModelv_B.Hqgw_a,
                  &QuadModelv_DW.Hqgw_a, &QuadModelv_P.Hqgw_a);

  // End of Outputs for SubSystem: '<S147>/Hqgw'

  // Outputs for Enabled SubSystem: '<S147>/Hrgw'
  // Constant: '<S147>/Constant3'
  QuadModelv_Hrgw(QuadModelv_P.DrydenWindTurbulenceModelDisc_g,
                  QuadModelv_B.UnitConversion, QuadModelv_B.Hvgwz_f.Sum,
                  QuadModelv_B.UnitConversion_f, &QuadModelv_B.Hrgw_a,
                  &QuadModelv_DW.Hrgw_a, &QuadModelv_P.Hrgw_a);

  // End of Outputs for SubSystem: '<S147>/Hrgw'

  // If: '<S152>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' incorporates:
  //   Constant: '<S67>/Constant_DCM'
  //   UnitConversion: '<S149>/Unit Conversion'

  if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if (rtb_count <= 1000.0) {
      rtAction = 0;
    } else if (rtb_count >= 2000.0) {
      rtAction = 1;
    } else {
      rtAction = 2;
    }

    QuadModelv_DW.ifHeightMaxlowaltitudeelseifH_e = rtAction;
  } else {
    rtAction = QuadModelv_DW.ifHeightMaxlowaltitudeelseifH_e;
  }

  switch (rtAction) {
   case 0:
    // Outputs for IfAction SubSystem: '<S152>/Low altitude  rates' incorporates:
    //   ActionPort: '<S167>/Action Port'

    QuadModelv_Lowaltituderates(QuadModelv_P.Constant_DCM_Value,
      QuadModelv_B.Hpgw_h.Sum, QuadModelv_B.Hqgw_a.Sum1,
      QuadModelv_B.Hrgw_a.Sum1, QuadModelv_B.UnitConversion_p, rtb_Merge);

    // End of Outputs for SubSystem: '<S152>/Low altitude  rates'
    break;

   case 1:
    break;

   case 2:
    // Outputs for IfAction SubSystem: '<S152>/Interpolate  rates' incorporates:
    //   ActionPort: '<S166>/Action Port'

    QuadModelv_Interpolaterates(QuadModelv_B.Hpgw_h.Sum,
      QuadModelv_B.Hqgw_a.Sum1, QuadModelv_B.Hrgw_a.Sum1,
      QuadModelv_P.Constant_DCM_Value, QuadModelv_B.UnitConversion_p, rtb_count,
      rtb_Merge, &QuadModelv_P.Interpolaterates_g);

    // End of Outputs for SubSystem: '<S152>/Interpolate  rates'
    break;
  }

  // End of If: '<S152>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' 
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // If: '<S203>/If1' incorporates:
    //   Constant: '<S203>/Constant'

    rtAction = -1;
    if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
      if (QuadModelv_P.DirectionCosineMatrixtoQuaterni != 1.0) {
        rtAction = 0;
      }

      QuadModelv_DW.If1_ActiveSubsystem = rtAction;
    } else {
      rtAction = QuadModelv_DW.If1_ActiveSubsystem;
    }

    if (rtAction == 0) {
      // Outputs for IfAction SubSystem: '<S203>/If Warning//Error' incorporates:
      //   ActionPort: '<S227>/if'

      QuadModelv_IfWarningError(VectorConcatenate,
        QuadModelv_P.DirectionCosineMatrixtoQuaterni,
        QuadModelv_P.DirectionCosineMatrixtoQuater_i,
        &QuadModelv_P.IfWarningError);

      // End of Outputs for SubSystem: '<S203>/If Warning//Error'
    }

    // End of If: '<S203>/If1'

    // If: '<S241>/If1' incorporates:
    //   Constant: '<S241>/Constant'

    rtAction = -1;
    if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
      if (QuadModelv_P.DirectionCosineMatrixtoQuater_p != 1.0) {
        rtAction = 0;
      }

      QuadModelv_DW.If1_ActiveSubsystem_o = rtAction;
    } else {
      rtAction = QuadModelv_DW.If1_ActiveSubsystem_o;
    }

    if (rtAction == 0) {
      // Outputs for IfAction SubSystem: '<S241>/If Warning//Error' incorporates:
      //   ActionPort: '<S265>/if'

      QuadModelv_IfWarningError(VectorConcatenate_p,
        QuadModelv_P.DirectionCosineMatrixtoQuater_p,
        QuadModelv_P.DirectionCosineMatrixtoQuate_i5,
        &QuadModelv_P.IfWarningError_f);

      // End of Outputs for SubSystem: '<S241>/If Warning//Error'
    }

    // End of If: '<S241>/If1'

    // If: '<S337>/If1' incorporates:
    //   Constant: '<S337>/Constant'

    rtAction = -1;
    if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
      if (QuadModelv_P.DirectionCosineMatrixtoQuater_c != 1.0) {
        rtAction = 0;
      }

      QuadModelv_DW.If1_ActiveSubsystem_d = rtAction;
    } else {
      rtAction = QuadModelv_DW.If1_ActiveSubsystem_d;
    }

    if (rtAction == 0) {
      // Outputs for IfAction SubSystem: '<S337>/If Warning//Error' incorporates:
      //   ActionPort: '<S361>/if'

      QuadModelv_IfWarningError(VectorConcatenate,
        QuadModelv_P.DirectionCosineMatrixtoQuater_c,
        QuadModelv_P.DirectionCosineMatrixtoQuater_b,
        &QuadModelv_P.IfWarningError_a);

      // End of Outputs for SubSystem: '<S337>/If Warning//Error'
    }

    // End of If: '<S337>/If1'
  }

  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[5] == 0) {
    // Gain: '<S330>/BiasGain1' incorporates:
    //   UniformRandomNumber: '<S330>/Uniform Random Number4'

    QuadModelv_B.BiasGain1[0] = QuadModelv_P.BiasGain1_Gain *
      QuadModelv_DW.UniformRandomNumber4_NextOutp_p[0];
    QuadModelv_B.BiasGain1[1] = QuadModelv_P.BiasGain1_Gain *
      QuadModelv_DW.UniformRandomNumber4_NextOutp_p[1];
    QuadModelv_B.BiasGain1[2] = QuadModelv_P.BiasGain1_Gain *
      QuadModelv_DW.UniformRandomNumber4_NextOutp_p[2];
  }

  // Gain: '<S433>/2*zeta * wn'
  rtb_Product3_o = 2.0 * QuadModelv_P.FaultParamAPI.FaultInParams[9] *
    QuadModelv_P.ThreeaxisInertialMeasurement_c1;

  // Gain: '<S433>/wn^2'
  FaultNum = QuadModelv_P.ThreeaxisInertialMeasurement_c1 *
    QuadModelv_P.ThreeaxisInertialMeasurement_c1;

  // Sum: '<S433>/Sum2' incorporates:
  //   Gain: '<S433>/2*zeta * wn'
  //   Gain: '<S433>/wn^2'
  //   SecondOrderIntegrator: '<S433>/Integrator, Second-Order Limited'
  //   Sum: '<S433>/Sum3'

  QuadModelv_B.Sum2[0] = (rtb_Sum4_a[0] -
    QuadModelv_X.IntegratorSecondOrderLimited_CS[0]) * FaultNum - rtb_Product3_o
    * QuadModelv_X.IntegratorSecondOrderLimited_CS[3];
  QuadModelv_B.Sum2[1] = (rtb_Sum4_a[1] -
    QuadModelv_X.IntegratorSecondOrderLimited_CS[1]) * FaultNum - rtb_Product3_o
    * QuadModelv_X.IntegratorSecondOrderLimited_CS[4];
  QuadModelv_B.Sum2[2] = (rtb_Sum4_a[2] -
    QuadModelv_X.IntegratorSecondOrderLimited_CS[2]) * FaultNum - rtb_Product3_o
    * QuadModelv_X.IntegratorSecondOrderLimited_CS[5];

  // Gain: '<S444>/2*zeta * wn'
  rtb_Product3_o = 2.0 * QuadModelv_P.FaultParamAPI.FaultInParams[13] *
    QuadModelv_P.ThreeaxisInertialMeasurementU_i;

  // Gain: '<S444>/wn^2'
  FaultNum = QuadModelv_P.ThreeaxisInertialMeasurementU_i *
    QuadModelv_P.ThreeaxisInertialMeasurementU_i;

  // Sum: '<S444>/Sum2' incorporates:
  //   Gain: '<S444>/2*zeta * wn'
  //   Gain: '<S444>/wn^2'
  //   SecondOrderIntegrator: '<S444>/Integrator, Second-Order Limited'
  //   Sum: '<S444>/Sum3'

  QuadModelv_B.Sum2_f[0] = (rtb_Mb[0] -
    QuadModelv_X.IntegratorSecondOrderLimited__i[0]) * FaultNum - rtb_Product3_o
    * QuadModelv_X.IntegratorSecondOrderLimited__i[3];
  QuadModelv_B.Sum2_f[1] = (rtb_Mb[1] -
    QuadModelv_X.IntegratorSecondOrderLimited__i[1]) * FaultNum - rtb_Product3_o
    * QuadModelv_X.IntegratorSecondOrderLimited__i[4];
  QuadModelv_B.Sum2_f[2] = (rtb_Mb[2] -
    QuadModelv_X.IntegratorSecondOrderLimited__i[2]) * FaultNum - rtb_Product3_o
    * QuadModelv_X.IntegratorSecondOrderLimited__i[5];
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Assertion: '<S445>/Assertion' incorporates:
    //   Constant: '<S445>/max_val'
    //   Constant: '<S445>/min_val'
    //   Logic: '<S445>/conjunction'
    //   RelationalOperator: '<S445>/max_relop'
    //   RelationalOperator: '<S445>/min_relop'

    utAssert((QuadModelv_P.CheckAltitude_min <= rtb_Saturation_1) &&
             (rtb_Saturation_1 <= QuadModelv_P.CheckAltitude_max));

    // Assertion: '<S446>/Assertion' incorporates:
    //   Constant: '<S446>/max_val'
    //   Constant: '<S446>/min_val'
    //   Logic: '<S446>/conjunction'
    //   RelationalOperator: '<S446>/max_relop'
    //   RelationalOperator: '<S446>/min_relop'

    utAssert((QuadModelv_P.CheckLatitude_min <= LoadLeakRatio) && (LoadLeakRatio
              <= QuadModelv_P.CheckLatitude_max));

    // Assertion: '<S447>/Assertion' incorporates:
    //   Constant: '<S447>/max_val'
    //   Constant: '<S447>/min_val'
    //   Logic: '<S447>/conjunction'
    //   RelationalOperator: '<S447>/max_relop'
    //   RelationalOperator: '<S447>/min_relop'

    utAssert((QuadModelv_P.CheckLongitude_min <= Y) && (Y <=
              QuadModelv_P.CheckLongitude_max));

    // Assertion: '<S449>/Assertion' incorporates:
    //   Constant: '<S449>/max_val'
    //   Constant: '<S449>/min_val'
    //   Logic: '<S449>/conjunction'
    //   RelationalOperator: '<S449>/max_relop'
    //   RelationalOperator: '<S449>/min_relop'

    utAssert((QuadModelv_P.Istimewithinmodellimits_min <= rtb_Sum_o1) &&
             (rtb_Sum_o1 <= QuadModelv_P.Istimewithinmodellimits_max));

    // MATLAB Function 'OutputPort1/HILSensorMavModel1/Subsystem/AccelNoiseGainFunction': '<S404>:1' 
    // '<S404>:1:3' if theta>0.1
    // MATLAB Function 'OutputPort1/HILSensorMavModel1/Subsystem/GyroNoiseGainFunction': '<S405>:1' 
    // '<S405>:1:3' if theta>0.1
    // MATLAB Function 'OutputPort1/HILSensorMavModel1/Subsystem/MagNoiseGainFunction': '<S406>:1' 
    // '<S406>:1:3' if theta>0.1
  }

  // Outport: '<Root>/PosGPS' incorporates:
  //   SignalConversion generated from: '<Root>/MavVehile3DInfo'

  QuadModelv_Y.PosGPS[0] = LoadLeakRatio;
  QuadModelv_Y.PosGPS[1] = Y;
  QuadModelv_Y.PosGPS[2] = rtb_Saturation_1;
  if (rtmIsMajorTimeStep((&QuadModelv_M))) {
    // Update for Integrator: '<S15>/q0 q1 q2 q3'
    QuadModelv_DW.q0q1q2q3_IWORK = 0;
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
      // Update for Memory: '<Root>/Memory' incorporates:
      //   Inport: '<Root>/inPWMs'

      std::memcpy(&QuadModelv_DW.Memory_PreviousInput[0], &QuadModelv_U.inPWMs[0],
                  sizeof(real_T) << 3U);
    }

    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[4] == 0) {
      // Update for RandomNumber: '<S297>/White Noise'
      QuadModelv_DW.NextOutput = rt_nrand_Upu32_Yd_f_pw_snf
        (&QuadModelv_DW.RandSeed) * QuadModelv_P.WhiteNoise_StdDev +
        QuadModelv_P.WhiteNoise_Mean;
    }

    // Update for Atomic SubSystem: '<S296>/MotorNonlinearDynamic1'
    Q_MotorNonlinearDynamic1_Update(&QuadModelv_DW.MotorNonlinearDynamic1);

    // End of Update for SubSystem: '<S296>/MotorNonlinearDynamic1'

    // Update for Atomic SubSystem: '<S296>/MotorNonlinearDynamic2'
    Q_MotorNonlinearDynamic1_Update(&QuadModelv_DW.MotorNonlinearDynamic2);

    // End of Update for SubSystem: '<S296>/MotorNonlinearDynamic2'

    // Update for Atomic SubSystem: '<S296>/MotorNonlinearDynamic3'
    Q_MotorNonlinearDynamic1_Update(&QuadModelv_DW.MotorNonlinearDynamic3);

    // End of Update for SubSystem: '<S296>/MotorNonlinearDynamic3'

    // Update for Atomic SubSystem: '<S296>/MotorNonlinearDynamic4'
    Q_MotorNonlinearDynamic1_Update(&QuadModelv_DW.MotorNonlinearDynamic4);

    // End of Update for SubSystem: '<S296>/MotorNonlinearDynamic4'

    // Update for Atomic SubSystem: '<S296>/MotorNonlinearDynamic5'
    Q_MotorNonlinearDynamic1_Update(&QuadModelv_DW.MotorNonlinearDynamic5);

    // End of Update for SubSystem: '<S296>/MotorNonlinearDynamic5'

    // Update for Atomic SubSystem: '<S296>/MotorNonlinearDynamic6'
    Q_MotorNonlinearDynamic1_Update(&QuadModelv_DW.MotorNonlinearDynamic6);

    // End of Update for SubSystem: '<S296>/MotorNonlinearDynamic6'

    // Update for Atomic SubSystem: '<S296>/MotorNonlinearDynamic7'
    Q_MotorNonlinearDynamic1_Update(&QuadModelv_DW.MotorNonlinearDynamic7);

    // End of Update for SubSystem: '<S296>/MotorNonlinearDynamic7'

    // Update for Atomic SubSystem: '<S296>/MotorNonlinearDynamic8'
    Q_MotorNonlinearDynamic1_Update(&QuadModelv_DW.MotorNonlinearDynamic8);

    // End of Update for SubSystem: '<S296>/MotorNonlinearDynamic8'
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[4] == 0) {
      // Update for RandomNumber: '<S523>/White Noise'
      QuadModelv_DW.NextOutput_g = rt_nrand_Upu32_Yd_f_pw_snf
        (&QuadModelv_DW.RandSeed_h) * QuadModelv_P.WhiteNoise_StdDev_b +
        QuadModelv_P.WhiteNoise_Mean_n;
    }

    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[2] == 0) {
      // Update for UniformRandomNumber: '<S67>/Uniform Random Number'
      QuadModelv_DW.UniformRandomNumber_NextOutput[0] =
        (QuadModelv_P.UniformRandomNumber_Maximum[0] -
         QuadModelv_P.UniformRandomNumber_Minimum[0]) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_o[0]) +
        QuadModelv_P.UniformRandomNumber_Minimum[0];
      QuadModelv_DW.UniformRandomNumber_NextOutput[1] =
        (QuadModelv_P.UniformRandomNumber_Maximum[1] -
         QuadModelv_P.UniformRandomNumber_Minimum[1]) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_o[1]) +
        QuadModelv_P.UniformRandomNumber_Minimum[1];
      QuadModelv_DW.UniformRandomNumber_NextOutput[2] =
        (QuadModelv_P.UniformRandomNumber_Maximum[2] -
         QuadModelv_P.UniformRandomNumber_Minimum[2]) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_o[2]) +
        QuadModelv_P.UniformRandomNumber_Minimum[2];
    }

    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
      // Update for RandomNumber: '<S157>/White Noise'
      QuadModelv_DW.NextOutput_c[0] = rt_nrand_Upu32_Yd_f_pw_snf
        (&QuadModelv_DW.RandSeed_p[0]) * QuadModelv_P.WhiteNoise_StdDev_j +
        QuadModelv_P.WhiteNoise_Mean_j;
      QuadModelv_DW.NextOutput_c[1] = rt_nrand_Upu32_Yd_f_pw_snf
        (&QuadModelv_DW.RandSeed_p[1]) * QuadModelv_P.WhiteNoise_StdDev_j +
        QuadModelv_P.WhiteNoise_Mean_j;
      QuadModelv_DW.NextOutput_c[2] = rt_nrand_Upu32_Yd_f_pw_snf
        (&QuadModelv_DW.RandSeed_p[2]) * QuadModelv_P.WhiteNoise_StdDev_j +
        QuadModelv_P.WhiteNoise_Mean_j;
      QuadModelv_DW.NextOutput_c[3] = rt_nrand_Upu32_Yd_f_pw_snf
        (&QuadModelv_DW.RandSeed_p[3]) * QuadModelv_P.WhiteNoise_StdDev_j +
        QuadModelv_P.WhiteNoise_Mean_j;
    }

    // Update for Enabled SubSystem: '<S148>/Hugw(z)'
    QuadModelv_Hugwz_Update(&QuadModelv_B.Hugwz_g, &QuadModelv_DW.Hugwz_g);

    // End of Update for SubSystem: '<S148>/Hugw(z)'

    // Update for Enabled SubSystem: '<S148>/Hvgw(z)'
    QuadModelv_Hvgwz_Update(&QuadModelv_B.Hvgwz_f, &QuadModelv_DW.Hvgwz_f);

    // End of Update for SubSystem: '<S148>/Hvgw(z)'

    // Update for Enabled SubSystem: '<S148>/Hwgw(z)'
    QuadModelv_Hwgwz_Update(&QuadModelv_B.Hwgwz_a, &QuadModelv_DW.Hwgwz_a);

    // End of Update for SubSystem: '<S148>/Hwgw(z)'
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
      // Update for RandomNumber: '<S118>/White Noise'
      QuadModelv_DW.NextOutput_p[0] = rt_nrand_Upu32_Yd_f_pw_snf
        (&QuadModelv_DW.RandSeed_or[0]) * QuadModelv_P.WhiteNoise_StdDev_k +
        QuadModelv_P.WhiteNoise_Mean_m;
      QuadModelv_DW.NextOutput_p[1] = rt_nrand_Upu32_Yd_f_pw_snf
        (&QuadModelv_DW.RandSeed_or[1]) * QuadModelv_P.WhiteNoise_StdDev_k +
        QuadModelv_P.WhiteNoise_Mean_m;
      QuadModelv_DW.NextOutput_p[2] = rt_nrand_Upu32_Yd_f_pw_snf
        (&QuadModelv_DW.RandSeed_or[2]) * QuadModelv_P.WhiteNoise_StdDev_k +
        QuadModelv_P.WhiteNoise_Mean_m;
      QuadModelv_DW.NextOutput_p[3] = rt_nrand_Upu32_Yd_f_pw_snf
        (&QuadModelv_DW.RandSeed_or[3]) * QuadModelv_P.WhiteNoise_StdDev_k +
        QuadModelv_P.WhiteNoise_Mean_m;
    }

    // Update for Enabled SubSystem: '<S109>/Hugw(z)'
    QuadModelv_Hugwz_Update(&QuadModelv_B.Hugwz, &QuadModelv_DW.Hugwz);

    // End of Update for SubSystem: '<S109>/Hugw(z)'

    // Update for Enabled SubSystem: '<S109>/Hvgw(z)'
    QuadModelv_Hvgwz_Update(&QuadModelv_B.Hvgwz, &QuadModelv_DW.Hvgwz);

    // End of Update for SubSystem: '<S109>/Hvgw(z)'

    // Update for Enabled SubSystem: '<S109>/Hwgw(z)'
    QuadModelv_Hwgwz_Update(&QuadModelv_B.Hwgwz, &QuadModelv_DW.Hwgwz);

    // End of Update for SubSystem: '<S109>/Hwgw(z)'
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[2] == 0) {
      // Update for UniformRandomNumber: '<S407>/Uniform Random Number5'
      QuadModelv_DW.UniformRandomNumber5_NextOutput[0] =
        (QuadModelv_P.UniformRandomNumber5_Maximum[0] -
         QuadModelv_P.UniformRandomNumber5_Minimum[0]) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_d[0]) +
        QuadModelv_P.UniformRandomNumber5_Minimum[0];

      // Update for UniformRandomNumber: '<S407>/Uniform Random Number1'
      QuadModelv_DW.UniformRandomNumber1_NextOutput[0] =
        (QuadModelv_P.UniformRandomNumber1_Maximum[0] -
         QuadModelv_P.UniformRandomNumber1_Minimum[0]) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_m[0]) +
        QuadModelv_P.UniformRandomNumber1_Minimum[0];

      // Update for UniformRandomNumber: '<S407>/Uniform Random Number5'
      QuadModelv_DW.UniformRandomNumber5_NextOutput[1] =
        (QuadModelv_P.UniformRandomNumber5_Maximum[1] -
         QuadModelv_P.UniformRandomNumber5_Minimum[1]) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_d[1]) +
        QuadModelv_P.UniformRandomNumber5_Minimum[1];

      // Update for UniformRandomNumber: '<S407>/Uniform Random Number1'
      QuadModelv_DW.UniformRandomNumber1_NextOutput[1] =
        (QuadModelv_P.UniformRandomNumber1_Maximum[1] -
         QuadModelv_P.UniformRandomNumber1_Minimum[1]) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_m[1]) +
        QuadModelv_P.UniformRandomNumber1_Minimum[1];

      // Update for UniformRandomNumber: '<S407>/Uniform Random Number5'
      QuadModelv_DW.UniformRandomNumber5_NextOutput[2] =
        (QuadModelv_P.UniformRandomNumber5_Maximum[2] -
         QuadModelv_P.UniformRandomNumber5_Minimum[2]) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_d[2]) +
        QuadModelv_P.UniformRandomNumber5_Minimum[2];

      // Update for UniformRandomNumber: '<S407>/Uniform Random Number1'
      QuadModelv_DW.UniformRandomNumber1_NextOutput[2] =
        (QuadModelv_P.UniformRandomNumber1_Maximum[2] -
         QuadModelv_P.UniformRandomNumber1_Minimum[2]) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_m[2]) +
        QuadModelv_P.UniformRandomNumber1_Minimum[2];

      // Update for UniformRandomNumber: '<S407>/Uniform Random Number7'
      QuadModelv_DW.UniformRandomNumber7_NextOutput[0] =
        (QuadModelv_P.UniformRandomNumber7_Maximum[0] -
         QuadModelv_P.UniformRandomNumber7_Minimum[0]) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_e[0]) +
        QuadModelv_P.UniformRandomNumber7_Minimum[0];
      QuadModelv_DW.UniformRandomNumber7_NextOutput[1] =
        (QuadModelv_P.UniformRandomNumber7_Maximum[1] -
         QuadModelv_P.UniformRandomNumber7_Minimum[1]) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_e[1]) +
        QuadModelv_P.UniformRandomNumber7_Minimum[1];
      QuadModelv_DW.UniformRandomNumber7_NextOutput[2] =
        (QuadModelv_P.UniformRandomNumber7_Maximum[2] -
         QuadModelv_P.UniformRandomNumber7_Minimum[2]) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_e[2]) +
        QuadModelv_P.UniformRandomNumber7_Minimum[2];
    }

    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
      // Update for Memory: '<S468>/otime' incorporates:
      //   Constant: '<S420>/Decimal Year'

      QuadModelv_DW.otime_PreviousInput = QuadModelv_P.DecimalYear_Value;

      // Update for Memory: '<S467>/olon'
      QuadModelv_DW.olon_PreviousInput = QuadModelv_B.Switch_p;

      // Update for Memory: '<S466>/olat'
      QuadModelv_DW.olat_PreviousInput = QuadModelv_B.Switch_l;

      // Update for Memory: '<S466>/oalt'
      QuadModelv_DW.oalt_PreviousInput = QuadModelv_B.Gain_f0;
    }

    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
      // Update for UniformRandomNumber: '<S407>/Uniform Random Number'
      QuadModelv_DW.UniformRandomNumber_NextOutpu_l =
        (QuadModelv_P.UniformRandomNumber_Maximum_m -
         QuadModelv_P.UniformRandomNumber_Minimum_d) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_ei) +
        QuadModelv_P.UniformRandomNumber_Minimum_d;

      // Update for UniformRandomNumber: '<S407>/Uniform Random Number4'
      QuadModelv_DW.UniformRandomNumber4_NextOutput =
        (QuadModelv_P.UniformRandomNumber4_Maximum -
         QuadModelv_P.UniformRandomNumber4_Minimum) * rt_urand_Upu32_Yd_f_pw_snf
        (&QuadModelv_DW.RandSeed_b) + QuadModelv_P.UniformRandomNumber4_Minimum;
    }

    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[5] == 0) {
      // Update for UniformRandomNumber: '<S374>/Uniform Random Number5'
      QuadModelv_DW.UniformRandomNumber5_NextOutp_m[0] =
        (QuadModelv_P.UniformRandomNumber5_Maximum_o[0] -
         QuadModelv_P.UniformRandomNumber5_Minimum_e[0]) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_he[0]) +
        QuadModelv_P.UniformRandomNumber5_Minimum_e[0];
      QuadModelv_DW.UniformRandomNumber5_NextOutp_m[1] =
        (QuadModelv_P.UniformRandomNumber5_Maximum_o[1] -
         QuadModelv_P.UniformRandomNumber5_Minimum_e[1]) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_he[1]) +
        QuadModelv_P.UniformRandomNumber5_Minimum_e[1];
      QuadModelv_DW.UniformRandomNumber5_NextOutp_m[2] =
        (QuadModelv_P.UniformRandomNumber5_Maximum_o[2] -
         QuadModelv_P.UniformRandomNumber5_Minimum_e[2]) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_he[2]) +
        QuadModelv_P.UniformRandomNumber5_Minimum_e[2];
    }

    // Update for Enabled SubSystem: '<S108>/Hpgw'
    QuadModelv_Hpgw_Update(&QuadModelv_B.Hpgw, &QuadModelv_DW.Hpgw);

    // End of Update for SubSystem: '<S108>/Hpgw'

    // Update for Enabled SubSystem: '<S108>/Hqgw'
    QuadModelv_Hqgw_Update(QuadModelv_B.Hwgwz.Sum, &QuadModelv_B.Hqgw,
      &QuadModelv_DW.Hqgw);

    // End of Update for SubSystem: '<S108>/Hqgw'

    // Update for Enabled SubSystem: '<S108>/Hrgw'
    QuadModelv_Hrgw_Update(QuadModelv_B.Hvgwz.Sum, &QuadModelv_B.Hrgw,
      &QuadModelv_DW.Hrgw);

    // End of Update for SubSystem: '<S108>/Hrgw'

    // Update for Enabled SubSystem: '<S147>/Hpgw'
    QuadModelv_Hpgw_Update(&QuadModelv_B.Hpgw_h, &QuadModelv_DW.Hpgw_h);

    // End of Update for SubSystem: '<S147>/Hpgw'

    // Update for Enabled SubSystem: '<S147>/Hqgw'
    QuadModelv_Hqgw_Update(QuadModelv_B.Hwgwz_a.Sum, &QuadModelv_B.Hqgw_a,
      &QuadModelv_DW.Hqgw_a);

    // End of Update for SubSystem: '<S147>/Hqgw'

    // Update for Enabled SubSystem: '<S147>/Hrgw'
    QuadModelv_Hrgw_Update(QuadModelv_B.Hvgwz_f.Sum, &QuadModelv_B.Hrgw_a,
      &QuadModelv_DW.Hrgw_a);

    // End of Update for SubSystem: '<S147>/Hrgw'
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[5] == 0) {
      // Update for UniformRandomNumber: '<S330>/Uniform Random Number4'
      QuadModelv_DW.UniformRandomNumber4_NextOutp_p[0] =
        (QuadModelv_P.UniformRandomNumber4_Maximum_c[0] -
         QuadModelv_P.UniformRandomNumber4_Minimum_o[0]) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_n[0]) +
        QuadModelv_P.UniformRandomNumber4_Minimum_o[0];
      QuadModelv_DW.UniformRandomNumber4_NextOutp_p[1] =
        (QuadModelv_P.UniformRandomNumber4_Maximum_c[1] -
         QuadModelv_P.UniformRandomNumber4_Minimum_o[1]) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_n[1]) +
        QuadModelv_P.UniformRandomNumber4_Minimum_o[1];
      QuadModelv_DW.UniformRandomNumber4_NextOutp_p[2] =
        (QuadModelv_P.UniformRandomNumber4_Maximum_c[2] -
         QuadModelv_P.UniformRandomNumber4_Minimum_o[2]) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_n[2]) +
        QuadModelv_P.UniformRandomNumber4_Minimum_o[2];
    }
  }                                    // end MajorTimeStep

  if (rtmIsMajorTimeStep((&QuadModelv_M))) {
    rt_ertODEUpdateContinuousStates(&(&QuadModelv_M)->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++(&QuadModelv_M)->Timing.clockTick0;
    (&QuadModelv_M)->Timing.t[0] = rtsiGetSolverStopTime(&(&QuadModelv_M)
      ->solverInfo);

    {
      // Update absolute timer for sample time: [0.001s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.001, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      (&QuadModelv_M)->Timing.clockTick1++;
    }

    rate_scheduler((&QuadModelv_M));
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void MulticopterModelClass::QuadModelv_derivatives()
{
  XDot_QuadModelv_T *_rtXdot;
  _rtXdot = ((XDot_QuadModelv_T *) (&QuadModelv_M)->derivs);

  // Derivatives for SecondOrderIntegrator: '<S433>/Integrator, Second-Order Limited' 
  if (QuadModelv_DW.IntegratorSecondOrderLimited_MO[0] == 0) {
    _rtXdot->IntegratorSecondOrderLimited_CS[0] =
      QuadModelv_X.IntegratorSecondOrderLimited_CS[3];
    _rtXdot->IntegratorSecondOrderLimited_CS[3] = QuadModelv_B.Sum2[0];
  }

  if (QuadModelv_DW.IntegratorSecondOrderLimited_MO[1] == 0) {
    _rtXdot->IntegratorSecondOrderLimited_CS[1] =
      QuadModelv_X.IntegratorSecondOrderLimited_CS[4];
    _rtXdot->IntegratorSecondOrderLimited_CS[4] = QuadModelv_B.Sum2[1];
  }

  if (QuadModelv_DW.IntegratorSecondOrderLimited_MO[2] == 0) {
    _rtXdot->IntegratorSecondOrderLimited_CS[2] =
      QuadModelv_X.IntegratorSecondOrderLimited_CS[5];
    _rtXdot->IntegratorSecondOrderLimited_CS[5] = QuadModelv_B.Sum2[2];
  }

  // End of Derivatives for SecondOrderIntegrator: '<S433>/Integrator, Second-Order Limited' 

  // Derivatives for Integrator: '<S15>/q0 q1 q2 q3'
  _rtXdot->q0q1q2q3_CSTATE[0] = QuadModelv_B.TmpSignalConversionAtq0q1q2q3_c[0];
  _rtXdot->q0q1q2q3_CSTATE[1] = QuadModelv_B.TmpSignalConversionAtq0q1q2q3_c[1];
  _rtXdot->q0q1q2q3_CSTATE[2] = QuadModelv_B.TmpSignalConversionAtq0q1q2q3_c[2];
  _rtXdot->q0q1q2q3_CSTATE[3] = QuadModelv_B.TmpSignalConversionAtq0q1q2q3_c[3];

  // Derivatives for Integrator: '<S14>/xe,ye,ze'
  _rtXdot->xeyeze_CSTATE[0] = QuadModelv_B.Product[0];

  // Derivatives for Integrator: '<S14>/p,q,r '
  _rtXdot->pqr_CSTATE[0] = QuadModelv_B.Product2[0];

  // Derivatives for Integrator: '<S14>/ub,vb,wb'
  _rtXdot->ubvbwb_CSTATE[0] = QuadModelv_B.Sum_p[0];

  // Derivatives for Integrator: '<S14>/xe,ye,ze'
  _rtXdot->xeyeze_CSTATE[1] = QuadModelv_B.Product[1];

  // Derivatives for Integrator: '<S14>/p,q,r '
  _rtXdot->pqr_CSTATE[1] = QuadModelv_B.Product2[1];

  // Derivatives for Integrator: '<S14>/ub,vb,wb'
  _rtXdot->ubvbwb_CSTATE[1] = QuadModelv_B.Sum_p[1];

  // Derivatives for Integrator: '<S14>/xe,ye,ze'
  _rtXdot->xeyeze_CSTATE[2] = QuadModelv_B.Product[2];

  // Derivatives for Integrator: '<S14>/p,q,r '
  _rtXdot->pqr_CSTATE[2] = QuadModelv_B.Product2[2];

  // Derivatives for Integrator: '<S14>/ub,vb,wb'
  _rtXdot->ubvbwb_CSTATE[2] = QuadModelv_B.Sum_p[2];

  // Derivatives for Enabled SubSystem: '<S298>/Distance into gust (x)'
  // Derivatives for Enabled SubSystem: '<S298>/Distance into gust (x)'
  // Constant: '<S295>/Constant'
  QuadMod_Distanceintogustx_Deriv(QuadModelv_P.FaultParamAPI.FaultInParams[1],
    QuadModelv_P.Distanceintogustx_d_m, &QuadModelv_DW.Distanceintogustx,
    &QuadModelv_P.Distanceintogustx, &QuadModelv_X.Distanceintogustx,
    &_rtXdot->Distanceintogustx);

  // End of Derivatives for SubSystem: '<S298>/Distance into gust (x)'
  // End of Derivatives for SubSystem: '<S298>/Distance into gust (x)'

  // Derivatives for Enabled SubSystem: '<S298>/Distance into gust (y)'
  // Derivatives for Enabled SubSystem: '<S298>/Distance into gust (y)'
  QuadMod_Distanceintogusty_Deriv(QuadModelv_P.FaultParamAPI.FaultInParams[1],
    QuadModelv_P.Distanceintogusty_d_m, &QuadModelv_DW.Distanceintogusty,
    &QuadModelv_P.Distanceintogusty, &QuadModelv_X.Distanceintogusty,
    &_rtXdot->Distanceintogusty);

  // End of Derivatives for SubSystem: '<S298>/Distance into gust (y)'
  // End of Derivatives for SubSystem: '<S298>/Distance into gust (y)'

  // Derivatives for Enabled SubSystem: '<S298>/Distance into gust (z)'
  // Derivatives for Enabled SubSystem: '<S298>/Distance into gust (z)'
  QuadMod_Distanceintogusty_Deriv(QuadModelv_P.FaultParamAPI.FaultInParams[1],
    QuadModelv_P.Distanceintogustz_d_m, &QuadModelv_DW.Distanceintogustz,
    &QuadModelv_P.Distanceintogustz, &QuadModelv_X.Distanceintogustz,
    &_rtXdot->Distanceintogustz);

  // End of Derivatives for SubSystem: '<S298>/Distance into gust (z)'
  // End of Derivatives for SubSystem: '<S298>/Distance into gust (z)'

  // Derivatives for Atomic SubSystem: '<S296>/MotorNonlinearDynamic1'
  Qu_MotorNonlinearDynamic1_Deriv(&QuadModelv_B.MotorNonlinearDynamic1,
    &_rtXdot->MotorNonlinearDynamic1);

  // End of Derivatives for SubSystem: '<S296>/MotorNonlinearDynamic1'

  // Derivatives for Atomic SubSystem: '<S296>/MotorNonlinearDynamic2'
  Qu_MotorNonlinearDynamic1_Deriv(&QuadModelv_B.MotorNonlinearDynamic2,
    &_rtXdot->MotorNonlinearDynamic2);

  // End of Derivatives for SubSystem: '<S296>/MotorNonlinearDynamic2'

  // Derivatives for Atomic SubSystem: '<S296>/MotorNonlinearDynamic3'
  Qu_MotorNonlinearDynamic1_Deriv(&QuadModelv_B.MotorNonlinearDynamic3,
    &_rtXdot->MotorNonlinearDynamic3);

  // End of Derivatives for SubSystem: '<S296>/MotorNonlinearDynamic3'

  // Derivatives for Atomic SubSystem: '<S296>/MotorNonlinearDynamic4'
  Qu_MotorNonlinearDynamic1_Deriv(&QuadModelv_B.MotorNonlinearDynamic4,
    &_rtXdot->MotorNonlinearDynamic4);

  // End of Derivatives for SubSystem: '<S296>/MotorNonlinearDynamic4'

  // Derivatives for Atomic SubSystem: '<S296>/MotorNonlinearDynamic5'
  Qu_MotorNonlinearDynamic1_Deriv(&QuadModelv_B.MotorNonlinearDynamic5,
    &_rtXdot->MotorNonlinearDynamic5);

  // End of Derivatives for SubSystem: '<S296>/MotorNonlinearDynamic5'

  // Derivatives for Atomic SubSystem: '<S296>/MotorNonlinearDynamic6'
  Qu_MotorNonlinearDynamic1_Deriv(&QuadModelv_B.MotorNonlinearDynamic6,
    &_rtXdot->MotorNonlinearDynamic6);

  // End of Derivatives for SubSystem: '<S296>/MotorNonlinearDynamic6'

  // Derivatives for Atomic SubSystem: '<S296>/MotorNonlinearDynamic7'
  Qu_MotorNonlinearDynamic1_Deriv(&QuadModelv_B.MotorNonlinearDynamic7,
    &_rtXdot->MotorNonlinearDynamic7);

  // End of Derivatives for SubSystem: '<S296>/MotorNonlinearDynamic7'

  // Derivatives for Atomic SubSystem: '<S296>/MotorNonlinearDynamic8'
  Qu_MotorNonlinearDynamic1_Deriv(&QuadModelv_B.MotorNonlinearDynamic8,
    &_rtXdot->MotorNonlinearDynamic8);

  // End of Derivatives for SubSystem: '<S296>/MotorNonlinearDynamic8'

  // Derivatives for Enabled SubSystem: '<S524>/Distance into gust (x)'
  // Derivatives for Enabled SubSystem: '<S524>/Distance into gust (x)'
  // Constant: '<S522>/Constant'
  QuadMod_Distanceintogustx_Deriv(QuadModelv_P.FaultParamAPI.FaultInParams[1],
    QuadModelv_P.Distanceintogustx_d_m_i, &QuadModelv_DW.Distanceintogustx_g,
    &QuadModelv_P.Distanceintogustx_g, &QuadModelv_X.Distanceintogustx_g,
    &_rtXdot->Distanceintogustx_g);

  // End of Derivatives for SubSystem: '<S524>/Distance into gust (x)'
  // End of Derivatives for SubSystem: '<S524>/Distance into gust (x)'

  // Derivatives for Enabled SubSystem: '<S524>/Distance into gust (y)'
  // Derivatives for Enabled SubSystem: '<S524>/Distance into gust (y)'
  QuadMod_Distanceintogusty_Deriv(QuadModelv_P.FaultParamAPI.FaultInParams[1],
    QuadModelv_P.Distanceintogusty_d_m_g, &QuadModelv_DW.Distanceintogusty_f,
    &QuadModelv_P.Distanceintogusty_f, &QuadModelv_X.Distanceintogusty_f,
    &_rtXdot->Distanceintogusty_f);

  // End of Derivatives for SubSystem: '<S524>/Distance into gust (y)'
  // End of Derivatives for SubSystem: '<S524>/Distance into gust (y)'

  // Derivatives for Enabled SubSystem: '<S524>/Distance into gust (z)'
  // Derivatives for Enabled SubSystem: '<S524>/Distance into gust (z)'
  QuadMod_Distanceintogusty_Deriv(QuadModelv_P.FaultParamAPI.FaultInParams[1],
    QuadModelv_P.Distanceintogustz_d_m_n, &QuadModelv_DW.Distanceintogustz_b,
    &QuadModelv_P.Distanceintogustz_b, &QuadModelv_X.Distanceintogustz_b,
    &_rtXdot->Distanceintogustz_b);

  // End of Derivatives for SubSystem: '<S524>/Distance into gust (z)'
  // End of Derivatives for SubSystem: '<S524>/Distance into gust (z)'

  // Derivatives for SecondOrderIntegrator: '<S444>/Integrator, Second-Order Limited' 
  if (QuadModelv_DW.IntegratorSecondOrderLimited__n[0] == 0) {
    _rtXdot->IntegratorSecondOrderLimited__i[0] =
      QuadModelv_X.IntegratorSecondOrderLimited__i[3];
    _rtXdot->IntegratorSecondOrderLimited__i[3] = QuadModelv_B.Sum2_f[0];
  }

  if (QuadModelv_DW.IntegratorSecondOrderLimited__n[1] == 0) {
    _rtXdot->IntegratorSecondOrderLimited__i[1] =
      QuadModelv_X.IntegratorSecondOrderLimited__i[4];
    _rtXdot->IntegratorSecondOrderLimited__i[4] = QuadModelv_B.Sum2_f[1];
  }

  if (QuadModelv_DW.IntegratorSecondOrderLimited__n[2] == 0) {
    _rtXdot->IntegratorSecondOrderLimited__i[2] =
      QuadModelv_X.IntegratorSecondOrderLimited__i[5];
    _rtXdot->IntegratorSecondOrderLimited__i[5] = QuadModelv_B.Sum2_f[2];
  }

  // End of Derivatives for SecondOrderIntegrator: '<S444>/Integrator, Second-Order Limited' 

  // Derivatives for TransferFcn: '<S376>/Transfer Fcn4'
  _rtXdot->TransferFcn4_CSTATE = 0.0;
  _rtXdot->TransferFcn4_CSTATE += QuadModelv_P.TransferFcn4_A *
    QuadModelv_X.TransferFcn4_CSTATE;
  _rtXdot->TransferFcn4_CSTATE += QuadModelv_B.BiasGain1[0];

  // Derivatives for TransferFcn: '<S376>/Transfer Fcn1'
  _rtXdot->TransferFcn1_CSTATE = 0.0;
  _rtXdot->TransferFcn1_CSTATE += QuadModelv_P.TransferFcn1_A *
    QuadModelv_X.TransferFcn1_CSTATE;
  _rtXdot->TransferFcn1_CSTATE += QuadModelv_B.BiasGain1[1];

  // Derivatives for TransferFcn: '<S376>/Transfer Fcn2'
  _rtXdot->TransferFcn2_CSTATE = 0.0;
  _rtXdot->TransferFcn2_CSTATE += QuadModelv_P.TransferFcn2_A *
    QuadModelv_X.TransferFcn2_CSTATE;
  _rtXdot->TransferFcn2_CSTATE += QuadModelv_B.BiasGain1[2];
}

// Model initialize function
void MulticopterModelClass::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  // non-finite (run-time) assignments
  QuadModelv_P.uftinf_UpperSat = rtInf;

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&(&QuadModelv_M)->solverInfo, &(&QuadModelv_M)
                          ->Timing.simTimeStep);
    rtsiSetTPtr(&(&QuadModelv_M)->solverInfo, &rtmGetTPtr((&QuadModelv_M)));
    rtsiSetStepSizePtr(&(&QuadModelv_M)->solverInfo, &(&QuadModelv_M)
                       ->Timing.stepSize0);
    rtsiSetdXPtr(&(&QuadModelv_M)->solverInfo, &(&QuadModelv_M)->derivs);
    rtsiSetContStatesPtr(&(&QuadModelv_M)->solverInfo, (real_T **)
                         &(&QuadModelv_M)->contStates);
    rtsiSetNumContStatesPtr(&(&QuadModelv_M)->solverInfo, &(&QuadModelv_M)
      ->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&(&QuadModelv_M)->solverInfo,
      &(&QuadModelv_M)->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&(&QuadModelv_M)->solverInfo,
      &(&QuadModelv_M)->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&(&QuadModelv_M)->solverInfo,
      &(&QuadModelv_M)->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&(&QuadModelv_M)->solverInfo, (&rtmGetErrorStatus
      ((&QuadModelv_M))));
    rtsiSetRTModelPtr(&(&QuadModelv_M)->solverInfo, (&QuadModelv_M));
  }

  rtsiSetSimTimeStep(&(&QuadModelv_M)->solverInfo, MAJOR_TIME_STEP);
  (&QuadModelv_M)->intgData.y = (&QuadModelv_M)->odeY;
  (&QuadModelv_M)->intgData.f[0] = (&QuadModelv_M)->odeF[0];
  (&QuadModelv_M)->intgData.f[1] = (&QuadModelv_M)->odeF[1];
  (&QuadModelv_M)->intgData.f[2] = (&QuadModelv_M)->odeF[2];
  (&QuadModelv_M)->intgData.f[3] = (&QuadModelv_M)->odeF[3];
  (&QuadModelv_M)->contStates = ((X_QuadModelv_T *) &QuadModelv_X);
  rtsiSetSolverData(&(&QuadModelv_M)->solverInfo, static_cast<void *>
                    (&(&QuadModelv_M)->intgData));
  rtsiSetIsMinorTimeStepWithModeChange(&(&QuadModelv_M)->solverInfo, false);
  rtsiSetSolverName(&(&QuadModelv_M)->solverInfo,"ode4");
  rtmSetTPtr((&QuadModelv_M), &(&QuadModelv_M)->Timing.tArray[0]);
  (&QuadModelv_M)->Timing.stepSize0 = 0.001;
  rtmSetFirstInitCond((&QuadModelv_M), 1);

  {
    real_T tmp;
    int32_T i;
    int32_T t;
    uint32_T tseed;
    static const uint32_T tmp_0[625] = { 5489U, 1301868182U, 2938499221U,
      2950281878U, 1875628136U, 751856242U, 944701696U, 2243192071U, 694061057U,
      219885934U, 2066767472U, 3182869408U, 485472502U, 2336857883U, 1071588843U,
      3418470598U, 951210697U, 3693558366U, 2923482051U, 1793174584U,
      2982310801U, 1586906132U, 1951078751U, 1808158765U, 1733897588U,
      431328322U, 4202539044U, 530658942U, 1714810322U, 3025256284U, 3342585396U,
      1937033938U, 2640572511U, 1654299090U, 3692403553U, 4233871309U,
      3497650794U, 862629010U, 2943236032U, 2426458545U, 1603307207U,
      1133453895U, 3099196360U, 2208657629U, 2747653927U, 931059398U, 761573964U,
      3157853227U, 785880413U, 730313442U, 124945756U, 2937117055U, 3295982469U,
      1724353043U, 3021675344U, 3884886417U, 4010150098U, 4056961966U,
      699635835U, 2681338818U, 1339167484U, 720757518U, 2800161476U, 2376097373U,
      1532957371U, 3902664099U, 1238982754U, 3725394514U, 3449176889U,
      3570962471U, 4287636090U, 4087307012U, 3603343627U, 202242161U,
      2995682783U, 1620962684U, 3704723357U, 371613603U, 2814834333U,
      2111005706U, 624778151U, 2094172212U, 4284947003U, 1211977835U, 991917094U,
      1570449747U, 2962370480U, 1259410321U, 170182696U, 146300961U, 2836829791U,
      619452428U, 2723670296U, 1881399711U, 1161269684U, 1675188680U,
      4132175277U, 780088327U, 3409462821U, 1036518241U, 1834958505U,
      3048448173U, 161811569U, 618488316U, 44795092U, 3918322701U, 1924681712U,
      3239478144U, 383254043U, 4042306580U, 2146983041U, 3992780527U,
      3518029708U, 3545545436U, 3901231469U, 1896136409U, 2028528556U,
      2339662006U, 501326714U, 2060962201U, 2502746480U, 561575027U, 581893337U,
      3393774360U, 1778912547U, 3626131687U, 2175155826U, 319853231U, 986875531U,
      819755096U, 2915734330U, 2688355739U, 3482074849U, 2736559U, 2296975761U,
      1029741190U, 2876812646U, 690154749U, 579200347U, 4027461746U, 1285330465U,
      2701024045U, 4117700889U, 759495121U, 3332270341U, 2313004527U,
      2277067795U, 4131855432U, 2722057515U, 1264804546U, 3848622725U,
      2211267957U, 4100593547U, 959123777U, 2130745407U, 3194437393U, 486673947U,
      1377371204U, 17472727U, 352317554U, 3955548058U, 159652094U, 1232063192U,
      3835177280U, 49423123U, 3083993636U, 733092U, 2120519771U, 2573409834U,
      1112952433U, 3239502554U, 761045320U, 1087580692U, 2540165110U, 641058802U,
      1792435497U, 2261799288U, 1579184083U, 627146892U, 2165744623U,
      2200142389U, 2167590760U, 2381418376U, 1793358889U, 3081659520U,
      1663384067U, 2009658756U, 2689600308U, 739136266U, 2304581039U,
      3529067263U, 591360555U, 525209271U, 3131882996U, 294230224U, 2076220115U,
      3113580446U, 1245621585U, 1386885462U, 3203270426U, 123512128U, 12350217U,
      354956375U, 4282398238U, 3356876605U, 3888857667U, 157639694U, 2616064085U,
      1563068963U, 2762125883U, 4045394511U, 4180452559U, 3294769488U,
      1684529556U, 1002945951U, 3181438866U, 22506664U, 691783457U, 2685221343U,
      171579916U, 3878728600U, 2475806724U, 2030324028U, 3331164912U,
      1708711359U, 1970023127U, 2859691344U, 2588476477U, 2748146879U,
      136111222U, 2967685492U, 909517429U, 2835297809U, 3206906216U, 3186870716U,
      341264097U, 2542035121U, 3353277068U, 548223577U, 3170936588U, 1678403446U,
      297435620U, 2337555430U, 466603495U, 1132321815U, 1208589219U, 696392160U,
      894244439U, 2562678859U, 470224582U, 3306867480U, 201364898U, 2075966438U,
      1767227936U, 2929737987U, 3674877796U, 2654196643U, 3692734598U,
      3528895099U, 2796780123U, 3048728353U, 842329300U, 191554730U, 2922459673U,
      3489020079U, 3979110629U, 1022523848U, 2202932467U, 3583655201U,
      3565113719U, 587085778U, 4176046313U, 3013713762U, 950944241U, 396426791U,
      3784844662U, 3477431613U, 3594592395U, 2782043838U, 3392093507U,
      3106564952U, 2829419931U, 1358665591U, 2206918825U, 3170783123U, 31522386U,
      2988194168U, 1782249537U, 1105080928U, 843500134U, 1225290080U,
      1521001832U, 3605886097U, 2802786495U, 2728923319U, 3996284304U,
      903417639U, 1171249804U, 1020374987U, 2824535874U, 423621996U, 1988534473U,
      2493544470U, 1008604435U, 1756003503U, 1488867287U, 1386808992U,
      732088248U, 1780630732U, 2482101014U, 976561178U, 1543448953U, 2602866064U,
      2021139923U, 1952599828U, 2360242564U, 2117959962U, 2753061860U,
      2388623612U, 4138193781U, 2962920654U, 2284970429U, 766920861U,
      3457264692U, 2879611383U, 815055854U, 2332929068U, 1254853997U,
      3740375268U, 3799380844U, 4091048725U, 2006331129U, 1982546212U,
      686850534U, 1907447564U, 2682801776U, 2780821066U, 998290361U, 1342433871U,
      4195430425U, 607905174U, 3902331779U, 2454067926U, 1708133115U,
      1170874362U, 2008609376U, 3260320415U, 2211196135U, 433538229U,
      2728786374U, 2189520818U, 262554063U, 1182318347U, 3710237267U,
      1221022450U, 715966018U, 2417068910U, 2591870721U, 2870691989U,
      3418190842U, 4238214053U, 1540704231U, 1575580968U, 2095917976U,
      4078310857U, 2313532447U, 2110690783U, 4056346629U, 4061784526U,
      1123218514U, 551538993U, 597148360U, 4120175196U, 3581618160U, 3181170517U,
      422862282U, 3227524138U, 1713114790U, 662317149U, 1230418732U, 928171837U,
      1324564878U, 1928816105U, 1786535431U, 2878099422U, 3290185549U,
      539474248U, 1657512683U, 552370646U, 1671741683U, 3655312128U, 1552739510U,
      2605208763U, 1441755014U, 181878989U, 3124053868U, 1447103986U,
      3183906156U, 1728556020U, 3502241336U, 3055466967U, 1013272474U,
      818402132U, 1715099063U, 2900113506U, 397254517U, 4194863039U, 1009068739U,
      232864647U, 2540223708U, 2608288560U, 2415367765U, 478404847U, 3455100648U,
      3182600021U, 2115988978U, 434269567U, 4117179324U, 3461774077U, 887256537U,
      3545801025U, 286388911U, 3451742129U, 1981164769U, 786667016U, 3310123729U,
      3097811076U, 2224235657U, 2959658883U, 3370969234U, 2514770915U,
      3345656436U, 2677010851U, 2206236470U, 271648054U, 2342188545U,
      4292848611U, 3646533909U, 3754009956U, 3803931226U, 4160647125U,
      1477814055U, 4043852216U, 1876372354U, 3133294443U, 3871104810U,
      3177020907U, 2074304428U, 3479393793U, 759562891U, 164128153U, 1839069216U,
      2114162633U, 3989947309U, 3611054956U, 1333547922U, 835429831U, 494987340U,
      171987910U, 1252001001U, 370809172U, 3508925425U, 2535703112U, 1276855041U,
      1922855120U, 835673414U, 3030664304U, 613287117U, 171219893U, 3423096126U,
      3376881639U, 2287770315U, 1658692645U, 1262815245U, 3957234326U,
      1168096164U, 2968737525U, 2655813712U, 2132313144U, 3976047964U,
      326516571U, 353088456U, 3679188938U, 3205649712U, 2654036126U, 1249024881U,
      880166166U, 691800469U, 2229503665U, 1673458056U, 4032208375U, 1851778863U,
      2563757330U, 376742205U, 1794655231U, 340247333U, 1505873033U, 396524441U,
      879666767U, 3335579166U, 3260764261U, 3335999539U, 506221798U, 4214658741U,
      975887814U, 2080536343U, 3360539560U, 571586418U, 138896374U, 4234352651U,
      2737620262U, 3928362291U, 1516365296U, 38056726U, 3599462320U, 3585007266U,
      3850961033U, 471667319U, 1536883193U, 2310166751U, 1861637689U,
      2530999841U, 4139843801U, 2710569485U, 827578615U, 2012334720U,
      2907369459U, 3029312804U, 2820112398U, 1965028045U, 35518606U, 2478379033U,
      643747771U, 1924139484U, 4123405127U, 3811735531U, 3429660832U,
      3285177704U, 1948416081U, 1311525291U, 1183517742U, 1739192232U,
      3979815115U, 2567840007U, 4116821529U, 213304419U, 4125718577U,
      1473064925U, 2442436592U, 1893310111U, 4195361916U, 3747569474U,
      828465101U, 2991227658U, 750582866U, 1205170309U, 1409813056U, 678418130U,
      1171531016U, 3821236156U, 354504587U, 4202874632U, 3882511497U,
      1893248677U, 1903078632U, 26340130U, 2069166240U, 3657122492U, 3725758099U,
      831344905U, 811453383U, 3447711422U, 2434543565U, 4166886888U, 3358210805U,
      4142984013U, 2988152326U, 3527824853U, 982082992U, 2809155763U, 190157081U,
      3340214818U, 2365432395U, 2548636180U, 2894533366U, 3474657421U,
      2372634704U, 2845748389U, 43024175U, 2774226648U, 1987702864U, 3186502468U,
      453610222U, 4204736567U, 1392892630U, 2471323686U, 2470534280U,
      3541393095U, 4269885866U, 3909911300U, 759132955U, 1482612480U, 667715263U,
      1795580598U, 2337923983U, 3390586366U, 581426223U, 1515718634U, 476374295U,
      705213300U, 363062054U, 2084697697U, 2407503428U, 2292957699U, 2426213835U,
      2199989172U, 1987356470U, 4026755612U, 2147252133U, 270400031U,
      1367820199U, 2369854699U, 2844269403U, 79981964U, 624U };

    // Start for Constant: '<S187>/FaultID'
    QuadModelv_B.FaultID = QuadModelv_P.FaultID_Value;

    // Start for If: '<S42>/If'
    QuadModelv_DW.If_ActiveSubsystem = -1;

    // Start for If: '<S194>/If'
    QuadModelv_DW.If_ActiveSubsystem_h = -1;

    // Start for If: '<S193>/If'
    QuadModelv_DW.If_ActiveSubsystem_l = -1;

    // Start for Constant: '<S296>/ModelInit_RPM'
    QuadModelv_B.ModelInit_RPM = QuadModelv_P.ModelInit_RPM;

    // Start for If: '<S153>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' 
    QuadModelv_DW.ifHeightMaxlowaltitudeelseifHei = -1;

    // Start for If: '<S114>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' 
    QuadModelv_DW.ifHeightMaxlowaltitudeelseifH_p = -1;

    // Start for If: '<S329>/If'
    QuadModelv_DW.If_ActiveSubsystem_m = -1;

    // Start for If: '<S113>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' 
    QuadModelv_DW.ifHeightMaxlowaltitudeelseifH_j = -1;

    // Start for If: '<S152>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' 
    QuadModelv_DW.ifHeightMaxlowaltitudeelseifH_e = -1;

    // Start for If: '<S203>/If1'
    QuadModelv_DW.If1_ActiveSubsystem = -1;

    // Start for If: '<S241>/If1'
    QuadModelv_DW.If1_ActiveSubsystem_o = -1;

    // Start for If: '<S337>/If1'
    QuadModelv_DW.If1_ActiveSubsystem_d = -1;

    // InitializeConditions for SecondOrderIntegrator: '<S433>/Integrator, Second-Order Limited' 
    QuadModelv_X.IntegratorSecondOrderLimited_CS[0] =
      QuadModelv_P.IntegratorSecondOrderLimited_IC;
    QuadModelv_X.IntegratorSecondOrderLimited_CS[1] =
      QuadModelv_P.IntegratorSecondOrderLimited_IC;
    QuadModelv_X.IntegratorSecondOrderLimited_CS[2] =
      QuadModelv_P.IntegratorSecondOrderLimited_IC;
    QuadModelv_X.IntegratorSecondOrderLimited_CS[3] =
      QuadModelv_P.IntegratorSecondOrderLimited__n;
    QuadModelv_X.IntegratorSecondOrderLimited_CS[4] =
      QuadModelv_P.IntegratorSecondOrderLimited__n;
    QuadModelv_X.IntegratorSecondOrderLimited_CS[5] =
      QuadModelv_P.IntegratorSecondOrderLimited__n;

    // InitializeConditions for Integrator: '<S15>/q0 q1 q2 q3'
    if (rtmIsFirstInitCond((&QuadModelv_M))) {
      QuadModelv_X.q0q1q2q3_CSTATE[0] = 0.0;
      QuadModelv_X.q0q1q2q3_CSTATE[1] = 0.0;
      QuadModelv_X.q0q1q2q3_CSTATE[2] = 0.0;
      QuadModelv_X.q0q1q2q3_CSTATE[3] = 0.0;
    }

    QuadModelv_DW.q0q1q2q3_IWORK = 1;

    // End of InitializeConditions for Integrator: '<S15>/q0 q1 q2 q3'

    // InitializeConditions for Integrator: '<S14>/xe,ye,ze'
    QuadModelv_X.xeyeze_CSTATE[0] = QuadModelv_P.ModelInit_PosE[0];

    // InitializeConditions for Integrator: '<S14>/p,q,r '
    QuadModelv_X.pqr_CSTATE[0] = QuadModelv_P.ModelInit_RateB[0];

    // InitializeConditions for Integrator: '<S14>/ub,vb,wb'
    QuadModelv_X.ubvbwb_CSTATE[0] = QuadModelv_P.ModelInit_VelB[0];

    // InitializeConditions for Integrator: '<S14>/xe,ye,ze'
    QuadModelv_X.xeyeze_CSTATE[1] = QuadModelv_P.ModelInit_PosE[1];

    // InitializeConditions for Integrator: '<S14>/p,q,r '
    QuadModelv_X.pqr_CSTATE[1] = QuadModelv_P.ModelInit_RateB[1];

    // InitializeConditions for Integrator: '<S14>/ub,vb,wb'
    QuadModelv_X.ubvbwb_CSTATE[1] = QuadModelv_P.ModelInit_VelB[1];

    // InitializeConditions for Integrator: '<S14>/xe,ye,ze'
    QuadModelv_X.xeyeze_CSTATE[2] = QuadModelv_P.ModelInit_PosE[2];

    // InitializeConditions for Integrator: '<S14>/p,q,r '
    QuadModelv_X.pqr_CSTATE[2] = QuadModelv_P.ModelInit_RateB[2];

    // InitializeConditions for Integrator: '<S14>/ub,vb,wb'
    QuadModelv_X.ubvbwb_CSTATE[2] = QuadModelv_P.ModelInit_VelB[2];

    // InitializeConditions for Memory: '<Root>/Memory'
    for (i = 0; i < 8; i++) {
      QuadModelv_DW.Memory_PreviousInput[i] =
        QuadModelv_P.Memory_InitialCondition;
    }

    // End of InitializeConditions for Memory: '<Root>/Memory'

    // InitializeConditions for RandomNumber: '<S297>/White Noise'
    tmp = std::floor(QuadModelv_P.BandLimitedWhiteNoise_seed);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed = tseed;
    QuadModelv_DW.NextOutput = rt_nrand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed) * QuadModelv_P.WhiteNoise_StdDev +
      QuadModelv_P.WhiteNoise_Mean;

    // End of InitializeConditions for RandomNumber: '<S297>/White Noise'

    // InitializeConditions for RandomNumber: '<S523>/White Noise'
    tmp = std::floor(QuadModelv_P.BandLimitedWhiteNoise_seed_h);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_h = tseed;
    QuadModelv_DW.NextOutput_g = rt_nrand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed_h) * QuadModelv_P.WhiteNoise_StdDev_b +
      QuadModelv_P.WhiteNoise_Mean_n;

    // End of InitializeConditions for RandomNumber: '<S523>/White Noise'

    // InitializeConditions for UniformRandomNumber: '<S67>/Uniform Random Number' 
    tmp = std::floor(QuadModelv_P.UniformRandomNumber_Seed[0]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_o[0] = tseed;
    QuadModelv_DW.UniformRandomNumber_NextOutput[0] =
      (QuadModelv_P.UniformRandomNumber_Maximum[0] -
       QuadModelv_P.UniformRandomNumber_Minimum[0]) * rt_urand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed_o[0]) + QuadModelv_P.UniformRandomNumber_Minimum
      [0];
    tmp = std::floor(QuadModelv_P.UniformRandomNumber_Seed[1]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_o[1] = tseed;
    QuadModelv_DW.UniformRandomNumber_NextOutput[1] =
      (QuadModelv_P.UniformRandomNumber_Maximum[1] -
       QuadModelv_P.UniformRandomNumber_Minimum[1]) * rt_urand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed_o[1]) + QuadModelv_P.UniformRandomNumber_Minimum
      [1];
    tmp = std::floor(QuadModelv_P.UniformRandomNumber_Seed[2]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_o[2] = tseed;
    QuadModelv_DW.UniformRandomNumber_NextOutput[2] =
      (QuadModelv_P.UniformRandomNumber_Maximum[2] -
       QuadModelv_P.UniformRandomNumber_Minimum[2]) * rt_urand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed_o[2]) + QuadModelv_P.UniformRandomNumber_Minimum
      [2];

    // End of InitializeConditions for UniformRandomNumber: '<S67>/Uniform Random Number' 

    // InitializeConditions for RandomNumber: '<S157>/White Noise'
    tmp = std::floor(QuadModelv_P.DrydenWindTurbulenceModelDisc_n[0]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_p[0] = tseed;
    QuadModelv_DW.NextOutput_c[0] = rt_nrand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed_p[0]) * QuadModelv_P.WhiteNoise_StdDev_j +
      QuadModelv_P.WhiteNoise_Mean_j;

    // InitializeConditions for RandomNumber: '<S118>/White Noise'
    tmp = std::floor(QuadModelv_P.DrydenWindTurbulenceModelDisc_i[0]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_or[0] = tseed;
    QuadModelv_DW.NextOutput_p[0] = rt_nrand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed_or[0]) * QuadModelv_P.WhiteNoise_StdDev_k +
      QuadModelv_P.WhiteNoise_Mean_m;

    // InitializeConditions for RandomNumber: '<S157>/White Noise'
    tmp = std::floor(QuadModelv_P.DrydenWindTurbulenceModelDisc_n[1]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_p[1] = tseed;
    QuadModelv_DW.NextOutput_c[1] = rt_nrand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed_p[1]) * QuadModelv_P.WhiteNoise_StdDev_j +
      QuadModelv_P.WhiteNoise_Mean_j;

    // InitializeConditions for RandomNumber: '<S118>/White Noise'
    tmp = std::floor(QuadModelv_P.DrydenWindTurbulenceModelDisc_i[1]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_or[1] = tseed;
    QuadModelv_DW.NextOutput_p[1] = rt_nrand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed_or[1]) * QuadModelv_P.WhiteNoise_StdDev_k +
      QuadModelv_P.WhiteNoise_Mean_m;

    // InitializeConditions for RandomNumber: '<S157>/White Noise'
    tmp = std::floor(QuadModelv_P.DrydenWindTurbulenceModelDisc_n[2]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_p[2] = tseed;
    QuadModelv_DW.NextOutput_c[2] = rt_nrand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed_p[2]) * QuadModelv_P.WhiteNoise_StdDev_j +
      QuadModelv_P.WhiteNoise_Mean_j;

    // InitializeConditions for RandomNumber: '<S118>/White Noise'
    tmp = std::floor(QuadModelv_P.DrydenWindTurbulenceModelDisc_i[2]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_or[2] = tseed;
    QuadModelv_DW.NextOutput_p[2] = rt_nrand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed_or[2]) * QuadModelv_P.WhiteNoise_StdDev_k +
      QuadModelv_P.WhiteNoise_Mean_m;

    // InitializeConditions for RandomNumber: '<S157>/White Noise'
    tmp = std::floor(QuadModelv_P.DrydenWindTurbulenceModelDisc_n[3]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_p[3] = tseed;
    QuadModelv_DW.NextOutput_c[3] = rt_nrand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed_p[3]) * QuadModelv_P.WhiteNoise_StdDev_j +
      QuadModelv_P.WhiteNoise_Mean_j;

    // InitializeConditions for RandomNumber: '<S118>/White Noise'
    tmp = std::floor(QuadModelv_P.DrydenWindTurbulenceModelDisc_i[3]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_or[3] = tseed;
    QuadModelv_DW.NextOutput_p[3] = rt_nrand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed_or[3]) * QuadModelv_P.WhiteNoise_StdDev_k +
      QuadModelv_P.WhiteNoise_Mean_m;

    // InitializeConditions for UniformRandomNumber: '<S407>/Uniform Random Number5' 
    tmp = std::floor(QuadModelv_P.UniformRandomNumber5_Seed[0]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_d[0] = tseed;
    QuadModelv_DW.UniformRandomNumber5_NextOutput[0] =
      (QuadModelv_P.UniformRandomNumber5_Maximum[0] -
       QuadModelv_P.UniformRandomNumber5_Minimum[0]) *
      rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_d[0]) +
      QuadModelv_P.UniformRandomNumber5_Minimum[0];

    // InitializeConditions for SecondOrderIntegrator: '<S444>/Integrator, Second-Order Limited' 
    QuadModelv_X.IntegratorSecondOrderLimited__i[0] =
      QuadModelv_P.IntegratorSecondOrderLimited__f;

    // InitializeConditions for UniformRandomNumber: '<S407>/Uniform Random Number5' 
    tmp = std::floor(QuadModelv_P.UniformRandomNumber5_Seed[1]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_d[1] = tseed;
    QuadModelv_DW.UniformRandomNumber5_NextOutput[1] =
      (QuadModelv_P.UniformRandomNumber5_Maximum[1] -
       QuadModelv_P.UniformRandomNumber5_Minimum[1]) *
      rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_d[1]) +
      QuadModelv_P.UniformRandomNumber5_Minimum[1];

    // InitializeConditions for SecondOrderIntegrator: '<S444>/Integrator, Second-Order Limited' 
    QuadModelv_X.IntegratorSecondOrderLimited__i[1] =
      QuadModelv_P.IntegratorSecondOrderLimited__f;

    // InitializeConditions for UniformRandomNumber: '<S407>/Uniform Random Number5' 
    tmp = std::floor(QuadModelv_P.UniformRandomNumber5_Seed[2]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_d[2] = tseed;
    QuadModelv_DW.UniformRandomNumber5_NextOutput[2] =
      (QuadModelv_P.UniformRandomNumber5_Maximum[2] -
       QuadModelv_P.UniformRandomNumber5_Minimum[2]) *
      rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_d[2]) +
      QuadModelv_P.UniformRandomNumber5_Minimum[2];

    // InitializeConditions for SecondOrderIntegrator: '<S444>/Integrator, Second-Order Limited' 
    QuadModelv_X.IntegratorSecondOrderLimited__i[2] =
      QuadModelv_P.IntegratorSecondOrderLimited__f;

    // InitializeConditions for Memory: '<S468>/otime'
    QuadModelv_DW.otime_PreviousInput = QuadModelv_P.otime_InitialCondition;

    // InitializeConditions for Memory: '<S467>/olon'
    QuadModelv_DW.olon_PreviousInput = QuadModelv_P.olon_InitialCondition;

    // InitializeConditions for Memory: '<S466>/olat'
    QuadModelv_DW.olat_PreviousInput = QuadModelv_P.olat_InitialCondition;

    // InitializeConditions for Memory: '<S466>/oalt'
    QuadModelv_DW.oalt_PreviousInput = QuadModelv_P.oalt_InitialCondition;

    // InitializeConditions for SecondOrderIntegrator: '<S444>/Integrator, Second-Order Limited' 
    QuadModelv_X.IntegratorSecondOrderLimited__i[3] =
      QuadModelv_P.IntegratorSecondOrderLimited__l;

    // InitializeConditions for UniformRandomNumber: '<S407>/Uniform Random Number1' 
    tmp = std::floor(QuadModelv_P.UniformRandomNumber1_Seed[0]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_m[0] = tseed;
    QuadModelv_DW.UniformRandomNumber1_NextOutput[0] =
      (QuadModelv_P.UniformRandomNumber1_Maximum[0] -
       QuadModelv_P.UniformRandomNumber1_Minimum[0]) *
      rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_m[0]) +
      QuadModelv_P.UniformRandomNumber1_Minimum[0];

    // InitializeConditions for UniformRandomNumber: '<S407>/Uniform Random Number7' 
    tmp = std::floor(QuadModelv_P.UniformRandomNumber7_Seed[0]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_e[0] = tseed;
    QuadModelv_DW.UniformRandomNumber7_NextOutput[0] =
      (QuadModelv_P.UniformRandomNumber7_Maximum[0] -
       QuadModelv_P.UniformRandomNumber7_Minimum[0]) *
      rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_e[0]) +
      QuadModelv_P.UniformRandomNumber7_Minimum[0];

    // InitializeConditions for SecondOrderIntegrator: '<S444>/Integrator, Second-Order Limited' 
    QuadModelv_X.IntegratorSecondOrderLimited__i[4] =
      QuadModelv_P.IntegratorSecondOrderLimited__l;

    // InitializeConditions for UniformRandomNumber: '<S407>/Uniform Random Number1' 
    tmp = std::floor(QuadModelv_P.UniformRandomNumber1_Seed[1]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_m[1] = tseed;
    QuadModelv_DW.UniformRandomNumber1_NextOutput[1] =
      (QuadModelv_P.UniformRandomNumber1_Maximum[1] -
       QuadModelv_P.UniformRandomNumber1_Minimum[1]) *
      rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_m[1]) +
      QuadModelv_P.UniformRandomNumber1_Minimum[1];

    // InitializeConditions for UniformRandomNumber: '<S407>/Uniform Random Number7' 
    tmp = std::floor(QuadModelv_P.UniformRandomNumber7_Seed[1]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_e[1] = tseed;
    QuadModelv_DW.UniformRandomNumber7_NextOutput[1] =
      (QuadModelv_P.UniformRandomNumber7_Maximum[1] -
       QuadModelv_P.UniformRandomNumber7_Minimum[1]) *
      rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_e[1]) +
      QuadModelv_P.UniformRandomNumber7_Minimum[1];

    // InitializeConditions for SecondOrderIntegrator: '<S444>/Integrator, Second-Order Limited' 
    QuadModelv_X.IntegratorSecondOrderLimited__i[5] =
      QuadModelv_P.IntegratorSecondOrderLimited__l;

    // InitializeConditions for UniformRandomNumber: '<S407>/Uniform Random Number1' 
    tmp = std::floor(QuadModelv_P.UniformRandomNumber1_Seed[2]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_m[2] = tseed;
    QuadModelv_DW.UniformRandomNumber1_NextOutput[2] =
      (QuadModelv_P.UniformRandomNumber1_Maximum[2] -
       QuadModelv_P.UniformRandomNumber1_Minimum[2]) *
      rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_m[2]) +
      QuadModelv_P.UniformRandomNumber1_Minimum[2];

    // InitializeConditions for UniformRandomNumber: '<S407>/Uniform Random Number7' 
    tmp = std::floor(QuadModelv_P.UniformRandomNumber7_Seed[2]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_e[2] = tseed;
    QuadModelv_DW.UniformRandomNumber7_NextOutput[2] =
      (QuadModelv_P.UniformRandomNumber7_Maximum[2] -
       QuadModelv_P.UniformRandomNumber7_Minimum[2]) *
      rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_e[2]) +
      QuadModelv_P.UniformRandomNumber7_Minimum[2];

    // InitializeConditions for UniformRandomNumber: '<S407>/Uniform Random Number' 
    tmp = std::floor(QuadModelv_P.UniformRandomNumber_Seed_g);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_ei = tseed;
    QuadModelv_DW.UniformRandomNumber_NextOutpu_l =
      (QuadModelv_P.UniformRandomNumber_Maximum_m -
       QuadModelv_P.UniformRandomNumber_Minimum_d) * rt_urand_Upu32_Yd_f_pw_snf(
      &QuadModelv_DW.RandSeed_ei) + QuadModelv_P.UniformRandomNumber_Minimum_d;

    // End of InitializeConditions for UniformRandomNumber: '<S407>/Uniform Random Number' 

    // InitializeConditions for UniformRandomNumber: '<S407>/Uniform Random Number4' 
    tmp = std::floor(QuadModelv_P.UniformRandomNumber4_Seed);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_b = tseed;
    QuadModelv_DW.UniformRandomNumber4_NextOutput =
      (QuadModelv_P.UniformRandomNumber4_Maximum -
       QuadModelv_P.UniformRandomNumber4_Minimum) * rt_urand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed_b) + QuadModelv_P.UniformRandomNumber4_Minimum;

    // End of InitializeConditions for UniformRandomNumber: '<S407>/Uniform Random Number4' 

    // InitializeConditions for UniformRandomNumber: '<S374>/Uniform Random Number5' 
    tmp = std::floor(QuadModelv_P.UniformRandomNumber5_Seed_l[0]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_he[0] = tseed;
    QuadModelv_DW.UniformRandomNumber5_NextOutp_m[0] =
      (QuadModelv_P.UniformRandomNumber5_Maximum_o[0] -
       QuadModelv_P.UniformRandomNumber5_Minimum_e[0]) *
      rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_he[0]) +
      QuadModelv_P.UniformRandomNumber5_Minimum_e[0];
    tmp = std::floor(QuadModelv_P.UniformRandomNumber5_Seed_l[1]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_he[1] = tseed;
    QuadModelv_DW.UniformRandomNumber5_NextOutp_m[1] =
      (QuadModelv_P.UniformRandomNumber5_Maximum_o[1] -
       QuadModelv_P.UniformRandomNumber5_Minimum_e[1]) *
      rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_he[1]) +
      QuadModelv_P.UniformRandomNumber5_Minimum_e[1];
    tmp = std::floor(QuadModelv_P.UniformRandomNumber5_Seed_l[2]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_he[2] = tseed;
    QuadModelv_DW.UniformRandomNumber5_NextOutp_m[2] =
      (QuadModelv_P.UniformRandomNumber5_Maximum_o[2] -
       QuadModelv_P.UniformRandomNumber5_Minimum_e[2]) *
      rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_he[2]) +
      QuadModelv_P.UniformRandomNumber5_Minimum_e[2];

    // End of InitializeConditions for UniformRandomNumber: '<S374>/Uniform Random Number5' 

    // InitializeConditions for TransferFcn: '<S376>/Transfer Fcn4'
    QuadModelv_X.TransferFcn4_CSTATE = 0.0;

    // InitializeConditions for TransferFcn: '<S376>/Transfer Fcn1'
    QuadModelv_X.TransferFcn1_CSTATE = 0.0;

    // InitializeConditions for TransferFcn: '<S376>/Transfer Fcn2'
    QuadModelv_X.TransferFcn2_CSTATE = 0.0;

    // InitializeConditions for UniformRandomNumber: '<S330>/Uniform Random Number4' 
    tmp = std::floor(QuadModelv_P.UniformRandomNumber4_Seed_e[0]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_n[0] = tseed;
    QuadModelv_DW.UniformRandomNumber4_NextOutp_p[0] =
      (QuadModelv_P.UniformRandomNumber4_Maximum_c[0] -
       QuadModelv_P.UniformRandomNumber4_Minimum_o[0]) *
      rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_n[0]) +
      QuadModelv_P.UniformRandomNumber4_Minimum_o[0];
    tmp = std::floor(QuadModelv_P.UniformRandomNumber4_Seed_e[1]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_n[1] = tseed;
    QuadModelv_DW.UniformRandomNumber4_NextOutp_p[1] =
      (QuadModelv_P.UniformRandomNumber4_Maximum_c[1] -
       QuadModelv_P.UniformRandomNumber4_Minimum_o[1]) *
      rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_n[1]) +
      QuadModelv_P.UniformRandomNumber4_Minimum_o[1];
    tmp = std::floor(QuadModelv_P.UniformRandomNumber4_Seed_e[2]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) + t) << 16U) + t) + i;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_n[2] = tseed;
    QuadModelv_DW.UniformRandomNumber4_NextOutp_p[2] =
      (QuadModelv_P.UniformRandomNumber4_Maximum_c[2] -
       QuadModelv_P.UniformRandomNumber4_Minimum_o[2]) *
      rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_n[2]) +
      QuadModelv_P.UniformRandomNumber4_Minimum_o[2];

    // End of InitializeConditions for UniformRandomNumber: '<S330>/Uniform Random Number4' 

    // SystemInitialize for IfAction SubSystem: '<S194>/Negative Trace'
    // '<S191>:1:6' hFault=false;
    // '<S191>:1:9' fParam=zeros(20,1);
    // '<S189>:1:6' hFault=false;
    // '<S189>:1:9' fParam=zeros(20,1);
    // '<S190>:1:6' hFault=false;
    // '<S190>:1:9' fParam=zeros(20,1);
    // '<S188>:1:8' tOld=0;
    QuadModelv_NegativeTrace_Init(&QuadModelv_DW.NegativeTrace_d);

    // End of SystemInitialize for SubSystem: '<S194>/Negative Trace'

    // SystemInitialize for Merge: '<S194>/Merge'
    QuadModelv_B.Merge[0] = QuadModelv_P.Merge_InitialOutput_f[0];
    QuadModelv_B.Merge[1] = QuadModelv_P.Merge_InitialOutput_f[1];
    QuadModelv_B.Merge[2] = QuadModelv_P.Merge_InitialOutput_f[2];
    QuadModelv_B.Merge[3] = QuadModelv_P.Merge_InitialOutput_f[3];

    // SystemInitialize for IfAction SubSystem: '<S193>/Negative Trace'
    QuadModelv_NegativeTrace_Init(&QuadModelv_DW.NegativeTrace);

    // End of SystemInitialize for SubSystem: '<S193>/Negative Trace'

    // SystemInitialize for Merge: '<S193>/Merge'
    QuadModelv_B.Merge_c[0] = QuadModelv_P.Merge_InitialOutput_h4[0];
    QuadModelv_B.Merge_c[1] = QuadModelv_P.Merge_InitialOutput_h4[1];
    QuadModelv_B.Merge_c[2] = QuadModelv_P.Merge_InitialOutput_h4[2];
    QuadModelv_B.Merge_c[3] = QuadModelv_P.Merge_InitialOutput_h4[3];

    // SystemInitialize for MATLAB Function: '<Root>/CollisionDetection'
    QuadModelv_DW.method_l = 7U;
    QuadModelv_DW.state_m = 1144108930U;
    QuadModelv_DW.state_fo[0] = 362436069U;
    QuadModelv_DW.state_fo[1] = 521288629U;

    // SystemInitialize for MATLAB Function: '<S7>/Ground Model'
    // '<S3>:1:7' isCol=int8(0);
    // '<S3>:1:13' fOut=[0;0;0;0;0;0];
    // '<S3>:1:19' mv0=[0;0;0];
    // '<S3>:1:25' tColi=0;
    // '<S196>:1:11' takeoffFlag=int8(0);
    // '<S196>:1:16' landFlag=int8(1);
    QuadModelv_DW.landFlag = 1;

    // '<S525>:1:6' hFault=false;
    // '<S525>:1:9' fParam=zeros(20,1);
    // '<S299>:1:6' hFault=false;
    // '<S299>:1:9' fParam=zeros(20,1);
    for (i = 0; i < 625; i++) {
      // SystemInitialize for MATLAB Function: '<Root>/CollisionDetection'
      tseed = tmp_0[i];
      QuadModelv_DW.state_p[i] = tseed;

      // SystemInitialize for MATLAB Function: '<Root>/Crash_Motor_Fault'
      QuadModelv_DW.state_g[i] = tseed;
    }

    // SystemInitialize for MATLAB Function: '<Root>/Crash_Motor_Fault'
    QuadModelv_DW.method_a = 7U;
    QuadModelv_DW.state_c = 1144108930U;
    QuadModelv_DW.state_f[0] = 362436069U;
    QuadModelv_DW.state_f[1] = 521288629U;

    // SystemInitialize for Enabled SubSystem: '<S298>/Distance into gust (x)'
    // '<S59>:1:6' hFault=false;
    // '<S59>:1:9' fParam=zeros(20,1);
    // '<S60>:1:6' hFault=false;
    // '<S60>:1:9' fParam=zeros(20,1);
    // '<S61>:1:6' hFault=false;
    // '<S61>:1:9' fParam=zeros(20,1);
    // '<S62>:1:6' hFault=false;
    // '<S62>:1:9' fParam=zeros(20,1);
    // '<S58>:1:15' discharge = 0;
    // '<S58>:1:20' startTime = 0;
    // '<S58>:1:25' reset = 0;
    QuadMode_Distanceintogustx_Init(&QuadModelv_B.Distanceintogustx,
      &QuadModelv_P.Distanceintogustx, &QuadModelv_X.Distanceintogustx);

    // End of SystemInitialize for SubSystem: '<S298>/Distance into gust (x)'

    // SystemInitialize for Enabled SubSystem: '<S298>/Distance into gust (y)'
    QuadMode_Distanceintogusty_Init(&QuadModelv_B.Distanceintogusty,
      &QuadModelv_P.Distanceintogusty, &QuadModelv_X.Distanceintogusty);

    // End of SystemInitialize for SubSystem: '<S298>/Distance into gust (y)'

    // SystemInitialize for Enabled SubSystem: '<S298>/Distance into gust (z)'
    QuadMode_Distanceintogusty_Init(&QuadModelv_B.Distanceintogustz,
      &QuadModelv_P.Distanceintogustz, &QuadModelv_X.Distanceintogustz);

    // End of SystemInitialize for SubSystem: '<S298>/Distance into gust (z)'

    // SystemInitialize for Atomic SubSystem: '<S296>/MotorNonlinearDynamic1'
    Qua_MotorNonlinearDynamic1_Init(&QuadModelv_DW.MotorNonlinearDynamic1,
      &QuadModelv_X.MotorNonlinearDynamic1);

    // End of SystemInitialize for SubSystem: '<S296>/MotorNonlinearDynamic1'

    // SystemInitialize for Atomic SubSystem: '<S296>/MotorNonlinearDynamic2'
    Qua_MotorNonlinearDynamic1_Init(&QuadModelv_DW.MotorNonlinearDynamic2,
      &QuadModelv_X.MotorNonlinearDynamic2);

    // End of SystemInitialize for SubSystem: '<S296>/MotorNonlinearDynamic2'

    // SystemInitialize for Atomic SubSystem: '<S296>/MotorNonlinearDynamic3'
    Qua_MotorNonlinearDynamic1_Init(&QuadModelv_DW.MotorNonlinearDynamic3,
      &QuadModelv_X.MotorNonlinearDynamic3);

    // End of SystemInitialize for SubSystem: '<S296>/MotorNonlinearDynamic3'

    // SystemInitialize for Atomic SubSystem: '<S296>/MotorNonlinearDynamic4'
    Qua_MotorNonlinearDynamic1_Init(&QuadModelv_DW.MotorNonlinearDynamic4,
      &QuadModelv_X.MotorNonlinearDynamic4);

    // End of SystemInitialize for SubSystem: '<S296>/MotorNonlinearDynamic4'

    // SystemInitialize for Atomic SubSystem: '<S296>/MotorNonlinearDynamic5'
    Qua_MotorNonlinearDynamic1_Init(&QuadModelv_DW.MotorNonlinearDynamic5,
      &QuadModelv_X.MotorNonlinearDynamic5);

    // End of SystemInitialize for SubSystem: '<S296>/MotorNonlinearDynamic5'

    // SystemInitialize for Atomic SubSystem: '<S296>/MotorNonlinearDynamic6'
    Qua_MotorNonlinearDynamic1_Init(&QuadModelv_DW.MotorNonlinearDynamic6,
      &QuadModelv_X.MotorNonlinearDynamic6);

    // End of SystemInitialize for SubSystem: '<S296>/MotorNonlinearDynamic6'

    // SystemInitialize for Atomic SubSystem: '<S296>/MotorNonlinearDynamic7'
    Qua_MotorNonlinearDynamic1_Init(&QuadModelv_DW.MotorNonlinearDynamic7,
      &QuadModelv_X.MotorNonlinearDynamic7);

    // End of SystemInitialize for SubSystem: '<S296>/MotorNonlinearDynamic7'

    // SystemInitialize for Atomic SubSystem: '<S296>/MotorNonlinearDynamic8'
    Qua_MotorNonlinearDynamic1_Init(&QuadModelv_DW.MotorNonlinearDynamic8,
      &QuadModelv_X.MotorNonlinearDynamic8);

    // End of SystemInitialize for SubSystem: '<S296>/MotorNonlinearDynamic8'

    // SystemInitialize for Enabled SubSystem: '<S524>/Distance into gust (x)'
    QuadMode_Distanceintogustx_Init(&QuadModelv_B.Distanceintogustx_g,
      &QuadModelv_P.Distanceintogustx_g, &QuadModelv_X.Distanceintogustx_g);

    // End of SystemInitialize for SubSystem: '<S524>/Distance into gust (x)'

    // SystemInitialize for Enabled SubSystem: '<S524>/Distance into gust (y)'
    QuadMode_Distanceintogusty_Init(&QuadModelv_B.Distanceintogusty_f,
      &QuadModelv_P.Distanceintogusty_f, &QuadModelv_X.Distanceintogusty_f);

    // End of SystemInitialize for SubSystem: '<S524>/Distance into gust (y)'

    // SystemInitialize for Enabled SubSystem: '<S524>/Distance into gust (z)'
    QuadMode_Distanceintogusty_Init(&QuadModelv_B.Distanceintogustz_b,
      &QuadModelv_P.Distanceintogustz_b, &QuadModelv_X.Distanceintogustz_b);

    // End of SystemInitialize for SubSystem: '<S524>/Distance into gust (z)'

    // SystemInitialize for Enabled SubSystem: '<S148>/Hugw(z)'
    // '<S100>:1:6' hFault=false;
    // '<S100>:1:9' fParam=zeros(20,1);
    // '<S96>:1:6' hFault=false;
    // '<S96>:1:9' fParam=zeros(20,1);
    // '<S97>:1:6' hFault=false;
    // '<S97>:1:9' fParam=zeros(20,1);
    // '<S98>:1:6' hFault=false;
    // '<S98>:1:9' fParam=zeros(20,1);
    // '<S99>:1:6' hFault=false;
    // '<S99>:1:9' fParam=zeros(20,1);
    QuadModelv_Hugwz_Init(&QuadModelv_B.Hugwz_g, &QuadModelv_DW.Hugwz_g,
                          &QuadModelv_P.Hugwz_g);

    // End of SystemInitialize for SubSystem: '<S148>/Hugw(z)'

    // SystemInitialize for Enabled SubSystem: '<S148>/Hvgw(z)'
    QuadModelv_Hvgwz_Init(&QuadModelv_B.Hvgwz_f, &QuadModelv_DW.Hvgwz_f,
                          &QuadModelv_P.Hvgwz_f);

    // End of SystemInitialize for SubSystem: '<S148>/Hvgw(z)'

    // SystemInitialize for Enabled SubSystem: '<S148>/Hwgw(z)'
    QuadModelv_Hwgwz_Init(&QuadModelv_B.Hwgwz_a, &QuadModelv_DW.Hwgwz_a,
                          &QuadModelv_P.Hwgwz_a);

    // End of SystemInitialize for SubSystem: '<S148>/Hwgw(z)'

    // SystemInitialize for MATLAB Function: '<S67>/MATLAB Function'
    std::memcpy(&QuadModelv_DW.state_a[0], &tmp_0[0], 625U * sizeof(uint32_T));
    QuadModelv_DW.method = 7U;
    QuadModelv_DW.state = 1144108930U;
    QuadModelv_DW.state_i[0] = 362436069U;
    QuadModelv_DW.state_i[1] = 521288629U;

    // '<S101>:1:12' t0=0;
    // '<S101>:1:17' isInGust=0;
    // '<S101>:1:22' t1=2;
    QuadModelv_DW.t1 = 2.0;

    // '<S101>:1:27' a=0.5;
    QuadModelv_DW.a = 0.5;

    // '<S101>:1:32' ang=pi/2;
    QuadModelv_DW.ang = 1.5707963267948966;

    // SystemInitialize for Enabled SubSystem: '<S109>/Hugw(z)'
    QuadModelv_Hugwz_Init(&QuadModelv_B.Hugwz, &QuadModelv_DW.Hugwz,
                          &QuadModelv_P.Hugwz);

    // End of SystemInitialize for SubSystem: '<S109>/Hugw(z)'

    // SystemInitialize for Enabled SubSystem: '<S109>/Hvgw(z)'
    QuadModelv_Hvgwz_Init(&QuadModelv_B.Hvgwz, &QuadModelv_DW.Hvgwz,
                          &QuadModelv_P.Hvgwz);

    // End of SystemInitialize for SubSystem: '<S109>/Hvgw(z)'

    // SystemInitialize for Enabled SubSystem: '<S109>/Hwgw(z)'
    QuadModelv_Hwgwz_Init(&QuadModelv_B.Hwgwz, &QuadModelv_DW.Hwgwz,
                          &QuadModelv_P.Hwgwz);

    // End of SystemInitialize for SubSystem: '<S109>/Hwgw(z)'

    // SystemInitialize for Enabled SubSystem: '<S454>/Convert from geodetic to  spherical coordinates ' 
    // SystemInitialize for Iterator SubSystem: '<S464>/For Iterator Subsystem'
    // InitializeConditions for UnitDelay: '<S511>/Unit Delay1'
    // '<S410>:1:6' hFault=false;
    // '<S410>:1:9' fParam=zeros(20,1);
    // '<S411>:1:6' hFault=false;
    // '<S411>:1:9' fParam=zeros(20,1);
    QuadModelv_DW.UnitDelay1_DSTATE[0] =
      QuadModelv_P.UnitDelay1_InitialCondition_d;
    QuadModelv_DW.UnitDelay1_DSTATE[1] =
      QuadModelv_P.UnitDelay1_InitialCondition_d;

    // End of SystemInitialize for SubSystem: '<S464>/For Iterator Subsystem'

    // SystemInitialize for SignalConversion generated from: '<S464>/sp[13]' incorporates:
    //   Outport: '<S464>/sp[13]'

    std::memcpy(&QuadModelv_B.OutportBufferForsp13[0], &QuadModelv_P.sp13_Y0[0],
                13U * sizeof(real_T));

    // SystemInitialize for SignalConversion generated from: '<S464>/cp[13]' incorporates:
    //   Outport: '<S464>/cp[13]'

    std::memcpy(&QuadModelv_B.OutportBufferForcp13[0], &QuadModelv_P.cp13_Y0[0],
                13U * sizeof(real_T));

    // End of SystemInitialize for SubSystem: '<S454>/Convert from geodetic to  spherical coordinates ' 

    // SystemInitialize for Enabled SubSystem: '<S454>/Convert from geodetic to  spherical coordinates' 
    // SystemInitialize for Sqrt: '<S463>/sqrt' incorporates:
    //   Outport: '<S463>/r'

    QuadModelv_B.sqrt_o = QuadModelv_P.r_Y0;

    // SystemInitialize for Product: '<S504>/Product4' incorporates:
    //   Outport: '<S463>/ct'

    QuadModelv_B.Product4_p = QuadModelv_P.ct_Y0;

    // SystemInitialize for Sqrt: '<S510>/sqrt' incorporates:
    //   Outport: '<S463>/st'

    QuadModelv_B.sqrt_d = QuadModelv_P.st_Y0;

    // SystemInitialize for Product: '<S509>/Product12' incorporates:
    //   Outport: '<S463>/sa'

    QuadModelv_B.Product12 = QuadModelv_P.sa_Y0;

    // SystemInitialize for Product: '<S503>/Product11' incorporates:
    //   Outport: '<S463>/ca'

    QuadModelv_B.Product11 = QuadModelv_P.ca_Y0;

    // End of SystemInitialize for SubSystem: '<S454>/Convert from geodetic to  spherical coordinates' 

    // SystemInitialize for Iterator SubSystem: '<S454>/Compute magnetic vector in spherical coordinates' 
    // SystemInitialize for Iterator SubSystem: '<S462>/For Iterator Subsystem'
    // SystemInitialize for Enabled SubSystem: '<S470>/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations' 
    // SystemInitialize for Merge: '<S472>/Merge1'
    QuadModelv_B.Merge1_a = QuadModelv_P.Merge1_InitialOutput;

    // SystemInitialize for Merge: '<S472>/Merge'
    QuadModelv_B.Merge_i = QuadModelv_P.Merge_InitialOutput;

    // End of SystemInitialize for SubSystem: '<S470>/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations' 

    // SystemInitialize for Enabled SubSystem: '<S470>/Time adjust the gauss coefficients' 
    // InitializeConditions for UnitDelay: '<S473>/Unit Delay'
    std::memcpy(&QuadModelv_DW.UnitDelay_DSTATE_k[0],
                &QuadModelv_P.UnitDelay_InitialCondition_b[0], 169U * sizeof
                (real_T));

    // InitializeConditions for UnitDelay: '<S500>/Unit Delay'
    std::memcpy(&QuadModelv_DW.UnitDelay_DSTATE_b[0],
                &QuadModelv_P.UnitDelay_InitialCondition_p[0], 169U * sizeof
                (real_T));

    // SystemInitialize for Sum: '<S473>/Sum2' incorporates:
    //   Outport: '<S473>/tc[13][13]'

    std::memcpy(&QuadModelv_B.Sum2_h[0], &QuadModelv_P.tc1313_Y0[0], 169U *
                sizeof(real_T));

    // End of SystemInitialize for SubSystem: '<S470>/Time adjust the gauss coefficients' 

    // SystemInitialize for Enabled SubSystem: '<S470>/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations' 
    // InitializeConditions for UnitDelay: '<S472>/Unit Delay'
    std::memcpy(&QuadModelv_DW.UnitDelay_DSTATE_i[0],
                &QuadModelv_P.UnitDelay_InitialCondition[0], 169U * sizeof
                (real_T));

    // InitializeConditions for UnitDelay: '<S472>/Unit Delay1'
    std::memcpy(&QuadModelv_DW.UnitDelay1_DSTATE_j[0],
                &QuadModelv_P.UnitDelay1_InitialCondition_j[0], 169U * sizeof
                (real_T));

    // SystemInitialize for Assignment: '<S472>/Assignment' incorporates:
    //   Outport: '<S472>/dp[13][13]'

    std::memcpy(&QuadModelv_B.Assignment_g[0], &QuadModelv_P.dp1313_Y0[0], 169U *
                sizeof(real_T));

    // SystemInitialize for Assignment: '<S472>/Assignment_snorm' incorporates:
    //   Outport: '<S472>/snorm[169]'

    std::memcpy(&QuadModelv_B.Assignment_snorm[0], &QuadModelv_P.snorm169_Y0[0],
                169U * sizeof(real_T));

    // End of SystemInitialize for SubSystem: '<S470>/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations' 

    // SystemInitialize for Enabled SubSystem: '<S471>/Special case - North//South Geographic Pole' 
    // InitializeConditions for UnitDelay: '<S474>/Unit Delay1'
    std::memcpy(&QuadModelv_DW.UnitDelay1_DSTATE_a[0],
                &QuadModelv_P.UnitDelay1_InitialCondition[0], 13U * sizeof
                (real_T));

    // SystemInitialize for IfAction SubSystem: '<S474>/If Action Subsystem1'
    // SystemInitialize for Assignment: '<S478>/Assignment2' incorporates:
    //   Outport: '<S478>/pp[13]'

    std::memcpy(&QuadModelv_B.Assignment2_a[0], &QuadModelv_P.pp13_Y0[0], 13U *
                sizeof(real_T));

    // End of SystemInitialize for SubSystem: '<S474>/If Action Subsystem1'

    // SystemInitialize for IfAction SubSystem: '<S474>/If Action Subsystem2'
    // SystemInitialize for Assignment: '<S479>/Assignment2' incorporates:
    //   Outport: '<S479>/pp[13]'

    std::memcpy(&QuadModelv_B.Assignment2_m[0], &QuadModelv_P.pp13_Y0_n[0], 13U *
                sizeof(real_T));

    // End of SystemInitialize for SubSystem: '<S474>/If Action Subsystem2'

    // SystemInitialize for Product: '<S474>/Product2' incorporates:
    //   Outport: '<S474>/bpp'

    QuadModelv_B.Product2_b = QuadModelv_P.bpp_Y0;

    // End of SystemInitialize for SubSystem: '<S471>/Special case - North//South Geographic Pole' 

    // SystemInitialize for Sum: '<S471>/Sum1' incorporates:
    //   Outport: '<S470>/bt'

    QuadModelv_B.Sum1_f = QuadModelv_P.bt_Y0;

    // SystemInitialize for Sum: '<S471>/Sum2' incorporates:
    //   Outport: '<S470>/bp'

    QuadModelv_B.Sum2_k = QuadModelv_P.bp_Y0;

    // SystemInitialize for Sum: '<S471>/Sum3' incorporates:
    //   Outport: '<S470>/br'

    QuadModelv_B.Sum3 = QuadModelv_P.br_Y0;

    // SystemInitialize for Sum: '<S471>/Sum5' incorporates:
    //   Outport: '<S470>/bpp'

    QuadModelv_B.Sum5 = QuadModelv_P.bpp_Y0_k;

    // End of SystemInitialize for SubSystem: '<S462>/For Iterator Subsystem'

    // SystemInitialize for Sum: '<S462>/Sum1' incorporates:
    //   Outport: '<S462>/bt,bp,br,bpp'

    QuadModelv_B.Sum1_l[0] = QuadModelv_P.btbpbrbpp_Y0[0];
    QuadModelv_B.Sum1_l[1] = QuadModelv_P.btbpbrbpp_Y0[1];
    QuadModelv_B.Sum1_l[2] = QuadModelv_P.btbpbrbpp_Y0[2];
    QuadModelv_B.Sum1_l[3] = QuadModelv_P.btbpbrbpp_Y0[3];

    // End of SystemInitialize for SubSystem: '<S454>/Compute magnetic vector in spherical coordinates' 

    // SystemInitialize for IfAction SubSystem: '<S329>/Negative Trace'
    // '<S412>:1:6' hFault=false;
    // '<S412>:1:9' fParam=zeros(20,1);
    // '<S413>:1:6' hFault=false;
    // '<S413>:1:9' fParam=zeros(20,1);
    QuadModelv_NegativeTrace_Init(&QuadModelv_DW.NegativeTrace_l);

    // End of SystemInitialize for SubSystem: '<S329>/Negative Trace'

    // SystemInitialize for Merge: '<S329>/Merge'
    QuadModelv_B.Merge_f[0] = QuadModelv_P.Merge_InitialOutput_fc[0];
    QuadModelv_B.Merge_f[1] = QuadModelv_P.Merge_InitialOutput_fc[1];
    QuadModelv_B.Merge_f[2] = QuadModelv_P.Merge_InitialOutput_fc[2];
    QuadModelv_B.Merge_f[3] = QuadModelv_P.Merge_InitialOutput_fc[3];

    // SystemInitialize for Enabled SubSystem: '<S108>/Hpgw'
    // '<S378>:1:6' hFault=false;
    // '<S378>:1:9' fParam=zeros(20,1);
    // '<S333>:1:5' xTemp=zeros(8,1);
    // '<S333>:1:10' kk=0;
    // '<S333>:1:15' ss=zeros(6,1);
    QuadModelv_Hpgw_Init(&QuadModelv_B.Hpgw, &QuadModelv_DW.Hpgw,
                         &QuadModelv_P.Hpgw);

    // End of SystemInitialize for SubSystem: '<S108>/Hpgw'

    // SystemInitialize for Enabled SubSystem: '<S108>/Hqgw'
    QuadModelv_Hqgw_Init(&QuadModelv_B.Hqgw, &QuadModelv_DW.Hqgw,
                         &QuadModelv_P.Hqgw);

    // End of SystemInitialize for SubSystem: '<S108>/Hqgw'

    // SystemInitialize for Enabled SubSystem: '<S108>/Hrgw'
    QuadModelv_Hrgw_Init(&QuadModelv_B.Hrgw, &QuadModelv_DW.Hrgw,
                         &QuadModelv_P.Hrgw);

    // End of SystemInitialize for SubSystem: '<S108>/Hrgw'

    // SystemInitialize for Enabled SubSystem: '<S147>/Hpgw'
    QuadModelv_Hpgw_Init(&QuadModelv_B.Hpgw_h, &QuadModelv_DW.Hpgw_h,
                         &QuadModelv_P.Hpgw_h);

    // End of SystemInitialize for SubSystem: '<S147>/Hpgw'

    // SystemInitialize for Enabled SubSystem: '<S147>/Hqgw'
    QuadModelv_Hqgw_Init(&QuadModelv_B.Hqgw_a, &QuadModelv_DW.Hqgw_a,
                         &QuadModelv_P.Hqgw_a);

    // End of SystemInitialize for SubSystem: '<S147>/Hqgw'

    // SystemInitialize for Enabled SubSystem: '<S147>/Hrgw'
    QuadModelv_Hrgw_Init(&QuadModelv_B.Hrgw_a, &QuadModelv_DW.Hrgw_a,
                         &QuadModelv_P.Hrgw_a);

    // End of SystemInitialize for SubSystem: '<S147>/Hrgw'
  }

  // set "at time zero" to false
  if (rtmIsFirstInitCond((&QuadModelv_M))) {
    rtmSetFirstInitCond((&QuadModelv_M), 0);
  }
}

// Model terminate function
void MulticopterModelClass::terminate()
{
  // (no terminate code required)
}

// Constructor
MulticopterModelClass::MulticopterModelClass() :
  QuadModelv_U(),
  QuadModelv_Y(),
  QuadModelv_B(),
  QuadModelv_DW(),
  QuadModelv_X(),
  QuadModelv_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
MulticopterModelClass::~MulticopterModelClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL_QuadModelv_T * MulticopterModelClass::getRTM()
{
  return (&QuadModelv_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
