//
// File: QuadModelv.h
//
// Code generated for Simulink model 'QuadModelv'.
//
// Model version                  : 12.1
// Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
// C/C++ source code generated on : Thu Jun 13 16:02:48 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_QuadModelv_h_
#define RTW_HEADER_QuadModelv_h_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include <cmath>
#include <cstring>
#include <stddef.h>

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

// Forward declaration for rtModel
typedef struct tag_RTM_QuadModelv_T RT_MODEL_QuadModelv_T;

#ifndef DEFINED_TYPEDEF_FOR_MavVehileInfo_
#define DEFINED_TYPEDEF_FOR_MavVehileInfo_

struct MavVehileInfo
{
  int32_T copterID;
  int32_T vehicleType;
  real_T runnedTime;
  real32_T VelE[3];
  real32_T PosE[3];
  real32_T AngEuler[3];
  real32_T AngQuatern[4];
  real32_T MotorRPMS[8];
  real32_T AccB[3];
  real32_T RateB[3];
  real_T PosGPS[3];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_MavLinkGPS_
#define DEFINED_TYPEDEF_FOR_MavLinkGPS_

struct MavLinkGPS
{
  uint32_T time_usec;
  int32_T lat;
  int32_T lon;
  int32_T alt;
  uint16_T eph;
  uint16_T epv;
  uint16_T vel;
  int16_T vn;
  int16_T ve;
  int16_T vd;
  uint16_T cog;
  uint8_T fix_type;
  uint8_T satellites_visible;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_HILGPS_
#define DEFINED_TYPEDEF_FOR_HILGPS_

struct HILGPS
{
  uint8_T fix_type;
  uint8_T satellites_visible;
  int32_T lat;
  int32_T lon;
  int32_T alt;
  uint32_T hAcc;
  uint32_T vAcc;
  int32_T velN;
  int32_T velE;
  int32_T velD;
  int32_T gSpeed;
  int32_T headMot;
  int32_T headVeh;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_MavLinkSensor_
#define DEFINED_TYPEDEF_FOR_MavLinkSensor_

struct MavLinkSensor
{
  uint32_T time_usec;
  real32_T xacc;
  real32_T yacc;
  real32_T zacc;
  real32_T xgyro;
  real32_T ygyro;
  real32_T zgyro;
  real32_T xmag;
  real32_T ymag;
  real32_T zmag;
  real32_T abs_pressure;
  real32_T diff_pressure;
  real32_T pressure_alt;
  real32_T temperature;
  uint32_T fields_updated;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_MavLinkStateQuat_
#define DEFINED_TYPEDEF_FOR_MavLinkStateQuat_

struct MavLinkStateQuat
{
  uint32_T time_usec;
  real32_T attitude_quaternion[4];
  real32_T rollspeed;
  real32_T pitchspeed;
  real32_T yawspeed;
  int32_T lat;
  int32_T lon;
  int32_T alt;
  int16_T vx;
  int16_T vy;
  int16_T vz;
  uint16_T ind_airspeed;
  uint16_T true_airspeed;
  int16_T xacc;
  int16_T yacc;
  int16_T zacc;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_ZU7dDHFykxpuVCSTcMRmtD_
#define DEFINED_TYPEDEF_FOR_struct_ZU7dDHFykxpuVCSTcMRmtD_

struct struct_ZU7dDHFykxpuVCSTcMRmtD
{
  real_T UseCusTomHoverTimeFaultID;
  real_T PowOffFaultID;
  real_T LowVoltageFaultID;
  real_T LowCapacityFaultID;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_rHAf2Ncw266Lr5SdOUZ2fD_
#define DEFINED_TYPEDEF_FOR_struct_rHAf2Ncw266Lr5SdOUZ2fD_

struct struct_rHAf2Ncw266Lr5SdOUZ2fD
{
  real_T ConstWindFaultID;
  real_T GustWindFaultID;
  real_T TurbWindFaultID;
  real_T SheerWindFaultID;
  real_T NoiseWindFaultID;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_cC8C5MZV75wwVdzQ00rR7E_
#define DEFINED_TYPEDEF_FOR_struct_cC8C5MZV75wwVdzQ00rR7E_

struct struct_cC8C5MZV75wwVdzQ00rR7E
{
  real_T FaultInParams[32];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_fWFVDjqhgtZ8Pr52nFAm1E_
#define DEFINED_TYPEDEF_FOR_struct_fWFVDjqhgtZ8Pr52nFAm1E_

struct struct_fWFVDjqhgtZ8Pr52nFAm1E
{
  real_T LoadFallFaultID;
  real_T LoadShiftFaultID;
  real_T LoadLeakFaultID;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_O6zquuSaY7mWQl2ZAiqFYB_
#define DEFINED_TYPEDEF_FOR_struct_O6zquuSaY7mWQl2ZAiqFYB_

struct struct_O6zquuSaY7mWQl2ZAiqFYB
{
  real_T FaultID;
  int32_T MotorNum;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_uC8KEmGrpTOUaEwVQj34DD_
#define DEFINED_TYPEDEF_FOR_struct_uC8KEmGrpTOUaEwVQj34DD_

struct struct_uC8KEmGrpTOUaEwVQj34DD
{
  real_T AccFaultID;
  real_T GyroFaultID;
  real_T MagFaultID;
  real_T BaroFaultID;
  real_T GPSFaultID;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_IIgj0sxnnecCOL2Kz6CFkH_
#define DEFINED_TYPEDEF_FOR_struct_IIgj0sxnnecCOL2Kz6CFkH_

struct struct_IIgj0sxnnecCOL2Kz6CFkH
{
  real_T FaultID;
  int32_T PropNum;
};

#endif

// Block signals for system '<S108>/Hpgw'
struct B_Hpgw_QuadModelv_T {
  real_T UnitDelay[2];                 // '<S119>/Unit Delay'
  real_T Sum[2];                       // '<S119>/Sum'
};

// Block states (default storage) for system '<S108>/Hpgw'
struct DW_Hpgw_QuadModelv_T {
  real_T UnitDelay_DSTATE[2];          // '<S119>/Unit Delay'
  boolean_T Hpgw_MODE;                 // '<S108>/Hpgw'
};

// Block signals for system '<S108>/Hqgw'
struct B_Hqgw_QuadModelv_T {
  real_T dt1;                          // '<S120>/dt1'
  real_T Sum2;                         // '<S120>/Sum2'
  real_T LugV2[2];                     // '<S120>/Lug//V2'
  real_T UnitDelay1[2];                // '<S120>/Unit Delay1'
  real_T Sum1[2];                      // '<S120>/Sum1'
};

// Block states (default storage) for system '<S108>/Hqgw'
struct DW_Hqgw_QuadModelv_T {
  real_T UnitDelay_DSTATE[2];          // '<S120>/Unit Delay'
  real_T UnitDelay1_DSTATE[2];         // '<S120>/Unit Delay1'
  boolean_T Hqgw_MODE;                 // '<S108>/Hqgw'
};

// Block signals for system '<S108>/Hrgw'
struct B_Hrgw_QuadModelv_T {
  real_T dt1;                          // '<S121>/dt1'
  real_T Sum2;                         // '<S121>/Sum2'
  real_T LugV2[2];                     // '<S121>/Lug//V2'
  real_T UnitDelay1[2];                // '<S121>/Unit Delay1'
  real_T Sum1[2];                      // '<S121>/Sum1'
};

// Block states (default storage) for system '<S108>/Hrgw'
struct DW_Hrgw_QuadModelv_T {
  real_T UnitDelay_DSTATE[2];          // '<S121>/Unit Delay'
  real_T UnitDelay1_DSTATE[2];         // '<S121>/Unit Delay1'
  boolean_T Hrgw_MODE;                 // '<S108>/Hrgw'
};

// Block signals for system '<S109>/Hugw(z)'
struct B_Hugwz_QuadModelv_T {
  real_T UnitDelay[2];                 // '<S122>/Unit Delay'
  real_T Sum[2];                       // '<S122>/Sum'
};

// Block states (default storage) for system '<S109>/Hugw(z)'
struct DW_Hugwz_QuadModelv_T {
  real_T UnitDelay_DSTATE[2];          // '<S122>/Unit Delay'
  boolean_T Hugwz_MODE;                // '<S109>/Hugw(z)'
};

// Block signals for system '<S109>/Hvgw(z)'
struct B_Hvgwz_QuadModelv_T {
  real_T UnitDelay[2];                 // '<S123>/Unit Delay'
  real_T Sum[2];                       // '<S123>/Sum'
};

// Block states (default storage) for system '<S109>/Hvgw(z)'
struct DW_Hvgwz_QuadModelv_T {
  real_T UnitDelay_DSTATE[2];          // '<S123>/Unit Delay'
  boolean_T Hvgwz_MODE;                // '<S109>/Hvgw(z)'
};

// Block signals for system '<S109>/Hwgw(z)'
struct B_Hwgwz_QuadModelv_T {
  real_T UnitDelay[2];                 // '<S124>/Unit Delay'
  real_T Sum[2];                       // '<S124>/Sum'
};

// Block states (default storage) for system '<S109>/Hwgw(z)'
struct DW_Hwgwz_QuadModelv_T {
  real_T UnitDelay_DSTATE[2];          // '<S124>/Unit Delay'
  boolean_T Hwgwz_MODE;                // '<S109>/Hwgw(z)'
};

// Block states (default storage) for system '<S193>/Negative Trace'
struct DW_NegativeTrace_QuadModelv_T {
  int8_T FindMaximumDiagonalValue_Active;// '<S201>/Find Maximum Diagonal Value' 
};

// Block signals for system '<S297>/Motor_Dynamics'
struct B_Motor_Dynamics_QuadModelv_T {
  real_T x;                            // '<S306>/Integrator'
  real_T dratedt;                      // '<S306>/Divide'
};

// Block states (default storage) for system '<S297>/Motor_Dynamics'
struct DW_Motor_Dynamics_QuadModelv_T {
  int_T Integrator_IWORK;              // '<S306>/Integrator'
};

// Continuous states for system '<S297>/Motor_Dynamics'
struct X_Motor_Dynamics_QuadModelv_T {
  real_T Integrator_CSTATE;            // '<S306>/Integrator'
};

// State derivatives for system '<S297>/Motor_Dynamics'
struct XDot_Motor_Dynamics_QuadModel_T {
  real_T Integrator_CSTATE;            // '<S306>/Integrator'
};

// State Disabled for system '<S297>/Motor_Dynamics'
struct XDis_Motor_Dynamics_QuadModel_T {
  boolean_T Integrator_CSTATE;         // '<S306>/Integrator'
};

// Block signals for system '<S294>/MotorNonlinearDynamic1'
struct B_MotorNonlinearDynamic1_Quad_T {
  real_T Product;                      // '<S297>/Product'
  B_Motor_Dynamics_QuadModelv_T Motor_Dynamics;// '<S297>/Motor_Dynamics'
};

// Block states (default storage) for system '<S294>/MotorNonlinearDynamic1'
struct DW_MotorNonlinearDynamic1_Qua_T {
  DW_Motor_Dynamics_QuadModelv_T Motor_Dynamics;// '<S297>/Motor_Dynamics'
};

// Continuous states for system '<S294>/MotorNonlinearDynamic1'
struct X_MotorNonlinearDynamic1_Quad_T {
  X_Motor_Dynamics_QuadModelv_T Motor_Dynamics;// '<S297>/Motor_Dynamics'
};

// State derivatives for system '<S294>/MotorNonlinearDynamic1'
struct XDot_MotorNonlinearDynamic1_Q_T {
  XDot_Motor_Dynamics_QuadModel_T Motor_Dynamics;// '<S297>/Motor_Dynamics'
};

// State Disabled for system '<S294>/MotorNonlinearDynamic1'
struct XDis_MotorNonlinearDynamic1_Q_T {
  XDis_Motor_Dynamics_QuadModel_T Motor_Dynamics;// '<S297>/Motor_Dynamics'
};

// Block signals for system '<S512>/Distance into gust (y)'
struct B_Distanceintogusty_QuadModel_T {
  real_T DistanceintoGustxLimitedtogustl;
                 // '<S516>/Distance into Gust (x) (Limited to gust length d) '
};

// Block states (default storage) for system '<S512>/Distance into gust (y)'
struct DW_Distanceintogusty_QuadMode_T {
  boolean_T Distanceintogusty_MODE;    // '<S512>/Distance into gust (y)'
};

// Continuous states for system '<S512>/Distance into gust (y)'
struct X_Distanceintogusty_QuadModel_T {
  real_T DistanceintoGustxLimitedtogustl;
                 // '<S516>/Distance into Gust (x) (Limited to gust length d) '
};

// State derivatives for system '<S512>/Distance into gust (y)'
struct XDot_Distanceintogusty_QuadMo_T {
  real_T DistanceintoGustxLimitedtogustl;
                 // '<S516>/Distance into Gust (x) (Limited to gust length d) '
};

// State Disabled for system '<S512>/Distance into gust (y)'
struct XDis_Distanceintogusty_QuadMo_T {
  boolean_T DistanceintoGustxLimitedtogustl;
                 // '<S516>/Distance into Gust (x) (Limited to gust length d) '
};

// Block signals (default storage)
struct B_QuadModelv_T {
  real_T q0;                           // '<S25>/q0'
  real_T q1;                           // '<S25>/q1'
  real_T q2;                           // '<S25>/q2'
  real_T q3;                           // '<S25>/q3'
  real_T m;                            // '<S6>/Add5'
  real_T SinCos_o1;                    // '<S70>/SinCos'
  real_T SinCos_o2;                    // '<S70>/SinCos'
  real_T Switch;                       // '<S78>/Switch'
  real_T TrigonometricFunction1;       // '<S84>/Trigonometric Function1'
  real_T TrigonometricFunction2;       // '<S84>/Trigonometric Function2'
  real_T Switch_a;                     // '<S79>/Switch'
  real_T MatrixConcatenate[3];         // '<S66>/Matrix Concatenate'
  real_T Product[3];                   // '<S22>/Product'
  real_T Merge[4];                     // '<S194>/Merge'
  real_T Merge_c[4];                   // '<S193>/Merge'
  real_T DataTypeConversion2[20];      // '<Root>/Data Type Conversion2'
  real_T ModelInit_RPM;                // '<S294>/ModelInit_RPM'
  real_T MotorFault3;                  // '<S2>/MotorFault3'
  real_T Gain;                         // '<S510>/Gain'
  real_T UniformRandomNumber[3];       // '<S67>/Uniform Random Number'
  real_T Sum1[3];                      // '<S67>/Sum1'
  real_T UnitConversion;               // '<S155>/Unit Conversion'
  real_T UnitConversion_g;             // '<S184>/Unit Conversion'
  real_T sigma_wg;                     // '<S165>/sigma_wg '
  real_T PreLookUpIndexSearchprobofexcee;
                          // '<S164>/PreLook-Up Index Search  (prob of exceed)'
  real_T Product_d[4];                 // '<S157>/Product'
  real_T UnitConversion_p;             // '<S146>/Unit Conversion'
  real_T UnitConversion_a;             // '<S116>/Unit Conversion'
  real_T UnitConversion_n;             // '<S145>/Unit Conversion'
  real_T sigma_wg_p;                   // '<S126>/sigma_wg '
  real_T PreLookUpIndexSearchprobofexc_i;
                          // '<S125>/PreLook-Up Index Search  (prob of exceed)'
  real_T Product_p[4];                 // '<S118>/Product'
  real_T UnitConversion_ab;            // '<S107>/Unit Conversion'
  real_T lnref_heightz0;               // '<S105>/ln(ref_height//z0)'
  real_T Windspeedatreferenceheight[3];
                                     // '<S105>/Wind speed at reference height'
  real_T Gain_f[3];                    // '<S415>/Gain'
  real_T Selector1[9];                 // '<S16>/Selector1'
  real_T Selector[9];                  // '<S16>/Selector'
  real_T Selector2[9];                 // '<S16>/Selector2'
  real_T Product2[3];                  // '<S16>/Product2'
  real_T Switch_p;                     // '<S445>/Switch'
  real_T Switch_l;                     // '<S444>/Switch'
  real_T olat;                         // '<S454>/olat'
  real_T Gain_f0;                      // '<S411>/Gain'
  real_T oalt;                         // '<S454>/oalt'
  real_T Product4;                     // '<S406>/Product4'
  real_T Gain_k;                       // '<S398>/Gain'
  real_T Product4_h;                   // '<S407>/Product4'
  real_T Merge_f[4];                   // '<S321>/Merge'
  real_T SinCos_o1_m;                  // '<S374>/SinCos'
  real_T SinCos_o2_i;                  // '<S374>/SinCos'
  real_T Switch_h;                     // '<S382>/Switch'
  real_T TrigonometricFunction1_d;     // '<S388>/Trigonometric Function1'
  real_T TrigonometricFunction2_i;     // '<S388>/Trigonometric Function2'
  real_T Switch_o;                     // '<S383>/Switch'
  real_T SumofElements[3];             // '<S53>/Sum of Elements'
  real_T Sum_p[3];                     // '<S14>/Sum'
  real_T TmpSignalConversionAtq0q1q2q3_c[4];// '<S15>/qdot'
  real_T UnitConversion_e;             // '<S111>/Unit Conversion'
  real_T UnitConversion_f;             // '<S150>/Unit Conversion'
  real_T BiasGain1[3];                 // '<S322>/BiasGain1'
  real_T Sum2[3];                      // '<S421>/Sum2'
  real_T Sum2_f[3];                    // '<S432>/Sum2'
  real_T y[8];                         // '<S510>/PropFaultModel'
  real_T DistanceintoGustxLimitedtogustl;
                  // '<S515>/Distance into Gust (x) (Limited to gust length d)'
  real_T y_p;                          // '<S398>/baro NoiseFun'
  real_T Gain_i[11];                   // '<S452>/Gain'
  real_T Gain1[11];                    // '<S452>/Gain1'
  real_T OutportBufferForcp13[13];
  real_T OutportBufferForsp13[13];
  real_T b2;                           // '<S451>/b2'
  real_T a2;                           // '<S451>/a2'
  real_T c2;                           // '<S494>/Sum1'
  real_T a4;                           // '<S496>/a4'
  real_T c4;                           // '<S496>/Sum9'
  real_T sqrt_o;                       // '<S451>/sqrt'
  real_T Product11;                    // '<S491>/Product11'
  real_T Product4_p;                   // '<S492>/Product4'
  real_T c2_h;                         // '<S497>/Sum1'
  real_T Product12;                    // '<S497>/Product12'
  real_T sqrt_d;                       // '<S498>/sqrt'
  real_T Sum1_l[4];                    // '<S450>/Sum1'
  real_T Sum1_f;                       // '<S459>/Sum1'
  real_T Sum2_k;                       // '<S459>/Sum2'
  real_T Sum3;                         // '<S459>/Sum3'
  real_T Sum5;                         // '<S459>/Sum5'
  real_T Assignment[169];              // '<S461>/Assignment'
  real_T Sum2_h[169];                  // '<S461>/Sum2'
  real_T Assignment2[169];             // '<S489>/Assignment2'
  real_T Merge1_a;                     // '<S460>/Merge1'
  real_T Assignment_g[169];            // '<S460>/Assignment'
  real_T Merge_i;                      // '<S460>/Merge'
  real_T Assignment_snorm[169];        // '<S460>/Assignment_snorm'
  real_T Product2_b;                   // '<S462>/Product2'
  real_T Assignment2_m[13];            // '<S467>/Assignment2'
  real_T Assignment2_a[13];            // '<S466>/Assignment2'
  real_T satellites;                   // '<S366>/Acc NoiseFun'
  real_T GPS3Dfixed;                   // '<S366>/Acc NoiseFun'
  real_T y_k[8];                       // '<S293>/MotorFaultModel'
  real_T y_m;                          // '<Root>/MATLAB Function'
  real_T F1[3];                        // '<S7>/Ground Model'
  real_T M1[3];                        // '<S7>/Ground Model'
  real_T uavlostMass;                  // '<S6>/Model Failure'
  real_T deltaM[3];                    // '<S6>/Model Failure'
  real_T gWind[3];                     // '<S67>/MATLAB Function'
  real_T FaultParam_a[20];             // '<S67>/FaultParamsExtract3'
  real_T FaultParam_i[20];             // '<S67>/FaultParamsExtract2'
  real_T FaultParam_g[20];             // '<S67>/FaultParamsExtract1'
  real_T FaultParam_p[20];             // '<S67>/FaultParamsExtract'
  real_T y_l[8];                       // '<Root>/Crash_Motor_Fault'
  real_T fm[6];                        // '<Root>/CollisionDetection'
  real_T Merge_e2;                     // '<S42>/Merge'
  uint32_T PreLookUpIndexSearchprobofexc_p;
                          // '<S164>/PreLook-Up Index Search  (prob of exceed)'
  uint32_T PreLookUpIndexSearchprobofexc_n;
                          // '<S125>/PreLook-Up Index Search  (prob of exceed)'
  uint32_T fields_updated;             // '<S394>/Data Type Conversion6'
  uint32_T hAcc;                       // '<S322>/Data Type Conversion14'
  uint32_T vAcc;                       // '<S322>/Data Type Conversion15'
  real32_T DataTypeConversion2_i[3];   // '<S398>/Data Type Conversion2'
  real32_T DataTypeConversion3[3];     // '<S398>/Data Type Conversion3'
  real32_T DataTypeConversion4[3];     // '<S398>/Data Type Conversion4'
  real32_T MotorRPMS[8];               // '<S12>/Data Type Conversion8'
  int32_T inSILInts[8];                // '<Root>/Data Type Conversion3'
  int32_T lat;                         // '<S322>/Data Type Conversion4'
  int32_T lon;                         // '<S322>/Data Type Conversion5'
  int32_T alt;                         // '<S322>/Data Type Conversion6'
  int32_T copterID;                    // '<S12>/Data Type Conversion'
  int32_T vehicleType;                 // '<S12>/Data Type Conversion2'
  uint16_T eph;                        // '<S322>/Data Type Conversion8'
  uint16_T epv;                        // '<S322>/Data Type Conversion9'
  int16_T DataTypeConversion8[3];      // '<S324>/Data Type Conversion8'
  uint8_T fix_type;                    // '<S322>/Data Type Conversion10'
  uint8_T satellites_visible;          // '<S322>/Data Type Conversion11'
  int8_T isCrash;                      // '<Root>/CollisionDetection'
  boolean_T HiddenBuf_InsertedFor_Distanc_c;// '<S512>/Logical Operator1'
  boolean_T HiddenBuf_InsertedFor_Distanc_m;// '<S512>/Logical Operator3'
  boolean_T hasFault_SheerWind;        // '<S67>/FaultParamsExtract3'
  boolean_T hasFault_TurbWind;         // '<S67>/FaultParamsExtract2'
  boolean_T hasFault_GustWind;         // '<S67>/FaultParamsExtract1'
  boolean_T hasFault_ConstWind;        // '<S67>/FaultParamsExtract'
  B_Distanceintogusty_QuadModel_T Distanceintogustz;// '<S512>/Distance into gust (z)' 
  B_Distanceintogusty_QuadModel_T Distanceintogusty;// '<S512>/Distance into gust (y)' 
  B_MotorNonlinearDynamic1_Quad_T MotorNonlinearDynamic8;// '<S294>/MotorNonlinearDynamic8' 
  B_MotorNonlinearDynamic1_Quad_T MotorNonlinearDynamic7;// '<S294>/MotorNonlinearDynamic7' 
  B_MotorNonlinearDynamic1_Quad_T MotorNonlinearDynamic6;// '<S294>/MotorNonlinearDynamic6' 
  B_MotorNonlinearDynamic1_Quad_T MotorNonlinearDynamic5;// '<S294>/MotorNonlinearDynamic5' 
  B_MotorNonlinearDynamic1_Quad_T MotorNonlinearDynamic4;// '<S294>/MotorNonlinearDynamic4' 
  B_MotorNonlinearDynamic1_Quad_T MotorNonlinearDynamic3;// '<S294>/MotorNonlinearDynamic3' 
  B_MotorNonlinearDynamic1_Quad_T MotorNonlinearDynamic2;// '<S294>/MotorNonlinearDynamic2' 
  B_MotorNonlinearDynamic1_Quad_T MotorNonlinearDynamic1;// '<S294>/MotorNonlinearDynamic1' 
  B_Hwgwz_QuadModelv_T Hwgwz_a;        // '<S148>/Hwgw(z)'
  B_Hvgwz_QuadModelv_T Hvgwz_f;        // '<S148>/Hvgw(z)'
  B_Hugwz_QuadModelv_T Hugwz_g;        // '<S148>/Hugw(z)'
  B_Hrgw_QuadModelv_T Hrgw_a;          // '<S147>/Hrgw'
  B_Hqgw_QuadModelv_T Hqgw_a;          // '<S147>/Hqgw'
  B_Hpgw_QuadModelv_T Hpgw_h;          // '<S147>/Hpgw'
  B_Hwgwz_QuadModelv_T Hwgwz;          // '<S109>/Hwgw(z)'
  B_Hvgwz_QuadModelv_T Hvgwz;          // '<S109>/Hvgw(z)'
  B_Hugwz_QuadModelv_T Hugwz;          // '<S109>/Hugw(z)'
  B_Hrgw_QuadModelv_T Hrgw;            // '<S108>/Hrgw'
  B_Hqgw_QuadModelv_T Hqgw;            // '<S108>/Hqgw'
  B_Hpgw_QuadModelv_T Hpgw;            // '<S108>/Hpgw'
};

// Block states (default storage) for system '<Root>'
struct DW_QuadModelv_T {
  real_T UnitDelay1_DSTATE[2];         // '<S499>/Unit Delay1'
  real_T UnitDelay_DSTATE_k[169];      // '<S461>/Unit Delay'
  real_T UnitDelay_DSTATE_b[169];      // '<S488>/Unit Delay'
  real_T UnitDelay_DSTATE_i[169];      // '<S460>/Unit Delay'
  real_T UnitDelay1_DSTATE_j[169];     // '<S460>/Unit Delay1'
  real_T UnitDelay1_DSTATE_a[13];      // '<S462>/Unit Delay1'
  real_T WGS84GravitySFunction_h;      // '<S66>/WGS84 Gravity S-Function'
  real_T WGS84GravitySFunction_phi;    // '<S66>/WGS84 Gravity S-Function'
  real_T WGS84GravitySFunction_lambda; // '<S66>/WGS84 Gravity S-Function'
  real_T WGS84GravitySFunction_gamma_h;// '<S66>/WGS84 Gravity S-Function'
  real_T WGS84GravitySFunction_gamma_phi;// '<S66>/WGS84 Gravity S-Function'
  real_T Memory_PreviousInput[8];      // '<Root>/Memory'
  real_T NextOutput;                   // '<S511>/White Noise'
  real_T UniformRandomNumber_NextOutput[3];// '<S67>/Uniform Random Number'
  real_T NextOutput_c[4];              // '<S157>/White Noise'
  real_T NextOutput_p[4];              // '<S118>/White Noise'
  real_T Product2_DWORK4[9];           // '<S16>/Product2'
  real_T otime_PreviousInput;          // '<S456>/otime'
  real_T olon_PreviousInput;           // '<S455>/olon'
  real_T olat_PreviousInput;           // '<S454>/olat'
  real_T oalt_PreviousInput;           // '<S454>/oalt'
  real_T UniformRandomNumber_NextOutpu_l;// '<S398>/Uniform Random Number'
  real_T UniformRandomNumber4_NextOutput;// '<S398>/Uniform Random Number4'
  real_T UniformRandomNumber4_NextOutp_p[3];// '<S322>/Uniform Random Number4'
  real_T fParam[20];                   // '<S510>/FaultParamsExtract'
  real_T xTemp[8];                     // '<S12>/Model Fail Assessment'
  real_T kk;                           // '<S12>/Model Fail Assessment'
  real_T ss[6];                        // '<S12>/Model Fail Assessment'
  real_T V2;                           // '<S398>/baro NoiseFun'
  real_T S;                            // '<S398>/baro NoiseFun'
  real_T V2_i;                         // '<S398>/Mag NoiseFun'
  real_T S_g;                          // '<S398>/Mag NoiseFun'
  real_T V2_e;                         // '<S398>/Gyro NoiseFun'
  real_T S_k;                          // '<S398>/Gyro NoiseFun'
  real_T fParam_n[20];                 // '<S398>/FaultParamsExtract3'
  real_T fParam_p[20];                 // '<S398>/FaultParamsExtract2'
  real_T fParam_i[20];                 // '<S398>/FaultParamsExtract1'
  real_T fParam_k[20];                 // '<S398>/FaultParamsExtract'
  real_T V2_j;                         // '<S398>/Acc NoiseFun'
  real_T S_m;                          // '<S398>/Acc NoiseFun'
  real_T fParam_d[20];                 // '<S366>/FaultParamsExtract'
  real_T V2_o;                         // '<S366>/Acc NoiseFun'
  real_T S_k1;                         // '<S366>/Acc NoiseFun'
  real_T V2_jb;                        // '<S293>/MotorFaultModel'
  real_T S_kr;                         // '<S293>/MotorFaultModel'
  real_T fParam_a[20];                 // '<S293>/FaultParamsExtract'
  real_T tOld;                         // '<S6>/Model Failure'
  real_T fParam_p1[20];                // '<S187>/FaultParamsExtract3'
  real_T fParam_an[20];                // '<S187>/FaultParamsExtract2'
  real_T fParam_ai[20];                // '<S187>/FaultParamsExtract1'
  real_T t0;                           // '<S67>/MATLAB Function'
  real_T isInGust;                     // '<S67>/MATLAB Function'
  real_T t1;                           // '<S67>/MATLAB Function'
  real_T a;                            // '<S67>/MATLAB Function'
  real_T ang;                          // '<S67>/MATLAB Function'
  real_T wlast;                        // '<S67>/MATLAB Function'
  real_T wNow;                         // '<S67>/MATLAB Function'
  real_T fParam_de[20];                // '<S67>/FaultParamsExtract4'
  real_T fParam_h[20];                 // '<S67>/FaultParamsExtract3'
  real_T fParam_l[20];                 // '<S67>/FaultParamsExtract2'
  real_T fParam_ae[20];                // '<S67>/FaultParamsExtract1'
  real_T fParam_la[20];                // '<S67>/FaultParamsExtract'
  real_T fOut[6];                      // '<Root>/CollisionDetection'
  real_T mv0[3];                       // '<Root>/CollisionDetection'
  real_T tColi;                        // '<Root>/CollisionDetection'
  real_T discharge;                    // '<S2>/MATLAB Function'
  real_T startTime;                    // '<S2>/MATLAB Function'
  real_T reset;                        // '<S2>/MATLAB Function'
  real_T fParam_f[20];                 // '<S57>/FaultParamsExtract3'
  real_T fParam_iy[20];                // '<S57>/FaultParamsExtract2'
  real_T fParam_j[20];                 // '<S57>/FaultParamsExtract'
  uint32_T RandSeed;                   // '<S511>/White Noise'
  uint32_T RandSeed_o[3];              // '<S67>/Uniform Random Number'
  uint32_T PreLookUpIndexSearchaltitude_DW;
                                // '<S164>/PreLook-Up Index Search  (altitude)'
  uint32_T PreLookUpIndexSearchprobofexcee;
                          // '<S164>/PreLook-Up Index Search  (prob of exceed)'
  uint32_T RandSeed_p[4];              // '<S157>/White Noise'
  uint32_T PreLookUpIndexSearchaltitude__b;
                                // '<S125>/PreLook-Up Index Search  (altitude)'
  uint32_T PreLookUpIndexSearchprobofexc_l;
                          // '<S125>/PreLook-Up Index Search  (prob of exceed)'
  uint32_T RandSeed_or[4];             // '<S118>/White Noise'
  uint32_T RandSeed_e;                 // '<S398>/Uniform Random Number'
  uint32_T RandSeed_b;                 // '<S398>/Uniform Random Number4'
  uint32_T RandSeed_n[3];              // '<S322>/Uniform Random Number4'
  uint32_T method;                     // '<S398>/baro NoiseFun'
  uint32_T state;                      // '<S398>/baro NoiseFun'
  uint32_T state_a[2];                 // '<S398>/baro NoiseFun'
  uint32_T state_i[625];               // '<S398>/baro NoiseFun'
  uint32_T method_d;                   // '<S398>/Mag NoiseFun'
  uint32_T state_g;                    // '<S398>/Mag NoiseFun'
  uint32_T state_j[2];                 // '<S398>/Mag NoiseFun'
  uint32_T state_b[625];               // '<S398>/Mag NoiseFun'
  uint32_T method_l;                   // '<S398>/Gyro NoiseFun'
  uint32_T state_e;                    // '<S398>/Gyro NoiseFun'
  uint32_T state_c[2];                 // '<S398>/Gyro NoiseFun'
  uint32_T state_n[625];               // '<S398>/Gyro NoiseFun'
  uint32_T method_j;                   // '<S398>/Acc NoiseFun'
  uint32_T state_d;                    // '<S398>/Acc NoiseFun'
  uint32_T state_k[2];                 // '<S398>/Acc NoiseFun'
  uint32_T state_f[625];               // '<S398>/Acc NoiseFun'
  uint32_T method_dt;                  // '<S366>/Acc NoiseFun'
  uint32_T state_ee;                   // '<S366>/Acc NoiseFun'
  uint32_T state_cw[2];                // '<S366>/Acc NoiseFun'
  uint32_T state_p[625];               // '<S366>/Acc NoiseFun'
  uint32_T method_l1;                  // '<S293>/MotorFaultModel'
  uint32_T state_ib;                   // '<S293>/MotorFaultModel'
  uint32_T state_po[2];                // '<S293>/MotorFaultModel'
  uint32_T state_jf[625];              // '<S293>/MotorFaultModel'
  uint32_T method_e;                   // '<S67>/MATLAB Function'
  uint32_T state_ny;                   // '<S67>/MATLAB Function'
  uint32_T state_km[2];                // '<S67>/MATLAB Function'
  uint32_T state_iu[625];              // '<S67>/MATLAB Function'
  uint32_T method_b;                   // '<Root>/Crash_Motor_Fault'
  uint32_T state_h;                    // '<Root>/Crash_Motor_Fault'
  uint32_T state_ef[2];                // '<Root>/Crash_Motor_Fault'
  uint32_T state_m[625];               // '<Root>/Crash_Motor_Fault'
  uint32_T method_do;                  // '<Root>/CollisionDetection'
  uint32_T state_g3;                   // '<Root>/CollisionDetection'
  uint32_T state_pv[2];                // '<Root>/CollisionDetection'
  uint32_T state_jn[625];              // '<Root>/CollisionDetection'
  int_T q0q1q2q3_IWORK;                // '<S15>/q0 q1 q2 q3'
  int_T IntegratorSecondOrderLimited_MO[3];
                                   // '<S421>/Integrator, Second-Order Limited'
  int_T IntegratorSecondOrderLimited__n[3];
                                   // '<S432>/Integrator, Second-Order Limited'
  int8_T If_ActiveSubsystem;           // '<S42>/If'
  int8_T If_ActiveSubsystem_h;         // '<S194>/If'
  int8_T If_ActiveSubsystem_l;         // '<S193>/If'
  int8_T ifHeightMaxlowaltitudeelseifHei;
  // '<S153>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' 
  int8_T ifHeightMaxlowaltitudeelseifH_p;
  // '<S114>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' 
  int8_T If_ActiveSubsystem_m;         // '<S321>/If'
  int8_T ifHeightMaxlowaltitudeelseifH_j;
  // '<S113>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' 
  int8_T ifHeightMaxlowaltitudeelseifH_e;
  // '<S152>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' 
  int8_T If1_ActiveSubsystem;          // '<S203>/If1'
  int8_T If1_ActiveSubsystem_o;        // '<S241>/If1'
  int8_T If1_ActiveSubsystem_d;        // '<S329>/If1'
  int8_T takeoffFlag;                  // '<S7>/Ground Model'
  int8_T landFlag;                     // '<S7>/Ground Model'
  int8_T isCol;                        // '<Root>/CollisionDetection'
  boolean_T IntegratorSecondOrderLimited_DW;
                                   // '<S421>/Integrator, Second-Order Limited'
  boolean_T IntegratorSecondOrderLimited__m;
                                   // '<S432>/Integrator, Second-Order Limited'
  boolean_T hFault;                    // '<S510>/FaultParamsExtract'
  boolean_T phase;                     // '<S398>/baro NoiseFun'
  boolean_T phase_b;                   // '<S398>/Mag NoiseFun'
  boolean_T phase_a;                   // '<S398>/Gyro NoiseFun'
  boolean_T hFault_a;                  // '<S398>/FaultParamsExtract3'
  boolean_T hFault_g;                  // '<S398>/FaultParamsExtract2'
  boolean_T hFault_l;                  // '<S398>/FaultParamsExtract1'
  boolean_T hFault_gx;                 // '<S398>/FaultParamsExtract'
  boolean_T phase_br;                  // '<S398>/Acc NoiseFun'
  boolean_T hFault_lp;                 // '<S366>/FaultParamsExtract'
  boolean_T phase_l;                   // '<S366>/Acc NoiseFun'
  boolean_T phase_o;                   // '<S293>/MotorFaultModel'
  boolean_T hFault_o;                  // '<S293>/FaultParamsExtract'
  boolean_T hFault_j;                  // '<S187>/FaultParamsExtract3'
  boolean_T hFault_n;                  // '<S187>/FaultParamsExtract2'
  boolean_T hFault_om;                 // '<S187>/FaultParamsExtract1'
  boolean_T wlast_not_empty;           // '<S67>/MATLAB Function'
  boolean_T wNow_not_empty;            // '<S67>/MATLAB Function'
  boolean_T hFault_ak;                 // '<S67>/FaultParamsExtract4'
  boolean_T hFault_b;                  // '<S67>/FaultParamsExtract3'
  boolean_T hFault_m;                  // '<S67>/FaultParamsExtract2'
  boolean_T hFault_i;                  // '<S67>/FaultParamsExtract1'
  boolean_T hFault_ax;                 // '<S67>/FaultParamsExtract'
  boolean_T hFault_d;                  // '<S57>/FaultParamsExtract3'
  boolean_T hFault_mv;                 // '<S57>/FaultParamsExtract2'
  boolean_T hFault_g5;                 // '<S57>/FaultParamsExtract1'
  boolean_T hFault_i0;                 // '<S57>/FaultParamsExtract'
  boolean_T Distanceintogustx_MODE;    // '<S512>/Distance into gust (x)'
  boolean_T Convertfromgeodetictosphericalc;
                   // '<S442>/Convert from geodetic to  spherical coordinates '
  boolean_T Convertfromgeodetictospherica_g;
                    // '<S442>/Convert from geodetic to  spherical coordinates'
  boolean_T SpecialcaseNorthSouthGeographic;
                        // '<S459>/Special case - North//South Geographic Pole'
  DW_Distanceintogusty_QuadMode_T Distanceintogustz;// '<S512>/Distance into gust (z)' 
  DW_Distanceintogusty_QuadMode_T Distanceintogusty;// '<S512>/Distance into gust (y)' 
  DW_NegativeTrace_QuadModelv_T NegativeTrace_l;// '<S321>/Negative Trace'
  DW_MotorNonlinearDynamic1_Qua_T MotorNonlinearDynamic8;// '<S294>/MotorNonlinearDynamic8' 
  DW_MotorNonlinearDynamic1_Qua_T MotorNonlinearDynamic7;// '<S294>/MotorNonlinearDynamic7' 
  DW_MotorNonlinearDynamic1_Qua_T MotorNonlinearDynamic6;// '<S294>/MotorNonlinearDynamic6' 
  DW_MotorNonlinearDynamic1_Qua_T MotorNonlinearDynamic5;// '<S294>/MotorNonlinearDynamic5' 
  DW_MotorNonlinearDynamic1_Qua_T MotorNonlinearDynamic4;// '<S294>/MotorNonlinearDynamic4' 
  DW_MotorNonlinearDynamic1_Qua_T MotorNonlinearDynamic3;// '<S294>/MotorNonlinearDynamic3' 
  DW_MotorNonlinearDynamic1_Qua_T MotorNonlinearDynamic2;// '<S294>/MotorNonlinearDynamic2' 
  DW_MotorNonlinearDynamic1_Qua_T MotorNonlinearDynamic1;// '<S294>/MotorNonlinearDynamic1' 
  DW_NegativeTrace_QuadModelv_T NegativeTrace_d;// '<S194>/Negative Trace'
  DW_NegativeTrace_QuadModelv_T NegativeTrace;// '<S193>/Negative Trace'
  DW_Hwgwz_QuadModelv_T Hwgwz_a;       // '<S148>/Hwgw(z)'
  DW_Hvgwz_QuadModelv_T Hvgwz_f;       // '<S148>/Hvgw(z)'
  DW_Hugwz_QuadModelv_T Hugwz_g;       // '<S148>/Hugw(z)'
  DW_Hrgw_QuadModelv_T Hrgw_a;         // '<S147>/Hrgw'
  DW_Hqgw_QuadModelv_T Hqgw_a;         // '<S147>/Hqgw'
  DW_Hpgw_QuadModelv_T Hpgw_h;         // '<S147>/Hpgw'
  DW_Hwgwz_QuadModelv_T Hwgwz;         // '<S109>/Hwgw(z)'
  DW_Hvgwz_QuadModelv_T Hvgwz;         // '<S109>/Hvgw(z)'
  DW_Hugwz_QuadModelv_T Hugwz;         // '<S109>/Hugw(z)'
  DW_Hrgw_QuadModelv_T Hrgw;           // '<S108>/Hrgw'
  DW_Hqgw_QuadModelv_T Hqgw;           // '<S108>/Hqgw'
  DW_Hpgw_QuadModelv_T Hpgw;           // '<S108>/Hpgw'
};

// Continuous states (default storage)
struct X_QuadModelv_T {
  real_T IntegratorSecondOrderLimited_CS[6];
                                   // '<S421>/Integrator, Second-Order Limited'
  real_T q0q1q2q3_CSTATE[4];           // '<S15>/q0 q1 q2 q3'
  real_T xeyeze_CSTATE[3];             // '<S14>/xe,ye,ze'
  real_T pqr_CSTATE[3];                // '<S14>/p,q,r '
  real_T ubvbwb_CSTATE[3];             // '<S14>/ub,vb,wb'
  real_T IntegratorSecondOrderLimited__i[6];
                                   // '<S432>/Integrator, Second-Order Limited'
  real_T TransferFcn4_CSTATE;          // '<S368>/Transfer Fcn4'
  real_T TransferFcn1_CSTATE;          // '<S368>/Transfer Fcn1'
  real_T TransferFcn2_CSTATE;          // '<S368>/Transfer Fcn2'
  X_Distanceintogusty_QuadModel_T Distanceintogustz;// '<S512>/Distance into gust (y)' 
  X_Distanceintogusty_QuadModel_T Distanceintogusty;// '<S512>/Distance into gust (y)' 
  real_T DistanceintoGustxLimitedtogus_e;
                  // '<S515>/Distance into Gust (x) (Limited to gust length d)'
  X_MotorNonlinearDynamic1_Quad_T MotorNonlinearDynamic8;// '<S294>/MotorNonlinearDynamic1' 
  X_MotorNonlinearDynamic1_Quad_T MotorNonlinearDynamic7;// '<S294>/MotorNonlinearDynamic1' 
  X_MotorNonlinearDynamic1_Quad_T MotorNonlinearDynamic6;// '<S294>/MotorNonlinearDynamic1' 
  X_MotorNonlinearDynamic1_Quad_T MotorNonlinearDynamic5;// '<S294>/MotorNonlinearDynamic1' 
  X_MotorNonlinearDynamic1_Quad_T MotorNonlinearDynamic4;// '<S294>/MotorNonlinearDynamic1' 
  X_MotorNonlinearDynamic1_Quad_T MotorNonlinearDynamic3;// '<S294>/MotorNonlinearDynamic1' 
  X_MotorNonlinearDynamic1_Quad_T MotorNonlinearDynamic2;// '<S294>/MotorNonlinearDynamic1' 
  X_MotorNonlinearDynamic1_Quad_T MotorNonlinearDynamic1;// '<S294>/MotorNonlinearDynamic1' 
};

// State derivatives (default storage)
struct XDot_QuadModelv_T {
  real_T IntegratorSecondOrderLimited_CS[6];
                                   // '<S421>/Integrator, Second-Order Limited'
  real_T q0q1q2q3_CSTATE[4];           // '<S15>/q0 q1 q2 q3'
  real_T xeyeze_CSTATE[3];             // '<S14>/xe,ye,ze'
  real_T pqr_CSTATE[3];                // '<S14>/p,q,r '
  real_T ubvbwb_CSTATE[3];             // '<S14>/ub,vb,wb'
  real_T IntegratorSecondOrderLimited__i[6];
                                   // '<S432>/Integrator, Second-Order Limited'
  real_T TransferFcn4_CSTATE;          // '<S368>/Transfer Fcn4'
  real_T TransferFcn1_CSTATE;          // '<S368>/Transfer Fcn1'
  real_T TransferFcn2_CSTATE;          // '<S368>/Transfer Fcn2'
  XDot_Distanceintogusty_QuadMo_T Distanceintogustz;// '<S512>/Distance into gust (y)' 
  XDot_Distanceintogusty_QuadMo_T Distanceintogusty;// '<S512>/Distance into gust (y)' 
  real_T DistanceintoGustxLimitedtogus_e;
                  // '<S515>/Distance into Gust (x) (Limited to gust length d)'
  XDot_MotorNonlinearDynamic1_Q_T MotorNonlinearDynamic8;// '<S294>/MotorNonlinearDynamic1' 
  XDot_MotorNonlinearDynamic1_Q_T MotorNonlinearDynamic7;// '<S294>/MotorNonlinearDynamic1' 
  XDot_MotorNonlinearDynamic1_Q_T MotorNonlinearDynamic6;// '<S294>/MotorNonlinearDynamic1' 
  XDot_MotorNonlinearDynamic1_Q_T MotorNonlinearDynamic5;// '<S294>/MotorNonlinearDynamic1' 
  XDot_MotorNonlinearDynamic1_Q_T MotorNonlinearDynamic4;// '<S294>/MotorNonlinearDynamic1' 
  XDot_MotorNonlinearDynamic1_Q_T MotorNonlinearDynamic3;// '<S294>/MotorNonlinearDynamic1' 
  XDot_MotorNonlinearDynamic1_Q_T MotorNonlinearDynamic2;// '<S294>/MotorNonlinearDynamic1' 
  XDot_MotorNonlinearDynamic1_Q_T MotorNonlinearDynamic1;// '<S294>/MotorNonlinearDynamic1' 
};

// State disabled
struct XDis_QuadModelv_T {
  boolean_T IntegratorSecondOrderLimited_CS[6];
                                   // '<S421>/Integrator, Second-Order Limited'
  boolean_T q0q1q2q3_CSTATE[4];        // '<S15>/q0 q1 q2 q3'
  boolean_T xeyeze_CSTATE[3];          // '<S14>/xe,ye,ze'
  boolean_T pqr_CSTATE[3];             // '<S14>/p,q,r '
  boolean_T ubvbwb_CSTATE[3];          // '<S14>/ub,vb,wb'
  boolean_T IntegratorSecondOrderLimited__i[6];
                                   // '<S432>/Integrator, Second-Order Limited'
  boolean_T TransferFcn4_CSTATE;       // '<S368>/Transfer Fcn4'
  boolean_T TransferFcn1_CSTATE;       // '<S368>/Transfer Fcn1'
  boolean_T TransferFcn2_CSTATE;       // '<S368>/Transfer Fcn2'
  XDis_Distanceintogusty_QuadMo_T Distanceintogustz;// '<S512>/Distance into gust (y)' 
  XDis_Distanceintogusty_QuadMo_T Distanceintogusty;// '<S512>/Distance into gust (y)' 
  boolean_T DistanceintoGustxLimitedtogus_e;
                  // '<S515>/Distance into Gust (x) (Limited to gust length d)'
  XDis_MotorNonlinearDynamic1_Q_T MotorNonlinearDynamic8;// '<S294>/MotorNonlinearDynamic1' 
  XDis_MotorNonlinearDynamic1_Q_T MotorNonlinearDynamic7;// '<S294>/MotorNonlinearDynamic1' 
  XDis_MotorNonlinearDynamic1_Q_T MotorNonlinearDynamic6;// '<S294>/MotorNonlinearDynamic1' 
  XDis_MotorNonlinearDynamic1_Q_T MotorNonlinearDynamic5;// '<S294>/MotorNonlinearDynamic1' 
  XDis_MotorNonlinearDynamic1_Q_T MotorNonlinearDynamic4;// '<S294>/MotorNonlinearDynamic1' 
  XDis_MotorNonlinearDynamic1_Q_T MotorNonlinearDynamic3;// '<S294>/MotorNonlinearDynamic1' 
  XDis_MotorNonlinearDynamic1_Q_T MotorNonlinearDynamic2;// '<S294>/MotorNonlinearDynamic1' 
  XDis_MotorNonlinearDynamic1_Q_T MotorNonlinearDynamic1;// '<S294>/MotorNonlinearDynamic1' 
};

#ifndef ODE4_INTG
#define ODE4_INTG

// ODE4 Integration Data
struct ODE4_IntgData {
  real_T *y;                           // output
  real_T *f[4];                        // derivatives
};

#endif

// External inputs (root inport signals with default storage)
struct ExtU_QuadModelv_T {
  real_T inPWMs[8];                    // '<Root>/inPWMs'
  real_T TerrainZ;                     // '<Root>/TerrainZ'
  real32_T inFloatsCollision[20];      // '<Root>/inFloatsCollision'
  int32_T inSILInts[8];                // '<Root>/inSILInts'
  real32_T inSILFloats[20];            // '<Root>/inSILFloats'
};

// External outputs (root outports fed by signals with default storage)
struct ExtY_QuadModelv_T {
  MavLinkSensor MavHILSensor;          // '<Root>/MavHILSensor'
  MavLinkStateQuat MavHILStateQuat;    // '<Root>/MavHILStateQuat'
  MavLinkGPS MavHILGPS;                // '<Root>/MavHILGPS'
  HILGPS PXIHILGPSData;                // '<Root>/PXIHILGPSData'
  int16_T magDec;                      // '<Root>/magDec'
  uint32_T SysTime;                    // '<Root>/SysTime'
  real_T VelE[3];                      // '<Root>/VelE'
  real_T PosE[3];                      // '<Root>/PosE'
  real_T AngEuler[3];                  // '<Root>/AngEuler'
  real_T AngQuatern[4];                // '<Root>/AngQuatern'
  real_T PosGPS[3];                    // '<Root>/PosGPS'
  real_T RateB[3];                     // '<Root>/RateB'
  real_T AccB[3];                      // '<Root>/AccB'
  real_T FailInfo[8];                  // '<Root>/FailInfo'
  int8_T landed;                       // '<Root>/landed'
  real_T BattCellVoltage;              // '<Root>/BattCellVoltage'
  real_T BattRemCapacity;              // '<Root>/BattRemCapacity'
  real_T MotorRPMS[8];                 // '<Root>/MotorRPMS'
  real_T wind[3];                      // '<Root>/wind'
  MavVehileInfo MavVehile3DInfo;       // '<Root>/MavVehile3DInfo'
  real_T outCopterData[32];            // '<Root>/outCopterData'
};

// Parameters for system: '<S108>/Hpgw'
struct P_Hpgw_QuadModelv_T_ {
  real_T pgw_Y0;                       // Expression: 0
                                          //  Referenced by: '<S119>/pgw'

  real_T Constant2_Value;              // Expression: 2.6
                                          //  Referenced by: '<S119>/Constant2'

  real_T u_Gain;                       // Expression: 2*dt
                                          //  Referenced by: '<S119>/2'

  real_T Constant_Value;               // Expression: 1
                                          //  Referenced by: '<S119>/Constant'

  real_T Constant1_Value;              // Expression: 0.95
                                          //  Referenced by: '<S119>/Constant1'

  real_T Constant3_Value;              // Expression: 1/3
                                          //  Referenced by: '<S119>/Constant3'

  real_T dt_Gain;                      // Expression: dt
                                          //  Referenced by: '<S119>/dt'

  real_T UnitDelay_InitialCondition;   // Expression: 0
                                          //  Referenced by: '<S119>/Unit Delay'

};

// Parameters for system: '<S108>/Hpgw'
typedef struct P_Hpgw_QuadModelv_T_ P_Hpgw_QuadModelv_T;

// Parameters for system: '<S108>/Hqgw'
struct P_Hqgw_QuadModelv_T_ {
  real_T qgw_Y0;                       // Expression: 0
                                          //  Referenced by: '<S120>/qgw'

  real_T Constant_Value;               // Expression: 1
                                          //  Referenced by: '<S120>/Constant'

  real_T dt1_Gain;                     // Expression: 4/pi
                                          //  Referenced by: '<S120>/dt1'

  real_T dt_Gain;                      // Expression: dt
                                          //  Referenced by: '<S120>/dt'

  real_T UnitDelay_InitialCondition;   // Expression: 0
                                          //  Referenced by: '<S120>/Unit Delay'

  real_T UnitDelay1_InitialCondition;  // Expression: 0
                                          //  Referenced by: '<S120>/Unit Delay1'

};

// Parameters for system: '<S108>/Hqgw'
typedef struct P_Hqgw_QuadModelv_T_ P_Hqgw_QuadModelv_T;

// Parameters for system: '<S108>/Hrgw'
struct P_Hrgw_QuadModelv_T_ {
  real_T rgw_Y0;                       // Expression: 0
                                          //  Referenced by: '<S121>/rgw'

  real_T Constant_Value;               // Expression: 1
                                          //  Referenced by: '<S121>/Constant'

  real_T dt1_Gain;                     // Expression: 3/pi
                                          //  Referenced by: '<S121>/dt1'

  real_T dt_Gain;                      // Expression: dt
                                          //  Referenced by: '<S121>/dt'

  real_T UnitDelay_InitialCondition;   // Expression: 0
                                          //  Referenced by: '<S121>/Unit Delay'

  real_T UnitDelay1_InitialCondition;  // Expression: 0
                                          //  Referenced by: '<S121>/Unit Delay1'

};

// Parameters for system: '<S108>/Hrgw'
typedef struct P_Hrgw_QuadModelv_T_ P_Hrgw_QuadModelv_T;

// Parameters for system: '<S109>/Hugw(z)'
struct P_Hugwz_QuadModelv_T_ {
  real_T ugw_Y0;                       // Expression: 0
                                          //  Referenced by: '<S122>/ugw'

  real_T u_Gain;                       // Expression: 2*dt
                                          //  Referenced by: '<S122>/2'

  real_T Constant_Value;               // Expression: 1
                                          //  Referenced by: '<S122>/Constant'

  real_T dt_Gain;                      // Expression: dt
                                          //  Referenced by: '<S122>/dt'

  real_T UnitDelay_InitialCondition;   // Expression: 0
                                          //  Referenced by: '<S122>/Unit Delay'

};

// Parameters for system: '<S109>/Hugw(z)'
typedef struct P_Hugwz_QuadModelv_T_ P_Hugwz_QuadModelv_T;

// Parameters for system: '<S109>/Hvgw(z)'
struct P_Hvgwz_QuadModelv_T_ {
  real_T vgw_Y0;                       // Expression: 0
                                          //  Referenced by: '<S123>/vgw'

  real_T u_Gain;                       // Expression: 2*dt
                                          //  Referenced by: '<S123>/2'

  real_T Constant_Value;               // Expression: 1
                                          //  Referenced by: '<S123>/Constant'

  real_T dt_Gain;                      // Expression: dt
                                          //  Referenced by: '<S123>/dt'

  real_T UnitDelay_InitialCondition;   // Expression: 0
                                          //  Referenced by: '<S123>/Unit Delay'

};

// Parameters for system: '<S109>/Hvgw(z)'
typedef struct P_Hvgwz_QuadModelv_T_ P_Hvgwz_QuadModelv_T;

// Parameters for system: '<S109>/Hwgw(z)'
struct P_Hwgwz_QuadModelv_T_ {
  real_T wgw_Y0;                       // Expression: 0
                                          //  Referenced by: '<S124>/wgw'

  real_T u_Gain;                       // Expression: 2*dt
                                          //  Referenced by: '<S124>/2'

  real_T Constant_Value;               // Expression: 1
                                          //  Referenced by: '<S124>/Constant'

  real_T dt_Gain;                      // Expression: dt
                                          //  Referenced by: '<S124>/dt'

  real_T UnitDelay_InitialCondition;   // Expression: 0
                                          //  Referenced by: '<S124>/Unit Delay'

};

// Parameters for system: '<S109>/Hwgw(z)'
typedef struct P_Hwgwz_QuadModelv_T_ P_Hwgwz_QuadModelv_T;

// Parameters for system: '<S113>/Interpolate  rates'
struct P_Interpolaterates_QuadModelv_T_ {
  real_T max_height_low_Value;         // Expression: max_height_low
                                          //  Referenced by: '<S127>/max_height_low'

  real_T min_height_high_Value;        // Expression: min_height_high
                                          //  Referenced by: '<S127>/min_height_high'

};

// Parameters for system: '<S113>/Interpolate  rates'
typedef struct P_Interpolaterates_QuadModelv_T_ P_Interpolaterates_QuadModelv_T;

// Parameters for system: '<S114>/Interpolate  velocities'
struct P_Interpolatevelocities_QuadM_T_ {
  real_T max_height_low_Value;         // Expression: max_height_low
                                          //  Referenced by: '<S135>/max_height_low'

  real_T min_height_high_Value;        // Expression: min_height_high
                                          //  Referenced by: '<S135>/min_height_high'

};

// Parameters for system: '<S114>/Interpolate  velocities'
typedef struct P_Interpolatevelocities_QuadM_T_ P_Interpolatevelocities_QuadM_T;

// Parameters for system: '<S193>/Positive Trace'
struct P_PositiveTrace_QuadModelv_T_ {
  real_T Constant_Value;               // Expression: 1
                                          //  Referenced by: '<S202>/Constant'

  real_T Gain_Gain;                    // Expression: 0.5
                                          //  Referenced by: '<S202>/Gain'

  real_T Gain1_Gain;                   // Expression: 2
                                          //  Referenced by: '<S202>/Gain1'

};

// Parameters for system: '<S193>/Positive Trace'
typedef struct P_PositiveTrace_QuadModelv_T_ P_PositiveTrace_QuadModelv_T;

// Parameters for system: '<S193>/Negative Trace'
struct P_NegativeTrace_QuadModelv_T_ {
  real_T Constant_Value;               // Expression: 1
                                          //  Referenced by: '<S218>/Constant'

  real_T Gain_Gain;                    // Expression: 0.5
                                          //  Referenced by: '<S206>/Gain'

  real_T Constant1_Value;              // Expression: 0.5
                                          //  Referenced by: '<S217>/Constant1'

  real_T Constant2_Value[2];           // Expression: [0 1]
                                          //  Referenced by: '<S217>/Constant2'

  real_T Gain1_Gain;                   // Expression: 1
                                          //  Referenced by: '<S206>/Gain1'

  real_T Gain3_Gain;                   // Expression: 1
                                          //  Referenced by: '<S206>/Gain3'

  real_T Gain4_Gain;                   // Expression: 1
                                          //  Referenced by: '<S206>/Gain4'

  real_T Constant_Value_n;             // Expression: 1
                                          //  Referenced by: '<S223>/Constant'

  real_T Gain_Gain_c;                  // Expression: 0.5
                                          //  Referenced by: '<S207>/Gain'

  real_T Constant1_Value_a;            // Expression: 0.5
                                          //  Referenced by: '<S222>/Constant1'

  real_T Constant2_Value_c[2];         // Expression: [0 1]
                                          //  Referenced by: '<S222>/Constant2'

  real_T Gain1_Gain_n;                 // Expression: 1
                                          //  Referenced by: '<S207>/Gain1'

  real_T Gain2_Gain;                   // Expression: 1
                                          //  Referenced by: '<S207>/Gain2'

  real_T Gain3_Gain_m;                 // Expression: 1
                                          //  Referenced by: '<S207>/Gain3'

  real_T Constant_Value_m;             // Expression: 1
                                          //  Referenced by: '<S213>/Constant'

  real_T Gain_Gain_l;                  // Expression: 0.5
                                          //  Referenced by: '<S205>/Gain'

  real_T Constant1_Value_i;            // Expression: 0.5
                                          //  Referenced by: '<S212>/Constant1'

  real_T Constant2_Value_b[2];         // Expression: [0 1]
                                          //  Referenced by: '<S212>/Constant2'

  real_T Gain1_Gain_b;                 // Expression: 1
                                          //  Referenced by: '<S205>/Gain1'

  real_T Gain2_Gain_d;                 // Expression: 1
                                          //  Referenced by: '<S205>/Gain2'

  real_T Gain3_Gain_n;                 // Expression: 1
                                          //  Referenced by: '<S205>/Gain3'

};

// Parameters for system: '<S193>/Negative Trace'
typedef struct P_NegativeTrace_QuadModelv_T_ P_NegativeTrace_QuadModelv_T;

// Parameters for system: '<S203>/If Warning//Error'
struct P_IfWarningError_QuadModelv_T_ {
  real_T Constant1_Value;              // Expression: 0
                                          //  Referenced by: '<S229>/Constant1'

  real_T Constant1_Value_k;            // Expression: 0
                                          //  Referenced by: '<S228>/Constant1'

  real_T Bias_Bias;                    // Expression: -1
                                          //  Referenced by: '<S231>/Bias'

  real_T Bias1_Bias[9];                // Expression: -eye(3)
                                          //  Referenced by: '<S230>/Bias1'

};

// Parameters for system: '<S203>/If Warning//Error'
typedef struct P_IfWarningError_QuadModelv_T_ P_IfWarningError_QuadModelv_T;

// Parameters for system: '<S294>/MotorNonlinearDynamic1'
struct P_MotorNonlinearDynamic1_Quad_T_ {
  real_T Signal_Saturation_UpperSat;   // Expression: 0.95
                                          //  Referenced by: '<S297>/Signal_Saturation'

  real_T Signal_Saturation_LowerSat;   // Expression: 0
                                          //  Referenced by: '<S297>/Signal_Saturation'

};

// Parameters for system: '<S294>/MotorNonlinearDynamic1'
typedef struct P_MotorNonlinearDynamic1_Quad_T_ P_MotorNonlinearDynamic1_Quad_T;

// Parameters for system: '<S512>/Distance into gust (y)'
struct P_Distanceintogusty_QuadModel_T_ {
  real_T x_Y0;                         // Expression: [0]
                                          //  Referenced by: '<S516>/x'

  real_T DistanceintoGustxLimitedtogustl;// Expression: 0
                                            //  Referenced by: '<S516>/Distance into Gust (x) (Limited to gust length d) '

  real_T DistanceintoGustxLimitedtogus_e;// Expression: 0
                                            //  Referenced by: '<S516>/Distance into Gust (x) (Limited to gust length d) '

};

// Parameters for system: '<S512>/Distance into gust (y)'
typedef struct P_Distanceintogusty_QuadModel_T_ P_Distanceintogusty_QuadModel_T;

// Parameters (default storage)
struct P_QuadModelv_T_ {
  struct_cC8C5MZV75wwVdzQ00rR7E FaultParamAPI;// Variable: FaultParamAPI
                                                 //  Referenced by:
                                                 //    '<S510>/Constant'
                                                 //    '<S510>/Gain'
                                                 //    '<S94>/Wind direction'
                                                 //    '<S94>/Windspeed at 20ft (6m)'
                                                 //    '<S95>/Wind direction'
                                                 //    '<S95>/Windspeed at 20ft (6m)'
                                                 //    '<S105>/Wind Direction'
                                                 //    '<S105>/Wind speed at reference height'
                                                 //    '<S512>/Gust start time'
                                                 //    '<S421>/2*zeta * wn'
                                                 //    '<S432>/2*zeta * wn'

  struct_IIgj0sxnnecCOL2Kz6CFkH PropFault;// Variable: PropFault
                                             //  Referenced by: '<S510>/FaultID1'

  struct_O6zquuSaY7mWQl2ZAiqFYB MotorFault1;// Variable: MotorFault1
                                               //  Referenced by: '<S293>/MotorNum'

  real_T ModelFailEnable;              // Variable: ModelFailEnable
                                          //  Referenced by: '<S12>/ModelFailEnable'

  real_T ModelInit_AngEuler[3];        // Variable: ModelInit_AngEuler
                                          //  Referenced by: '<S15>/Initial Euler Angles'

  real_T ModelInit_PosE[3];            // Variable: ModelInit_PosE
                                          //  Referenced by: '<S14>/xe,ye,ze'

  real_T ModelInit_RPM;                // Variable: ModelInit_RPM
                                          //  Referenced by: '<S294>/ModelInit_RPM'

  real_T ModelInit_RateB[3];           // Variable: ModelInit_RateB
                                          //  Referenced by: '<S14>/p,q,r '

  real_T ModelInit_VelB[3];            // Variable: ModelInit_VelB
                                          //  Referenced by: '<S14>/ub,vb,wb'

  real_T ModelParam_BattHoverMinutes;  // Variable: ModelParam_BattHoverMinutes
                                          //  Referenced by: '<S2>/ModelParam.BattHoverMinutes'

  real_T ModelParam_BattHoverThr;      // Variable: ModelParam_BattHoverThr
                                          //  Referenced by: '<S2>/ModelParam.BattHoverThr'

  real_T ModelParam_GPSEphFinal;       // Variable: ModelParam_GPSEphFinal
                                          //  Referenced by: '<S322>/ModelParam.GPSEphFinal'

  real_T ModelParam_GPSEpvFinal;       // Variable: ModelParam_GPSEpvFinal
                                          //  Referenced by: '<S322>/ModelParam.GPSEpvFinal'

  real_T ModelParam_GPSLatLong[2];     // Variable: ModelParam_GPSLatLong
                                          //  Referenced by:
                                          //    '<S63>/ref_position'
                                          //    '<S371>/ref_position'

  real_T ModelParam_envAltitude;       // Variable: ModelParam_envAltitude
                                          //  Referenced by:
                                          //    '<S5>/ModelParam.envAltitude'
                                          //    '<S366>/ModelParam.envAltitude'

  real_T ModelParam_motorCr;           // Variable: ModelParam_motorCr
                                          //  Referenced by: '<S294>/ModelParam.motorCr'

  real_T ModelParam_motorJm;           // Variable: ModelParam_motorJm
                                          //  Referenced by: '<S7>/ModelParam.motorJm'

  real_T ModelParam_motorMinThr;       // Variable: ModelParam_motorMinThr
                                          //  Referenced by:
                                          //    '<S305>/Constant'
                                          //    '<S307>/Constant'
                                          //    '<S309>/Constant'
                                          //    '<S311>/Constant'
                                          //    '<S313>/Constant'
                                          //    '<S315>/Constant'
                                          //    '<S317>/Constant'
                                          //    '<S319>/Constant'

  real_T ModelParam_motorT;            // Variable: ModelParam_motorT
                                          //  Referenced by: '<S294>/ModelParam.motorT'

  real_T ModelParam_motorWb;           // Variable: ModelParam_motorWb
                                          //  Referenced by: '<S294>/ModelParam.motorWb'

  real_T ModelParam_noisePowerOffGainAccel;
                                  // Variable: ModelParam_noisePowerOffGainAccel
                                     //  Referenced by: '<S394>/AccelNoiseGainFunction'

  real_T ModelParam_noisePowerOffGainAccelZ;
                                 // Variable: ModelParam_noisePowerOffGainAccelZ
                                    //  Referenced by: '<S394>/AccelNoiseGainFunction'

  real_T ModelParam_noisePowerOffGainGyro;
                                   // Variable: ModelParam_noisePowerOffGainGyro
                                      //  Referenced by: '<S394>/GyroNoiseGainFunction'

  real_T ModelParam_noisePowerOffGainGyroZ;
                                  // Variable: ModelParam_noisePowerOffGainGyroZ
                                     //  Referenced by: '<S394>/GyroNoiseGainFunction'

  real_T ModelParam_noisePowerOffGainMag;
                                    // Variable: ModelParam_noisePowerOffGainMag
                                       //  Referenced by: '<S394>/MagNoiseGainFunction'

  real_T ModelParam_noisePowerOffGainMagZ;
                                   // Variable: ModelParam_noisePowerOffGainMagZ
                                      //  Referenced by: '<S394>/MagNoiseGainFunction'

  real_T ModelParam_noisePowerOnGainAccel;
                                   // Variable: ModelParam_noisePowerOnGainAccel
                                      //  Referenced by: '<S394>/AccelNoiseGainFunction'

  real_T ModelParam_noisePowerOnGainAccelZ;
                                  // Variable: ModelParam_noisePowerOnGainAccelZ
                                     //  Referenced by: '<S394>/AccelNoiseGainFunction'

  real_T ModelParam_noisePowerOnGainGyro;
                                    // Variable: ModelParam_noisePowerOnGainGyro
                                       //  Referenced by: '<S394>/GyroNoiseGainFunction'

  real_T ModelParam_noisePowerOnGainGyroZ;
                                   // Variable: ModelParam_noisePowerOnGainGyroZ
                                      //  Referenced by: '<S394>/GyroNoiseGainFunction'

  real_T ModelParam_noisePowerOnGainMag;
                                     // Variable: ModelParam_noisePowerOnGainMag
                                        //  Referenced by: '<S394>/MagNoiseGainFunction'

  real_T ModelParam_noisePowerOnGainMagZ;
                                    // Variable: ModelParam_noisePowerOnGainMagZ
                                       //  Referenced by: '<S394>/MagNoiseGainFunction'

  real_T ModelParam_rotorCm;           // Variable: ModelParam_rotorCm
                                          //  Referenced by: '<S7>/ModelParam.rotorCm'

  real_T ModelParam_rotorCt;           // Variable: ModelParam_rotorCt
                                          //  Referenced by: '<S7>/ModelParam.rotorCt'

  real_T ModelParam_uavCCm[3];         // Variable: ModelParam_uavCCm
                                          //  Referenced by: '<S7>/ModelParam.uavCCm'

  real_T ModelParam_uavCd;             // Variable: ModelParam_uavCd
                                          //  Referenced by: '<S7>/ModelParam.uavCd'

  real_T ModelParam_uavDearo;          // Variable: ModelParam_uavDearo
                                          //  Referenced by: '<S7>/ModelParam.uavDearo'

  real_T ModelParam_uavJ[9];           // Variable: ModelParam_uavJ
                                          //  Referenced by: '<S6>/uavJ'

  real_T ModelParam_uavMass;           // Variable: ModelParam_uavMass
                                          //  Referenced by: '<S6>/ModelParam.uavMass'

  real_T ModelParam_uavR;              // Variable: ModelParam_uavR
                                          //  Referenced by: '<S6>/ModelParam.uavR'

  int8_T ModelParam_uavMotNumbs;       // Variable: ModelParam_uavMotNumbs
                                          //  Referenced by: '<S2>/ModelParam.uavMotNumbs'

  int8_T ModelParam_uavType;           // Variable: ModelParam_uavType
                                          //  Referenced by:
                                          //    '<S7>/ModelParam.uavType'
                                          //    '<S12>/UAVType'

  real_T BandLimitedWhiteNoise_Cov; // Mask Parameter: BandLimitedWhiteNoise_Cov
                                       //  Referenced by: '<S511>/Output'

  real_T DrydenWindTurbulenceModelDiscre;
                              // Mask Parameter: DrydenWindTurbulenceModelDiscre
                                 //  Referenced by: '<S183>/Medium//High Altitude'

  real_T DrydenWindTurbulenceModelDisc_m;
                              // Mask Parameter: DrydenWindTurbulenceModelDisc_m
                                 //  Referenced by: '<S144>/Medium//High Altitude'

  real_T DrydenWindTurbulenceModelDisc_n[4];
                              // Mask Parameter: DrydenWindTurbulenceModelDisc_n
                                 //  Referenced by: '<S157>/White Noise'

  real_T DrydenWindTurbulenceModelDisc_i[4];
                              // Mask Parameter: DrydenWindTurbulenceModelDisc_i
                                 //  Referenced by: '<S118>/White Noise'

  real_T DrydenWindTurbulenceModelDisc_g;
                              // Mask Parameter: DrydenWindTurbulenceModelDisc_g
                                 //  Referenced by:
                                 //    '<S147>/Constant1'
                                 //    '<S147>/Constant2'
                                 //    '<S147>/Constant3'
                                 //    '<S148>/Constant3'

  real_T DrydenWindTurbulenceModelDisc_k;
                              // Mask Parameter: DrydenWindTurbulenceModelDisc_k
                                 //  Referenced by:
                                 //    '<S108>/Constant1'
                                 //    '<S108>/Constant2'
                                 //    '<S108>/Constant3'
                                 //    '<S109>/Constant3'

  real_T WhiteNoise_Ts;                // Mask Parameter: WhiteNoise_Ts
                                          //  Referenced by: '<S157>/Constant1'

  real_T WhiteNoise_Ts_a;              // Mask Parameter: WhiteNoise_Ts_a
                                          //  Referenced by: '<S118>/Constant1'

  real_T DrydenWindTurbulenceModelDis_n3;
                              // Mask Parameter: DrydenWindTurbulenceModelDis_n3
                                 //  Referenced by: '<S164>/Probability of  Exceedance'

  real_T DrydenWindTurbulenceModelDis_gr;
                              // Mask Parameter: DrydenWindTurbulenceModelDis_gr
                                 //  Referenced by: '<S125>/Probability of  Exceedance'

  real_T DrydenWindTurbulenceModelDis_np;
                              // Mask Parameter: DrydenWindTurbulenceModelDis_np
                                 //  Referenced by: '<S94>/Wingspan'

  real_T DrydenWindTurbulenceModelDisc_d;
                              // Mask Parameter: DrydenWindTurbulenceModelDisc_d
                                 //  Referenced by: '<S95>/Wingspan'

  real_T ThreeaxisInertialMeasurementUni[3];
                              // Mask Parameter: ThreeaxisInertialMeasurementUni
                                 //  Referenced by: '<S415>/Measurement bias'

  real_T ThreeaxisInertialMeasurementU_h[9];
                              // Mask Parameter: ThreeaxisInertialMeasurementU_h
                                 //  Referenced by: '<S415>/Scale factors & Cross-coupling  errors'

  real_T DirectionCosineMatrixtoQuaterni;
                              // Mask Parameter: DirectionCosineMatrixtoQuaterni
                                 //  Referenced by:
                                 //    '<S203>/If Warning//Error'
                                 //    '<S203>/Constant'

  real_T DirectionCosineMatrixtoQuater_p;
                              // Mask Parameter: DirectionCosineMatrixtoQuater_p
                                 //  Referenced by:
                                 //    '<S241>/If Warning//Error'
                                 //    '<S241>/Constant'

  real_T DirectionCosineMatrixtoQuater_c;
                              // Mask Parameter: DirectionCosineMatrixtoQuater_c
                                 //  Referenced by:
                                 //    '<S329>/If Warning//Error'
                                 //    '<S329>/Constant'

  real_T CompareToConstant_const;     // Mask Parameter: CompareToConstant_const
                                         //  Referenced by: '<S82>/Constant'

  real_T CompareToConstant_const_i; // Mask Parameter: CompareToConstant_const_i
                                       //  Referenced by: '<S80>/Constant'

  real_T CompareToConstant_const_j; // Mask Parameter: CompareToConstant_const_j
                                       //  Referenced by: '<S83>/Constant'

  real_T CompareToConstant_const_ji;
                                   // Mask Parameter: CompareToConstant_const_ji
                                      //  Referenced by: '<S76>/Constant'

  real_T CompareToConstant_const_k; // Mask Parameter: CompareToConstant_const_k
                                       //  Referenced by: '<S74>/Constant'

  real_T CompareToConstant_const_f; // Mask Parameter: CompareToConstant_const_f
                                       //  Referenced by: '<S77>/Constant'

  real_T CompareToConstant_const_n; // Mask Parameter: CompareToConstant_const_n
                                       //  Referenced by: '<S448>/Constant'

  real_T CompareToConstant_const_a; // Mask Parameter: CompareToConstant_const_a
                                       //  Referenced by: '<S446>/Constant'

  real_T CompareToConstant_const_e; // Mask Parameter: CompareToConstant_const_e
                                       //  Referenced by: '<S449>/Constant'

  real_T CompareToConstant_const_b; // Mask Parameter: CompareToConstant_const_b
                                       //  Referenced by: '<S386>/Constant'

  real_T CompareToConstant_const_g; // Mask Parameter: CompareToConstant_const_g
                                       //  Referenced by: '<S384>/Constant'

  real_T CompareToConstant_const_iz;
                                   // Mask Parameter: CompareToConstant_const_iz
                                      //  Referenced by: '<S387>/Constant'

  real_T CompareToConstant_const_bz;
                                   // Mask Parameter: CompareToConstant_const_bz
                                      //  Referenced by: '<S380>/Constant'

  real_T CompareToConstant_const_d; // Mask Parameter: CompareToConstant_const_d
                                       //  Referenced by: '<S378>/Constant'

  real_T CompareToConstant_const_ex;
                                   // Mask Parameter: CompareToConstant_const_ex
                                      //  Referenced by: '<S381>/Constant'

  real_T Distanceintogustx_d_m;        // Mask Parameter: Distanceintogustx_d_m
                                          //  Referenced by: '<S515>/Distance into Gust (x) (Limited to gust length d)'

  real_T Distanceintogusty_d_m;        // Mask Parameter: Distanceintogusty_d_m
                                          //  Referenced by: '<S512>/Distance into gust (y)'

  real_T Distanceintogustz_d_m;        // Mask Parameter: Distanceintogustz_d_m
                                          //  Referenced by: '<S512>/Distance into gust (z)'

  real_T DiscreteWindGustModel_d_m[3];
                                    // Mask Parameter: DiscreteWindGustModel_d_m
                                       //  Referenced by: '<S512>/pi//Gust length'

  real_T ThreeaxisInertialMeasurementU_b[3];
                              // Mask Parameter: ThreeaxisInertialMeasurementU_b
                                 //  Referenced by: '<S416>/Measurement bias'

  real_T ThreeaxisInertialMeasurementU_c[3];
                              // Mask Parameter: ThreeaxisInertialMeasurementU_c
                                 //  Referenced by: '<S416>/g-sensitive bias'

  real_T ThreeaxisInertialMeasurementU_m[9];
                              // Mask Parameter: ThreeaxisInertialMeasurementU_m
                                 //  Referenced by: '<S416>/Scale factors & Cross-coupling  errors '

  real_T ThreeaxisInertialMeasurementU_e[3];
                              // Mask Parameter: ThreeaxisInertialMeasurementU_e
                                 //  Referenced by: '<S415>/wl_ins'

  real_T CustomVariableMass6DOFQuaternio;
                              // Mask Parameter: CustomVariableMass6DOFQuaternio
                                 //  Referenced by: '<S26>/High Gain Quaternion Normalization'

  real_T CheckAltitude_max;            // Mask Parameter: CheckAltitude_max
                                          //  Referenced by: '<S433>/max_val'

  real_T CheckLatitude_max;            // Mask Parameter: CheckLatitude_max
                                          //  Referenced by: '<S434>/max_val'

  real_T CheckLongitude_max;           // Mask Parameter: CheckLongitude_max
                                          //  Referenced by: '<S435>/max_val'

  real_T Istimewithinmodellimits_max;
                                  // Mask Parameter: Istimewithinmodellimits_max
                                     //  Referenced by: '<S437>/max_val'

  real_T CheckAltitude_min;            // Mask Parameter: CheckAltitude_min
                                          //  Referenced by: '<S433>/min_val'

  real_T CheckLatitude_min;            // Mask Parameter: CheckLatitude_min
                                          //  Referenced by: '<S434>/min_val'

  real_T CheckLongitude_min;           // Mask Parameter: CheckLongitude_min
                                          //  Referenced by: '<S435>/min_val'

  real_T Istimewithinmodellimits_min;
                                  // Mask Parameter: Istimewithinmodellimits_min
                                     //  Referenced by: '<S437>/min_val'

  real_T FlatEarthtoLLA_psi;           // Mask Parameter: FlatEarthtoLLA_psi
                                          //  Referenced by: '<S63>/ref_rotation'

  real_T FlatEarthtoLLA_psi_e;         // Mask Parameter: FlatEarthtoLLA_psi_e
                                          //  Referenced by: '<S371>/ref_rotation'

  real_T WhiteNoise_pwr[4];            // Mask Parameter: WhiteNoise_pwr
                                          //  Referenced by: '<S157>/Constant'

  real_T WhiteNoise_pwr_d[4];          // Mask Parameter: WhiteNoise_pwr_d
                                          //  Referenced by: '<S118>/Constant'

  real_T BandLimitedWhiteNoise_seed;
                                   // Mask Parameter: BandLimitedWhiteNoise_seed
                                      //  Referenced by: '<S511>/White Noise'

  real_T DirectionCosineMatrixtoQuater_i;
                              // Mask Parameter: DirectionCosineMatrixtoQuater_i
                                 //  Referenced by: '<S203>/If Warning//Error'

  real_T DirectionCosineMatrixtoQuate_i5;
                              // Mask Parameter: DirectionCosineMatrixtoQuate_i5
                                 //  Referenced by: '<S241>/If Warning//Error'

  real_T DirectionCosineMatrixtoQuater_b;
                              // Mask Parameter: DirectionCosineMatrixtoQuater_b
                                 //  Referenced by: '<S329>/If Warning//Error'

  real_T DiscreteWindGustModel_v_m[3];
                                    // Mask Parameter: DiscreteWindGustModel_v_m
                                       //  Referenced by: '<S512>/Gust magnitude//2.0'

  real_T ThreeaxisInertialMeasurement_c1;
                              // Mask Parameter: ThreeaxisInertialMeasurement_c1
                                 //  Referenced by:
                                 //    '<S421>/2*zeta * wn'
                                 //    '<S421>/wn^2'

  real_T ThreeaxisInertialMeasurementU_i;
                              // Mask Parameter: ThreeaxisInertialMeasurementU_i
                                 //  Referenced by:
                                 //    '<S432>/2*zeta * wn'
                                 //    '<S432>/wn^2'

  boolean_T DiscreteWindGustModel_Gx;// Mask Parameter: DiscreteWindGustModel_Gx
                                        //  Referenced by: '<S512>/Constant'

  boolean_T DiscreteWindGustModel_Gy;// Mask Parameter: DiscreteWindGustModel_Gy
                                        //  Referenced by: '<S512>/Constant1'

  boolean_T DiscreteWindGustModel_Gz;// Mask Parameter: DiscreteWindGustModel_Gz
                                        //  Referenced by: '<S512>/Constant2'

  real_T Constant_Value;               // Expression: 1
                                          //  Referenced by: '<S43>/Constant'

  real_T Constant_Value_l;             // Expression: 1
                                          //  Referenced by: '<S44>/Constant'

  real_T Bias_Bias;                    // Expression: -90
                                          //  Referenced by: '<S72>/Bias'

  real_T Gain_Gain;                    // Expression: -1
                                          //  Referenced by: '<S72>/Gain'

  real_T Bias1_Bias;                   // Expression: +90
                                          //  Referenced by: '<S72>/Bias1'

  real_T Bias_Bias_i;                  // Expression: 180
                                          //  Referenced by: '<S75>/Bias'

  real_T Bias1_Bias_o;                 // Expression: -180
                                          //  Referenced by: '<S75>/Bias1'

  real_T Bias_Bias_l;                  // Expression: 180
                                          //  Referenced by: '<S73>/Bias'

  real_T Bias1_Bias_c;                 // Expression: -180
                                          //  Referenced by: '<S73>/Bias1'

  real_T Bias_Bias_g;                  // Expression: -90
                                          //  Referenced by: '<S78>/Bias'

  real_T Gain_Gain_m;                  // Expression: -1
                                          //  Referenced by: '<S78>/Gain'

  real_T Bias1_Bias_g;                 // Expression: +90
                                          //  Referenced by: '<S78>/Bias1'

  real_T Constant2_Value;              // Expression: 360
                                          //  Referenced by: '<S81>/Constant2'

  real_T Bias_Bias_m;                  // Expression: 180
                                          //  Referenced by: '<S81>/Bias'

  real_T Bias1_Bias_h;                 // Expression: -180
                                          //  Referenced by: '<S81>/Bias1'

  real_T Constant_Value_n;             // Expression: 180
                                          //  Referenced by: '<S69>/Constant'

  real_T Constant1_Value;              // Expression: 0
                                          //  Referenced by: '<S69>/Constant1'

  real_T Constant2_Value_b;            // Expression: 360
                                          //  Referenced by: '<S79>/Constant2'

  real_T Bias_Bias_c;                  // Expression: 180
                                          //  Referenced by: '<S79>/Bias'

  real_T Bias1_Bias_n;                 // Expression: -180
                                          //  Referenced by: '<S79>/Bias1'

  real_T Gain_Gain_p;                  // Expression: 1
                                          //  Referenced by: '<S129>/Gain'

  real_T Gain_Gain_pq;                 // Expression: 1
                                          //  Referenced by: '<S137>/Gain'

  real_T Gain_Gain_k;                  // Expression: 1
                                          //  Referenced by: '<S168>/Gain'

  real_T Gain_Gain_h;                  // Expression: 1
                                          //  Referenced by: '<S176>/Gain'

  real_T Bias_Bias_e;                  // Expression: -90
                                          //  Referenced by: '<S376>/Bias'

  real_T Gain_Gain_d;                  // Expression: -1
                                          //  Referenced by: '<S376>/Gain'

  real_T Bias1_Bias_g2;                // Expression: +90
                                          //  Referenced by: '<S376>/Bias1'

  real_T Bias_Bias_ee;                 // Expression: 180
                                          //  Referenced by: '<S379>/Bias'

  real_T Bias1_Bias_k;                 // Expression: -180
                                          //  Referenced by: '<S379>/Bias1'

  real_T Bias_Bias_gf;                 // Expression: 180
                                          //  Referenced by: '<S377>/Bias'

  real_T Bias1_Bias_a;                 // Expression: -180
                                          //  Referenced by: '<S377>/Bias1'

  real_T Bias_Bias_d;                  // Expression: -90
                                          //  Referenced by: '<S382>/Bias'

  real_T Gain_Gain_i;                  // Expression: -1
                                          //  Referenced by: '<S382>/Gain'

  real_T Bias1_Bias_at;                // Expression: +90
                                          //  Referenced by: '<S382>/Bias1'

  real_T Constant2_Value_m;            // Expression: 360
                                          //  Referenced by: '<S385>/Constant2'

  real_T Bias_Bias_j;                  // Expression: 180
                                          //  Referenced by: '<S385>/Bias'

  real_T Bias1_Bias_o4;                // Expression: -180
                                          //  Referenced by: '<S385>/Bias1'

  real_T Constant_Value_d;             // Expression: 180
                                          //  Referenced by: '<S373>/Constant'

  real_T Constant1_Value_b;            // Expression: 0
                                          //  Referenced by: '<S373>/Constant1'

  real_T Constant2_Value_h;            // Expression: 360
                                          //  Referenced by: '<S383>/Constant2'

  real_T Bias_Bias_f;                  // Expression: 180
                                          //  Referenced by: '<S383>/Bias'

  real_T Bias1_Bias_gl;                // Expression: -180
                                          //  Referenced by: '<S383>/Bias1'

  real_T Bias_Bias_c0;                 // Expression: -90
                                          //  Referenced by: '<S444>/Bias'

  real_T Gain_Gain_kn;                 // Expression: -1
                                          //  Referenced by: '<S444>/Gain'

  real_T Bias1_Bias_j;                 // Expression: +90
                                          //  Referenced by: '<S444>/Bias1'

  real_T Bias_Bias_el;                 // Expression: 180
                                          //  Referenced by: '<S447>/Bias'

  real_T Bias1_Bias_jm;                // Expression: -180
                                          //  Referenced by: '<S447>/Bias1'

  real_T Bias_Bias_b;                  // Expression: 180
                                          //  Referenced by: '<S445>/Bias'

  real_T Bias1_Bias_p;                 // Expression: -180
                                          //  Referenced by: '<S445>/Bias1'

  real_T pp13_Y0[13];                  // Expression: ones(1,maxdef+1)
                                          //  Referenced by: '<S466>/pp[13]'

  real_T Constant_Value_k;             // Expression: 1
                                          //  Referenced by: '<S466>/Constant'

  real_T pp13_Y0_n[13];                // Expression: ones(1,maxdef+1)
                                          //  Referenced by: '<S467>/pp[13]'

  real_T k1313_Value[169];             // Expression: k
                                          //  Referenced by: '<S467>/k[13][13]'

  real_T bpp_Y0;                       // Expression: 0
                                          //  Referenced by: '<S462>/bpp'

  real_T Constant_Value_p;             // Expression: fm(2)
                                          //  Referenced by: '<S462>/Constant'

  real_T Constant1_Value_i;            // Expression: 1
                                          //  Referenced by: '<S462>/Constant1'

  real_T UnitDelay1_InitialCondition[13];// Expression: ones(1,maxdef+1)
                                            //  Referenced by: '<S462>/Unit Delay1'

  real_T Constant_Value_g;             // Expression: 1
                                          //  Referenced by: '<S470>/Constant'

  real_T Gain1_Gain;                   // Expression: 1
                                          //  Referenced by: '<S470>/Gain1'

  real_T Gain2_Gain;                   // Expression: 1
                                          //  Referenced by: '<S470>/Gain2'

  real_T Constant_Value_i;             // Expression: 1
                                          //  Referenced by: '<S472>/Constant'

  real_T Constant_Value_k4;            // Expression: 1
                                          //  Referenced by: '<S473>/Constant'

  real_T Constant1_Value_o;            // Expression: 0
                                          //  Referenced by: '<S476>/Constant1'

  real_T Constant_Value_m;             // Expression: 0
                                          //  Referenced by: '<S476>/Constant'

  real_T k1313_Value_c[169];           // Expression: k
                                          //  Referenced by: '<S476>/k[13][13]'

  real_T dp1313_Y0[169];               // Expression: zeros(maxdef+1,maxdef+1)
                                          //  Referenced by: '<S460>/dp[13][13]'

  real_T snorm169_Y0[169];             // Expression: snorm
                                          //  Referenced by: '<S460>/snorm[169]'

  real_T UnitDelay_InitialCondition[169];// Expression: zeros(maxdef+1,maxdef+1)
                                            //  Referenced by: '<S460>/Unit Delay'

  real_T UnitDelay1_InitialCondition_j[169];// Expression: snorm
                                               //  Referenced by: '<S460>/Unit Delay1'

  real_T Merge1_InitialOutput;         // Expression: 0
                                          //  Referenced by: '<S460>/Merge1'

  real_T Merge_InitialOutput;          // Expression: 0
                                          //  Referenced by: '<S460>/Merge'

  real_T Gain_Gain_ps;                 // Expression: 1
                                          //  Referenced by: '<S489>/Gain'

  real_T zerosmaxdef1maxdef1_Value[169];// Expression: zeros(maxdef+1,maxdef+1)
                                           //  Referenced by: '<S488>/zeros(maxdef+1,maxdef+1)'

  real_T tc1313_Y0[169];               // Expression: zeros(maxdef+1,maxdef+1)
                                          //  Referenced by: '<S461>/tc[13][13]'

  real_T UnitDelay_InitialCondition_b[169];// Expression: zeros(maxdef+1,maxdef+1)
                                              //  Referenced by: '<S461>/Unit Delay'

  real_T cmaxdefmaxdef_Value[169];     // Expression: c
                                          //  Referenced by: '<S461>/c[maxdef][maxdef]'

  real_T cdmaxdefmaxdef_Value[169];    // Expression: cd
                                          //  Referenced by: '<S461>/cd[maxdef][maxdef]'

  real_T UnitDelay_InitialCondition_p[169];// Expression: zeros(maxdef+1,maxdef+1)
                                              //  Referenced by: '<S488>/Unit Delay'

  real_T bt_Y0;                        // Expression: 0
                                          //  Referenced by: '<S458>/bt'

  real_T bp_Y0;                        // Expression: 0
                                          //  Referenced by: '<S458>/bp'

  real_T br_Y0;                        // Expression: 0
                                          //  Referenced by: '<S458>/br'

  real_T bpp_Y0_k;                     // Expression: 0
                                          //  Referenced by: '<S458>/bpp'

  real_T Constant1_Value_n;            // Expression: 1
                                          //  Referenced by: '<S464>/Constant1'

  real_T Merge_InitialOutput_h;        // Expression: 0
                                          //  Referenced by: '<S464>/Merge'

  real_T fm_Value[13];                 // Expression: fm
                                          //  Referenced by: '<S459>/fm'

  real_T Merge1_InitialOutput_k;       // Expression: 0
                                          //  Referenced by: '<S464>/Merge1'

  real_T fn_Value[13];                 // Expression: fn
                                          //  Referenced by: '<S459>/fn'

  real_T Constant1_Value_h;            // Expression: 0
                                          //  Referenced by: '<S465>/Constant1'

  real_T UnitDelay1_InitialCondition_g;// Expression: 0
                                          //  Referenced by: '<S459>/Unit Delay1'

  real_T UnitDelay3_InitialCondition;  // Expression: 0
                                          //  Referenced by: '<S459>/Unit Delay3'

  real_T UnitDelay2_InitialCondition;  // Expression: 0
                                          //  Referenced by: '<S459>/Unit Delay2'

  real_T UnitDelay4_InitialCondition;  // Expression: 0
                                          //  Referenced by: '<S459>/Unit Delay4'

  real_T btbpbrbpp_Y0[4];              // Expression: [0 0 0 0]
                                          //  Referenced by: '<S450>/bt,bp,br,bpp'

  real_T UnitDelay_InitialCondition_pm;// Expression: 0
                                          //  Referenced by: '<S450>/Unit Delay'

  real_T UnitDelay2_InitialCondition_j[4];// Expression: [0 0 0 0]
                                             //  Referenced by: '<S450>/Unit Delay2'

  real_T r_Y0;                         // Expression: 6378.137
                                          //  Referenced by: '<S451>/r'

  real_T ct_Y0;                        // Expression: 1
                                          //  Referenced by: '<S451>/ct'

  real_T st_Y0;                        // Expression: 0
                                          //  Referenced by: '<S451>/st'

  real_T sa_Y0;                        // Expression: 0
                                          //  Referenced by: '<S451>/sa'

  real_T ca_Y0;                        // Expression: 0
                                          //  Referenced by: '<S451>/ca'

  real_T b_Value;                      // Expression: 6356.7523142
                                          //  Referenced by: '<S451>/b'

  real_T a_Value;                      // Expression: 6378.137
                                          //  Referenced by: '<S451>/a'

  real_T Gain_Gain_j;                  // Expression: 2
                                          //  Referenced by: '<S496>/Gain'

  real_T Constant_Value_nh;            // Expression: 1
                                          //  Referenced by: '<S498>/Constant'

  real_T sp11_Y0[11];                  // Expression: (1:(maxdef-1))
                                          //  Referenced by: '<S499>/sp[11]'

  real_T cp11_Y0[11];                  // Expression: (1:(maxdef-1))
                                          //  Referenced by: '<S499>/cp[11]'

  real_T ForIterator_IterationLimit;   // Expression: maxdef-1
                                          //  Referenced by: '<S499>/For Iterator'

  real_T Constant_Value_lw[11];        // Expression: [1:maxdef-1]
                                          //  Referenced by: '<S499>/Constant'

  real_T UnitDelay1_InitialCondition_d;// Expression: 0
                                          //  Referenced by: '<S499>/Unit Delay1'

  real_T cpm1spm1_Threshold;           // Expression: 1
                                          //  Referenced by: '<S499>/cp[m-1] sp[m-1]'

  real_T Constant1_Value_n1[11];       // Expression: [1:maxdef-1]
                                          //  Referenced by: '<S499>/Constant1'

  real_T sp13_Y0[13];                  // Expression: [0 0 (1:(maxdef-1))]
                                          //  Referenced by: '<S452>/sp[13]'

  real_T cp13_Y0[13];                  // Expression: [1 1 (1:(maxdef-1))]
                                          //  Referenced by: '<S452>/cp[13]'

  real_T Gain_Gain_a;                  // Expression: 1
                                          //  Referenced by: '<S452>/Gain'

  real_T Gain1_Gain_n;                 // Expression: 1
                                          //  Referenced by: '<S452>/Gain1'

  real_T cp1_Value;                    // Expression: 1
                                          //  Referenced by: '<S452>/cp[1]'

  real_T sp1_Value;                    // Expression: 0
                                          //  Referenced by: '<S452>/sp[1]'

  real_T x_Y0;                         // Expression: [0]
                                          //  Referenced by: '<S515>/x'

  real_T DistanceintoGustxLimitedtogustl;// Expression: 0
                                            //  Referenced by: '<S515>/Distance into Gust (x) (Limited to gust length d)'

  real_T DistanceintoGustxLimitedtogus_j;// Expression: 0
                                            //  Referenced by: '<S515>/Distance into Gust (x) (Limited to gust length d)'

  real_T Gain_clock_Gain;              // Expression: 1E6
                                          //  Referenced by: '<S12>/Gain_clock'

  real_T IntegratorSecondOrderLimited_IC;// Expression: 0
                                            //  Referenced by: '<S421>/Integrator, Second-Order Limited'

  real_T IntegratorSecondOrderLimited__n;// Expression: 0
                                            //  Referenced by: '<S421>/Integrator, Second-Order Limited'

  real_T Constant_Value_iy;            // Expression: dtype_a
                                          //  Referenced by: '<S417>/Constant'

  real_T u2_Gain;                      // Expression: 0.5
                                          //  Referenced by: '<S25>/1//2'

  real_T Gain_Gain_pu;                 // Expression: 2
                                          //  Referenced by: '<S30>/Gain'

  real_T Gain_Gain_o;                  // Expression: 2
                                          //  Referenced by: '<S33>/Gain'

  real_T Gain_Gain_n;                  // Expression: 2
                                          //  Referenced by: '<S28>/Gain'

  real_T Gain_Gain_nd;                 // Expression: 2
                                          //  Referenced by: '<S34>/Gain'

  real_T Gain_Gain_af;                 // Expression: 2
                                          //  Referenced by: '<S29>/Gain'

  real_T Gain_Gain_c;                  // Expression: 2
                                          //  Referenced by: '<S32>/Gain'

  real_T Constant_Value_b[8];    // Expression: [123544 123544 123544 0 0 0 0 0]
                                    //  Referenced by: '<Root>/Constant'

  real_T Constant1_Value_a[20];
                      // Expression: [3 0 5 1 -1 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
                         //  Referenced by: '<Root>/Constant1'

  real_T Constant2_Value_mm;           // Expression: 1
                                          //  Referenced by: '<S84>/Constant2'

  real_T Constant1_Value_d;            // Expression: R
                                          //  Referenced by: '<S84>/Constant1'

  real_T Constant_Value_kx;            // Expression: 1
                                          //  Referenced by: '<S87>/Constant'

  real_T Constant_Value_n5;            // Expression: 1
                                          //  Referenced by: '<S89>/Constant'

  real_T Constant_Value_c;             // Expression: F
                                          //  Referenced by: '<S88>/Constant'

  real_T f_Value;                      // Expression: 1
                                          //  Referenced by: '<S88>/f'

  real_T Constant_Value_lo;            // Expression: 1
                                          //  Referenced by: '<S84>/Constant'

  real_T Constant3_Value;              // Expression: 1
                                          //  Referenced by: '<S84>/Constant3'

  real_T Constant2_Value_n;            // Expression: 360
                                          //  Referenced by: '<S75>/Constant2'

  real_T Constant_Value_gw;            // Expression: 180
                                          //  Referenced by: '<S68>/Constant'

  real_T Constant1_Value_nq;           // Expression: 0
                                          //  Referenced by: '<S68>/Constant1'

  real_T Constant2_Value_o;            // Expression: 360
                                          //  Referenced by: '<S73>/Constant2'

  real_T Saturation_1_UpperSat;        // Expression: 100000
                                          //  Referenced by: '<S5>/Saturation_1'

  real_T Saturation_1_LowerSat;        // Expression: 0
                                          //  Referenced by: '<S5>/Saturation_1'

  real_T JulianDate_Value;             // Expression: juliandate(year,month,day)
                                          //  Referenced by: '<S66>/Julian Date'

  real_T Merge_InitialOutput_f[4];     // Expression: [1 0 0 0]
                                          //  Referenced by: '<S194>/Merge'

  real_T Merge_InitialOutput_h4[4];    // Expression: [1 0 0 0]
                                          //  Referenced by: '<S193>/Merge'

  real_T Memory_InitialCondition;      // Expression: 0
                                          //  Referenced by: '<Root>/Memory'

  real_T MotorFault3_Gain;             // Expression: 1/4.2
                                          //  Referenced by: '<S2>/MotorFault3'

  real_T WhiteNoise_Mean;              // Expression: 0
                                          //  Referenced by: '<S511>/White Noise'

  real_T WhiteNoise_StdDev;            // Computed Parameter: WhiteNoise_StdDev
                                          //  Referenced by: '<S511>/White Noise'

  real_T u_Value;                      // Expression: 1.0
                                          //  Referenced by: '<S512>/2'

  real_T Constant_111_Value[3];        // Expression: [1,1,1]
                                          //  Referenced by: '<S67>/Constant_[1 1 1]'

  real_T UniformRandomNumber_Minimum[3];// Expression: [-1,-1,-0.5]
                                           //  Referenced by: '<S67>/Uniform Random Number'

  real_T UniformRandomNumber_Maximum[3];// Expression: [1,1,0.5]
                                           //  Referenced by: '<S67>/Uniform Random Number'

  real_T UniformRandomNumber_Seed[3];  // Expression: [564565,6846798,46545]
                                          //  Referenced by: '<S67>/Uniform Random Number'

  real_T Gain_1_Gain;                  // Expression: -1
                                          //  Referenced by: '<S67>/Gain_-1'

  real_T Saturation_2_UpperSat;        // Expression: 10000
                                          //  Referenced by: '<S67>/Saturation_2'

  real_T Saturation_2_LowerSat;        // Expression: 0
                                          //  Referenced by: '<S67>/Saturation_2'

  real_T Constant_V_Value;             // Expression: 5
                                          //  Referenced by: '<S67>/Constant_V'

  real_T LimitFunction10ftto1000ft_Upper;// Expression: max_height_low
                                            //  Referenced by: '<S182>/Limit Function 10ft to 1000ft'

  real_T LimitFunction10ftto1000ft_Lower;// Expression: 10
                                            //  Referenced by: '<S182>/Limit Function 10ft to 1000ft'

  real_T LimitHeighth1000ft_UpperSat;  // Expression: max_height_low
                                          //  Referenced by: '<S165>/Limit Height h<1000ft'

  real_T LimitHeighth1000ft_LowerSat;  // Expression: 0
                                          //  Referenced by: '<S165>/Limit Height h<1000ft'

  real_T sigma_wg_Gain;                // Expression: 0.1
                                          //  Referenced by: '<S165>/sigma_wg '

  real_T PreLookUpIndexSearchaltitude_Br[12];// Expression: h_vec
                                                //  Referenced by: '<S164>/PreLook-Up Index Search  (altitude)'

  real_T PreLookUpIndexSearchprobofexcee[7];// Expression: [1:7]
                                               //  Referenced by: '<S164>/PreLook-Up Index Search  (prob of exceed)'

  real_T MediumHighAltitudeIntensity_Tab[84];// Expression: sigma_vec'
                                                //  Referenced by: '<S164>/Medium//High Altitude Intensity'

  real_T WhiteNoise_Mean_j;            // Expression: 0
                                          //  Referenced by: '<S157>/White Noise'

  real_T WhiteNoise_StdDev_j;         // Computed Parameter: WhiteNoise_StdDev_j
                                         //  Referenced by: '<S157>/White Noise'

  real_T Lv_Gain;                      // Expression: 1
                                          //  Referenced by: '<S154>/Lv'

  real_T Lw_Gain;                      // Expression: 1
                                          //  Referenced by: '<S154>/Lw'

  real_T Constant_DCM_Value[9];        // Expression: eye(3)
                                          //  Referenced by: '<S67>/Constant_DCM'

  real_T LimitFunction10ftto1000ft_Upp_d;// Expression: max_height_low
                                            //  Referenced by: '<S143>/Limit Function 10ft to 1000ft'

  real_T LimitFunction10ftto1000ft_Low_j;// Expression: 10
                                            //  Referenced by: '<S143>/Limit Function 10ft to 1000ft'

  real_T LimitHeighth1000ft_UpperSat_e;// Expression: max_height_low
                                          //  Referenced by: '<S126>/Limit Height h<1000ft'

  real_T LimitHeighth1000ft_LowerSat_o;// Expression: 0
                                          //  Referenced by: '<S126>/Limit Height h<1000ft'

  real_T sigma_wg_Gain_o;              // Expression: 0.1
                                          //  Referenced by: '<S126>/sigma_wg '

  real_T PreLookUpIndexSearchaltitude__g[12];// Expression: h_vec
                                                //  Referenced by: '<S125>/PreLook-Up Index Search  (altitude)'

  real_T PreLookUpIndexSearchprobofexc_n[7];// Expression: [1:7]
                                               //  Referenced by: '<S125>/PreLook-Up Index Search  (prob of exceed)'

  real_T MediumHighAltitudeIntensity_T_b[84];// Expression: sigma_vec'
                                                //  Referenced by: '<S125>/Medium//High Altitude Intensity'

  real_T WhiteNoise_Mean_m;            // Expression: 0
                                          //  Referenced by: '<S118>/White Noise'

  real_T WhiteNoise_StdDev_k;         // Computed Parameter: WhiteNoise_StdDev_k
                                         //  Referenced by: '<S118>/White Noise'

  real_T Lv_Gain_o;                    // Expression: 1
                                          //  Referenced by: '<S115>/Lv'

  real_T Lw_Gain_c;                    // Expression: 1
                                          //  Referenced by: '<S115>/Lw'

  real_T uftinf_UpperSat;              // Expression: inf
                                          //  Referenced by: '<S105>/3ft-->inf'

  real_T uftinf_LowerSat;              // Expression: 3
                                          //  Referenced by: '<S105>/3ft-->inf'

  real_T hz0_Gain;                     // Expression: 1/z0
                                          //  Referenced by: '<S105>/h//z0'

  real_T ref_heightz0_Value;           // Expression: 20/z0
                                          //  Referenced by: '<S105>/ref_height//z0'

  real_T Wdeg1_Value;                  // Expression: 0
                                          //  Referenced by: '<S105>/Wdeg1'

  real_T ZeroOrderHold1_Gain;          // Expression: 1
                                          //  Referenced by: '<S415>/Zero-Order Hold1'

  real_T ZeroOrderHold2_Gain;          // Expression: 1
                                          //  Referenced by: '<S415>/Zero-Order Hold2'

  real_T ZeroOrderHold_Gain;           // Expression: 1
                                          //  Referenced by: '<S415>/Zero-Order Hold'

  real_T centerofgravity_Value[3];     // Expression: [0 0 0]
                                          //  Referenced by: '<S398>/center of gravity'

  real_T ZeroOrderHold4_Gain;          // Expression: 1
                                          //  Referenced by: '<S415>/Zero-Order Hold4'

  real_T Gain_Gain_hr[3];              // Expression: [1 -1 1]
                                          //  Referenced by: '<S415>/Gain'

  real_T Constant_dIdt_Value[9];       // Expression: zeros(3,3)
                                          //  Referenced by: '<S1>/Constant_dI//dt'

  real_T ZeroOrderHold3_Gain;          // Expression: 1
                                          //  Referenced by: '<S415>/Zero-Order Hold3'

  real_T Switch_Threshold;             // Expression: 0.5
                                          //  Referenced by: '<S417>/Switch'

  real_T Saturation_UpperSat[3];       // Expression: a_sath
                                          //  Referenced by: '<S415>/Saturation'

  real_T Saturation_LowerSat[3];       // Expression: a_satl
                                          //  Referenced by: '<S415>/Saturation'

  real_T IntegratorSecondOrderLimited__f;// Expression: 0
                                            //  Referenced by: '<S432>/Integrator, Second-Order Limited'

  real_T IntegratorSecondOrderLimited__l;// Expression: 0
                                            //  Referenced by: '<S432>/Integrator, Second-Order Limited'

  real_T Constant_Value_e;             // Expression: dtype_g
                                          //  Referenced by: '<S430>/Constant'

  real_T ZeroOrderHold_Gain_b;         // Expression: 1
                                          //  Referenced by: '<S416>/Zero-Order Hold'

  real_T ZeroOrderHold1_Gain_i;        // Expression: 1
                                          //  Referenced by: '<S416>/Zero-Order Hold1'

  real_T Switch_Threshold_p;           // Expression: 0.5
                                          //  Referenced by: '<S430>/Switch'

  real_T Saturation_UpperSat_j[3];     // Expression: g_sath
                                          //  Referenced by: '<S416>/Saturation'

  real_T Saturation_LowerSat_a[3];     // Expression: g_satl
                                          //  Referenced by: '<S416>/Saturation'

  real_T epoch_Value;                  // Expression: epoch
                                          //  Referenced by: '<S442>/epoch'

  real_T DecimalYear_Value;            // Expression: dyear
                                          //  Referenced by: '<S411>/Decimal Year'

  real_T otime_InitialCondition;       // Expression: -1000
                                          //  Referenced by: '<S456>/otime'

  real_T Constant_Value_go;            // Expression: 180
                                          //  Referenced by: '<S438>/Constant'

  real_T Constant2_Value_e;            // Expression: 360
                                          //  Referenced by: '<S447>/Constant2'

  real_T Constant1_Value_ak;           // Expression: 0
                                          //  Referenced by: '<S438>/Constant1'

  real_T Constant2_Value_c;            // Expression: 360
                                          //  Referenced by: '<S445>/Constant2'

  real_T olon_InitialCondition;        // Expression: -1000
                                          //  Referenced by: '<S455>/olon'

  real_T olat_InitialCondition;        // Expression: -1000
                                          //  Referenced by: '<S454>/olat'

  real_T Gain_Gain_d0;                 // Expression: 0.001
                                          //  Referenced by: '<S411>/Gain'

  real_T oalt_InitialCondition;        // Expression: -1000
                                          //  Referenced by: '<S454>/oalt'

  real_T re_Value;                     // Expression: 6371.2
                                          //  Referenced by: '<S442>/re'

  real_T Gain_Mag_Gain;                // Expression: 1
                                          //  Referenced by: '<S398>/Gain_Mag'

  real_T nT2Gauss_Gain;                // Expression: 1E-5
                                          //  Referenced by: '<S398>/nT2Gauss'

  real_T SeaLevelTemperature_Value;    // Expression: T0
                                          //  Referenced by: '<S408>/Sea Level  Temperature'

  real_T SeaLevelTemperature_Value_i;  // Expression: T0
                                          //  Referenced by: '<S64>/Sea Level  Temperature'

  real_T Limitaltitudetotroposhere_Upper;// Expression: h_trop
                                            //  Referenced by: '<S64>/Limit  altitude  to troposhere'

  real_T Limitaltitudetotroposhere_Lower;// Expression: h0
                                            //  Referenced by: '<S64>/Limit  altitude  to troposhere'

  real_T LapseRate_Gain;               // Expression: L
                                          //  Referenced by: '<S64>/Lapse Rate'

  real_T uT0_Gain;                     // Expression: 1/T0
                                          //  Referenced by: '<S64>/1//T0'

  real_T Constant_Value_mq;            // Expression: g/(L*R)
                                          //  Referenced by: '<S64>/Constant'

  real_T rho0_Gain;                    // Expression: rho0
                                          //  Referenced by: '<S64>/rho0'

  real_T AltitudeofTroposphere_Value;  // Expression: h_trop
                                          //  Referenced by: '<S64>/Altitude of Troposphere'

  real_T LimitaltitudetoStratosphere_Upp;// Expression: 0
                                            //  Referenced by: '<S64>/Limit  altitude  to Stratosphere'

  real_T LimitaltitudetoStratosphere_Low;// Expression: h_trop-h_strat
                                            //  Referenced by: '<S64>/Limit  altitude  to Stratosphere'

  real_T gR_Gain;                      // Expression: g/R
                                          //  Referenced by: '<S64>/g//R'

  real_T u2rhoV2_Gain;                 // Expression: 1/2
                                          //  Referenced by: '<S400>/1//2rhoV^2'

  real_T UniformRandomNumber_Minimum_d;// Expression: -2
                                          //  Referenced by: '<S398>/Uniform Random Number'

  real_T UniformRandomNumber_Maximum_m;// Expression: 2
                                          //  Referenced by: '<S398>/Uniform Random Number'

  real_T UniformRandomNumber_Seed_g;   // Expression: 15634
                                          //  Referenced by: '<S398>/Uniform Random Number'

  real_T Gain5_Gain;                   // Expression: 0.2
                                          //  Referenced by: '<S398>/Gain5'

  real_T Constant2_Value_oq;           // Expression: 0.3
                                          //  Referenced by: '<S406>/Constant2'

  real_T Limitaltitudetotroposhere_Upp_h;// Expression: h_trop
                                            //  Referenced by: '<S408>/Limit  altitude  to troposhere'

  real_T Limitaltitudetotroposhere_Low_g;// Expression: h0
                                            //  Referenced by: '<S408>/Limit  altitude  to troposhere'

  real_T LapseRate_Gain_g;             // Expression: L
                                          //  Referenced by: '<S408>/Lapse Rate'

  real_T uT0_Gain_d;                   // Expression: 1/T0
                                          //  Referenced by: '<S408>/1//T0'

  real_T Constant_Value_o;             // Expression: g/(L*R)
                                          //  Referenced by: '<S408>/Constant'

  real_T P0_Gain;                      // Expression: P0
                                          //  Referenced by: '<S408>/P0'

  real_T AltitudeofTroposphere_Value_f;// Expression: h_trop
                                          //  Referenced by: '<S408>/Altitude of Troposphere'

  real_T LimitaltitudetoStratosphere_U_p;// Expression: 0
                                            //  Referenced by: '<S408>/Limit  altitude  to Stratosphere'

  real_T LimitaltitudetoStratosphere_L_f;// Expression: h_trop-h_strat
                                            //  Referenced by: '<S408>/Limit  altitude  to Stratosphere'

  real_T gR_Gain_d;                    // Expression: g/R
                                          //  Referenced by: '<S408>/g//R'

  real_T Gain_Gain_io;                 // Expression: 0.01
                                          //  Referenced by: '<S398>/Gain'

  real_T UniformRandomNumber4_Minimum; // Expression: -1
                                          //  Referenced by: '<S398>/Uniform Random Number4'

  real_T UniformRandomNumber4_Maximum; // Expression: 1
                                          //  Referenced by: '<S398>/Uniform Random Number4'

  real_T UniformRandomNumber4_Seed;    // Expression: 25634
                                          //  Referenced by: '<S398>/Uniform Random Number4'

  real_T Gain9_Gain;                   // Expression: 0.01
                                          //  Referenced by: '<S398>/Gain9'

  real_T Constant_Value_ly;            // Expression: 0.5
                                          //  Referenced by: '<S407>/Constant'

  real_T Gain2_Gain_a;                 // Expression: 0.7
                                          //  Referenced by: '<S407>/Gain2'

  real_T Constant2_Value_i;            // Expression: 0.3
                                          //  Referenced by: '<S407>/Constant2'

  real_T Gain1_Gain_g;                 // Expression: 0.01
                                          //  Referenced by: '<S398>/Gain1'

  real_T Constant1_Value_j;            // Expression: 10
                                          //  Referenced by: '<S5>/Constant1'

  real_T Constant_Value_gb;            // Expression: 8191
                                          //  Referenced by: '<S394>/Constant'

  real_T Merge_InitialOutput_fc[4];    // Expression: [1 0 0 0]
                                          //  Referenced by: '<S321>/Merge'

  real_T Gain_Gain_b;                  // Expression: 1E7
                                          //  Referenced by: '<S324>/Gain'

  real_T Gain1_Gain_k;                 // Expression: 1E7
                                          //  Referenced by: '<S324>/Gain1'

  real_T Gain3_Gain;                   // Expression: 1E3
                                          //  Referenced by: '<S324>/Gain3'

  real_T Gain4_Gain;                   // Expression: 100
                                          //  Referenced by: '<S324>/Gain4'

  real_T u2rhoV2_Gain_a;               // Expression: 1/2
                                          //  Referenced by: '<S507>/1//2rhoV^2'

  real_T Gain2_Gain_j;                 // Expression: 2/1.225
                                          //  Referenced by: '<S324>/Gain2'

  real_T Gain7_Gain;                   // Expression: 100
                                          //  Referenced by: '<S324>/Gain7'

  real_T Constant2_Value_k;            // Expression: 1
                                          //  Referenced by: '<S388>/Constant2'

  real_T Constant1_Value_d0;           // Expression: R
                                          //  Referenced by: '<S388>/Constant1'

  real_T Constant_Value_cc;            // Expression: 1
                                          //  Referenced by: '<S391>/Constant'

  real_T Constant_Value_ol;            // Expression: 1
                                          //  Referenced by: '<S393>/Constant'

  real_T Constant_Value_na;            // Expression: F
                                          //  Referenced by: '<S392>/Constant'

  real_T f_Value_l;                    // Expression: 1
                                          //  Referenced by: '<S392>/f'

  real_T Constant_Value_br;            // Expression: 1
                                          //  Referenced by: '<S388>/Constant'

  real_T Constant3_Value_l;            // Expression: 1
                                          //  Referenced by: '<S388>/Constant3'

  real_T Constant2_Value_nr;           // Expression: 360
                                          //  Referenced by: '<S379>/Constant2'

  real_T Constant_Value_bd;            // Expression: 180
                                          //  Referenced by: '<S372>/Constant'

  real_T Constant1_Value_f;            // Expression: 0
                                          //  Referenced by: '<S372>/Constant1'

  real_T Constant2_Value_g;            // Expression: 360
                                          //  Referenced by: '<S377>/Constant2'

  real_T Saturation_UpperSat_h;        // Expression: 100000
                                          //  Referenced by: '<S366>/Saturation'

  real_T Saturation_LowerSat_au;       // Expression: 0
                                          //  Referenced by: '<S366>/Saturation'

  real_T latScale_Gain;                // Expression: 1E7
                                          //  Referenced by: '<S322>/latScale'

  real_T lonScale_Gain;                // Expression: 1E7
                                          //  Referenced by: '<S322>/lonScale'

  real_T altScale_Gain;                // Expression: 1E3
                                          //  Referenced by: '<S322>/altScale'

  real_T Gain6_Gain;                   // Expression: 100
                                          //  Referenced by: '<S322>/Gain6'

  real_T Gain8_Gain;                   // Expression: 100
                                          //  Referenced by: '<S322>/Gain8'

  real_T TransferFcn4_A;               // Computed Parameter: TransferFcn4_A
                                          //  Referenced by: '<S368>/Transfer Fcn4'

  real_T TransferFcn4_C;               // Computed Parameter: TransferFcn4_C
                                          //  Referenced by: '<S368>/Transfer Fcn4'

  real_T TransferFcn1_A;               // Computed Parameter: TransferFcn1_A
                                          //  Referenced by: '<S368>/Transfer Fcn1'

  real_T TransferFcn1_C;               // Computed Parameter: TransferFcn1_C
                                          //  Referenced by: '<S368>/Transfer Fcn1'

  real_T TransferFcn2_A;               // Computed Parameter: TransferFcn2_A
                                          //  Referenced by: '<S368>/Transfer Fcn2'

  real_T TransferFcn2_C;               // Computed Parameter: TransferFcn2_C
                                          //  Referenced by: '<S368>/Transfer Fcn2'

  real_T VelScale_Gain;                // Expression: 1E2
                                          //  Referenced by: '<S322>/VelScale'

  real_T VeScale_Gain;                 // Expression: 1E2
                                          //  Referenced by: '<S322>/VeScale'

  real_T AngleScale_Gain;              // Expression: 1E2
                                          //  Referenced by: '<S322>/AngleScale'

  real_T Gain1_Gain_c;                 // Expression: 1E3
                                          //  Referenced by: '<S322>/Gain1'

  real_T Gain3_Gain_l;                 // Expression: 1E3
                                          //  Referenced by: '<S322>/Gain3'

  real_T VeScale1_Gain;                // Expression: 1E3
                                          //  Referenced by: '<S322>/VeScale1'

  real_T VelScale1_Gain;               // Expression: 1E3
                                          //  Referenced by: '<S322>/VelScale1'

  real_T headMotAngle_Gain;            // Expression: 1E5
                                          //  Referenced by: '<S322>/headMotAngle'

  real_T headVehAngleScale_Gain;       // Expression: 1E5
                                          //  Referenced by: '<S322>/headVehAngleScale'

  real_T magDecGain_Gain;              // Expression: 100
                                          //  Referenced by: '<S398>/magDecGain'

  real_T Gain_Gain_ja;                 // Expression: 60/2/pi
                                          //  Referenced by: '<S12>/Gain'

  real_T CopterID_Value;               // Expression: 1
                                          //  Referenced by: '<S12>/CopterID'

  real_T Constant_Value_f[23];
                  // Expression: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                     //  Referenced by: '<S8>/Constant'

  real_T Constant_Value_h;             // Expression: 1
                                          //  Referenced by: '<S26>/Constant'

  real_T UniformRandomNumber4_Minimum_o[3];// Expression: [-1,-1,-2]
                                              //  Referenced by: '<S322>/Uniform Random Number4'

  real_T UniformRandomNumber4_Maximum_c[3];// Expression: [1,1,2]
                                              //  Referenced by: '<S322>/Uniform Random Number4'

  real_T UniformRandomNumber4_Seed_e[3];// Expression: [5445,45433,33433]
                                           //  Referenced by: '<S322>/Uniform Random Number4'

  real_T BiasGain1_Gain;               // Expression: 0.1
                                          //  Referenced by: '<S322>/BiasGain1'

  int32_T Constant1_Value_l;           // Computed Parameter: Constant1_Value_l
                                          //  Referenced by: '<S468>/Constant1'

  int32_T Constant_Value_op;           // Computed Parameter: Constant_Value_op
                                          //  Referenced by: '<S469>/Constant'

  int32_T Constant_Value_mh;           // Computed Parameter: Constant_Value_mh
                                          //  Referenced by: '<S467>/Constant'

  int32_T Constant_Value_os;           // Computed Parameter: Constant_Value_os
                                          //  Referenced by: '<S478>/Constant'

  int32_T Gain_Gain_bd;                // Computed Parameter: Gain_Gain_bd
                                          //  Referenced by: '<S478>/Gain'

  int32_T Constant_Value_es;           // Computed Parameter: Constant_Value_es
                                          //  Referenced by: '<S481>/Constant'

  int32_T Gain_Gain_hz;                // Computed Parameter: Gain_Gain_hz
                                          //  Referenced by: '<S480>/Gain'

  int32_T Constant_Value_py;           // Computed Parameter: Constant_Value_py
                                          //  Referenced by: '<S484>/Constant'

  int32_T Constant1_Value_c;           // Computed Parameter: Constant1_Value_c
                                          //  Referenced by: '<S484>/Constant1'

  int32_T Constant1_Value_fy;          // Computed Parameter: Constant1_Value_fy
                                          //  Referenced by: '<S485>/Constant1'

  int32_T Constant_Value_g2;           // Computed Parameter: Constant_Value_g2
                                          //  Referenced by: '<S483>/Constant'

  int32_T Constant1_Value_m;           // Computed Parameter: Constant1_Value_m
                                          //  Referenced by: '<S482>/Constant1'

  int32_T Gain_Gain_aa;                // Computed Parameter: Gain_Gain_aa
                                          //  Referenced by: '<S482>/Gain'

  int32_T Constant1_Value_hs;          // Computed Parameter: Constant1_Value_hs
                                          //  Referenced by: '<S486>/Constant1'

  int32_T Constant_Value_fi;           // Computed Parameter: Constant_Value_fi
                                          //  Referenced by: '<S460>/Constant'

  int32_T Constant_Value_lp;           // Computed Parameter: Constant_Value_lp
                                          //  Referenced by: '<S477>/Constant'

  int32_T Gain_Gain_ob;                // Computed Parameter: Gain_Gain_ob
                                          //  Referenced by: '<S477>/Gain'

  int32_T Constant_Value_g0;           // Computed Parameter: Constant_Value_g0
                                          //  Referenced by: '<S487>/Constant'

  int32_T Constant1_Value_g;           // Computed Parameter: Constant1_Value_g
                                          //  Referenced by: '<S487>/Constant1'

  int32_T Constant_Value_nf;           // Computed Parameter: Constant_Value_nf
                                          //  Referenced by: '<S489>/Constant'

  int32_T tc_old_Threshold;            // Computed Parameter: tc_old_Threshold
                                          //  Referenced by: '<S488>/tc_old'

  int32_T Constant_Value_j;            // Computed Parameter: Constant_Value_j
                                          //  Referenced by: '<S459>/Constant'

  int32_T Constant1_Value_ja;          // Computed Parameter: Constant1_Value_ja
                                          //  Referenced by: '<S459>/Constant1'

  int32_T Constant_Value_cy;           // Computed Parameter: Constant_Value_cy
                                          //  Referenced by: '<S458>/Constant'

  int32_T Constant_Value_pe;           // Computed Parameter: Constant_Value_pe
                                          //  Referenced by: '<S463>/Constant'

  int32_T Gain_Gain_ij;                // Computed Parameter: Gain_Gain_ij
                                          //  Referenced by: '<S463>/Gain'

  int32_T Constant_Value_a;            // Computed Parameter: Constant_Value_a
                                          //  Referenced by: '<S465>/Constant'

  int32_T ForIterator_IterationLimit_h;
                             // Computed Parameter: ForIterator_IterationLimit_h
                                //  Referenced by: '<S450>/For Iterator'

  int32_T Constant_Value_lq;           // Computed Parameter: Constant_Value_lq
                                          //  Referenced by: '<S450>/Constant'

  int32_T arn_Threshold;               // Computed Parameter: arn_Threshold
                                          //  Referenced by: '<S450>/ar(n)'

  int32_T FaultID_Value;         // Expression: FaultParamStruct.LoadFallFaultID
                                    //  Referenced by: '<S187>/FaultID'

  int32_T FaultID1_Value;       // Expression: FaultParamStruct.LoadShiftFaultID
                                   //  Referenced by: '<S187>/FaultID1'

  int32_T FaultID2_Value;        // Expression: FaultParamStruct.LoadLeakFaultID
                                    //  Referenced by: '<S187>/FaultID2'

  int32_T FaultID_Value_d;             // Expression: FaultParamStruct.FaultID
                                          //  Referenced by: '<S510>/FaultID'

  int32_T FaultID_Value_h;
                       // Expression: FaultParamStruct.UseCusTomHoverTimeFaultID
                          //  Referenced by: '<S57>/FaultID'

  int32_T FaultID1_Value_h;        // Expression: FaultParamStruct.PowOffFaultID
                                      //  Referenced by: '<S57>/FaultID1'

  int32_T FaultID2_Value_m;    // Expression: FaultParamStruct.LowVoltageFaultID
                                  //  Referenced by: '<S57>/FaultID2'

  int32_T FaultID3_Value;     // Expression: FaultParamStruct.LowCapacityFaultID
                                 //  Referenced by: '<S57>/FaultID3'

  int32_T FaultID_Value_n;             // Expression: FaultParamStruct.FaultID
                                          //  Referenced by: '<S293>/FaultID'

  int32_T FaultID4_Value;       // Expression: FaultParamStruct.NoiseWindFaultID
                                   //  Referenced by: '<S67>/FaultID4'

  int32_T FaultID_Value_a;      // Expression: FaultParamStruct.ConstWindFaultID
                                   //  Referenced by: '<S67>/FaultID'

  int32_T FaultID1_Value_p;      // Expression: FaultParamStruct.GustWindFaultID
                                    //  Referenced by: '<S67>/FaultID1'

  int32_T FaultID2_Value_j;      // Expression: FaultParamStruct.TurbWindFaultID
                                    //  Referenced by: '<S67>/FaultID2'

  int32_T FaultID3_Value_d;     // Expression: FaultParamStruct.SheerWindFaultID
                                   //  Referenced by: '<S67>/FaultID3'

  int32_T FaultID_Value_f;            // Expression: FaultParamStruct.AccFaultID
                                         //  Referenced by: '<S398>/FaultID'

  int32_T FaultID1_Value_i;          // Expression: FaultParamStruct.GyroFaultID
                                        //  Referenced by: '<S398>/FaultID1'

  int32_T FaultID2_Value_d;           // Expression: FaultParamStruct.MagFaultID
                                         //  Referenced by: '<S398>/FaultID2'

  int32_T FaultID3_Value_l;          // Expression: FaultParamStruct.BaroFaultID
                                        //  Referenced by: '<S398>/FaultID3'

  int32_T FaultID_Value_p;            // Expression: FaultParamStruct.GPSFaultID
                                         //  Referenced by: '<S366>/FaultID'

  real32_T Gain5_Gain_p;               // Computed Parameter: Gain5_Gain_p
                                          //  Referenced by: '<S324>/Gain5'

  uint32_T MediumHighAltitudeIntensity_max[2];
                          // Computed Parameter: MediumHighAltitudeIntensity_max
                             //  Referenced by: '<S164>/Medium//High Altitude Intensity'

  uint32_T MediumHighAltitudeIntensity_m_i[2];
                          // Computed Parameter: MediumHighAltitudeIntensity_m_i
                             //  Referenced by: '<S125>/Medium//High Altitude Intensity'

  P_Distanceintogusty_QuadModel_T Distanceintogustz;// '<S512>/Distance into gust (z)' 
  P_Distanceintogusty_QuadModel_T Distanceintogusty;// '<S512>/Distance into gust (y)' 
  P_IfWarningError_QuadModelv_T IfWarningError_a;// '<S329>/If Warning//Error'
  P_NegativeTrace_QuadModelv_T NegativeTrace_l;// '<S321>/Negative Trace'
  P_PositiveTrace_QuadModelv_T PositiveTrace_b;// '<S321>/Positive Trace'
  P_MotorNonlinearDynamic1_Quad_T MotorNonlinearDynamic8;// '<S294>/MotorNonlinearDynamic8' 
  P_MotorNonlinearDynamic1_Quad_T MotorNonlinearDynamic7;// '<S294>/MotorNonlinearDynamic7' 
  P_MotorNonlinearDynamic1_Quad_T MotorNonlinearDynamic6;// '<S294>/MotorNonlinearDynamic6' 
  P_MotorNonlinearDynamic1_Quad_T MotorNonlinearDynamic5;// '<S294>/MotorNonlinearDynamic5' 
  P_MotorNonlinearDynamic1_Quad_T MotorNonlinearDynamic4;// '<S294>/MotorNonlinearDynamic4' 
  P_MotorNonlinearDynamic1_Quad_T MotorNonlinearDynamic3;// '<S294>/MotorNonlinearDynamic3' 
  P_MotorNonlinearDynamic1_Quad_T MotorNonlinearDynamic2;// '<S294>/MotorNonlinearDynamic2' 
  P_MotorNonlinearDynamic1_Quad_T MotorNonlinearDynamic1;// '<S294>/MotorNonlinearDynamic1' 
  P_IfWarningError_QuadModelv_T IfWarningError_f;// '<S241>/If Warning//Error'
  P_NegativeTrace_QuadModelv_T NegativeTrace_d;// '<S194>/Negative Trace'
  P_PositiveTrace_QuadModelv_T PositiveTrace_d;// '<S194>/Positive Trace'
  P_IfWarningError_QuadModelv_T IfWarningError;// '<S203>/If Warning//Error'
  P_NegativeTrace_QuadModelv_T NegativeTrace;// '<S193>/Negative Trace'
  P_PositiveTrace_QuadModelv_T PositiveTrace;// '<S193>/Positive Trace'
  P_Interpolatevelocities_QuadM_T Interpolatevelocities_i;// '<S153>/Interpolate  velocities' 
  P_Interpolaterates_QuadModelv_T Interpolaterates_g;// '<S152>/Interpolate  rates' 
  P_Hwgwz_QuadModelv_T Hwgwz_a;        // '<S148>/Hwgw(z)'
  P_Hvgwz_QuadModelv_T Hvgwz_f;        // '<S148>/Hvgw(z)'
  P_Hugwz_QuadModelv_T Hugwz_g;        // '<S148>/Hugw(z)'
  P_Hrgw_QuadModelv_T Hrgw_a;          // '<S147>/Hrgw'
  P_Hqgw_QuadModelv_T Hqgw_a;          // '<S147>/Hqgw'
  P_Hpgw_QuadModelv_T Hpgw_h;          // '<S147>/Hpgw'
  P_Interpolatevelocities_QuadM_T Interpolatevelocities;// '<S114>/Interpolate  velocities' 
  P_Interpolaterates_QuadModelv_T Interpolaterates;// '<S113>/Interpolate  rates' 
  P_Hwgwz_QuadModelv_T Hwgwz;          // '<S109>/Hwgw(z)'
  P_Hvgwz_QuadModelv_T Hvgwz;          // '<S109>/Hvgw(z)'
  P_Hugwz_QuadModelv_T Hugwz;          // '<S109>/Hugw(z)'
  P_Hrgw_QuadModelv_T Hrgw;            // '<S108>/Hrgw'
  P_Hqgw_QuadModelv_T Hqgw;            // '<S108>/Hqgw'
  P_Hpgw_QuadModelv_T Hpgw;            // '<S108>/Hpgw'
};

// Parameters (default storage)
typedef struct P_QuadModelv_T_ P_QuadModelv_T;

// Real-time Model Data Structure
struct tag_RTM_QuadModelv_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_QuadModelv_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis_QuadModelv_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[39];
  real_T odeF[4][39];
  ODE4_IntgData intgData;

  //
  //  Sizes:
  //  The following substructure contains sizes information
  //  for many of the model attributes such as inputs, outputs,
  //  dwork, sample times, etc.

  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    boolean_T firstInitCondFlag;
    struct {
      uint16_T TID[6];
    } TaskCounters;

    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[6];
  } Timing;
};

extern "C"
{
  static real_T rtGetInf(void);
  static real32_T rtGetInfF(void);
  static real_T rtGetMinusInf(void);
  static real32_T rtGetMinusInfF(void);
}                                      // extern "C"

extern "C"
{
  static real_T rtGetNaN(void);
  static real32_T rtGetNaNF(void);
}                                      // extern "C"

#define NOT_USING_NONFINITE_LITERALS   1

extern "C"
{
  extern real_T rtInf;
  extern real_T rtMinusInf;
  extern real_T rtNaN;
  extern real32_T rtInfF;
  extern real32_T rtMinusInfF;
  extern real32_T rtNaNF;
  static void rt_InitInfAndNaN(size_t realSize);
  static boolean_T rtIsInf(real_T value);
  static boolean_T rtIsInfF(real32_T value);
  static boolean_T rtIsNaN(real_T value);
  static boolean_T rtIsNaNF(real32_T value);
  struct BigEndianIEEEDouble {
    struct {
      uint32_T wordH;
      uint32_T wordL;
    } words;
  };

  struct LittleEndianIEEEDouble {
    struct {
      uint32_T wordL;
      uint32_T wordH;
    } words;
  };

  struct IEEESingle {
    union {
      real32_T wordLreal;
      uint32_T wordLuint;
    } wordL;
  };
}                                      // extern "C"

// Class declaration for model QuadModelv
class MulticopterModelClass
{
  // public data and function members
 public:
  // Real-Time Model get method
  RT_MODEL_QuadModelv_T * getRTM();

  // External inputs
  ExtU_QuadModelv_T QuadModelv_U;

  // External outputs
  ExtY_QuadModelv_T QuadModelv_Y;

  // Tunable parameters
  static P_QuadModelv_T QuadModelv_P;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  static void terminate();

  // Constructor
  MulticopterModelClass();

  // Destructor
  ~MulticopterModelClass();

  // private data and function members
 private:
  // Block signals
  B_QuadModelv_T QuadModelv_B;

  // Block states
  DW_QuadModelv_T QuadModelv_DW;

  // Block continuous states
  X_QuadModelv_T QuadModelv_X;

  // private member function(s) for subsystem '<S108>/Hpgw'
  static void QuadModelv_Hpgw_Init(B_Hpgw_QuadModelv_T *localB,
    DW_Hpgw_QuadModelv_T *localDW, P_Hpgw_QuadModelv_T *localP);
  static void QuadModelv_Hpgw_Reset(DW_Hpgw_QuadModelv_T *localDW,
    P_Hpgw_QuadModelv_T *localP);
  static void QuadModelv_Hpgw_Disable(B_Hpgw_QuadModelv_T *localB,
    DW_Hpgw_QuadModelv_T *localDW, P_Hpgw_QuadModelv_T *localP);
  void QuadModelv_Hpgw_Update(B_Hpgw_QuadModelv_T *localB, DW_Hpgw_QuadModelv_T *
    localDW);
  void QuadModelv_Hpgw(real_T rtu_Enable, const real_T rtu_L_wg[2], real_T
                       rtu_sigma_wg, real_T rtu_sigma_wg_e, real_T rtu_Noise,
                       real_T rtu_wingspan, B_Hpgw_QuadModelv_T *localB,
                       DW_Hpgw_QuadModelv_T *localDW, P_Hpgw_QuadModelv_T
                       *localP);

  // private member function(s) for subsystem '<S108>/Hqgw'
  static void QuadModelv_Hqgw_Init(B_Hqgw_QuadModelv_T *localB,
    DW_Hqgw_QuadModelv_T *localDW, P_Hqgw_QuadModelv_T *localP);
  static void QuadModelv_Hqgw_Reset(DW_Hqgw_QuadModelv_T *localDW,
    P_Hqgw_QuadModelv_T *localP);
  static void QuadModelv_Hqgw_Disable(B_Hqgw_QuadModelv_T *localB,
    DW_Hqgw_QuadModelv_T *localDW, P_Hqgw_QuadModelv_T *localP);
  void QuadModelv_Hqgw_Update(const real_T rtu_wg[2], B_Hqgw_QuadModelv_T
    *localB, DW_Hqgw_QuadModelv_T *localDW);
  void QuadModelv_Hqgw(real_T rtu_Enable, real_T rtu_V, const real_T rtu_wg[2],
                       real_T rtu_wingspan, B_Hqgw_QuadModelv_T *localB,
                       DW_Hqgw_QuadModelv_T *localDW, P_Hqgw_QuadModelv_T
                       *localP);

  // private member function(s) for subsystem '<S108>/Hrgw'
  static void QuadModelv_Hrgw_Init(B_Hrgw_QuadModelv_T *localB,
    DW_Hrgw_QuadModelv_T *localDW, P_Hrgw_QuadModelv_T *localP);
  static void QuadModelv_Hrgw_Reset(DW_Hrgw_QuadModelv_T *localDW,
    P_Hrgw_QuadModelv_T *localP);
  static void QuadModelv_Hrgw_Disable(B_Hrgw_QuadModelv_T *localB,
    DW_Hrgw_QuadModelv_T *localDW, P_Hrgw_QuadModelv_T *localP);
  void QuadModelv_Hrgw_Update(const real_T rtu_vg[2], B_Hrgw_QuadModelv_T
    *localB, DW_Hrgw_QuadModelv_T *localDW);
  void QuadModelv_Hrgw(real_T rtu_Enable, real_T rtu_V, const real_T rtu_vg[2],
                       real_T rtu_wingspan, B_Hrgw_QuadModelv_T *localB,
                       DW_Hrgw_QuadModelv_T *localDW, P_Hrgw_QuadModelv_T
                       *localP);

  // private member function(s) for subsystem '<S109>/Hugw(z)'
  static void QuadModelv_Hugwz_Init(B_Hugwz_QuadModelv_T *localB,
    DW_Hugwz_QuadModelv_T *localDW, P_Hugwz_QuadModelv_T *localP);
  static void QuadModelv_Hugwz_Reset(DW_Hugwz_QuadModelv_T *localDW,
    P_Hugwz_QuadModelv_T *localP);
  static void QuadModelv_Hugwz_Disable(B_Hugwz_QuadModelv_T *localB,
    DW_Hugwz_QuadModelv_T *localDW, P_Hugwz_QuadModelv_T *localP);
  void QuadModelv_Hugwz_Update(B_Hugwz_QuadModelv_T *localB,
    DW_Hugwz_QuadModelv_T *localDW);
  void QuadModelv_Hugwz(real_T rtu_Enable, real_T rtu_V, real_T rtu_L_ug, real_T
                        rtu_L_ug_i, real_T rtu_sigma_ug, real_T rtu_sigma_ug_j,
                        real_T rtu_Noise, B_Hugwz_QuadModelv_T *localB,
                        DW_Hugwz_QuadModelv_T *localDW, P_Hugwz_QuadModelv_T
                        *localP);

  // private member function(s) for subsystem '<S109>/Hvgw(z)'
  static void QuadModelv_Hvgwz_Init(B_Hvgwz_QuadModelv_T *localB,
    DW_Hvgwz_QuadModelv_T *localDW, P_Hvgwz_QuadModelv_T *localP);
  static void QuadModelv_Hvgwz_Reset(DW_Hvgwz_QuadModelv_T *localDW,
    P_Hvgwz_QuadModelv_T *localP);
  static void QuadModelv_Hvgwz_Disable(B_Hvgwz_QuadModelv_T *localB,
    DW_Hvgwz_QuadModelv_T *localDW, P_Hvgwz_QuadModelv_T *localP);
  void QuadModelv_Hvgwz_Update(B_Hvgwz_QuadModelv_T *localB,
    DW_Hvgwz_QuadModelv_T *localDW);
  void QuadModelv_Hvgwz(real_T rtu_Enable, real_T rtu_sigma_vg, real_T
                        rtu_sigma_vg_e, const real_T rtu_L_vg[2], real_T rtu_V,
                        real_T rtu_Noise, B_Hvgwz_QuadModelv_T *localB,
                        DW_Hvgwz_QuadModelv_T *localDW, P_Hvgwz_QuadModelv_T
                        *localP);

  // private member function(s) for subsystem '<S109>/Hwgw(z)'
  static void QuadModelv_Hwgwz_Init(B_Hwgwz_QuadModelv_T *localB,
    DW_Hwgwz_QuadModelv_T *localDW, P_Hwgwz_QuadModelv_T *localP);
  static void QuadModelv_Hwgwz_Reset(DW_Hwgwz_QuadModelv_T *localDW,
    P_Hwgwz_QuadModelv_T *localP);
  static void QuadModelv_Hwgwz_Disable(B_Hwgwz_QuadModelv_T *localB,
    DW_Hwgwz_QuadModelv_T *localDW, P_Hwgwz_QuadModelv_T *localP);
  void QuadModelv_Hwgwz_Update(B_Hwgwz_QuadModelv_T *localB,
    DW_Hwgwz_QuadModelv_T *localDW);
  void QuadModelv_Hwgwz(real_T rtu_Enable, real_T rtu_V, const real_T rtu_L_wg[2],
                        real_T rtu_sigma_wg, real_T rtu_sigma_wg_j, real_T
                        rtu_Noise, B_Hwgwz_QuadModelv_T *localB,
                        DW_Hwgwz_QuadModelv_T *localDW, P_Hwgwz_QuadModelv_T
                        *localP);

  // private member function(s) for subsystem '<S113>/Low altitude  rates'
  static void QuadModelv_Lowaltituderates(const real_T rtu_DCM[9], const real_T
    rtu_pgw_hl[2], const real_T rtu_qgw_hl[2], const real_T rtu_rgw_hl[2],
    real_T rtu_Winddirection, real_T rty_pgwqgwrgw[3]);

  // private member function(s) for subsystem '<S113>/Interpolate  rates'
  static void QuadModelv_Interpolaterates(const real_T rtu_pgw_hl[2], const
    real_T rtu_qgw_hl[2], const real_T rtu_rgw_hl[2], const real_T rtu_DCM[9],
    real_T rtu_Winddirection, real_T rtu_Altitude, real_T rty_pgwqgwrgw[3],
    P_Interpolaterates_QuadModelv_T *localP);

  // private member function(s) for subsystem '<S114>/Interpolate  velocities'
  static void QuadModel_Interpolatevelocities(const real_T rtu_ugw_hl[2], const
    real_T rtu_vgw_hl[2], const real_T rtu_wgw_hl[2], const real_T rtu_DCM[9],
    real_T rtu_Winddirection, real_T rtu_Altitude, real_T rty_ugwvgwwgw[3],
    P_Interpolatevelocities_QuadM_T *localP);

  // private member function(s) for subsystem '<S193>/Positive Trace'
  static void QuadModelv_PositiveTrace(real_T rtu_traceDCM, const real_T
    rtu_DCM[9], real_T *rty_qwqxqyqz, real_T rty_qwqxqyqz_a[3],
    P_PositiveTrace_QuadModelv_T *localP);

  // private member function(s) for subsystem '<S193>/Negative Trace'
  static void QuadModelv_NegativeTrace_Init(DW_NegativeTrace_QuadModelv_T
    *localDW);
  void QuadModelv_NegativeTrace(const real_T rtu_DCM[9], real_T rty_qwqxqyqz[4],
    DW_NegativeTrace_QuadModelv_T *localDW, P_NegativeTrace_QuadModelv_T *localP);

  // private member function(s) for subsystem '<S203>/If Warning//Error'
  static void QuadModelv_IfWarningError(const real_T rtu_dcm[9], real_T
    rtp_action, real_T rtp_tolerance, P_IfWarningError_QuadModelv_T *localP);

  // private member function(s) for subsystem '<S297>/Motor_Dynamics'
  void QuadModelv_Motor_Dynamics_Init(DW_Motor_Dynamics_QuadModelv_T *localDW,
    X_Motor_Dynamics_QuadModelv_T *localX) const;
  static void QuadModelv_Motor_Dynamics_Deriv(B_Motor_Dynamics_QuadModelv_T
    *localB, XDot_Motor_Dynamics_QuadModel_T *localXdot);
  static void QuadModel_Motor_Dynamics_Update(DW_Motor_Dynamics_QuadModelv_T
    *localDW);
  static void QuadModelv_Motor_Dynamics(real_T rtu_motor_rate_d, real_T
    rtu_ModelInit_RPM, real_T rtu_motorT, B_Motor_Dynamics_QuadModelv_T *localB,
    DW_Motor_Dynamics_QuadModelv_T *localDW, X_Motor_Dynamics_QuadModelv_T
    *localX);

  // private member function(s) for subsystem '<S294>/MotorNonlinearDynamic1'
  void Qua_MotorNonlinearDynamic1_Init(DW_MotorNonlinearDynamic1_Qua_T *localDW,
    X_MotorNonlinearDynamic1_Quad_T *localX) const;
  static void Qu_MotorNonlinearDynamic1_Deriv(B_MotorNonlinearDynamic1_Quad_T
    *localB, XDot_MotorNonlinearDynamic1_Q_T *localXdot);
  static void Q_MotorNonlinearDynamic1_Update(DW_MotorNonlinearDynamic1_Qua_T
    *localDW);
  void QuadMode_MotorNonlinearDynamic1(real_T rtu_Wb, real_T rtu_Cr, real_T
    rtu_motorT, real_T rtu_ModelInit_RPM, real_T rtu_PWM,
    B_MotorNonlinearDynamic1_Quad_T *localB, DW_MotorNonlinearDynamic1_Qua_T
    *localDW, P_MotorNonlinearDynamic1_Quad_T *localP, P_QuadModelv_T
    *QuadModelv_P, X_MotorNonlinearDynamic1_Quad_T *localX);

  // private member function(s) for subsystem '<S512>/Distance into gust (y)'
  static void QuadMode_Distanceintogusty_Init(B_Distanceintogusty_QuadModel_T
    *localB, P_Distanceintogusty_QuadModel_T *localP,
    X_Distanceintogusty_QuadModel_T *localX);
  static void QuadMod_Distanceintogusty_Reset(P_Distanceintogusty_QuadModel_T
    *localP, X_Distanceintogusty_QuadModel_T *localX);
  void QuadMod_Distanceintogusty_Deriv(real_T rtu_V, real_T rtp_d_m,
    DW_Distanceintogusty_QuadMode_T *localDW, P_Distanceintogusty_QuadModel_T
    *localP, X_Distanceintogusty_QuadModel_T *localX,
    XDot_Distanceintogusty_QuadMo_T *localXdot);
  static void QuadM_Distanceintogusty_Disable(DW_Distanceintogusty_QuadMode_T
    *localDW);
  void QuadModelv_Distanceintogusty(boolean_T rtu_Enable, real_T rtp_d_m,
    B_Distanceintogusty_QuadModel_T *localB, DW_Distanceintogusty_QuadMode_T
    *localDW, P_Distanceintogusty_QuadModel_T *localP,
    X_Distanceintogusty_QuadModel_T *localX);

  // private member function(s) for subsystem '<Root>'
  real_T QuadModelv_eml_rand_mt19937ar(uint32_T state[625]);
  real_T QuadModelv_eml_rand_mcg16807(uint32_T *state);
  real_T QuadModelv_rand(void);
  void QuadModelv_rand_a(real_T r[3]);
  real_T QuadModelv_rand_g(void);
  real_T QuadModelv_rand_f(void);
  real_T QuadModelv_rand_g3(void);
  real_T QuadModelv_rand_e(void);
  real_T QuadModelv_rand_i(void);
  real_T QuadModelv_rand_ah(void);
  real_T QuadModelv_rand_l(void);
  real_T QuadModelv_rand_d(void);
  real_T QuadModelv_norm(const real_T x[3]);

  // Continuous states update member function
  void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si );

  // Derivatives member function
  void QuadModelv_derivatives();

  // Real-Time Model
  RT_MODEL_QuadModelv_T QuadModelv_M;
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S63>/Data Type Duplicate' : Unused code path elimination
//  Block '<S5>/Gain_-1' : Unused code path elimination
//  Block '<S64>/P0' : Unused code path elimination
//  Block '<S64>/Product2' : Unused code path elimination
//  Block '<S64>/a' : Unused code path elimination
//  Block '<S64>/gamma*R' : Unused code path elimination
//  Block '<S5>/Saturation_2' : Unused code path elimination
//  Block '<S94>/Cast' : Unused code path elimination
//  Block '<S95>/Cast' : Unused code path elimination
//  Block '<S371>/Data Type Duplicate' : Unused code path elimination
//  Block '<S394>/Scope' : Unused code path elimination
//  Block '<S408>/Product' : Unused code path elimination
//  Block '<S408>/Product3' : Unused code path elimination
//  Block '<S408>/a' : Unused code path elimination
//  Block '<S408>/gamma*R' : Unused code path elimination
//  Block '<S408>/rho0' : Unused code path elimination
//  Block '<S440>/Unit Conversion' : Unused code path elimination
//  Block '<S441>/Unit Conversion' : Unused code path elimination
//  Block '<S36>/Reshape (9) to [3x3] column-major' : Reshape block reduction
//  Block '<S49>/Reshape1' : Reshape block reduction
//  Block '<S49>/Reshape2' : Reshape block reduction
//  Block '<S50>/Reshape1' : Reshape block reduction
//  Block '<S50>/Reshape2' : Reshape block reduction
//  Block '<S16>/Reshape' : Reshape block reduction
//  Block '<S16>/Reshape1' : Reshape block reduction
//  Block '<S17>/Reshape' : Reshape block reduction
//  Block '<S19>/Unit Conversion' : Eliminated nontunable gain of 1
//  Block '<S20>/Unit Conversion' : Eliminated nontunable gain of 1
//  Block '<S21>/Unit Conversion' : Eliminated nontunable gain of 1
//  Block '<S22>/Reshape1' : Reshape block reduction
//  Block '<S22>/Reshape2' : Reshape block reduction
//  Block '<Root>/Data Type Conversion1' : Eliminate redundant data type conversion
//  Block '<S5>/Reshape' : Reshape block reduction
//  Block '<S90>/Unit Conversion' : Eliminated nontunable gain of 1
//  Block '<S92>/Unit Conversion' : Eliminated nontunable gain of 1
//  Block '<S94>/Cast To Double' : Eliminate redundant data type conversion
//  Block '<S94>/Cast To Double1' : Eliminate redundant data type conversion
//  Block '<S94>/Cast To Double2' : Eliminate redundant data type conversion
//  Block '<S94>/Cast To Double3' : Eliminate redundant data type conversion
//  Block '<S94>/Cast To Double4' : Eliminate redundant data type conversion
//  Block '<S131>/Reshape' : Reshape block reduction
//  Block '<S131>/Reshape1' : Reshape block reduction
//  Block '<S133>/Reshape' : Reshape block reduction
//  Block '<S139>/Reshape' : Reshape block reduction
//  Block '<S139>/Reshape1' : Reshape block reduction
//  Block '<S141>/Reshape' : Reshape block reduction
//  Block '<S95>/Cast To Double' : Eliminate redundant data type conversion
//  Block '<S95>/Cast To Double1' : Eliminate redundant data type conversion
//  Block '<S95>/Cast To Double2' : Eliminate redundant data type conversion
//  Block '<S95>/Cast To Double3' : Eliminate redundant data type conversion
//  Block '<S95>/Cast To Double4' : Eliminate redundant data type conversion
//  Block '<S170>/Reshape' : Reshape block reduction
//  Block '<S170>/Reshape1' : Reshape block reduction
//  Block '<S172>/Reshape' : Reshape block reduction
//  Block '<S178>/Reshape' : Reshape block reduction
//  Block '<S178>/Reshape1' : Reshape block reduction
//  Block '<S180>/Reshape' : Reshape block reduction
//  Block '<S105>/Cast To Double' : Eliminate redundant data type conversion
//  Block '<S105>/Reshape' : Reshape block reduction
//  Block '<S105>/Reshape1' : Reshape block reduction
//  Block '<S193>/Reshape 3x3 -> 9' : Reshape block reduction
//  Block '<S230>/Reshape' : Reshape block reduction
//  Block '<S194>/Reshape 3x3 -> 9' : Reshape block reduction
//  Block '<S268>/Reshape' : Reshape block reduction
//  Block '<S286>/Reshape (9) to [3x3] column-major' : Reshape block reduction
//  Block '<S8>/Data Type Conversion3' : Eliminate redundant data type conversion
//  Block '<S12>/Data Type Conversion11' : Eliminate redundant data type conversion
//  Block '<S12>/Data Type Conversion3' : Eliminate redundant data type conversion
//  Block '<S321>/Reshape 3x3 -> 9' : Reshape block reduction
//  Block '<S356>/Reshape' : Reshape block reduction
//  Block '<S415>/Reshape1' : Reshape block reduction
//  Block '<S437>/maxtype' : Eliminate redundant data type conversion
//  Block '<S437>/mintype' : Eliminate redundant data type conversion
//  Block '<S439>/Unit Conversion' : Eliminated nontunable gain of 1
//  Block '<S411>/Unit Conversion2' : Eliminated nontunable gain of 1
//  Block '<S467>/Reshape' : Reshape block reduction
//  Block '<S474>/Reshape' : Reshape block reduction
//  Block '<S475>/Reshape' : Reshape block reduction
//  Block '<S476>/Reshape' : Reshape block reduction
//  Block '<S476>/Reshape1' : Reshape block reduction
//  Block '<S512>/Cast' : Eliminate redundant data type conversion
//  Block '<S512>/Cast To Double' : Eliminate redundant data type conversion
//  Block '<S518>/Unit Conversion' : Eliminated nontunable gain of 1


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'QuadModelv'
//  '<S1>'   : 'QuadModelv/6DOF'
//  '<S2>'   : 'QuadModelv/Battery FaultModel'
//  '<S3>'   : 'QuadModelv/CollisionDetection'
//  '<S4>'   : 'QuadModelv/Crash_Motor_Fault'
//  '<S5>'   : 'QuadModelv/Environment FaultModel1'
//  '<S6>'   : 'QuadModelv/Fail Model'
//  '<S7>'   : 'QuadModelv/Force and Moment Model'
//  '<S8>'   : 'QuadModelv/LogSelectModel'
//  '<S9>'   : 'QuadModelv/MATLAB Function'
//  '<S10>'  : 'QuadModelv/Model Info'
//  '<S11>'  : 'QuadModelv/Motor FaultModel'
//  '<S12>'  : 'QuadModelv/OutputPort1'
//  '<S13>'  : 'QuadModelv/Prop FaultModel'
//  '<S14>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)'
//  '<S15>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles'
//  '<S16>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate omega_dot'
//  '<S17>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Determine Force,  Mass & Inertia'
//  '<S18>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Vbxw'
//  '<S19>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Velocity Conversion'
//  '<S20>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Velocity Conversion1'
//  '<S21>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Velocity Conversion2'
//  '<S22>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/transform to Inertial axes '
//  '<S23>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix'
//  '<S24>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles'
//  '<S25>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Rotation Angles to Quaternions'
//  '<S26>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/qdot'
//  '<S27>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A11'
//  '<S28>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A12'
//  '<S29>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A13'
//  '<S30>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A21'
//  '<S31>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A22'
//  '<S32>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A23'
//  '<S33>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A31'
//  '<S34>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A32'
//  '<S35>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A33'
//  '<S36>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/Create 3x3 Matrix'
//  '<S37>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/Quaternion Normalize'
//  '<S38>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus'
//  '<S39>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
//  '<S40>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Angle Calculation'
//  '<S41>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Quaternion Normalize'
//  '<S42>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input'
//  '<S43>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem'
//  '<S44>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem1'
//  '<S45>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem2'
//  '<S46>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus'
//  '<S47>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
//  '<S48>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate omega_dot/3x3 Cross Product'
//  '<S49>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate omega_dot/I x w'
//  '<S50>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate omega_dot/I x w1'
//  '<S51>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate omega_dot/3x3 Cross Product/Subsystem'
//  '<S52>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Calculate omega_dot/3x3 Cross Product/Subsystem1'
//  '<S53>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Determine Force,  Mass & Inertia/Mass input//output  momentum'
//  '<S54>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Determine Force,  Mass & Inertia/Mass input//output  momentum/For Each Subsystem'
//  '<S55>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Vbxw/Subsystem'
//  '<S56>'  : 'QuadModelv/6DOF/Custom Variable Mass 6DOF (Quaternion)/Vbxw/Subsystem1'
//  '<S57>'  : 'QuadModelv/Battery FaultModel/BatteryFault'
//  '<S58>'  : 'QuadModelv/Battery FaultModel/MATLAB Function'
//  '<S59>'  : 'QuadModelv/Battery FaultModel/BatteryFault/FaultParamsExtract'
//  '<S60>'  : 'QuadModelv/Battery FaultModel/BatteryFault/FaultParamsExtract1'
//  '<S61>'  : 'QuadModelv/Battery FaultModel/BatteryFault/FaultParamsExtract2'
//  '<S62>'  : 'QuadModelv/Battery FaultModel/BatteryFault/FaultParamsExtract3'
//  '<S63>'  : 'QuadModelv/Environment FaultModel1/Flat Earth to LLA'
//  '<S64>'  : 'QuadModelv/Environment FaultModel1/ISA Atmosphere Model'
//  '<S65>'  : 'QuadModelv/Environment FaultModel1/Temperature Conversion'
//  '<S66>'  : 'QuadModelv/Environment FaultModel1/WGS84 Gravity Model '
//  '<S67>'  : 'QuadModelv/Environment FaultModel1/WindFault'
//  '<S68>'  : 'QuadModelv/Environment FaultModel1/Flat Earth to LLA/LatLong wrap'
//  '<S69>'  : 'QuadModelv/Environment FaultModel1/Flat Earth to LLA/LatLong wrap1'
//  '<S70>'  : 'QuadModelv/Environment FaultModel1/Flat Earth to LLA/LongLat_offset'
//  '<S71>'  : 'QuadModelv/Environment FaultModel1/Flat Earth to LLA/pos_deg'
//  '<S72>'  : 'QuadModelv/Environment FaultModel1/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90'
//  '<S73>'  : 'QuadModelv/Environment FaultModel1/Flat Earth to LLA/LatLong wrap/Wrap Longitude'
//  '<S74>'  : 'QuadModelv/Environment FaultModel1/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Compare To Constant'
//  '<S75>'  : 'QuadModelv/Environment FaultModel1/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Wrap Angle 180'
//  '<S76>'  : 'QuadModelv/Environment FaultModel1/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Wrap Angle 180/Compare To Constant'
//  '<S77>'  : 'QuadModelv/Environment FaultModel1/Flat Earth to LLA/LatLong wrap/Wrap Longitude/Compare To Constant'
//  '<S78>'  : 'QuadModelv/Environment FaultModel1/Flat Earth to LLA/LatLong wrap1/Latitude Wrap 90'
//  '<S79>'  : 'QuadModelv/Environment FaultModel1/Flat Earth to LLA/LatLong wrap1/Wrap Longitude'
//  '<S80>'  : 'QuadModelv/Environment FaultModel1/Flat Earth to LLA/LatLong wrap1/Latitude Wrap 90/Compare To Constant'
//  '<S81>'  : 'QuadModelv/Environment FaultModel1/Flat Earth to LLA/LatLong wrap1/Latitude Wrap 90/Wrap Angle 180'
//  '<S82>'  : 'QuadModelv/Environment FaultModel1/Flat Earth to LLA/LatLong wrap1/Latitude Wrap 90/Wrap Angle 180/Compare To Constant'
//  '<S83>'  : 'QuadModelv/Environment FaultModel1/Flat Earth to LLA/LatLong wrap1/Wrap Longitude/Compare To Constant'
//  '<S84>'  : 'QuadModelv/Environment FaultModel1/Flat Earth to LLA/LongLat_offset/Find Radian//Distance'
//  '<S85>'  : 'QuadModelv/Environment FaultModel1/Flat Earth to LLA/LongLat_offset/rotation_rad'
//  '<S86>'  : 'QuadModelv/Environment FaultModel1/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/Angle Conversion2'
//  '<S87>'  : 'QuadModelv/Environment FaultModel1/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/denom'
//  '<S88>'  : 'QuadModelv/Environment FaultModel1/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/e'
//  '<S89>'  : 'QuadModelv/Environment FaultModel1/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/e^4'
//  '<S90>'  : 'QuadModelv/Environment FaultModel1/WGS84 Gravity Model /Acceleration Conversion'
//  '<S91>'  : 'QuadModelv/Environment FaultModel1/WGS84 Gravity Model /Angle Conversion'
//  '<S92>'  : 'QuadModelv/Environment FaultModel1/WGS84 Gravity Model /Length Conversion'
//  '<S93>'  : 'QuadModelv/Environment FaultModel1/WGS84 Gravity Model /Velocity Conversion2'
//  '<S94>'  : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))'
//  '<S95>'  : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1'
//  '<S96>'  : 'QuadModelv/Environment FaultModel1/WindFault/FaultParamsExtract'
//  '<S97>'  : 'QuadModelv/Environment FaultModel1/WindFault/FaultParamsExtract1'
//  '<S98>'  : 'QuadModelv/Environment FaultModel1/WindFault/FaultParamsExtract2'
//  '<S99>'  : 'QuadModelv/Environment FaultModel1/WindFault/FaultParamsExtract3'
//  '<S100>' : 'QuadModelv/Environment FaultModel1/WindFault/FaultParamsExtract4'
//  '<S101>' : 'QuadModelv/Environment FaultModel1/WindFault/MATLAB Function'
//  '<S102>' : 'QuadModelv/Environment FaultModel1/WindFault/MATLAB Function1'
//  '<S103>' : 'QuadModelv/Environment FaultModel1/WindFault/SheerWindStrength_Dec_Switch'
//  '<S104>' : 'QuadModelv/Environment FaultModel1/WindFault/TurbWindStrength_Dec_Switch'
//  '<S105>' : 'QuadModelv/Environment FaultModel1/WindFault/Wind Shear Model'
//  '<S106>' : 'QuadModelv/Environment FaultModel1/WindFault/noiseUpperWindBodySwitch'
//  '<S107>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Angle Conversion'
//  '<S108>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Filters on angular rates'
//  '<S109>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Filters on velocities'
//  '<S110>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Length Conversion'
//  '<S111>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Length Conversion1'
//  '<S112>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/RMS turbulence  intensities'
//  '<S113>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Select angular rates'
//  '<S114>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Select velocities'
//  '<S115>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Turbulence scale lengths'
//  '<S116>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Velocity Conversion'
//  '<S117>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Velocity Conversion2'
//  '<S118>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/White Noise'
//  '<S119>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Filters on angular rates/Hpgw'
//  '<S120>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Filters on angular rates/Hqgw'
//  '<S121>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Filters on angular rates/Hrgw'
//  '<S122>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Filters on velocities/Hugw(z)'
//  '<S123>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Filters on velocities/Hvgw(z)'
//  '<S124>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Filters on velocities/Hwgw(z)'
//  '<S125>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/RMS turbulence  intensities/High Altitude Intensity'
//  '<S126>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/RMS turbulence  intensities/Low Altitude Intensity'
//  '<S127>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Select angular rates/Interpolate  rates'
//  '<S128>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Select angular rates/Low altitude  rates'
//  '<S129>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Select angular rates/Medium//High  altitude rates'
//  '<S130>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Select angular rates/Merge Subsystems'
//  '<S131>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Select angular rates/Interpolate  rates/wind to body transformation'
//  '<S132>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Select angular rates/Interpolate  rates/wind to body transformation/convert to earth coords'
//  '<S133>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Select angular rates/Low altitude  rates/wind to body transformation'
//  '<S134>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Select angular rates/Low altitude  rates/wind to body transformation/convert to earth coords'
//  '<S135>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Select velocities/Interpolate  velocities'
//  '<S136>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Select velocities/Low altitude  velocities'
//  '<S137>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Select velocities/Medium//High  altitude velocities'
//  '<S138>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Select velocities/Merge Subsystems'
//  '<S139>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Select velocities/Interpolate  velocities/wind to body transformation'
//  '<S140>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Select velocities/Interpolate  velocities/wind to body transformation/convert to earth coords'
//  '<S141>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Select velocities/Low altitude  velocities/wind to body transformation'
//  '<S142>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Select velocities/Low altitude  velocities/wind to body transformation/convert to earth coords'
//  '<S143>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Turbulence scale lengths/Low altitude scale length'
//  '<S144>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Turbulence scale lengths/Medium//High altitude scale length'
//  '<S145>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))/Turbulence scale lengths/Medium//High altitude scale length/Length Conversion'
//  '<S146>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Angle Conversion'
//  '<S147>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Filters on angular rates'
//  '<S148>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Filters on velocities'
//  '<S149>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Length Conversion'
//  '<S150>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Length Conversion1'
//  '<S151>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/RMS turbulence  intensities'
//  '<S152>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Select angular rates'
//  '<S153>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Select velocities'
//  '<S154>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Turbulence scale lengths'
//  '<S155>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Velocity Conversion'
//  '<S156>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Velocity Conversion2'
//  '<S157>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/White Noise'
//  '<S158>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Filters on angular rates/Hpgw'
//  '<S159>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Filters on angular rates/Hqgw'
//  '<S160>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Filters on angular rates/Hrgw'
//  '<S161>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Filters on velocities/Hugw(z)'
//  '<S162>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Filters on velocities/Hvgw(z)'
//  '<S163>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Filters on velocities/Hwgw(z)'
//  '<S164>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/RMS turbulence  intensities/High Altitude Intensity'
//  '<S165>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/RMS turbulence  intensities/Low Altitude Intensity'
//  '<S166>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Select angular rates/Interpolate  rates'
//  '<S167>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Select angular rates/Low altitude  rates'
//  '<S168>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Select angular rates/Medium//High  altitude rates'
//  '<S169>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Select angular rates/Merge Subsystems'
//  '<S170>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Select angular rates/Interpolate  rates/wind to body transformation'
//  '<S171>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Select angular rates/Interpolate  rates/wind to body transformation/convert to earth coords'
//  '<S172>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Select angular rates/Low altitude  rates/wind to body transformation'
//  '<S173>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Select angular rates/Low altitude  rates/wind to body transformation/convert to earth coords'
//  '<S174>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Select velocities/Interpolate  velocities'
//  '<S175>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Select velocities/Low altitude  velocities'
//  '<S176>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Select velocities/Medium//High  altitude velocities'
//  '<S177>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Select velocities/Merge Subsystems'
//  '<S178>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Select velocities/Interpolate  velocities/wind to body transformation'
//  '<S179>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Select velocities/Interpolate  velocities/wind to body transformation/convert to earth coords'
//  '<S180>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Select velocities/Low altitude  velocities/wind to body transformation'
//  '<S181>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Select velocities/Low altitude  velocities/wind to body transformation/convert to earth coords'
//  '<S182>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Turbulence scale lengths/Low altitude scale length'
//  '<S183>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Turbulence scale lengths/Medium//High altitude scale length'
//  '<S184>' : 'QuadModelv/Environment FaultModel1/WindFault/Dryden Wind Turbulence Model  (Discrete (-q +r))1/Turbulence scale lengths/Medium//High altitude scale length/Length Conversion'
//  '<S185>' : 'QuadModelv/Environment FaultModel1/WindFault/Wind Shear Model/Angle Conversion'
//  '<S186>' : 'QuadModelv/Environment FaultModel1/WindFault/Wind Shear Model/Length Conversion'
//  '<S187>' : 'QuadModelv/Fail Model/LoadFault'
//  '<S188>' : 'QuadModelv/Fail Model/Model Failure'
//  '<S189>' : 'QuadModelv/Fail Model/LoadFault/FaultParamsExtract1'
//  '<S190>' : 'QuadModelv/Fail Model/LoadFault/FaultParamsExtract2'
//  '<S191>' : 'QuadModelv/Fail Model/LoadFault/FaultParamsExtract3'
//  '<S192>' : 'QuadModelv/Force and Moment Model/Body AeroCenter'
//  '<S193>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions'
//  '<S194>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1'
//  '<S195>' : 'QuadModelv/Force and Moment Model/Euler to DCM'
//  '<S196>' : 'QuadModelv/Force and Moment Model/Ground Model'
//  '<S197>' : 'QuadModelv/Force and Moment Model/OnGroundFaceup'
//  '<S198>' : 'QuadModelv/Force and Moment Model/Propeller Model old Version'
//  '<S199>' : 'QuadModelv/Force and Moment Model/Quaternion Inverse'
//  '<S200>' : 'QuadModelv/Force and Moment Model/Quaternion Multiplication'
//  '<S201>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Negative Trace'
//  '<S202>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Positive Trace'
//  '<S203>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Validate DCM'
//  '<S204>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/trace(DCM)'
//  '<S205>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)'
//  '<S206>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)'
//  '<S207>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)'
//  '<S208>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Negative Trace/diag(DCM)'
//  '<S209>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) -sin(theta)'
//  '<S210>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/cos(theta)sin(phi) - (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
//  '<S211>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/cos(theta)sin(psi) + (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
//  '<S212>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/if s~=0; s=0.5//s'
//  '<S213>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/u(1) -(u(5)+u(9)) +1'
//  '<S214>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) +sin(theta)'
//  '<S215>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/cos(theta)sin(phi) + (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
//  '<S216>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/cos(theta)sin(psi) + (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
//  '<S217>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/if s~=0; s=0.5//s'
//  '<S218>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/u(5) -(u(1)+u(9)) +1'
//  '<S219>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) -sin(theta)'
//  '<S220>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/cos(theta)sin(phi) + (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
//  '<S221>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/cos(theta)sin(psi) - (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
//  '<S222>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/if s~=0; s=0.5//s'
//  '<S223>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/u(9) -(u(1)+u(5)) +1'
//  '<S224>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Positive Trace/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) +sin(theta)'
//  '<S225>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Positive Trace/cos(theta)sin(phi) - (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
//  '<S226>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Positive Trace/cos(theta)sin(psi) - (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
//  '<S227>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error'
//  '<S228>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/Else If Not Orthogonal'
//  '<S229>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/If Not Proper'
//  '<S230>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotOrthogonal'
//  '<S231>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotProper'
//  '<S232>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/Else If Not Orthogonal/Error'
//  '<S233>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/Else If Not Orthogonal/Warning'
//  '<S234>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/If Not Proper/Error'
//  '<S235>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/If Not Proper/Warning'
//  '<S236>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotOrthogonal/transpose*dcm ~= eye(3)'
//  '<S237>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotProper/Determinant of 3x3 Matrix'
//  '<S238>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotProper/determinant does not equal 1'
//  '<S239>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Negative Trace'
//  '<S240>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Positive Trace'
//  '<S241>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Validate DCM'
//  '<S242>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/trace(DCM)'
//  '<S243>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(1,1)'
//  '<S244>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(2,2)'
//  '<S245>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(3,3)'
//  '<S246>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/diag(DCM)'
//  '<S247>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(1,1)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) -sin(theta)'
//  '<S248>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(1,1)/cos(theta)sin(phi) - (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
//  '<S249>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(1,1)/cos(theta)sin(psi) + (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
//  '<S250>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(1,1)/if s~=0; s=0.5//s'
//  '<S251>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(1,1)/u(1) -(u(5)+u(9)) +1'
//  '<S252>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(2,2)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) +sin(theta)'
//  '<S253>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(2,2)/cos(theta)sin(phi) + (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
//  '<S254>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(2,2)/cos(theta)sin(psi) + (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
//  '<S255>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(2,2)/if s~=0; s=0.5//s'
//  '<S256>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(2,2)/u(5) -(u(1)+u(9)) +1'
//  '<S257>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(3,3)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) -sin(theta)'
//  '<S258>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(3,3)/cos(theta)sin(phi) + (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
//  '<S259>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(3,3)/cos(theta)sin(psi) - (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
//  '<S260>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(3,3)/if s~=0; s=0.5//s'
//  '<S261>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(3,3)/u(9) -(u(1)+u(5)) +1'
//  '<S262>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Positive Trace/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) +sin(theta)'
//  '<S263>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Positive Trace/cos(theta)sin(phi) - (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
//  '<S264>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Positive Trace/cos(theta)sin(psi) - (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
//  '<S265>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Validate DCM/If Warning//Error'
//  '<S266>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Validate DCM/If Warning//Error/Else If Not Orthogonal'
//  '<S267>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Validate DCM/If Warning//Error/If Not Proper'
//  '<S268>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Validate DCM/If Warning//Error/isNotOrthogonal'
//  '<S269>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Validate DCM/If Warning//Error/isNotProper'
//  '<S270>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Validate DCM/If Warning//Error/Else If Not Orthogonal/Error'
//  '<S271>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Validate DCM/If Warning//Error/Else If Not Orthogonal/Warning'
//  '<S272>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Validate DCM/If Warning//Error/If Not Proper/Error'
//  '<S273>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Validate DCM/If Warning//Error/If Not Proper/Warning'
//  '<S274>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Validate DCM/If Warning//Error/isNotOrthogonal/transpose*dcm ~= eye(3)'
//  '<S275>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Validate DCM/If Warning//Error/isNotProper/Determinant of 3x3 Matrix'
//  '<S276>' : 'QuadModelv/Force and Moment Model/Direction Cosine Matrix  to Quaternions1/Validate DCM/If Warning//Error/isNotProper/determinant does not equal 1'
//  '<S277>' : 'QuadModelv/Force and Moment Model/Euler to DCM/A11'
//  '<S278>' : 'QuadModelv/Force and Moment Model/Euler to DCM/A12'
//  '<S279>' : 'QuadModelv/Force and Moment Model/Euler to DCM/A13'
//  '<S280>' : 'QuadModelv/Force and Moment Model/Euler to DCM/A21'
//  '<S281>' : 'QuadModelv/Force and Moment Model/Euler to DCM/A22'
//  '<S282>' : 'QuadModelv/Force and Moment Model/Euler to DCM/A23'
//  '<S283>' : 'QuadModelv/Force and Moment Model/Euler to DCM/A31'
//  '<S284>' : 'QuadModelv/Force and Moment Model/Euler to DCM/A32'
//  '<S285>' : 'QuadModelv/Force and Moment Model/Euler to DCM/A33'
//  '<S286>' : 'QuadModelv/Force and Moment Model/Euler to DCM/Create Transformation Matrix'
//  '<S287>' : 'QuadModelv/Force and Moment Model/Quaternion Inverse/Quaternion Conjugate'
//  '<S288>' : 'QuadModelv/Force and Moment Model/Quaternion Inverse/Quaternion Norm'
//  '<S289>' : 'QuadModelv/Force and Moment Model/Quaternion Multiplication/q0'
//  '<S290>' : 'QuadModelv/Force and Moment Model/Quaternion Multiplication/q1'
//  '<S291>' : 'QuadModelv/Force and Moment Model/Quaternion Multiplication/q2'
//  '<S292>' : 'QuadModelv/Force and Moment Model/Quaternion Multiplication/q3'
//  '<S293>' : 'QuadModelv/Motor FaultModel/MotorFault'
//  '<S294>' : 'QuadModelv/Motor FaultModel/Motor_Fun'
//  '<S295>' : 'QuadModelv/Motor FaultModel/MotorFault/FaultParamsExtract'
//  '<S296>' : 'QuadModelv/Motor FaultModel/MotorFault/MotorFaultModel'
//  '<S297>' : 'QuadModelv/Motor FaultModel/Motor_Fun/MotorNonlinearDynamic1'
//  '<S298>' : 'QuadModelv/Motor FaultModel/Motor_Fun/MotorNonlinearDynamic2'
//  '<S299>' : 'QuadModelv/Motor FaultModel/Motor_Fun/MotorNonlinearDynamic3'
//  '<S300>' : 'QuadModelv/Motor FaultModel/Motor_Fun/MotorNonlinearDynamic4'
//  '<S301>' : 'QuadModelv/Motor FaultModel/Motor_Fun/MotorNonlinearDynamic5'
//  '<S302>' : 'QuadModelv/Motor FaultModel/Motor_Fun/MotorNonlinearDynamic6'
//  '<S303>' : 'QuadModelv/Motor FaultModel/Motor_Fun/MotorNonlinearDynamic7'
//  '<S304>' : 'QuadModelv/Motor FaultModel/Motor_Fun/MotorNonlinearDynamic8'
//  '<S305>' : 'QuadModelv/Motor FaultModel/Motor_Fun/MotorNonlinearDynamic1/Minimum Throttle  Cutoff'
//  '<S306>' : 'QuadModelv/Motor FaultModel/Motor_Fun/MotorNonlinearDynamic1/Motor_Dynamics'
//  '<S307>' : 'QuadModelv/Motor FaultModel/Motor_Fun/MotorNonlinearDynamic2/Minimum Throttle  Cutoff'
//  '<S308>' : 'QuadModelv/Motor FaultModel/Motor_Fun/MotorNonlinearDynamic2/Motor_Dynamics'
//  '<S309>' : 'QuadModelv/Motor FaultModel/Motor_Fun/MotorNonlinearDynamic3/Minimum Throttle  Cutoff'
//  '<S310>' : 'QuadModelv/Motor FaultModel/Motor_Fun/MotorNonlinearDynamic3/Motor_Dynamics'
//  '<S311>' : 'QuadModelv/Motor FaultModel/Motor_Fun/MotorNonlinearDynamic4/Minimum Throttle  Cutoff'
//  '<S312>' : 'QuadModelv/Motor FaultModel/Motor_Fun/MotorNonlinearDynamic4/Motor_Dynamics'
//  '<S313>' : 'QuadModelv/Motor FaultModel/Motor_Fun/MotorNonlinearDynamic5/Minimum Throttle  Cutoff'
//  '<S314>' : 'QuadModelv/Motor FaultModel/Motor_Fun/MotorNonlinearDynamic5/Motor_Dynamics'
//  '<S315>' : 'QuadModelv/Motor FaultModel/Motor_Fun/MotorNonlinearDynamic6/Minimum Throttle  Cutoff'
//  '<S316>' : 'QuadModelv/Motor FaultModel/Motor_Fun/MotorNonlinearDynamic6/Motor_Dynamics'
//  '<S317>' : 'QuadModelv/Motor FaultModel/Motor_Fun/MotorNonlinearDynamic7/Minimum Throttle  Cutoff'
//  '<S318>' : 'QuadModelv/Motor FaultModel/Motor_Fun/MotorNonlinearDynamic7/Motor_Dynamics'
//  '<S319>' : 'QuadModelv/Motor FaultModel/Motor_Fun/MotorNonlinearDynamic8/Minimum Throttle  Cutoff'
//  '<S320>' : 'QuadModelv/Motor FaultModel/Motor_Fun/MotorNonlinearDynamic8/Motor_Dynamics'
//  '<S321>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions'
//  '<S322>' : 'QuadModelv/OutputPort1/HILGPSModel'
//  '<S323>' : 'QuadModelv/OutputPort1/HILSensorMavModel1'
//  '<S324>' : 'QuadModelv/OutputPort1/HILStateMavModel'
//  '<S325>' : 'QuadModelv/OutputPort1/Model Fail Assessment'
//  '<S326>' : 'QuadModelv/OutputPort1/ZLimit'
//  '<S327>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Negative Trace'
//  '<S328>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Positive Trace'
//  '<S329>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Validate DCM'
//  '<S330>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/trace(DCM)'
//  '<S331>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)'
//  '<S332>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)'
//  '<S333>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)'
//  '<S334>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Negative Trace/diag(DCM)'
//  '<S335>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) -sin(theta)'
//  '<S336>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/cos(theta)sin(phi) - (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
//  '<S337>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/cos(theta)sin(psi) + (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
//  '<S338>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/if s~=0; s=0.5//s'
//  '<S339>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/u(1) -(u(5)+u(9)) +1'
//  '<S340>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) +sin(theta)'
//  '<S341>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/cos(theta)sin(phi) + (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
//  '<S342>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/cos(theta)sin(psi) + (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
//  '<S343>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/if s~=0; s=0.5//s'
//  '<S344>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/u(5) -(u(1)+u(9)) +1'
//  '<S345>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) -sin(theta)'
//  '<S346>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/cos(theta)sin(phi) + (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
//  '<S347>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/cos(theta)sin(psi) - (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
//  '<S348>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/if s~=0; s=0.5//s'
//  '<S349>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/u(9) -(u(1)+u(5)) +1'
//  '<S350>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Positive Trace/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) +sin(theta)'
//  '<S351>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Positive Trace/cos(theta)sin(phi) - (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
//  '<S352>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Positive Trace/cos(theta)sin(psi) - (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
//  '<S353>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error'
//  '<S354>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/Else If Not Orthogonal'
//  '<S355>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/If Not Proper'
//  '<S356>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotOrthogonal'
//  '<S357>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotProper'
//  '<S358>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/Else If Not Orthogonal/Error'
//  '<S359>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/Else If Not Orthogonal/Warning'
//  '<S360>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/If Not Proper/Error'
//  '<S361>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/If Not Proper/Warning'
//  '<S362>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotOrthogonal/transpose*dcm ~= eye(3)'
//  '<S363>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotProper/Determinant of 3x3 Matrix'
//  '<S364>' : 'QuadModelv/OutputPort1/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotProper/determinant does not equal 1'
//  '<S365>' : 'QuadModelv/OutputPort1/HILGPSModel/-pi-pi---->0-2pi'
//  '<S366>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault'
//  '<S367>' : 'QuadModelv/OutputPort1/HILGPSModel/GenCogVel'
//  '<S368>' : 'QuadModelv/OutputPort1/HILGPSModel/NoiseFilter1'
//  '<S369>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault/Acc NoiseFun'
//  '<S370>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault/FaultParamsExtract'
//  '<S371>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault/Flat Earth to LLA'
//  '<S372>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault/Flat Earth to LLA/LatLong wrap'
//  '<S373>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault/Flat Earth to LLA/LatLong wrap1'
//  '<S374>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault/Flat Earth to LLA/LongLat_offset'
//  '<S375>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault/Flat Earth to LLA/pos_deg'
//  '<S376>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90'
//  '<S377>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault/Flat Earth to LLA/LatLong wrap/Wrap Longitude'
//  '<S378>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Compare To Constant'
//  '<S379>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Wrap Angle 180'
//  '<S380>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Wrap Angle 180/Compare To Constant'
//  '<S381>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault/Flat Earth to LLA/LatLong wrap/Wrap Longitude/Compare To Constant'
//  '<S382>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault/Flat Earth to LLA/LatLong wrap1/Latitude Wrap 90'
//  '<S383>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault/Flat Earth to LLA/LatLong wrap1/Wrap Longitude'
//  '<S384>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault/Flat Earth to LLA/LatLong wrap1/Latitude Wrap 90/Compare To Constant'
//  '<S385>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault/Flat Earth to LLA/LatLong wrap1/Latitude Wrap 90/Wrap Angle 180'
//  '<S386>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault/Flat Earth to LLA/LatLong wrap1/Latitude Wrap 90/Wrap Angle 180/Compare To Constant'
//  '<S387>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault/Flat Earth to LLA/LatLong wrap1/Wrap Longitude/Compare To Constant'
//  '<S388>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault/Flat Earth to LLA/LongLat_offset/Find Radian//Distance'
//  '<S389>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault/Flat Earth to LLA/LongLat_offset/rotation_rad'
//  '<S390>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/Angle Conversion2'
//  '<S391>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/denom'
//  '<S392>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/e'
//  '<S393>' : 'QuadModelv/OutputPort1/HILGPSModel/GPSFault/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/e^4'
//  '<S394>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem'
//  '<S395>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/AccelNoiseGainFunction'
//  '<S396>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/GyroNoiseGainFunction'
//  '<S397>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/MagNoiseGainFunction'
//  '<S398>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault'
//  '<S399>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/Acc NoiseFun'
//  '<S400>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/Dynamic Pressure'
//  '<S401>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/FaultParamsExtract'
//  '<S402>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/FaultParamsExtract1'
//  '<S403>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/FaultParamsExtract2'
//  '<S404>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/FaultParamsExtract3'
//  '<S405>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/Gyro NoiseFun'
//  '<S406>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/GyroNoiseSwitch3'
//  '<S407>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/GyroNoiseSwitch4'
//  '<S408>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/ISA Atmosphere Model'
//  '<S409>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/Mag NoiseFun'
//  '<S410>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/Three-axis Inertial Measurement Unit3'
//  '<S411>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015'
//  '<S412>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/baro NoiseFun'
//  '<S413>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/Dynamic Pressure/dot'
//  '<S414>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/Three-axis Inertial Measurement Unit3/Acceleration Conversion'
//  '<S415>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/Three-axis Inertial Measurement Unit3/Three-axis Accelerometer'
//  '<S416>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/Three-axis Inertial Measurement Unit3/Three-axis Gyroscope'
//  '<S417>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/Three-axis Inertial Measurement Unit3/Three-axis Accelerometer/Dynamics'
//  '<S418>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/Three-axis Inertial Measurement Unit3/Three-axis Accelerometer/w x (w x d)'
//  '<S419>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/Three-axis Inertial Measurement Unit3/Three-axis Accelerometer/wdot x d'
//  '<S420>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/Three-axis Inertial Measurement Unit3/Three-axis Accelerometer/Dynamics/No Dynamics'
//  '<S421>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/Three-axis Inertial Measurement Unit3/Three-axis Accelerometer/Dynamics/Second-order Dynamics'
//  '<S422>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/Three-axis Inertial Measurement Unit3/Three-axis Accelerometer/w x (w x d)/w x (w x d)'
//  '<S423>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/Three-axis Inertial Measurement Unit3/Three-axis Accelerometer/w x (w x d)/w x d'
//  '<S424>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/Three-axis Inertial Measurement Unit3/Three-axis Accelerometer/w x (w x d)/w x (w x d)/Subsystem'
//  '<S425>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/Three-axis Inertial Measurement Unit3/Three-axis Accelerometer/w x (w x d)/w x (w x d)/Subsystem1'
//  '<S426>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/Three-axis Inertial Measurement Unit3/Three-axis Accelerometer/w x (w x d)/w x d/Subsystem'
//  '<S427>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/Three-axis Inertial Measurement Unit3/Three-axis Accelerometer/w x (w x d)/w x d/Subsystem1'
//  '<S428>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/Three-axis Inertial Measurement Unit3/Three-axis Accelerometer/wdot x d/Subsystem'
//  '<S429>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/Three-axis Inertial Measurement Unit3/Three-axis Accelerometer/wdot x d/Subsystem1'
//  '<S430>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/Three-axis Inertial Measurement Unit3/Three-axis Gyroscope/Dynamics'
//  '<S431>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/Three-axis Inertial Measurement Unit3/Three-axis Gyroscope/Dynamics/No Dynamics'
//  '<S432>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/Three-axis Inertial Measurement Unit3/Three-axis Gyroscope/Dynamics/Second-order Dynamics'
//  '<S433>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/Check Altitude'
//  '<S434>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/Check Latitude'
//  '<S435>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/Check Longitude'
//  '<S436>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/Compute x,y,z, and h components of magnetic field'
//  '<S437>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/Is time within model limits'
//  '<S438>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/LatLong wrap'
//  '<S439>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/Length Conversion'
//  '<S440>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/MagField Conversion'
//  '<S441>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/MagField Conversion1'
//  '<S442>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag'
//  '<S443>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/Compute x,y,z, and h components of magnetic field/Angle Conversion'
//  '<S444>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/LatLong wrap/Latitude Wrap 90'
//  '<S445>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/LatLong wrap/Wrap Longitude'
//  '<S446>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/LatLong wrap/Latitude Wrap 90/Compare To Constant'
//  '<S447>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/LatLong wrap/Latitude Wrap 90/Wrap Angle 180'
//  '<S448>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/LatLong wrap/Latitude Wrap 90/Wrap Angle 180/Compare To Constant'
//  '<S449>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/LatLong wrap/Wrap Longitude/Compare To Constant'
//  '<S450>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates'
//  '<S451>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Convert from geodetic to  spherical coordinates'
//  '<S452>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Convert from geodetic to  spherical coordinates '
//  '<S453>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Get Cosine and Sine  of Latitude and Longitude'
//  '<S454>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Has altitude or latitude changed'
//  '<S455>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Has longitude changed '
//  '<S456>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Has time changed'
//  '<S457>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Rotate magnetic vector components  to geodetic from spherical and  compute declination, inclination  and total intensity'
//  '<S458>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem'
//  '<S459>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion'
//  '<S460>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations'
//  '<S461>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Time adjust the gauss coefficients'
//  '<S462>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion/Special case - North//South Geographic Pole'
//  '<S463>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion/calculate  index'
//  '<S464>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion/calculate temp values'
//  '<S465>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion/special case'
//  '<S466>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion/Special case - North//South Geographic Pole/If Action Subsystem1'
//  '<S467>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion/Special case - North//South Geographic Pole/If Action Subsystem2'
//  '<S468>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion/Special case - North//South Geographic Pole/If Action Subsystem2/calculate  indices'
//  '<S469>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion/Special case - North//South Geographic Pole/If Action Subsystem2/calculate  row and column'
//  '<S470>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion/calculate temp values/If Action Subsystem'
//  '<S471>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion/calculate temp values/If Action Subsystem1'
//  '<S472>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion/calculate temp values/If Action Subsystem1/m,n'
//  '<S473>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion/calculate temp values/If Action Subsystem1/n,m-1'
//  '<S474>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/If Action Subsystem'
//  '<S475>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/If Action Subsystem1'
//  '<S476>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/If Action Subsystem2'
//  '<S477>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/calculate  index'
//  '<S478>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/If Action Subsystem/calculate  index'
//  '<S479>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/If Action Subsystem/calculate  row and column'
//  '<S480>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/If Action Subsystem1/calculate  index'
//  '<S481>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/If Action Subsystem1/calculate  row and column'
//  '<S482>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/If Action Subsystem2/calculate  indices'
//  '<S483>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/If Action Subsystem2/calculate  row and column1'
//  '<S484>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/If Action Subsystem2/calculate  row and column2'
//  '<S485>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/If Action Subsystem2/m<n-2'
//  '<S486>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/If Action Subsystem2/m<n-2 '
//  '<S487>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Time adjust the gauss coefficients/If Action Subsystem'
//  '<S488>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Time adjust the gauss coefficients/if (m~=0)'
//  '<S489>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Time adjust the gauss coefficients/if (m~=0)/If Action Subsystem1'
//  '<S490>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Time adjust the gauss coefficients/if (m~=0)/If Action Subsystem2'
//  '<S491>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Convert from geodetic to  spherical coordinates/calculate ca'
//  '<S492>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Convert from geodetic to  spherical coordinates/calculate ct'
//  '<S493>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Convert from geodetic to  spherical coordinates/calculate d'
//  '<S494>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Convert from geodetic to  spherical coordinates/calculate q'
//  '<S495>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Convert from geodetic to  spherical coordinates/calculate q2'
//  '<S496>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Convert from geodetic to  spherical coordinates/calculate r2'
//  '<S497>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Convert from geodetic to  spherical coordinates/calculate sa'
//  '<S498>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Convert from geodetic to  spherical coordinates/calculate st'
//  '<S499>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Convert from geodetic to  spherical coordinates /For Iterator Subsystem'
//  '<S500>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Get Cosine and Sine  of Latitude and Longitude/Angle Conversion2'
//  '<S501>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Rotate magnetic vector components  to geodetic from spherical and  compute declination, inclination  and total intensity/Calculate bx'
//  '<S502>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Rotate magnetic vector components  to geodetic from spherical and  compute declination, inclination  and total intensity/Calculate by'
//  '<S503>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Rotate magnetic vector components  to geodetic from spherical and  compute declination, inclination  and total intensity/Calculate bz'
//  '<S504>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Rotate magnetic vector components  to geodetic from spherical and  compute declination, inclination  and total intensity/Compute declination, inclination,  and total intensity'
//  '<S505>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Rotate magnetic vector components  to geodetic from spherical and  compute declination, inclination  and total intensity/Compute declination, inclination,  and total intensity/Angle Conversion'
//  '<S506>' : 'QuadModelv/OutputPort1/HILSensorMavModel1/Subsystem/SensorFault/World Magnetic Model 2015/geomag/Rotate magnetic vector components  to geodetic from spherical and  compute declination, inclination  and total intensity/Compute declination, inclination,  and total intensity/Angle Conversion1'
//  '<S507>' : 'QuadModelv/OutputPort1/HILStateMavModel/Dynamic Pressure'
//  '<S508>' : 'QuadModelv/OutputPort1/HILStateMavModel/dot1'
//  '<S509>' : 'QuadModelv/OutputPort1/HILStateMavModel/Dynamic Pressure/dot'
//  '<S510>' : 'QuadModelv/Prop FaultModel/PropFault'
//  '<S511>' : 'QuadModelv/Prop FaultModel/PropFault/Band-Limited White Noise'
//  '<S512>' : 'QuadModelv/Prop FaultModel/PropFault/Discrete Wind Gust Model'
//  '<S513>' : 'QuadModelv/Prop FaultModel/PropFault/FaultParamsExtract'
//  '<S514>' : 'QuadModelv/Prop FaultModel/PropFault/PropFaultModel'
//  '<S515>' : 'QuadModelv/Prop FaultModel/PropFault/Discrete Wind Gust Model/Distance into gust (x)'
//  '<S516>' : 'QuadModelv/Prop FaultModel/PropFault/Discrete Wind Gust Model/Distance into gust (y)'
//  '<S517>' : 'QuadModelv/Prop FaultModel/PropFault/Discrete Wind Gust Model/Distance into gust (z)'
//  '<S518>' : 'QuadModelv/Prop FaultModel/PropFault/Discrete Wind Gust Model/Velocity Conversion'

#endif                                 // RTW_HEADER_QuadModelv_h_

//
// File trailer for generated code.
//
// [EOF]
//
