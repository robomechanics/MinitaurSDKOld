/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.8 at Tue Jun 19 13:15:42 2018. */

#ifndef PB_SIMPLE_PB_H_INCLUDED
#define PB_SIMPLE_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Enum definitions */
typedef enum _JoyType {
    JoyType_PWM_QUANUM = 0,
    JoyType_SERIAL_APP = 1,
    JoyType_PWM_FRSKY_X7 = 2,
    JoyType_NONE = 3,
    JoyType_FRSKY_XSR = 4
} JoyType;
#define _JoyType_MIN JoyType_PWM_QUANUM
#define _JoyType_MAX JoyType_FRSKY_XSR
#define _JoyType_ARRAYSIZE ((JoyType)(JoyType_FRSKY_XSR+1))

typedef enum _PlatformType {
    PlatformType_MCU = 0,
    PlatformType_OBC = 1
} PlatformType;
#define _PlatformType_MIN PlatformType_MCU
#define _PlatformType_MAX PlatformType_OBC
#define _PlatformType_ARRAYSIZE ((PlatformType)(PlatformType_OBC+1))

typedef enum _MotorType {
    MotorType_U8_DD = 0,
    MotorType_U10P_G7 = 1,
    MotorType_U8II_G25 = 2,
    MotorType_DXL_MX106 = 3,
    MotorType_DCX32_GPX = 4,
    MotorType_U8II_G30 = 5
} MotorType;
#define _MotorType_MIN MotorType_U8_DD
#define _MotorType_MAX MotorType_U8II_G30
#define _MotorType_ARRAYSIZE ((MotorType)(MotorType_U8II_G30+1))

typedef enum _JointMode {
    JointMode_OFF = 0,
    JointMode_PWM = 1,
    JointMode_POSITION = 2,
    JointMode_TORQUE = 4,
    JointMode_CURRENT = 5
} JointMode;
#define _JointMode_MIN JointMode_OFF
#define _JointMode_MAX JointMode_CURRENT
#define _JointMode_ARRAYSIZE ((JointMode)(JointMode_CURRENT+1))

typedef enum _JointParams_Type {
    JointParams_Type_GRBL = 0,
    JointParams_Type_GRBLE = 1,
    JointParams_Type_RIGID = 2,
    JointParams_Type_DXL_PWM = 3,
    JointParams_Type_DXL_POS = 4
} JointParams_Type;
#define _JointParams_Type_MIN JointParams_Type_GRBL
#define _JointParams_Type_MAX JointParams_Type_DXL_POS
#define _JointParams_Type_ARRAYSIZE ((JointParams_Type)(JointParams_Type_DXL_POS+1))

typedef enum _LimbParams_Type {
    LimbParams_Type_SYMM5BAR_EXT_RAD = 0,
    LimbParams_Type_SYMM5BAR_EXT_M = 1,
    LimbParams_Type_PARA5BAR_EXT_RAD = 2,
    LimbParams_Type_PARA5BAR_EXT_M = 3,
    LimbParams_Type_PARA5BAR_CART = 4
} LimbParams_Type;
#define _LimbParams_Type_MIN LimbParams_Type_SYMM5BAR_EXT_RAD
#define _LimbParams_Type_MAX LimbParams_Type_PARA5BAR_CART
#define _LimbParams_Type_ARRAYSIZE ((LimbParams_Type)(LimbParams_Type_PARA5BAR_CART+1))

typedef enum _RobotParams_Type {
    RobotParams_Type_MINITAUR = 0,
    RobotParams_Type_MINITAUR_E = 1,
    RobotParams_Type_NGR = 2
} RobotParams_Type;
#define _RobotParams_Type_MIN RobotParams_Type_MINITAUR
#define _RobotParams_Type_MAX RobotParams_Type_NGR
#define _RobotParams_Type_ARRAYSIZE ((RobotParams_Type)(RobotParams_Type_NGR+1))

typedef enum _RobotCommand_Mode {
    RobotCommand_Mode_BEHAVIOR = 0,
    RobotCommand_Mode_LIMB = 1,
    RobotCommand_Mode_JOINT = 2
} RobotCommand_Mode;
#define _RobotCommand_Mode_MIN RobotCommand_Mode_BEHAVIOR
#define _RobotCommand_Mode_MAX RobotCommand_Mode_JOINT
#define _RobotCommand_Mode_ARRAYSIZE ((RobotCommand_Mode)(RobotCommand_Mode_JOINT+1))

/* Struct definitions */
typedef struct _RobotModelParams {
    char dummy_field;
/* @@protoc_insertion_point(struct:RobotModelParams) */
} RobotModelParams;

typedef struct _BatteryState {
    float voltage;
    float current;
    float percentage;
    float cell_voltage;
/* @@protoc_insertion_point(struct:BatteryState) */
} BatteryState;

typedef struct _BehaviorInfo {
    pb_size_t namei_count;
    int32_t namei[5];
/* @@protoc_insertion_point(struct:BehaviorInfo) */
} BehaviorInfo;

typedef struct _JointCmd {
    JointMode mode;
    float setpoint;
    float Kp;
    float Kd;
/* @@protoc_insertion_point(struct:JointCmd) */
} JointCmd;

typedef struct _JointParams {
    JointParams_Type type;
    uint32_t address;
    float gearRatio;
    int32_t direction;
    float zero;
    MotorType motorType;
/* @@protoc_insertion_point(struct:JointParams) */
} JointParams;

typedef struct _JointState {
    float position;
    float velocity;
    float voltage;
    float current;
    float temperature;
    float torqueEst;
/* @@protoc_insertion_point(struct:JointState) */
} JointState;

typedef struct _Joy {
    pb_size_t axes_count;
    float axes[4];
    pb_size_t buttons_count;
    int32_t buttons[10];
/* @@protoc_insertion_point(struct:Joy) */
} Joy;

typedef struct _MotorModelParams {
    float kE;
    float kT;
    float kR;
    float kJv;
    pb_size_t currents_count;
    float currents[10];
    pb_size_t torques_count;
    float torques[10];
/* @@protoc_insertion_point(struct:MotorModelParams) */
} MotorModelParams;

typedef struct _Quaternion {
    float x;
    float y;
    float z;
    float w;
/* @@protoc_insertion_point(struct:Quaternion) */
} Quaternion;

typedef struct _Vector3 {
    float x;
    float y;
    float z;
/* @@protoc_insertion_point(struct:Vector3) */
} Vector3;

typedef struct _Imu {
    Quaternion orientation;
    Vector3 angular_velocity;
    Vector3 linear_acceleration;
    Vector3 euler;
    pb_size_t orientation_covariance_count;
    float orientation_covariance[9];
    float linear_acceleration_covariance;
    float angular_velocity_covariance;
/* @@protoc_insertion_point(struct:Imu) */
} Imu;

typedef struct _LimbParams {
    LimbParams_Type type;
    pb_size_t kinParams_count;
    float kinParams[12];
    uint64_t fkFun;
    pb_size_t jointInd_count;
    uint32_t jointInd[6];
    Vector3 d;
/* @@protoc_insertion_point(struct:LimbParams) */
} LimbParams;

typedef struct _Pose {
    Vector3 position;
    Quaternion orientation;
/* @@protoc_insertion_point(struct:Pose) */
} Pose;

typedef struct _Twist {
    Vector3 linear;
    Vector3 angular;
/* @@protoc_insertion_point(struct:Twist) */
} Twist;

typedef struct _BehaviorCmd {
    uint32_t id;
    Twist twist;
    Pose pose;
    uint32_t mode;
/* @@protoc_insertion_point(struct:BehaviorCmd) */
} BehaviorCmd;

typedef struct _RobotParams {
    RobotParams_Type type;
    pb_size_t joints_count;
    JointParams joints[30];
    pb_size_t limbs_count;
    LimbParams limbs[6];
/* @@protoc_insertion_point(struct:RobotParams) */
} RobotParams;

typedef struct _RobotState {
    pb_size_t joints_count;
    JointState joints[30];
    Imu imu;
    BatteryState batt;
    Joy joy;
    Twist twist;
    uint32_t cmdMode;
    pb_size_t behaviors_count;
    BehaviorInfo behaviors[10];
    uint32_t behaviorId;
    uint32_t behaviorMode;
    uint32_t millis;
/* @@protoc_insertion_point(struct:RobotState) */
} RobotState;

typedef struct _RobotCommand {
    BehaviorCmd behavior;
    pb_size_t joints_count;
    JointCmd joints[30];
    RobotCommand_Mode mode;
/* @@protoc_insertion_point(struct:RobotCommand) */
} RobotCommand;

/* Default values for struct fields */

/* Initializer values for message structs */
#define Vector3_init_default                     {0, 0, 0}
#define Quaternion_init_default                  {0, 0, 0, 0}
#define Twist_init_default                       {Vector3_init_default, Vector3_init_default}
#define Pose_init_default                        {Vector3_init_default, Quaternion_init_default}
#define Imu_init_default                         {Quaternion_init_default, Vector3_init_default, Vector3_init_default, Vector3_init_default, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, 0}
#define JointState_init_default                  {0, 0, 0, 0, 0, 0}
#define BatteryState_init_default                {0, 0, 0, 0}
#define Joy_init_default                         {0, {0, 0, 0, 0}, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}
#define MotorModelParams_init_default            {0, 0, 0, 0, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}
#define RobotModelParams_init_default            {0}
#define JointParams_init_default                 {(JointParams_Type)0, 0, 0, 0, 0, (MotorType)0}
#define LimbParams_init_default                  {(LimbParams_Type)0, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, 0, {0, 0, 0, 0, 0, 0}, Vector3_init_default}
#define JointCmd_init_default                    {(JointMode)0, 0, 0, 0}
#define BehaviorInfo_init_default                {0, {0, 0, 0, 0, 0}}
#define BehaviorCmd_init_default                 {0, Twist_init_default, Pose_init_default, 0}
#define RobotParams_init_default                 {(RobotParams_Type)0, 0, {JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default, JointParams_init_default}, 0, {LimbParams_init_default, LimbParams_init_default, LimbParams_init_default, LimbParams_init_default, LimbParams_init_default, LimbParams_init_default}}
#define RobotState_init_default                  {0, {JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default, JointState_init_default}, Imu_init_default, BatteryState_init_default, Joy_init_default, Twist_init_default, 0, 0, {BehaviorInfo_init_default, BehaviorInfo_init_default, BehaviorInfo_init_default, BehaviorInfo_init_default, BehaviorInfo_init_default, BehaviorInfo_init_default, BehaviorInfo_init_default, BehaviorInfo_init_default, BehaviorInfo_init_default, BehaviorInfo_init_default}, 0, 0, 0}
#define RobotCommand_init_default                {BehaviorCmd_init_default, 0, {JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default, JointCmd_init_default}, (RobotCommand_Mode)0}
#define Vector3_init_zero                        {0, 0, 0}
#define Quaternion_init_zero                     {0, 0, 0, 0}
#define Twist_init_zero                          {Vector3_init_zero, Vector3_init_zero}
#define Pose_init_zero                           {Vector3_init_zero, Quaternion_init_zero}
#define Imu_init_zero                            {Quaternion_init_zero, Vector3_init_zero, Vector3_init_zero, Vector3_init_zero, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, 0}
#define JointState_init_zero                     {0, 0, 0, 0, 0, 0}
#define BatteryState_init_zero                   {0, 0, 0, 0}
#define Joy_init_zero                            {0, {0, 0, 0, 0}, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}
#define MotorModelParams_init_zero               {0, 0, 0, 0, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}
#define RobotModelParams_init_zero               {0}
#define JointParams_init_zero                    {(JointParams_Type)0, 0, 0, 0, 0, (MotorType)0}
#define LimbParams_init_zero                     {(LimbParams_Type)0, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, 0, {0, 0, 0, 0, 0, 0}, Vector3_init_zero}
#define JointCmd_init_zero                       {(JointMode)0, 0, 0, 0}
#define BehaviorInfo_init_zero                   {0, {0, 0, 0, 0, 0}}
#define BehaviorCmd_init_zero                    {0, Twist_init_zero, Pose_init_zero, 0}
#define RobotParams_init_zero                    {(RobotParams_Type)0, 0, {JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero, JointParams_init_zero}, 0, {LimbParams_init_zero, LimbParams_init_zero, LimbParams_init_zero, LimbParams_init_zero, LimbParams_init_zero, LimbParams_init_zero}}
#define RobotState_init_zero                     {0, {JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero, JointState_init_zero}, Imu_init_zero, BatteryState_init_zero, Joy_init_zero, Twist_init_zero, 0, 0, {BehaviorInfo_init_zero, BehaviorInfo_init_zero, BehaviorInfo_init_zero, BehaviorInfo_init_zero, BehaviorInfo_init_zero, BehaviorInfo_init_zero, BehaviorInfo_init_zero, BehaviorInfo_init_zero, BehaviorInfo_init_zero, BehaviorInfo_init_zero}, 0, 0, 0}
#define RobotCommand_init_zero                   {BehaviorCmd_init_zero, 0, {JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero, JointCmd_init_zero}, (RobotCommand_Mode)0}

/* Field tags (for use in manual encoding/decoding) */
#define BatteryState_voltage_tag                 1
#define BatteryState_current_tag                 2
#define BatteryState_percentage_tag              3
#define BatteryState_cell_voltage_tag            4
#define BehaviorInfo_namei_tag                   2
#define JointCmd_mode_tag                        1
#define JointCmd_setpoint_tag                    2
#define JointCmd_Kp_tag                          3
#define JointCmd_Kd_tag                          4
#define JointParams_type_tag                     1
#define JointParams_address_tag                  2
#define JointParams_gearRatio_tag                3
#define JointParams_direction_tag                4
#define JointParams_zero_tag                     5
#define JointParams_motorType_tag                8
#define JointState_position_tag                  1
#define JointState_velocity_tag                  2
#define JointState_voltage_tag                   3
#define JointState_current_tag                   4
#define JointState_temperature_tag               5
#define JointState_torqueEst_tag                 6
#define Joy_axes_tag                             1
#define Joy_buttons_tag                          2
#define MotorModelParams_kE_tag                  1
#define MotorModelParams_kT_tag                  2
#define MotorModelParams_kR_tag                  3
#define MotorModelParams_kJv_tag                 4
#define MotorModelParams_currents_tag            5
#define MotorModelParams_torques_tag             6
#define Quaternion_x_tag                         1
#define Quaternion_y_tag                         2
#define Quaternion_z_tag                         3
#define Quaternion_w_tag                         4
#define Vector3_x_tag                            1
#define Vector3_y_tag                            2
#define Vector3_z_tag                            3
#define Imu_orientation_tag                      1
#define Imu_angular_velocity_tag                 2
#define Imu_linear_acceleration_tag              3
#define Imu_euler_tag                            4
#define Imu_orientation_covariance_tag           5
#define Imu_linear_acceleration_covariance_tag   6
#define Imu_angular_velocity_covariance_tag      7
#define LimbParams_type_tag                      1
#define LimbParams_kinParams_tag                 2
#define LimbParams_fkFun_tag                     3
#define LimbParams_jointInd_tag                  4
#define LimbParams_d_tag                         5
#define Pose_position_tag                        1
#define Pose_orientation_tag                     2
#define Twist_linear_tag                         1
#define Twist_angular_tag                        2
#define BehaviorCmd_id_tag                       1
#define BehaviorCmd_twist_tag                    2
#define BehaviorCmd_pose_tag                     3
#define BehaviorCmd_mode_tag                     4
#define RobotParams_type_tag                     1
#define RobotParams_joints_tag                   2
#define RobotParams_limbs_tag                    3
#define RobotState_joints_tag                    1
#define RobotState_imu_tag                       2
#define RobotState_batt_tag                      3
#define RobotState_joy_tag                       4
#define RobotState_twist_tag                     5
#define RobotState_cmdMode_tag                   6
#define RobotState_behaviors_tag                 7
#define RobotState_behaviorId_tag                8
#define RobotState_behaviorMode_tag              9
#define RobotState_millis_tag                    11
#define RobotCommand_behavior_tag                1
#define RobotCommand_joints_tag                  3
#define RobotCommand_mode_tag                    4

/* Struct field encoding specification for nanopb */
extern const pb_field_t Vector3_fields[4];
extern const pb_field_t Quaternion_fields[5];
extern const pb_field_t Twist_fields[3];
extern const pb_field_t Pose_fields[3];
extern const pb_field_t Imu_fields[8];
extern const pb_field_t JointState_fields[7];
extern const pb_field_t BatteryState_fields[5];
extern const pb_field_t Joy_fields[3];
extern const pb_field_t MotorModelParams_fields[7];
extern const pb_field_t RobotModelParams_fields[1];
extern const pb_field_t JointParams_fields[7];
extern const pb_field_t LimbParams_fields[6];
extern const pb_field_t JointCmd_fields[5];
extern const pb_field_t BehaviorInfo_fields[2];
extern const pb_field_t BehaviorCmd_fields[5];
extern const pb_field_t RobotParams_fields[4];
extern const pb_field_t RobotState_fields[11];
extern const pb_field_t RobotCommand_fields[4];

/* Maximum encoded size of messages (where known) */
#define Vector3_size                             15
#define Quaternion_size                          20
#define Twist_size                               34
#define Pose_size                                39
#define Imu_size                                 128
#define JointState_size                          30
#define BatteryState_size                        20
#define Joy_size                                 130
#define MotorModelParams_size                    120
#define RobotModelParams_size                    0
#define JointParams_size                         31
#define LimbParams_size                          126
#define JointCmd_size                            17
#define BehaviorInfo_size                        55
#define BehaviorCmd_size                         89
#define RobotParams_size                         1760
#define RobotState_size                          1876
#define RobotCommand_size                        663

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define SIMPLE_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
