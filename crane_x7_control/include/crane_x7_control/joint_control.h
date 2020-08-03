#ifndef DXL_JOINT_CONTROL_H
#define DXL_JOINT_CONTROL_H

#include    <cstdio>
#include    <string>
#include    <joint_limits_interface/joint_limits.h>
#include    <joint_limits_interface/joint_limits_interface.h>
#include    <joint_limits_interface/joint_limits_rosparam.h>
#include    <dynamic_reconfigure/server.h>
#include    <crane_x7_msgs/ServoParameterConfig.h>

// Protocol version
#define     PROTOCOL_VERSION                (2.0)              // See which protocol version is used in the Dynamixel

// DYNAMIXEL REGISTER TABLE (Dynamixel X)
typedef enum {
    enDXL_ROM,
    enDXL_RAM
} EN_DXL_MEMTYPE;
typedef struct {
    std::string     name;           /* データ名称 */
    uint32_t        address;        /* アドレス */
    uint32_t        length;         /* データ長 */
    uint32_t        init_value;     /* 初期値 */
    EN_DXL_MEMTYPE  type;           /* メモリ種別 */
    bool            selfcheck;      /* セルフチェック対象 */
} ST_DYNAMIXEL_REG_TABLE;

#define     REG_LENGTH_BYTE                 (1)
#define     REG_LENGTH_WORD                 (2)
#define     REG_LENGTH_DWORD                (4)

typedef enum {
    enTableId_ReturnDelay = 0,
    enTableId_DriveMode,
    enTableId_OpeMode,
    enTableId_HomingOffset,
    enTableId_MovingThreshold,
    enTableId_TempLimit,
    enTableId_MaxVolLimit,
    enTableId_MinVolLimit,
    enTableId_CurrentLimit,
    enTableId_Shutdown,
    enTableId_TorqueEnable,
    enTableId_VelocityIGain,
    enTableId_VelocityPGain,
    enTableId_PositionDGain,
    enTableId_PositionIGain,
    enTableId_PositionPGain,
    enTableId_BusWatchdog,
    enTableId_GoalCurrent,
    enTableId_GoalVelocity,
    enTableId_GoalPosition,
    enTableId_PresentCurrent,
    enTableId_PresentVelocity,
    enTableId_PresentPosition,
    enTableId_PresentTemp,
} EN_TABLE_ID;

static const ST_DYNAMIXEL_REG_TABLE RegTable[] ={
    /*  NAME                ADDR    LEN INIT    TYPE   */
    { "RETURN_DELAY_TIME",  9,      REG_LENGTH_BYTE,  250,    enDXL_ROM,  false },
    { "DRIVE_MODE",         10,     REG_LENGTH_BYTE,  0,      enDXL_ROM,  false },
    { "OPERATION_MODE",     11,     REG_LENGTH_BYTE,  3,      enDXL_ROM,  false },
    { "HOMING_OFFSET",      20,     REG_LENGTH_DWORD, 0,      enDXL_ROM,  false },
    { "MOVING_THRESHOLD",   24,     REG_LENGTH_DWORD, 10,     enDXL_ROM,  false },
    { "TEMPRATURE_LIMIT",   31,     REG_LENGTH_BYTE,  80,     enDXL_ROM,  false },
    { "MAX_VOL_LIMIT",      32,     REG_LENGTH_WORD,  160,    enDXL_ROM,  false },
    { "MIN_VOL_LIMIT",      34,     REG_LENGTH_WORD,  95,     enDXL_ROM,  false },
    { "CURRENT_LIMIT",      38,     REG_LENGTH_WORD,  1193,   enDXL_ROM,  false },
    { "SHUTDOWN",           63,     REG_LENGTH_BYTE,  52,     enDXL_ROM,  false },
    { "TORQUE_ENABLE",      64,     REG_LENGTH_BYTE,  0,      enDXL_RAM,  false },
    { "VELOCITY_I_GAIN",    76,     REG_LENGTH_WORD,  1920,   enDXL_RAM,  true  },
    { "VELOCITY_P_GAIN",    78,     REG_LENGTH_WORD,  100,    enDXL_RAM,  true  },
    { "POSITION_D_GAIN",    80,     REG_LENGTH_WORD,  0,      enDXL_RAM,  true  },
    { "POSITION_I_GAIN",    82,     REG_LENGTH_WORD,  0,      enDXL_RAM,  true  },
    { "POSITION_P_GAIN",    84,     REG_LENGTH_WORD,  800,    enDXL_RAM,  false },
    { "BUS_WATCHDOG",       98,     REG_LENGTH_BYTE,  0,      enDXL_RAM,  false },
    { "GOAL_CURRENT",       102,    REG_LENGTH_WORD,  0,      enDXL_RAM,  false },
    { "GOAL_VELOCITY",      104,    REG_LENGTH_DWORD, 0,      enDXL_RAM,  false },
    { "GOAL_POSITION",      116,    REG_LENGTH_DWORD, 0,      enDXL_RAM,  false },
    { "PRESENT_CURRENT",    126,    REG_LENGTH_WORD,  0,      enDXL_RAM,  false },
    { "PRESENT_VELOCITY",   128,    REG_LENGTH_DWORD, 0,      enDXL_RAM,  false },
    { "PRESENT_POSITION",   132,    REG_LENGTH_DWORD, 0,      enDXL_RAM,  false },
    { "PRESENT_TEMPRATURE", 146,    REG_LENGTH_BYTE,  0,      enDXL_RAM,  false },
};

typedef struct ST_JOINT_PARAM
{
    uint8_t  dxl_id;
    uint8_t  return_delay_time;
    uint8_t  drive_mode;
    uint8_t  operation_mode;
    uint16_t moving_threshold;
    int32_t homing_offset;
    uint8_t  temprature_limit;
    uint8_t  max_vol_limit;
    uint8_t  min_vol_limit;
    uint16_t current_limit;
    uint8_t  torque_enable;
    uint16_t velocity_i_gain;
    uint16_t velocity_p_gain;
    uint16_t position_d_gain;
    uint16_t position_i_gain;
    uint16_t position_p_gain;
} ST_JOINT_PARAM;

// Parameter
#define     BAUDRATE                        (3000000)         // 通信速度
#define     ADDR_RETURN_DELAY               (RegTable[enTableId_ReturnDelay].address)
#define     LEN_PRETURN_DELAY               (RegTable[enTableId_ReturnDelay].length)
#define     ADDR_DRIVE_MODE                 (RegTable[enTableId_DriveMode].address)
#define     LEN_DRIVE_MODE                  (RegTable[enTableId_DriveMode].length)
#define     ADDR_OPE_MODE                   (RegTable[enTableId_OpeMode].address)
#define     LEN_OPE_MODE                    (RegTable[enTableId_OpeMode].length)
#define     ADDR_HOMING_OFFSET              (RegTable[enTableId_HomingOffset].address)
#define     LEN_HOMING_OFFSET               (RegTable[enTableId_HomingOffset].length)
#define     ADDR_MOVING_THRESHOLD           (RegTable[enTableId_MovingThreshold].address)
#define     LEN_MOVING_THRESHOLD            (RegTable[enTableId_MovingThreshold].length)
#define     ADDR_TEMPRATURE_LIMIT           (RegTable[enTableId_TempLimit].address)
#define     LEN_TEMPRATURE_LIMIT            (RegTable[enTableId_TempLimit].length)
#define     ADDR_MAX_VOL_LIMIT              (RegTable[enTableId_MaxVolLimit].address)
#define     LEN_MAX_VOL_LIMIT               (RegTable[enTableId_MaxVolLimit].length)
#define     ADDR_MIN_VOL_LIMIT              (RegTable[enTableId_MinVolLimit].address)
#define     LEN_MIN_VOL_LIMIT               (RegTable[enTableId_MinVolLimit].length)
#define     ADDR_CURRENT_LIMIT              (RegTable[enTableId_CurrentLimit].address)
#define     LEN_CURRENT_LIMIT               (RegTable[enTableId_CurrentLimit].length)
#define     ADDR_TORQUE_ENABLE              (RegTable[enTableId_TorqueEnable].address)              // Control table address is different in Dynamixel model
#define     ADDR_VELOCITY_IGAIN             (RegTable[enTableId_VelocityIGain].address)
#define     LEN_VELOCITY_IGAIN              (RegTable[enTableId_VelocityIGain].length)
#define     ADDR_VELOCITY_PGAIN             (RegTable[enTableId_VelocityPGain].address)
#define     LEN_VELOCITY_PGAIN              (RegTable[enTableId_VelocityPGain].length)
#define     ADDR_POSITION_DGAIN             (RegTable[enTableId_PositionDGain].address)
#define     LEN_POSITION_DGAIN              (RegTable[enTableId_PositionDGain].length)
#define     ADDR_POSITION_IGAIN             (RegTable[enTableId_PositionIGain].address)
#define     LEN_POSITION_IGAIN              (RegTable[enTableId_PositionIGain].length)
#define     ADDR_POSITION_PGAIN             (RegTable[enTableId_PositionPGain].address)
#define     LEN_POSITION_PGAIN              (RegTable[enTableId_PositionPGain].length)
#define     ADDR_BUS_WATCHDOG               (RegTable[enTableId_BusWatchdog].address)
#define     LEN_BUS_WATCHDOG                (RegTable[enTableId_BusWatchdog].length)
#define     ADDR_GOAL_CURRENT               (RegTable[enTableId_GoalCurrent].address)             // ゴールカレントアドレス
#define     LEN_GOAL_CURRENT                (RegTable[enTableId_GoalCurrent].length)              // ゴールカレント
#define     ADDR_GOAL_POSITION              (RegTable[enTableId_GoalPosition].address)            // ゴールポジションアドレス
#define     LEN_GOAL_POSITION               (RegTable[enTableId_GoalPosition].length)             // ゴールポジション
#define     ADDR_PRESENT_CURRENT            (RegTable[enTableId_PresentCurrent].address)
#define     LEN_PRESENT_CURRENT             (RegTable[enTableId_PresentCurrent].length)
#define     ADDR_PRESENT_VEL                (RegTable[enTableId_PresentVelocity].address)
#define     LEN_PRESENT_VEL                 (RegTable[enTableId_PresentVelocity].length)
#define     ADDR_PRESENT_POSITION           (RegTable[enTableId_PresentPosition].address)
#define     LEN_PRESENT_POSITION            (RegTable[enTableId_PresentPosition].length)
#define     ADDR_PRESENT_TEMP               (RegTable[enTableId_PresentTemp].address)
#define     LEN_PRESENT_TEMP                (RegTable[enTableId_PresentTemp].length)

#define     ADDR_PRESENT_MOVEMENT           (ADDR_PRESENT_CURRENT)
#define     LEN_PRESENT_MOVEMENT             (LEN_PRESENT_CURRENT+LEN_PRESENT_VEL+LEN_PRESENT_POSITION)

#define     TORQUE_ENABLE                   (1)                // Value for enabling the torque
#define     TORQUE_DISABLE                  (0)                // Value for disabling the torque

#define     POSITION_STEP                   (4096.0)
#define     DXL_MIN_LIMIT                   (0.0)
#define     DXL_MAX_LIMIT                   (4095.0)
#define     DXL_CURRENT_UNIT                (2.69)             // mA
#define     DXL_EFFORT_CONST                (1.79)
#define     DXL_TEMP_READ_DURATION          (5)
#define     DXL_PGAIN_MAX                   (16383)
#define     DXL_DEFAULT_PGAIN               (800)
#define     DXL_FREE_PGAIN                  (0)
#define     DXL_FREE_IGAIN                  (0)
#define     DXL_FREE_DGAIN                  (0)

#define     DXL_OFFSET_DEFAULT              (2048)

#define     DXL_TORQUE_ON_TIME              (5*1000)           // msec
#define     DXL_TORQUR_ON_STEP              (20)               // Hz
#define     DXL_TORQUE_ON_STEP_MAX          (DXL_TORQUE_ON_TIME / (1000 / DXL_TORQUR_ON_STEP))

#define     DXL_CURRENT2EFFORT(c,coef)      (DXL_CURRENT_UNIT * (c) * (coef) * 0.001)// (mA*CONST*0.001)=Nm
#define     EFFORT2DXL_CURRENT(e,coef)      ((e) / (coef) / 0.001 / DXL_CURRENT_UNIT)// (Nm/CONST/0.001)=mA
#define     DEFAULT_MAX_EFFORT              (5.0)
#define     EFFORT_LIMITING_CNT             (10)
#define     DXL_VELOCITY2RAD_S(v)           ((v) * 0.229 * 0.1047)

#define     DXL_WATCHDOG_RESET_VALUE        (0)
#define     MSEC2DXL_WATCHDOG(msec)         ((uint8_t)(msec) / 20)

#define     OPERATING_MODE_CURRENT          (0)
#define     OPERATING_MODE_VELOCITY         (1)
#define     OPERATING_MODE_POSITION         (3)
#define     OPERATING_MODE_EXT_POS          (4)
#define     OPERATING_MODE_CURR_POS         (5)
#define     OPERATING_MODE_PWM              (16)


// Joint control class
class JOINT_CONTROL
{
public:
    JOINT_CONTROL(void);
    JOINT_CONTROL( std::string init_name, uint8_t init_dxlid, uint16_t init_center, uint16_t init_home, double init_eff_const, uint8_t init_mode );
    JOINT_CONTROL( const JOINT_CONTROL &src );
    ~JOINT_CONTROL(void){ /* Nothing todo... */ }
    void                init_parameter( std::string init_name, uint8_t init_dxlid, uint16_t init_center, uint16_t init_home, double init_eff_const, uint8_t init_mode );
    void                set_joint_name( std::string set_name ) { name = set_name; }
    void                set_dxl_id( uint8_t set_id ){ id = set_id; }
    void                set_position( double set_rad ){ pos = set_rad; }
    void                set_velocity( double set_vel ){ vel = set_vel; }
    void                set_effort( double set_eff ){ eff = set_eff; }
    void                set_command( double set_cmd ){ cmd = set_cmd; }
    void                set_torque( bool set_trq ){ torque = set_trq; }
    void                set_center( uint16_t set_center ){ center = set_center; }
    void                set_home( uint16_t set_home ){ home = set_home; }
    void                set_current( double set_curr ){ curr = set_curr; }
    void                set_temprature( double set_temp ){ temp = set_temp; }
    void                set_connect( bool set_connect ){ connect = set_connect; }
    void                set_dxl_pos( uint32_t set_dxl_pos ){ dxl_pos = set_dxl_pos; }
    void                set_dxl_curr( uint16_t set_dxl_curr ){ dxl_curr = set_dxl_curr; }
    void                set_dxl_temp( uint8_t set_dxl_temp ){ dxl_temp = set_dxl_temp; }
    //void                set_dxl_goal( uint32_t set_dxl_goal ){ dxl_goal = set_dxl_goal; }/* TBD   */
    void                set_limits( joint_limits_interface::JointLimits &set_limits ){ limits = set_limits; }
    void                set_eff_limiting( bool set_limiting ){ eff_limiting = set_limiting; }
    void                inc_eff_over( void ){ ++eff_over_cnt; }
    void                clear_eff_over( void ){ eff_over_cnt = 0; }
    void                set_joint_param( ST_JOINT_PARAM set_param){ param = set_param; }
    
    std::string         get_joint_name( void ) { return name; }
    uint8_t             get_dxl_id( void ){ return id; }
    double              get_position( void ){ return pos; }
    double*             get_position_addr( void ){ return &pos; }
    double              get_velocity( void ){ return vel; }
    double*             get_velocity_addr( void ){ return &vel; }
    double              get_effort( void ){ return eff; }
    double              get_max_effort( void ){ return (limits.has_effort_limits?limits.max_effort:DEFAULT_MAX_EFFORT); }
    double*             get_effort_addr( void ){ return &eff; }
    double              get_command( void ){ return cmd; }
    double*             get_command_addr( void ){ return &cmd; }
    bool                get_torque( void ){ return torque; }
    uint16_t            get_center( void ){ return center; }
    uint16_t            get_home( void ){ return home; }
    double              get_current( void ){ return curr; }
    double              get_temprature( void ){ return temp; }
    bool                is_connect( void ){ return connect; }
    uint32_t            get_dxl_pos( void ){ return dxl_pos; }
    uint16_t            get_dxl_curr( void ){ return dxl_curr; }
    uint8_t             get_dxl_temp( void ){ return dxl_temp; }
    uint8_t             get_dxl_goal( void ){ return (uint32_t)(dxl_goal[0]&0x000000FF)
                                                     | (uint32_t)((dxl_goal[1]<<8)&0x0000FF00)
                                                     | (uint32_t)((dxl_goal[2]<<16)&0x00FF0000)
                                                     | (uint32_t)((dxl_goal[3]<<24)&0xFF0000FF); }
    uint8_t*            get_dxl_goal_addr( void ){ return dxl_goal; }
    bool                is_effort_limiting( void ){ return eff_limiting; }
    uint8_t             get_eff_over_cnt( void ){ return eff_over_cnt; }
    double              get_eff_const( void ){ return eff_const; }
    uint8_t             get_ope_mode( void ){ return ope_mode; }
    ST_JOINT_PARAM      get_joint_param( void ){ return param; }
    void                updt_d_command( double val ){ d_cmd = (val - prev_cmd); prev_cmd = val; }
    double              get_d_command( void ){ return d_cmd; }

private:
    std::string         name;       // ROS joint name
    uint8_t             id;         // Dynamixel ServoID
    double              pos;        // Present position
    double              vel;        // Present verocity
    double              eff;        // Present effort
    double              cmd;        // ROS HwInterface value
    double              d_cmd;      // cmd value delta
    double              prev_cmd;   // Previous cmd value
    double              curr;       // Present current[mA]
    double              temp;       // Present temprature
    bool                torque;     // Servo torque
    uint16_t            center;     // Servo center position offset( dynamixel position value, default:2048 )
    uint16_t            home;       // Servo home position( dynamixel position value )
    bool                connect;    // Servo connect status
    double              eff_const;  // Servo effort constant
    bool                eff_limiting;// Effort limiting status
    uint8_t             eff_over_cnt;// Effort limiting status
    uint8_t             ope_mode;   // Operating mode

    double              goal_pos;   // Goal position[rad]
    double              goal_vel;   // Goal velocity
    double              goal_eff;   // Goal effort

    uint32_t            dxl_pos;    // Dynamixel present position
    uint16_t            dxl_curr;   // Dynamixel current
    uint8_t             dxl_temp;   // Dynamixel temprature
    uint8_t             dxl_goal[4];// Dynamixel goal position

    joint_limits_interface::JointLimits limits;// Joint limit parameters

    ST_JOINT_PARAM      param;
};
#endif /*DXL_JOINT_CONTROL_H */