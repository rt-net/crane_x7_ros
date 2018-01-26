#ifndef DXL_JOINT_CONTROL_H
#define DXL_JOINT_CONTROL_H

#include    <cstdio>
#include    <string>

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
static const ST_DYNAMIXEL_REG_TABLE RegTable[] ={
    /*  NAME                ADDR    LEN INIT    TYPE   */
    { "RETURN_DELAY_TIME",  9,      1,  250,    enDXL_ROM,  false },/* 0 */
    { "DRIVE_MODE",         10,     1,  0,      enDXL_ROM,  false },/* 1 */
    { "OPERATION_MODE",     11,     1,  3,      enDXL_ROM,  false },/* 2 */
    { "MOVING_THRESHOLD",   24,     4,  10,     enDXL_ROM,  false },/* 3 */
    { "TEMPRATURE_LIMIT",   31,     1,  80,     enDXL_ROM,  false },/* 4 */
    { "MAX_VOL_LIMIT",      32,     2,  160,    enDXL_ROM,  false },/* 5 */
    { "MIN_VOL_LIMIT",      34,     2,  95,     enDXL_ROM,  false },/* 6 */
    { "CURRENT_LIMIT",      38,     2,  1193,   enDXL_ROM,  false },/* 7 */
    { "SHUTDOWN",           63,     1,  52,     enDXL_ROM,  false },/* 8 */
    { "TORQUE_ENABLE",      64,     1,  0,      enDXL_RAM,  false },/* 9 */
    { "VELOCITY_I_GAIN",    76,     2,  1920,   enDXL_RAM,  true  },/* 10 */
    { "VELOCITY_P_GAIN",    78,     2,  100,    enDXL_RAM,  true  },/* 11 */
    { "POSITION_D_GAIN",    80,     2,  0,      enDXL_RAM,  true  },/* 12 */
    { "POSITION_I_GAIN",    82,     2,  0,      enDXL_RAM,  true  },/* 13 */
    { "POSITION_P_GAIN",    84,     2,  800,    enDXL_RAM,  false },/* 14 */
    { "GOAL_CURRENT",       102,    2,  0,      enDXL_RAM,  false },/* 15 */
    { "GOAL_VELOCITY",      104,    2,  0,      enDXL_RAM,  false },/* 16 */
    { "GOAL_POSITION",      116,    4,  0,      enDXL_RAM,  false },/* 17 */
    { "PRESENT_CURRENT",    126,    2,  0,      enDXL_RAM,  false },/* 18 */
    { "PRESENT_VELOCITY",   128,    2,  0,      enDXL_RAM,  false },/* 19 */
    { "PRESENT_POSITION",   132,    4,  0,      enDXL_RAM,  false },/* 20 */
    { "PRESENT_TEMPRATURE", 146,    1,  0,      enDXL_RAM,  false },/* 21 */
};

// Parameter
#define     BAUDRATE                        (3000000)         // 通信速度
#define     ADDR_TORQUE_ENABLE              (RegTable[9].address)              // Control table address is different in Dynamixel model
#define     ADDR_GOAL_POSITION              (RegTable[17].address)             // ゴールポジションアドレス
#define     LEN_GOAL_POSITION               (RegTable[17].length)               // ゴールポジション
#define     ADDR_PRESENT_POSITION           (RegTable[20].address)
#define     LEN_PRESENT_POSITION            (RegTable[20].length)
#define     ADDR_PRESENT_CURRENT            (RegTable[18].address)
#define     LEN_PRESENT_CURRENT             (RegTable[18].length)
#define     ADDR_PRESENT_TEMP               (RegTable[21].address)
#define     LEN_PRESENT_TEMP                (RegTable[21].length)
#define     ADDR_POSITION_PGAIN             (RegTable[14].address)
#define     LEN_POSITION_PGAIN              (RegTable[14].length)

#define     TORQUE_ENABLE                   (1)                // Value for enabling the torque
#define     TORQUE_DISABLE                  (0)                // Value for disabling the torque

#define     POSITION_STEP                   (4096.0)
#define     DXL_MIN_LIMIT                   (0.0)
#define     DXL_MAX_LIMIT                   (4095.0)
#define     DXL_CURRENT_UNIT                (2.69)             // mA
#define     DXL_TEMP_READ_DURATION          (5)
#define     DXL_PGAIN_MAX                   (16383)
#define     DXL_DEFAULT_PGAIN               (800)
#define     DXL_FREE_PGAIN                  (0)

#define     DXL_OFFSET_DEFAULT              (2048)

#define     DXL_TORQUE_ON_TIME              (5*1000)           // msec
#define     DXL_TORQUR_ON_STEP              (20)               // Hz
#define     DXL_TORQUE_ON_STEP_MAX          (DXL_TORQUE_ON_TIME / (1000 / DXL_TORQUR_ON_STEP))

// Joint control class
class JOINT_CONTROL
{
public:
    JOINT_CONTROL(void);
    JOINT_CONTROL( std::string init_name, uint8_t init_dxlid, uint16_t init_center, uint16_t init_home );
    JOINT_CONTROL( const JOINT_CONTROL &src );
    ~JOINT_CONTROL(void){ /* Nothing todo... */ }
    void                init_parameter( std::string init_name, uint8_t init_dxlid, uint16_t init_center, uint16_t init_home );
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
    
    std::string         get_joint_name( void ) { return name; }
    uint8_t             get_dxl_id( void ){ return id; }
    double              get_position( void ){ return pos; }
    double*             get_position_addr( void ){ return &pos; }
    double              get_velocity( void ){ return vel; }
    double*             get_velocity_addr( void ){ return &vel; }
    double              get_effort( void ){ return eff; }
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
    
private:
    std::string         name;       // ROS joint name
    uint8_t             id;         // Dynamixel ServoID
    double              pos;        // Present position
    double              vel;        // Present verocity
    double              eff;        // Present effort
    double              cmd;        // ROS HwInterface value
    double              curr;       // Present current[mA]
    double              temp;       // Present temprature
    bool                torque;     // Servo torque
    uint16_t            center;     // Servo center position offset( dynamixel position value, default:2048 )
    uint16_t            home;       // Servo home position( dynamixel position value )
    bool                connect;    // Servo connect status

    double              goal_pos;   // Goal position[rad]
    double              goal_vel;   // Goal velocity
    double              goal_eff;   // Goal effort

    uint32_t            dxl_pos;    // Dynamixel present position
    uint16_t            dxl_curr;   // Dynamixel current
    uint8_t             dxl_temp;   // Dynamixel temprature
    uint8_t             dxl_goal[4];// Dynamixel goal position
};
#endif /*DXL_JOINT_CONTROL_H */