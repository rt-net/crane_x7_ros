#ifndef DXLPORT_CONTROL_H
#define DXLPORT_CONTROL_H

#include    <map>
#include    <string>
#include    <vector>
#include    <ros/ros.h>
#include    <hardware_interface/joint_command_interface.h>
#include    <hardware_interface/joint_state_interface.h>
#include    <hardware_interface/robot_hw.h>

#include    <dynamixel_sdk/dynamixel_sdk.h>
#include    <crane_x7_control/joint_control.h>
#include    <crane_x7_control/control_setting.h>


// Motion data
typedef struct HOME_MOTION_DATA {
    uint16_t    home;
    double      home_rad;
    uint16_t    start;
    double      start_rad;
    uint16_t    step;
    double      step_rad;
} ST_HOME_MOTION_DATA;


// Dynamixel protocol serial port class
class DXLPORT_CONTROL : public hardware_interface::RobotHW
{
public:
    DXLPORT_CONTROL( ros::NodeHandle handle, CONTROL_SETTING &setting );
    ~DXLPORT_CONTROL();
    ros::Time getTime() const { return ros::Time::now(); }
    ros::Duration       getDuration( ros::Time t ) const { return (ros::Time::now() - t); }
    void                read( ros::Time, ros::Duration );
    void                readPos( ros::Time, ros::Duration );
    void                readCurrent( ros::Time, ros::Duration );
    void                readTemp( ros::Time, ros::Duration );
    void                readVel( ros::Time, ros::Duration );
    void                write( ros::Time, ros::Duration );
    void                set_torque( bool torque );
    void                startup_motion( void );
    void                set_gain_all( uint16_t gain );
    void                set_gain( uint8_t dxl_id, uint16_t gain );
    void                set_goal_current_all( uint16_t current );
    void                set_goal_current( uint8_t dxl_id, uint16_t current );
    bool                get_init_stat( void ){ return init_stat; }
    uint8_t             get_joint_num( void ){ return joint_num; }
    std::string         self_check( void );
    void                effort_limitter( void );
    std::string                                 last_error;
    uint32_t                                    tempCount;
    std::vector<JOINT_CONTROL>                  joints;
    
private:
    uint8_t                                     joint_num;
    bool                                        port_stat;
    dynamixel::PacketHandler                   *packetHandler;
    dynamixel::PortHandler                     *portHandler;
    hardware_interface::JointStateInterface     joint_stat_if;
    hardware_interface::PositionJointInterface  joint_pos_if;
    hardware_interface::EffortJointInterface    joint_eff_if;
    joint_limits_interface::PositionJointSoftLimitsInterface joint_limits_if;
    dynamixel::GroupBulkRead                   *readPosGroup;
    dynamixel::GroupBulkRead                   *readCurrentGroup;
    dynamixel::GroupBulkRead                   *readTempGroup;
    dynamixel::GroupBulkRead                   *readVelGroup;
    dynamixel::GroupBulkWrite                  *writeGoalGroup;

    bool                                        init_stat;
    uint32_t                                    rx_err;
    uint32_t                                    tx_err;
    ros::Time                                   tempTime;
//    uint32_t                                    tempCount;

    bool                check_servo_param( uint8_t dxl_id, uint32_t test_addr, uint8_t equal );
    bool                check_servo_param( uint8_t dxl_id, uint32_t test_addr, uint16_t equal );
    bool                check_servo_param( uint8_t dxl_id, uint32_t test_addr, uint32_t equal );
};

#endif /* DXLPORT_CONTROL_H */
