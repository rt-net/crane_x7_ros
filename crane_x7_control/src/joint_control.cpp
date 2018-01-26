#include    <ros/ros.h>
#include    <ros/package.h>
#include    <angles/angles.h>
#include    <crane_x7_control/joint_control.h>

void JOINT_CONTROL::init_parameter( std::string init_name, uint8_t init_dxlid, uint16_t init_center, uint16_t init_home )
{
    name     = init_name;
    id       = init_dxlid;
    pos      = 0.0;
    vel      = 0.0;
    eff      = 0.0;
    curr     = 0.0;
    torque   = false;
    center   = init_center;
    home     = init_home;
    connect  = false;

    goal_pos = 0.0;
    goal_vel = 0.0;
    goal_eff = 0.0;

    dxl_pos  = 0;
    dxl_curr = 0;
    dxl_temp = 0;
    for( int i=0 ; i<sizeof(dxl_goal) ; ++i ){
        dxl_goal[i] = 0;
    }
}
JOINT_CONTROL::JOINT_CONTROL(void)
{ 
    init_parameter( std::string(""), 0, DXL_OFFSET_DEFAULT, DXL_OFFSET_DEFAULT );
}
JOINT_CONTROL::JOINT_CONTROL( std::string init_name, uint8_t init_dxlid, uint16_t init_center, uint16_t init_home )
{
    init_parameter( init_name, init_dxlid, init_center, init_home );
}
JOINT_CONTROL::JOINT_CONTROL( const JOINT_CONTROL &src )
{
    name     = src.name;
    id       = src.id;
    pos      = src.pos;
    vel      = src.vel;
    eff      = src.eff;
    curr     = src.curr;
    torque   = src.torque;
    center   = src.center;
    home     = src.home;
    connect  = src.connect;

    goal_pos = src.goal_pos;
    goal_vel = src.goal_vel;
    goal_eff = src.goal_eff;

    dxl_pos  = src.dxl_pos;
    dxl_curr = src.dxl_curr;
    dxl_temp = src.dxl_temp;
    for( int i=0 ; i<sizeof(dxl_goal) ; ++i ){
        dxl_goal[i] = src.dxl_goal[i];
    }
}
