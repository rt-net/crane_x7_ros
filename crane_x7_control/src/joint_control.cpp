#include    <ros/ros.h>
#include    <ros/package.h>
#include    <angles/angles.h>
#include    <crane_x7_control/joint_control.h>

void JOINT_CONTROL::init_parameter( std::string init_name, uint8_t init_dxlid, uint16_t init_center, uint16_t init_home, double init_eff_const, uint8_t init_mode )
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
    for( int ii=0 ; ii<sizeof(dxl_goal) ; ++ii ){
        dxl_goal[ii] = 0;
    }
    eff_const = init_eff_const;
    eff_limiting = false;
    eff_over_cnt = 0;

    if( init_mode == OPERATING_MODE_CURRENT ){
        ope_mode = OPERATING_MODE_CURRENT;
    }else{
        ope_mode = OPERATING_MODE_POSITION;
    }
}
JOINT_CONTROL::JOINT_CONTROL(void)
{ 
    init_parameter( std::string(""), 0, DXL_OFFSET_DEFAULT, DXL_OFFSET_DEFAULT, DXL_EFFORT_CONST, OPERATING_MODE_POSITION );
}
JOINT_CONTROL::JOINT_CONTROL( std::string init_name, uint8_t init_dxlid, uint16_t init_center, uint16_t init_home, double init_eff_const, uint8_t init_mode )
{
    init_parameter( init_name, init_dxlid, init_center, init_home, init_eff_const, init_mode );
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
    for( int ii=0 ; ii<sizeof(dxl_goal) ; ++ii ){
        dxl_goal[ii] = src.dxl_goal[ii];
    }
    eff_const = src.eff_const;
    eff_limiting = src.eff_limiting;
    eff_over_cnt = src.eff_over_cnt;

    ope_mode = src.ope_mode;
}

