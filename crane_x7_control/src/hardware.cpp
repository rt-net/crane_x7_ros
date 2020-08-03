#include    <signal.h>
#include    <ros/ros.h>
#include    <ros/console.h>
#include    <controller_manager/controller_manager.h>
#include    <crane_x7_control/dxlport_control.h>
#include    <std_msgs/Int32.h>
#include    <std_msgs/Int16.h>
#include    <std_msgs/UInt16.h>
#include    <std_msgs/Int8.h>
#include    <std_msgs/String.h>
#include    <std_msgs/Float64.h>
#include    <sstream>
#include    <queue>

/* ROS rate setting */
//#define REACTIVE_RATE_FUNCTION
#ifdef REACTIVE_RATE_FUNCTION
#define     CONTROL_WAIT_HZ   (50)
#define     CONTROL_ACTIVE_HZ (200)
#else
#define     CONTROL_HZ   (200)
#endif

static std_msgs::String             lasterror_out;
static std::vector<ros::Publisher>  current_pub;
static std::vector<ros::Publisher>  dxl_position_pub;
static std::vector<ros::Publisher>  temp_pub;
static std::vector<ros::Subscriber> gain_sub;
typedef dynamic_reconfigure::Server<crane_x7_msgs::ServoParameterConfig> RECONFIG_TYPE;
static std::vector<std::unique_ptr<RECONFIG_TYPE>>  reconfig_srv;
static DXLPORT_CONTROL*             driver_addr;

typedef struct SET_GAIN_QUEUE
{
    SET_GAIN_QUEUE()
    {
        dxl_id = 0;
        gain = 0;
    }
    SET_GAIN_QUEUE( const SET_GAIN_QUEUE &src )
    {
        dxl_id = src.dxl_id;
        gain = src.gain;
    }
    uint8_t  dxl_id;
    uint16_t gain;
} ST_SET_GAIN_QUEUE;
static std::queue<ST_SET_GAIN_QUEUE> set_gain_request;

static std::queue<ST_JOINT_PARAM> set_joint_param_request;

std::vector<std::string> split(const std::string& input, char delimiter)
{
    std::istringstream stream(input);

    std::string field;
    std::vector<std::string> result;
    while (std::getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}

void gainCallback(const ros::MessageEvent<std_msgs::UInt16 const>& event)
{
    const ros::M_string& header = event.getConnectionHeader();
    std::string topic = header.at("topic");
    const std_msgs::UInt16ConstPtr& msg = event.getMessage();
    uint16_t set_gain_data = msg->data;
    if( set_gain_data >= DXL_PGAIN_MAX ){
        set_gain_data = DXL_PGAIN_MAX;
    }

    std::vector<std::string> topic_list = split(topic,'/');
    std::string joint_name = topic_list[ topic_list.size()-2 ];// Parent topic name

    for( std::vector<JOINT_CONTROL>::iterator it=driver_addr->joints.begin() ; it!=driver_addr->joints.end() ; ++it ){
        if( it->get_joint_name() == joint_name ){
            ST_SET_GAIN_QUEUE set_req_data;
            set_req_data.dxl_id = it->get_dxl_id();
            set_req_data.gain = set_gain_data;
            set_gain_request.push( set_req_data );
            break;
        }
    }
}
void reconfigureCallback(crane_x7_msgs::ServoParameterConfig &config, uint32_t level, uint8_t id )
{
    ST_JOINT_PARAM set_req_data;
    set_req_data.dxl_id            = id;
    set_req_data.return_delay_time = config.return_delay_time;
    set_req_data.drive_mode        = config.drive_mode;
    set_req_data.operation_mode    = config.operation_mode;
    set_req_data.moving_threshold  = config.moving_threshold;
    set_req_data.homing_offset     = config.homing_offset;
    set_req_data.temprature_limit  = config.temprature_limit;
    set_req_data.max_vol_limit     = config.max_vol_limit;
    set_req_data.min_vol_limit     = config.min_vol_limit;
    set_req_data.current_limit     = config.current_limit;
    set_req_data.torque_enable     = config.torque_enable?1:0;
    set_req_data.velocity_i_gain   = config.velocity_i_gain;
    set_req_data.velocity_p_gain   = config.velocity_p_gain;
    set_req_data.position_d_gain   = config.position_d_gain;
    set_req_data.position_i_gain   = config.position_i_gain;
    set_req_data.position_p_gain   = config.position_p_gain;
    if( level != 0 ){
        for( std::vector<JOINT_CONTROL>::iterator it=driver_addr->joints.begin() ; it!=driver_addr->joints.end() ; ++it ){
            if( it->get_dxl_id() == id ){
                ST_JOINT_PARAM load_data = it->get_joint_param();
                config.return_delay_time = load_data.return_delay_time;
                config.drive_mode        = load_data.drive_mode;
                config.operation_mode    = load_data.operation_mode;
                config.moving_threshold  = load_data.moving_threshold;
                config.homing_offset     = load_data.homing_offset;
                config.temprature_limit  = load_data.temprature_limit;
                config.max_vol_limit     = load_data.max_vol_limit;
                config.min_vol_limit     = load_data.min_vol_limit;
                config.current_limit     = load_data.current_limit;
                config.torque_enable     = true;
                config.velocity_i_gain   = load_data.velocity_i_gain;
                config.velocity_p_gain   = load_data.velocity_p_gain;
                config.position_d_gain   = load_data.position_d_gain;
                config.position_i_gain   = load_data.position_i_gain;
                config.position_p_gain   = DXL_DEFAULT_PGAIN;
            }
        }
    }else{
        set_joint_param_request.push( set_req_data );
    }
}

void init_topics( DXLPORT_CONTROL *driver, ros::NodeHandle nh )
{
    std::string current_name;
    std::string dxl_pos_name;
    std::string temp_name;
    std::string gain_name;
    std::string joint_name;

    for( std::vector<JOINT_CONTROL>::iterator it=driver->joints.begin() ; it!=driver->joints.end() ; ++it ){
        joint_name = it->get_joint_name();
        current_name = ( joint_name + "/current");
        current_pub.push_back( nh.advertise<std_msgs::Float64>(current_name, 10) );
        dxl_pos_name = ( joint_name + "/dxl_position");
        dxl_position_pub.push_back( nh.advertise<std_msgs::Int16>(dxl_pos_name, 10) );
        temp_name = ( joint_name + "/temp");
        temp_pub.push_back( nh.advertise<std_msgs::Float64>(temp_name, 10) );
        gain_name = ( joint_name + "/gain");
        gain_sub.push_back( nh.subscribe(gain_name, 100, gainCallback) );
    }
}

void init_reconfigure(  DXLPORT_CONTROL *driver )
{
    for( std::vector<JOINT_CONTROL>::iterator it=driver->joints.begin() ; it!=driver->joints.end() ; ++it ){
        RECONFIG_TYPE* server = new RECONFIG_TYPE( "~/"+it->get_joint_name() );
        dynamic_reconfigure::Server<crane_x7_msgs::ServoParameterConfig>::CallbackType f;
        f = boost::bind(reconfigureCallback, _1, _2, it->get_dxl_id());
        server->setCallback(f);
        reconfig_srv.push_back( std::unique_ptr<RECONFIG_TYPE>(server) );
     }
}

void publish_topic_data( DXLPORT_CONTROL *driver, bool temp_flg )
{
    int joint_index;
    std::vector<ros::Publisher>::iterator current_it;
    std::vector<ros::Publisher>::iterator dxl_pos_it;
    std::vector<ros::Publisher>::iterator temp_it;

    std_msgs::Float64 current_out;
    std_msgs::Int16 dxl_pos_out;
    std_msgs::Float64 temp_out;

    current_it = current_pub.begin();
    dxl_pos_it = dxl_position_pub.begin();
    temp_it    = temp_pub.begin();

    for( std::vector<JOINT_CONTROL>::iterator it=driver->joints.begin() ; it!=driver->joints.end() ; ++it ){
        current_out.data = it->get_current();
        current_it->publish( current_out );
        ++current_it;
        dxl_pos_out.data = it->get_dxl_pos();
        dxl_pos_it->publish( dxl_pos_out );
        ++dxl_pos_it;
        if( temp_flg ){
            temp_out.data = it->get_temprature();
            temp_it->publish( temp_out );
            ++temp_it;
        }
    }
}

void write_joint_param( DXLPORT_CONTROL& driver, ST_JOINT_PARAM set_param )
{
    uint8_t dxl_id = set_param.dxl_id;

    driver.set_param_delay_time( dxl_id, set_param.return_delay_time );
    driver.set_param_drive_mode( dxl_id, set_param.drive_mode );
    driver.set_param_ope_mode( dxl_id, set_param.operation_mode );
    driver.set_param_home_offset( dxl_id, set_param.homing_offset );
    driver.set_param_moving_threshold( dxl_id, set_param.moving_threshold );
    driver.set_param_temp_limit( dxl_id, set_param.temprature_limit );
    driver.set_param_vol_limit( dxl_id, set_param.max_vol_limit, set_param.min_vol_limit );
    driver.set_param_current_limit( dxl_id, set_param.current_limit );
    driver.set_param_vel_gain( dxl_id, set_param.velocity_p_gain, set_param.velocity_i_gain );
    driver.set_param_pos_gain( dxl_id, set_param.position_p_gain, set_param.position_i_gain, set_param.position_d_gain );
    driver.set_torque( dxl_id, (set_param.torque_enable?true:false) );
}

void SigintHandler( int sig )
{
    ros::shutdown();
}

int main( int argc, char* argv[] )
{
    ros::init( argc, argv, ros::this_node::getName(), ros::init_options::NoSigintHandler );
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");
    std::string config_ns_str;
    nhPrivate.getParam("config_ns", config_ns_str);
    ros::NodeHandle nhConfig(config_ns_str);
    signal(SIGINT, SigintHandler);

    CONTROL_SETTING setting( nhConfig );
    if( !setting.load() ){
        return -1;
    }

    DXLPORT_CONTROL crane_x7( nhPrivate, setting );
    if( !crane_x7.get_init_stat() ){
        std::string errorlog;
        if( crane_x7.get_error( errorlog ) > 0 ){
            ROS_INFO("Initialize error (%s)",errorlog.c_str());
        }
        return -1;
    }
    controller_manager::ControllerManager cm( &crane_x7, nh );
    driver_addr = &crane_x7;

    ros::Publisher lasterror_pub = nhPrivate.advertise<std_msgs::String>("lasterror", 10);
    init_topics( &crane_x7, nhPrivate );

#ifdef REACTIVE_RATE_FUNCTION
    ros::Rate rate_wait( CONTROL_WAIT_HZ );
    ros::Rate rate_active( CONTROL_ACTIVE_HZ );
#else
    ros::Rate rate( CONTROL_HZ );
#endif
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Time t = crane_x7.getTime();
    ros::Duration d = crane_x7.getDuration(t);
    uint32_t prev_tempCount = 0;
    bool read_temp_flg = false;

    ROS_INFO( "%s", crane_x7.self_check().c_str() );
    init_reconfigure( &crane_x7 );

    crane_x7.set_watchdog_all( DXL_WATCHDOG_RESET_VALUE );
    crane_x7.startup_motion();
    crane_x7.set_watchdog_all( MSEC2DXL_WATCHDOG(1000) ); //1秒の間,無通信で停止

    while( ros::ok() ){
        d = crane_x7.getDuration(t);
        t = crane_x7.getTime();

        if( crane_x7.tempCount != prev_tempCount ){
            read_temp_flg = true;
            prev_tempCount = crane_x7.tempCount;
        }else{
            read_temp_flg = false;
        }
        publish_topic_data( &crane_x7, read_temp_flg );

        crane_x7.read( t, d );
        cm.update( t, d );
        crane_x7.write( t, d );

        while( set_gain_request.size() > 0 ){
            ST_SET_GAIN_QUEUE gain_data = set_gain_request.front();
            crane_x7.set_gain( gain_data.dxl_id, gain_data.gain );
            set_gain_request.pop();
        }

        // Dynamixelとの通信タイムアウトを防ぐため、
        // write_joint_param()は1制御ループで1回のみ実行する
        if( set_joint_param_request.size() > 0 ){
            write_joint_param( crane_x7, set_joint_param_request.front() );
            set_joint_param_request.pop();
        }
        crane_x7.effort_limitter();

        std::string errorlog;
        while( crane_x7.get_error( errorlog ) > 0  ){    // error log check
            lasterror_out.data = errorlog;
            lasterror_pub.publish( lasterror_out );
        }

#ifdef REACTIVE_RATE_FUNCTION
        if( crane_x7.is_change_positions() ){
            rate_active.sleep();
        }else{
            rate_wait.sleep();
        }
#else
        rate.sleep();
#endif
    }

    crane_x7.set_param_pos_gain_all( DXL_FREE_PGAIN, DXL_FREE_IGAIN, DXL_FREE_DGAIN );
    crane_x7.set_goal_current_all( 0 );
    spinner.stop();

    return 0;
}
