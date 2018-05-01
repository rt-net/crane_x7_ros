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
#define     CONTROL_HZ (1000)

static std_msgs::String             lasterror_out;
static std::vector<ros::Publisher>  current_pub;
static std::vector<ros::Publisher>  dxl_position_pub;
static std::vector<ros::Publisher>  temp_pub;
static std::vector<ros::Subscriber> gain_sub;
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

void SigintHandler( int sig )
{
    ros::shutdown();
}

int main( int argc, char* argv[] )
{
    ros::init( argc, argv, ros::this_node::getName(), ros::init_options::NoSigintHandler );
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");
    signal(SIGINT, SigintHandler);

    CONTROL_SETTING setting( nhPrivate );
    if( !setting.load() ){
        return -1;
    }

    DXLPORT_CONTROL crane_x7( nhPrivate, setting );
    if( !crane_x7.get_init_stat() ){
        ROS_INFO("%s",crane_x7.last_error.c_str());
        return -1;
    }
    controller_manager::ControllerManager cm( &crane_x7, nh );
    driver_addr = &crane_x7;

    ros::Publisher lasterror_pub = nhPrivate.advertise<std_msgs::String>("lasterror", 10);
    init_topics( &crane_x7, nhPrivate );

    ros::Rate rate( CONTROL_HZ );
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Time t = crane_x7.getTime();
    ros::Duration d = crane_x7.getDuration(t);
    uint32_t prev_tempCount = 0;
    bool read_temp_flg = false;

    ROS_INFO( "%s", crane_x7.self_check().c_str() );

    crane_x7.startup_motion();

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
        if( lasterror_out.data != crane_x7.last_error ){
            lasterror_out.data = crane_x7.last_error;
            lasterror_pub.publish( lasterror_out );
        }

        crane_x7.read( t, d );
        cm.update( t, d );
        crane_x7.write( t, d );
        while( set_gain_request.size() > 0 ){
            ST_SET_GAIN_QUEUE gain_data = set_gain_request.front();
            crane_x7.set_gain( gain_data.dxl_id, gain_data.gain );
            set_gain_request.pop();
        }
        crane_x7.effort_limitter();
        ros::spinOnce();
        rate.sleep();
    }
    spinner.stop();
    crane_x7.set_gain_all( DXL_FREE_PGAIN );

    return 0;
}
