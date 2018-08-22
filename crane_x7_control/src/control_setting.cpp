#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/console.h>
#include <string>
#include <sstream>
#include <map>
#include <crane_x7_control/control_setting.h>

CONTROL_SETTING::CONTROL_SETTING( ros::NodeHandle handle  )
  : node_handle(handle)
{
    joint_num = 0;
    port_name = "";
    joint_list.clear();
}

CONTROL_SETTING::~CONTROL_SETTING()
{
    /* Nothing todo... */
}

bool CONTROL_SETTING::load( void )
{
    bool result;

    /* Loading tty port name */
    std::string device_name;
    uint32_t baudrate;

    /* Loading tty port name */
    result = true;
    device_name = loadPortName();
    if( device_name.length() > 0 ){
        port_name = device_name;
    }else{
        return false;
    }
    /* Loading tty baudrate */
    baudrate = loadBaudRate();
    if( baudrate > 0 ){
        port_baud_rate = baudrate;
    }else{
        result = false;
    }
    /* Loading joints list */
    if( loadJointList() ){
        loadJointParam();
    }else{
        result = false;
    }

    return result;
}

std::string CONTROL_SETTING::loadPortName( void )
{
    std::string key_port_name = KEY_DXL_PORT;
    std::string result;

    key_port_name += KEY_PORTNAME;
    if( !node_handle.getParam( key_port_name, result ) ){
        ROS_ERROR("Undefined key %s.", key_port_name.c_str());
        result = "";
    }

    return result;
}

uint32_t CONTROL_SETTING::loadBaudRate( void )
{
    std::string key_baud_rate = KEY_DXL_PORT;
    key_baud_rate += KEY_BAUDRATE;
    int result;
    
    if( !node_handle.getParam( key_baud_rate, result ) ){
        ROS_ERROR("Undefined key %s.", key_baud_rate.c_str());
        result = 0;
    }

    return (uint32_t)result;
}

bool CONTROL_SETTING::loadJointList( void )
{
    std::string key_joint_list = KEY_DXL_PORT;
    bool result = false;
    XmlRpc::XmlRpcValue load_joints;

    key_joint_list += KEY_JOINTS;
    if( !node_handle.getParam( key_joint_list, load_joints ) ){
        ROS_ERROR("Undefined key %s.", key_joint_list.c_str());
    }else{
        if( load_joints.getType() != XmlRpc::XmlRpcValue::TypeArray ){
            ROS_ERROR("XmlRpc get type error! line%d", __LINE__);
        }else{
            for( int32_t i = 0; i < load_joints.size(); ++i ){
                if( load_joints[i].getType() != XmlRpc::XmlRpcValue::TypeString ){
                    ROS_ERROR("XmlRpc get type[%d] error! line%d", i, __LINE__);
                }else{
                    XmlRpc::XmlRpcValue &joint_name_work = load_joints[i];
                    ST_SERVO_PARAM work;
                    work.name = (std::string)joint_name_work;
                    joint_list.push_back( work );
                }
            }
            joint_num = joint_list.size();
            result = true;
        }
    }
    return result;
}

bool CONTROL_SETTING::loadJointParam( void )
{
    std::string key_joint_param = std::string(KEY_DXL_PORT) + "/";
    XmlRpc::XmlRpcValue load_joint_param;
    int load_id = 0;
    int load_center = DEFAULT_CENTER;
    int load_home = DEFAULT_CENTER;
    double load_eff_cnst = DEFAULT_EFF_CNST;
    int load_mode = DEFAULT_OPE_MODE;
    bool load_result;
    bool result = true;
    
    for( int jj=0 ; jj<joint_list.size() ; ++jj ){
        std::string key_jname = (key_joint_param + joint_list[jj].name);
        std::string key_jparam_id = (key_jname + KEY_JPARAM_ID);
        std::string key_jparam_center = (key_jname + KEY_JPARAM_CENTER);
        std::string key_jparam_home = (key_jname + KEY_JPARAM_HOME);
        std::string key_jparam_eff_cnst = (key_jname + KEY_JPARAM_EFFCNST);
        std::string key_jparam_mode = (key_jname + KEY_JPARAM_OPEMODE);
        load_result = true;
        
        if( !node_handle.getParam( key_jparam_id, load_id ) ){
            ROS_ERROR("Undefined joint id key %s. line%d", key_jparam_id.c_str(), __LINE__);
            load_result = false;
        }
        if( !node_handle.getParam( key_jparam_center, load_center ) ){
            ROS_ERROR("Undefined joint id key %s. line%d", key_jparam_id.c_str(), __LINE__);
            load_result = false;
        }
        if( !node_handle.getParam( key_jparam_home, load_home ) ){
            ROS_ERROR("Undefined joint id key %s. line%d", key_jparam_id.c_str(), __LINE__);
            load_result = false;
        }
        if( !node_handle.getParam( key_jparam_eff_cnst, load_eff_cnst ) ){
            ROS_ERROR("Undefined joint id key %s. line%d", key_jparam_id.c_str(), __LINE__);
            load_result = false;
        }
        if( !node_handle.getParam( key_jparam_mode, load_mode ) ){
            ROS_ERROR("Undefined joint id key %s. line%d", key_jparam_id.c_str(), __LINE__);
            load_result = false;
        }
        if( load_result ){
            joint_list[jj].id = load_id;
            joint_list[jj].center = load_center;
            joint_list[jj].home = load_home;
            joint_list[jj].eff_cnst = load_eff_cnst;
            joint_list[jj].mode = load_mode;
        }else{
            result = false;
            break;
        }
    }

    return result;
}

