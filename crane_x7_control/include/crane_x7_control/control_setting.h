#ifndef DXL_CONTROL_CONTROL_SETTING_H
#define DXL_CONTROL_CONTROL_SETTING_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/console.h>
#include <string>
#include <sstream>
#include <map>

#define     DEFAULT_CENTER     (2048)

/* SETTING KEY STRINGS */
#define     KEY_DXL_PORT       ("dynamixel_port")
#define     KEY_PORTNAME       ("/port_name")
#define     KEY_BAUDRATE       ("/baud_rate")
#define     KEY_JOINTS         ("/joints")
#define     KEY_JPARAM_ID      ("/id")
#define     KEY_JPARAM_CENTER  ("/center")
#define     KEY_JPARAM_HOME    ("/home")


typedef struct SERVO_PARAM {
    SERVO_PARAM(){
        name = "";
        id = 0;
        center = DEFAULT_CENTER;
    }
    std::string name;
    uint8_t     id;
    uint16_t    center;
    uint16_t    home;
} ST_SERVO_PARAM;

class CONTROL_SETTING
{
public:
    CONTROL_SETTING( ros::NodeHandle handle );
    ~CONTROL_SETTING();
    bool load( void );
    std::string getPortName( void ){ return port_name; }
    std::string loadPortName( void );
    std::vector<ST_SERVO_PARAM> getServoParam( void ){ return joint_list; }
    uint32_t getjointNum( void ){ return joint_num; }
    uint32_t getBaudrate( void ){ return port_baud_rate; }
    
private:
    ros::NodeHandle node_handle;
    uint8_t port_num;
    std::string port_name;
    uint32_t port_baud_rate;
    uint32_t joint_num;
    std::vector<ST_SERVO_PARAM> joint_list;
    
    bool isPortDefine( void );
    uint32_t loadBaudRate( void );
    bool loadJointList( void );
    bool loadJointParam( void );
};

#endif/* DXL_CONTROL_CONTROL_SETTING_H */
