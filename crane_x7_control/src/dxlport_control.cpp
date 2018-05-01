#include    <ros/ros.h>
#include    <ros/package.h>
#include    <angles/angles.h>
#include    <crane_x7_control/dxlport_control.h>
#include    <math.h>
#include    <stdio.h>

/* MACRO */
#define     DXLPOS2RAD(pos)    (( ((pos)*(360.0/POSITION_STEP) )/360) *2*M_PI)
#define     RAD2DXLPOS(rad)    (( ((rad)/2.0/M_PI)*360.0 ) * (POSITION_STEP / 360.0))
#define     INIT_EFFCNST_UNIT(c) ((c)*0.001)


DXLPORT_CONTROL::DXLPORT_CONTROL( ros::NodeHandle handle, CONTROL_SETTING &setting )
{
    int j;
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    joint_limits_interface::JointLimits limits;
    joint_limits_interface::SoftJointLimits soft_limits;
    int position_mode_joint_num = 0;
    int current_mode_joint_num = 0;

    init_stat = false;
    tx_err = rx_err = 0;
    tempCount = 0;

    portHandler      = NULL;
    readPosGroup     = NULL;
    writeGoalGroup   = NULL;
    readCurrentGroup = NULL;
    readTempGroup    = NULL;
    readVelGroup     = NULL;

    joint_num = setting.getjointNum();
    std::vector<ST_SERVO_PARAM> list = setting.getServoParam();
    for( j=0 ; j<joint_num ; ++j ){
        JOINT_CONTROL work( list[j].name, list[j].id, list[j].center, list[j].home, list[j].eff_cnst, list[j].mode );
        joints.push_back( work );
    }

    /* DynamixelSDKとros_controlへの接続初期化 */
    packetHandler    = dynamixel::PacketHandler::getPacketHandler( PROTOCOL_VERSION );
    portHandler      = dynamixel::PortHandler::getPortHandler( setting.getPortName().c_str() );
    readPosGroup     = new dynamixel::GroupBulkRead( portHandler, packetHandler );
    writeGoalGroup   = new dynamixel::GroupBulkWrite( portHandler, packetHandler );
    readTempGroup    = new dynamixel::GroupBulkRead( portHandler, packetHandler );
    readCurrentGroup = new dynamixel::GroupBulkRead( portHandler, packetHandler );
    readVelGroup     = new dynamixel::GroupBulkRead( portHandler, packetHandler );
    
    for( j=0 ; j<joint_num ; ++j ){
        uint8_t dxl_id = joints[j].get_dxl_id();
        if( !readPosGroup->addParam( dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION ) ){
            last_error = "Bulk pos read setting failed.";
            return;
        }
        if( joints[j].get_ope_mode() == OPERATING_MODE_CURRENT ){
            //CURRENT MODE
            if( !writeGoalGroup->addParam( dxl_id, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT, joints[j].get_dxl_goal_addr() ) ){// [TODO]
                last_error = "Bulk current write setting failed.";
                return;
            }
        }else{
            //POSITION MODE
            if( !writeGoalGroup->addParam( dxl_id, ADDR_GOAL_POSITION, LEN_GOAL_POSITION, joints[j].get_dxl_goal_addr() ) ){// [TODO]
                last_error = "Bulk pos write setting failed.";
                return;
            }
        }
        if( !readTempGroup->addParam( dxl_id, ADDR_PRESENT_TEMP, LEN_PRESENT_TEMP ) ){
            last_error = "Bulk temp read setting failed.";
            return;
        }
        if( !readCurrentGroup->addParam( dxl_id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT ) ){
            last_error = "Bulk current read setting failed.";
            return;
        }
        if( joints[j].get_ope_mode() == OPERATING_MODE_CURRENT ){
            if( !readVelGroup->addParam( dxl_id, ADDR_PRESENT_VEL, LEN_PRESENT_VEL ) ){
                last_error = "Bulk velocity read setting failed.";
                return;
            }
        }
    }

    // Open port
    last_error = "";
    if( !portHandler->openPort() ){
        last_error = "Port open failed.";
        port_stat = false;
    }else{
        // Set port baudrate
        if( !portHandler->setBaudRate( setting.getBaudrate() ) ){
            last_error = "Setup baudrate failed.";
            port_stat = false;
        }else{
            for( j=0 ; j<joint_num ; ++j ){
                uint8_t dxl_id = joints[j].get_dxl_id();
                int32_t present_pos = 0;

                dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION, (uint32_t*)&present_pos, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS){
                    last_error = packetHandler->getTxRxResult( dxl_comm_result );
                    return;
                }else if ( dxl_error != 0 ){
                    last_error = packetHandler->getRxPacketError( dxl_error );
                    return;
                }else{
                    joints[j].set_position( DXLPOS2RAD( present_pos ) );
                }
            }
            port_stat = true;
        }
    }
    for( j=0 ; j<joint_num ; ++j ){
        hardware_interface::JointStateHandle reg_state_handle( joints[j].get_joint_name(), joints[j].get_position_addr(), joints[j].get_velocity_addr(), joints[j].get_effort_addr() );
        joint_stat_if.registerHandle( reg_state_handle );
    }
    registerInterface( &joint_stat_if );

    for( j=0 ; j<joint_num ; ++j ){
        hardware_interface::JointHandle reg_joint_handle( joint_stat_if.getHandle(joints[j].get_joint_name()), joints[j].get_command_addr() );
        if( joints[j].get_ope_mode() == OPERATING_MODE_CURRENT ){
            joint_eff_if.registerHandle( reg_joint_handle );
            ++current_mode_joint_num;
        }else{
            joint_pos_if.registerHandle( reg_joint_handle );
            ++position_mode_joint_num;
        }
        // Get limits
        if( joint_limits_interface::getJointLimits( joints[j].get_joint_name(), handle, limits ) ){
            joints[j].set_limits( limits );
            soft_limits.k_position = 1.0;
            soft_limits.k_velocity = 1.0;
            soft_limits.max_position = limits.max_position;
            soft_limits.min_position = limits.min_position;
            joint_limits_interface::PositionJointSoftLimitsHandle
                            reg_limits_handle( reg_joint_handle, limits, soft_limits );
            joint_limits_if.registerHandle( reg_limits_handle );
        }
    }
    if( position_mode_joint_num > 0 ){
        registerInterface( &joint_pos_if );
    }
    if( current_mode_joint_num > 0 ){
        registerInterface( &joint_eff_if );
    }

    init_stat = true;
}

DXLPORT_CONTROL::~DXLPORT_CONTROL()
{
    portHandler->closePort();
	delete( portHandler );
	/* packetHandlerはdeleteしないほうが良さそう*/
    if(readPosGroup!=NULL)     delete( readPosGroup );
    if(readTempGroup!=NULL)    delete( readTempGroup );
    if(writeGoalGroup!=NULL)    delete( writeGoalGroup );
    if(readCurrentGroup!=NULL) delete( readCurrentGroup );
    if(readVelGroup!=NULL)     delete( readVelGroup );
}

void DXLPORT_CONTROL::read( ros::Time time, ros::Duration period )
{
    if( !port_stat ){
        return;
    }
    readPos( time, period );
    readVel( time, period );
    readCurrent( time, period );
    if( (time - tempTime).toSec() > DXL_TEMP_READ_DURATION ){
        readTemp( time, period );
        tempTime = time;
    }
}

void DXLPORT_CONTROL::readPos( ros::Time time, ros::Duration period )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    bool dxl_getdata_result = false;
    uint8_t dxl_error = 0;                          // Dynamixel error

    last_error = "";
    dxl_comm_result = readPosGroup->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS){
        last_error = packetHandler->getTxRxResult( dxl_comm_result );
        ++rx_err;
    }else{
        for( int j=0 ; j<joint_num ; ++j ){
            int32_t present_pos = 0;
            uint8_t dxl_id = joints[j].get_dxl_id();
            dxl_getdata_result = readPosGroup->isAvailable( dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION );
            if( !dxl_getdata_result ){
                ++rx_err;
                break;
            }else{
                present_pos = readPosGroup->getData( dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION );
                joints[j].set_dxl_pos( present_pos );
                present_pos = (present_pos - joints[j].get_center());
                joints[j].set_position( DXLPOS2RAD( present_pos ) );
            }
        }
    }
}

void DXLPORT_CONTROL::readCurrent( ros::Time time, ros::Duration period )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    bool dxl_getdata_result = false;
    uint8_t dxl_error = 0;                          // Dynamixel error

    last_error = "";
    dxl_comm_result = readCurrentGroup->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS){
        last_error = packetHandler->getTxRxResult( dxl_comm_result );
        ++rx_err;
    }else{
        for( int j=0 ; j<joint_num ; ++j ){
            int16_t present_current = 0;
            uint8_t dxl_id = joints[j].get_dxl_id();
            dxl_getdata_result = readCurrentGroup->isAvailable( dxl_id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT );
            if( !dxl_getdata_result ){
                ++rx_err;
                break;
            }else{
                present_current = readCurrentGroup->getData( dxl_id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT );
                joints[j].set_dxl_curr( present_current );
                joints[j].set_current( (DXL_CURRENT_UNIT * present_current) );
                joints[j].set_effort( fabs(DXL_CURRENT2EFFORT( present_current, joints[j].get_eff_const() )) );//ROSは力のかかっている方向を扱わないので絶対値に加工する
            }
        }
    }
}

void DXLPORT_CONTROL::readTemp( ros::Time time, ros::Duration period )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    bool dxl_getdata_result = false;
    uint8_t dxl_error = 0;                          // Dynamixel error
   
    last_error = "";
    dxl_comm_result = readTempGroup->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS){
        last_error = packetHandler->getTxRxResult( dxl_comm_result );
        ++rx_err;
    }else{
        for( int j=0 ; j<joint_num ; ++j ){
            uint8_t dxl_id = joints[j].get_dxl_id();
            dxl_getdata_result = readTempGroup->isAvailable( dxl_id, ADDR_PRESENT_TEMP, LEN_PRESENT_TEMP );
            if( !dxl_getdata_result ){
                ++rx_err;
                break;
            }else{
                uint8_t present_current = readTempGroup->getData( dxl_id, ADDR_PRESENT_TEMP, LEN_PRESENT_TEMP );
                joints[j].set_dxl_temp( present_current );
                joints[j].set_temprature( present_current );
            }
        }
        ++tempCount;
    }
}

void DXLPORT_CONTROL::readVel( ros::Time time, ros::Duration period )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    bool dxl_getdata_result = false;
    uint8_t dxl_error = 0;                          // Dynamixel error
   
    last_error = "";
    dxl_comm_result = readVelGroup->txRxPacket();
     if (dxl_comm_result != COMM_SUCCESS){
        last_error = packetHandler->getTxRxResult( dxl_comm_result );
        ++rx_err;
    }else{
        for( int j=0 ; j<joint_num ; ++j ){
            if( joints[j].get_ope_mode() != OPERATING_MODE_CURRENT ){
                continue;
            }
            uint8_t dxl_id = joints[j].get_dxl_id();
            dxl_getdata_result = readVelGroup->isAvailable( dxl_id, ADDR_PRESENT_VEL, LEN_PRESENT_VEL );
            if( !dxl_getdata_result ){
                ++rx_err;
                break;
            }else{
                int16_t present_velocity = readVelGroup->getData( dxl_id, ADDR_PRESENT_VEL, LEN_PRESENT_VEL );
                joints[j].set_velocity( DXL_VELOCITY2RAD_S(present_velocity) );
            }
        }
        ++tempCount;
    }
}

void DXLPORT_CONTROL::write( ros::Time time, ros::Duration period )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    double get_cmd;

    if( !port_stat){
        for( int j=0 ; j<joint_num ; ++j ){
            get_cmd = joints[j].get_command();
            joints[j].updt_d_command( get_cmd );
            joints[j].set_position( get_cmd );
        }
        return;
    }
    last_error = "";
    for( int j=0 ; j<joint_num ; ++j ){
        get_cmd = joints[j].get_command();
        if( joints[j].get_ope_mode() == OPERATING_MODE_CURRENT ){
            // Current control
            double work_cur = EFFORT2DXL_CURRENT( get_cmd, joints[j].get_eff_const() );
            joints[j].updt_d_command( 0.0 );

            uint16_t dxl_cur = (uint32_t)round( work_cur );
            uint8_t* goal_data = joints[j].get_dxl_goal_addr();
            goal_data[0] = (uint8_t)(dxl_cur&0x000000FF);
            goal_data[1] = (uint8_t)((dxl_cur&0x0000FF00)>>8);

            writeGoalGroup->changeParam( joints[j].get_dxl_id(), ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT, goal_data );
        }else{
            // Position control
            double work_pos = RAD2DXLPOS( get_cmd );
            joints[j].updt_d_command( get_cmd );
            work_pos += joints[j].get_center();          // ROS(-180 <=> +180) => DXL(0 <=> 4095)
            if( work_pos < DXL_MIN_LIMIT ){
                work_pos = DXL_MIN_LIMIT;
            }
            if( work_pos > DXL_MAX_LIMIT ){
                work_pos = DXL_MAX_LIMIT;
            }

            uint32_t dxl_pos = (uint32_t)round( work_pos );
            uint8_t* goal_data = joints[j].get_dxl_goal_addr();

            goal_data[0] = (uint8_t)(dxl_pos&0x000000FF);
            goal_data[1] = (uint8_t)((dxl_pos&0x0000FF00)>>8);
            goal_data[2] = (uint8_t)((dxl_pos&0x00FF0000)>>16);
            goal_data[3] = (uint8_t)((dxl_pos&0xFF000000)>>24);

            writeGoalGroup->changeParam( joints[j].get_dxl_id(), ADDR_GOAL_POSITION, LEN_GOAL_POSITION, goal_data );
        }
    }
    dxl_comm_result = writeGoalGroup->txPacket();
    if( dxl_comm_result != COMM_SUCCESS ){
        last_error = packetHandler->getTxRxResult( dxl_comm_result );
        ++tx_err;
    }else if( dxl_error != 0 ){
        last_error = packetHandler->getRxPacketError( dxl_error );
        ++tx_err;
    }
}

void DXLPORT_CONTROL::set_gain_all( uint16_t gain )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    if( !port_stat ){
        return;
    }
    last_error = "";
    for( int j=0 ; j<joint_num ; ++j ){
        if( joints[j].get_ope_mode() == OPERATING_MODE_CURRENT ){
            continue;
        }
        set_gain( joints[j].get_dxl_id(), gain );
    }
}

void DXLPORT_CONTROL::set_gain( uint8_t dxl_id, uint16_t gain )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    dxl_comm_result = packetHandler->write2ByteTxRx( portHandler, dxl_id, ADDR_POSITION_PGAIN, gain, &dxl_error );
    if( dxl_comm_result != COMM_SUCCESS ){
        last_error = packetHandler->getTxRxResult( dxl_comm_result );
        ++tx_err;
    }else if( dxl_error != 0 ){
        last_error = packetHandler->getRxPacketError( dxl_error );
        ++tx_err;
    }
}

void DXLPORT_CONTROL::set_goal_current_all( uint16_t current )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    if( !port_stat ){
        return;
    }
    last_error = "";
    for( int j=0 ; j<joint_num ; ++j ){
        if( joints[j].get_ope_mode() == OPERATING_MODE_CURRENT ){
            set_goal_current( joints[j].get_dxl_id(), current );
        }
    }
}


void DXLPORT_CONTROL::set_goal_current( uint8_t dxl_id, uint16_t current )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    dxl_comm_result = packetHandler->write2ByteTxRx( portHandler, dxl_id, ADDR_GOAL_CURRENT, current, &dxl_error );
    if( dxl_comm_result != COMM_SUCCESS ){
        last_error = packetHandler->getTxRxResult( dxl_comm_result );
        ++tx_err;
    }else if( dxl_error != 0 ){
        last_error = packetHandler->getRxPacketError( dxl_error );
        ++tx_err;
    }
}

bool DXLPORT_CONTROL::set_torque( uint8_t dxl_id, bool torque )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint32_t set_param = torque ? TORQUE_ENABLE:TORQUE_DISABLE;
    uint8_t dxl_error = 0;                          // Dynamixel error
    bool result = false;

    last_error = "";
    if( !port_stat ){
        return true;
    }
    dxl_comm_result = packetHandler->write1ByteTxRx( portHandler, dxl_id, ADDR_TORQUE_ENABLE, set_param, &dxl_error );
    if( dxl_comm_result != COMM_SUCCESS ){
        last_error = packetHandler->getTxRxResult( dxl_comm_result );
        ++tx_err;
    }else if( dxl_error != 0 ){
        last_error = packetHandler->getRxPacketError( dxl_error );
        ++tx_err;
    }else{
        result = true;
    }
    return result;
}
void DXLPORT_CONTROL::set_torque_all( bool torque )
{
    for( uint8_t j=0 ; j<joint_num; ++j ){
        if( set_torque( joints[j].get_dxl_id(), torque ) ){
            joints[j].set_torque( torque );
        }
    }
}

/* 起動時モーション */
void DXLPORT_CONTROL::startup_motion( void )
{
    uint8_t dxl_error = 0;                          // Dynamixel error
    ros::Rate rate( DXL_TORQUR_ON_STEP );
    int step_max = DXL_TORQUE_ON_STEP_MAX;
    ros::Time t = getTime();
    ros::Duration d = getDuration(t);
    int step_a, step_b;
    std::vector<ST_HOME_MOTION_DATA> home_motion_data;

    /* 開始位置取り込みと差分計算 */
    readPos( t, d );

    set_torque_all( true );                         // 全関節トルクON
    set_gain_all( DXL_DEFAULT_PGAIN );

    for( int j=0 ; j<joint_num ; ++j ){
        ST_HOME_MOTION_DATA motion_work;
        motion_work.home      = joints[j].get_home();
        motion_work.home_rad  = DXLPOS2RAD( motion_work.home ) - DXLPOS2RAD( joints[j].get_center() );
        motion_work.start_rad = joints[j].get_position();
        motion_work.start     = RAD2DXLPOS( motion_work.start_rad ) + joints[j].get_center();
        motion_work.step_rad  = 
            (motion_work.home > motion_work.start) ? ((motion_work.home_rad - motion_work.start_rad)/step_max)
                                                   : -((motion_work.start_rad - motion_work.home_rad)/step_max);
        if( joints[j].get_ope_mode() == OPERATING_MODE_CURRENT ){
            joints[j].set_command( 0.0 );
        }else{
            joints[j].set_command( joints[j].get_position() );
        }
        home_motion_data.push_back( motion_work );
    }
    write( t, d );

    /* ホームポジションへ移動する */
    for( int step=0 ; step<step_max ; ++step ){
        d = getDuration(t);
        t = getTime();
        readPos( t, d );
        if( !port_stat ){
            for( int j=0 ; j<joint_num ; ++j ){
                if( joints[j].get_ope_mode() == OPERATING_MODE_CURRENT ){
                    continue;
                }
                joints[j].set_command( DXLPOS2RAD( joints[j].get_home() ) - DXLPOS2RAD( joints[j].get_center() ) );
            }
            continue;
        }
        for( int j=0 ; j<joint_num ; ++j ){
            if( joints[j].get_ope_mode() == OPERATING_MODE_CURRENT ){
                continue;
            }
            joints[j].set_command( joints[j].get_command() + home_motion_data[j].step_rad );
        }
        write( t, d );
        rate.sleep();
    }
    for( int j=0 ; j<joint_num ; ++j ){
        if( joints[j].get_ope_mode() == OPERATING_MODE_CURRENT ){
            continue;
        }
        joints[j].set_command( home_motion_data[j].home_rad );
    }
    write( t, d );
    for( int j=0 ; j<joint_num ; ++j ){
        joints[j].updt_d_command( 0.0 );//差分の初期化
    }
}

/* セルフチェック */
bool DXLPORT_CONTROL::check_servo_param( uint8_t dxl_id, uint32_t test_addr, uint8_t equal, uint8_t& read_val )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    uint8_t read_data;
    bool result = false;

    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, dxl_id, test_addr, &read_data, &dxl_error);
    if( dxl_comm_result != COMM_SUCCESS ){
        last_error = packetHandler->getTxRxResult( dxl_comm_result );
        ++rx_err;
    }else if( dxl_error != 0 ){
        last_error = packetHandler->getRxPacketError( dxl_error );
        ++rx_err;
    }
    if( read_data == equal ){
        result = true;
    }
    read_val = read_data;
    return result;
}
bool DXLPORT_CONTROL::check_servo_param( uint8_t dxl_id, uint32_t test_addr, uint16_t equal, uint16_t& read_val )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    uint16_t read_data;
    bool result = false;

    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, dxl_id, test_addr, &read_data, &dxl_error);
    if( dxl_comm_result != COMM_SUCCESS ){
        last_error = packetHandler->getTxRxResult( dxl_comm_result );
        ++rx_err;
    }else if( dxl_error != 0 ){
        last_error = packetHandler->getRxPacketError( dxl_error );
        ++rx_err;
    }
    if( read_data == equal ){
        result = true;
    }
    read_val = read_data;
    return result;
}
bool DXLPORT_CONTROL::check_servo_param( uint8_t dxl_id, uint32_t test_addr, uint32_t equal, uint32_t& read_val )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    uint32_t read_data;
    bool result = false;

    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, test_addr, &read_data, &dxl_error);
    if( dxl_comm_result != COMM_SUCCESS ){
        last_error = packetHandler->getTxRxResult( dxl_comm_result );
        ++rx_err;
    }else if( dxl_error != 0 ){
        last_error = packetHandler->getRxPacketError( dxl_error );
        ++rx_err;
    }
    if( read_data == equal ){
        result = true;
    }
    read_val = read_data;
    return result;
}

void DXLPORT_CONTROL::init_joint_params( ST_JOINT_PARAM &param, int table_id, int value )
{
    switch( table_id ){
    case enTableId_ReturnDelay:
        param.return_delay_time = (uint8_t)value;
        break;
    case enTableId_DriveMode:
        param.drive_mode = (uint8_t)value;
        break;
    case enTableId_OpeMode:
        param.operation_mode = (uint8_t)value;
        break;
    case enTableId_HomingOffset:
        param.homing_offset = (int32_t)value;
        break;
    case enTableId_MovingThreshold:
        param.moving_threshold = (uint16_t)value;
        break;
    case enTableId_TempLimit:
        param.temprature_limit = (uint8_t)value;
        break;
    case enTableId_MaxVolLimit:
        param.max_vol_limit = (uint8_t)value;
        break;
    case enTableId_MinVolLimit:
        param.min_vol_limit = (uint8_t)value;
        break;
    case enTableId_CurrentLimit:
        param.current_limit = (uint16_t)value;
        break;
    case enTableId_TorqueEnable:
        param.torque_enable = (uint8_t)value;
        break;
    case enTableId_VelocityIGain:
        param.velocity_i_gain = (uint16_t)value;
        break;
    case enTableId_VelocityPGain:
        param.velocity_p_gain = (uint16_t)value;
        break;
    case enTableId_PositionDGain:
        param.position_d_gain = (uint16_t)value;
        break;
    case enTableId_PositionIGain:
        param.position_i_gain = (uint16_t)value;
        break;
    case enTableId_PositionPGain:
        param.position_p_gain = (uint16_t)value;
        break;
    case enTableId_GoalCurrent:
    case enTableId_GoalVelocity:
    case enTableId_GoalPosition:
    case enTableId_PresentCurrent:
    case enTableId_PresentVelocity:
    case enTableId_PresentPosition:
    case enTableId_PresentTemp:
    case enTableId_Shutdown:
    default:
        break;
    }
}

std::string DXLPORT_CONTROL::self_check( void )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    std::string res_str = "[DYNAMIXEL PARAMETER SELF CHECK]\n";
    uint8_t chk_8data, read_8data;
    uint16_t chk_16data, read_16data;
    uint32_t chk_32data, read_32data;

    if( !port_stat ){
        res_str = "SKIP SELF CHECK...";
        return res_str;
    }
    for( int i=0 ; i<(sizeof(RegTable)/sizeof(ST_DYNAMIXEL_REG_TABLE)) ; ++i ){
        bool check_result = true;
        bool read_result;
        if( RegTable[i].selfcheck ){
            res_str += RegTable[i].name + " test...\n";
        }
        switch( RegTable[i].length ){
        case REG_LENGTH_BYTE:
            chk_8data = (uint8_t)RegTable[i].init_value;
            if( RegTable[i].name == "OPERATION_MODE" ){
                chk_8data = joints[i].get_ope_mode();
            }
            for( int j=0 ; j<joint_num ; ++j ){
                read_result = check_servo_param( joints[j].get_dxl_id(), RegTable[i].address, chk_8data, read_8data );
                if( RegTable[i].selfcheck && !read_result ){
                    res_str += " ID: " + std::to_string(joints[j].get_dxl_id()) + " check NG\n";
                    check_result = false;
                }
                ST_JOINT_PARAM work = joints[j].get_joint_param();
                init_joint_params( work, i, read_8data );
                joints[j].set_joint_param( work );
            }
            break;
        case REG_LENGTH_WORD:
            chk_16data = (uint16_t)RegTable[i].init_value;
            for( int j=0 ; j<joint_num ; ++j ){
                read_result = check_servo_param( joints[j].get_dxl_id(), RegTable[i].address, chk_16data, read_16data );
                if( RegTable[i].selfcheck && !read_result ){
                    res_str += " ID: " + std::to_string(joints[j].get_dxl_id()) + " check NG\n";
                    check_result = false;
                }
                ST_JOINT_PARAM work = joints[j].get_joint_param();
                init_joint_params( work, i, read_16data );
                joints[j].set_joint_param( work );
            }
            break;
        case REG_LENGTH_DWORD:
            chk_32data = (uint32_t)RegTable[i].init_value;
            for( int j=0 ; j<joint_num ; ++j ){
                read_result = check_servo_param( joints[j].get_dxl_id(), RegTable[i].address, chk_32data, read_32data );
                if( RegTable[i].selfcheck && !read_result ){
                    res_str += " ID: " + std::to_string(joints[j].get_dxl_id()) + " check NG\n";
                    check_result = false;
                }
                ST_JOINT_PARAM work = joints[j].get_joint_param();
                init_joint_params( work, i, read_32data );
                joints[j].set_joint_param( work );
            }
            break;
        }
        if( RegTable[i].selfcheck ){
            if( check_result ){
                res_str += " CHECK OK\n";
            }else{
                res_str += " CHECK NG!\n";
            }
        }
    }
    return res_str;
}

void DXLPORT_CONTROL::effort_limitter( void )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    double now_eff, max_eff;
    uint16_t set_pgain = DXL_DEFAULT_PGAIN;
    uint16_t get_pgain;
    double eff_coeff = 0.95;

    last_error = "";
    if( !port_stat ){
        return;
    }
    for( int j=0 ; j<joint_num; ++j ){
        now_eff = fabs( joints[j].get_effort() );
        max_eff = joints[j].get_max_effort();
        if( now_eff > max_eff ){
            joints[j].inc_eff_over();
            if( joints[j].get_eff_over_cnt() >= EFFORT_LIMITING_CNT ){
                // 設計回数以上連続して最大トルクを上回った
                set_pgain = DXL_DEFAULT_PGAIN;
                if( joints[j].is_effort_limiting() ){
                    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, joints[j].get_dxl_id(), ADDR_POSITION_PGAIN, &get_pgain, &dxl_error);
                    if( dxl_comm_result != COMM_SUCCESS ){
                        last_error = packetHandler->getTxRxResult( dxl_comm_result );
                        ++rx_err;
                    }else if( dxl_error != 0 ){
                        last_error = packetHandler->getRxPacketError( dxl_error );
                        ++rx_err;
                    }else{
                        set_pgain = get_pgain;
                    }
                }
                set_pgain = (uint16_t)(set_pgain * ((max_eff*eff_coeff) / now_eff));
                set_gain( joints[j].get_dxl_id(), set_pgain );
                joints[j].set_eff_limiting( true );
            }
        }else{
            // 制限中で電流値が下回った
            if( joints[j].is_effort_limiting() ){
                dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, joints[j].get_dxl_id(), ADDR_POSITION_PGAIN, &get_pgain, &dxl_error);
                if( dxl_comm_result != COMM_SUCCESS ){
                    last_error = packetHandler->getTxRxResult( dxl_comm_result );
                    ++rx_err;
                }else if( dxl_error != 0 ){
                    last_error = packetHandler->getRxPacketError( dxl_error );
                    ++rx_err;
                }else if( get_pgain >= DXL_DEFAULT_PGAIN ){
                    joints[j].clear_eff_over();
                    joints[j].set_eff_limiting( false );
                }else{
                    set_gain( joints[j].get_dxl_id(), (get_pgain+2) );
                    joints[j].set_eff_limiting( true );
                }
            }
        }

    }
}



void DXLPORT_CONTROL::set_param_delay_time( uint8_t dxl_id, int val )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t set_param = (uint8_t)val;
    uint8_t dxl_error = 0;                          // Dynamixel error
    bool result = false;

    last_error = "";
    if( !port_stat ){
        return;
    }
    for( int j=0 ; j<joint_num; ++j ){
        if( dxl_id == joints[j].get_dxl_id() ){
            ST_JOINT_PARAM new_param = joints[j].get_joint_param();
            if( new_param.return_delay_time != set_param ){
                dxl_comm_result = packetHandler->write1ByteTxRx( portHandler, dxl_id, ADDR_RETURN_DELAY, set_param, &dxl_error );
                if( dxl_comm_result != COMM_SUCCESS ){
                    last_error = packetHandler->getTxRxResult( dxl_comm_result );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    last_error = packetHandler->getRxPacketError( dxl_error );
                    ++tx_err;
                }
            }
            new_param.return_delay_time = set_param;
            joints[j].set_joint_param( new_param );
            break;
        }
    }
}
void DXLPORT_CONTROL::set_param_drive_mode( uint8_t dxl_id, int val )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t set_param = (uint8_t)val;
    uint8_t dxl_error = 0;                          // Dynamixel error
    bool result = false;

    last_error = "";
    if( !port_stat ){
        return;
    }
    for( int j=0 ; j<joint_num; ++j ){
        if( dxl_id == joints[j].get_dxl_id() ){
            ST_JOINT_PARAM new_param = joints[j].get_joint_param();
            if( new_param.drive_mode != set_param ){
                dxl_comm_result = packetHandler->write1ByteTxRx( portHandler, dxl_id, ADDR_DRIVE_MODE, set_param, &dxl_error );
                if( dxl_comm_result != COMM_SUCCESS ){
                    last_error = packetHandler->getTxRxResult( dxl_comm_result );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    last_error = packetHandler->getRxPacketError( dxl_error );
                    ++tx_err;
                }
            }
            new_param.drive_mode = set_param;
            joints[j].set_joint_param( new_param );
            break;
        }
    }
}
void DXLPORT_CONTROL::set_param_ope_mode( uint8_t dxl_id, int val )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t set_param = (uint8_t)val;
    uint8_t dxl_error = 0;                          // Dynamixel error
    bool result = false;

    last_error = "";
    if( !port_stat ){
        return;
    }
    for( int j=0 ; j<joint_num; ++j ){
        if( dxl_id == joints[j].get_dxl_id() ){
            ST_JOINT_PARAM new_param = joints[j].get_joint_param();
            if( new_param.operation_mode != set_param ){
                dxl_comm_result = packetHandler->write1ByteTxRx( portHandler, dxl_id, ADDR_OPE_MODE, set_param, &dxl_error );
                if( dxl_comm_result != COMM_SUCCESS ){
                    last_error = packetHandler->getTxRxResult( dxl_comm_result );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    last_error = packetHandler->getRxPacketError( dxl_error );
                    ++tx_err;
                }
            }
            new_param.operation_mode = set_param;
            joints[j].set_joint_param( new_param );
            break;
        }
    }
}
void DXLPORT_CONTROL::set_param_home_offset( uint8_t dxl_id, int val )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint32_t set_param = (uint32_t)val;
    uint8_t dxl_error = 0;                          // Dynamixel error
    bool result = false;

    last_error = "";
    if( !port_stat ){
        return;
    }
    for( int j=0 ; j<joint_num; ++j ){
        if( dxl_id == joints[j].get_dxl_id() ){
            ST_JOINT_PARAM new_param = joints[j].get_joint_param();
            if( new_param.homing_offset != set_param ){
                dxl_comm_result = packetHandler->write4ByteTxRx( portHandler, dxl_id, ADDR_MOVING_THRESHOLD, set_param, &dxl_error );
                if( dxl_comm_result != COMM_SUCCESS ){
                    last_error = packetHandler->getTxRxResult( dxl_comm_result );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    last_error = packetHandler->getRxPacketError( dxl_error );
                    ++tx_err;
                }
            }
            new_param.homing_offset = (int32_t)set_param;
            joints[j].set_joint_param( new_param );
            break;
        }
    }
}
void DXLPORT_CONTROL::set_param_moving_threshold( uint8_t dxl_id, int val )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint32_t set_param = (uint32_t)val;
    uint8_t dxl_error = 0;                          // Dynamixel error
    bool result = false;

    last_error = "";
    if( !port_stat ){
        return;
    }
    for( int j=0 ; j<joint_num; ++j ){
        if( dxl_id == joints[j].get_dxl_id() ){
            ST_JOINT_PARAM new_param = joints[j].get_joint_param();
            if( new_param.moving_threshold != set_param ){
                dxl_comm_result = packetHandler->write4ByteTxRx( portHandler, dxl_id, ADDR_MOVING_THRESHOLD, set_param, &dxl_error );
                if( dxl_comm_result != COMM_SUCCESS ){
                    last_error = packetHandler->getTxRxResult( dxl_comm_result );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    last_error = packetHandler->getRxPacketError( dxl_error );
                    ++tx_err;
                }
            }
            new_param.moving_threshold = set_param;
            joints[j].set_joint_param( new_param );
            break;
        }
    }
}
void DXLPORT_CONTROL::set_param_temp_limit( uint8_t dxl_id, int val )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t set_param = (uint8_t)val;
    uint8_t dxl_error = 0;                          // Dynamixel error
    bool result = false;

    last_error = "";
    if( !port_stat ){
        return;
    }
    for( int j=0 ; j<joint_num; ++j ){
        if( dxl_id == joints[j].get_dxl_id() ){
            ST_JOINT_PARAM new_param = joints[j].get_joint_param();
            if( new_param.temprature_limit != set_param ){
                dxl_comm_result = packetHandler->write1ByteTxRx( portHandler, dxl_id, ADDR_TEMPRATURE_LIMIT, set_param, &dxl_error );
                if( dxl_comm_result != COMM_SUCCESS ){
                    last_error = packetHandler->getTxRxResult( dxl_comm_result );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    last_error = packetHandler->getRxPacketError( dxl_error );
                    ++tx_err;
                }
            }
            new_param.temprature_limit = set_param;
            joints[j].set_joint_param( new_param );
            break;
        }
    }
}
void DXLPORT_CONTROL::set_param_vol_limit( uint8_t dxl_id, int max, int min )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint16_t set_max_param = (uint32_t)max;
    uint16_t set_min_param = (uint32_t)min;
    uint8_t dxl_error = 0;                          // Dynamixel error
    bool result = false;

    last_error = "";
    if( !port_stat ){
        return;
    }
    for( int j=0 ; j<joint_num; ++j ){
        if( dxl_id == joints[j].get_dxl_id() ){
            ST_JOINT_PARAM new_param = joints[j].get_joint_param();
            if( new_param.max_vol_limit != set_max_param ){
                dxl_comm_result = packetHandler->write2ByteTxRx( portHandler, dxl_id, ADDR_MAX_VOL_LIMIT, set_max_param, &dxl_error );
                if( dxl_comm_result != COMM_SUCCESS ){
                    last_error = packetHandler->getTxRxResult( dxl_comm_result );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    last_error = packetHandler->getRxPacketError( dxl_error );
                    ++tx_err;
                }
            }
            if( new_param.min_vol_limit != set_min_param ){
                dxl_comm_result = packetHandler->write2ByteTxRx( portHandler, dxl_id, ADDR_MIN_VOL_LIMIT, set_min_param, &dxl_error );
                if( dxl_comm_result != COMM_SUCCESS ){
                    last_error = packetHandler->getTxRxResult( dxl_comm_result );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    last_error = packetHandler->getRxPacketError( dxl_error );
                    ++tx_err;
                }
            }
            new_param.max_vol_limit = set_max_param;
            new_param.min_vol_limit = set_min_param;
            joints[j].set_joint_param( new_param );
            break;
        }
    }
}
void DXLPORT_CONTROL::set_param_current_limit( uint8_t dxl_id, int val )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint16_t set_param = (uint16_t)val;
    uint8_t dxl_error = 0;                          // Dynamixel error
    bool result = false;

    last_error = "";
    if( !port_stat ){
        return;
    }
    for( int j=0 ; j<joint_num; ++j ){
        if( dxl_id == joints[j].get_dxl_id() ){
            ST_JOINT_PARAM new_param = joints[j].get_joint_param();
            if( new_param.current_limit != set_param ){
                dxl_comm_result = packetHandler->write2ByteTxRx( portHandler, dxl_id, ADDR_CURRENT_LIMIT, set_param, &dxl_error );
                if( dxl_comm_result != COMM_SUCCESS ){
                    last_error = packetHandler->getTxRxResult( dxl_comm_result );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    last_error = packetHandler->getRxPacketError( dxl_error );
                    ++tx_err;
                }
            }
            new_param.current_limit = set_param;
            joints[j].set_joint_param( new_param );
            break;
        }
    }
}
void DXLPORT_CONTROL::set_param_vel_gain( uint8_t dxl_id, int p, int i )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint16_t set_p_param = (uint16_t)p;
    uint16_t set_i_param = (uint16_t)i;
    uint8_t dxl_error = 0;                          // Dynamixel error
    bool result = false;

    last_error = "";
    if( !port_stat ){
        return;
    }
    for( int j=0 ; j<joint_num; ++j ){
        if( dxl_id == joints[j].get_dxl_id() ){
            ST_JOINT_PARAM new_param = joints[j].get_joint_param();
            if( (new_param.velocity_p_gain != set_p_param) ){
                dxl_comm_result = packetHandler->write2ByteTxRx( portHandler, dxl_id, ADDR_VELOCITY_PGAIN, set_p_param, &dxl_error );
                if( dxl_comm_result != COMM_SUCCESS ){
                    last_error = packetHandler->getTxRxResult( dxl_comm_result );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    last_error = packetHandler->getRxPacketError( dxl_error );
                    ++tx_err;
                }
            }
            if( (new_param.velocity_i_gain != set_i_param) ){
                dxl_comm_result = packetHandler->write2ByteTxRx( portHandler, dxl_id, ADDR_VELOCITY_IGAIN, set_i_param, &dxl_error );
                if( dxl_comm_result != COMM_SUCCESS ){
                    last_error = packetHandler->getTxRxResult( dxl_comm_result );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    last_error = packetHandler->getRxPacketError( dxl_error );
                    ++tx_err;
                }
            }
            new_param.velocity_p_gain = set_p_param;
            new_param.velocity_i_gain = set_i_param;
            joints[j].set_joint_param( new_param );
            break;
        }
    }
}
void DXLPORT_CONTROL::set_param_pos_gain( uint8_t dxl_id, int p, int i, int d )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint16_t set_p_param = (uint16_t)p;
    uint16_t set_i_param = (uint16_t)i;
    uint16_t set_d_param = (uint16_t)d;
    uint8_t dxl_error = 0;                          // Dynamixel error
    bool result = false;

    last_error = "";
    if( !port_stat ){
        return;
    }
    for( int j=0 ; j<joint_num; ++j ){
        if( dxl_id == joints[j].get_dxl_id() ){
            ST_JOINT_PARAM new_param = joints[j].get_joint_param();
            if( new_param.position_p_gain != set_p_param ){
                dxl_comm_result = packetHandler->write2ByteTxRx( portHandler, dxl_id, ADDR_POSITION_PGAIN, set_p_param, &dxl_error );
                if( dxl_comm_result != COMM_SUCCESS ){
                    last_error = packetHandler->getTxRxResult( dxl_comm_result );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    last_error = packetHandler->getRxPacketError( dxl_error );
                    ++tx_err;
                }
            }
            if( new_param.position_i_gain != set_i_param ){
                dxl_comm_result = packetHandler->write2ByteTxRx( portHandler, dxl_id, ADDR_POSITION_IGAIN, set_i_param, &dxl_error );
                if( dxl_comm_result != COMM_SUCCESS ){
                    last_error = packetHandler->getTxRxResult( dxl_comm_result );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    last_error = packetHandler->getRxPacketError( dxl_error );
                    ++tx_err;
                }
            }
            if( new_param.position_d_gain != set_d_param ){
                dxl_comm_result = packetHandler->write2ByteTxRx( portHandler, dxl_id, ADDR_POSITION_DGAIN, set_d_param, &dxl_error );
                if( dxl_comm_result != COMM_SUCCESS ){
                    last_error = packetHandler->getTxRxResult( dxl_comm_result );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    last_error = packetHandler->getRxPacketError( dxl_error );
                    ++tx_err;
                }
            }
            new_param.position_p_gain = set_p_param;
            new_param.position_i_gain = set_i_param;
            new_param.position_d_gain = set_d_param;
            joints[j].set_joint_param( new_param );
            break;
        }
    }
}
