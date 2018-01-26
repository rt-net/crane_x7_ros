#!/usr/bin/env python
# license removed for brevity
import rospy
import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

#os.sys.path.append('dynamixel_functions_py')             # Path setting
import dynamixel_functions as dynamixel                    # Uses Dynamixel SDK library
import time
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

ADDR_PRO_TORQUE_ENABLE      = 64
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132
ADDR_PRO_P_TORQUE           = 84
PROTOCOL_VERSION2           = 2
DXL2_ID                     = 2                             # Dynamixel ID: 2
BAUDRATE                    = 1024000
DEVICENAME                  = "/dev/ttyUSB0".encode('utf-8')# Check which port is being used on your controller
TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque
DXL2_MINIMUM_POSITION_VALUE = 0
DXL2_MAXIMUM_POSITION_VALUE = 4095
DXL2_MOVING_STATUS_THRESHOLD = 20
ESC_ASCII_VALUE             = 0x1b
COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

port_num = dynamixel.portHandler(DEVICENAME)
dxl_comm_result = COMM_TX_FAIL                              # Communication result
dxl_error = 0                                               # Dynamixel error

servo_ids = [2, 3, 4, 5, 6, 7, 8]
read_positions = [0] * 7
read_pos_raw = [0] * 7

INIT_CENTER_ENABLE          = 1
INIT_CENTER_DISABLE         = 0
INIT_POSITION_CENTER        = INIT_CENTER_ENABLE           # Initialize position to center

def comsetup():
    dynamixel.packetHandler()
    # Open port
    if dynamixel.openPort(port_num):
        print("Succeeded to open the port!")
    else:
        print("Failed to open the port!")
        print("Press any key to terminate...")
        getch()
        quit()
    # Set port baudrate
    if dynamixel.setBaudRate(port_num, BAUDRATE):
        print("Succeeded to change the baudrate!")
    else:
        print("Failed to change the baudrate!")
        print("Press any key to terminate...")
        getch()
        quit()

# Dynamixel setup func
def dxl_setup():
    setindex = 0
    for index in servo_ids:
        if INIT_POSITION_CENTER == INIT_CENTER_ENABLE:
            # Torque ON & Move to center
            dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION2, index, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
#        dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION2, index, ADDR_PRO_GOAL_POSITION, 2048)
        dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION2)
        dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION2)
        if dxl_comm_result != COMM_SUCCESS:
            print(dynamixel.getTxRxResult(PROTOCOL_VERSION2, dxl_comm_result))
            # Disable error id
            servo_ids[setindex] = 0
        elif dxl_error != 0:
            print(dynamixel.getRxPacketError(PROTOCOL_VERSION2, dxl_error))
            # Disable error id
            servo_ids[setindex] = 0
        else:
            read_pos_raw[setindex] = dxl2_present_position = dynamixel.read4ByteTxRx(port_num, PROTOCOL_VERSION2, index, ADDR_PRO_PRESENT_POSITION)
            print("[Servo#%d] has been successfully connected" % (index))
        # Wait for servo centering
        if INIT_POSITION_CENTER == INIT_CENTER_ENABLE:
            time.sleep(1)
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION2, index, ADDR_PRO_P_TORQUE, 10)
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION2, index, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
        setindex += 1

# Read servo position
def read_motors():
    setindex = 0
    coef = (360.0 / 4095.0)
    for index in servo_ids:
        # Get position by Dynamixel API
        dxl2_present_position = dynamixel.read4ByteTxRx(port_num, PROTOCOL_VERSION2, index, ADDR_PRO_PRESENT_POSITION)
        # Correction of rotation amount
        if (index == 2) or (index == 4) or (index == 6) or (index == 8):
            read_positions[setindex] = math.radians(coef * (dxl2_present_position)) - math.radians(coef * 2048.0)
        else:
            read_positions[setindex] = math.radians(coef * (4095 - dxl2_present_position)) - math.radians(coef * 2048.0)
        setindex += 1
        # [Debug]
        # if index == 2:
        #    print("[Servo#%d] present_pos=#%d" % (index, dxl2_present_position))

def talker():
    pub = rospy.Publisher('joint_states_source', JointState, queue_size=1000)
    rospy.init_node('joint_state_source')
    rate = rospy.Rate(50) # 1khz
    while not rospy.is_shutdown():
        read_motors()
        master_data = JointState()
        master_data.header = Header()
        master_data.header.stamp = rospy.Time.now()
        master_data.name = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5', 'arm_joint6', 'arm_joint7']
        master_data.position = read_positions
        master_data.velocity = []
        master_data.effort = []
        pub.publish(master_data)
        # [Debug]
        print(read_positions)
        rate.sleep()

if __name__ == '__main__':
    try:
        comsetup()
        dxl_setup()
        talker()
    except rospy.ROSInterruptException:
        pass
