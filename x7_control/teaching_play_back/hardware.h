// *********    Example teaching play back    *********
//
//
#ifndef _HARDWARE_H_
#define _HARDWARE_H_


#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "dynamixel_sdk.h"									// Dynamixel SDK libraryのインクルード

// Control table address									// Dynamixel コントロールテーブル参照のアドレス値
#define ADDR_PRO_TORQUE_ENABLE			64
#define ADDR_PRO_GOAL_POSITION			116
#define ADDR_PRO_PRESENT_POSITION		132
#define ADDR_PRO_PROFILE_VELOCITY		112

// Data Byte Length											// Dynamixel アドレス毎のデータ長
#define LEN_PRO_LED_RED					1
#define LEN_PRO_GOAL_POSITION			4
#define LEN_PRO_PRESENT_POSITION		4
#define LEN_PRO_PROFILE_VELOCITY		4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // Dynamixel通信プロトコル

// Default setting
#define BAUDRATE                        3000000				// サーボの通信レート
#define DEVICENAME                      "/dev/ttyUSB0"		// PCに接続しているポート
// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // トルクON用のフラグ
#define TORQUE_DISABLE                  0                   // トルクOFF用のフラグ

#define DXL_MOVING_STATUS_THRESHOLD		20					// Dynamixel moving status threshold
#define DXL_CENTER_POSITION_VALUE		2048				// Offset(出力軸の中心角度)　(value)
#define DXL_PROFILE_VELOCITY			30					// サーボモータの動作速度　(rpm)

#define JOINT_NUM						8					// サーボモータの個数(自由度は7)

static int		ID[JOINT_NUM]			=	{	2,		3,		4,		5,		6,		7,		8,		9};			// サーボモータのID
static double	JOINT_MIN[JOINT_NUM]	=	{	262,	1024,	262,	228,	262,	1024,	148,	1991};		// サーボモータの最小動作角(value)
static double	JOINT_MAX[JOINT_NUM]	=	{	3834,	3072,	3834,	2048,	3834,	3072,	3948,	3072};		// サーボモータの最大動作角(value)
static double	goal_pose[JOINT_NUM]	=	{	0,		0,		0,		0,		0,		0,		0,		0};			// Move_goal_position関数の引数(deg)

/**
 * @brief CRANE-x7 動作用クラス
 */
class CR7 {
	public:
	// Initialize PortHandler instance
	// Set the port path
	// Get methods and members of PortHandlerLinux or PortHandlerWindows
	// 使用するポートの各種設定(Dynamixel SDK)
	dynamixel::PortHandler *portHandler;

	// Initialize PacketHandler instance
	// Set the protocol version
	// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
	// サーボモータと通信するためのパケットの設定(Dynamixel SDK)
	dynamixel::PacketHandler *packetHandler;

	// Initialize GroupBulkWrite instance
	// 複数サーボの書き込み用関数の呼び出し(Dynamixel SDK)
	dynamixel::GroupBulkWrite *groupBulkWrite;

	// Initialize GroupBulkRead instance
	// 複数サーボの読み込み用関数の呼び出し(Dynamixel SDK)
	dynamixel::GroupBulkRead *groupBulkRead;

	CR7();		//コンストラクタ

	//各種エラー出力用変数
	int dxl_comm_result;             // Communication result
	bool dxl_addparam_result;               // addParam result
	bool dxl_getdata_result;                // GetParam result
	uint8_t dxl_error;                          // Dynamixel error

	uint8_t param_goal_position[4];		//通信パケット用に変換したgoal_positionの変数
	uint8_t param_value[4];				//通信パケット用に変換したvalueの変数
	
	int32_t dxl_present_position;		//サーボの現在位置取得用の変数
	
	double joint_pose[JOINT_NUM];
	
	//　play back用テキストデータの定義
	FILE *fp;
	const char *fname = "data.txt";

	bool Open_port();							//通信ポートを開く
	bool Set_port_baudrate();					//通信レートの設定
	void Enable_Dynamixel_Torque();				//全サーボのトルクをON
	void Disable_Dynamixel_Torque();			//全サーボのトルクをOFF
	void Move_Goal_Position(double *goal_pose);	//設定されているgoal positionに移動
	void Move_Offset_Position();				//初期位置(出力軸中心角)への移動
	bool Teaching_Play_Frame();					//play back用のテキストデータを作成
	bool Play_Back();							//テキストデータを元に動作
	void Close_port();							//通信ポートを閉じる
};

#endif
