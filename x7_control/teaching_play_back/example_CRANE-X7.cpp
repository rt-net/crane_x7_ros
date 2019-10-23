// *********    Example teaching play back    *********
//
//

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

double deg2value( double deg ){ return (deg + 180) * 4096 / 360; }		// degをvalueに変換(サーボ出力軸の基準を0)(value)
double value2deg( double value){ return value * 360 / 4096 - 180; }		// valueをdegに変換(サーボ出力軸の基準を0)(deg)

/**
 * @fn		int getch()
 * @brief	キーボード入力用の関数(Dynamixel SDK sample)
 * @return	getchar() キー入力
*/
int getch()		
{
	struct termios oldt, newt;
	int ch;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= (ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	ch = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	return ch;
}

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

/**
 * @brief コンストラクタ
 */
CR7::CR7(){
	portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
	packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
	groupBulkWrite = new dynamixel::GroupBulkWrite( portHandler, packetHandler);
	groupBulkRead = new dynamixel::GroupBulkRead( portHandler, packetHandler);

	dxl_comm_result = COMM_TX_FAIL;             // Communication result
	dxl_addparam_result = false;               // addParam result
	dxl_getdata_result = false;                // GetParam result
	dxl_error = 0;                          // Dynamixel error

	dxl_present_position = 0;
}

/**
 * @fn		bool Open_port()
 * @brief	設定した DEVICENAMEのポートを開く
 * @return	bool 1:成功　0:失敗
 */
bool CR7::Open_port(){
	//Open port
	if (portHandler->openPort())
	{
		printf("Succeeded to open the port!¥n");
		return 1;
	}
	else
	{
		printf("Failed to open the port!¥n");
		printf("Press any key to terminate...¥n");
		getch();
		return 0;
	}
}

/**
 * @fn		bool Set_port_baudrate()
 * @brief	設定した BAUDRATEで通信の設定をする
 * @return	bool 1:成功 0:失敗
 */
bool CR7::Set_port_baudrate(){
	// Set port baudrate
	if (portHandler->setBaudRate(BAUDRATE))
	{
		printf("Succeeded to change the baudrate!¥n");
		return 1;
	}
	else
	{
		printf("Failed to change the baudrate!¥n");
		printf("Press any key to terminate...¥n");
		getch();
		return 0;
	}
}

/**
 * @fn		void Enable_Dynamixel_Torque()
 * @brief	全サーボのトルクをONにする
 *			全サーボの回転速度をDXL_PROFILE_VELOCITYに設定
 */
void CR7::Enable_Dynamixel_Torque(){
	// Enable Dynamixel Torque
	for(int i=0;i<JOINT_NUM;i++){
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, ID[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);			//該当IDのサーボのトルク管理のアドレスにONを書き込む
		param_value[0] = DXL_LOBYTE(DXL_LOWORD(DXL_PROFILE_VELOCITY));																	//設定した回転速度を通信パケット用にデータを分ける
		param_value[1] = DXL_HIBYTE(DXL_LOWORD(DXL_PROFILE_VELOCITY));
		param_value[2] = DXL_LOBYTE(DXL_HIWORD(DXL_PROFILE_VELOCITY));
		param_value[3] = DXL_HIBYTE(DXL_HIWORD(DXL_PROFILE_VELOCITY));
		dxl_addparam_result = groupBulkWrite->addParam(ID[i], ADDR_PRO_PROFILE_VELOCITY, LEN_PRO_PROFILE_VELOCITY, param_value);		//書き込み用のパケットに作成したデータを追加
		printf("[ ID : %d : ", ID[i]);																									//各サーボが送信したパケットどうりに動いているか確認
		if (dxl_comm_result != COMM_SUCCESS) printf(" result : %s", packetHandler->getTxRxResult(dxl_comm_result));						//正しいコマンドが送信されているか確認
		else if (dxl_error != 0) printf(" error : %s", packetHandler->getRxPacketError(dxl_error));										//エラーが発生した場合のコメント
		else printf(" successfully connected ");																						//正常にサーボがトルクON
		printf(" ]¥n");
	}

	// Bulkwrite goal position 
	dxl_comm_result = groupBulkWrite->txPacket();
	if (dxl_comm_result != COMM_SUCCESS) printf("%s¥n", packetHandler->getTxRxResult(dxl_comm_result));

	// Clear bulkwrite parameter storage
	groupBulkWrite->clearParam();
}

/**
 * @fn		void Disable_Dynamixel_Torque()
 * @brief	全サーボのトルクをOFFにする
 */
void CR7::Disable_Dynamixel_Torque(){
	// Disable Dynamixel Torque
	for(int i=0;i<JOINT_NUM;i++)
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, ID[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);		//該当IDのサーボのトルク管理のアドレスにOFFを書き込む

	// Bulkwrite goal position 
	dxl_comm_result = groupBulkWrite->txPacket();
	if (dxl_comm_result != COMM_SUCCESS) printf("%s¥n", packetHandler->getTxRxResult(dxl_comm_result));

	// Clear bulkwrite parameter storage
	groupBulkWrite->clearParam();
}

/**
 * @fn		void Move_Goal_Position()
 * @brief	設定してあるGoal Positionへ移動
 * @param	goal_pose[8](static double goal_pose[8]) サーボの個数分のデータ(deg)
 */
void CR7::Move_Goal_Position( double *goal_pose){
	//Move target goal position
	for(int i=0;i<JOINT_NUM;i++){
		printf("[ ID[%d] : %lf ]", ID[i], goal_pose[i]);																				//指定したサーボとデータの確認
		if((JOINT_MIN[i] > deg2value(goal_pose[i])) || (JOINT_MAX[i] < deg2value(goal_pose[i]))){										//動作角度外の角度が入力された場合
			printf("over range!¥n");																										//入力データがdegなのでvalueに変換(中心が0になる)
			break;
		}

		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(deg2value(goal_pose[i])));														//通信用にデータを分ける
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(deg2value(goal_pose[i])));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(deg2value(goal_pose[i])));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(deg2value(goal_pose[i])));

		dxl_addparam_result = groupBulkWrite->addParam(ID[i], ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, param_goal_position);		//書き込み用のパケットに追加
		if(dxl_addparam_result != true) printf("goal pose error!¥n");

	}printf("¥n");
	
	// Bulkwrite goal position 
	dxl_comm_result = groupBulkWrite->txPacket();
	if (dxl_comm_result != COMM_SUCCESS) printf("%s¥n", packetHandler->getTxRxResult(dxl_comm_result));

	// Clear bulkwrite parameter storage
	groupBulkWrite->clearParam();
}

/**
 * @fn		void Move_Offset_Position()
 * @brief	サーボの初期位置(出力軸中心角度)へ移動
 */
void CR7::Move_Offset_Position(){
	// Move offset position
	for(int i=0;i<JOINT_NUM;i++){ 
		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(DXL_CENTER_POSITION_VALUE));														//通信用にデータを分ける
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(DXL_CENTER_POSITION_VALUE));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(DXL_CENTER_POSITION_VALUE));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(DXL_CENTER_POSITION_VALUE));

		dxl_addparam_result = groupBulkWrite->addParam(ID[i], ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, param_goal_position);		//書き込み用のパケットに追加
	}
	if(dxl_addparam_result != true) printf("offset error!¥n");

	// Bulkwrite goal position 
	dxl_comm_result = groupBulkWrite->txPacket();
	if (dxl_comm_result != COMM_SUCCESS) printf("%s¥n", packetHandler->getTxRxResult(dxl_comm_result));

	// Clear bulkwrite parameter storage
	groupBulkWrite->clearParam();
}

/**
 * @fn		bool Teaching_Play_Frame()
 * @brief	Play back用のテキストデータの作成
 * @return	bool 1:成功 -1:失敗(テキストデータを開けない)
 */
bool CR7::Teaching_Play_Frame(){
	//Teaching play back motion frame
	fp = fopen( fname, "w");																											//モード:w(書き込み専用)でファイルを開く
	if(fp == NULL){
		printf("can not open¥n");
		return -1;
	}
	while(1){																															//データ取得の開始
		printf("teaching play frame( type q end teaching )¥n");
		char ch2 = getch();																												//　"q"　以外のキーが押された時の角度をテキストデータへ保存
		if(ch2 == 'q')break;
		for(int i=0;i<JOINT_NUM;i++){
			dxl_addparam_result = groupBulkRead->addParam(ID[i], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);					//読み込みのデータを設定(現在角度)
			//if( dxl_addparam_result != true) printf(" ID[%d] : groupBulkRead addParam failed¥n", ID[i]);

			//Bulkread present position 
			dxl_comm_result = groupBulkRead->txRxPacket();																				//返信データの読み込み
			if(dxl_comm_result != COMM_SUCCESS) printf(" discommect ¥n");

			//Check if groupbulkread data of Dynamixel is available																		//返信データが利用できるか確認
			dxl_getdata_result = groupBulkRead->isAvailable(ID[i], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
			if(dxl_getdata_result != true) printf(" ID[%d] : groupBulkRead getdata failed¥n", ID[i]);

			dxl_present_position = groupBulkRead->getData(ID[i], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);					//返信データから指定のデータを読む
			printf("[ ID[%d] : %lf ]", ID[i], value2deg(dxl_present_position));
			fprintf(fp, "%d,", dxl_present_position);
		}printf("¥n");
		fprintf(fp,"¥n");																												//サーボ数分を1つとして改行
	}
	fclose(fp);																															//ファイルを閉じる
	return 1;
}

/**
 * @fn		bool Play_Back()
 * @brief	テキストデータの値を再生
 * @return	bool 1:成功 -1:失敗(テキストデータを開けない)
 */
bool CR7::Play_Back(){
	double pose[8];
	//Play back data position
	fp = fopen( fname, "r");																											//モード:r(読み込み専用)でファイルを開く
	if(fp == NULL){
		printf("can not open¥n");
		return -1;
	}
	while(	(fscanf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,", &pose[0],&pose[1],&pose[2],&pose[3],&pose[4],&pose[5],&pose[6],&pose[7])) != EOF){
		for(int i=0;i<JOINT_NUM;i++){																									//EOFまで1行ずつデータを読み込む
			printf("[ ID[%d] : %lf ]", ID[i], value2deg(pose[i]));
			if((JOINT_MIN[i] > pose[i]) || (JOINT_MAX[i] < pose[i])){															//動作角度外の角度が入力された場合
				printf("over range!¥n");
				break;
			}

			param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(pose[i]));																//通信用データに分ける
			param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(pose[i]));
			param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(pose[i]));
			param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(pose[i]));

			dxl_addparam_result = groupBulkWrite->addParam(ID[i], ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, param_goal_position);	//書き込み用パケットにデータを追加
			if(dxl_addparam_result != true) printf("goal pose error!¥n");

			// Bulkwrite goal position 
			dxl_comm_result = groupBulkWrite->txPacket();
			if (dxl_comm_result != COMM_SUCCESS) printf("%s¥n", packetHandler->getTxRxResult(dxl_comm_result));

			// Clear bulkwrite parameter storage
			groupBulkWrite->clearParam();
		}printf("¥n");
		sleep(3);																														//データが再生し終わる用にsleepを行う
	}
	fclose(fp);
	return 1;
}

/**
 * @fn		void Close_port()
 * @brief	通信ポートを閉じる
 */
void CR7::Close_port(){
	// Close port
	portHandler->closePort();
	printf("port close and exit program¥n");
}

/**
 * @fn		main()
 * @brief	main
 */
int main()
{
	CR7 cr;																																//クラスの宣言
	if(!cr.Open_port()) return 0;																										//COMポートを開く
	if(!cr.Set_port_baudrate()) return 0;																								//ポートの通信レートを設定

	while(1){
		printf("q:exit o:[SERVO ON] i:[SERVO OFF] p:[MOVE POSITION] s:[Offset Position] t:[Teaching] l:[Play back data]¥n");
		char ch = getchar();							//キー入力　"q":プログラム終了
		switch(ch){
			case 'q':									//プログラム終了
				cr.Disable_Dynamixel_Torque();				//トルクOFF
				return 0;
				break;
			case 'o':									//サーボトルクON
				cr.Enable_Dynamixel_Torque();
				break;
			case 'i':									//サーボトルクOFF
				cr.Disable_Dynamixel_Torque();
				break;
			case 'p':									//指定の角度に移動
				cr.Move_Goal_Position( goal_pose );
				break;
			case 's':									//初期姿勢に移動
				cr.Move_Offset_Position();
				break;
			case 't':									//テキストに角度を保存
				cr.Disable_Dynamixel_Torque();				//トルクOFF
				cr.Teaching_Play_Frame();
				break;
			case 'l':									//テキストデータの再生
				printf("----ATTENTION!----¥n");
				cr.Enable_Dynamixel_Torque();				//トルクON
				sleep(3);									//突然動作しないようにスリープ
				cr.Move_Offset_Position();					//初期姿勢に移動
				cr.Play_Back();								//データの再生
				cr.Move_Offset_Position();					//初期姿勢に移動
				printf("--FINISH--¥n");
				break;
		}	
	}
	cr.Close_port();																													//COMポートを閉じる
	return 0;
}

