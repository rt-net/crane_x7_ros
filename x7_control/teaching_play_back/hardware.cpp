#include"hardware.h"

double deg2value( double deg ){return (deg + 180) * 4096 / 360; }
double value2deg( double value ){return value * 360 / 4096 -180; }
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
		//getch();
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
		//getch();
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
	//fp = fopen( fname, "w");																											//モード:w(書き込み専用)でファイルを開く
	/*if(fp == NULL){
		printf("can not open¥n");
		return -1;
	}*/
	//while(1){																															//データ取得の開始
		printf("teaching play frame( type q end teaching )¥n");
		//char ch2 = getch();																												//　"q"　以外のキーが押された時の角度をテキストデータへ保存
		//if(ch2 == 'q')break;
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
			//printf("[ ID[%d] : %lf ]", ID[i], value2deg(dxl_present_position));
			joint_pose[i] = dxl_present_position;
			//fprintf(fp, "%d,", dxl_present_position);
		}printf("\n");
		//fprintf(fp,"¥n");																												//サーボ数分を1つとして改行
	//}
	//fclose(fp);																															//ファイルを閉じる
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

