#include"hardware.h"

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
				for(int i=0;i<JOINT_NUM;i++){
					printf("%lf",cr.joint_pose[i]);
				}printf("\n");
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

