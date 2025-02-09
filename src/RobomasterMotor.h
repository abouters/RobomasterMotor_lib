#ifndef ROBOMASTERMOTOR_H
#define ROBOMASTERMOTOR_H

#include <Arduino.h>
#include <PID.h>
#include <CAN.h>

#define M2006 1
#define M3508 2

#define MAXRPM_M3508 10000					// M3508の最大目標RPM
#define MAXRPM_M2006 10000					// M2006の最大目標RPM
#define MAXRPM_M3508_POSTION_CONTROL 8000	// M3508の位置制御時における最大目標RPM
#define MAXRPM_M2006_POSTION_CONTROL 5000	// M2006の位置制御時における最大目標RPM
#define MAXANPARE_M3508 16384				// M3508の最大出力電流
#define MAXANPARE_M2006 10000				// M2006の最大出力電流

#define RESOLUTION 8192					// ロボマスモーター内部のエンコーダの１回転あたりの分解能

typedef struct{
	int16_t angle = 0;			// モータ内部にあるエンコーダの機械的角度
	int16_t pre_angle = 0;
	int64_t position = 0;			// モータ内部にあるエンコーダの機械的角度
	int16_t rpm = 0;			// モータの回転速度
	int16_t ampare = 0;			// モータに流れるトルク電流
	int16_t temperature = 0;	// モータの温度
}MotorParam;

typedef struct{
	int64_t target_pos = 0;			// モータ内部にあるエンコーダの機械的角度
	int16_t target_rpm = 0;			// モータの回転速度
	int16_t target_ampare = 0;			// モータに流れるトルク電流
}MotorContorlData;

class RobomasterMotor{
    public:
		RobomasterMotor(CanControl& motor_can, double control_cycle);
        ~RobomasterMotor();

		void setMotorType(uint8_t id, uint8_t type);		// モーターの型を指定する(M3508 or M2006)

		void setTargetRpm(uint8_t id, int16_t rpm);
		void setTargetPosition(uint8_t id, int64_t pos);
		
		void setRpmPIDgain(uint8_t id, PIDGain rpm_gain);
		void setPositionPIDgain(uint8_t id, PIDGain pos_gain);

		int16_t getAngle(uint8_t id)			{return paramList[id-1].angle;}
		int64_t getPosition(uint8_t id)			{return paramList[id-1].position;}
		int16_t getRpm(uint8_t id)				{return paramList[id-1].rpm;}
		int16_t getAmpare(uint8_t id)			{return paramList[id-1].ampare;}
		int16_t getTemperature(uint8_t id)		{return paramList[id-1].temperature;}

		void resetPosition(uint8_t id, int64_t position)		{paramList[id-1].position = 0;}  // モーターの現在位置の値を0にリセットする（緊急停止からの原点復帰用）
		void setCurrent(uint8_t id, int64_t current)			{controlDataList[id-1].target_ampare = current;} // 目標の電流値を設定する

		int16_t getOutputValue(uint8_t id);			// PID制御系から求まった出力電流値を取得する
		void readMotorParam(uint8_t id);			// モーターからのデータを読み取る
        void sendMotorData();						// モーターにデータを送信する

		static void interruptHandler();				// タイマー割り込みで呼び出す関数

    private:
		double control_cycle;						// 制御周期[ms]
		static RobomasterMotor *instance;			// タイマー割り込みハンドラ用のインスタンス
		CanControl& motor_can;						// モーター制御用CAN
		PID pid_rpm[8];								// 速度制御用PID
		PID pid_pos[8];								// 位置制御用PID
		MotorParam paramList[8] = {};				// モーターデータを格納するリスト
		MotorContorlData controlDataList[8] = {}; 	// 制御用データを格納するリスト
		bool flag_motorControl[8] = {}; 			// モーターを制御するかどうかのフラグ
		bool is_spdControl[8] = {}; 				// 速度制御を行うかどうか
		bool is_posControl[8] = {}; 				// 位置制御を行うかどうか

		int16_t calculateVelocity(uint8_t id);		// 位置制御PIDから求まった目標速度を計算する
		int16_t calculateCurrent(uint8_t id);		// 速度制御PIDから求まった目標電流値を計算する
		void measurePosition(uint8_t id);			// エンコーダの値の差分からモーター位置を算出する
};

#endif