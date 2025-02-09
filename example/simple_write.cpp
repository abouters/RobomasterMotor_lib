#include <Arduino.h>
#include <RobomasterMotor.h>
#include <CAN.h>
#include <PID.h>
#include <IntervalTimer.h>
#include <Metro.h>

const PIDGain Rpmmotor = {3, 0.3, 0.5};
const PIDGain Posmotor = {0.03, 0.0, 0.};
const double control_cycle = 1.0; // 制御周期[ms]
CanControl can1(1);
RobomasterMotor motor(can1, control_cycle);

IntervalTimer calc_timer; // タイマー割り込み用インスタンス「

void setup() {
  Serial.begin(115200);
  motor.setMotorType(1, M3508);           // 最初にモーターを指定する(M3508 or M2006)
  motor.setRpmPIDgain(1, Rpmmotor);       // 速度制御用PIDゲインを設定する
  motor.setPositionPIDgain(1, Posmotor);  // 位置制御用PIDゲインを設定する
  calc_timer.begin(motor.interruptHandler, control_cycle*1000); // タイマー割り込みを開始する　[μs]なので制御周期を1000倍する
}

void loop() {
  // 以下の速度制御もしくは位置制御用関数のどちらかを実行する
  // motor.setTargetRpm(1, 0);
  motor.setTargetPosition(1, 0);
}

