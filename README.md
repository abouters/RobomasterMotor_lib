# RobomasterMotor ライブラリ

## 概要
RobomasterMotor は、Robomaster M3508 および M2006 モーターを CAN 通信で制御するためのライブラリです。速度型PID制御を用いた速度および位置制御をサポートしています。

## 特徴
- M3508 および M2006 モーターに対応
- PID 制御による精密なトルク、速度、位置制御
- CAN 通信によるデータ送受信
- モーターの状態（角度、速度、電流、温度）の取得
- 緊急停止および原点復帰機能

## インストール
platformio.iniに以下の文を追加してください。

```cpp
lib_deps =  https://github.com/abouters/PID
            https://github.com/abouters/CAN
            https://github.com/abouters/RobomasterMotor_lib
```

## 使用方法

### 1. 初期化
```cpp
CanControl can1(1); // CAN1 を使用
RobomasterMotor motor(can1, 1.0); // 制御周期 1ms
```

### 2. モーターの種類を設定
```cpp
motor.setMotorType(1, M3508); // ID 1 のモーターを M3508 に設定
```

### 3. PID ゲインを設定
```cpp
PIDGain rpm_gain = {1.0, 0.1, 0.01};
PIDGain pos_gain = {2.0, 0.2, 0.02};
motor.setRpmPIDgain(1, rpm_gain);
motor.setPositionPIDgain(1, pos_gain);
```

### 4. タイマー割り込みを設定
```cpp
IntervalTimer calc_timer; // タイマー割り込み用インスタンス
calc_timer.begin(motor.interruptHandler, control_cycle*1000); // タイマー割り込みを開始する　[μs]なので制御周期を1000倍する
```

### 5. 目標速度または位置を設定
```cpp
motor.setTargetRpm(1, 5000); // ID 1 のモーターを 5000 rpm に設定
motor.setTargetPosition(1, 100000); // ID 1 のモーターを位置 100000 に移動
```
### 6. 制御ループの例
```cpp
void loop() {
    motor.setTargetRpm(1, 5000); // ID 1 のモーターを 5000 rpm に設定
}
```

## API リファレンス

### **1. 初期化**
```cpp
RobomasterMotor(CanControl& motor_can, double control_cycle);
```
- モーター制御クラスのインスタンスを作成

### **2. モーター設定**
```cpp
void setMotorType(uint8_t id, uint8_t type);
```
- モーターの種類（M3508 / M2006）を設定

### **3. 目標設定**
```cpp
void setTargetRpm(uint8_t id, int16_t rpm);
void setTargetPosition(uint8_t id, int64_t pos);
```
- 目標速度または位置を設定

### **4. PID 設定**
```cpp
void setRpmPIDgain(uint8_t id, PIDGain rpm_gain);
void setPositionPIDgain(uint8_t id, PIDGain pos_gain);
```
- PID ゲインを設定

### **5. モーター情報取得**
```cpp
int16_t getAngle(uint8_t id);
int64_t getPosition(uint8_t id);
int16_t getRpm(uint8_t id);
int16_t getAmpare(uint8_t id);
int16_t getTemperature(uint8_t id);
```
- モーターの状態を取得

### **6. 制御およびデータ送信**
```cpp
void readMotorParam(uint8_t id);
void sendMotorData();
```
- モーターからデータを取得し、送信

### **7. その他の機能**
```cpp
void resetPosition(uint8_t id, int64_t position);
void setCurrent(uint8_t id, int64_t current);
```
- 原点復帰とトルク制御

