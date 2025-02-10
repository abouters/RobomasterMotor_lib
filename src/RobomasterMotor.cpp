#include <RobomasterMotor.h>

RobomasterMotor* RobomasterMotor::instance = nullptr;

RobomasterMotor::RobomasterMotor(CanControl& motor_can, double control_cycle): control_cycle(control_cycle), motor_can(motor_can){
    motor_can.init(1e6);
    instance = this;
    
};

RobomasterMotor::~RobomasterMotor(){}

void RobomasterMotor::setTargetRpm(uint8_t id, int16_t rpm){
    is_spdControl[id-1] = true;
    is_posControl[id-1] = false;
    controlDataList[id-1].target_rpm = rpm;
}
void RobomasterMotor::setTargetPosition(uint8_t id, int64_t pos){
    is_posControl[id-1] = true;
    is_spdControl[id-1] = false;
    controlDataList[id-1].target_pos = pos;
}

void RobomasterMotor::setMotorType(uint8_t id, uint8_t type) {
    switch(type){
        case M3508:
            pid_rpm[id-1] = PID(control_cycle, -MAXANPARE_M3508, MAXANPARE_M3508);
            pid_pos[id-1] = PID(control_cycle, -MAXRPM_M3508_POSTION_CONTROL, MAXRPM_M3508_POSTION_CONTROL);
            break;
        case M2006:
            pid_rpm[id-1] = PID(control_cycle, -MAXANPARE_M2006, MAXANPARE_M2006);
            pid_pos[id-1] = PID(control_cycle, -MAXRPM_M2006_POSTION_CONTROL, MAXRPM_M2006_POSTION_CONTROL);
            break;
        default:
            break;
    }
    flag_motorControl[id-1] = true;
    is_torque[id-1] = true;
}

void RobomasterMotor::setMotorTorque(uint8_t id, bool is_torque){
    this->is_torque[id-1] = is_torque;
}

void RobomasterMotor::setRpmPIDgain(uint8_t id, PIDGain rpm_gain){
    if(flag_motorControl[id-1] == false) return;
    pid_rpm[id-1].setGain(rpm_gain);
}
void RobomasterMotor::setPositionPIDgain(uint8_t id, PIDGain pos_gain){
    if(flag_motorControl[id-1] == false) return;
    pid_pos[id-1].setGain(pos_gain);
}	

int16_t RobomasterMotor::getOutputValue(uint8_t id){
    if(flag_motorControl[id-1] == false || motor_can.check_is_contact(0x200+id) == false) return 0;
    
    if(is_posControl[id-1]) setTargetRpm(id, calculateVelocity(id));
    return calculateCurrent(id);
}

int16_t RobomasterMotor::calculateVelocity(uint8_t id){
	int64_t error = controlDataList[id-1].target_pos - paramList[id-1].position;
	return (int16_t)pid_pos[id-1].calculate(error);
}

int16_t RobomasterMotor::calculateCurrent(uint8_t id){
	int32_t error = controlDataList[id-1].target_rpm - paramList[id-1].rpm;
	return (int16_t)pid_rpm[id-1].calculate(error);
}

void RobomasterMotor::measurePosition(uint8_t id){
	int32_t diff = paramList[id-1].angle - paramList[id-1].pre_angle;
    if      (diff < -(RESOLUTION / 2))       diff += RESOLUTION;
    else if (diff >  (RESOLUTION / 2))       diff -= RESOLUTION;
    paramList[id-1].position += diff;
	paramList[id-1].pre_angle = paramList[id-1].angle;
}


void RobomasterMotor::readMotorParam(uint8_t id){
    uint8_t read_data[8];
    motor_can.CANDataPull(0x200+id, read_data);
    paramList[id-1].angle       = (read_data[0] << 8) + read_data[1];		
	paramList[id-1].rpm         = (read_data[2] << 8) + read_data[3];			
	paramList[id-1].ampare      = (read_data[4] << 8) + read_data[5];	
	paramList[id-1].temperature = read_data[6];
    measurePosition(id);
}

void RobomasterMotor::sendMotorData() {
    CAN_message_t send_msg[2];
    send_msg[0].id = 0x200;
    send_msg[1].id = 0x1FF;
    for(int i=0; i<2; i++){
        for (int j = 0; j < 4; j++) {
            uint8_t index = i*4 + j;
            send_msg[i].buf[j*2] = controlDataList[index].target_ampare >> 8;     // 上位バイト
            send_msg[i].buf[j*2 + 1] = controlDataList[index].target_ampare & 0xFF; // 下位バイト
        }
    }

    for(int i=0; i<2; i++){
        motor_can.CANMsgWrite(send_msg[i]);
    }
}

void RobomasterMotor::interruptHandler(){
    for(int id=1; id<=8; id++){
        instance->readMotorParam(id);
        if(instance->is_torque[id-1] == true){
            instance->controlDataList[id-1].target_ampare = instance->getOutputValue(id);
        }else{
            instance->controlDataList[id-1].target_ampare = 0;
        }
    }
    instance->sendMotorData();
}