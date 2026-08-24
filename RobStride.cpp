/**
 * RobStride.cpp
 * 拡張CAN ID (29bit) 対応実装 - 修正版
 */

#include "RobStride.h"

// パラメータインデックス (Manual 4.1.16)
#define IDX_RUN_MODE  0x7005
#define IDX_IQ_REF    0x7006 // 電流モード目標値
#define IDX_SPD_REF   0x700A // 速度モード目標値
#define IDX_LOC_REF   0x7016
// --- 追加定義 ---
#define IDX_LIMIT_CUR 0x7018 // 速度・位置モード電流制限 [cite: 2098]
#define IDX_ACC_RAD   0x7022 // 速度モード加速度 [cite: 2102]
#define IDX_VEL_MAX   0x7024 // PPモード 最大速度
#define IDX_ACC_SET   0x7025 // PPモード 加速度

// ----------------

RobStride::RobStride(CAN &can, uint8_t motor_id, uint8_t master_id) 
    : _can(can), _motor_id(motor_id), _master_id(master_id) {
    _fb_p = 0; _fb_v = 0; _fb_t = 0; _fb_temp = 0;
}

int RobStride::float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float RobStride::uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int * span / ((float)((1 << bits) - 1))) + offset;
}

void RobStride::enable() {
    CANMessage msg;
    msg.format = CANExtended;
    msg.id = (CMD_ENABLE << 24) | (_master_id << 8) | _motor_id;
    msg.len = 8;
    memset(msg.data, 0, 8);
    _can.write(msg);
}

void RobStride::disable() {
    CANMessage msg;
    msg.format = CANExtended;
    msg.id = (CMD_DISABLE << 24) | (_master_id << 8) | _motor_id;
    msg.len = 8;
    memset(msg.data, 0, 8);
    _can.write(msg);
}

void RobStride::setZero() {
    CANMessage msg;
    msg.format = CANExtended;
    msg.id = (CMD_SET_ZERO << 24) | (_master_id << 8) | _motor_id;
    msg.len = 8;
    memset(msg.data, 0, 8);
    msg.data[0] = 1; 
    _can.write(msg);
}

void RobStride::sendMotionCommand(float torque, float position, float velocity, float kp, float kd) {
    CANMessage msg;
    msg.format = CANExtended;
    msg.len = 8;

    uint16_t t_int = float_to_uint(torque, T_MIN, T_MAX, 16);
    uint16_t p_int = float_to_uint(position, P_MIN, P_MAX, 16);
    uint16_t v_int = float_to_uint(velocity, V_MIN, V_MAX, 16);
    uint16_t kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 16);
    uint16_t kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 16);

    msg.id = (CMD_MOTION_CTRL << 24) | (t_int << 8) | _motor_id;

    msg.data[0] = p_int >> 8;
    msg.data[1] = p_int & 0xFF;
    msg.data[2] = v_int >> 8;
    msg.data[3] = v_int & 0xFF;
    msg.data[4] = kp_int >> 8;
    msg.data[5] = kp_int & 0xFF;
    msg.data[6] = kd_int >> 8;
    msg.data[7] = kd_int & 0xFF;

    _can.write(msg);
}

void RobStride::sendParam(uint16_t index, float value) {
    CANMessage msg;
    msg.format = CANExtended;
    msg.id = (CMD_WRITE_PARAM << 24) | (_master_id << 8) | _motor_id;
    msg.len = 8;
    
    msg.data[0] = index & 0xFF;
    msg.data[1] = index >> 8;
    msg.data[2] = 0x00;
    msg.data[3] = 0x00;
    memcpy(&msg.data[4], &value, 4);
    
    _can.write(msg);
}

void RobStride::sendParam(uint16_t index, uint8_t value) {
    CANMessage msg;
    msg.format = CANExtended;
    msg.id = (CMD_WRITE_PARAM << 24) | (_master_id << 8) | _motor_id;
    msg.len = 8;
    
    memset(msg.data, 0, 8);
    msg.data[0] = index & 0xFF;
    msg.data[1] = index >> 8;
    msg.data[4] = value;
    
    _can.write(msg);
}

void RobStride::setRunMode(RunMode mode) { sendParam(IDX_RUN_MODE, (uint8_t)mode); }
void RobStride::setPosition(float pos)     { sendParam(IDX_LOC_REF, pos); }
void RobStride::setVelocity(float vel)     { sendParam(IDX_SPD_REF, vel); }
void RobStride::setCurrent(float cur)      { sendParam(IDX_IQ_REF, cur); }

// --- 追加実装 ---
void RobStride::setCurrentLimit(float limit) { sendParam(IDX_LIMIT_CUR, limit); }
void RobStride::setAcceleration(float acc)   { sendParam(IDX_ACC_RAD, acc); }
void RobStride::setMaxSpeedPP(float speed) { sendParam(IDX_VEL_MAX, speed); }
void RobStride::setAccelerationPP(float acc) { sendParam(IDX_ACC_SET, acc); }
// ---------------

int RobStride::update() {//そんなに使わない
    CANMessage msg;
    int return_can = _can.read(msg);
    if (return_can) {
        uint8_t type = (msg.id >> 24) & 0x1F;
        uint8_t src_id = (msg.id >> 8) & 0xFF;

        if (msg.format == CANExtended && type == CMD_FEEDBACK && src_id == _motor_id) {
            uint16_t p_int = (msg.data[0] << 8) | msg.data[1];
            uint16_t v_int = (msg.data[2] << 8) | msg.data[3];
            uint16_t t_int = (msg.data[4] << 8) | msg.data[5];
            uint16_t temp_int = (msg.data[6] << 8) | msg.data[7];

            _fb_p = uint_to_float(p_int, P_MIN, P_MAX, 16);
            _fb_v = uint_to_float(v_int, V_MIN, V_MAX, 16);
            _fb_t = uint_to_float(t_int, T_MIN, T_MAX, 16);
            _fb_temp = static_cast<float>(temp_int / 10.0f);
        }
    }
    return return_can;
}

float RobStride::getPosition() { return _fb_p; }
float RobStride::getVelocity() { return _fb_v; }
float RobStride::getTorque() { return _fb_t; }
float RobStride::getTemperature() { return _fb_temp; }

bool RobStride::handle_message(const CANMessage &msg){
    uint8_t type = (msg.id >> 24) & 0x1F;
    uint8_t src_id = (msg.id >> 8) & 0xFF;

    if (msg.format == CANExtended && type == CMD_FEEDBACK && src_id == _motor_id) {
        uint16_t p_int = (msg.data[0] << 8) | msg.data[1];
        uint16_t v_int = (msg.data[2] << 8) | msg.data[3];
        uint16_t t_int = (msg.data[4] << 8) | msg.data[5];
        uint16_t temp_int = (msg.data[6] << 8) | msg.data[7];

        _fb_p = uint_to_float(p_int, P_MIN, P_MAX, 16);
        _fb_v = uint_to_float(v_int, V_MIN, V_MAX, 16);
        _fb_t = uint_to_float(t_int, T_MIN, T_MAX, 16);
        _fb_temp = static_cast<float>(temp_int / 10.0f);
        return true;
    }
    return false;
}
