/**
 * RobStride.h
 * RobStride 05 (EL05) Motor Library for Mbed (Extended CAN ID Version)
 * Modified for Velocity Mode limits
 */

#ifndef ROBSTRIDE_H
#define ROBSTRIDE_H

#include "mbed.h"
#include "CANManager.h"
// 物理パラメータ制限
#define P_MIN -12.57f
#define P_MAX 12.57f
#define V_MIN -50.0f
#define V_MAX 50.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -6.0f
#define T_MAX 6.0f

// 通信タイプ
enum CommType {
    CMD_MOTION_CTRL = 0x01,
    CMD_FEEDBACK    = 0x02,
    CMD_ENABLE      = 0x03,
    CMD_DISABLE     = 0x04,
    CMD_SET_ZERO    = 0x06,
    CMD_WRITE_PARAM = 0x12
};

// 実行モード
enum RunMode {
    MODE_OPERATION = 0,
    MODE_POSITION  = 1,
    MODE_VELOCITY  = 2,
    MODE_CURRENT   = 3,
    MODE_CSP       = 5
};

class RobStride :public CANReceiver{
public:
    RobStride(CAN &can, uint8_t motor_id, uint8_t master_id = 0x00);

    void enable();
    void disable();
    void setZero();

    // MIT Mode
    void sendMotionCommand(float torque, float position, float velocity, float kp, float kd);

    // パラメータ設定 (Type 18)
    void setRunMode(RunMode mode);
    void setPosition(float position_rad);
    void setVelocity(float velocity_rads);
    void setCurrent(float current_a);

    // --- 追加修正箇所: 制限設定 ---
    void setCurrentLimit(float current_limit_a); // 最大電流設定
    void setAcceleration(float acc_rad_s2);      // 加速度設定
    // ---------------------------
    // PPモード用設定 (位置制御)
    void setMaxSpeedPP(float speed_rad_s);      // 移動速度制限 (0x7024)
    void setAccelerationPP(float acc_rad_s2);   // 加速度 (0x7025)
    

    int update();

    float getPosition();
    float getVelocity();
    float getTorque();
    float getTemperature();

    // 汎用パラメータ送信 (publicに公開)
    void sendParam(uint16_t index, float value);
    void sendParam(uint16_t index, uint8_t value);

    virtual bool handle_message(const CANMessage &msg) override;

private:
    CAN &_can;
    CANMessage rcv_msg;
    uint8_t _motor_id;
    uint8_t _master_id;

    float _fb_p, _fb_v, _fb_t, _fb_temp;

    int float_to_uint(float x, float x_min, float x_max, int bits);
    float uint_to_float(int x_int, float x_min, float x_max, int bits);
};

#endif
