#pragma once
#include "RobStride.h"
#include <cstdint>
#include <vector>

// 継承（public RobStride）を外し、包含する設計に変更
class RobStrideMultiple {
public:
    RobStrideMultiple(CAN &can, uint8_t motor_id, uint8_t motor_num, uint8_t master_id = 0x00) {
        // メモリの再確保を防ぐため、あらかじめ必要な要素数を予約しておく
        _robstride.reserve(motor_num);

        for(int node = 0; node < motor_num; node++) {
            _robstride.emplace_back(can, motor_id + node, master_id);
        }
    }

    // ==========================================
    // 全モータ一括操作メソッド (_all)
    // ==========================================
    void enable_all() { for (RobStride &r : _robstride) r.enable(); }
    void disable_all() { for (RobStride &r : _robstride) r.disable(); }
    void setZero_all() { for (RobStride &r : _robstride) r.setZero(); }
    
    void sendMotionCommand_all(float torque, float position, float velocity, float kp, float kd) {
        for (RobStride &r : _robstride) r.sendMotionCommand(torque, position, velocity, kp, kd);
    }
    
    void setRunMode_all(RunMode mode) { for (RobStride &r : _robstride) r.setRunMode(mode); }
    void setPosition_all(float position_rad) { for (RobStride &r : _robstride) r.setPosition(position_rad); }
    void setVelocity_all(float velocity_rads) { for (RobStride &r : _robstride) r.setVelocity(velocity_rads); }
    void setCurrent_all(float current_a) { for (RobStride &r : _robstride) r.setCurrent(current_a); }
    
    // 最大電流設定
    void setCurrentLimit_all(float current_limit_a) { for (RobStride &r : _robstride) r.setCurrentLimit(current_limit_a); }
    // 加速度設定
    void setAcceleration_all(float acc_rad_s2) { for (RobStride &r : _robstride) r.setAcceleration(acc_rad_s2); }
    // 移動速度制限 (0x7024)
    void setMaxSpeedPP_all(float speed_rad_s) { for (RobStride &r : _robstride) r.setMaxSpeedPP(speed_rad_s); }
    // 加速度 (0x7025)
    void setAccelerationPP_all(float acc_rad_s2) { for (RobStride &r : _robstride) r.setAccelerationPP(acc_rad_s2); }

    int update_all() {
        int status = 0;
        for (RobStride &r : _robstride) {
            // 戻り値がエラーフラグ等であることを想定し、ビットORで全モータのステータスをまとめる
            status |= r.update(); 
        }
        return status;
    }

    // --- 一括取得メソッド ---
    // ※ 呼び出し側でモータ数分（motor_num）の要素を持つ配列を用意してポインタを渡す想定
    void getPosition_all(float *position) {
        for (size_t i = 0; i < _robstride.size(); ++i) position[i] = _robstride[i].getPosition();
    }
    void getVelocity_all(float *velocity) {
        for (size_t i = 0; i < _robstride.size(); ++i) velocity[i] = _robstride[i].getVelocity();
    }
    void getTorque_all(float *torque) {
        for (size_t i = 0; i < _robstride.size(); ++i) torque[i] = _robstride[i].getTorque();
    }
    void getTemperature_all(float *temperature) {
        for (size_t i = 0; i < _robstride.size(); ++i) temperature[i] = _robstride[i].getTemperature();
    }

    void sendParam_all(uint16_t index, float value) { for (RobStride &r : _robstride) r.sendParam(index, value); }
    void sendParam_all(uint16_t index, uint8_t value) { for (RobStride &r : _robstride) r.sendParam(index, value); }


    // ==========================================
    // 単体モータ操作メソッド (node_zerobase指定)
    // ==========================================
    void enable(uint8_t node_zerobase) { _robstride[node_zerobase].enable(); }
    void disable(uint8_t node_zerobase) { _robstride[node_zerobase].disable(); }
    void setZero(uint8_t node_zerobase) { _robstride[node_zerobase].setZero(); }

    // MIT Mode
    void sendMotionCommand(float torque, float position, float velocity, float kp, float kd, uint8_t node_zerobase) {
        _robstride[node_zerobase].sendMotionCommand(torque, position, velocity, kp, kd);
    }

    // パラメータ設定 (Type 18)
    void setRunMode(RunMode mode, uint8_t node_zerobase) { _robstride[node_zerobase].setRunMode(mode); }
    void setPosition(float position_rad, uint8_t node_zerobase) { _robstride[node_zerobase].setPosition(position_rad); }
    void setVelocity(float velocity_rads, uint8_t node_zerobase) { _robstride[node_zerobase].setVelocity(velocity_rads); }
    void setCurrent(float current_a, uint8_t node_zerobase) { _robstride[node_zerobase].setCurrent(current_a); }
    
    // 最大電流設定
    void setCurrentLimit(float current_limit_a, uint8_t node_zerobase) { _robstride[node_zerobase].setCurrentLimit(current_limit_a); }
    // 加速度設定
    void setAcceleration(float acc_rad_s2, uint8_t node_zerobase) { _robstride[node_zerobase].setAcceleration(acc_rad_s2); }
    // 移動速度制限 (0x7024)
    void setMaxSpeedPP(float speed_rad_s, uint8_t node_zerobase) { _robstride[node_zerobase].setMaxSpeedPP(speed_rad_s); }
    // 加速度 (0x7025)
    void setAccelerationPP(float acc_rad_s2, uint8_t node_zerobase) { _robstride[node_zerobase].setAccelerationPP(acc_rad_s2); }
    
    int update(uint8_t node_zerobase) { return _robstride[node_zerobase].update(); }

    float getPosition(uint8_t node_zerobase) { return _robstride[node_zerobase].getPosition(); }
    float getVelocity(uint8_t node_zerobase) { return _robstride[node_zerobase].getVelocity(); }
    float getTorque(uint8_t node_zerobase) { return _robstride[node_zerobase].getTorque(); }
    float getTemperature(uint8_t node_zerobase) { return _robstride[node_zerobase].getTemperature(); }

    // 汎用パラメータ送信 (publicに公開)
    void sendParam(uint16_t index, float value, uint8_t node_zerobase) { _robstride[node_zerobase].sendParam(index, value); }
    void sendParam(uint16_t index, uint8_t value, uint8_t node_zerobase) { _robstride[node_zerobase].sendParam(index, value); }

private:
    std::vector<RobStride> _robstride;
};