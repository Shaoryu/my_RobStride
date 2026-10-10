#include "RobStride.h"
#include <cstring>

RobStride::RobStride(CAN &can, uint8_t motor_id, uint8_t master_id, osPriority_t priority, uint32_t stack_size)
    : _can(can),
      _motor_id(motor_id),
      _master_id(master_id),
      _thread(priority, stack_size),
      _has_new_msg(false),
      _last_rx_time(),
      _has_received_any(false),
      _tx_fail_count(0),
      _current_run_mode(RunMode::OPERATION),
      _is_first_rx(false),
      _last_raw_angle(0.0f),
      _accumulated_angle(0.0f),
      _origin_offset(0.0f),
      _fb_raw_angle(0.0f),
      _fb_v(0.0f),
      _fb_t(0.0f),
      _fb_temp(0.0f),
      _mode_state(0),
      _target_angle(0.0f),
      _pos_kp(5.0f),
      _pos_ki(0.0f),
      _pos_kd(0.1f),
      _pos_integral(0.0f),
      _pos_prev_err(0.0f),
      _max_output_spd(20.0f),
      _damping_kd(1.0f),
      _limit_enabled(false),
      _limit_min_rad(0.0f),
      _limit_max_rad(0.0f) {

    _fault_status.raw = 0;
    for (size_t i = 0; i < CACHE_SIZE; ++i) {
        _param_caches[i].valid = false;
    }
}

void RobStride::start() {
    _pid_timer.start();
    _thread.start(callback(this, &RobStride::control_loop));
}

bool RobStride::write_can_retry(const CANMessage &msg) {
    for (int retry = 0; retry < 3; ++retry) {
        if (_can.write(msg)) {
            return true;
        }
        wait_us(10);
    }
    {
        CriticalSectionLock lock;
        _tx_fail_count++;
    }
    return false;
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

// 割り込みコンテキスト(ISR)安全: バッファ保存とフラグ通知のみ
bool RobStride::handle_message(const CANMessage &msg) {
    if (msg.format != CANExtended) return false;

    uint8_t type = (msg.id >> 24) & 0x1F;
    uint8_t motor_id = 0;

    if (type == CMD_FEEDBACK || type == CMD_READ_PARAM || type == CMD_FAULT_FB) {
        motor_id = (msg.id >> 8) & 0xFF;
    }

    if (motor_id == _motor_id) {
        {
            CriticalSectionLock lock;
            _rx_buffer = msg;
            _has_new_msg = true;
        }
        _event_flags.set(0x01);
        return true;
    }
    return false;
}

void RobStride::control_loop() {
    while (true) {
        _event_flags.wait_any(0x01);

        CANMessage msg;
        bool has_data = false;

        {
            CriticalSectionLock lock;
            if (_has_new_msg) {
                msg = _rx_buffer;
                _has_new_msg = false;
                has_data = true;
                _last_rx_time = Kernel::Clock::now();
                _has_received_any = true;
            }
        }

        if (!has_data) continue;

        uint8_t type = (msg.id >> 24) & 0x1F;

        if (type == CMD_FEEDBACK) {
            // タイプ2: ビッグエンディアン
            uint16_t p_raw = ((uint16_t)msg.data[0] << 8) | msg.data[1];
            uint16_t v_raw = ((uint16_t)msg.data[2] << 8) | msg.data[3];
            uint16_t t_raw = ((uint16_t)msg.data[4] << 8) | msg.data[5];
            uint16_t temp_raw = ((uint16_t)msg.data[6] << 8) | msg.data[7];

            float cur_raw_p = uint_to_float(p_raw, P_MIN, P_MAX, 16);
            float cur_v = uint_to_float(v_raw, V_MIN, V_MAX, 16);
            float cur_t = uint_to_float(t_raw, T_MIN, T_MAX, 16);
            float cur_temp = (float)temp_raw * 0.1f;

            uint8_t mode = (msg.id >> 22) & 0x03;
            uint8_t fault_bits = (msg.id >> 16) & 0x3F;

            float cur_pos = 0.0f;
            float target_pos = 0.0f;
            float kp, ki, kd, max_v, damping;
            RunMode cur_run_mode;

            {
                CriticalSectionLock lock;

                // 累積角度計算 (-4π ~ 4π 跨ぎ判定)
                if (!_is_first_rx) {
                    _origin_offset = cur_raw_p;
                    _last_raw_angle = cur_raw_p;
                    _accumulated_angle = 0.0f;
                    _is_first_rx = true;
                } else {
                    float diff = cur_raw_p - _last_raw_angle;
                    const float range = 8.0f * (float)M_PI;
                    const float wrap_thresh = 4.0f * (float)M_PI;

                    if (diff < -wrap_thresh && cur_v >= 0.0f) {
                        diff += range;
                    } else if (diff > wrap_thresh && cur_v <= 0.0f) {
                        diff -= range;
                    }

                    _accumulated_angle += diff;
                    _last_raw_angle = cur_raw_p;
                }

                _fb_raw_angle = cur_raw_p;
                _fb_v = cur_v;
                _fb_t = cur_t;
                _fb_temp = cur_temp;
                _mode_state = mode;

                _fault_status.under_volt     = (fault_bits >> 0) & 1;
                _fault_status.c_phase_oc     = (fault_bits >> 1) & 1;
                _fault_status.over_temp      = (fault_bits >> 2) & 1;
                _fault_status.drv_fault      = (fault_bits >> 3) & 1;
                _fault_status.stall_overload = (fault_bits >> 4) & 1;
                _fault_status.uncalibrated   = (fault_bits >> 5) & 1;

                cur_pos = _accumulated_angle - _origin_offset;
                target_pos = _target_angle;
                if (_limit_enabled) {
                    if (target_pos > _limit_max_rad) target_pos = _limit_max_rad;
                    if (target_pos < _limit_min_rad) target_pos = _limit_min_rad;
                }

                kp = _pos_kp;
                ki = _pos_ki;
                kd = _pos_kd;
                max_v = _max_output_spd;
                damping = _damping_kd;
                cur_run_mode = _current_run_mode;
            }

            // Mbed側で運控モード(MODE_OPERATION)として動作している場合のみPID計算・送信
            if (cur_run_mode == RunMode::OPERATION) {
                float dt = std::chrono::duration<float>(_pid_timer.elapsed_time()).count();
                _pid_timer.reset();
                if (dt <= 0.0f || dt > 0.05f) {
                    dt = 0.001f;
                    _pos_integral = 0.0f;
                    _pos_prev_err = 0.0f;
                }

                float err = target_pos - cur_pos;
                _pos_integral += (err + _pos_prev_err) * dt * 0.5f;
                if (ki > 0.0f) {
                    float i_lim = max_v / ki;
                    if (_pos_integral > i_lim) _pos_integral = i_lim;
                    else if (_pos_integral < -i_lim) _pos_integral = -i_lim;
                }
                float deriv = (err - _pos_prev_err) / dt;
                _pos_prev_err = err;

                float out_spd = (kp * err) + (ki * _pos_integral) + (kd * deriv);
                if (out_spd > max_v) out_spd = max_v;
                else if (out_spd < -max_v) out_spd = -max_v;

                sendMotionCommand(0.0f, 0.0f, out_spd, 0.0f, damping);
            }

        } else if (type == CMD_READ_PARAM) {
            // タイプ17: リトルエンディアン
            uint16_t index = (uint16_t)msg.data[0] | ((uint16_t)msg.data[1] << 8);
            uint32_t val = (uint32_t)msg.data[4] |
                           ((uint32_t)msg.data[5] << 8) |
                           ((uint32_t)msg.data[6] << 16) |
                           ((uint32_t)msg.data[7] << 24);
            update_param_cache(index, val);

        } else if (type == CMD_FAULT_FB) {
            // タイプ21: リトルエンディアン
            uint32_t fault_raw = (uint32_t)msg.data[0] |
                                 ((uint32_t)msg.data[1] << 8) |
                                 ((uint32_t)msg.data[2] << 16) |
                                 ((uint32_t)msg.data[3] << 24);
            CriticalSectionLock lock;
            _fault_status.raw = fault_raw;
        }
    }
}

void RobStride::sendMotionCommand(float torque, float position, float velocity, float kp, float kd) {
    CANMessage msg;
    msg.format = CANExtended;
    msg.len = 8;

    uint16_t t_int  = float_to_uint(torque, T_MIN, T_MAX, 16);
    uint16_t p_int  = float_to_uint(position, P_MIN, P_MAX, 16);
    uint16_t v_int  = float_to_uint(velocity, V_MIN, V_MAX, 16);
    uint16_t kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 16);
    uint16_t kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 16);

    msg.id = (CMD_MOTION_CTRL << 24) | (t_int << 8) | _motor_id;

    // タイプ1: ビッグエンディアン
    msg.data[0] = (uint8_t)(p_int >> 8);
    msg.data[1] = (uint8_t)(p_int & 0xFF);
    msg.data[2] = (uint8_t)(v_int >> 8);
    msg.data[3] = (uint8_t)(v_int & 0xFF);
    msg.data[4] = (uint8_t)(kp_int >> 8);
    msg.data[5] = (uint8_t)(kp_int & 0xFF);
    msg.data[6] = (uint8_t)(kd_int >> 8);
    msg.data[7] = (uint8_t)(kd_int & 0xFF);

    write_can_retry(msg);
}

void RobStride::enable() {
    CANMessage msg;
    msg.format = CANExtended;
    msg.id = (CMD_ENABLE << 24) | (_master_id << 8) | _motor_id;
    msg.len = 8;
    memset(msg.data, 0, 8);
    write_can_retry(msg);
}

void RobStride::disable() {
    CANMessage msg;
    msg.format = CANExtended;
    msg.id = (CMD_DISABLE << 24) | (_master_id << 8) | _motor_id;
    msg.len = 8;
    memset(msg.data, 0, 8);
    write_can_retry(msg);
}

void RobStride::setZero() {
    CANMessage msg;
    msg.format = CANExtended;
    msg.id = (CMD_SET_ZERO << 24) | (_master_id << 8) | _motor_id;
    msg.len = 8;
    memset(msg.data, 0, 8);
    msg.data[0] = 1;
    write_can_retry(msg);
}

void RobStride::clearFault() {
    CANMessage msg;
    msg.format = CANExtended;
    msg.id = (CMD_DISABLE << 24) | (_master_id << 8) | _motor_id;
    msg.len = 8;
    memset(msg.data, 0, 8);
    msg.data[0] = 1;
    write_can_retry(msg);
}

void RobStride::setRunMode(RunMode mode) {
    {
        CriticalSectionLock lock;
        _current_run_mode = mode;
    }
    sendParam((uint16_t)IDX_RUN_MODE, static_cast<uint8_t>(mode));
}

RunMode RobStride::getRunMode() {
    CriticalSectionLock lock;
    return _current_run_mode;
}

// PP modeのみ実行可能
void RobStride::setPosition(float position_rad) {
    {
        CriticalSectionLock lock;
        if (_current_run_mode != RunMode::PP) return;
    }
    sendParam((uint16_t)IDX_LOC_REF, position_rad);
}

// Velocity modeのみ実行可能
void RobStride::setVelocity(float velocity_rad_s) {
    {
        CriticalSectionLock lock;
        if (_current_run_mode != RunMode::VELOCITY) return;
    }
    sendParam((uint16_t)IDX_SPD_REF, velocity_rad_s);
}

// Current modeのみ実行可能
void RobStride::setCurrent(float current_a) {
    {
        CriticalSectionLock lock;
        if (_current_run_mode != RunMode::CURRENT) return;
    }
    sendParam((uint16_t)IDX_IQ_REF, current_a);
}

void RobStride::setCurrentLimit(float current_limit_a) {
    sendParam((uint16_t)IDX_LIMIT_CUR, current_limit_a);
}

void RobStride::setAcceleration(float acc_rad_s2) {
    sendParam((uint16_t)IDX_ACC_RAD, acc_rad_s2);
}

void RobStride::setMaxSpeedPP(float speed_rad_s) {
    sendParam((uint16_t)IDX_VEL_MAX, speed_rad_s);
}

void RobStride::setAccelerationPP(float acc_rad_s2) {
    sendParam((uint16_t)IDX_ACC_SET, acc_rad_s2);
}

void RobStride::setCanTimeout(uint32_t timeout_val) {
    sendParam((uint16_t)IDX_CAN_TIMEOUT, timeout_val);
}

// タイプ18: リトルエンディアン
void RobStride::sendParam(uint16_t index, float value) {
    CANMessage msg;
    msg.format = CANExtended;
    msg.id = (CMD_WRITE_PARAM << 24) | (_master_id << 8) | _motor_id;
    msg.len = 8;
    memset(msg.data, 0, 8);
    msg.data[0] = (uint8_t)(index & 0xFF);
    msg.data[1] = (uint8_t)(index >> 8);
    memcpy(&msg.data[4], &value, 4);
    write_can_retry(msg);
}

void RobStride::sendParam(uint16_t index, uint8_t value) {
    CANMessage msg;
    msg.format = CANExtended;
    msg.id = (CMD_WRITE_PARAM << 24) | (_master_id << 8) | _motor_id;
    msg.len = 8;
    memset(msg.data, 0, 8);
    msg.data[0] = (uint8_t)(index & 0xFF);
    msg.data[1] = (uint8_t)(index >> 8);
    msg.data[4] = value;
    write_can_retry(msg);
}

void RobStride::sendParam(uint16_t index, uint16_t value) {
    CANMessage msg;
    msg.format = CANExtended;
    msg.id = (CMD_WRITE_PARAM << 24) | (_master_id << 8) | _motor_id;
    msg.len = 8;
    memset(msg.data, 0, 8);
    msg.data[0] = (uint8_t)(index & 0xFF);
    msg.data[1] = (uint8_t)(index >> 8);
    msg.data[4] = (uint8_t)(value & 0xFF);
    msg.data[5] = (uint8_t)(value >> 8);
    write_can_retry(msg);
}

void RobStride::sendParam(uint16_t index, uint32_t value) {
    CANMessage msg;
    msg.format = CANExtended;
    msg.id = (CMD_WRITE_PARAM << 24) | (_master_id << 8) | _motor_id;
    msg.len = 8;
    memset(msg.data, 0, 8);
    msg.data[0] = (uint8_t)(index & 0xFF);
    msg.data[1] = (uint8_t)(index >> 8);
    memcpy(&msg.data[4], &value, 4);
    write_can_retry(msg);
}

// タイプ17: リトルエンディアン
void RobStride::requestParam(uint16_t index) {
    CANMessage msg;
    msg.format = CANExtended;
    msg.id = (CMD_READ_PARAM << 24) | (_master_id << 8) | _motor_id;
    msg.len = 8;
    memset(msg.data, 0, 8);
    msg.data[0] = (uint8_t)(index & 0xFF);
    msg.data[1] = (uint8_t)(index >> 8);
    write_can_retry(msg);
}

void RobStride::update_param_cache(uint16_t index, uint32_t val) {
    CriticalSectionLock lock;
    for (size_t i = 0; i < CACHE_SIZE; ++i) {
        if (_param_caches[i].valid && _param_caches[i].index == index) {
            _param_caches[i].raw_val = val;
            return;
        }
    }
    for (size_t i = 0; i < CACHE_SIZE; ++i) {
        if (!_param_caches[i].valid) {
            _param_caches[i].index = index;
            _param_caches[i].raw_val = val;
            _param_caches[i].valid = true;
            return;
        }
    }
}

bool RobStride::getParamCache(uint16_t index, float &value) {
    CriticalSectionLock lock;
    for (size_t i = 0; i < CACHE_SIZE; ++i) {
        if (_param_caches[i].valid && _param_caches[i].index == index) {
            memcpy(&value, &_param_caches[i].raw_val, sizeof(float));
            return true;
        }
    }
    return false;
}

bool RobStride::getParamCache(uint16_t index, uint32_t &value) {
    CriticalSectionLock lock;
    for (size_t i = 0; i < CACHE_SIZE; ++i) {
        if (_param_caches[i].valid && _param_caches[i].index == index) {
            value = _param_caches[i].raw_val;
            return true;
        }
    }
    return false;
}

void RobStride::setPositionGains(float kp, float ki, float kd, float max_speed_rad_s, float damping_kd) {
    CriticalSectionLock lock;
    _pos_kp = kp;
    _pos_ki = ki;
    _pos_kd = kd;
    _max_output_spd = max_speed_rad_s;
    _damping_kd = damping_kd;
}

void RobStride::setTargetAngle(float angle_rad) {
    CriticalSectionLock lock;
    _target_angle = angle_rad;
}

void RobStride::setTargetAngleDeg(float angle_deg) {
    setTargetAngle(angle_deg * ((float)M_PI / 180.0f));
}

void RobStride::setAngleLimits(float min_rad, float max_rad, bool enable) {
    CriticalSectionLock lock;
    _limit_min_rad = min_rad;
    _limit_max_rad = max_rad;
    _limit_enabled = enable;
}

bool RobStride::is_arrived(float tolerance_rad) {
    CriticalSectionLock lock;
    float current = _accumulated_angle - _origin_offset;
    float target = _target_angle;
    return std::fabs(target - current) <= tolerance_rad;
}

void RobStride::setOriginOffset(float offset_rad) {
    CriticalSectionLock lock;
    _origin_offset = offset_rad;
}

float RobStride::getInitialOffset() {
    CriticalSectionLock lock;
    return _origin_offset;
}

float RobStride::getPosition() {
    CriticalSectionLock lock;
    return _accumulated_angle - _origin_offset;
}

float RobStride::getRawAngle() {
    CriticalSectionLock lock;
    return _fb_raw_angle;
}

float RobStride::getVelocity() {
    CriticalSectionLock lock;
    return _fb_v;
}

float RobStride::getTorque() {
    CriticalSectionLock lock;
    return _fb_t;
}

float RobStride::getTemperature() {
    CriticalSectionLock lock;
    return _fb_temp;
}

bool RobStride::isMotorMode() {
    CriticalSectionLock lock;
    return (_mode_state == 2);
}

bool RobStride::has_error() {
    CriticalSectionLock lock;
    return (_fault_status.raw != 0);
}

bool RobStride::isTimeout(uint32_t timeout_ms) {
    CriticalSectionLock lock;
    if (!_has_received_any) return true;
    auto elapsed = Kernel::Clock::now() - _last_rx_time;
    return std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() > timeout_ms;
}

uint32_t RobStride::getTxRetryFailures() {
    CriticalSectionLock lock;
    return _tx_fail_count;
}

MotorFaultStatus RobStride::getFaultStatus() {
    CriticalSectionLock lock;
    return _fault_status;
}

void RobStride::setLimitTorque(float torque_nm) {
    if (torque_nm < 0.0f) torque_nm = 0.0f;
    if (torque_nm > T_MAX) torque_nm = T_MAX;
    sendParam((uint16_t)IDX_LIMIT_TORQ, torque_nm);
}