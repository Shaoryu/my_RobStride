#ifndef ROBSTRIDE_H
#define ROBSTRIDE_H

#include "mbed.h"
#include "CANManager.h"
#include "platform/CriticalSectionLock.h"
#include <cmath>
#include <chrono>

#ifndef M_PI
#define M_PI 3.1415926535
#endif
// 物理パラメータ制限 (-4π ~ 4π)
#define P_MIN (-4.0f * (float)M_PI)
#define P_MAX (4.0f * (float)M_PI)
#define V_MIN -50.0f
#define V_MAX 50.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -6.0f
#define T_MAX 6.0f

// パラメータインデックス (RobStride Manual)
#define IDX_RUN_MODE   0x7005
#define IDX_IQ_REF     0x7006 // 電流モード目標値 (-11 ~ 11A)
#define IDX_SPD_REF    0x700A // 速度モード目標値 (-50 ~ 50rad/s)
#define IDX_LIMIT_TORQ 0x700B // トルク制限
#define IDX_LOC_REF    0x7016 // 位置モード目標値 (rad)
#define IDX_LIMIT_SPD  0x7017 // 位置モード最大速度
#define IDX_LIMIT_CUR  0x7018 // 速度・位置モード電流制限
#define IDX_ACC_RAD    0x7022 // 速度モード加速度
#define IDX_VEL_MAX    0x7024 // PPモード 最大速度
#define IDX_ACC_SET    0x7025 // PPモード 加速度
#define IDX_CAN_TIMEOUT 0x7028 // CANタイムアウト値

// 通信タイプ
enum CommType : uint8_t {
    CMD_GET_ID       = 0x00,
    CMD_MOTION_CTRL  = 0x01,
    CMD_FEEDBACK     = 0x02,
    CMD_ENABLE       = 0x03,
    CMD_DISABLE      = 0x04,
    CMD_SET_ZERO     = 0x06,
    CMD_SET_CAN_ID   = 0x07,
    CMD_READ_PARAM   = 0x11,
    CMD_WRITE_PARAM  = 0x12,
    CMD_FAULT_FB     = 0x15
};

// 実行モード
enum class RunMode : uint8_t {
    OPERATION = 0,
    PP        = 1,
    VELOCITY  = 2,
    CURRENT   = 3,
    CSP       = 5
};

// タイプ2および21の障害フラグ
union MotorFaultStatus {
    uint32_t raw;
    struct __attribute__((packed)) {
        uint32_t over_temp      : 1; // bit0: 過熱
        uint32_t drv_fault      : 1; // bit1: ドライバ異常
        uint32_t under_volt     : 1; // bit2: 低電圧
        uint32_t over_volt      : 1; // bit3: 過電圧
        uint32_t b_phase_oc     : 1; // bit4: B相過電流
        uint32_t c_phase_oc     : 1; // bit5: C相過電流
        uint32_t reserved1      : 1; // bit6
        uint32_t uncalibrated   : 1; // bit7: 未校正
        uint32_t hw_id_fault    : 1; // bit8: ハード識別異常
        uint32_t pos_init_fault : 1; // bit9: 位置初期化異常
        uint32_t reserved2      : 4; // bit10-13
        uint32_t stall_overload : 1; // bit14: 脱調/過負荷
        uint32_t reserved3      : 1; // bit15
        uint32_t a_phase_oc     : 1; // bit16: A相過電流
        uint32_t reserved4      : 15;
    };
};

class RobStride : public CANReceiver {
public:
    RobStride(CAN &can, uint8_t motor_id, uint8_t master_id = 0xFD,
              osPriority_t priority = osPriorityAboveNormal, uint32_t stack_size = 2048);
    virtual ~RobStride() = default;

    void start();

    // 基本動作制御
    void enable();
    void disable();
    void setZero();
    void clearFault();

    // モード設定および各モード専用指令（モードチェックガード付き）
    void setRunMode(RunMode mode);
    RunMode getRunMode();
    void setPosition(float position_rad);     // PPモードのみ実行可能
    void setVelocity(float velocity_rad_s);   // VELOCITYモードのみ実行可能
    void setCurrent(float current_a);         // CURRENTモードのみ実行可能

    // 制限・プロファイル設定
    void setCurrentLimit(float current_limit_a); // 速度・位置モード電流制限 (0x7018)
    void setAcceleration(float acc_rad_s2);      // 速度モード加速度 (0x7022)
    void setMaxSpeedPP(float speed_rad_s);       // PPモード最大速度 (0x7024)
    void setAccelerationPP(float acc_rad_s2);    // PPモード加速度 (0x7025)
    void setCanTimeout(uint32_t timeout_val);    // 20000 = 1s (0x7028)
    void setLimitTorque(float torque_nm);        // 運控モード等のトルク制限 0~6Nm (0x700B)

    // 汎用パラメータ書き込み (Type 18)
    void sendParam(uint16_t index, float value);
    void sendParam(uint16_t index, uint8_t value);
    void sendParam(uint16_t index, uint16_t value);
    void sendParam(uint16_t index, uint32_t value);

    // 非同期パラメータ読み取り (Type 17)
    void requestParam(uint16_t index);
    bool getParamCache(uint16_t index, float &value);
    bool getParamCache(uint16_t index, uint32_t &value);

    // Mbed側位置PIDゲイン設定（運控モード用）
    void setPositionGains(float kp, float ki, float kd, float max_speed_rad_s, float damping_kd = 1.0f);

    // Mbed側目標累積角度設定・リミット
    void setTargetAngle(float angle_rad);
    void setTargetAngleDeg(float angle_deg);
    void setAngleLimits(float min_rad, float max_rad, bool enable = true);

    // 到達判定
    bool is_arrived(float tolerance_rad = 0.05f);

    // 原点オフセット操作
    void setOriginOffset(float offset_rad);
    float getInitialOffset();

    // 状態取得
    float getPosition();     // オフセット適用後の累積ラジアン
    float getRawAngle();     // 生角度 (-4π ~ 4π)
    float getVelocity();
    float getTorque();
    float getTemperature();
    bool isMotorMode();
    bool has_error();
    bool isTimeout(uint32_t timeout_ms = 500);
    uint32_t getTxRetryFailures();

    MotorFaultStatus getFaultStatus();

    // 運控モード生送信
    void sendMotionCommand(float torque, float position, float velocity, float kp, float kd);

    // 割り込み受信ハンドラ (ISRセーフ)
    virtual bool handle_message(const CANMessage &msg) override;

private:
    CAN &_can;
    uint8_t _motor_id;
    uint8_t _master_id;

    // RTOSスレッド・フラグ
    rtos::Thread _thread;
    rtos::EventFlags _event_flags;

    // 受信バッファ (handle_message用)
    CANMessage _rx_buffer;
    bool _has_new_msg;

    // 時間管理 (Chrono)
    Timer _pid_timer;
    rtos::Kernel::Clock::time_point _last_rx_time;
    bool _has_received_any;

    // CAN送信リトライ失敗数
    uint32_t _tx_fail_count;

    // モータ運転モード追跡
    RunMode _current_run_mode;

    // 累積角度計算
    bool _is_first_rx;
    float _last_raw_angle;
    float _accumulated_angle;
    float _origin_offset;

    // フィードバックステータス
    float _fb_raw_angle;
    float _fb_v;
    float _fb_t;
    float _fb_temp;
    uint8_t _mode_state;
    MotorFaultStatus _fault_status;

    // Mbed側位置PID制御
    float _target_angle;
    float _pos_kp;
    float _pos_ki;
    float _pos_kd;
    float _pos_integral;
    float _pos_prev_err;
    float _max_output_spd;
    float _damping_kd;

    // ソフトウェアリミット
    bool _limit_enabled;
    float _limit_min_rad;
    float _limit_max_rad;

    // パラメータキャッシュ
    static constexpr size_t CACHE_SIZE = 16;
    struct ParamCache {
        uint16_t index;
        uint32_t raw_val;
        bool valid;
    } _param_caches[CACHE_SIZE];

    void control_loop();
    bool write_can_retry(const CANMessage &msg);
    void update_param_cache(uint16_t index, uint32_t val);

    static int float_to_uint(float x, float x_min, float x_max, int bits);
    static float uint_to_float(int x_int, float x_min, float x_max, int bits);
};

#endif // ROBSTRIDE_H