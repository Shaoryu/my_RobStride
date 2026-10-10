# RobStride Mbed OS 6 制御ライブラリ (EL05 対応)

RobStride 製 準直駆モータ（EL05 等）を Mbed OS 6 上で CAN 通信制御するための C++ ライブラリです。  
CAN 割込コンテキスト（ISR）と RTOS スレッドを分離した「rbms スタイル」の高信頼・低遅延アーキテクチャを採用しています。

---

## 1. ライブラリ概要とアーキテクチャ

### 特長
- **ISR セーフ設計:**  
  CAN 割込受信ハンドラ（`handle_message()`）内では、受信データを内部バッファへコピーし `rtos::EventFlags` をセットする極小処理のみを実施。排他制御には `CriticalSectionLock` を使用し、ISR 内でのデッドロックやハードフォールトを完全に排除。
- **自律的スレッド処理:**  
  クラス内部に専用 RTOS `Thread` を保持。受信通知イベントを待機し、スレッドコンテキスト内で生データパース、$-4\pi \sim 4\pi$ の跨ぎ判定による多回転累積ラジアン計算、Mbed 側位置 PID 演算、CAN 送信を一貫して実行。
- **高精度時間管理:**  
  位置 PID ループの経過時間 $dt$ には `mbed::Timer` の `elapsed_time()`（$\mu\text{s}$ 精度）を使用。通信途絶検知には `Kernel::Clock::now()` を使用。
- **厳格なエンディアン切り分け:**  
  - 通信タイプ 1（運控指令）＆ タイプ 2（フィードバック）: ビッグエンディアン
  - 通信タイプ 17（パラメータ読出）＆ 18（パラメータ書込）: リトルエンディアン
- **安全な Velocity 運控モード制御:**  
  Mbed 側で累積角度の位置 PID を計算し、モータへは運控モード（タイプ 1）で $p_{\text{set}}=0, K_p=0, v_{\text{set}}=\text{PID出力}, K_d=\text{ダンピング}, t_{\text{ff}}=0$ として指令を送出。

---

## 2. API リファレンス

### 初期化・ライフサイクル
- **`RobStride(CAN &can, uint8_t motor_id, uint8_t master_id = 0xFD, osPriority_t priority = osPriorityAboveNormal, uint32_t stack_size = 2048)`**  
  コンストラクタ。対象モータ ID、ホスト ID、内部スレッドの優先度・スタックサイズを指定。
- **`void start()`**  
  内部 PID タイマおよび RTOS 制御スレッドを起動。
- **`void enable()`**  
  通信タイプ 3 を送信し、モータを励磁・動作可能状態（Operation Enable）にする。
- **`void disable()`**  
  通信タイプ 4 を送信し、モータを脱力・停止状態（Reset / Disable）にする。
- **`void setZero()`**  
  通信タイプ 6 を送信し、現在の機械位置をモータ不揮発メモリの原点（0 rad）として記録。
- **`void clearFault()`**  
  通信タイプ 4（Byte0=1）を送信し、モータの異常・保護状態をリセット。

### 制御モード・目標値設定
- **`void setRunMode(RunMode mode)`**  
  動作モード（`RunMode::OPERATION`, `RunMode::PP`, `RunMode::VELOCITY`, `RunMode::CURRENT`, `RunMode::CSP`）を設定。
- **`RunMode getRunMode()`**  
  現在設定されている動作モードを取得。
- **`void setPosition(float position_rad)`**  
  PP モード時のみ有効。目標位置 [rad] を送信。
- **`void setVelocity(float velocity_rad_s)`**  
  Velocity モード時のみ有効。目標速度 [rad/s] を送信。
- **`void setCurrent(float current_a)`**  
  Current モード時のみ有効。目標トルク電流 $I_q$ [A] を送信。

### Mbed 側位置制御（運控モード用）
- **`void setPositionGains(float kp, float ki, float kd, float max_speed_rad_s, float damping_kd = 1.0f)`**  
  Mbed 側位置 PID ゲイン、最大出力速度 [rad/s]、モータ内部速度ダンピングゲイン $K_d$ を設定。
- **`void setTargetAngle(float angle_rad)`**  
  目標累積角度 [rad] を設定（運控モードで追従）。
- **`void setTargetAngleDeg(float angle_deg)`**  
  目標累積角度 [deg] を設定。
- **`void setAngleLimits(float min_rad, float max_rad, bool enable = true)`**  
  目標角度のソフトウェアリミット（クリッピング）を設定・有効化。
- **`bool is_arrived(float tolerance_rad = 0.05f)`**  
  目標角度と現在累積角度の誤差が指定閾値以内（到達）かどうかを判定。
- **`void setOriginOffset(float offset_rad)`**  
  累積角度の原点オフセットを手動上書き。
- **`float getInitialOffset()`**  
  初回通信時に自動記録された初期オフセット値を取得。

### モータ内部パラメータ・制限設定
- **`void setLimitTorque(float torque_nm)`**  
  運控モード等で有効なトルク制限 [N・m] を設定（Index: `0x700B`）。
- **`void setCurrentLimit(float current_limit_a)`**  
  速度・位置モード用の最大相電流制限 [A] を設定（Index: `0x7018`）。
- **`void setAcceleration(float acc_rad_s2)`**  
  速度モード用の加速度 [rad/s²] を設定（Index: `0x7022`）。
- **`void setMaxSpeedPP(float speed_rad_s)`**  
  PP モード用の最大速度 [rad/s] を設定（Index: `0x7024`）。
- **`void setAccelerationPP(float acc_rad_s2)`**  
  PP モード用の加速度 [rad/s²] を設定（Index: `0x7025`）。
- **`void setCanTimeout(uint32_t timeout_val)`**  
  モータ内部の CAN 通信タイムアウト閾値を設定（Index: `0x7028`、20000 = 1秒、0で無効）。

### パラメータ読出（非同期キャッシュ方式）
- **`void requestParam(uint16_t index)`**  
  通信タイプ 17 を送信し、指定インデックスのパラメータ読み出しを要求。
- **`bool getParamCache(uint16_t index, float &value)`**  
  受信キャッシュから float 値をノンブロッキングで取得（取得成功時 `true`）。
- **`bool getParamCache(uint16_t index, uint32_t &value)`**  
  受信キャッシュから uint32_t 値をノンブロッキングで取得。

### 状態監視
- **`float getPosition()`**  
  原点オフセット補正後の累積回転角度 [rad] を取得。
- **`float getRawAngle()`**  
  モータから返送された単回転生角度 [$-4\pi \sim 4\pi$ rad] を取得。
- **`float getVelocity()`**  
  現在の出力軸角速度 [rad/s] を取得。
- **`float getTorque()`**  
  現在の出力軸トルク [N・m] を取得。
- **`float getTemperature()`**  
  現在のコイル温度 [℃] を取得。
- **`bool isMotorMode()`**  
  モータが正常運転モード（Motor 状態）にあるか判定。
- **`bool has_error()`**  
  故障フラグが 1 つでも発生しているか判定。
- **`MotorFaultStatus getFaultStatus()`**  
  ビットフィールド構造体で詳細エラー状態（過熱、過電圧、過電流、脱調等）を取得。
- **`bool isTimeout(uint32_t timeout_ms = 500)`**  
  最終受信時刻から指定時間以上経過しているか判定。
- **`uint32_t getTxRetryFailures()`**  
  CAN 送信リトライ（最大 3 回）がすべて失敗した累計回数を取得。

---

## 3. 通常制御サンプルプログラム

```cpp
#include "mbed.h"
#include "CANManager.h"
#include "RobStride.h"

CAN can(PA_11, PA_12, 1000000); // CAN1: 1Mbps
CANManager can_manager(can);

// モータID: 1, ホストID: 0xFD
RobStride motor(can, 1, 0xFD);

int main() {
    // 受信レシーバ登録とCANManager開始
    can_manager.add_receiver(&motor);
    can_manager.start();

    // モータ制御スレッド開始
    motor.start();
    ThisThread::sleep_for(100ms);

    // モータ内部のCANタイムアウト設定 (1秒 = 20000)
    motor.setCanTimeout(20000);
    // 運控モード用トルク制限 (例: 4.0 Nm)
    motor.setLimitTorque(4.0f);

    // Mbed側位置PIDゲイン設定 (Kp, Ki, Kd, MaxSpeed[rad/s], DampingKd)
    motor.setPositionGains(5.0f, 0.0f, 0.05f, 15.0f, 1.0f);

    // 角度リミット設定 (例: ±10π rad)
    motor.setAngleLimits(-10.0f * M_PI, 10.0f * M_PI, true);

    // モータ励磁
    motor.enable();
    ThisThread::sleep_for(200ms);

    printf("Motor Enabled. Starting Control Loop...\n");

    float target_angles_deg[] = {90.0f, 180.0f, 360.0f, 0.0f};
    int target_idx = 0;

    while (true) {
        if (motor.isTimeout(500)) {
            printf("[WARNING] CAN Communication Timeout!\n");
        } else if (motor.has_error()) {
            MotorFaultStatus fault = motor.getFaultStatus();
            printf("[ERROR] Motor Fault: raw=0x%08lX (OverTemp:%d, OverVolt:%d, Stall:%d)\n",
                   fault.raw, fault.over_temp, fault.over_volt, fault.stall_overload);
            motor.clearFault();
        } else {
            // 目標角度更新
            motor.setTargetAngleDeg(target_angles_deg[target_idx]);

            printf("Pos: %.2f rad | Vel: %.2f rad/s | Trq: %.2f Nm | Temp: %.1f C\n",
                   motor.getPosition(), motor.getVelocity(), motor.getTorque(), motor.getTemperature());

            if (motor.is_arrived(0.05f)) { // 約2.8度以内
                printf("Reached target: %.1f deg\n", target_angles_deg[target_idx]);
                target_idx = (target_idx + 1) % 4;
                ThisThread::sleep_for(1s);
            }
        }

        ThisThread::sleep_for(50ms);
    }
}
```

---

## 4. モータ CAN ID 変更用サンプルプログラム

誤設定や意図しない ID 書き換えを防ぐため、ID 変更処理はライブラリ本体から独立した単機能スクリプトとして実行します。  
公式仕様「通信タイプ 7（設置電機 CAN_ID）」を使用します。

> **注意:** 変更対象のモータ **1台のみ** を CAN バスに接続した状態で実行してください。

```cpp
#include "mbed.h"

CAN can(PA_11, PA_12, 1000000); // 1Mbps

/**
 * @brief モータの CAN ID を変更する独立関数
 * @param current_id 変更前のモータ CAN ID
 * @param new_id     変更後のモータ CAN ID (1~127)
 * @param master_id  ホスト CAN ID (デフォルト: 0xFD)
 */
bool change_robstride_id(uint8_t current_id, uint8_t new_id, uint8_t master_id = 0xFD) {
    CANMessage msg;
    msg.format = CANExtended;
    msg.len = 8;
    memset(msg.data, 0, 8);

    // 通信タイプ 7:
    // Bit28~24: 0x07
    // Bit23~16: new_id (予定CAN ID)
    // Bit15~8 : master_id
    // Bit7~0  : current_id (対象モータID)
    msg.id = (0x07 << 24) | ((uint32_t)new_id << 16) | ((uint32_t)master_id << 8) | current_id;

    printf("Sending Change ID Command: [Current: %d] -> [New: %d] (CAN ID: 0x%08lX)\n",
           current_id, new_id, msg.id);

    for (int retry = 0; retry < 5; ++retry) {
        if (can.write(msg)) {
            printf("Command Sent successfully.\n");
            break;
        }
        ThisThread::sleep_for(10ms);
    }

    // 応答確認 (通信タイプ 0 のブロードキャストが返送される)
    Timer timeout_timer;
    timeout_timer.start();
    while (timeout_timer.elapsed_time() < 500ms) {
        CANMessage rx;
        if (can.read(rx)) {
            uint8_t type = (rx.id >> 24) & 0x1F;
            uint8_t res_id = rx.id & 0xFF;
            if (type == 0x00 && res_id == 0xFE) {
                printf("Success! Motor responded with UID. New CAN ID is %d.\n", new_id);
                return true;
            }
        }
        ThisThread::sleep_for(5ms);
    }

    printf("No confirmation response received. (Check connection and reboot motor)\n");
    return false;
}

int main() {
    ThisThread::sleep_for(1s);
    printf("=== RobStride CAN ID Changer ===\n");

    const uint8_t CURRENT_ID = 1;
    const uint8_t TARGET_ID  = 2;

    change_robstride_id(CURRENT_ID, TARGET_ID);

    while (true) {
        ThisThread::sleep_for(1s);
    }
}
```

---

## 5. 設計上の注意点とノウハウ

### 1. PID ゲインのチューニング指針
- ロボマスター系（M3508/M2006）の制御コードとは**単位系が根本的に異なります**。  
  ロボマスが `deg` / `rpm` / `電流値` であるのに対し、本ライブラリは **`rad` / `rad/s` / `rad/s`** です。  
  （$1\text{ rad} \approx 57.3^\circ$ のため、度単位のゲインをそのまま代入すると約 57 倍の過大ゲインとなり激しく発振します。）
- EL05 は 9:1 の準直駆（バックドライバビリティが高い）のため、モータ側の速度ダンピング `damping_kd`（$K_d = 0.5 \sim 1.5$ 程度）を効かせ、Mbed 側の $K_d$ は `0.0` 〜 `0.05` 前後の極小値から設定してください。

### 2. 運控モードとパラメータ制限の挙動
- `setCurrentLimit()`（`0x7018`）や `setAcceleration()`（`0x7022`）は、モータ内部の「速度・位置モード」専用パラメータです。運控モード（タイプ 1）実行中は内部補間器を通らないため適用されません。
- 運控モード時のトルク制限には **`setLimitTorque()`（`0x700B`）** を使用してください。

### 3. 通信断時のフェイルセーフ
- `motor.setCanTimeout(20000)`（1秒）を設定しておくことで、Mbed の停止や配線抜けが発生した際にモータ側が自律的に Reset モード（脱力）へ移行し、暴走を防止できます。