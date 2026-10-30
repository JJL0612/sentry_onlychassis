/**
 * @file app_robot.c
 * @brief 哨兵机器人的业务控制逻辑
 */

#include "app_task.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "alg_swerve.h"
#include "app_gimbal_link.h"
#include "app_host_protocol.h"
#include "app_protocol.h"
#include "app_referee.h"
#include "bsp_fdcan.h"
#include "bsp_uart.h"
#include "cmsis_os2.h"
#include "dev_bmi088.h"
#include "dev_dji_dt7.h"
#include "dev_dji_motor.h"
#include "dev_dm_motor.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

/* The host-side source extraction tests intentionally compile the legacy
 * local control loop without the inter-board endpoint. The target build keeps
 * this enabled and uses the v4 path below. */
#define ROBOT_ENABLE_INTERBOARD_V4 1

// 哨兵机器人的Skywalker实现，集中覆盖四个RTOS任务入口。

// 仅用于不接电机时观察CAN指令；接回电机前必须改为0。
// 可用编译参数 -DROBOT_CHASSIS_NO_MOTOR_TX_TEST=0 覆盖。
#ifndef ROBOT_CHASSIS_NO_MOTOR_TX_TEST
#define ROBOT_CHASSIS_NO_MOTOR_TX_TEST 0
#endif

// VOFA 调参观测的 GM6020 CAN ID（1~4）；也可用编译参数覆盖。
#ifndef ROBOT_GM6020_TELEMETRY_ID
#define ROBOT_GM6020_TELEMETRY_ID 1
#endif
#if ROBOT_GM6020_TELEMETRY_ID < 1 || ROBOT_GM6020_TELEMETRY_ID > 4
#error "ROBOT_GM6020_TELEMETRY_ID must be in [1, 4]"
#endif

enum {
  // 1U: 接入裁判系统；0U: 不接入裁判系统，使用本文件内默认参数。
  kRobotRefereeEnabled = 0U,
  // 四组舵轮底盘：M3508 ID 1~4，GM6020 ID 1~4。
  kRobotChassisEnabled = 1U,
  // DT7在线时优先遥控；1U允许DT7离线时由UART7接管，0U离线停车。
  kRobotChassisHostControlEnabled = 1U,
  // 0U: 遥控平移按大 Yaw/云台坐标执行，使用底盘本地大 Yaw 关节角。
  // 1U 仅用于不接云台板的单底盘调试，此时按车身坐标平移。
  kRobotChassisStandaloneMode = 0U,
  // 非零旋转指令时启用小陀螺平移降速；换系角始终来自编码器。
  kRobotChassisSpinTranslationEnabled = 1U,
  // USART1（PA9 TX / PA10 RX）用于云台相对角/底盘状态通信。
  // 与 GM6020 FireWater 遥测和裁判串口互斥，下面已关闭遥测。
  kRobotGimbalLinkEnabled = 1U,
  // USART1 已被云台链路占用，GM6020 遥测关闭。
  kRobotGm6020TelemetryEnabled = 0U,
  kRobotGimbalEnabled = 0U,
  // 大 Yaw 由底盘板执行；陀螺仪补偿使用电机已配置的速度模式。
  kRobotBigYawEnabled = 1U,
  // 1U: 底盘角速度前馈 + 云台板载 IMU 航向/角速度闭环 + 滚轮转向。
  // 0U: 编码器位置/速度串级，保持相对底盘角度。
  kRobotBigYawGyroCompensationEnabled = 1U,
  // 1U: 无有效板间请求时，底盘本地 DT7 滚轮控制大 Yaw。
  kRobotBigYawRcFallback = 1U,
  kRobotShootEnabled = 0U,
  kRobotVisionEnabled = 0U,
  kRobotBigYawFeedbackTimeoutMs = 100U,
  kRobotBigYawFeedbackPeriodMs = 100U,
  kRobotBigYawEnableRetryMs = 500U,
  kRobotKeyboardMouseEnabled = 1U,
  kRobotRemoteSwitchSafeModeEnabled = 0U,
  kRobotManagerPeriodMs = 10,
  kRobotGimbalPeriodMs = 2,
  kRobotChassisPeriodMs = 2,
  kRobotShootPeriodMs = 2,
  kRobotChassisMotorCount = 4U,
  kRobotCanDataLength = 8,
  kRobotDt7OfflineTimeoutMs = 100,
  kRobotHostCommandTimeoutMs = 100,
  kRobotChassisNoMotorTxTest = ROBOT_CHASSIS_NO_MOTOR_TX_TEST,
  // 平移输入归中时仍允许左拨轮原地旋转；设为0也允许QE/小陀螺原地旋转。
  kRobotSteeringIdleOverridesYaw = 1U,
  kRobotImuGyroZAxis = 2,
};

_Static_assert(!(kRobotGimbalLinkEnabled && kRobotRefereeEnabled),
               "USART1 cannot serve gimbal link and referee simultaneously");
_Static_assert(!(kRobotGimbalLinkEnabled && kRobotGm6020TelemetryEnabled),
               "USART1 cannot serve gimbal link and GM6020 telemetry simultaneously");
_Static_assert(!(kRobotGimbalEnabled && kRobotBigYawEnabled),
               "yaw_motor cannot serve gimbal and big yaw simultaneously");
_Static_assert(!kRobotBigYawGyroCompensationEnabled ||
                   (kRobotBigYawEnabled && kRobotChassisEnabled),
               "Gyro compensation requires chassis IMU and big yaw control");

// 俯视底盘坐标：+X 向前，+Y 向左，正转角为逆时针。
// 默认方形底盘，沿用原0.15m中心到轮距离；必须按实际轮距修改。
static const float kRobotChassisHalfLengthM = 0.106066f;
static const float kRobotChassisHalfWidthM = 0.106066f;
static const float kRobotWheelRadiusM = 0.01f;
// M3508 到轮子的减速比：3591 / 187 ≈ 19.2。
// 轮半径仍沿用原工程的 0.01 m 占位值，必须按实车轮径核对。
static const float kRobotDriveReductionRatio = 3591.0f / 187.0f;
static const float kRobotChassisTranslationDeadband = 0.05f;
static const float kRobotChassisTranslationStartDeadband = 0.08f;
// 物理轮位数组顺序：左前、右前、左后、右后；以下编码值对应轮子朝 +X（车头）。
static const float kRobotSteeringForwardAngleRad[4] = {
    3060.0f / 8192.0f * 2.0f * (float)M_PI,  // 左前 ID1
    986.0f / 8192.0f * 2.0f * (float)M_PI,   // 右前 ID4
    2383.0f / 8192.0f * 2.0f * (float)M_PI,  // 左后 ID2
    4390.0f / 8192.0f * 2.0f * (float)M_PI,  // 右后 ID3
};
// 俯视左上角为 ID1，逆时针递增：
// ID1 左前、ID2 左后、ID3 右后、ID4 右前。
// 数组下标是 CAN 槽位（电机 ID - 1），值是物理轮位数组索引；
// 物理轮位数组顺序固定为左前、右前、左后、右后。
static const uint8_t kRobotChassisCanSlotWheelIndex[4] = {0U, 2U, 3U, 1U};
// 四个 GM6020 均倒置安装：相对前向零位的编码器角度偏移与轮组舵角相反。
// 平移、原地旋转和混合运动统一使用此安装符号，零位编码值保持不变。
static const float kRobotSteeringDirection[4] = {-1.0f, -1.0f, -1.0f, -1.0f};
// ID3、ID4 的 3508 平移方向实测与整体相反，四轮统一取 -1。
static const float kRobotDriveDirection[4] = {-1.0f, -1.0f, -1.0f, -1.0f};
static const float kRobotMaxVxMps = 5.0f;
static const float kRobotMaxVyMps = 5.0f;
// 四轮补偿后等比例限速，保持原直行速度上限，避免斜向合速度超限。
static const float kRobotChassisMaxWheelSpeedMps = 5.0f;
static const float kRobotSteeringCosineDeadbandRad = 0.05f;
static const float kRobotMaxOmegaRadps = 10.0f;
// 编码器串级模式的目标角速度限幅。
static const float kRobotBigYawMaxRateRadps = 1.0f;
// 电机正转与底盘正转物理方向相同时取 +1；反向安装取 -1。
static const float kRobotBigYawGyroDirection = -1.0f;
// 电机输出轴速度 / 大 Yaw 轴速度；直驱取 1，有外部减速需按实物配置。
static const float kRobotBigYawGyroRatio = 1.0f;
static const float kRobotBigYawGyroMaxRateRadps = 10.0f;
// 电机重新校准后，用户对齐车头；实时 CAN 五帧均为 -1.132792 rad（2026-10-28）。
// 不会将首次上电/重新使能的位置自动当成车头方向。
static const float kRobotBigYawForwardZeroRad = -1.132792f;
// J4310 MIT 力矩模式：电机端 kp/kd=0，应用层完成位置/速度串级。
static const float kRobotBigYawTorqueLimitNm = 3.0f;
static const float kRobotBigYawPositionOutputMaxRadps = 2.0f;
// 连续旋转关节用“反馈角 ±90°”限制目标超前量，避免跟随失败时目标
// 越过半圈后误差取最短分支导致力矩方向翻转。
static const float kRobotBigYawMaxPositionErrorRad = (float)M_PI / 2.0f;
static const float kRobotBigYawCascadeFallbackDtS = 0.002f;
static const float kRobotBigYawCascadeMaxDtS = 0.020f;
static const float kRobotChassisWheelMaxOmegaRadps = -14.0f;
static const float kRobotNoRefereeCurrentScale = 0.5f;
static const float kRobotChassisYawRateFeedbackKp = 0.5f;
static const float kRobotChassisAutoSpinOmegaRadps = 4.0f;
// 小陀螺：换系角不做预测，横移/纵移保留 40%/30%。
static const float kRobotChassisSpinLateralScale = 0.4f;
static const float kRobotChassisSpinForwardScale = 0.3f;
static const float kRobotChassisSpinCommandEpsilon = 0.0001f;
static const float kRobotChassisYawWheelDeadband = 0.05f;
static const float kRobotImuYawRateSign = 1.0f;
static const float kRobotGimbalYawFieldKp = 6.0f;
static const float kRobotGimbalYawMotorSign = 1.0f;
static const float kRobotGimbalYawRcSign = -1.0f;
static const float kRobotGimbalYawMaxOmegaRadps = 3.0f;
static const float kRobotGimbalYawDeadband = 0.05f;
static const float kRobotGimbalPitchRcSign = 1.0f;
static const float kRobotGimbalPitchMinRad = -(float)M_PI / 4.0f;
static const float kRobotGimbalPitchMaxRad = (float)M_PI / 4.0f;
static const float kRobotGimbalPitchMaxOmegaRadps = 1.5f;
static const float kRobotGimbalPitchDeadband = 0.05f;
static const float kRobotMouseYawRadPerCount = 0.0015f;
static const float kRobotMousePitchRadPerCount = 0.0008f;
static const float kRobotImuHeaterTargetTemperatureC = 40.0f;
static const float kRobotPitchOffsetRad = -1.66286434f;
static const float kRobotBulletFeedOmegaRadps = 910.0f;
static const float kRobotFrictionOmegaRadps = 800.0f;
static const float kRobotDefaultChassisPowerW = 100.0f;

// M3508底盘功率估计参数，保留旧车测试得到的基础模型。
static const float kRobotPowerK0 = 0.30f;
static const float kRobotPowerK1 = 0.37f;
static const float kRobotPowerK2 = 8.30f;
static const float kRobotPowerK3 = 0.0f;

// 启动前先对齐；普通运动使用误差滞回，小陀螺运行中使用每轮 cos³ 降速。
static const float kRobotSteeringDriveEnableErrorRad = 0.10f;
static const float kRobotSteeringDriveDisableErrorRad = 0.40f;
// 合速度接近零时 atan2 对噪声非常敏感，使用滞回保持上一舵角。
static const float kRobotSteeringVectorHoldSpeedMps = 0.05f;
static const float kRobotSteeringVectorResumeSpeedMps = 0.10f;
// 舵角目标按位置环允许的最大速度推进，并在 180 度附近锁定旋转方向。
static const float kRobotSteeringTargetRateMaxRadps = 4.0f;
static const float kRobotSteeringDirectionLockBandRad = 0.10f;
enum { kRobotChassisFeedbackTimeoutMs = 50U };

static PidConfig g_robot_bmi088_heater_pid = {
    .kp = 0.20f,
    .ki = 0.03f,
    .kd = 0.0f,
    .sampling_time = 0.002f,
    .is_antiwindup_enabled = true,
    .output_max = 1.0f,
    .integral_max = 10.0f,
};

typedef struct {
  float vx;      // m/s，+X 向前。
  float vy;      // m/s，+Y 向左。
  float omega_z; // rad/s，逆时针为正。
} RobotVelocity;

typedef struct {
  RobotVelocity desired_velocity;
  RobotVelocity actual_velocity;
  float estimated_power;
  float yaw_angle;
  float target_yaw_angle;
  float yaw_rate;
  float control_dt;
  Bmi088Status imu_status;
  bool imu_ready;
  bool translation_stick_active;
  bool steering_idle;
  bool spin_translation_active;
  bool spin_drive_tracking;
  float spin_translation_relative_yaw;
  bool big_yaw_heading_valid;
  float big_yaw_encoder_angle_rad;
  float big_yaw_relative_yaw_rad;
  float big_yaw_relative_rate_radps;
  uint32_t big_yaw_encoder_age_ms;
  uint32_t imu_sample_count;
  uint32_t imu_error_count;
} RobotChassisState;

typedef struct {
  float yaw_angle;
  float target_yaw_angle;
  float pitch_angle;
  float yaw_rate;
  float pitch_rate;
} RobotGimbalState;

typedef struct {
  DjiMotor chassis_motor_3508[kRobotChassisMotorCount];
  DjiMotor chassis_motor_6020[kRobotChassisMotorCount];
  volatile uint32_t chassis_3508_last_feedback_tick[kRobotChassisMotorCount];
  volatile uint32_t chassis_6020_last_feedback_tick[kRobotChassisMotorCount];
  volatile bool chassis_3508_has_feedback[kRobotChassisMotorCount];
  volatile bool chassis_6020_has_feedback[kRobotChassisMotorCount];
  bool chassis_drive_steering_ready;
  bool chassis_host_control_active;
  bool chassis_interboard_control_active;
  bool chassis_steering_vector_active[kRobotChassisMotorCount];
  bool chassis_steering_target_initialized[kRobotChassisMotorCount];
  float chassis_steering_requested_angle[kRobotChassisMotorCount];
  float chassis_steering_last_direction[kRobotChassisMotorCount];
  DjiMotor friction_motor[2];
  DjiMotor bullet_feed_motor;
  DjiMotor pitch_motor;
  DmMotor yaw_motor;
  // Encoder protocol wraps every 25 rad, not every physical revolution.
  volatile bool big_yaw_encoder_initialized;
  volatile float big_yaw_encoder_previous_raw;
  volatile int32_t big_yaw_encoder_wraps;
  volatile double big_yaw_encoder_continuous_rad;
  PidObject big_yaw_position_pid;
  PidObject big_yaw_velocity_pid;
  bool big_yaw_heading_initialized;
  uint64_t big_yaw_heading_boot_id;
  uint32_t big_yaw_heading_zero_generation;
  float big_yaw_heading_target_rad;
  float big_yaw_heading_error_rad;
  float big_yaw_world_rate_target_radps;
  float big_yaw_rate_integral;
  float big_yaw_previous_operator_rate;
  bool big_yaw_target_initialized;
  float big_yaw_target_angle;
  float big_yaw_target_rate;
  float big_yaw_torque;
  uint32_t big_yaw_cascade_last_tick;
  RobotChassisState chassis;
  RobotGimbalState gimbal;
  uint16_t bullet_count;
  uint32_t chassis_imu_last_tick;
  uint32_t gimbal_last_tick;
  volatile uint32_t yaw_feedback_tick;
  volatile bool yaw_feedback_valid;
  bool big_yaw_request_valid;
  bool big_yaw_manual_active;
  bool big_yaw_gyro_authorized;
  bool big_yaw_gyro_active;
  bool big_yaw_active;
  bool big_yaw_enabled;
  bool big_yaw_feedback_lost;
  bool big_yaw_fault_latched;
  uint8_t big_yaw_enable_repeat;
  uint32_t big_yaw_enable_tick;
  uint32_t big_yaw_last_control_tick;
  uint32_t big_yaw_last_command_sequence;
  uint32_t big_yaw_feedback_last_tick;
  uint32_t keyboard_mouse_last_frame_count;
  volatile bool is_initialized;
  volatile bool is_initializing;
} RobotApp;

static RobotApp g_robot;

static void RobotBigYawControl(void);

static float RobotClamp(float value, float min, float max) {
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}

static float RobotRcNormalize(int16_t value) {
  return RobotClamp((float)value / 660.0f, -1.0f, 1.0f);
}

static float RobotApplyDeadband(float value, float deadband) {
  if (value > -deadband && value < deadband) {
    return 0.0f;
  }
  return value;
}

static bool RobotAutoSpinIsEnabled(void) {
  return SwitchIsUp(g_dt7_object.rocker.sw1);
}

static float RobotWrapPi(float angle) {
  while (angle > (float)M_PI) {
    angle -= 2.0f * (float)M_PI;
  }
  while (angle < -(float)M_PI) {
    angle += 2.0f * (float)M_PI;
  }
  return angle;
}

static bool RobotChassisStickActive(float vx, float vy, bool was_active) {
  float stick_magnitude = fmaxf(fabsf(vx), fabsf(vy));
  return was_active ? stick_magnitude >= kRobotChassisTranslationDeadband
                    : stick_magnitude > kRobotChassisTranslationStartDeadband;
}

static float RobotSteeringTargetAngle(uint8_t index,
                                      float angle_from_forward) {
  // 安装方向统一由每轮配置决定，平移和旋转使用同一套坐标变换。
  return RobotWrapPi(kRobotSteeringForwardAngleRad[index] +
                     kRobotSteeringDirection[index] * angle_from_forward);
}

static bool RobotSteeringVectorActive(float wheel_speed, bool was_active) {
  return was_active ? wheel_speed >= kRobotSteeringVectorHoldSpeedMps
                    : wheel_speed > kRobotSteeringVectorResumeSpeedMps;
}

static void RobotSteeringUpdateTarget(uint8_t index, DjiMotor *steer) {
  if (!g_robot.chassis_steering_target_initialized[index]) {
    steer->state.given_angle = steer->state.angle;
    g_robot.chassis_steering_last_direction[index] = 0.0f;
    g_robot.chassis_steering_target_initialized[index] = true;
  }

  float target_error = RobotWrapPi(
      g_robot.chassis_steering_requested_angle[index] -
      steer->state.given_angle);
  if (fabsf(fabsf(target_error) - (float)M_PI) <
          kRobotSteeringDirectionLockBandRad &&
      g_robot.chassis_steering_last_direction[index] != 0.0f) {
    target_error = copysignf(fabsf(target_error),
                             g_robot.chassis_steering_last_direction[index]);
  }

  float max_step = kRobotSteeringTargetRateMaxRadps *
                   (float)kRobotChassisPeriodMs * 0.001f;
  float target_step = RobotClamp(target_error, -max_step, max_step);
  if (target_step != 0.0f) {
    g_robot.chassis_steering_last_direction[index] = target_step;
  }
  steer->state.given_angle =
      RobotWrapPi(steer->state.given_angle + target_step);
}

static bool RobotChassisFeedbackValid(bool has_feedback,
                                      uint32_t last_feedback_tick,
                                      uint32_t now) {
  return kRobotChassisNoMotorTxTest != 0U ||
         (has_feedback &&
          (uint32_t)(now - last_feedback_tick) <=
              kRobotChassisFeedbackTimeoutMs);
}

static bool RobotSteeringDriveReady(void) {
  if (kRobotChassisNoMotorTxTest != 0U) {
    return true;
  }
  uint32_t now = osKernelGetTickCount();
  if (!g_robot.chassis.spin_translation_active) {
    g_robot.chassis.spin_drive_tracking = false;
  }
  float max_error = 0.0f;
  for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
    const DjiMotor *steer = &g_robot.chassis_motor_6020[i];
    if (!RobotChassisFeedbackValid(g_robot.chassis_6020_has_feedback[i],
                                   g_robot.chassis_6020_last_feedback_tick[i],
                                   now)) {
      g_robot.chassis_drive_steering_ready = false;
      g_robot.chassis.spin_drive_tracking = false;
      return false;
    }
    // 检查最终优化目标，避免斜坡目标贴近反馈时提前放行驱动轮。
    float error = fabsf(RobotWrapPi(g_robot.chassis_steering_requested_angle[i] -
                                      steer->state.angle));
    max_error = fmaxf(max_error, error);
  }
  if (g_robot.chassis.spin_translation_active) {
    // 小陀螺每次进入/停车恢复后先对齐。运行时目标连续变化，跟踪误差
    // 由各轮 cos³ 补偿降低驱动力，不用普通运动的 0.4 rad 阈值切断全车。
    if (!g_robot.chassis.spin_drive_tracking &&
        max_error < kRobotSteeringDriveEnableErrorRad) {
      g_robot.chassis.spin_drive_tracking = true;
    }
    g_robot.chassis_drive_steering_ready = g_robot.chassis.spin_drive_tracking;
    return g_robot.chassis_drive_steering_ready;
  }
  if (g_robot.chassis_drive_steering_ready) {
    if (max_error > kRobotSteeringDriveDisableErrorRad) {
      g_robot.chassis_drive_steering_ready = false;
    }
  } else if (max_error < kRobotSteeringDriveEnableErrorRad) {
    g_robot.chassis_drive_steering_ready = true;
  }
  return g_robot.chassis_drive_steering_ready;
}

static void RobotUpdateKeyboardMouseInput(void) {
  if (kRobotKeyboardMouseEnabled == 0U) {
    g_robot.keyboard_mouse_last_frame_count = Dt7FrameCount();
    return;
  }

  uint32_t frame_count = Dt7FrameCount();
  if (frame_count == g_robot.keyboard_mouse_last_frame_count) {
    return;
  }
  g_robot.keyboard_mouse_last_frame_count = frame_count;

  g_robot.gimbal.target_yaw_angle = RobotWrapPi(
      g_robot.gimbal.target_yaw_angle +
      (float)g_dt7_object.mouse.x * kRobotMouseYawRadPerCount);
  g_robot.gimbal.pitch_angle -=
      (float)g_dt7_object.mouse.y * kRobotMousePitchRadPerCount;
}

static float RobotChassisYawRateCommand(float wheel_input) {
  float wheel_yaw_rate = -wheel_input * kRobotChassisWheelMaxOmegaRadps;
  float key_yaw_rate = ((float)g_dt7_object.key.e -
                        (float)g_dt7_object.key.q) * kRobotMaxOmegaRadps;
  return RobotClamp(wheel_yaw_rate + key_yaw_rate,
                    -kRobotMaxOmegaRadps, kRobotMaxOmegaRadps);
}

static float RobotGimbalYawRateCommand(void) {
  float yaw_input = RobotApplyDeadband(
      RobotRcNormalize(g_dt7_object.rocker.ch0),
      kRobotGimbalYawDeadband);
  float yaw_rate =
      kRobotGimbalYawRcSign * yaw_input * kRobotGimbalYawMaxOmegaRadps;
  return RobotClamp(yaw_rate, -kRobotGimbalYawMaxOmegaRadps,
                       kRobotGimbalYawMaxOmegaRadps);
}

static float RobotGimbalPitchRateCommand(void) {
  float pitch_input = RobotApplyDeadband(
      RobotRcNormalize(g_dt7_object.rocker.ch1),
      kRobotGimbalPitchDeadband);
  float pitch_rate = kRobotGimbalPitchRcSign * pitch_input *
                     kRobotGimbalPitchMaxOmegaRadps;
  return RobotClamp(pitch_rate, -kRobotGimbalPitchMaxOmegaRadps,
                       kRobotGimbalPitchMaxOmegaRadps);
}

static void RobotPackDjiCurrents(const DjiMotor *motor_0,
                                    const DjiMotor *motor_1,
                                    const DjiMotor *motor_2,
                                    const DjiMotor *motor_3,
                                    uint8_t tx_data[8]) {
  const DjiMotor *motors[4] = {motor_0, motor_1, motor_2, motor_3};
  memset(tx_data, 0, 8);
  for (uint8_t i = 0; i < 4U; i++) {
    if (motors[i] == NULL) {
      continue;
    }
    int16_t current_units = DjiMotorCurrentAmperesToUnits(
        motors[i]->type, motors[i]->state.given_current);
    tx_data[i * 2U] = (uint8_t)(current_units >> 8);
    tx_data[i * 2U + 1U] = (uint8_t)(current_units & 0xFFU);
  }
}

// FireWater 纯数字 CSV，10 ms 一行；当前调试输出：
// 左摇杆 ch3、6020 请求角、PID 目标角、实际角、反馈有效。
static uint16_t RobotFormatGm6020Telemetry(char *buffer, size_t capacity) {
  if (buffer == NULL || capacity == 0U) {
    return 0U;
  }
  const uint8_t index =
      kRobotChassisCanSlotWheelIndex[ROBOT_GM6020_TELEMETRY_ID - 1U];
  // 短临界区复制电机与控制状态，浮点计算和格式化在临界区外进行。
  uint32_t lock = __get_PRIMASK();
  __disable_irq();
  int16_t left_stick_raw = g_dt7_object.rocker.ch3;
  DjiMotorState state = g_robot.chassis_motor_6020[index].state;
  DjiMotorMode mode = g_robot.chassis_motor_6020[index].mode;
  float requested_angle = g_robot.chassis_steering_requested_angle[index];
  bool target_initialized = g_robot.chassis_steering_target_initialized[index];
  bool has_feedback = g_robot.chassis_6020_has_feedback[index];
  uint32_t feedback_tick = g_robot.chassis_6020_last_feedback_tick[index];
  uint32_t now = osKernelGetTickCount();
  __set_PRIMASK(lock);

  uint32_t age_ms = has_feedback ? now - feedback_tick : 0U;
  bool feedback_valid = has_feedback && age_ms <= kRobotChassisFeedbackTimeoutMs;
  bool active = feedback_valid && target_initialized && state.is_enabled;
  bool position_active = active && mode == kDjiMotorModePosition;
  // 给请求角选择距当前反馈最近的连续角分支；有效目标使用 PID 实际 reference。
  float requested_continuous = state.continuous_angle;
  if (active) {
    requested_continuous += RobotWrapPi(requested_angle - state.angle);
  }
  float target_continuous = position_active ? state.position_loop.reference
                                           : state.continuous_angle;
  float left_stick = RobotRcNormalize(left_stick_raw);
  int length = snprintf(buffer, capacity, "%.4f,%.4f,%.4f,%.4f,%u\n",
                        (double)left_stick, (double)requested_continuous,
                        (double)target_continuous,
                        (double)state.continuous_angle,
                        (unsigned)feedback_valid);
  if (length < 0 || (size_t)length >= capacity) {
    return 0U;
  }
  return (uint16_t)length;
}

static void RobotSendGm6020Telemetry(void) {
  if (huart1.gState != HAL_UART_STATE_READY) {
    return;
  }
  static char tx_data[kUartBufferSize];
  uint16_t length = RobotFormatGm6020Telemetry(tx_data, sizeof(tx_data));
  if (length != 0U) {
    UartTransmit(&huart1, (uint8_t *)tx_data, length);
  }
}

// FDCAN1 接收底盘 GM6020 ID 1~4 的反馈（0x205~0x208）。
static void RobotFdcan1RxCallback(uint32_t std_id, uint8_t *rx_buffer) {
  if (std_id >= 0x205U && std_id <= 0x208U) {
    uint8_t can_slot = (uint8_t)(std_id - 0x205U);
    uint8_t index = kRobotChassisCanSlotWheelIndex[can_slot];
    DjiMotorUpdate(&g_robot.chassis_motor_6020[index], rx_buffer);
    g_robot.chassis_6020_last_feedback_tick[index] = osKernelGetTickCount();
    g_robot.chassis_6020_has_feedback[index] = true;
  }
}

/* 底盘 3508 与 6020 使用同一组轮组 ID，共用 CAN 槽位到物理轮位的映射。 */
static void RobotUpdateChassis3508(uint32_t std_id, uint8_t *rx_buffer) {
  if (std_id >= 0x201U && std_id <= 0x204U) {
    uint8_t can_slot = (uint8_t)(std_id - 0x201U);
    uint8_t index = kRobotChassisCanSlotWheelIndex[can_slot];
    DjiMotorUpdate(&g_robot.chassis_motor_3508[index], rx_buffer);
    g_robot.chassis_3508_last_feedback_tick[index] = osKernelGetTickCount();
    g_robot.chassis_3508_has_feedback[index] = true;
  }
}

static void RobotFdcan2RxCallback(uint32_t std_id, uint8_t *rx_buffer) {
  /* CAN2 承载底盘 3508 和云台 pitch；舵向 6020 在 CAN1，大 Yaw 在 CAN3。
   * 发射机构开启时 0x201/0x202 属于摩擦轮；底盘单板调试时
   * kRobotShootEnabled=0，这两路反馈归底盘 3508。 */
  if (kRobotShootEnabled != 0U &&
      (std_id == 0x201U || std_id == 0x202U)) {
    DjiMotorUpdate(&g_robot.friction_motor[std_id - 0x201U], rx_buffer);
    return;
  }
  if (std_id >= 0x201U && std_id <= 0x204U) {
    RobotUpdateChassis3508(std_id, rx_buffer);
    return;
  }
  if (std_id == 0x20BU) {
    DjiMotorUpdate(&g_robot.pitch_motor, rx_buffer);
  }
}

// Called for every valid CAN sample, so crossings between control ticks are retained.
// Initial sample selects the current protocol branch; power-off turns are unknown.
static void RobotBigYawEncoderUpdate(void) {
  const float raw = g_robot.yaw_motor.state.angle;
  if (g_robot.yaw_motor.state.error_code != 0U || !isfinite(raw) ||
      raw < kDmMotorPositionMin || raw > kDmMotorPositionMax) return;
  const float span = kDmMotorPositionMax - kDmMotorPositionMin;
  if (g_robot.big_yaw_encoder_initialized) {
    const float delta = raw - g_robot.big_yaw_encoder_previous_raw;
    if (delta < -0.5f * span) ++g_robot.big_yaw_encoder_wraps;
    else if (delta > 0.5f * span) --g_robot.big_yaw_encoder_wraps;
  } else {
    g_robot.big_yaw_encoder_wraps = 0;
    g_robot.big_yaw_encoder_initialized = true;
  }
  g_robot.big_yaw_encoder_previous_raw = raw;
  g_robot.big_yaw_encoder_continuous_rad =
      (double)raw + (double)g_robot.big_yaw_encoder_wraps * (double)span;
}

static void RobotFdcan3RxCallback(uint32_t std_id, uint8_t *rx_buffer) {
  /* CAN3 承载大 Yaw 和拨弹电机反馈。 */
  if (std_id == 0x205U) {
    DjiMotorUpdate(&g_robot.bullet_feed_motor, rx_buffer);
    return;
  }
  if (std_id == g_robot.yaw_motor.state.id ||
      std_id == g_robot.yaw_motor.state.master_id) {
    DmMotorUpdate(&g_robot.yaw_motor, rx_buffer);
    RobotBigYawEncoderUpdate();
    g_robot.yaw_feedback_tick = HAL_GetTick();
    g_robot.yaw_feedback_valid = true;
  }
}

static void RobotMotorInit(void) {
  static PidConfig m2006_speed_pid = {
      .kp = 1.515502929687f,
      .ki = 1.165771484375f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_antiwindup_enabled = true,
      .output_max = 5.0f,
      .integral_max = 6.103515625f,
  };
  static PidConfig m3508_speed_pid = {
      .kp = 0.0515502929687f,
      .ki = 0.00f,
      .kd = 0.00f,
      .sampling_time = 0.002f,
      .is_antiwindup_enabled = true,
      .output_max = 14.6484375f,
      .integral_max = 6.103515625f,
      .is_deadband_enabled = true,
      .deadband_threshold = 1.0f,
  };
  static PidConfig chassis_m3508_speed_pid = {
      .kp = 0.0315f,
      .ki = 0.0f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_antiwindup_enabled = true,
      .output_max = 14.6484375f,
      .integral_max = 6.103515625f,
  };
  static PidConfig chassis_gm6020_position_pid = {
      .kp = 40.0f,
      .ki = 0.005f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_antiwindup_enabled = true,
      .output_max = 5.0f,
      .integral_max = 0.0f,
  };
  static PidConfig chassis_gm6020_speed_pid = {
      .kp = 0.25f,
      .ki = 0.005f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_antiwindup_enabled = true,
      .output_max = 2.0f,
      .integral_max = 1.5f,
  };
  static PidConfig gimble_gm6020_speed_pid = {
      .kp = 3.51f,
      .ki = 1.1f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_antiwindup_enabled = true,
      .output_max = 0.54f,
      .integral_max = 0.54f,
  };
  static PidConfig gimbie_gm6020_position_pid = {
      .kp = 3.51f,
      .ki = 0.51f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_feedforward_enabled = true,
      .kff = 0.1f,
      .is_antiwindup_enabled = true,
      .output_max = 10.0f,
      .integral_max = 10.0f,
  };
  // 大 Yaw 与云台 pitch 使用同一套串级结构；增益先沿用 pitch 量级，
  // 力矩限幅按 J4310 额定值放大，上板后必须重新整定。
  static PidConfig big_yaw_position_pid = {
      .kp = 10.0f,
      .ki = 0.1f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_derivativeonfeedback_enabled = true,
      .is_antiwindup_enabled = true,
      .output_max = 2.0f,
      /* 0.3 / ki */
      .integral_max = 3.0f,
      .is_deadband_enabled = true,
      .deadband_threshold = 0.01f,
  };
  static PidConfig big_yaw_velocity_pid = {
      .kp = 0.25f,
      .ki = 0.2f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_antiwindup_enabled = true,
      .output_max = 3.0f,
      /* 1.5 / ki */
      .integral_max = 7.5f,
  };
  for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
    DjiMotorInit(&g_robot.chassis_motor_3508[i], kDjiMotorM3508,
                 kDjiMotorModeSpeed, &chassis_m3508_speed_pid, NULL);
    DjiMotorInit(&g_robot.chassis_motor_6020[i], kDjiMotorGm6020,
                 kDjiMotorModePosition, &chassis_gm6020_speed_pid,
                 &chassis_gm6020_position_pid);
  }
  DjiMotorInit(&g_robot.friction_motor[0], kDjiMotorM3508,
               kDjiMotorModeSpeed, &m3508_speed_pid, NULL);
  DjiMotorInit(&g_robot.friction_motor[1], kDjiMotorM3508,
               kDjiMotorModeSpeed, &m3508_speed_pid, NULL);
  DjiMotorInit(&g_robot.bullet_feed_motor, kDjiMotorM2006,
               kDjiMotorModeSpeed, &m2006_speed_pid, NULL);
  DjiMotorInit(&g_robot.pitch_motor, kDjiMotorGm6020,
               kDjiMotorModePosition, &gimble_gm6020_speed_pid,&gimbie_gm6020_position_pid);
  DmMotorInit(&g_robot.yaw_motor, kDmMotorTypeJ4310,
              (kRobotGimbalEnabled != 0U || kRobotBigYawGyroCompensationEnabled != 0U)
                  ? kDmMotorModeVelocity : kDmMotorModeMit,
              0x06U, 0x11U);
  // MIT 帧只传递力矩，电机端不再叠加位置/速度增益。
  g_robot.yaw_motor.state.kp = 0.0f;
  g_robot.yaw_motor.state.kd = 0.0f;
  PidInit(&g_robot.big_yaw_position_pid, &big_yaw_position_pid);
  PidInit(&g_robot.big_yaw_velocity_pid, &big_yaw_velocity_pid);
}

static void RobotSharedInit(void) {
  if (g_robot.is_initialized) {
    return;
  }

  uint32_t lock = osKernelLock();
  if (g_robot.is_initialized || g_robot.is_initializing) {
    (void)osKernelRestoreLock(lock);
    while (!g_robot.is_initialized) {
      osDelay(1);
    }
    return;
  }
  memset(&g_robot, 0, sizeof(g_robot));
  g_robot.chassis.steering_idle = true;
  g_robot.is_initializing = true;
  (void)osKernelRestoreLock(lock);

  RobotMotorInit();
  if (kRobotChassisEnabled != 0U) {
    FdcanInit(&hfdcan1, RobotFdcan1RxCallback);
  }
  if (kRobotChassisEnabled != 0U || kRobotGimbalEnabled != 0U ||
      kRobotShootEnabled != 0U || kRobotBigYawEnabled != 0U) {
    FdcanInit(&hfdcan2, RobotFdcan2RxCallback);
  }
  if (kRobotShootEnabled != 0U || kRobotBigYawEnabled != 0U) {
    FdcanInit(&hfdcan3, RobotFdcan3RxCallback);
  }
  UartInit(&huart5, Dt7RxCallback);
  HostVelocityProtocolInit();
  UartInit(&huart7, HostVelocityProtocolRxCallback);
  if (kRobotGimbalLinkEnabled != 0U) {
    GimbalLinkInit();
    UartInit(&huart1, GimbalLinkRxCallback);
  }
  if (kRobotVisionEnabled != 0U) {
    UartInit(&huart10, VisionProtocolRxCallback);
  }
  if (kRobotRefereeEnabled != 0U) {
    RefereeInit(&g_referee, &huart1, &huart1);
  } else if (kRobotChassisEnabled != 0U && kRobotGimbalLinkEnabled == 0U) {
    UartInit(&huart1, NULL);
  }
  if (kRobotChassisEnabled != 0U) {
    g_robot.chassis.imu_status = Bmi088Init(&g_bmi088, &hspi2);
    if (g_robot.chassis.imu_status == kBmi088NoError) {
      Bmi088HeaterInit(&g_bmi088, &htim3, TIM_CHANNEL_4,
                       &g_robot_bmi088_heater_pid,
                       kRobotImuHeaterTargetTemperatureC);
      g_robot.chassis.imu_status =
          Bmi088CalibrateGyroOffset(&g_bmi088, 300U, 1U);
      g_robot.chassis.imu_ready =
          g_robot.chassis.imu_status == kBmi088NoError;
    }
    g_robot.chassis.target_yaw_angle = g_robot.chassis.yaw_angle;
    g_robot.gimbal.target_yaw_angle = g_robot.chassis.target_yaw_angle;
  }
  g_robot.gimbal.pitch_angle = kRobotPitchOffsetRad;

  if (kRobotGimbalEnabled != 0U) {
    uint8_t tx_data[kRobotCanDataLength] = {0};
    DmMotorPackEnable(&g_robot.yaw_motor, tx_data);
    FdcanTransmit(&hfdcan3, tx_data, kRobotCanDataLength,
                  DmMotorTxId(&g_robot.yaw_motor));
  }
  g_robot.chassis_imu_last_tick = osKernelGetTickCount();
  g_robot.gimbal_last_tick = osKernelGetTickCount();
  g_robot.is_initialized = true;
  g_robot.is_initializing = false;
}

static float RobotEstimateChassisPower(void) {
  float total_power = 0.0f;
  for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
    float current = g_robot.chassis_motor_3508[i].state.given_current;
    float omega = g_robot.chassis_motor_3508[i].state.omega;
    float torque = kRobotPowerK0 * current;
    float wheel_power =
        torque * omega + kRobotPowerK1 * fabsf(omega) +
        kRobotPowerK2 * torque * torque +
        kRobotPowerK3 / (float)kRobotChassisMotorCount;
    total_power += fmaxf(wheel_power, 0.0f);
  }
  return total_power;
}

static void RobotApplyChassisPowerLimit(float max_power) {
  float command_power = RobotEstimateChassisPower();
  if (max_power <= 0.0f) {
    for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
      g_robot.chassis_motor_3508[i].state.given_current = 0.0f;
    }
    g_robot.chassis.estimated_power = 0.0f;
    return;
  }

  if (command_power > max_power) {
    float scale = max_power / command_power;
    for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
      g_robot.chassis_motor_3508[i].state.given_current *= scale;
    }
    g_robot.chassis.estimated_power = max_power;
  } else {
    g_robot.chassis.estimated_power = command_power;
  }
}

static float RobotChassisPowerLimit(void) {
  if (kRobotRefereeEnabled != 0U &&
      g_referee.game_robot_state.chassis_power_limit > 0U) {
    return (float)g_referee.game_robot_state.chassis_power_limit;
  }
  return kRobotDefaultChassisPowerW;
}

static void RobotChassisImuUpdate(void) {
  uint32_t now = osKernelGetTickCount();
  uint32_t elapsed_ms = now - g_robot.chassis_imu_last_tick;
  g_robot.chassis_imu_last_tick = now;
  g_robot.chassis.control_dt = (float)elapsed_ms * 0.001f;

  // 运行中读失败可以重试；初始化或零偏标定失败不能在运动中直接补标定。
  if (!g_bmi088.is_inited || !g_bmi088.is_gyro_offset_inited) {
    g_robot.chassis.imu_ready = false;
    g_robot.chassis.yaw_rate = 0.0f;
    return;
  }

  bool was_ready = g_robot.chassis.imu_ready;
  Bmi088Status status = Bmi088Update(&g_bmi088);
  if (g_bmi088.is_heater_enabled) {
    (void)Bmi088HeaterControl(&g_bmi088);
  }
  float yaw_rate =
      kRobotImuYawRateSign * g_bmi088.corrected_gyro[kRobotImuGyroZAxis];
  if (status == kBmi088NoError && !isfinite(yaw_rate)) {
    status = kBmi088NoSensor;
  }
  g_robot.chassis.imu_status = status;
  if (status != kBmi088NoError) {
    ++g_robot.chassis.imu_error_count;
    g_robot.chassis.imu_ready = false;
    g_robot.chassis.yaw_rate = 0.0f;
    g_robot.chassis.target_yaw_angle = g_robot.chassis.yaw_angle;
    return;
  }

  g_robot.chassis.imu_ready = true;
  ++g_robot.chassis.imu_sample_count;
  g_robot.chassis.yaw_rate = yaw_rate;
  // 恢复后的首帧不把失效期间的时间按当前角速度积分。
  if (was_ready) {
    g_robot.chassis.yaw_angle = RobotWrapPi(
        g_robot.chassis.yaw_angle + yaw_rate * (float)elapsed_ms * 0.001f);
  }
}

// Absolute motor feedback defines gimbal -> chassis orientation directly.
// A fixed calibrated zero survives disable, IMU reset and board reboot.
static void RobotChassisGimbalHeadingUpdate(bool motion_authorized) {
  (void)motion_authorized;
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  const bool available = g_robot.yaw_feedback_valid;
  const uint32_t stamp = g_robot.yaw_feedback_tick;
  const float angle = g_robot.yaw_motor.state.angle;
  const double continuous = g_robot.big_yaw_encoder_continuous_rad;
  const bool encoder_initialized = g_robot.big_yaw_encoder_initialized;
  const float rate = g_robot.yaw_motor.state.omega;
  const uint8_t error = g_robot.yaw_motor.state.error_code;
  __set_PRIMASK(mask);
  const uint32_t age = HAL_GetTick() - stamp;
  g_robot.chassis.big_yaw_encoder_angle_rad = angle;
  g_robot.chassis.big_yaw_encoder_age_ms = age;
  g_robot.chassis.big_yaw_heading_valid = available && encoder_initialized &&
      age <= kRobotBigYawFeedbackTimeoutMs && error == 0U &&
      isfinite(angle) && isfinite(rate);
  if (!g_robot.chassis.big_yaw_heading_valid) return;
  // Unwrap 25-rad protocol first; only then reduce physical heading modulo 2*pi.
  const double relative = (double)kRobotBigYawGyroDirection *
      (continuous - (double)kRobotBigYawForwardZeroRad) / (double)kRobotBigYawGyroRatio;
  g_robot.chassis.big_yaw_relative_yaw_rad =
      RobotWrapPi((float)fmod(relative, 2.0 * M_PI));
  g_robot.chassis.big_yaw_relative_rate_radps =
      kRobotBigYawGyroDirection * rate / kRobotBigYawGyroRatio;
}

static bool RobotChassisRemoteVelocity(bool *spin_in_place) {
  float forward_stick = RobotRcNormalize(g_dt7_object.rocker.ch3);
  float left_stick = -RobotRcNormalize(g_dt7_object.rocker.ch2);
  g_robot.chassis.translation_stick_active = RobotChassisStickActive(
      forward_stick, left_stick, g_robot.chassis.translation_stick_active);
  float vx_input = (g_robot.chassis.translation_stick_active
                        ? RobotApplyDeadband(forward_stick,
                                             kRobotChassisTranslationDeadband)
                        : 0.0f) +
                   (float)g_dt7_object.key.w - (float)g_dt7_object.key.s;
  float vy_input = (g_robot.chassis.translation_stick_active
                        ? RobotApplyDeadband(left_stick,
                                             kRobotChassisTranslationDeadband)
                        : 0.0f) +
                   (float)g_dt7_object.key.a - (float)g_dt7_object.key.d;
  bool translation_key_active = g_dt7_object.key.w || g_dt7_object.key.a ||
                                g_dt7_object.key.s || g_dt7_object.key.d;
  float wheel_input = RobotApplyDeadband(
      RobotRcNormalize(g_dt7_object.rocker.wheel),
      kRobotChassisYawWheelDeadband);
  (void)wheel_input; // wheel is consumed by big-Yaw control only
  bool wheel_spin_active = false;
  float desired_yaw_rate = RobotChassisYawRateCommand(0.0f);
  bool auto_spin_enabled = RobotAutoSpinIsEnabled();
  if (auto_spin_enabled) {
    if (g_robot.chassis.imu_ready) {
      g_robot.chassis.target_yaw_angle = g_robot.chassis.yaw_angle;
    }
    desired_yaw_rate += kRobotChassisAutoSpinOmegaRadps;
    desired_yaw_rate = RobotClamp(desired_yaw_rate, -kRobotMaxOmegaRadps,
                                     kRobotMaxOmegaRadps);
  }

  // 必须在IMU角速度补偿前判断，摇杆归中时停止舵向角度控制。
  bool steering_idle = !g_robot.chassis.translation_stick_active &&
                       !translation_key_active &&
                       !wheel_spin_active &&
                       (kRobotSteeringIdleOverridesYaw != 0U ||
                        desired_yaw_rate == 0.0f);
  g_robot.chassis.desired_velocity.vx =
      RobotClamp(vx_input, -1.0f, 1.0f) * kRobotMaxVxMps;
  g_robot.chassis.desired_velocity.vy =
      RobotClamp(vy_input, -1.0f, 1.0f) * kRobotMaxVyMps;
  g_robot.chassis.desired_velocity.omega_z = desired_yaw_rate;
  *spin_in_place = wheel_spin_active &&
                   g_robot.chassis.desired_velocity.vx == 0.0f &&
                   g_robot.chassis.desired_velocity.vy == 0.0f;
  return steering_idle;
}

static void RobotChassisSpinTranslation(float *vx, float *vy,
                                        float desired_yaw_rate,
                                        bool input_in_body_frame) {
  const float reference_vx = *vx;
  const float reference_vy = *vy;
  const bool spinning = kRobotChassisSpinTranslationEnabled != 0U &&
      fabsf(desired_yaw_rate) > kRobotChassisSpinCommandEpsilon;
  float theta = 0.0f;
  if (!input_in_body_frame) {
    if (!g_robot.chassis.big_yaw_heading_valid) {
      *vx = 0.0f; *vy = 0.0f;
      g_robot.chassis.spin_translation_active = false;
      g_robot.chassis.spin_drive_tracking = false;
      g_robot.chassis.spin_translation_relative_yaw = 0.0f;
      return;
    }
    theta = g_robot.chassis.big_yaw_relative_yaw_rad;
  }
  g_robot.chassis.spin_translation_active = spinning;
  g_robot.chassis.spin_translation_relative_yaw = theta;
  float scale = 1.0f;
  if (spinning) {
    scale = fabsf(reference_vy) > kRobotChassisSpinCommandEpsilon
        ? kRobotChassisSpinLateralScale
        : (fabsf(reference_vx) > kRobotChassisSpinCommandEpsilon
            ? kRobotChassisSpinForwardScale : 1.0f);
  }
  // Joystick forward follows the gimbal; output is expressed in the wheel/body frame.
  const float c = cosf(theta), s = sinf(theta);
  *vx = (c * reference_vx - s * reference_vy) * scale;
  *vy = (s * reference_vx + c * reference_vy) * scale;
}

static void RobotChassisKinematics(const HostVelocityCommand *host_command) {
  bool steering_idle;
  bool wheel_spin_in_place;
  if (host_command != NULL) {
    // 统一读取转发指令；下方按来源选择云台坐标或 UART7 车身坐标。
    g_robot.chassis.desired_velocity.vx =
        RobotClamp(host_command->vx, -kRobotMaxVxMps, kRobotMaxVxMps);
    g_robot.chassis.desired_velocity.vy =
        RobotClamp(host_command->vy, -kRobotMaxVyMps, kRobotMaxVyMps);
    g_robot.chassis.desired_velocity.omega_z =
        RobotClamp(host_command->wz, -kRobotMaxOmegaRadps, kRobotMaxOmegaRadps);
    steering_idle = g_robot.chassis.desired_velocity.vx == 0.0f &&
                    g_robot.chassis.desired_velocity.vy == 0.0f &&
                    g_robot.chassis.desired_velocity.omega_z == 0.0f;
    wheel_spin_in_place = !steering_idle &&
                         g_robot.chassis.desired_velocity.vx == 0.0f &&
                         g_robot.chassis.desired_velocity.vy == 0.0f;
    g_robot.chassis.translation_stick_active = false;
  } else {
    steering_idle = RobotChassisRemoteVelocity(&wheel_spin_in_place);
  }
  g_robot.chassis.steering_idle = steering_idle;
  if (steering_idle) {
    g_robot.chassis.spin_translation_active = false;
    g_robot.chassis.spin_drive_tracking = false;
    g_robot.chassis.spin_translation_relative_yaw = 0.0f;
    memset(&g_robot.chassis.desired_velocity, 0,
           sizeof(g_robot.chassis.desired_velocity));
    g_robot.chassis.actual_velocity.vx = 0.0f;
    g_robot.chassis.actual_velocity.vy = 0.0f;
    g_robot.chassis.actual_velocity.omega_z = g_robot.chassis.yaw_rate;
    for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
      g_robot.chassis_motor_3508[i].state.given_omega = 0.0f;
      g_robot.chassis_steering_vector_active[i] = false;
      g_robot.chassis_steering_requested_angle[i] =
          g_robot.chassis_motor_6020[i].state.angle;
    }
    return;
  }

  // 本地及云台板转发的遥控都使用实时云台坐标；UART7 保持当前车身坐标。
  // 不接云台板时没有相对角，单独底盘测试把 kRobotChassisStandaloneMode
  // 置 1，遥控平移直接按车身坐标执行，否则会被安全检查置零。
  // 接回云台板后改为 0，恢复云台坐标转换。
  bool input_in_body_frame =
      (host_command != NULL && !g_robot.chassis_interboard_control_active) ||
      kRobotChassisStandaloneMode != 0U;
  float vx = g_robot.chassis.desired_velocity.vx;
  float vy = g_robot.chassis.desired_velocity.vy;
  float desired_yaw_rate = g_robot.chassis.desired_velocity.omega_z;
  RobotChassisSpinTranslation(&vx, &vy, desired_yaw_rate,
                              input_in_body_frame);
  float wz = desired_yaw_rate;
  if (g_robot.chassis.imu_ready) {
    wz += kRobotChassisYawRateFeedbackKp *
          (desired_yaw_rate - g_robot.chassis.yaw_rate);
  }
  wz = RobotClamp(wz, -kRobotMaxOmegaRadps, kRobotMaxOmegaRadps);
  g_robot.chassis.actual_velocity.vx = vx;
  g_robot.chassis.actual_velocity.vy = vy;
  g_robot.chassis.actual_velocity.omega_z = g_robot.chassis.yaw_rate;
  // 齐奇舵轮解算：向量叠加 -> 最优角反向 -> cos^3 -> 四轮等比例限速。
  // 按本车 +X 向前、+Y 向左、实际轮距和 SI 单位适配轮组解算。
  SwerveWheelVector wheels[kSwerveWheelCount];
  float drive_speeds[kSwerveWheelCount] = {0};
  SwerveResolve(vx, vy, wz, kRobotChassisHalfWidthM,
                 kRobotChassisHalfLengthM, wheels);
  float cosine_deadband = fabsf(wz) > 0.0001f
                             ? 0.0f : kRobotSteeringCosineDeadbandRad;
  for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
    DjiMotor *steer = &g_robot.chassis_motor_6020[i];
    float wheel_speed = wheels[i].speed_mps;
    g_robot.chassis_steering_vector_active[i] =
        (wheel_spin_in_place && wheel_speed > 0.0f) ||
        RobotSteeringVectorActive(
            wheel_speed, g_robot.chassis_steering_vector_active[i]);
    if (!g_robot.chassis_steering_vector_active[i]) {
      g_robot.chassis_motor_3508[i].state.given_omega = 0.0f;
      continue;
    }
    float target_angle = RobotSteeringTargetAngle(
        i, wheels[i].angle_from_forward);
    SwerveWheelTarget target = SwerveOptimizeWheel(
        target_angle, steer->state.angle, wheel_speed, cosine_deadband);
    g_robot.chassis_steering_requested_angle[i] = target.angle_rad;
    drive_speeds[i] = target.drive_speed_mps;
  }
  SwerveLimitWheelSpeeds(drive_speeds, kRobotChassisMaxWheelSpeedMps);
  for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
    g_robot.chassis_motor_3508[i].state.given_omega =
        kRobotDriveDirection[i] * drive_speeds[i] *
        kRobotDriveReductionRatio / kRobotWheelRadiusM;
  }
}

static void RobotChassisControl(const HostVelocityCommand *host_command) {
  RobotChassisImuUpdate();
  RobotChassisGimbalHeadingUpdate(true);
  // 保留原有指令源选择；只有已通过底盘权限检查的控制分支进入这里。
  if (kRobotBigYawGyroCompensationEnabled != 0U) {
    g_robot.big_yaw_gyro_authorized = true;
    RobotBigYawControl();
  }
  RobotChassisKinematics(host_command);

  // 零速度时停止舵向角度控制并发送零电流；再次移动时从当前角度起步。
  bool steering_idle = g_robot.chassis.steering_idle;
  bool no_motor_test_idle = kRobotChassisNoMotorTxTest != 0U &&
                            g_robot.chassis.desired_velocity.vx == 0.0f &&
                            g_robot.chassis.desired_velocity.vy == 0.0f &&
                            g_robot.chassis.desired_velocity.omega_z == 0.0f;
  uint32_t now = osKernelGetTickCount();
  bool all_steer_feedback_valid = true;
  bool all_drive_feedback_valid = true;
  for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
    DjiMotor *steer = &g_robot.chassis_motor_6020[i];
    bool steer_feedback_valid = RobotChassisFeedbackValid(
        g_robot.chassis_6020_has_feedback[i],
        g_robot.chassis_6020_last_feedback_tick[i], now);
    all_steer_feedback_valid =
        all_steer_feedback_valid && steer_feedback_valid;
    if (!steering_idle && !no_motor_test_idle && steer_feedback_valid) {
      RobotSteeringUpdateTarget(i, steer);
      DjiMotorCurrentCalculate(steer);
    } else {
      steer->state.given_angle = steer->state.angle;
      steer->state.given_omega = 0.0f;
      steer->state.given_current = 0.0f;
      PidReset(&steer->state.position_loop);
      PidReset(&steer->state.speed_loop);
      g_robot.chassis_steering_target_initialized[i] = false;
      g_robot.chassis_steering_last_direction[i] = 0.0f;
    }
    all_drive_feedback_valid =
        all_drive_feedback_valid &&
        RobotChassisFeedbackValid(g_robot.chassis_3508_has_feedback[i],
                                  g_robot.chassis_3508_last_feedback_tick[i],
                                  now);
  }
  bool drive_ready = !steering_idle && !no_motor_test_idle &&
                      all_drive_feedback_valid &&
                     all_steer_feedback_valid && RobotSteeringDriveReady();
  if (!drive_ready) {
    g_robot.chassis_drive_steering_ready = false;
    g_robot.chassis.spin_drive_tracking = false;
  }
  for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
    DjiMotor *drive = &g_robot.chassis_motor_3508[i];
    if (drive_ready) {
      DjiMotorCurrentCalculate(drive);
    } else {
      drive->state.given_omega = 0.0f;
      drive->state.given_current = 0.0f;
      PidReset(&drive->state.speed_loop);
    }
  }
  if (kRobotRefereeEnabled != 0U) {
    RobotApplyChassisPowerLimit(RobotChassisPowerLimit());
  } else {
    for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
      g_robot.chassis_motor_3508[i].state.given_current *=
          kRobotNoRefereeCurrentScale;
    }
    g_robot.chassis.estimated_power = RobotEstimateChassisPower();
  }
  uint8_t tx_data[kRobotCanDataLength] = {0};
  RobotPackDjiCurrents(
      &g_robot.chassis_motor_3508[kRobotChassisCanSlotWheelIndex[0]],
      &g_robot.chassis_motor_3508[kRobotChassisCanSlotWheelIndex[1]],
      &g_robot.chassis_motor_3508[kRobotChassisCanSlotWheelIndex[2]],
      &g_robot.chassis_motor_3508[kRobotChassisCanSlotWheelIndex[3]], tx_data);
  FdcanTransmit(&hfdcan2, tx_data, kRobotCanDataLength, 0x200U);
  RobotPackDjiCurrents(
      &g_robot.chassis_motor_6020[kRobotChassisCanSlotWheelIndex[0]],
      &g_robot.chassis_motor_6020[kRobotChassisCanSlotWheelIndex[1]],
      &g_robot.chassis_motor_6020[kRobotChassisCanSlotWheelIndex[2]],
      &g_robot.chassis_motor_6020[kRobotChassisCanSlotWheelIndex[3]], tx_data);
  FdcanTransmit(&hfdcan1, tx_data, kRobotCanDataLength, 0x1FEU);
}

static void RobotGimbalControl(void) {
  if (g_vision_to_gimbal.mode == 1U || g_vision_to_gimbal.mode == 2U) {
    g_robot.gimbal.yaw_rate = g_vision_to_gimbal.yaw_velocity;
    g_robot.gimbal.pitch_angle =
        g_vision_to_gimbal.pitch + kRobotPitchOffsetRad;
    g_robot.gimbal.pitch_rate = g_vision_to_gimbal.pitch_velocity;
  } else {
    uint32_t now = osKernelGetTickCount();
    uint32_t elapsed_ms = now - g_robot.gimbal_last_tick;
    g_robot.gimbal_last_tick = now;
    float dt = (float)elapsed_ms * 0.001f;

    g_robot.gimbal.target_yaw_angle = RobotWrapPi(
        g_robot.gimbal.target_yaw_angle +
        RobotGimbalYawRateCommand() * dt);
    if (RobotAutoSpinIsEnabled()) {
      g_robot.gimbal.target_yaw_angle =
          g_robot.chassis.target_yaw_angle;
    }

    float chassis_yaw = g_robot.chassis.imu_ready
                            ? g_robot.chassis.yaw_angle
                            : 0.0f;
    float yaw_error =
        RobotWrapPi(g_robot.gimbal.target_yaw_angle - chassis_yaw);
    g_robot.gimbal.yaw_rate =
        kRobotGimbalYawMotorSign *
        (kRobotGimbalYawFieldKp * yaw_error -
         g_robot.chassis.yaw_rate);

    g_robot.gimbal.pitch_rate = RobotGimbalPitchRateCommand();
    g_robot.gimbal.pitch_angle += g_robot.gimbal.pitch_rate * dt;
  }

  g_robot.gimbal.pitch_angle =
      RobotClamp(g_robot.gimbal.pitch_angle,
                    kRobotPitchOffsetRad + kRobotGimbalPitchMinRad,
                    kRobotPitchOffsetRad + kRobotGimbalPitchMaxRad);
  g_robot.pitch_motor.state.given_angle = g_robot.gimbal.pitch_angle;
  DjiMotorCurrentCalculate(&g_robot.pitch_motor);

  // GM6020 pitch 在 FDCAN2，反馈 ID 0x20B 为电机 ID 7，对应 0x2FE 第3路电流。
  uint8_t pitch_tx_data[kRobotCanDataLength] = {0};
  RobotPackDjiCurrents(NULL, NULL, &g_robot.pitch_motor, NULL,
                          pitch_tx_data);
  FdcanTransmit(&hfdcan2, pitch_tx_data, kRobotCanDataLength,
                0x2FEU);

  g_robot.yaw_motor.state.given_omega = g_robot.gimbal.yaw_rate;

  //g_robot.yaw_motor.state.given_omega = 1.0f;
  uint8_t yaw_tx_data[kRobotCanDataLength] = {0};
  DmMotorPackVelocity(&g_robot.yaw_motor, yaw_tx_data);
  FdcanTransmit(&hfdcan3, yaw_tx_data, kRobotCanDataLength,
                DmMotorTxId(&g_robot.yaw_motor));
  if (g_robot.yaw_motor.state.error_code != 0U) {
    DmMotorPackClearError(&g_robot.yaw_motor, yaw_tx_data);
    FdcanTransmit(&hfdcan3, yaw_tx_data, kRobotCanDataLength,
                  DmMotorTxId(&g_robot.yaw_motor));
  }
}

static void RobotShootControl(void) {
  bool should_feed = SwitchIsUp(g_dt7_object.rocker.sw2) ||
                     g_dt7_object.mouse.l != 0U ||
                     g_vision_to_gimbal.mode == 2U;
  bool should_spin = SwitchIsUp(g_dt7_object.rocker.sw2) || should_feed;

  g_robot.bullet_feed_motor.state.given_omega =
      should_feed ? kRobotBulletFeedOmegaRadps : 0.0f;
  g_robot.friction_motor[0].state.given_omega =
      should_spin ? -kRobotFrictionOmegaRadps : 0.0f;
  g_robot.friction_motor[1].state.given_omega =
      should_spin ? kRobotFrictionOmegaRadps : 0.0f;

  if (should_feed) {
    DjiMotorCurrentCalculate(&g_robot.bullet_feed_motor);
  } else {
    g_robot.bullet_feed_motor.state.given_current = 0.0f;
  }
  uint8_t bullet_tx_data[kRobotCanDataLength] = {0};
  RobotPackDjiCurrents(&g_robot.bullet_feed_motor, NULL, NULL, NULL,
                          bullet_tx_data);
  FdcanTransmit(&hfdcan3, bullet_tx_data, kRobotCanDataLength, 0x1FFU);

  if (should_spin) {
    DjiMotorCurrentCalculate(&g_robot.friction_motor[0]);
    DjiMotorCurrentCalculate(&g_robot.friction_motor[1]);
  } else {
    g_robot.friction_motor[0].state.given_current = 0.0f;
    g_robot.friction_motor[1].state.given_current = 0.0f;
  }
  uint8_t friction_tx_data[kRobotCanDataLength] = {0};
  RobotPackDjiCurrents(&g_robot.friction_motor[0],
                          &g_robot.friction_motor[1], NULL, NULL,
                          friction_tx_data);
  FdcanTransmit(&hfdcan2, friction_tx_data, kRobotCanDataLength,
                0x200U);
}

static void RobotStopChassisMotors(void) {
  memset(&g_robot.chassis.desired_velocity, 0,
         sizeof(g_robot.chassis.desired_velocity));
  memset(&g_robot.chassis.actual_velocity, 0,
         sizeof(g_robot.chassis.actual_velocity));
  g_robot.chassis_drive_steering_ready = false;
  g_robot.chassis.translation_stick_active = false;
  g_robot.chassis.steering_idle = true;
  g_robot.chassis.spin_translation_active = false;
  g_robot.chassis.spin_drive_tracking = false;
  g_robot.chassis.spin_translation_relative_yaw = 0.0f;
  for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
    g_robot.chassis_steering_vector_active[i] = false;
    g_robot.chassis_steering_target_initialized[i] = false;
    g_robot.chassis_steering_requested_angle[i] = 0.0f;
    g_robot.chassis_steering_last_direction[i] = 0.0f;
    g_robot.chassis_motor_3508[i].state.given_omega = 0.0f;
    g_robot.chassis_motor_3508[i].state.given_current = 0.0f;
    PidReset(&g_robot.chassis_motor_3508[i].state.speed_loop);
    g_robot.chassis_motor_6020[i].state.given_angle = 0.0f;
    g_robot.chassis_motor_6020[i].state.given_omega = 0.0f;
    g_robot.chassis_motor_6020[i].state.given_current = 0.0f;
    PidReset(&g_robot.chassis_motor_6020[i].state.position_loop);
    PidReset(&g_robot.chassis_motor_6020[i].state.speed_loop);
  }
  g_robot.chassis.target_yaw_angle = g_robot.chassis.yaw_angle;
  g_robot.chassis_imu_last_tick = osKernelGetTickCount();
}

static void RobotChassisSelectControlSource(bool host_control) {
  if (g_robot.chassis_host_control_active != host_control) {
    // 切换来源时清除旧PID、舵角和旋转参考，再从当前反馈重新起步。
    RobotStopChassisMotors();
    g_robot.chassis_host_control_active = host_control;
  }
}

static void RobotBigYawSendCommand(DmMotorCommand command) {
  uint8_t tx_data[kRobotCanDataLength] = {0};
  DmMotorPackCommand(&g_robot.yaw_motor, command, tx_data);
  FdcanTransmit(&hfdcan3, tx_data, kRobotCanDataLength,
                DmMotorTxId(&g_robot.yaw_motor));
}

static void RobotBigYawSendMitTorque(float torque_nm) {
  float torque = isfinite(torque_nm)
                     ? RobotClamp(torque_nm, -kRobotBigYawTorqueLimitNm,
                                  kRobotBigYawTorqueLimitNm)
                     : 0.0f;
  // MIT 帧的 kp/kd 固定为 0：位置/速度闭环全部在应用层完成。
  g_robot.yaw_motor.state.given_angle = 0.0f;
  g_robot.yaw_motor.state.given_omega = 0.0f;
  g_robot.yaw_motor.state.given_torque = torque;
  g_robot.yaw_motor.state.kp = 0.0f;
  g_robot.yaw_motor.state.kd = 0.0f;
  uint8_t tx_data[kRobotCanDataLength] = {0};
  DmMotorPackMit(&g_robot.yaw_motor, tx_data);
  FdcanTransmit(&hfdcan3, tx_data, kRobotCanDataLength,
                DmMotorTxId(&g_robot.yaw_motor));
  g_robot.big_yaw_torque = torque;
}

static void RobotBigYawSendVelocity(float rate_rad_s) {
  float rate = isfinite(rate_rad_s)
                   ? RobotClamp(rate_rad_s, -kRobotBigYawGyroMaxRateRadps,
                                kRobotBigYawGyroMaxRateRadps)
                   : 0.0f;
  g_robot.yaw_motor.state.given_omega = rate;
  g_robot.yaw_motor.state.given_torque = 0.0f;
  g_robot.big_yaw_torque = 0.0f;
  uint8_t tx_data[kRobotCanDataLength] = {0};
  DmMotorPackVelocity(&g_robot.yaw_motor, tx_data);
  // Native velocity payload is one float: DLC=4 (special commands still use DLC=8).
  FdcanTransmit(&hfdcan3, tx_data, sizeof(float),
                DmMotorTxId(&g_robot.yaw_motor));
}

static void RobotBigYawSendStop(void) {
  if (g_robot.yaw_motor.mode == kDmMotorModeVelocity) {
    RobotBigYawSendVelocity(0.0f);
  } else {
    RobotBigYawSendMitTorque(0.0f);
  }
}

static void RobotBigYawResetCascade(void) {
  PidReset(&g_robot.big_yaw_position_pid);
  PidReset(&g_robot.big_yaw_velocity_pid);
  g_robot.big_yaw_heading_initialized = false;
  g_robot.big_yaw_heading_target_rad = 0.0f;
  g_robot.big_yaw_heading_error_rad = 0.0f;
  g_robot.big_yaw_world_rate_target_radps = 0.0f;
  g_robot.big_yaw_rate_integral = 0.0f;
  g_robot.big_yaw_previous_operator_rate = 0.0f;
  g_robot.big_yaw_target_initialized = false;
  g_robot.big_yaw_target_angle = 0.0f;
  g_robot.big_yaw_target_rate = 0.0f;
  g_robot.big_yaw_torque = 0.0f;
}

// Gimbal heading is relative to its own IMU origin, independent of the
// encoder zero used for chassis translation. Predict only by sample age.
static float RobotBigYawStabilizedRate(const GimbalImuYaw *imu,
                                       float operator_rate, float dt) {
  const float heading = RobotWrapPi(imu->yaw_delta_rad +
      imu->yaw_rate_rad_s * (float)imu->source_age_ms * 0.001f);
  if (!g_robot.big_yaw_heading_initialized ||
      g_robot.big_yaw_heading_boot_id != imu->sender_boot_id ||
      g_robot.big_yaw_heading_zero_generation != imu->zero_generation) {
    g_robot.big_yaw_heading_initialized = true;
    g_robot.big_yaw_heading_boot_id = imu->sender_boot_id;
    g_robot.big_yaw_heading_zero_generation = imu->zero_generation;
    g_robot.big_yaw_heading_target_rad = heading;
    g_robot.big_yaw_rate_integral = 0.0f;
  }
  // Release captures the measured heading instead of chasing a lagging target.
  if (operator_rate == 0.0f && g_robot.big_yaw_previous_operator_rate != 0.0f) {
    g_robot.big_yaw_heading_target_rad = heading;
    g_robot.big_yaw_rate_integral = 0.0f;
  }
  g_robot.big_yaw_previous_operator_rate = operator_rate;
  float error = RobotClamp(RobotWrapPi(
      g_robot.big_yaw_heading_target_rad + operator_rate * dt - heading),
      -kRobotBigYawMaxPositionErrorRad, kRobotBigYawMaxPositionErrorRad);
  g_robot.big_yaw_heading_target_rad = RobotWrapPi(heading + error);
  g_robot.big_yaw_heading_error_rad = error;
  const float world_rate = operator_rate + RobotClamp(2.0f * error, -1.5f, 1.5f);
  g_robot.big_yaw_world_rate_target_radps = world_rate;
  const float rate_error = world_rate - imu->yaw_rate_rad_s;
  const float feedforward = world_rate - g_robot.chassis.yaw_rate;
  const float proportional = 0.25f * rate_error;
  const float integral = g_robot.big_yaw_rate_integral + 0.5f * rate_error * dt;
  const float relative_limit = kRobotBigYawGyroMaxRateRadps /
      fabsf(kRobotBigYawGyroRatio);
  const float candidate = feedforward + proportional + integral;
  // Include feedforward in anti-windup so high base rates cannot accumulate I.
  if (fabsf(candidate) <= relative_limit ||
      (candidate > relative_limit && rate_error < 0.0f) ||
      (candidate < -relative_limit && rate_error > 0.0f)) {
    g_robot.big_yaw_rate_integral = integral;
  }
  const float relative_rate = feedforward + proportional +
      g_robot.big_yaw_rate_integral;
  return RobotClamp(kRobotBigYawGyroDirection * kRobotBigYawGyroRatio * relative_rate,
      -kRobotBigYawGyroMaxRateRadps, kRobotBigYawGyroMaxRateRadps);
}

/* CAN3 0x206 velocity mode: base gyro feedforward and gimbal onboard BMI088
 * heading/rate feedback. Missing IMU data disables output; recovery recaptures
 * heading. Encoder-based translation retains its independent fixed zero. */
static void RobotBigYawControl(void) {
  if (kRobotBigYawEnabled == 0U) {
    return;
  }
  const uint32_t now = HAL_GetTick();
  const uint32_t previous_tick = g_robot.big_yaw_cascade_last_tick;
  uint32_t elapsed_ms = now - previous_tick;
  g_robot.big_yaw_cascade_last_tick = now;
  float dt = kRobotBigYawCascadeFallbackDtS;
  if (previous_tick != 0U && elapsed_ms != 0U &&
      (float)elapsed_ms * 0.001f <= kRobotBigYawCascadeMaxDtS) {
    dt = (float)elapsed_ms * 0.001f;
  }

  BoardLinkBigYawRequest request;
  bool request_valid = kRobotGimbalLinkEnabled != 0U &&
                       GimbalLinkGetBigYawRequest(&request);
  bool request_active = request_valid &&
                        request.mode == kBoardLinkBigYawFollowCenter &&
                        request.permission_valid && request.permission_enabled;
  /* Dt7LastUpdateTick() uses the RTOS tick, not HAL_GetTick(); mixing the
   * two time bases can make a live receiver look offline. */
  bool dt7_online = Dt7FrameCount() != 0U &&
                    (uint32_t)(osKernelGetTickCount() - Dt7LastUpdateTick()) <=
                        kRobotDt7OfflineTimeoutMs;
  /* Any fresh BigYawRequest, including an explicit disabled/permission
   * revoked request, owns the actuator decision.  Only when the request has
   * expired entirely may a live DT7 receiver take over.  Do not gate this
   * fallback on the heartbeat: a disconnected or electrically noisy UART can
   * leave the heartbeat snapshot briefly online and make the motor toggle
   * between disable and enable. */
  bool manual_active = kRobotBigYawRcFallback != 0U &&
                       kRobotGimbalEnabled == 0U && !request_valid &&
                       dt7_online;
  float operator_yaw_rate = 0.0f;
  if (request_active) {
    operator_yaw_rate = RobotClamp(request.target_rate_rad_s,
        -kRobotBigYawMaxRateRadps, kRobotBigYawMaxRateRadps);
  } else if (!request_valid && !g_robot.chassis_interboard_control_active && dt7_online) {
    operator_yaw_rate = RobotApplyDeadband(
        RobotRcNormalize(g_dt7_object.rocker.wheel), kRobotChassisYawWheelDeadband) *
        kRobotBigYawMaxRateRadps;
  }
  GimbalImuYaw gimbal_imu = {0};
  const bool gimbal_imu_valid = kRobotBigYawGyroCompensationEnabled != 0U &&
      GimbalLinkGetImuYaw(&gimbal_imu) && gimbal_imu.valid &&
      gimbal_imu.source_age_ms <= 50U &&
      isfinite(gimbal_imu.yaw_delta_rad) && isfinite(gimbal_imu.yaw_rate_rad_s);
  bool gyro_active = kRobotBigYawGyroCompensationEnabled != 0U &&
      g_robot.big_yaw_gyro_authorized && g_robot.chassis.imu_ready &&
      isfinite(g_robot.chassis.yaw_rate) && gimbal_imu_valid &&
      (request_active || manual_active);
  // 速度模式合成滚轮期望世界角速度与底盘自稳补偿；不叠加右杆位置环。
  if (kRobotBigYawGyroCompensationEnabled != 0U) {
    request_active = false;
    manual_active = false;
  }
  g_robot.big_yaw_request_valid = request_valid;
  g_robot.big_yaw_manual_active = manual_active;
  g_robot.big_yaw_gyro_active = gyro_active;

  if (!request_active && !manual_active && !gyro_active) {
    if (g_robot.big_yaw_enabled) {
      RobotBigYawSendStop();
      RobotBigYawSendCommand(kDmMotorCommandDisable);
      g_robot.big_yaw_enabled = false;
    }
    g_robot.big_yaw_enable_repeat = 0U;
    g_robot.big_yaw_feedback_lost = false;
    g_robot.big_yaw_fault_latched = false;
    g_robot.big_yaw_active = false;
    RobotBigYawResetCascade();
    g_robot.big_yaw_last_command_sequence =
        request_valid ? request.source_sequence : 0U;
    return;
  }

  /* CAN ISR 会成组更新反馈；控制侧先取一致快照，避免跨帧组合。 */
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const float feedback_angle = (float)fmod(
      g_robot.big_yaw_encoder_continuous_rad, 2.0 * M_PI);
  const float feedback_omega = g_robot.yaw_motor.state.omega;
  const uint8_t error_code = g_robot.yaw_motor.state.error_code;
  __set_PRIMASK(primask);

  if (error_code != 0U) {
    if (!g_robot.big_yaw_fault_latched) {
      RobotBigYawSendStop();
      RobotBigYawSendCommand(kDmMotorCommandDisable);
      RobotBigYawSendCommand(kDmMotorCommandClearError);
      g_robot.big_yaw_fault_latched = true;
    }
    g_robot.big_yaw_enabled = false;
    g_robot.big_yaw_enable_repeat = 0U;
    g_robot.big_yaw_active = false;
    RobotBigYawResetCascade();
    g_robot.big_yaw_last_command_sequence =
        request_valid ? request.source_sequence : 0U;
    return;
  }

  bool feedback_fresh =
      g_robot.yaw_feedback_valid &&
      (uint32_t)(now - g_robot.yaw_feedback_tick) <=
          kRobotBigYawFeedbackTimeoutMs &&
      isfinite(feedback_angle) && isfinite(feedback_omega);
  if (g_robot.big_yaw_fault_latched) {
    /* ClearError is sent once.  Re-arm only after a fresh, error-free
     * feedback frame proves that the motor accepted the recovery. */
    if (!feedback_fresh) {
      g_robot.big_yaw_active = false;
      return;
    }
    g_robot.big_yaw_fault_latched = false;
  }
  if (g_robot.yaw_feedback_valid && !feedback_fresh) {
    /* A motor that was reporting feedback has gone silent.  Stop it once and
     * wait for feedback to return before attempting another enable cycle. */
    if (!g_robot.big_yaw_feedback_lost) {
      RobotBigYawSendStop();
      RobotBigYawSendCommand(kDmMotorCommandDisable);
      g_robot.big_yaw_feedback_lost = true;
    }
    g_robot.big_yaw_enabled = false;
    g_robot.big_yaw_enable_repeat = 0U;
    g_robot.big_yaw_active = false;
    RobotBigYawResetCascade();
    return;
  }
  if (feedback_fresh) {
    g_robot.big_yaw_feedback_lost = false;
  }
  if (!g_robot.big_yaw_enabled) {
    g_robot.big_yaw_enabled = true;
    g_robot.big_yaw_enable_repeat = 5U;
    g_robot.big_yaw_enable_tick = now;
    RobotBigYawResetCascade();
  }
  if (g_robot.big_yaw_enable_repeat > 0U) {
    RobotBigYawSendCommand(kDmMotorCommandEnable);
    g_robot.big_yaw_enable_repeat--;
    /* 使能窗口内发送当前模式的零输出，等待反馈和状态稳定。 */
    RobotBigYawSendStop();
    g_robot.big_yaw_active = false;
    return;
  } else if (!g_robot.yaw_feedback_valid &&
             (uint32_t)(now - g_robot.big_yaw_enable_tick) >=
                 kRobotBigYawEnableRetryMs) {
    /* 电机掉电/重启后软件仍记得已使能，定期补发 Enable，直到反馈恢复。 */
    RobotBigYawSendCommand(kDmMotorCommandEnable);
    g_robot.big_yaw_enable_tick = now;
  }

  if (!feedback_fresh) {
    /* 尚未收到反馈时保持零输出；保持使能并等待反馈或下一次 Enable 重试。 */
    RobotBigYawSendStop();
    g_robot.big_yaw_active = false;
    RobotBigYawResetCascade();
    return;
  }

  if (!g_robot.big_yaw_target_initialized) {
    g_robot.big_yaw_target_angle = RobotWrapPi(feedback_angle);
    g_robot.big_yaw_target_initialized = true;
    g_robot.big_yaw_target_rate = 0.0f;
    PidReset(&g_robot.big_yaw_position_pid);
    PidReset(&g_robot.big_yaw_velocity_pid);
  }

  if (gyro_active) {
    float target_rate = RobotBigYawStabilizedRate(&gimbal_imu, operator_yaw_rate, dt);
    g_robot.big_yaw_target_rate = target_rate;
    g_robot.big_yaw_target_angle = feedback_angle;
    g_robot.big_yaw_last_command_sequence = request_valid ? request.source_sequence : 0U;
    PidReset(&g_robot.big_yaw_position_pid);
    PidReset(&g_robot.big_yaw_velocity_pid);
    RobotBigYawSendVelocity(target_rate);
    g_robot.big_yaw_active = true;
    g_robot.big_yaw_last_control_tick = now;
    return;
  }

  float target_rate;
  if (request_active) {
    target_rate = kRobotBigYawGyroDirection * kRobotBigYawGyroRatio *
                  request.target_rate_rad_s;
    g_robot.big_yaw_last_command_sequence = request.source_sequence;
  } else {
    float rc = RobotApplyDeadband(
        RobotRcNormalize(g_dt7_object.rocker.wheel),
        kRobotChassisYawWheelDeadband);
    target_rate = kRobotBigYawGyroDirection * kRobotBigYawGyroRatio *
                  rc * kRobotBigYawMaxRateRadps;
    g_robot.big_yaw_last_command_sequence = request_valid ? request.source_sequence : 0U;
  }
  target_rate = RobotClamp(target_rate, -kRobotBigYawMaxRateRadps,
                           kRobotBigYawMaxRateRadps);
  g_robot.big_yaw_target_rate = target_rate;

  /* 连续目标角：大 Yaw 可多圈旋转，目标按 2π 归一，误差取最短分支。
   * 目标超前量限制在 ±90°，跟随失败时保持同一力矩方向。 */
  float angle_error = RobotWrapPi(
      g_robot.big_yaw_target_angle + target_rate * dt - feedback_angle);
  angle_error = RobotClamp(angle_error, -kRobotBigYawMaxPositionErrorRad,
                           kRobotBigYawMaxPositionErrorRad);
  g_robot.big_yaw_target_angle = RobotWrapPi(feedback_angle + angle_error);
  const float target_nearest = feedback_angle + angle_error;

  g_robot.big_yaw_position_pid.sampling_time = dt;
  g_robot.big_yaw_velocity_pid.sampling_time = dt;
  float desired_omega = PidCalculate(&g_robot.big_yaw_position_pid,
                                     target_nearest, feedback_angle);
  desired_omega = RobotClamp(desired_omega,
                             -kRobotBigYawPositionOutputMaxRadps,
                             kRobotBigYawPositionOutputMaxRadps);
  float torque = PidCalculate(&g_robot.big_yaw_velocity_pid, desired_omega,
                              feedback_omega);
  RobotBigYawSendMitTorque(torque);
  g_robot.big_yaw_active = true;
  g_robot.big_yaw_last_control_tick = now;
}

static void RobotSendBigYawFeedback(void) {
  if (kRobotBigYawEnabled == 0U || kRobotGimbalLinkEnabled == 0U) {
    return;
  }
  const uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - g_robot.big_yaw_feedback_last_tick) <
      kRobotBigYawFeedbackPeriodMs) {
    return;
  }
  g_robot.big_yaw_feedback_last_tick = now;

  bool feedback_valid =
      g_robot.yaw_feedback_valid &&
      (uint32_t)(now - g_robot.yaw_feedback_tick) <=
          kRobotBigYawFeedbackTimeoutMs;
  BoardLinkBigYawFeedback feedback = {
      .execution_state = g_robot.big_yaw_active
                             ? kBoardLinkExecutionRunning
                             : ((g_robot.big_yaw_request_valid ||
                                 g_robot.big_yaw_manual_active ||
                                 g_robot.big_yaw_gyro_active)
                                    ? kBoardLinkExecutionWaiting
                                    : kBoardLinkExecutionDisabled),
      .ready = g_robot.big_yaw_active,
      .armed = g_robot.big_yaw_enabled && g_robot.big_yaw_active,
      .valid = feedback_valid,
      .actual_rate_rad_s = g_robot.yaw_motor.state.omega,
      .active_reasons = 0U,
      .last_command_sequence = g_robot.big_yaw_last_command_sequence,
      .source_sequence = g_robot.big_yaw_last_command_sequence,
      .production_age_ms =
          g_robot.big_yaw_active
              ? (uint32_t)(now - g_robot.big_yaw_last_control_tick)
              : 0U,
  };
  (void)GimbalLinkSendBigYawFeedback(&feedback);
}

static void RobotStopGimbalMotors(void) {
  g_robot.gimbal.target_yaw_angle = g_robot.chassis.target_yaw_angle;
  g_robot.gimbal.pitch_angle = kRobotPitchOffsetRad;
  g_robot.gimbal.pitch_rate = 0.0f;
  g_robot.pitch_motor.state.given_angle = g_robot.gimbal.pitch_angle;
  g_robot.pitch_motor.state.given_omega = 0.0f;
  g_robot.pitch_motor.state.given_current = 0.0f;
  g_robot.yaw_motor.state.given_omega = 0.0f;
  g_robot.gimbal_last_tick = osKernelGetTickCount();
}

static void RobotStopShootMotors(void) {
  g_robot.bullet_feed_motor.state.given_omega = 0.0f;
  g_robot.bullet_feed_motor.state.given_current = 0.0f;
  g_robot.friction_motor[0].state.given_omega = 0.0f;
  g_robot.friction_motor[0].state.given_current = 0.0f;
  g_robot.friction_motor[1].state.given_omega = 0.0f;
  g_robot.friction_motor[1].state.given_current = 0.0f;
}

static void RobotTransmitGimbalStopCurrents(void) {
  uint8_t pitch_tx_data[kRobotCanDataLength] = {0};
  uint8_t yaw_tx_data[kRobotCanDataLength] = {0};
  DmMotorPackVelocity(&g_robot.yaw_motor, yaw_tx_data);

  FdcanTransmit(&hfdcan2, pitch_tx_data, kRobotCanDataLength,
                0x2FEU);
  FdcanTransmit(&hfdcan3, yaw_tx_data, kRobotCanDataLength,
                DmMotorTxId(&g_robot.yaw_motor));
}

static void RobotTransmitChassisStopCurrents(void) {
  uint8_t tx_data[kRobotCanDataLength] = {0};

  FdcanTransmit(&hfdcan2, tx_data, kRobotCanDataLength, 0x200U);
  FdcanTransmit(&hfdcan1, tx_data, kRobotCanDataLength, 0x1FEU);
}

static void RobotTransmitShootStopCurrents(void) {
  uint8_t friction_tx_data[kRobotCanDataLength] = {0};
  uint8_t bullet_tx_data[kRobotCanDataLength] = {0};

  FdcanTransmit(&hfdcan2, friction_tx_data, kRobotCanDataLength,
                0x200U);
  FdcanTransmit(&hfdcan3, bullet_tx_data, kRobotCanDataLength, 0x1FFU);
}

static bool RobotDt7IsOnline(void) {
  if (Dt7FrameCount() == 0U) {
    return false;
  }
  return (uint32_t)(osKernelGetTickCount() - Dt7LastUpdateTick()) <=
         kRobotDt7OfflineTimeoutMs;
}

static bool RobotIsSafeMode(void) {
  if (!RobotDt7IsOnline()) {
    return true;
  }
  return kRobotRemoteSwitchSafeModeEnabled != 0U &&
         SwitchIsDown(g_dt7_object.rocker.sw1);
}

static void RobotEnterGimbalSafeMode(void) {
  RobotStopGimbalMotors();
  RobotTransmitGimbalStopCurrents();
}

static void RobotEnterChassisSafeMode(void) {
  RobotChassisImuUpdate();
  RobotChassisGimbalHeadingUpdate(false);
  if (kRobotBigYawGyroCompensationEnabled != 0U) {
    g_robot.big_yaw_gyro_authorized = false;
    RobotBigYawControl();
  }
  RobotStopChassisMotors();
  RobotTransmitChassisStopCurrents();
}

static void RobotEnterShootSafeMode(void) {
  RobotStopShootMotors();
  RobotTransmitShootStopCurrents();
}

static void RobotUpdateVisionTx(void) {
  g_gimbal_to_vision.mode = 0U;
  g_gimbal_to_vision.q[0] = 1.0f;
  g_gimbal_to_vision.q[1] = 0.0f;
  g_gimbal_to_vision.q[2] = 0.0f;
  g_gimbal_to_vision.q[3] = 0.0f;
  g_gimbal_to_vision.yaw = g_robot.yaw_motor.state.angle;
  g_gimbal_to_vision.yaw_velocity = g_robot.yaw_motor.state.omega;
  g_gimbal_to_vision.pitch = g_robot.pitch_motor.state.angle;
  g_gimbal_to_vision.pitch_velocity = g_robot.pitch_motor.state.omega;
  g_gimbal_to_vision.bullet_speed =
      (kRobotRefereeEnabled != 0U) ? g_referee.shoot_data.bullet_speed
                                      : 0.0f;
  g_gimbal_to_vision.bullet_count = g_robot.bullet_count;
}

uint32_t ManagerTaskInit(void) {
  RobotSharedInit();
  return kRobotManagerPeriodMs;
}

static void RobotSendGimbalLinkStatus(void) __attribute__((unused));

static void RobotSendGimbalLinkStatus(void) {
  if (kRobotGimbalLinkEnabled == 0U || huart1.gState != HAL_UART_STATE_READY) {
    return;
  }
  static uint8_t sequence;
  const uint32_t now = HAL_GetTick();
  // Chassis task can update these fields; take one bounded snapshot.
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  bool yaw_ready =
      g_robot.yaw_feedback_valid &&
      (uint32_t)(now - g_robot.yaw_feedback_tick) <=
          kRobotBigYawFeedbackTimeoutMs;
  bool imu_ready = g_robot.chassis.imu_ready;
  float yaw_rate = g_robot.chassis.yaw_rate;
  bool host_control = g_robot.chassis_host_control_active;
  bool drive_ready = g_robot.chassis_drive_steering_ready;
  __set_PRIMASK(primask);
  uint8_t flags = yaw_ready ? kBoardLinkStatusYawReady : 0U;
  if (imu_ready) flags |= kBoardLinkStatusImuReady;
  if (host_control) flags |= kBoardLinkStatusHostControl;
  if (drive_ready) flags |= kBoardLinkStatusDriveReady;
  if (!imu_ready || !isfinite(yaw_rate)) yaw_rate = 0.0f;
  BoardLinkMessage message = {
      .type = kBoardLinkStatus, .sequence = sequence++, .flags = flags,
      .value = (int16_t)lroundf(RobotClamp(yaw_rate * 1000.0f, -32768.0f, 32767.0f)),
  };
  uint8_t frame[kBoardLinkFrameSize];
  BoardLinkEncode(&message, frame);
  UartTransmit(&huart1, frame, sizeof(frame));
}

/* Retired V1 call site kept in source for migration tooling: RobotSendGimbalLinkStatus(); */

static void RobotSendGimbalLinkV4Telemetry(void) {
  if (kRobotGimbalLinkEnabled == 0U) {
    return;
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  bool ready = g_robot.chassis_drive_steering_ready;
  bool host_control = g_robot.chassis_host_control_active;
  bool imu_ready = g_robot.chassis.imu_ready;
  float vx = g_robot.chassis.actual_velocity.vx;
  float vy = g_robot.chassis.actual_velocity.vy;
  float wz = g_robot.chassis.actual_velocity.omega_z;
  float power = g_robot.chassis.estimated_power;
  __set_PRIMASK(primask);

  uint8_t safety_state = host_control && ready
                             ? kBoardLinkSafetyActive
                             : kBoardLinkSafetyWaiting;
  uint32_t reasons = 0U;
  if (!imu_ready) {
    reasons |= 1U << 7; /* InvalidConfiguration/measurement unavailable. */
  }
  (void)GimbalLinkSendHeartbeat(safety_state, reasons, ready,
                                !GimbalLinkPeerOnline());
  /* This firmware has command and power estimates, not wheel odometry or a
   * calibrated power sensor. Keep valid_fields clear until those sources are
   * wired, while still reporting finite diagnostic values. */
  (void)GimbalLinkSendChassisFeedback(
      ready ? kBoardLinkExecutionRunning : kBoardLinkExecutionWaiting,
      safety_state, ready, ready, reasons, 0U, 0U, vx, vy, wz, power);
}

bool GimbalLinkTransportSend(const uint8_t *frame, uint16_t length) {
  if (frame == NULL || length == 0U ||
      huart1.gState != HAL_UART_STATE_READY) {
    return false;
  }
  UartTransmit(&huart1, (uint8_t *)frame, length);
  return true;
}

void ManagerTaskLoop(void) {
  if (kRobotRefereeEnabled != 0U) {
    RefereeUpdate(&g_referee);
  }
  RobotUpdateKeyboardMouseInput();
  RobotSendGimbalLinkV4Telemetry();
  RobotSendBigYawFeedback();
  if (kRobotGm6020TelemetryEnabled != 0U &&
      kRobotChassisEnabled != 0U && kRobotRefereeEnabled == 0U) {
    RobotSendGm6020Telemetry();
  }
  if (kRobotVisionEnabled != 0U) {
    RobotUpdateVisionTx();
    VisionProtocolTransmitGimbalState();
  }
}

uint32_t GimbalTaskInit(void) {
  if (kRobotGimbalEnabled == 0U) {
    return kRobotGimbalPeriodMs;
  }
  RobotSharedInit();
  return kRobotGimbalPeriodMs;
}

void GimbalTaskLoop(void) {
  if (kRobotGimbalEnabled == 0U) {
    return;
  }
  if (RobotIsSafeMode()) {
    RobotEnterGimbalSafeMode();
    return;
  }
  RobotGimbalControl();
}

uint32_t ChassisTaskInit(void) {
  if (kRobotChassisEnabled == 0U && kRobotBigYawEnabled == 0U) {
    return kRobotChassisPeriodMs;
  }
  RobotSharedInit();
  return kRobotChassisPeriodMs;
}

void ChassisTaskLoop(void) {
  g_robot.chassis_interboard_control_active = false;
  if (kRobotBigYawEnabled != 0U && kRobotBigYawGyroCompensationEnabled == 0U) {
    RobotBigYawControl();
  }
  if (kRobotChassisEnabled == 0U) {
    return;
  }

#if ROBOT_ENABLE_INTERBOARD_V4
  BoardLinkChassisControl interboard_command;
  if (kRobotGimbalLinkEnabled != 0U &&
      GimbalLinkGetChassisControl(&interboard_command)) {
    if (interboard_command.mode == kBoardLinkModeDisabled ||
        interboard_command.global_action != 2U) {
      RobotChassisSelectControlSource(true);
      RobotEnterChassisSafeMode();
      return;
    }
    HostVelocityCommand command = {
        .vx = interboard_command.vx_m_s,
        .vy = interboard_command.vy_m_s,
        .wz = interboard_command.wz_rad_s,
    };
    RobotChassisSelectControlSource(true);
    g_robot.chassis_interboard_control_active = true;
    RobotChassisControl(&command);
    return;
  }
  if (kRobotGimbalLinkEnabled != 0U && GimbalLinkPeerOnline()) {
    /* A live peer with no fresh control must not fall back to an old local
     * command source. */
    RobotChassisSelectControlSource(true);
    RobotEnterChassisSafeMode();
    return;
  }
#endif

  if (RobotDt7IsOnline()) {
    RobotChassisSelectControlSource(false);
    if (kRobotRemoteSwitchSafeModeEnabled != 0U &&
        SwitchIsDown(g_dt7_object.rocker.sw1)) {
      RobotEnterChassisSafeMode();
      return;
    }
    RobotChassisControl(NULL);
    return;
  }

  HostVelocityCommand command;
  if (kRobotChassisHostControlEnabled != 0U &&
      HostVelocityProtocolGetCommand(&command, kRobotHostCommandTimeoutMs)) {
    RobotChassisSelectControlSource(true);
    RobotChassisControl(&command);
    return;
  }
  RobotEnterChassisSafeMode();
}

uint32_t ShootTaskInit(void) {
  if (kRobotShootEnabled == 0U) {
    return kRobotShootPeriodMs;
  }
  RobotSharedInit();
  return kRobotShootPeriodMs;
}

void ShootTaskLoop(void) {
  if (kRobotShootEnabled == 0U) {
    return;
  }
  if (RobotIsSafeMode()) {
    RobotEnterShootSafeMode();
    return;
  }
  RobotShootControl();
}
