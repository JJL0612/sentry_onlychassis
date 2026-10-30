"""Run the task's real IMU/gyro velocity loop, safety and native velocity CAN packing."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "Skywalker/Application/Src/app_robot.c").read_text()


def function(name):
    match = re.search(r"^(?:static )?[^\n]*\b" + name +
                      r"\([^{};]*\) \{.*?^\}", source, re.M | re.S)
    assert match, name
    return match.group()


config = re.search(r"^enum \{.*?^};", source, re.M | re.S).group()
state = source[source.index("typedef struct {"):source.index("static float RobotClamp")]
constants = "\n".join(re.search(r"^static const float " + name + r"\b.*?;",
                                 source, re.M | re.S).group() for name in (
    "kRobotChassisYawWheelDeadband", "kRobotBigYawGyroDirection", "kRobotBigYawGyroRatio", "kRobotBigYawGyroMaxRateRadps",
    "kRobotImuYawRateSign", "kRobotBigYawTorqueLimitNm", "kRobotBigYawMaxRateRadps",
    "kRobotBigYawCascadeFallbackDtS", "kRobotBigYawCascadeMaxDtS",
    "kRobotBigYawMaxPositionErrorRad", "kRobotBigYawPositionOutputMaxRadps",
))
motor_init = re.search(r"DmMotorInit\(&g_robot.yaw_motor,.*?;", source, re.S).group()
pid_configs = "\n".join(re.search(r"static PidConfig " + name + r" = \{.*?\n  };",
                                  source, re.S).group() for name in (
    "big_yaw_position_pid", "big_yaw_velocity_pid"))
prefix = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "dev_dji_motor.h"
#include "dev_dm_motor.h"
#include "app_gimbal_link.h"
#include "app_host_protocol.h"
#define ROBOT_CHASSIS_NO_MOTOR_TX_TEST 0
#define ROBOT_ENABLE_INTERBOARD_V4 1
typedef int Bmi088Status;
enum { kBmi088NoError = 0, kBmi088NoSensor = 255 };
"""
stubs = r"""
static uint32_t tick = 100, primask, rc_frames, rc_tick;
static int hfdcan3, sensor_status;
static float sensor_rate, sent_torque, sent_rate;
static unsigned gimbal_imu_reads, imu_reads, enable_calls, disable_calls, clear_calls, chassis_calls, stops;
static bool board_available = true, peer_online = true, host_available, host_selected;
static bool request_available;
static bool external_imu_available = true;
static GimbalImuYaw external_imu = {.sender_boot_id=42, .zero_generation=1, .valid=true};
bool GimbalLinkGetImuYaw(GimbalImuYaw *out) {
  ++gimbal_imu_reads; *out = external_imu; return external_imu_available && peer_online;
}
static BoardLinkChassisControl board = {.mode = 1, .global_action = 2};
static BoardLinkBigYawRequest request;
static struct { struct { int16_t ch0, wheel; uint8_t sw1; } rocker; } g_dt7_object;
static struct {
  bool is_inited, is_gyro_offset_inited, is_heater_enabled;
  float corrected_gyro[3];
} g_bmi088;
static uint32_t osKernelGetTickCount(void) { return tick; }
static uint32_t HAL_GetTick(void) { return tick; }
static uint32_t __get_PRIMASK(void) { return primask; }
static void __disable_irq(void) { primask = 1; }
static void __set_PRIMASK(uint32_t value) { primask = value; }
static uint32_t Dt7FrameCount(void) { return rc_frames; }
static uint32_t Dt7LastUpdateTick(void) { return rc_tick; }
static bool SwitchIsDown(uint8_t v) { return v == 2; }
static Bmi088Status Bmi088Update(void *object) {
  assert(object == &g_bmi088); ++imu_reads;
  g_bmi088.corrected_gyro[kRobotImuGyroZAxis] = sensor_rate;
  return sensor_status;
}
static float Bmi088HeaterControl(void *object) { (void)object; return 0; }
bool GimbalLinkGetChassisControl(BoardLinkChassisControl *out) {
  *out = board; return board_available;
}
bool GimbalLinkGetBigYawRequest(BoardLinkBigYawRequest *out) {
  *out = request; return request_available;
}
bool GimbalLinkPeerOnline(void) { return peer_online; }
bool HostVelocityProtocolGetCommand(HostVelocityCommand *out, uint32_t age) {
  (void)age; *out = (HostVelocityCommand){.wz = 0.8f}; return host_available;
}
static void FdcanTransmit(int *bus, uint8_t *data, uint16_t length, uint32_t id) {
  assert(bus == &hfdcan3);
  assert(id == (kRobotBigYawGyroCompensationEnabled ? 0x206U : 6U));
  bool command = length == 8;
  for (int i = 0; i < 7; ++i) command = command && data[i] == 0xFF;
  if (command) {
    if (data[7] == kDmMotorCommandEnable) ++enable_calls;
    else if (data[7] == kDmMotorCommandDisable) ++disable_calls;
    else { assert(data[7] == kDmMotorCommandClearError); ++clear_calls; }
    return;
  }
  if (kRobotBigYawGyroCompensationEnabled) {
    assert(length == 4);
    memcpy(&sent_rate, data, sizeof(sent_rate));
    for (int i=4;i<8;i++) assert(data[i]==0);
    assert(isfinite(sent_rate) && fabsf(sent_rate)<=kRobotBigYawGyroMaxRateRadps);
    return;
  }
  assert(length == 8);
  // MIT p/v/kp/kd fields remain zero targets/gains; decode actual wire torque.
  assert((data[3] & 15) == 0 && data[4] == 0 && data[5] == 0 && (data[6] >> 4) == 0);
  unsigned raw = ((unsigned)(data[6] & 15) << 8) | data[7];
  sent_torque = kDmMotorTorqueMin + raw *
      (kDmMotorTorqueMax - kDmMotorTorqueMin) / 4095.0f;
  assert(isfinite(sent_torque) && fabsf(sent_rate) <= kRobotBigYawTorqueLimitNm + 0.01f);
}
static void RobotChassisSelectControlSource(bool host) { host_selected = host; }
static void RobotStopChassisMotors(void) { ++stops; }
static void RobotTransmitChassisStopCurrents(void) {}
static void RobotChassisImuUpdate(void);
static void RobotChassisGimbalHeadingUpdate(bool authorized) { (void)authorized; }
"""
# The real entry to chassis control samples the IMU and invokes yaw control.
# Wheel geometry and PID/CAN outputs are covered by the existing spin tests.
control_entry = function("RobotChassisControl").split(
    "  RobotChassisKinematics(host_command);")[0] + r"""
  (void)host_command;
  assert(g_robot.chassis_imu_last_tick == tick);
  ++chassis_calls;
}
"""
functions = "\n".join(function(name) for name in (
    "RobotClamp", "RobotWrapPi", "RobotRcNormalize", "RobotApplyDeadband", "RobotBigYawEncoderUpdate",
    "RobotChassisImuUpdate", "RobotBigYawSendCommand", "RobotBigYawSendMitTorque",
    "RobotBigYawSendVelocity", "RobotBigYawSendStop", "RobotBigYawResetCascade", "RobotBigYawStabilizedRate", "RobotBigYawControl", "RobotDt7IsOnline",
    "RobotEnterChassisSafeMode", "ChassisTaskLoop",
))
tests = r"""
static void feedback(void) {
  RobotBigYawEncoderUpdate();
  g_robot.yaw_feedback_valid = true;
  g_robot.yaw_feedback_tick = tick;
}
static void step(bool with_feedback) {
  tick += 2; rc_tick = tick;
  if (with_feedback) feedback();
  unsigned reads = imu_reads;
  ChassisTaskLoop();
  assert(imu_reads == reads + (g_bmi088.is_inited && g_bmi088.is_gyro_offset_inited ? 1U : 0U));
}
static void settle(void) { for (int i = 0; i < 8; ++i) step(true); }
static void near(float a, float b) { assert(fabsf(a - b) < 0.0001f); }
int main(void) {
  InitMotor();
  PidInit(&g_robot.big_yaw_position_pid, &big_yaw_position_pid);
  PidInit(&g_robot.big_yaw_velocity_pid, &big_yaw_velocity_pid);
  g_bmi088.is_inited = g_bmi088.is_gyro_offset_inited = true;
  request_available=true;
  request=(BoardLinkBigYawRequest){.mode=kBoardLinkBigYawFollowCenter,
    .permission_valid=true,.permission_enabled=true};
  sensor_rate=2;settle();
  assert(g_robot.big_yaw_active && g_robot.big_yaw_gyro_active);
  assert(g_robot.yaw_motor.mode==kDmMotorModeVelocity);
  near(sent_rate,kRobotBigYawGyroDirection*kRobotBigYawGyroRatio*(-2));
  sensor_rate=-3;step(true);
  near(sent_rate,kRobotBigYawGyroDirection*kRobotBigYawGyroRatio*3);
  // Feedforward uses the measured base rate, never the requested spin rate.
  board.wz_rad_s=3.4f;sensor_rate=0;step(true);near(sent_rate,0);
  external_imu.yaw_delta_rad=0.1f;step(true);
  assert(g_robot.big_yaw_heading_error_rad < -0.09f);
  assert(sent_rate/kRobotBigYawGyroDirection < 0); // restore displaced heading
  external_imu.yaw_delta_rad=0;external_imu.yaw_rate_rad_s=0.2f;
  RobotBigYawResetCascade();step(true);
  assert(sent_rate/kRobotBigYawGyroDirection < 0); // reject residual world rate
  external_imu.yaw_rate_rad_s=0;
  request.target_rate_rad_s=0.5f;step(true);
  assert(g_robot.big_yaw_world_rate_target_radps>0.5f);
  external_imu.yaw_delta_rad=0.25f;request.target_rate_rad_s=0;step(true);
  near(g_robot.big_yaw_heading_target_rad,0.25f);
  near(g_robot.big_yaw_heading_error_rad,0);
  // Origin changes recapture instead of chasing an old heading.
  external_imu.sender_boot_id++;external_imu.yaw_delta_rad=1;step(true);
  near(g_robot.big_yaw_heading_error_rad,0);
  external_imu.zero_generation++;external_imu.yaw_delta_rad=-1;step(true);
  near(g_robot.big_yaw_heading_error_rad,0);
  // Shortest heading error across +/-pi and age prediction.
  RobotBigYawResetCascade();external_imu.yaw_delta_rad=3.13f;step(true);
  external_imu.yaw_delta_rad=-3.13f;step(true);
  assert(fabsf(g_robot.big_yaw_heading_error_rad)<0.03f);
  RobotBigYawResetCascade();external_imu.yaw_delta_rad=0.1f;
  external_imu.yaw_rate_rad_s=0.2f;external_imu.source_age_ms=20;step(true);
  near(g_robot.big_yaw_heading_target_rad,0.104f);
  external_imu.source_age_ms=0;external_imu.yaw_rate_rad_s=0;
  // Missing, invalid, stale and nonfinite gimbal data disable; recover at new heading.
  for(int fault=0;fault<5;fault++) {
    external_imu_available=fault!=0;external_imu.valid=fault!=1;
    external_imu.source_age_ms=fault==2?51:0;
    external_imu.yaw_delta_rad=fault==3?NAN:0.7f;
    external_imu.yaw_rate_rad_s=fault==4?NAN:0;
    step(true);assert(!g_robot.big_yaw_active);near(sent_rate,0);
    assert(!g_robot.big_yaw_heading_initialized);
    external_imu_available=true;external_imu.valid=true;external_imu.source_age_ms=0;
    external_imu.yaw_delta_rad=0.9f;external_imu.yaw_rate_rad_s=0;settle();
    assert(g_robot.big_yaw_active);near(g_robot.big_yaw_heading_target_rad,0.9f);
  }
  sensor_status=kBmi088NoSensor;step(true);assert(!g_robot.big_yaw_active);near(sent_rate,0);
  sensor_status=kBmi088NoError;sensor_rate=NAN;step(true);assert(!g_robot.big_yaw_active);
  sensor_rate=0;settle();assert(g_robot.big_yaw_active);
  request.permission_enabled=false;step(true);assert(!g_robot.big_yaw_active);
  request.permission_enabled=true;settle();assert(g_robot.big_yaw_active);
  board.global_action=0;step(true);assert(!g_robot.big_yaw_active);
  board.global_action=2;settle();assert(g_robot.big_yaw_active);
  tick+=kRobotBigYawFeedbackTimeoutMs+1;step(false);assert(!g_robot.big_yaw_active);
  settle();assert(g_robot.big_yaw_active);
  g_robot.yaw_motor.state.error_code=8;step(true);unsigned clears=clear_calls;
  step(true);assert(clear_calls==clears&&!g_robot.big_yaw_active);
  g_robot.yaw_motor.state.error_code=0;settle();assert(g_robot.big_yaw_active);
  // Anti-windup includes saturated base-rate feedforward.
  sensor_rate=34;external_imu.yaw_rate_rad_s=1;
  RobotBigYawResetCascade();settle();
  near(sent_rate,kRobotBigYawGyroMaxRateRadps);near(g_robot.big_yaw_rate_integral,0);
  // Simulated 20ms motor lag, 10% gain error and 0.03rad/s base gyro bias.
  // Drive a 4s 2..3.4rad/s spin, then reverse; test wheel motion/release as well.
  sensor_rate=0;external_imu.yaw_rate_rad_s=0;external_imu.yaw_delta_rad=0;
  RobotBigYawResetCascade();settle();
  float heading=0, relative=0, previous_world=0, max_error=0;
  for(int i=0;i<60000;i++) {
    float t=i*0.002f;
    float base=(2.7f+0.7f*cosf(2*(float)M_PI*t/4))*(i<30000?1:-1);
    sensor_rate=base+0.03f;
    request.target_rate_rad_s=(i>=10000&&i<12000)?0.5f:0;
    // Sample every 20ms; report actual sample age between packets.
    if(i%10==0) {
      external_imu.yaw_delta_rad=heading;
      external_imu.yaw_rate_rad_s=previous_world;
    }
    external_imu.source_age_ms=(i%10)*2;
    step(true);assert(g_robot.big_yaw_active);
    float wanted_relative=sent_rate/(kRobotBigYawGyroDirection*kRobotBigYawGyroRatio);
    relative+=0.002f/0.020f*(0.9f*wanted_relative-relative);
    previous_world=base+relative;
    heading=RobotWrapPi(heading+previous_world*0.002f);
    if(i>15000)max_error=fmaxf(max_error,fabsf(RobotWrapPi(heading-g_robot.big_yaw_heading_target_rad)));
  }
  printf("Simulated heading error: final %.5f rad, peak after wheel release %.5f rad\n",
    fabsf(RobotWrapPi(heading-g_robot.big_yaw_heading_target_rad)),max_error);
  assert(fabsf(RobotWrapPi(heading-g_robot.big_yaw_heading_target_rad))<0.03f);
  assert(max_error<0.2f);
  // Lost board link cannot silently continue base-only cancellation.
  peer_online=false;step(true);assert(!g_robot.big_yaw_active);
  assert(gimbal_imu_reads>0);
  puts("PASS: gimbal heading/rate feedback + base feedforward, release, origins, IMU loss, permissions, faults, saturation and DLC4");
}
"""
legacy_tests = tests.split("int main(void)")[0] + r"""
int main(void) {
  InitMotor();
  PidInit(&g_robot.big_yaw_position_pid, &big_yaw_position_pid);
  PidInit(&g_robot.big_yaw_velocity_pid, &big_yaw_velocity_pid);
  g_bmi088.is_inited = g_bmi088.is_gyro_offset_inited = true;
  sensor_rate = 2;
  request_available = true;
  request = (BoardLinkBigYawRequest){.mode = kBoardLinkBigYawFollowCenter,
      .permission_valid = true, .permission_enabled = true, .target_rate_rad_s = 0.25f};
  settle(); near(g_robot.big_yaw_target_rate, -0.25f);
  assert(g_robot.big_yaw_active && !g_robot.big_yaw_gyro_active && sent_torque < 0);
  // A fresh revoked request still prevents wheel fallback in encoder mode.
  request.permission_enabled = false; rc_frames = 1; g_dt7_object.rocker.wheel = 660;
  step(true); assert(!g_robot.big_yaw_active && fabsf(sent_torque) < 0.01f);
  request_available = false; settle(); near(g_robot.big_yaw_target_rate, -1);
  assert(g_robot.big_yaw_manual_active && g_robot.big_yaw_active);
  // No gimbal IMU reads, even with drift, invalid samples or missing sensors.
  g_dt7_object.rocker.wheel = 0;
  external_imu_available = false;
  g_bmi088.is_inited = g_bmi088.is_gyro_offset_inited = false;
  g_robot.chassis.imu_ready = false;
  g_robot.yaw_motor.state.angle = 0.3f;
  RobotBigYawResetCascade();
  settle();
  const float held = g_robot.big_yaw_target_angle;
  for (int i=0; i<1000; ++i) {
    external_imu.yaw_delta_rad = i * 0.01f;
    external_imu.yaw_rate_rad_s = NAN;
    external_imu_available = i % 2;
    step(true);
    assert(g_robot.big_yaw_active && !g_robot.big_yaw_gyro_active);
    near(g_robot.big_yaw_target_angle, held);
    near(g_robot.big_yaw_target_rate, 0);
  }
  assert(gimbal_imu_reads == 0);
  // Position error is corrected using the encoder alone.
  g_robot.yaw_motor.state.angle = held + 0.1f;
  step(true); assert(sent_torque < 0);
  // Lost motor feedback still disables output.
  for(int i=0;i<60;++i) step(false);
  assert(!g_robot.big_yaw_active && !g_robot.big_yaw_enabled);
  puts("PASS: encoder cascade, wheel/revocation, fixed hold, zero gimbal IMU reads, sensor loss and motor timeout");
}
"""
with tempfile.TemporaryDirectory(prefix="big_yaw_gyro_") as directory:
    work = Path(directory)
    for mode, cases in ((1, tests), (0, legacy_tests)):
        selected = re.sub(r"kRobotBigYawGyroCompensationEnabled = [01]U",
                                  f"kRobotBigYawGyroCompensationEnabled = {mode}U", config)
        (work / "test.c").write_text(prefix + selected + state + constants + pid_configs +
                                     stubs + "static void InitMotor(void) {" + motor_init + "}\n" +
                                     control_entry + functions + cases)
        subprocess.run([
            "gcc", "-std=gnu11", "-Wall", "-Wextra", "-Werror",
            "-I", str(ROOT / "Skywalker/Algorithm/Inc"),
            "-I", str(ROOT / "Skywalker/Device/Inc"),
            "-I", str(ROOT / "Skywalker/Application/Inc"), str(work / "test.c"),
            str(ROOT / "Skywalker/Device/Src/dev_dm_motor.c"),
            str(ROOT / "Skywalker/Algorithm/Src/alg_pid.c"), "-lm", "-o", str(work / "test")], check=True)
        subprocess.run([str(work / "test")], check=True)
