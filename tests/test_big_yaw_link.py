"""Host regression for the chassis-side v4 BigYawRequest snapshot."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
stub = """
#include <stdint.h>
extern uint32_t tick, primask;
static inline uint32_t HAL_GetTick(void) { return tick; }
static inline uint32_t __get_PRIMASK(void) { return primask; }
static inline void __disable_irq(void) { primask = 1; }
static inline void __set_PRIMASK(uint32_t value) { primask = value; }
"""
harness = r"""
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "app_gimbal_link.h"

uint32_t tick, primask;

static void near(float a, float b) { assert(fabsf(a - b) < 0.0001f); }

int main(void) {
  uint8_t frame[kBoardLinkMaxFrameSize];
  uint16_t length = 0U;
  GimbalLinkInit();
  tick = 100U;

  BoardLinkBigYawRequest request = {
      .mode = kBoardLinkBigYawFollowCenter,
      .permission_valid = true,
      .permission_enabled = true,
      .target_rate_rad_s = 0.25f,
      .source_sequence = 10U,
      .source_age_ms = 0U,
      .command_age_ms = 0U,
      .receiver_boot_id = GimbalLinkLocalBootId(),
      .permission_age_ms = 0U,
  };
  BoardLinkBigYawRequest decoded;

  assert(!GimbalLinkGetBigYawRequest(&decoded));
  assert(BoardLinkEncodeBigYawRequest(&request, 1U, frame, sizeof(frame),
                                      &length));
  GimbalLinkRxCallback(frame, length);
  assert(GimbalLinkGetBigYawRequest(&decoded));
  near(decoded.target_rate_rad_s, 0.25f);
  assert(decoded.mode == kBoardLinkBigYawFollowCenter);
  assert(decoded.permission_valid && decoded.permission_enabled);

  /* An older producer sequence must not replace the current target. */
  request.source_sequence = 9U;
  request.target_rate_rad_s = -0.5f;
  assert(BoardLinkEncodeBigYawRequest(&request, 2U, frame, sizeof(frame),
                                      &length));
  GimbalLinkRxCallback(frame, length);
  assert(GimbalLinkGetBigYawRequest(&decoded));
  near(decoded.target_rate_rad_s, 0.25f);

  /* A request bound to another chassis boot id must be rejected. */
  request.source_sequence = 11U;
  request.receiver_boot_id = GimbalLinkLocalBootId() + 1U;
  assert(BoardLinkEncodeBigYawRequest(&request, 3U, frame, sizeof(frame),
                                      &length));
  GimbalLinkRxCallback(frame, length);
  assert(GimbalLinkGetBigYawRequest(&decoded));
  near(decoded.target_rate_rad_s, 0.25f);

  /* Local receive timeout follows the configured BigYaw request window. */
  tick = 100U + kGimbalLinkBigYawTimeoutMs;
  assert(GimbalLinkGetBigYawRequest(&decoded));
  tick = 100U + kGimbalLinkBigYawTimeoutMs + 1U;
  assert(!GimbalLinkGetBigYawRequest(&decoded));

  /* A fresh request for this boot id is accepted again. */
  request.receiver_boot_id = GimbalLinkLocalBootId();
  request.source_sequence = 12U;
  request.target_rate_rad_s = 0.1f;
  assert(BoardLinkEncodeBigYawRequest(&request, 4U, frame, sizeof(frame),
                                      &length));
  GimbalLinkRxCallback(frame, length);
  assert(GimbalLinkGetBigYawRequest(&decoded));
  near(decoded.target_rate_rad_s, 0.1f);
  puts("PASS: big-yaw request boot/sequence/freshness");
  return 0;
}
"""

if __name__ == "__main__":
    with tempfile.TemporaryDirectory(prefix="big_yaw_link_") as directory:
        work = Path(directory)
        (work / "stm32h7xx_hal.h").write_text(stub)
        (work / "test.c").write_text(harness)
        exe = work / "test"
        subprocess.run([
            "gcc", "-std=gnu11", "-Wall", "-Wextra", "-Werror",
            "-I", str(work), "-I", str(ROOT / "Skywalker/Algorithm/Inc"),
            "-I", str(ROOT / "Skywalker/Application/Inc"), str(work / "test.c"),
            str(ROOT / "Skywalker/Algorithm/Src/alg_board_link.c"),
            str(ROOT / "Skywalker/Application/Src/app_gimbal_link.c"),
            "-lm", "-o", str(exe),
        ], check=True)
        subprocess.run([str(exe)], check=True)
