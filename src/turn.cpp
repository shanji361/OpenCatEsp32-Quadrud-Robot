#include "turn.h"
#include "imu.h"          // for ypr[], icm, mpu, etc.
#include "motion.h"       // for tQueue and T_GAIT
#include "SerialCommand.h" // for PT(), PTL()

float getYaw() {
#ifdef IMU_ICM42670
  return fmod((icm.ypr[0] + 360.0), 360.0);
#elif defined(IMU_MPU6050)
  return fmod((mpu.ypr[0] + 360.0), 360.0);
#else
  return 0;  // fallback if no IMU is defined
#endif
}

float angleDiff(float a, float b) {
  return ((a - b + 180.0f) - floor((a - b + 180.0f) / 360.0f) * 360.0f) - 180.0f;
}

void turnRight90DegreesFirmware() {
  PTL("Starting 90-degree right turn...");

  float initialYaw = getYaw();
  float targetYaw = fmod(initialYaw + 90.0, 360.0);
  float currentYaw = initialYaw;

  PT("Initial Yaw: "); PTL(initialYaw);
  PT("Target Yaw: "); PTL(targetYaw);

  tQueue->addTask('k', "balance");  // stop gait control if needed
  delay(200);

  while (abs(angleDiff(currentYaw, targetYaw)) > 5.0) {
    tQueue->addTask('m', "vtR");
    delay(600);
    currentYaw = getYaw();
    PT("Current Yaw: "); PTL(currentYaw);
  }

  tQueue->addTask('k', "balance");
  PTL("Turn complete.");
}
