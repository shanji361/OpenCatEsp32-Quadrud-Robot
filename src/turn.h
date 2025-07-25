#ifndef TURN_H
#define TURN_H

#include "imu.h"
#include "taskQueue.h"

extern TaskQueue* tQueue;

float normalizeYaw(float yaw) {
  while (yaw < -180) yaw += 360;
  while (yaw > 180) yaw -= 360;
  return yaw;
}

float getYaw() {
#ifdef IMU_ICM42670
  if (icmQ)
    return icm.ypr[0];
#endif
#ifdef IMU_MPU6050
  if (mpuQ)
    return mpu.ypr[0];
#endif
  return 0;
}

// blocking turn function
void turnLeft90() {
  float initialYaw = normalizeYaw(getYaw());
  float targetYaw = normalizeYaw(initialYaw - 90);

  PTL("Starting 90 deg turn");
  while (true) {
    float currentYaw = normalizeYaw(getYaw());
    float delta = normalizeYaw(currentYaw - targetYaw);

    PT("Current Yaw: "); PT(currentYaw); PT("\tTarget: "); PTL(targetYaw);

    if (abs(delta) < 5) {  // within 5° of target
      PTL("Turn complete");
      break;
    }

    tQueue->addTask('k', "vtL", 300); // use gait to turn left
    delay(400); // give time for move before polling IMU again
  }

  tQueue->addTask('k', "balance", 500); // recover position
}

#endif
