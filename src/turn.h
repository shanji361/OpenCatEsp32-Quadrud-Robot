#ifndef TURN_H
#define TURN_H

#include <Arduino.h>
#include "OpenCat.h"
#include "imu.h" // for icm or mpu yaw data

// Get yaw angle from active IMU and normalize it
float getYaw() {
#ifdef IMU_ICM42670
  if (icmQ)
    return fmod((icm.ypr[0] + 360.0), 360.0);  // Normalize to [0, 360)
#endif

#ifdef IMU_MPU6050
  if (mpuQ)
    return fmod((mpu.ypr[0] + 360.0), 360.0);
#endif

  return 0.0;  // fallback if no IMU active
}

// Compute shortest angle difference (±180°)
float angleDiff(float a, float b) {
  return ((a - b + 180.0f) - floor((a - b + 180.0f) / 360.0f) * 360.0f) - 180.0f;
}

// Perform a 90-degree right turn based on yaw
void turnRight90DegreesFirmware() {
  PTL("Starting 90-degree right turn...");

  float initialYaw = getYaw();
  float targetYaw = fmod(initialYaw + 90.0, 360.0);
  float currentYaw = initialYaw;

  PT("Initial Yaw: "); PTL(initialYaw);
  PT("Target Yaw: "); PTL(targetYaw);

  delay(200);  // Give robot time to settle

  while (abs(angleDiff(currentYaw, targetYaw)) > 5.0) {
    tQueue->addTask(T_GAIT, "vtR");  // Turn right gait
    delay(600);                      // Let it move a bit
    currentYaw = getYaw();
    PT("Current Yaw: "); PTL(currentYaw);
  }

  tQueue->addTask(T_SKILL, "balance");  // Return to neutral
  PTL("Turn complete.");
}

#endif
