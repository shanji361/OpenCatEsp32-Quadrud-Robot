#ifndef TURN_H
#define TURN_H 
// State variables for non-blocking turn
bool turningLeft = false;
bool turningRight = false;
float turnStartYaw = 0;
int turnAttempts = 0;
unsigned long lastTurnTime = 0;



void startTurnLeft90() {
  if (!turningLeft && !turningRight) {
    turningLeft = true;
    turnAttempts = 0;
    lastTurnTime = millis();
    
#ifdef IMU_ICM42670
    if (icmQ) {
      turnStartYaw = icm.ypr[0];
    }
#endif
#ifdef IMU_MPU6050
    if (mpuQ) {
      turnStartYaw = mpu.ypr[0];
    }
#endif

    PTL("Starting Left Turn 90 Degrees...");
    PT("Start Yaw: "); PTL(turnStartYaw);
    
    // Add the first turn task
    tQueue->addTask('k', "vtL", 700);
  }
}

void startTurnRight90() {
  if (!turningLeft && !turningRight) {
    turningRight = true;
    turnAttempts = 0;
    lastTurnTime = millis();
    
#ifdef IMU_ICM42670
    if (icmQ) {
      turnStartYaw = icm.ypr[0];
    }
#endif
#ifdef IMU_MPU6050
    if (mpuQ) {
      turnStartYaw = mpu.ypr[0];
    }
#endif

    PTL("Starting Right Turn 90 Degrees...");
    PT("Start Yaw: "); PTL(turnStartYaw);
    
    // Add the first turn task
    tQueue->addTask('k', "vtR", 700);
  }
}

void checkTurnProgress() {
  if (!turningLeft && !turningRight) return;
  
  // Check progress every 800ms (700ms task + 100ms buffer)
  if (millis() - lastTurnTime > 800) {
    float currentYaw = 0.0;
    
#ifdef IMU_ICM42670
    if (icmQ) {
      currentYaw = icm.ypr[0];
    }
#endif
#ifdef IMU_MPU6050
    if (mpuQ) {
      currentYaw = mpu.ypr[0];
    }
#endif

    float yawDiff;
    if (turningLeft) {
      yawDiff = turnStartYaw - currentYaw;  // Left turn: positive difference
    } else {
      yawDiff = currentYaw - turnStartYaw;  // Right turn: positive difference
    }
    
    if (yawDiff < -180.0) yawDiff += 360.0;
    if (yawDiff > 180.0) yawDiff -= 360.0;

    PT("Current Yaw: "); PTL(currentYaw);
    PT("Yaw Diff: "); PTL(yawDiff);
    PT("Attempt: "); PTL(turnAttempts + 1);

    if (abs(yawDiff) >= 78.0) {
      if (turningLeft) {
        PTL("Left turn complete!");
      } else {
        PTL("Right turn complete!");
      }
      turningLeft = false;
      turningRight = false;
      tQueue->addTask('k', "balance", 500);
    } else if (turnAttempts < 10) {
      // Continue turning
      turnAttempts++;
      if (turningLeft) {
        tQueue->addTask('k', "vtL", 400);
      } else {
        tQueue->addTask('k', "vtR", 400);
      }
      lastTurnTime = millis();
    } else {
      PTL("Turn failed - max attempts reached");
      turningLeft = false;
      turningRight = false;
      tQueue->addTask('k', "balance", 500);
    }
  }
}
#endif
