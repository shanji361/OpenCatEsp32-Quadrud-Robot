// modify the model and board definitions
//***********************
#define BITTLE  // Petoi 9 DOF robot dog: 1 on head + 8 on leg
// #define NYBBLE  // Petoi 11 DOF robot cat: 2 on head + 1 on tail + 8 on leg
// #define VT
// #define CUB
// #define MINI


// #define BiBoard_V0_1  //ESP32 Board with 12 channels of built-in PWM for joints
#define BiBoard_V0_2
//#define BiBoard_V1_0
// #define BiBoard2  //ESP32 Board with 16 channels of PCA9685 PWM for joints
//***********************

// Send '!' token to reset the birthmark in the EEPROM so that the robot will restart to reset
// #define AUTO_INIT  //activate it to automatically reset joint and imu calibration without prompts

// you can also activate the following modes by the 'X' token defined in src/OpenCat.h
#define VOICE                     // Petoi Grove voice module
#define ULTRASONIC                // for Petoi RGB ultrasonic distance sensor
#define PIR                       // for PIR (Passive Infrared) sensor
#define DOUBLE_TOUCH              // for double touch sensor
#define DOUBLE_LIGHT              // for double light sensor
#define DOUBLE_INFRARED_DISTANCE  // for double distance sensor
#define GESTURE                   // for Gesture module
#define CAMERA                    // for Mu Vision camera
#define QUICK_DEMO                // for quick demo
// #define ROBOT_ARM                 // for attaching head clip arm
#include "src/OpenCat.h"

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

    if (abs(yawDiff) >= 85.0) {
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
        tQueue->addTask('k', "vtL", 700);
      } else {
        tQueue->addTask('k', "vtR", 700);
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);  // USB serial
  Serial.setTimeout(SERIAL_TIMEOUT);
  //  Serial1.begin(115200); //second serial port
  while (Serial.available() && Serial.read())
    ;  // empty buffer
  initRobot();
}

void loop() {
#ifdef VOLTAGE
  lowBattery();
#endif
  //  //—self-initiative
  //  if (autoSwitch) { //the switch can be toggled on/off by the 'z' token
  //    randomMind();//allow the robot to auto rest and do random stuff in randomMind.h
  //    powerSaver(POWER_SAVER);//make the robot rest after a certain period, the unit is seconds
  //
  //  }
  //  //— read environment sensors (low level)
  readEnvironment();  // update the gyro data
  //  //— special behaviors based on sensor events
  dealWithExceptions();  // low battery, fall over, lifted, etc.
  print6Axis();
  
  // Check turn progress every loop iteration
  checkTurnProgress();
  
  if (!tQueue->cleared()) {
    tQueue->popTask();
  } else {
    readSignal();
    if (token == 'E') {
      startTurnLeft90();  // Left turn
    }
    if (token == 'e') {
      startTurnRight90(); // Right turn
    }
    
#ifdef QUICK_DEMO
    if (moduleList[moduleIndex] == EXTENSION_QUICK_DEMO)
      quickDemo();
#endif
    //  readHuman();
  }
  //  //— generate behavior by fusing all sensors and instruction
  //  decision();

  //  //— action
  //  //playSound();
#ifdef NEOPIXEL_PIN
  playLight();
#endif
  reaction();

#ifdef WEB_SERVER
  WebServerLoop();  // 处理异步Web请求
#endif
}

#ifdef QUICK_DEMO  // enter XQ in the serial monitor to activate the following section
int prevReading = 0;
void quickDemo() {  // this is an example that use the analog input pin ANALOG1 as a touch pad
  // if the pin is not connected to anything, the reading will be noise
  int currentReading = analogRead(ANALOG1);
  if (abs(currentReading - prevReading) > 50)  // if the reading is significantly different from the previous reading
  {
    PT("Reading on pin ANALOG1:\t");
    PTL(currentReading);
    if (currentReading < 50) {                                        // touch and hold on the A2 pin until the condition is met
      tQueue->addTask(T_BEEP, "12 4 14 4 16 2");                      // make melody
      tQueue->addTask(T_INDEXED_SEQUENTIAL_ASC, "0 30 0 -30", 1000);  // move the neck, left shoulder, right shoulder one by one
    } else if (abs(currentReading - prevReading) < 100) {
      if (strcmp(lastCmd, "sit"))
        tQueue->addTask(T_SKILL, "sit", 1000);  // make the robot sit. more tokens are defined in OpenCat.h
    } else {
      if (strcmp(lastCmd, "up"))
        tQueue->addTask(T_SKILL, "up", 1000);  // make the robot stand up. more tokens are defined in OpenCat.h
    }
  }
  prevReading = currentReading;
}
#endif
