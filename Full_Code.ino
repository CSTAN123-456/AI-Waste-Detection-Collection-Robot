#include <Arduino.h>
#include <ESP32Servo.h>
#include <esp_arduino_version.h>

// ============================================================
// MOTOR PINS
// ============================================================

const int LEFT_RPWM  = 33;
const int LEFT_LPWM  = 32;

const int RIGHT_RPWM = 12;
const int RIGHT_LPWM = 13;

// ============================================================
// MOTOR PWM CHANNELS
//
// High-numbered channels are used to reduce the chance of
// interfering with ESP32Servo channels.
// ============================================================

const int LEFT_RPWM_CHANNEL  = 12;
const int LEFT_LPWM_CHANNEL  = 13;
const int RIGHT_RPWM_CHANNEL = 14;
const int RIGHT_LPWM_CHANNEL = 15;

const int MOTOR_PWM_FREQUENCY = 1000;
const int MOTOR_PWM_RESOLUTION = 8;

// ============================================================
// ARM SERVO PINS
// ============================================================

const int SHOULDER_PIN = 16;
const int ELBOW_PIN    = 17;
const int WRIST_PIN    = 18;
const int ROTATE_PIN   = 19;
const int GRIPPER_PIN  = 21;

// ============================================================
// ULTRASONIC SENSOR PINS
//
// Physical sequence from left to right:
//
// 1. Left 45-degree
// 2. Left straight
// 3. Centre below arm
// 4. Right straight
// 5. Right 45-degree
// ============================================================

const int LEFT_45_TRIG_PIN = 25;
const int LEFT_45_ECHO_PIN = 36;

const int LEFT_TRIG_PIN = 26;
const int LEFT_ECHO_PIN = 39;

const int CENTER_TRIG_PIN = 27;
const int CENTER_ECHO_PIN = 34;

const int RIGHT_TRIG_PIN = 22;
const int RIGHT_ECHO_PIN = 14;

const int RIGHT_45_TRIG_PIN = 23;
const int RIGHT_45_ECHO_PIN = 35;

// ============================================================
// ULTRASONIC SETTINGS
// ============================================================

const unsigned long ECHO_TIMEOUT_US = 20000;

// Retained from the previously working ultrasonic code.
const int SENSOR_GAP_MS = 45;

// Obstacle is detected at 10 cm or closer.
const float OBSTACLE_DISTANCE_CM = 10.0;

// Once avoidance begins, require 13 cm clearance before
// returning to forward roaming.
const float OBSTACLE_CLEAR_DISTANCE_CM = 13.0;

// Outer sensors are scanned autonomously during roaming.
const unsigned long ROAM_SCAN_INTERVAL_MS = 20;

unsigned long lastRoamingScanTime = 0;

// Minimum right-turn time after either front straight sensor
// causes the robot to reverse.
const unsigned long STRAIGHT_TURN_DURATION_MS = 5000;

// ============================================================
// TARGET DISTANCE SETTINGS
// ============================================================

// Normal approach above 25 cm.
const float FINE_ALIGNMENT_START_CM = 25.0;

// Stop and begin pickup confirmation at 20 cm.
const float STOP_DISTANCE_CM = 20.0;

// Return to normal approach above 30 cm.
const float FINE_ALIGNMENT_EXIT_CM = 30.0;

const int FINAL_CONFIRM_REQUIRED = 3;

// ============================================================
// ADJUSTABLE MOTOR SPEEDS
//
// Valid range: 0 to 255
// ============================================================

// ------------------------------------------------------------
// CONTINUOUS ROAMING
// ------------------------------------------------------------

const int ROAM_FORWARD_SPEED = 120;

// ------------------------------------------------------------
// CONTINUOUS TARGET ALIGNMENT
// ------------------------------------------------------------

const int TARGET_TURN_LEFT_SPEED = 75;
const int TARGET_TURN_RIGHT_SPEED = 75;

const int TARGET_FINE_TURN_LEFT_SPEED = 50;
const int TARGET_FINE_TURN_RIGHT_SPEED = 50;

// ------------------------------------------------------------
// CONTINUOUS TARGET FORWARD APPROACH
// ------------------------------------------------------------

const int TARGET_FORWARD_SPEED = 95;
const int TARGET_FINE_FORWARD_SPEED = 60;

// ------------------------------------------------------------
// OBSTACLE AVOIDANCE
// ------------------------------------------------------------

// Left 45° sensor causes right turn.
const int LEFT45_AVOID_TURN_SPEED = 80;

// Right 45° sensor causes left turn.
const int RIGHT45_AVOID_TURN_SPEED = 80;

// Either straight sensor causes reverse, then right turn.
const int STRAIGHT_REVERSE_SPEED = 90;
const int STRAIGHT_TURN_RIGHT_SPEED = 150;

// ------------------------------------------------------------
// ANTI-STUCK ESCAPE
// ------------------------------------------------------------

const int ANTI_STUCK_REVERSE_SPEED = 90;
const int ANTI_STUCK_TURN_SPEED = 85;

// ------------------------------------------------------------
// PICKUP FORWARD MOVEMENT
// ------------------------------------------------------------

const int PICKUP_FORWARD_SPEED = 105;

// ============================================================
// SMOOTH-MOTION SETTINGS
// ============================================================

// Acceleration step for continuous roaming and tracking.
const int ACCELERATION_STEP = 5;

// Deceleration step before changing direction.
const int DECELERATION_STEP = 12;

// Delay between ramp steps.
const int ACCELERATION_DELAY_MS = 8;
const int DECELERATION_DELAY_MS = 5;

// Protects the motor drivers during direction reversal.
const int MOTOR_DIRECTION_DEADTIME_MS = 30;

// ============================================================
// OBSTACLE AVOIDANCE TIMING
// ============================================================

const unsigned long AVOIDANCE_INITIAL_STOP_MS = 100;

const unsigned long REVERSE_DURATION_MS = 3000;

const unsigned long DIRECTION_CHANGE_PAUSE_MS = 150;

const int MAX_AVOID_TURN_SCANS = 40;

const unsigned long ESCAPE_REVERSE_DURATION_MS = 3000;

// ============================================================
// PICKUP MOVEMENT SETTINGS
// ============================================================

const unsigned long PICKUP_FORWARD_DURATION_MS = 2500;

const unsigned long AFTER_PICKUP_FORWARD_PAUSE_MS = 500;

// ============================================================
// COMMUNICATION SETTINGS
// ============================================================

const float MIN_CONFIDENCE = 0.60;

// Long enough to prevent ordinary scanning from causing
// accidental stops.
const unsigned long COMMUNICATION_TIMEOUT_MS = 5000;

unsigned long lastCommandTime = 0;

// ============================================================
// ROBOT MODES
// ============================================================

enum RobotMode {
  MODE_STOPPED,
  MODE_ROAMING,
  MODE_TARGET_TRACKING,
  MODE_PICKUP
};

RobotMode currentRobotMode = MODE_STOPPED;

// ============================================================
// MOTOR MOTION TYPES
// ============================================================

enum MotionType {
  MOTION_STOP,
  MOTION_FORWARD,
  MOTION_BACKWARD,
  MOTION_LEFT,
  MOTION_RIGHT
};

MotionType currentDriveMotion = MOTION_STOP;

int currentDriveSpeed = 0;

// ============================================================
// ROAMING AVOIDANCE STATES
// ============================================================

enum RoamingAvoidanceState {
  ROAM_AVOID_NONE,
  ROAM_AVOID_TURN_RIGHT,
  ROAM_AVOID_TURN_LEFT
};

RoamingAvoidanceState roamingAvoidanceState =
  ROAM_AVOID_NONE;

bool reversePerformedForCurrentObstacle = false;

int avoidanceTurnScanCount = 0;

int currentAvoidanceTurnSpeed = 0;

const char* lastRoamingAction = "WAITING_FOR_ROAMING";

// ============================================================
// SERVO OBJECTS
// ============================================================

Servo shoulderServo;
Servo elbowServo;
Servo wristServo;
Servo rotateServo;
Servo gripperServo;

// ============================================================
// CONFIRMED ARM STARTING ANGLES
//
// S, E, W, R, G
// 135, 130, 120, 170, 95
// ============================================================

int currentS = 135;
int currentE = 130;
int currentW = 120;
int currentR = 170;
int currentG = 95;

// ============================================================
// SERVO MOVEMENT SETTINGS
// ============================================================

const int SERVO_STEP_DELAY = 15;
const int JOINT_PAUSE = 300;
const int STATE_PAUSE = 500;

// ============================================================
// ARM STATES
// ============================================================

enum ArmState {
  INITIAL_POSITION,
  READY_TO_GRAB_POSITION,
  FORWARD_MOVEMENT_COMPLETE,
  GRAB_POSITION,
  DISPOSE_POSITION
};

ArmState currentArmState = INITIAL_POSITION;

bool armBusy = false;

// ============================================================
// TARGET ALIGNMENT STATE
// ============================================================

bool fineAlignmentMode = false;

int finalConfirmCount = 0;

bool finalAlignmentConfirmed = false;

// ============================================================
// ULTRASONIC READINGS
// ============================================================

float left45DistanceCm = -1.0;
float leftDistanceCm = -1.0;
float rightDistanceCm = -1.0;
float right45DistanceCm = -1.0;

// ============================================================
// MOTOR PWM COMPATIBILITY
//
// Supports ESP32 Arduino core 2.x and 3.x.
// ============================================================

void attachMotorPwm(
  int pin,
  int channel
)
{
#if ESP_ARDUINO_VERSION_MAJOR >= 3

  ledcAttachChannel(
    pin,
    MOTOR_PWM_FREQUENCY,
    MOTOR_PWM_RESOLUTION,
    channel
  );

#else

  ledcSetup(
    channel,
    MOTOR_PWM_FREQUENCY,
    MOTOR_PWM_RESOLUTION
  );

  ledcAttachPin(
    pin,
    channel
  );

#endif
}

void writeMotorPwm(
  int pin,
  int channel,
  int duty
)
{
  duty = constrain(
    duty,
    0,
    255
  );

#if ESP_ARDUINO_VERSION_MAJOR >= 3

  ledcWrite(
    pin,
    duty
  );

#else

  ledcWrite(
    channel,
    duty
  );

#endif
}

// ============================================================
// APPLY MOTOR MOTION
// ============================================================

void applyMotorOutput(
  MotionType motion,
  int speedValue
)
{
  speedValue = constrain(
    speedValue,
    0,
    255
  );

  int leftRpwmDuty = 0;
  int leftLpwmDuty = 0;

  int rightRpwmDuty = 0;
  int rightLpwmDuty = 0;

  switch (motion) {
    case MOTION_FORWARD:
      leftLpwmDuty = speedValue;
      rightRpwmDuty = speedValue;
      break;

    case MOTION_BACKWARD:
      leftRpwmDuty = speedValue;
      rightLpwmDuty = speedValue;
      break;

    case MOTION_LEFT:
      leftRpwmDuty = speedValue;
      rightRpwmDuty = speedValue;
      break;

    case MOTION_RIGHT:
      leftLpwmDuty = speedValue;
      rightLpwmDuty = speedValue;
      break;

    case MOTION_STOP:
    default:
      break;
  }

  writeMotorPwm(
    LEFT_RPWM,
    LEFT_RPWM_CHANNEL,
    leftRpwmDuty
  );

  writeMotorPwm(
    LEFT_LPWM,
    LEFT_LPWM_CHANNEL,
    leftLpwmDuty
  );

  writeMotorPwm(
    RIGHT_RPWM,
    RIGHT_RPWM_CHANNEL,
    rightRpwmDuty
  );

  writeMotorPwm(
    RIGHT_LPWM,
    RIGHT_LPWM_CHANNEL,
    rightLpwmDuty
  );
}

// ============================================================
// IMMEDIATE MOTOR STOP
// ============================================================

void stopDriveMotorsImmediate()
{
  applyMotorOutput(
    MOTION_STOP,
    0
  );

  currentDriveMotion = MOTION_STOP;
  currentDriveSpeed = 0;
}

// ============================================================
// RAMP CURRENT SPEED
// ============================================================

void rampDriveSpeed(
  int targetSpeed,
  int stepSize,
  int stepDelayMs
)
{
  targetSpeed = constrain(
    targetSpeed,
    0,
    255
  );

  stepSize = max(
    stepSize,
    1
  );

  while (
    currentDriveSpeed != targetSpeed
  ) {
    if (
      currentDriveSpeed <
      targetSpeed
    ) {
      currentDriveSpeed += stepSize;

      if (
        currentDriveSpeed >
        targetSpeed
      ) {
        currentDriveSpeed =
          targetSpeed;
      }
    }
    else {
      currentDriveSpeed -= stepSize;

      if (
        currentDriveSpeed <
        targetSpeed
      ) {
        currentDriveSpeed =
          targetSpeed;
      }
    }

    applyMotorOutput(
      currentDriveMotion,
      currentDriveSpeed
    );

    delay(stepDelayMs);
  }
}

// ============================================================
// SMOOTH CONTINUOUS MOVEMENT
//
// Repeated commands with the same motion do not stop or
// restart the motors.
// ============================================================

void setDriveMotionSmooth(
  MotionType newMotion,
  int newSpeed
)
{
  newSpeed = constrain(
    newSpeed,
    0,
    255
  );

  if (
    newMotion == MOTION_STOP ||
    newSpeed == 0
  ) {
    stopDriveMotorsImmediate();
    return;
  }

  // Same direction: adjust only the speed.
  if (
    currentDriveMotion ==
    newMotion
  ) {
    if (
      currentDriveSpeed ==
      newSpeed
    ) {
      return;
    }

    if (
      currentDriveSpeed <
      newSpeed
    ) {
      rampDriveSpeed(
        newSpeed,
        ACCELERATION_STEP,
        ACCELERATION_DELAY_MS
      );
    }
    else {
      rampDriveSpeed(
        newSpeed,
        DECELERATION_STEP,
        DECELERATION_DELAY_MS
      );
    }

    return;
  }

  // Direction is changing.
  if (
    currentDriveSpeed > 0 &&
    currentDriveMotion != MOTION_STOP
  ) {
    rampDriveSpeed(
      0,
      DECELERATION_STEP,
      DECELERATION_DELAY_MS
    );
  }

  applyMotorOutput(
    MOTION_STOP,
    0
  );

  currentDriveMotion = MOTION_STOP;
  currentDriveSpeed = 0;

  delay(
    MOTOR_DIRECTION_DEADTIME_MS
  );

  currentDriveMotion = newMotion;
  currentDriveSpeed = 0;

  applyMotorOutput(
    currentDriveMotion,
    currentDriveSpeed
  );

  rampDriveSpeed(
    newSpeed,
    ACCELERATION_STEP,
    ACCELERATION_DELAY_MS
  );
}

// ============================================================
// IMMEDIATE TIMED MOTOR MOVEMENT
//
// Used only for the known pickup forward movement.
// ============================================================

void setDriveMotionImmediate(
  MotionType motion,
  int speedValue
)
{
  speedValue = constrain(
    speedValue,
    0,
    255
  );

  currentDriveMotion = motion;
  currentDriveSpeed = speedValue;

  applyMotorOutput(
    currentDriveMotion,
    currentDriveSpeed
  );
}

// ============================================================
// READ ONE ULTRASONIC SENSOR
// ============================================================

float readDistanceCm(
  int trigPin,
  int echoPin
)
{
  digitalWrite(
    trigPin,
    LOW
  );

  delayMicroseconds(3);

  digitalWrite(
    trigPin,
    HIGH
  );

  delayMicroseconds(10);

  digitalWrite(
    trigPin,
    LOW
  );

  unsigned long echoDuration =
    pulseIn(
      echoPin,
      HIGH,
      ECHO_TIMEOUT_US
    );

  if (
    echoDuration == 0
  ) {
    return -1.0;
  }

  return (
    echoDuration *
    0.0343f /
    2.0f
  );
}

// ============================================================
// SCAN OUTER SENSORS
//
// Straight sensors are scanned first.
//
// This function operates only during roaming.
// ============================================================

void scanOuterSensors()
{
  if (
    currentRobotMode !=
    MODE_ROAMING
  ) {
    return;
  }

  // Left straight first.
  leftDistanceCm =
    readDistanceCm(
      LEFT_TRIG_PIN,
      LEFT_ECHO_PIN
    );

  delay(SENSOR_GAP_MS);

  // Right straight second.
  rightDistanceCm =
    readDistanceCm(
      RIGHT_TRIG_PIN,
      RIGHT_ECHO_PIN
    );

  delay(SENSOR_GAP_MS);

  // Left angled sensor.
  left45DistanceCm =
    readDistanceCm(
      LEFT_45_TRIG_PIN,
      LEFT_45_ECHO_PIN
    );

  delay(SENSOR_GAP_MS);

  // Right angled sensor.
  right45DistanceCm =
    readDistanceCm(
      RIGHT_45_TRIG_PIN,
      RIGHT_45_ECHO_PIN
    );
}

// ============================================================
// CENTRE SENSOR
//
// Used only during target tracking.
// ============================================================

float readCenterDistanceCm()
{
  return readDistanceCm(
    CENTER_TRIG_PIN,
    CENTER_ECHO_PIN
  );
}

// ============================================================
// SENSOR CHECK FUNCTIONS
// ============================================================

bool validDistance(
  float distanceCm
)
{
  return distanceCm > 0.0;
}

bool sensorAtOrBelow(
  float distanceCm,
  float thresholdCm
)
{
  return (
    validDistance(distanceCm) &&
    distanceCm <= thresholdCm
  );
}

bool left45ObstacleDetected()
{
  return (
    currentRobotMode ==
    MODE_ROAMING
    &&
    sensorAtOrBelow(
      left45DistanceCm,
      OBSTACLE_DISTANCE_CM
    )
  );
}

bool leftStraightObstacleDetected()
{
  return (
    currentRobotMode ==
    MODE_ROAMING
    &&
    sensorAtOrBelow(
      leftDistanceCm,
      OBSTACLE_DISTANCE_CM
    )
  );
}

bool rightStraightObstacleDetected()
{
  return (
    currentRobotMode ==
    MODE_ROAMING
    &&
    sensorAtOrBelow(
      rightDistanceCm,
      OBSTACLE_DISTANCE_CM
    )
  );
}

bool right45ObstacleDetected()
{
  return (
    currentRobotMode ==
    MODE_ROAMING
    &&
    sensorAtOrBelow(
      right45DistanceCm,
      OBSTACLE_DISTANCE_CM
    )
  );
}

bool anyOuterObstacleAtDistance(
  float thresholdCm
)
{
  if (
    currentRobotMode !=
    MODE_ROAMING
  ) {
    return false;
  }

  return (
    sensorAtOrBelow(
      left45DistanceCm,
      thresholdCm
    )
    ||
    sensorAtOrBelow(
      leftDistanceCm,
      thresholdCm
    )
    ||
    sensorAtOrBelow(
      rightDistanceCm,
      thresholdCm
    )
    ||
    sensorAtOrBelow(
      right45DistanceCm,
      thresholdCm
    )
  );
}

bool allOuterSensorsClear()
{
  return !anyOuterObstacleAtDistance(
    OBSTACLE_CLEAR_DISTANCE_CM
  );
}

// ============================================================
// PRINT HELPERS
// ============================================================

void printDistanceValue(
  float distanceCm
)
{
  if (
    distanceCm < 0.0
  ) {
    Serial.print(
      "NO_ECHO"
    );
  }
  else {
    Serial.print(
      distanceCm,
      1
    );
  }
}

void printRobotModeName()
{
  switch (
    currentRobotMode
  ) {
    case MODE_STOPPED:
      Serial.print("STOPPED");
      break;

    case MODE_ROAMING:
      Serial.print("ROAMING");
      break;

    case MODE_TARGET_TRACKING:
      Serial.print(
        "TARGET_TRACKING"
      );
      break;

    case MODE_PICKUP:
      Serial.print("PICKUP");
      break;
  }
}

void printRoamingAvoidanceState()
{
  switch (
    roamingAvoidanceState
  ) {
    case ROAM_AVOID_NONE:
      Serial.print("NONE");
      break;

    case ROAM_AVOID_TURN_RIGHT:
      Serial.print("TURN_RIGHT");
      break;

    case ROAM_AVOID_TURN_LEFT:
      Serial.print("TURN_LEFT");
      break;
  }
}

void printRoamingResponse()
{
  Serial.print("ACK:MODE=");
  printRobotModeName();

  Serial.print(":LEFT45_CM=");
  printDistanceValue(
    left45DistanceCm
  );

  Serial.print(":LEFT_CM=");
  printDistanceValue(
    leftDistanceCm
  );

  Serial.print(":RIGHT_CM=");
  printDistanceValue(
    rightDistanceCm
  );

  Serial.print(":RIGHT45_CM=");
  printDistanceValue(
    right45DistanceCm
  );

  Serial.print(":AVOID_STATE=");
  printRoamingAvoidanceState();

  Serial.print(":SCAN_COUNT=");
  Serial.print(
    avoidanceTurnScanCount
  );

  Serial.print(":MOTION_SPEED=");
  Serial.print(
    currentDriveSpeed
  );

  Serial.print(":ACTION=");
  Serial.println(
    lastRoamingAction
  );
}

// ============================================================
// RESET FUNCTIONS
// ============================================================

void resetFinalConfirmation()
{
  finalConfirmCount = 0;

  finalAlignmentConfirmed =
    false;
}

void resetRoamingAvoidance()
{
  roamingAvoidanceState =
    ROAM_AVOID_NONE;

  reversePerformedForCurrentObstacle =
    false;

  avoidanceTurnScanCount = 0;

  currentAvoidanceTurnSpeed = 0;
}

// ============================================================
// SERVO SETUP
// ============================================================

void attachServoSafely(
  Servo& servo,
  int pin
)
{
  servo.setPeriodHertz(50);

  servo.attach(
    pin,
    500,
    2500
  );
}

// ============================================================
// MOVE ONE SERVO AT A TIME
// ============================================================

void moveServoSlowly(
  Servo& servo,
  int& currentAngle,
  int targetAngle
)
{
  targetAngle = constrain(
    targetAngle,
    0,
    180
  );

  if (
    targetAngle >
    currentAngle
  ) {
    for (
      int angle = currentAngle;
      angle <= targetAngle;
      angle++
    ) {
      servo.write(angle);

      delay(
        SERVO_STEP_DELAY
      );
    }
  }
  else if (
    targetAngle <
    currentAngle
  ) {
    for (
      int angle = currentAngle;
      angle >= targetAngle;
      angle--
    ) {
      servo.write(angle);

      delay(
        SERVO_STEP_DELAY
      );
    }
  }

  currentAngle =
    targetAngle;

  delay(
    JOINT_PAUSE
  );
}

// ============================================================
// PRINT ARM ANGLES
// ============================================================

void printCurrentAngles()
{
  Serial.println(
    "Current arm angles:"
  );

  Serial.print("S = ");
  Serial.println(currentS);

  Serial.print("E = ");
  Serial.println(currentE);

  Serial.print("W = ");
  Serial.println(currentW);

  Serial.print("R = ");
  Serial.println(currentR);

  Serial.print("G = ");
  Serial.println(currentG);
}

// ============================================================
// ARM: INITIAL TO READY
//
// Exact confirmed sequence:
//
// 1. R -> 170
// 2. S -> 55
// 3. E -> 170
// ============================================================

void moveToReadyToGrab()
{
  stopDriveMotorsImmediate();

  Serial.println(
    "ARM: MOVING TO READY TO GRAB"
  );

  moveServoSlowly(
    rotateServo,
    currentR,
    170
  );

  moveServoSlowly(
    shoulderServo,
    currentS,
    55
  );

  moveServoSlowly(
    elbowServo,
    currentE,
    170
  );

  currentArmState =
    READY_TO_GRAB_POSITION;

  Serial.println(
    "ARM: READY TO GRAB REACHED"
  );

  printCurrentAngles();
}

// ============================================================
// PICKUP FORWARD MOVEMENT
//
// Exact confirmed location in the sequence:
//
// Ready -> forward 1 second -> grab
// ============================================================

void moveForwardForPickup()
{
  stopDriveMotorsImmediate();

  Serial.println(
    "ROBOT: PICKUP FORWARD START"
  );

  Serial.print("Duration: ");
  Serial.print(
    PICKUP_FORWARD_DURATION_MS
  );
  Serial.println(" ms");

  Serial.print("Speed: ");
  Serial.println(
    PICKUP_FORWARD_SPEED
  );

  setDriveMotionImmediate(
    MOTION_FORWARD,
    PICKUP_FORWARD_SPEED
  );

  delay(
    PICKUP_FORWARD_DURATION_MS
  );

  stopDriveMotorsImmediate();

  currentArmState =
    FORWARD_MOVEMENT_COMPLETE;

  Serial.println(
    "ROBOT: PICKUP FORWARD COMPLETE"
  );

  delay(
    AFTER_PICKUP_FORWARD_PAUSE_MS
  );
}

// ============================================================
// ARM: READY TO GRAB
//
// G -> 30
// ============================================================

void moveToGrab()
{
  stopDriveMotorsImmediate();

  Serial.println(
    "ARM: MOVING TO GRAB"
  );

  moveServoSlowly(
    gripperServo,
    currentG,
    30
  );

  currentArmState =
    GRAB_POSITION;

  Serial.println(
    "ARM: GRAB REACHED"
  );

  printCurrentAngles();
}

// ============================================================
// ARM: GRAB TO DISPOSE
//
// Exact confirmed sequence:
//
// 1. S -> 110
// 2. R -> 140
// 3. E -> 50
// 4. G -> 95
// ============================================================

void moveToDispose()
{
  stopDriveMotorsImmediate();

  Serial.println(
    "ARM: MOVING TO DISPOSE"
  );

  moveServoSlowly(
    shoulderServo,
    currentS,
    110
  );

  moveServoSlowly(
    rotateServo,
    currentR,
    140
  );

  moveServoSlowly(
    elbowServo,
    currentE,
    50
  );

  moveServoSlowly(
    gripperServo,
    currentG,
    95
  );

  currentArmState =
    DISPOSE_POSITION;

  Serial.println(
    "ARM: DISPOSE REACHED"
  );

  printCurrentAngles();
}

// ============================================================
// ARM: DISPOSE TO INITIAL
//
// Exact confirmed sequence:
//
// 1. E -> 130
// 2. S -> 135
//
// R remains at 140 after the first cycle.
// ============================================================

void returnToInitial()
{
  stopDriveMotorsImmediate();

  Serial.println(
    "ARM: RETURNING TO INITIAL"
  );

  moveServoSlowly(
    elbowServo,
    currentE,
    130
  );

  moveServoSlowly(
    shoulderServo,
    currentS,
    135
  );

  currentArmState =
    INITIAL_POSITION;

  Serial.println(
    "ARM: INITIAL REACHED"
  );

  printCurrentAngles();
}

// ============================================================
// COMPLETE AUTOMATIC PICKUP
// ============================================================

void runAutomaticPickup()
{
  if (armBusy) {
    return;
  }

  if (
    currentArmState !=
    INITIAL_POSITION
  ) {
    Serial.println(
      "ERROR:ARM_NOT_INITIAL"
    );

    return;
  }

  armBusy = true;

  currentRobotMode =
    MODE_PICKUP;

  resetRoamingAvoidance();

  stopDriveMotorsImmediate();

  // Must be the first response read by Python.
  Serial.println(
    "PICKUP_START"
  );

  Serial.flush();

  delay(200);

  moveToReadyToGrab();

  delay(
    STATE_PAUSE
  );

  moveForwardForPickup();

  delay(
    STATE_PAUSE
  );

  moveToGrab();

  delay(
    STATE_PAUSE
  );

  moveToDispose();

  delay(
    STATE_PAUSE
  );

  returnToInitial();

  stopDriveMotorsImmediate();

  fineAlignmentMode =
    false;

  resetFinalConfirmation();
  resetRoamingAvoidance();

  lastCommandTime =
    millis();

  armBusy = false;

  // Remain stopped until Python finishes the requested
  // two-second post-pickup delay and sends NONE.
  currentRobotMode =
    MODE_STOPPED;

  Serial.println(
    "PICKUP_DONE"
  );

  Serial.flush();
}

// ============================================================
// TARGET RESPONSE FUNCTIONS
// ============================================================

void printTurnResponse(
  const String& objectName,
  const String& direction,
  const String& action
)
{
  Serial.print("ACK:MODE=");
  printRobotModeName();

  Serial.print(":OBJECT=");
  Serial.print(objectName);

  Serial.print(":DIRECTION=");
  Serial.print(direction);

  Serial.print(
    ":CENTER_CM=SKIPPED"
  );

  Serial.print(
    ":OUTER_SENSORS=SKIPPED"
  );

  Serial.print(":FINE=");

  if (
    fineAlignmentMode
  ) {
    Serial.print("YES");
  }
  else {
    Serial.print("NO");
  }

  Serial.print(":CONFIRM=");
  Serial.print(
    finalConfirmCount
  );

  Serial.print("/");

  Serial.print(
    FINAL_CONFIRM_REQUIRED
  );

  Serial.print(":MOTION_SPEED=");
  Serial.print(
    currentDriveSpeed
  );

  Serial.print(":ACTION=");
  Serial.println(action);
}

void printCenterResponse(
  const String& objectName,
  float centerDistanceCm,
  const String& action
)
{
  Serial.print("ACK:MODE=");
  printRobotModeName();

  Serial.print(":OBJECT=");
  Serial.print(objectName);

  Serial.print(
    ":DIRECTION=CENTER"
  );

  Serial.print(
    ":CENTER_CM="
  );

  printDistanceValue(
    centerDistanceCm
  );

  Serial.print(
    ":OUTER_SENSORS=SKIPPED"
  );

  Serial.print(":FINE=");

  if (
    fineAlignmentMode
  ) {
    Serial.print("YES");
  }
  else {
    Serial.print("NO");
  }

  Serial.print(":CONFIRM=");
  Serial.print(
    finalConfirmCount
  );

  Serial.print("/");

  Serial.print(
    FINAL_CONFIRM_REQUIRED
  );

  Serial.print(":MOTION_SPEED=");
  Serial.print(
    currentDriveSpeed
  );

  Serial.print(":ACTION=");
  Serial.println(action);
}

// ============================================================
// PARSE TARGET MESSAGE
//
// Format:
//
// TARGET,OBJECT,DIRECTION,CENTER_X,FRAME_WIDTH,CONFIDENCE
// ============================================================

bool parseTargetMessage(
  const String& message,
  String& objectName,
  String& direction,
  float& confidence
)
{
  int comma1 =
    message.indexOf(',');

  int comma2 =
    message.indexOf(
      ',',
      comma1 + 1
    );

  int comma3 =
    message.indexOf(
      ',',
      comma2 + 1
    );

  int comma4 =
    message.indexOf(
      ',',
      comma3 + 1
    );

  int comma5 =
    message.indexOf(
      ',',
      comma4 + 1
    );

  if (
    comma1 < 0 ||
    comma2 < 0 ||
    comma3 < 0 ||
    comma4 < 0 ||
    comma5 < 0
  ) {
    return false;
  }

  objectName =
    message.substring(
      comma1 + 1,
      comma2
    );

  direction =
    message.substring(
      comma2 + 1,
      comma3
    );

  confidence =
    message.substring(
      comma5 + 1
    ).toFloat();

  objectName.trim();
  objectName.toUpperCase();

  direction.trim();
  direction.toUpperCase();

  if (
    objectName != "CAN" &&
    objectName != "BOTTLE"
  ) {
    return false;
  }

  if (
    direction != "LEFT" &&
    direction != "CENTER" &&
    direction != "RIGHT"
  ) {
    return false;
  }

  if (
    confidence < 0.0 ||
    confidence > 1.0
  ) {
    return false;
  }

  return true;
}

// ============================================================
// PROCESS AI TARGET
//
// Continuous:
// - Target alignment
// - Target forward approach
//
// Outer sensors are disabled here.
// ============================================================

void processAITarget(
  const String& objectName,
  const String& direction,
  float confidence
)
{
  currentRobotMode =
    MODE_TARGET_TRACKING;

  resetRoamingAvoidance();

  lastCommandTime =
    millis();

  if (armBusy) {
    stopDriveMotorsImmediate();

    Serial.println(
      "ACK:MODE=PICKUP:ACTION=ARM_BUSY"
    );

    return;
  }

  if (
    currentArmState !=
    INITIAL_POSITION
  ) {
    stopDriveMotorsImmediate();

    Serial.println(
      "ACK:MODE=TARGET_TRACKING:ACTION=ARM_NOT_INITIAL"
    );

    return;
  }

  if (
    confidence <
    MIN_CONFIDENCE
  ) {
    stopDriveMotorsImmediate();

    fineAlignmentMode =
      false;

    resetFinalConfirmation();

    Serial.println(
      "ACK:MODE=TARGET_TRACKING:ACTION=LOW_CONFIDENCE"
    );

    return;
  }

  // ----------------------------------------------------------
  // TARGET LEFT
  // ----------------------------------------------------------

  if (
    direction == "LEFT"
  ) {
    resetFinalConfirmation();

    if (
      fineAlignmentMode
    ) {
      setDriveMotionSmooth(
        MOTION_LEFT,
        TARGET_FINE_TURN_LEFT_SPEED
      );

      printTurnResponse(
        objectName,
        direction,
        "CONTINUOUS_FINE_TURN_LEFT"
      );
    }
    else {
      setDriveMotionSmooth(
        MOTION_LEFT,
        TARGET_TURN_LEFT_SPEED
      );

      printTurnResponse(
        objectName,
        direction,
        "CONTINUOUS_TURN_LEFT"
      );
    }

    return;
  }

  // ----------------------------------------------------------
  // TARGET RIGHT
  // ----------------------------------------------------------

  if (
    direction == "RIGHT"
  ) {
    resetFinalConfirmation();

    if (
      fineAlignmentMode
    ) {
      setDriveMotionSmooth(
        MOTION_RIGHT,
        TARGET_FINE_TURN_RIGHT_SPEED
      );

      printTurnResponse(
        objectName,
        direction,
        "CONTINUOUS_FINE_TURN_RIGHT"
      );
    }
    else {
      setDriveMotionSmooth(
        MOTION_RIGHT,
        TARGET_TURN_RIGHT_SPEED
      );

      printTurnResponse(
        objectName,
        direction,
        "CONTINUOUS_TURN_RIGHT"
      );
    }

    return;
  }

  // ----------------------------------------------------------
  // TARGET CENTER
  // ----------------------------------------------------------

  if (
    direction == "CENTER"
  ) {
    float centerDistanceCm =
      readCenterDistanceCm();

    if (
      centerDistanceCm < 0.0
    ) {
      stopDriveMotorsImmediate();

      resetFinalConfirmation();

      printCenterResponse(
        objectName,
        centerDistanceCm,
        "STOP_NO_ECHO"
      );

      return;
    }

    if (
      centerDistanceCm >
      FINE_ALIGNMENT_EXIT_CM
    ) {
      fineAlignmentMode =
        false;
    }

    if (
      centerDistanceCm <=
      FINE_ALIGNMENT_START_CM
    ) {
      fineAlignmentMode =
        true;
    }

    // --------------------------------------------------------
    // FINAL PICKUP POSITION
    // --------------------------------------------------------

    if (
      centerDistanceCm <=
      STOP_DISTANCE_CM
    ) {
      stopDriveMotorsImmediate();

      finalConfirmCount++;

      if (
        finalConfirmCount >=
        FINAL_CONFIRM_REQUIRED
      ) {
        finalConfirmCount =
          FINAL_CONFIRM_REQUIRED;

        finalAlignmentConfirmed =
          true;

        runAutomaticPickup();
      }
      else {
        printCenterResponse(
          objectName,
          centerDistanceCm,
          "CONFIRMING_FINAL_POSITION"
        );
      }

      return;
    }

    resetFinalConfirmation();

    // --------------------------------------------------------
    // FINE CONTINUOUS APPROACH
    // --------------------------------------------------------

    if (
      fineAlignmentMode
    ) {
      setDriveMotionSmooth(
        MOTION_FORWARD,
        TARGET_FINE_FORWARD_SPEED
      );

      printCenterResponse(
        objectName,
        centerDistanceCm,
        "CONTINUOUS_FINE_FORWARD"
      );

      return;
    }

    // --------------------------------------------------------
    // NORMAL CONTINUOUS APPROACH
    // --------------------------------------------------------

    setDriveMotionSmooth(
      MOTION_FORWARD,
      TARGET_FORWARD_SPEED
    );

    printCenterResponse(
      objectName,
      centerDistanceCm,
      "CONTINUOUS_FORWARD"
    );

    return;
  }

  stopDriveMotorsImmediate();

  resetFinalConfirmation();

  Serial.println(
    "ACK:MODE=TARGET_TRACKING:ACTION=STOPPED"
  );
}

// ============================================================
// STRAIGHT-SENSOR AVOIDANCE
//
// Stop -> reverse -> stop -> turn right continuously.
// ============================================================

void startStraightSensorAvoidance()
{
  stopDriveMotorsImmediate();

  delay(
    AVOIDANCE_INITIAL_STOP_MS
  );

  if (
    !reversePerformedForCurrentObstacle
  ) {
    setDriveMotionSmooth(
      MOTION_BACKWARD,
      STRAIGHT_REVERSE_SPEED
    );

    delay(
      REVERSE_DURATION_MS
    );

    stopDriveMotorsImmediate();

    delay(
      DIRECTION_CHANGE_PAUSE_MS
    );

    reversePerformedForCurrentObstacle =
      true;
  }

  roamingAvoidanceState =
    ROAM_AVOID_TURN_RIGHT;

  currentAvoidanceTurnSpeed =
    STRAIGHT_TURN_RIGHT_SPEED;

  avoidanceTurnScanCount = 1;

  setDriveMotionSmooth(
    MOTION_RIGHT,
    currentAvoidanceTurnSpeed
  );

  // Keep turning right for this minimum amount of time.
  delay(STRAIGHT_TURN_DURATION_MS);

  lastRoamingAction =
    "STRAIGHT_REVERSE_START_TURN_RIGHT";
}

// ============================================================
// CONTINUE AVOIDANCE TURN
// ============================================================

void continueRoamingAvoidanceTurn()
{
  if (
    roamingAvoidanceState ==
    ROAM_AVOID_TURN_RIGHT
  ) {
    setDriveMotionSmooth(
      MOTION_RIGHT,
      currentAvoidanceTurnSpeed
    );

    lastRoamingAction =
      "CONTINUE_TURN_RIGHT_UNTIL_CLEAR";
  }
  else if (
    roamingAvoidanceState ==
    ROAM_AVOID_TURN_LEFT
  ) {
    setDriveMotionSmooth(
      MOTION_LEFT,
      currentAvoidanceTurnSpeed
    );

    lastRoamingAction =
      "CONTINUE_TURN_LEFT_UNTIL_CLEAR";
  }

  avoidanceTurnScanCount++;
}

// ============================================================
// ANTI-STUCK ESCAPE
//
// Stop -> longer reverse -> stop -> switch turn direction.
// ============================================================

void performAntiStuckEscape()
{
  stopDriveMotorsImmediate();

  delay(
    AVOIDANCE_INITIAL_STOP_MS
  );

  setDriveMotionSmooth(
    MOTION_BACKWARD,
    ANTI_STUCK_REVERSE_SPEED
  );

  delay(
    ESCAPE_REVERSE_DURATION_MS
  );

  stopDriveMotorsImmediate();

  delay(
    DIRECTION_CHANGE_PAUSE_MS
  );

  if (
    roamingAvoidanceState ==
    ROAM_AVOID_TURN_RIGHT
  ) {
    roamingAvoidanceState =
      ROAM_AVOID_TURN_LEFT;
  }
  else {
    roamingAvoidanceState =
      ROAM_AVOID_TURN_RIGHT;
  }

  currentAvoidanceTurnSpeed =
    ANTI_STUCK_TURN_SPEED;

  avoidanceTurnScanCount = 1;

  reversePerformedForCurrentObstacle =
    true;

  if (
    roamingAvoidanceState ==
    ROAM_AVOID_TURN_RIGHT
  ) {
    setDriveMotionSmooth(
      MOTION_RIGHT,
      currentAvoidanceTurnSpeed
    );
  }
  else {
    setDriveMotionSmooth(
      MOTION_LEFT,
      currentAvoidanceTurnSpeed
    );
  }

  lastRoamingAction =
    "ANTI_STUCK_REVERSE_SWITCH_DIRECTION";
}

// ============================================================
// AUTONOMOUS ROAMING UPDATE
//
// This runs directly on the ESP32 while MODE_ROAMING is active.
//
// Python does not need to trigger every individual sensor scan.
// ============================================================

void updateRoamingAutonomy()
{
  if (
    currentRobotMode !=
    MODE_ROAMING
  ) {
    return;
  }

  scanOuterSensors();

  bool left45Blocked =
    left45ObstacleDetected();

  bool leftStraightBlocked =
    leftStraightObstacleDetected();

  bool rightStraightBlocked =
    rightStraightObstacleDetected();

  bool right45Blocked =
    right45ObstacleDetected();

  bool straightSensorBlocked =
    leftStraightBlocked ||
    rightStraightBlocked;

  // ----------------------------------------------------------
  // CURRENTLY AVOIDING
  // ----------------------------------------------------------

  if (
    roamingAvoidanceState !=
    ROAM_AVOID_NONE
  ) {
    // Straight sensor appears during angled avoidance.
    if (
      straightSensorBlocked &&
      !reversePerformedForCurrentObstacle
    ) {
      startStraightSensorAvoidance();
      return;
    }

    if (
      !allOuterSensorsClear()
    ) {
      if (
        avoidanceTurnScanCount >=
        MAX_AVOID_TURN_SCANS
      ) {
        performAntiStuckEscape();
        return;
      }

      continueRoamingAvoidanceTurn();
      return;
    }

    // All outer sensors are clear.
    resetRoamingAvoidance();

    setDriveMotionSmooth(
      MOTION_FORWARD,
      ROAM_FORWARD_SPEED
    );

    lastRoamingAction =
      "PATH_CLEAR_CONTINUOUS_FORWARD";

    return;
  }

  // ----------------------------------------------------------
  // STRAIGHT SENSOR PRIORITY
  // ----------------------------------------------------------

  if (
    straightSensorBlocked
  ) {
    startStraightSensorAvoidance();
    return;
  }

  // ----------------------------------------------------------
  // BOTH ANGLED SENSORS
  // ----------------------------------------------------------

  if (
    left45Blocked &&
    right45Blocked
  ) {
    startStraightSensorAvoidance();

    lastRoamingAction =
      "BOTH_45_REVERSE_START_TURN_RIGHT";

    return;
  }

  // ----------------------------------------------------------
  // LEFT 45° SENSOR
  // ----------------------------------------------------------

  if (
    left45Blocked
  ) {
    stopDriveMotorsImmediate();

    delay(
      AVOIDANCE_INITIAL_STOP_MS
    );

    roamingAvoidanceState =
      ROAM_AVOID_TURN_RIGHT;

    reversePerformedForCurrentObstacle =
      false;

    avoidanceTurnScanCount = 1;

    currentAvoidanceTurnSpeed =
      LEFT45_AVOID_TURN_SPEED;

    setDriveMotionSmooth(
      MOTION_RIGHT,
      currentAvoidanceTurnSpeed
    );

    lastRoamingAction =
      "LEFT45_START_TURN_RIGHT";

    return;
  }

  // ----------------------------------------------------------
  // RIGHT 45° SENSOR
  // ----------------------------------------------------------

  if (
    right45Blocked
  ) {
    stopDriveMotorsImmediate();

    delay(
      AVOIDANCE_INITIAL_STOP_MS
    );

    roamingAvoidanceState =
      ROAM_AVOID_TURN_LEFT;

    reversePerformedForCurrentObstacle =
      false;

    avoidanceTurnScanCount = 1;

    currentAvoidanceTurnSpeed =
      RIGHT45_AVOID_TURN_SPEED;

    setDriveMotionSmooth(
      MOTION_LEFT,
      currentAvoidanceTurnSpeed
    );

    lastRoamingAction =
      "RIGHT45_START_TURN_LEFT";

    return;
  }

  // ----------------------------------------------------------
  // CLEAR PATH
  //
  // Repeated scans do not stop or restart the motor PWM.
  // ----------------------------------------------------------

  resetRoamingAvoidance();

  setDriveMotionSmooth(
    MOTION_FORWARD,
    ROAM_FORWARD_SPEED
  );

  lastRoamingAction =
    "CONTINUOUS_ROAM_FORWARD";
}

// ============================================================
// PROCESS SERIAL COMMAND
// ============================================================

void processCommand(
  String command
)
{
  command.trim();
  command.toUpperCase();

  if (
    command.length() == 0
  ) {
    return;
  }

  // ----------------------------------------------------------
  // NONE = ENTER OR REMAIN IN ROAMING
  // ----------------------------------------------------------

  if (
    command == "NONE"
  ) {
    bool enteringRoaming =
      currentRobotMode !=
      MODE_ROAMING;

    currentRobotMode =
      MODE_ROAMING;

    lastCommandTime =
      millis();

    fineAlignmentMode =
      false;

    resetFinalConfirmation();

    if (
      enteringRoaming
    ) {
      resetRoamingAvoidance();

      lastRoamingScanTime = 0;

      lastRoamingAction =
        "ENTERING_ROAMING";
    }

    printRoamingResponse();

    return;
  }

  // ----------------------------------------------------------
  // TARGET = CAMERA TARGET TRACKING
  // ----------------------------------------------------------

  if (
    command.startsWith(
      "TARGET,"
    )
  ) {
    String objectName;
    String direction;

    float confidence = 0.0;

    bool validMessage =
      parseTargetMessage(
        command,
        objectName,
        direction,
        confidence
      );

    if (
      !validMessage
    ) {
      stopDriveMotorsImmediate();

      currentRobotMode =
        MODE_STOPPED;

      resetRoamingAvoidance();

      fineAlignmentMode =
        false;

      resetFinalConfirmation();

      Serial.println(
        "ERROR:INVALID_TARGET"
      );

      return;
    }

    processAITarget(
      objectName,
      direction,
      confidence
    );

    return;
  }

  // ----------------------------------------------------------
  // MANUAL PICKUP TEST
  // ----------------------------------------------------------

  if (
    command == "PICKUP"
  ) {
    lastCommandTime =
      millis();

    runAutomaticPickup();

    return;
  }

  // ----------------------------------------------------------
  // MANUAL OUTER SENSOR SCAN
  // ----------------------------------------------------------

  if (
    command == "SCAN_OUTER"
  ) {
    lastCommandTime =
      millis();

    stopDriveMotorsImmediate();

    RobotMode previousMode =
      currentRobotMode;

    currentRobotMode =
      MODE_ROAMING;

    scanOuterSensors();

    printRoamingResponse();

    currentRobotMode =
      previousMode;

    return;
  }

  // ----------------------------------------------------------
  // MANUAL CENTRE SENSOR SCAN
  // ----------------------------------------------------------

  if (
    command == "SCAN_CENTER"
  ) {
    lastCommandTime =
      millis();

    stopDriveMotorsImmediate();

    float distanceCm =
      readCenterDistanceCm();

    Serial.print(
      "CENTER_CM="
    );

    printDistanceValue(
      distanceCm
    );

    Serial.println();

    return;
  }

  // ----------------------------------------------------------
  // PRINT ARM POSITION
  // ----------------------------------------------------------

  if (
    command == "POSITION" ||
    command == "P"
  ) {
    lastCommandTime =
      millis();

    stopDriveMotorsImmediate();

    printCurrentAngles();

    return;
  }

  // ----------------------------------------------------------
  // PRINT ROBOT MODE
  // ----------------------------------------------------------

  if (
    command == "MODE"
  ) {
    lastCommandTime =
      millis();

    Serial.print(
      "MODE="
    );

    printRobotModeName();

    Serial.println();

    return;
  }

  // ----------------------------------------------------------
  // STOP
  //
  // Stops roaming autonomy as well as the motors.
  // ----------------------------------------------------------

  if (
    command == "STOP" ||
    command == "S"
  ) {
    lastCommandTime =
      millis();

    currentRobotMode =
      MODE_STOPPED;

    stopDriveMotorsImmediate();

    resetRoamingAvoidance();

    fineAlignmentMode =
      false;

    resetFinalConfirmation();

    Serial.println(
      "DRIVE_MOTORS_STOPPED"
    );

    return;
  }

  Serial.println(
    "ERROR:UNKNOWN_COMMAND"
  );
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  Serial.setTimeout(100);

  // ----------------------------------------------------------
  // MOTOR PINS START LOW
  // ----------------------------------------------------------

  pinMode(
    LEFT_RPWM,
    OUTPUT
  );

  pinMode(
    LEFT_LPWM,
    OUTPUT
  );

  pinMode(
    RIGHT_RPWM,
    OUTPUT
  );

  pinMode(
    RIGHT_LPWM,
    OUTPUT
  );

  digitalWrite(
    LEFT_RPWM,
    LOW
  );

  digitalWrite(
    LEFT_LPWM,
    LOW
  );

  digitalWrite(
    RIGHT_RPWM,
    LOW
  );

  digitalWrite(
    RIGHT_LPWM,
    LOW
  );

  // ----------------------------------------------------------
  // ULTRASONIC PINS
  // ----------------------------------------------------------

  pinMode(
    LEFT_45_TRIG_PIN,
    OUTPUT
  );

  pinMode(
    LEFT_45_ECHO_PIN,
    INPUT
  );

  pinMode(
    LEFT_TRIG_PIN,
    OUTPUT
  );

  pinMode(
    LEFT_ECHO_PIN,
    INPUT
  );

  pinMode(
    CENTER_TRIG_PIN,
    OUTPUT
  );

  pinMode(
    CENTER_ECHO_PIN,
    INPUT
  );

  pinMode(
    RIGHT_TRIG_PIN,
    OUTPUT
  );

  pinMode(
    RIGHT_ECHO_PIN,
    INPUT
  );

  pinMode(
    RIGHT_45_TRIG_PIN,
    OUTPUT
  );

  pinMode(
    RIGHT_45_ECHO_PIN,
    INPUT
  );

  digitalWrite(
    LEFT_45_TRIG_PIN,
    LOW
  );

  digitalWrite(
    LEFT_TRIG_PIN,
    LOW
  );

  digitalWrite(
    CENTER_TRIG_PIN,
    LOW
  );

  digitalWrite(
    RIGHT_TRIG_PIN,
    LOW
  );

  digitalWrite(
    RIGHT_45_TRIG_PIN,
    LOW
  );

  // ----------------------------------------------------------
  // ATTACH SERVOS FIRST
  // ----------------------------------------------------------

  attachServoSafely(
    shoulderServo,
    SHOULDER_PIN
  );

  attachServoSafely(
    elbowServo,
    ELBOW_PIN
  );

  attachServoSafely(
    wristServo,
    WRIST_PIN
  );

  attachServoSafely(
    rotateServo,
    ROTATE_PIN
  );

  attachServoSafely(
    gripperServo,
    GRIPPER_PIN
  );

  shoulderServo.write(
    currentS
  );

  elbowServo.write(
    currentE
  );

  wristServo.write(
    currentW
  );

  rotateServo.write(
    currentR
  );

  gripperServo.write(
    currentG
  );

  delay(2500);

  // ----------------------------------------------------------
  // ATTACH MOTOR PWM AFTER SERVOS
  // ----------------------------------------------------------

  attachMotorPwm(
    LEFT_RPWM,
    LEFT_RPWM_CHANNEL
  );

  attachMotorPwm(
    LEFT_LPWM,
    LEFT_LPWM_CHANNEL
  );

  attachMotorPwm(
    RIGHT_RPWM,
    RIGHT_RPWM_CHANNEL
  );

  attachMotorPwm(
    RIGHT_LPWM,
    RIGHT_LPWM_CHANNEL
  );

  stopDriveMotorsImmediate();

  // ----------------------------------------------------------
  // INITIAL STATE
  // ----------------------------------------------------------

  currentArmState =
    INITIAL_POSITION;

  armBusy = false;

  currentRobotMode =
    MODE_STOPPED;

  fineAlignmentMode =
    false;

  resetFinalConfirmation();
  resetRoamingAvoidance();

  left45DistanceCm = -1.0;
  leftDistanceCm = -1.0;
  rightDistanceCm = -1.0;
  right45DistanceCm = -1.0;

  lastCommandTime =
    millis();

  lastRoamingScanTime = 0;

  Serial.println(
    "CORRECTED_CONTINUOUS_SYSTEM_READY"
  );
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  // Process Python commands first.
  if (
    Serial.available() > 0
  ) {
    String command =
      Serial.readStringUntil(
        '\n'
      );

    processCommand(
      command
    );
  }

  // Autonomous outer-sensor scanning and obstacle response.
  if (
    !armBusy &&
    currentRobotMode ==

    MODE_ROAMING
  ) {
    if (
      millis() -
      lastRoamingScanTime >=

      ROAM_SCAN_INTERVAL_MS
    ) {
      updateRoamingAutonomy();

      lastRoamingScanTime =
        millis();
    }
  }

  // Communication-loss safety.
  if (
    !armBusy &&
    currentRobotMode !=
    MODE_STOPPED &&
    millis() -
    lastCommandTime >
    COMMUNICATION_TIMEOUT_MS
  ) {
    currentRobotMode =
      MODE_STOPPED;

    stopDriveMotorsImmediate();

    fineAlignmentMode =
      false;

    resetFinalConfirmation();
    resetRoamingAvoidance();
  }
}