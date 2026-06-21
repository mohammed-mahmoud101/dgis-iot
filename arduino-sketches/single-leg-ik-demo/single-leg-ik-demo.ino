/*

  ESP32 + PCA9685 Quadruped Leg Walking Demo

  Features:

  - Inverse Kinematics for natural leg movement

  - Bezier curves for smooth, organic motion

  - Single leg walking cycle demonstration

  - Clean, readable code structure

  Hardware:

  - ESP32 microcontroller

  - PCA9685 PWM servo driver

  - 3 servos per leg (Coxa, Femur, Tibia)

*/

  

#include <Wire.h>

#include <Adafruit_PWMServoDriver.h>

  

// ========================================

// HARDWARE CONFIGURATION

// ========================================

  

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

  

#define SERVOMIN  80   // Servo minimum pulse length

#define SERVOMAX  600  // Servo maximum pulse length

  

// Servo pin assignments for one leg

#define COXA_SERVO   0  // Hip rotation (left/right)

#define FEMUR_SERVO  1  // Upper leg (up/down)

#define TIBIA_SERVO  2  // Lower leg (knee)

  

// ========================================

// LEG GEOMETRY (in millimeters)

// ========================================

  

const float COXA_LENGTH  = 85.0;  // Hip segment length

const float FEMUR_LENGTH = 104.0;  // Upper leg length

const float TIBIA_LENGTH = 154.0; // Lower leg length

  

// ========================================

// POSITION STRUCTURE

// ========================================

  

struct Point3D {

  float x;  // Forward/backward

  float y;  // Left/right

  float z;  // Up/down

};

  

struct JointAngles {

  float coxa;   // Hip angle in degrees

  float femur;  // Upper leg angle in degrees

  float tibia;  // Lower leg angle in degrees

};

  

// ========================================

// WALKING CYCLE PARAMETERS

// ========================================

  

// Idle standing position (foot position in 3D space)

const Point3D IDLE_POSITION = {16, -10, -120.0};

  

// Walking trajectory control points for Bezier curve (calibrated)

const Point3D STEP_START    = {166.00, 63.00, 50.00};    // Back position, foot on ground

const Point3D STEP_LIFT     = {110.00, 15.00, 102.00};   // Lift foot up

const Point3D STEP_FORWARD  = {51.00, -67.00, 102.00};   // Move forward in air

const Point3D STEP_END      = {23.00, -70.20, 50.00};    // Front position, foot down

  

const int STEP_DURATION_MS = 1000;  // Time for one complete step

const int STEP_RESOLUTION  = 50;    // Number of points in walk cycle

  

// ========================================

// INVERSE KINEMATICS

// ========================================

  

// Calculate joint angles needed to reach a 3D point

JointAngles inverseKinematics(Point3D target) {

  JointAngles angles;

  // Step 1: Calculate coxa (hip rotation) angle

  angles.coxa = atan2(target.y, target.x) * 180.0 / PI;

  // Step 2: Calculate distance from coxa joint to target

  float horizontalDist = sqrt(target.x * target.x + target.y * target.y) - COXA_LENGTH;

  float verticalDist = -target.z;  // Z is negative downward

  float targetDist = sqrt(horizontalDist * horizontalDist + verticalDist * verticalDist);

  // Step 3: Use law of cosines for femur and tibia angles

  // Calculate femur angle

  float cosAngleFemur = (FEMUR_LENGTH * FEMUR_LENGTH + targetDist * targetDist -

                         TIBIA_LENGTH * TIBIA_LENGTH) /

                        (2.0 * FEMUR_LENGTH * targetDist);

  float angleFemur = acos(constrain(cosAngleFemur, -1.0, 1.0));

  float baseAngle = atan2(verticalDist, horizontalDist);

  angles.femur = (baseAngle + angleFemur) * 180.0 / PI;

  // Calculate tibia angle

  float cosAngleTibia = (FEMUR_LENGTH * FEMUR_LENGTH + TIBIA_LENGTH * TIBIA_LENGTH -

                         targetDist * targetDist) /

                        (2.0 * FEMUR_LENGTH * TIBIA_LENGTH);

  angles.tibia = acos(constrain(cosAngleTibia, -1.0, 1.0)) * 180.0 / PI;

  return angles;

}

  

// ========================================

// BEZIER CURVE INTERPOLATION

// ========================================

  

// Calculate point on cubic Bezier curve

// t goes from 0.0 to 1.0

Point3D bezierPoint(Point3D p0, Point3D p1, Point3D p2, Point3D p3, float t) {

  float u = 1.0 - t;

  float tt = t * t;

  float uu = u * u;

  float uuu = uu * u;

  float ttt = tt * t;

  Point3D result;

  result.x = uuu * p0.x + 3 * uu * t * p1.x + 3 * u * tt * p2.x + ttt * p3.x;

  result.y = uuu * p0.y + 3 * uu * t * p1.y + 3 * u * tt * p2.y + ttt * p3.y;

  result.z = uuu * p0.z + 3 * uu * t * p1.z + 3 * u * tt * p2.z + ttt * p3.z;

  return result;

}

  

// ========================================

// SERVO CONTROL

// ========================================

  

// Move servo to specified angle (0-180 degrees)

void setServoAngle(uint8_t servoNum, float angle) {

  // Constrain angle to valid range

  angle = constrain(angle, 0, 180);

  // Convert angle to pulse width

  int pulseWidth = map(angle * 10, 0, 1800, SERVOMIN, SERVOMAX);

  // Set servo position

  pca9685.setPWM(servoNum, 0, pulseWidth);

}

  

// Move entire leg to target joint angles

void moveLegToAngles(JointAngles angles) {

  setServoAngle(COXA_SERVO, angles.coxa + 90);    // Offset to center

  setServoAngle(FEMUR_SERVO, angles.femur);

  setServoAngle(TIBIA_SERVO, angles.tibia);

}

  

// Move leg to 3D position using inverse kinematics

void moveLegToPosition(Point3D position) {

  JointAngles angles = inverseKinematics(position);

  moveLegToAngles(angles);

}

  

// ========================================

// WALKING CYCLE

// ========================================

  

// Execute one complete walking step with Bezier smoothing

void walkingCycle() {

  for (int i = 0; i <= STEP_RESOLUTION; i++) {

    // Calculate position along Bezier curve (0.0 to 1.0)

    float t = (float)i / (float)STEP_RESOLUTION;

    // Generate smooth foot position using cubic Bezier curve

    Point3D footPosition = bezierPoint(STEP_START, STEP_LIFT, STEP_FORWARD, STEP_END, t);

    // Move leg to calculated position

    moveLegToPosition(footPosition);

    // Small delay for smooth motion

    delay(STEP_DURATION_MS / STEP_RESOLUTION);

    // Debug output

    if (i % 10 == 0) {

      Serial.print("Step progress: ");

      Serial.print((int)(t * 100));

      Serial.print("% - Position: X=");

      Serial.print(footPosition.x);

      Serial.print(", Y=");

      Serial.print(footPosition.y);

      Serial.print(", Z=");

      Serial.println(footPosition.z);

    }

  }

}

  

// ========================================

// SETUP AND MAIN LOOP

// ========================================

  

void setup() {

  // Initialize serial communication

  Serial.begin(115200);

  delay(1000);

  Serial.println("\n=================================");

  Serial.println("Quadruped Leg Walking Demo");

  Serial.println("=================================\n");

  // Initialize PCA9685

  if (!pca9685.begin()) {

    Serial.println("ERROR: PCA9685 not found!");

    while (1);

  }

  pca9685.setPWMFreq(50);  // 50Hz for servos

  delay(100);

  Serial.println("Hardware initialized successfully\n");

  // Move to idle position

  Serial.println("Moving to idle position...");

  moveLegToPosition(IDLE_POSITION);

  delay(2000);

  Serial.println("Starting walking cycle...\n");

}

  

void loop() {

  // Continuously perform walking cycle

  walkingCycle();

  

  // Brief pause between cycles

  delay(500);

}
