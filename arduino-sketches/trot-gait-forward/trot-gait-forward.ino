#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm;

#define SERVO_MIN 100
#define SERVO_MAX 600
#define PWM_FREQ  50

// ---------------------------------------------------------
// Servo channel map (matches your wiring comment):
//   0  1  2   -> Front-Left  (FL): base, hip, leg
//   3  4  5   -> Front-Right (FR): base, hip, leg
//   6  7  8   -> Back-Left   (BL): base, hip, leg
//   9 10 11   -> Back-Right  (BR): base, hip, leg
// ---------------------------------------------------------
#define FL_BASE 0
#define FL_HIP  1
#define FL_LEG  2

#define FR_BASE 3
#define FR_HIP  4
#define FR_LEG  5

#define BL_BASE 6
#define BL_HIP  7
#define BL_LEG  8

#define BR_BASE 9
#define BR_HIP  10
#define BR_LEG  11

// ---------------------------------------------------------
// Tunable angles - adjust these to match your physical build
// ---------------------------------------------------------
const float NEUTRAL_HIP   = 60;   // standing hip angle (leg down, body supported)
const float NEUTRAL_LEG   = 60;   // standing knee/leg angle
const float LIFT_HIP      = 90;   // hip angle when leg is raised (lifts foot off ground)
const float LIFT_LEG      = 90;   // leg angle when raised

const float BASE_CENTER   = 90;   // base servo "neutral" rotation
const float BASE_FORWARD  = 120;  // base servo swung forward (toward direction of travel)
const float BASE_BACKWARD = 60;   // base servo swung backward (push/drive stroke)

const int   STEP_DELAY    = 15;   // ms between interpolation steps (lower = faster, less smooth)
const int   PHASE_PAUSE   = 200;  // ms pause after each phase completes

// Convert degree (0-180) to PWM pulse
uint16_t degreeToPulse(float degree) {
  degree = constrain(degree, 0, 180);
  return map(degree, 0, 180, SERVO_MIN, SERVO_MAX);
}

void setServo(uint8_t channel, float degree) {
  pwm.setPWM(channel, 0, degreeToPulse(degree));
}

// Smoothly move two legs' worth of servos (6 channels) from current
// angles to target angles together, so motion looks fluid rather than snapping.
void moveServosSmooth(uint8_t channels[], float startAngles[], float endAngles[], int count) {
  int steps = 30;
  for (int s = 0; s <= steps; s++) {
    for (int i = 0; i < count; i++) {
      float angle = startAngles[i] + (endAngles[i] - startAngles[i]) * s / (float)steps;
      setServo(channels[i], angle);
    }
    delay(STEP_DELAY);
  }
}

// ---------------------------------------------------------
// Helper to set one leg's 3 servos directly (no interpolation)
// ---------------------------------------------------------
void setLeg(uint8_t baseCh, uint8_t hipCh, uint8_t legCh,
            float baseDeg, float hipDeg, float legDeg) {
  setServo(baseCh, baseDeg);
  setServo(hipCh,  hipDeg);
  setServo(legCh,  legDeg);
}

// ---------------------------------------------------------
// Stand all 4 legs in neutral position
// ---------------------------------------------------------
void standNeutral() {
  setLeg(FL_BASE, FL_HIP, FL_LEG, BASE_CENTER, NEUTRAL_HIP, NEUTRAL_LEG);
  setLeg(FR_BASE, FR_HIP, FR_LEG, BASE_CENTER, NEUTRAL_HIP, NEUTRAL_LEG);
  setLeg(BL_BASE, BL_HIP, BL_LEG, BASE_CENTER, NEUTRAL_HIP, NEUTRAL_LEG);
  setLeg(BR_BASE, BR_HIP, BR_LEG, BASE_CENTER, NEUTRAL_HIP, NEUTRAL_LEG);
  delay(500);
}

// ---------------------------------------------------------
// Trot gait: diagonal pairs move together.
//   Diagonal A = Front-Left  + Back-Right
//   Diagonal B = Front-Right + Back-Left
//
// One full step cycle for a diagonal pair:
//   1) Lift the pair (raise hip/leg)
//   2) Swing the pair's base servo forward while lifted
//   3) Lower the pair back down (foot plants on ground, forward of start)
//   4) Drive: rotate base servo backward while down -> pushes body forward
// While one diagonal pair does this, the other diagonal pair stays planted
// and its base servos rotate backward in sync with step 4 to also help
// drive the body forward (stance/support phase).
// ---------------------------------------------------------

void liftPair(uint8_t baseA, uint8_t hipA, uint8_t legA,
              uint8_t baseB, uint8_t hipB, uint8_t legB) {
  uint8_t channels[6]   = {hipA, legA, hipB, legB, baseA, baseB};
  float startAngles[6]  = {NEUTRAL_HIP, NEUTRAL_LEG, NEUTRAL_HIP, NEUTRAL_LEG, BASE_BACKWARD, BASE_BACKWARD};
  float endAngles[6]    = {LIFT_HIP, LIFT_LEG, LIFT_HIP, LIFT_LEG, BASE_BACKWARD, BASE_BACKWARD};
  moveServosSmooth(channels, startAngles, endAngles, 6);
}

void swingPairForward(uint8_t baseA, uint8_t baseB) {
  uint8_t channels[2]  = {baseA, baseB};
  float startAngles[2] = {BASE_BACKWARD, BASE_BACKWARD};
  float endAngles[2]   = {BASE_FORWARD, BASE_FORWARD};
  moveServosSmooth(channels, startAngles, endAngles, 2);
}

void lowerPair(uint8_t baseA, uint8_t hipA, uint8_t legA,
               uint8_t baseB, uint8_t hipB, uint8_t legB) {
  uint8_t channels[6]   = {hipA, legA, hipB, legB, baseA, baseB};
  float startAngles[6]  = {LIFT_HIP, LIFT_LEG, LIFT_HIP, LIFT_LEG, BASE_FORWARD, BASE_FORWARD};
  float endAngles[6]    = {NEUTRAL_HIP, NEUTRAL_LEG, NEUTRAL_HIP, NEUTRAL_LEG, BASE_FORWARD, BASE_FORWARD};
  moveServosSmooth(channels, startAngles, endAngles, 6);
}

// Drive stroke: planted feet rotate base servo from FORWARD back to BACKWARD,
// which pushes the body forward over the ground (this is what actually
// propels the robot). Applies to whichever pair is currently down/stance.
void driveStroke(uint8_t baseA, uint8_t baseB) {
  uint8_t channels[2]  = {baseA, baseB};
  float startAngles[2] = {BASE_FORWARD, BASE_FORWARD};
  float endAngles[2]   = {BASE_BACKWARD, BASE_BACKWARD};
  moveServosSmooth(channels, startAngles, endAngles, 2);
}

// One full trot step: swap which diagonal pair is swinging vs driving
void trotStep(bool diagA_swings) {
  if (diagA_swings) {
    // Diagonal A (FL+BR) swings through the air, Diagonal B (FR+BL) drives
    liftPair(FL_BASE, FL_HIP, FL_LEG, BR_BASE, BR_HIP, BR_LEG);
    delay(PHASE_PAUSE);
    swingPairForward(FL_BASE, BR_BASE);
    // simultaneously drive the planted diagonal B backward to push body forward
    driveStroke(FR_BASE, BL_BASE);
    delay(PHASE_PAUSE);
    lowerPair(FL_BASE, FL_HIP, FL_LEG, BR_BASE, BR_HIP, BR_LEG);
    delay(PHASE_PAUSE);
  } else {
    // Diagonal B (FR+BL) swings through the air, Diagonal A (FL+BR) drives
    liftPair(FR_BASE, FR_HIP, FR_LEG, BL_BASE, BL_HIP, BL_LEG);
    delay(PHASE_PAUSE);
    swingPairForward(FR_BASE, BL_BASE);
    driveStroke(FL_BASE, BR_BASE);
    delay(PHASE_PAUSE);
    lowerPair(FR_BASE, FR_HIP, FR_LEG, BL_BASE, BL_HIP, BL_LEG);
    delay(PHASE_PAUSE);
  }
}

void walkForward(int numSteps) {
  bool diagA_swings = true;
  for (int i = 0; i < numSteps; i++) {
    trotStep(diagA_swings);
    diagA_swings = !diagA_swings; // alternate diagonal pairs each step
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Quadruped hardcoded forward walk - starting");

  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);
  delay(500);

  standNeutral();
  delay(1000);

  walkForward(10);   // walk forward 10 trot steps
}

void loop() {
  // Keep walking continuously. Remove/comment this if you only want
  // the 10 steps from setup() and then stop.
  walkForward(1);
}
