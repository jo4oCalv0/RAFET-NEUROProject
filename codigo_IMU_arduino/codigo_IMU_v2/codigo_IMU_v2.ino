#include <Servo.h>

Servo servo_pitch; // up/down
Servo servo_roll;  // left/right
Servo servo_yaw;   // twist

int angle_pitch = 90;
int angle_roll = 90;
int angle_yaw = 90;

// Pins for the 3 main joints
const int PITCH_PIN = 5;
const int ROLL_PIN = 6;
const int YAW_PIN = 4;

void setup() {
  Serial.begin(9600);
  servo_pitch.attach(PITCH_PIN);
  servo_roll.attach(ROLL_PIN);
  servo_yaw.attach(YAW_PIN);

  // Initialize to neutral positions
  servo_pitch.write(angle_pitch);
  servo_roll.write(angle_roll);
  servo_yaw.write(angle_yaw);
}

void loop() {
  static String input = "";

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      parseAndSetAngles(input);
      input = "";
    } else {
      input += c;
    }
  }
}

void parseAndSetAngles(String data) {
  float pitch = 0.0, roll = 0.0, yaw = 0.0;

  int pIndex = data.indexOf("PITCH:");
  int rIndex = data.indexOf("ROLL:");
  int yIndex = data.indexOf("YAW:");

  if (pIndex >= 0 && rIndex >= 0 && yIndex >= 0) {
    pitch = data.substring(pIndex + 6, rIndex - 1).toFloat();
    roll = data.substring(rIndex + 5, yIndex - 1).toFloat();
    yaw = data.substring(yIndex + 4).toFloat();

    // Map angle ranges from [-90, +90] to [0, 180] for servos
    angle_pitch = constrain(mapFloat(pitch, -90, 90, 0, 180), 0, 180);
    angle_roll  = constrain(mapFloat(roll,  -90, 90, 0, 180), 0, 180);
    angle_yaw   = constrain(mapFloat(yaw,   -90, 90, 0, 180), 0, 180);

    // Move servos
    servo_pitch.write(angle_pitch);
    servo_roll.write(angle_roll);
    servo_yaw.write(angle_yaw);

    Serial.print("Set angles: ");
    Serial.print(angle_pitch); Serial.print(", ");
    Serial.print(angle_roll); Serial.print(", ");
    Serial.println(angle_yaw);
  }
}

// int limitChange(int current, int target, int maxStep) {
//   if (abs(target - current) <= maxStep) return target;
//   return current + (target > current ? maxStep : -maxStep);
// }

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
