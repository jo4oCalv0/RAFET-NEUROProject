#include <Servo.h>

Servo servoPitch;  // sobe/desce braço
Servo servoYaw;    // gira base
Servo servoRoll;   // movimento frente/trás (eixo X)

void setup() {
  Serial.begin(9600);
  servoPitch.attach(6);  // pino servo pitch
  servoYaw.attach(4);    // pino servo yaw
  servoRoll.attach(5);   // pino servo roll (frente/trás no eixo X)
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    
    // Parse for three values (pitch, yaw, roll)
    int firstComma = input.indexOf(',');
    int secondComma = input.indexOf(',', firstComma + 1);
    
    if (firstComma > 0 && secondComma > 0) {
      int pitch = input.substring(0, firstComma).toInt();
      int yaw = input.substring(firstComma + 1, secondComma).toInt();
      int roll = input.substring(secondComma + 1).toInt();

      servoPitch.write(constrain(pitch, 0, 180));
      servoYaw.write(constrain(yaw, 0, 180));
      servoRoll.write(constrain(roll, 0, 180));
    }
  }
}