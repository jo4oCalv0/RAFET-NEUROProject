#include <Servo.h>

Servo servoPitch;  // Ex: servo do braço (subir/descer)
Servo servoYaw;    // Ex: servo da base (esquerda/direita)

void setup() {
  Serial.begin(9600);
  servoPitch.attach(5);  // pino do servo para pitch (ajusta conforme o teu braço)
  servoYaw.attach(4);     // pino do servo para yaw
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');

    int commaIndex = input.indexOf(',');
    if (commaIndex > 0) {
      int pitch = input.substring(0, commaIndex).toInt();
      int yaw = input.substring(commaIndex + 1).toInt();

      pitch = constrain(pitch, 0, 180);
      yaw = constrain(yaw, 0, 180);

      servoPitch.write(pitch);
      servoYaw.write(yaw);

      // Debug (opcional):
      Serial.print("Pitch: ");
      Serial.print(pitch);
      Serial.print(" | Yaw: ");
      Serial.println(yaw);
    }
  }
}