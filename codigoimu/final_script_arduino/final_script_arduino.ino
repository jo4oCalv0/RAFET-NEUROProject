#include <Servo.h>

// Criação dos objetos Servo
Servo servo_yaw;     // Movimento da base (Yaw)
Servo servo_pitch;   // Movimento vertical (Pitch)
Servo servo_pulse;
Servo servo_gripper;

// Pinos correspondentes
const int SERVO_YAW_PIN = 4;
const int SERVO_PITCH = 5;
const int SERVO_PITCH_PULSE = 6;
const int SERVO_GRIP = 7;

void setup() {
  Serial.begin(9600);
  servo_yaw.attach(SERVO_YAW_PIN);
  servo_pitch.attach(SERVO_PITCH);
  servo_gripper.attach(SERVO_GRIP);
  servo_pulse.attach(SERVO_PITCH_PULSE);

  // Posições iniciais (neutras)
  servo_yaw.write(90);
  servo_pitch.write(90);
  servo_pulse.write(90);
  servo_gripper.write(175);
}

void loop() {
  if (Serial.available()) {
    // Lê a linha recebida (espera algo como "120,90\n")
    String data = Serial.readStringUntil('\n');

    // Encontra a posição da vírgula
    int commaIndex = data.indexOf(',');

    // Garante que a vírgula existe
    if (commaIndex > 0) {
      // Extrai os dois valores como strings
      String yawStr = data.substring(0, commaIndex);
      String pitchStr = data.substring(commaIndex + 1);

      // Converte para inteiros
      int yawVal = yawStr.toInt();
      int pitchVal = pitchStr.toInt();

      // Limita os valores entre 0 e 180 graus
      yawVal = constrain(yawVal, 0, 180);
      pitchVal = constrain(pitchVal, 0, 180);

      // Atualiza os servos
      servo_yaw.write(yawVal);
      servo_pulse.write(pitchVal);

      // (Opcional) Debug:
      // Serial.print("Yaw: "); Serial.print(yawVal);
      // Serial.print(" | Pitch: "); Serial.println(pitchVal);
    }
  }
}