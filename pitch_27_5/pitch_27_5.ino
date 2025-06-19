#include <Servo.h>

Servo servo_pulso;    // Para o movimento Pitch (cima-baixo) - Ex: Servo 10
Servo servo_yaw;
Servo servo_gripper;
Servo servo_pitch;

const int SERVO_PULSO = 6; 
const int SERVO_PITCH = 5;
const int SERVO_YAW = 4;
const int SERVO_GRIP = 7;

void setup() {
  Serial.begin(9600);
  servo_pulso.attach(SERVO_PULSO);
  servo_yaw.attach(SERVO_YAW);
  servo_gripper.attach(SERVO_GRIP);
  servo_pitch.attach(SERVO_PITCH);
  
  // Posição inicial do servo (centro ou seguro)
  servo_pulso.write(180);
  servo_yaw.write(90);
  servo_gripper.write(130);
  servo_pitch.write(90);
}

void loop() {
  if (Serial.available()) {
    // Lê a string completa enviada pelo Python até encontrar uma nova linha '\n'
    String data = Serial.readStringUntil('\n');

    // Converte a string para um número inteiro
    int pitch_val = data.toInt();

    // Garante que o valor está dentro do range de 0 a 180 para o servo
    pitch_val = constrain(pitch_val, 0, 180);
    
    servo_pitch.write(pitch_val);    // Controla o servo do ombro com o pitch
    
    // Opcional: Imprime o valor recebido no Serial Monitor do Arduino para depuração
    // Serial.print("Recebido Pitch: "); Serial.println(pitch_val);
  }
}