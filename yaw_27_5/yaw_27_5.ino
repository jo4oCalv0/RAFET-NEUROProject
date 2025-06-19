#include <Servo.h>

// Definição do Servo 4 - Servo da Base do Robô
Servo servo_pulso;    // Para o movimento Pitch (cima-baixo) - Ex: Servo 10
Servo servo_yaw;
Servo servo_gripper;
Servo servo_pitch;

const int SERVO_PULSO = 6; 
const int SERVO_PITCH = 5;
const int SERVO_YAW = 4;
const int SERVO_GRIP = 7;

void setup() {
  Serial.begin(9600); // Inicializa a comunicação serial com o mesmo baud rate do Python
  servo_pulso.attach(SERVO_PULSO);
  servo_yaw.attach(SERVO_YAW);
  servo_gripper.attach(SERVO_GRIP);
  servo_pitch.attach(SERVO_PITCH);
  
  // Posição inicial do servo (centro ou seguro)
  servo_pulso.write(90);
  servo_yaw.write(90);
  servo_gripper.write(175);
  servo_pitch.write(90);
}

void loop() {
  if (Serial.available()) {
    // Lê a string completa enviada pelo Python até encontrar uma nova linha '\n'
    String data = Serial.readStringUntil('\n');

    // Converte a string para um número inteiro
    // Como estamos a enviar apenas um número, não precisamos de parsing complexo
    int yaw_angle_received = data.toInt();

    // Garante que o valor está dentro do range de 0 a 180 para o servo
    yaw_angle_received = constrain(yaw_angle_received, 0, 180);

    // Escreve o ângulo recebido para o Servo 4 (Base)
    servo_yaw.write(yaw_angle_received);

    // Opcional: Imprime o valor recebido no Serial Monitor do Arduino para depuração
    // Serial.print("Recebido Yaw: ");
    // Serial.println(yaw_angle_received);
  }
}