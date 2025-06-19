#include <Servo.h>

Servo servo;
int currentAngle = 90; // ângulo inicial (posição neutra)

const int angulo_subir = 120;   // ângulo do servo para braço subir
const int angulo_descer = 60;   // ângulo do servo para braço descer

void setup() {
  Serial.begin(9600);
  servo.attach(5);           // pino do servo
  servo.write(currentAngle); // começa no meio
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    float az = input.toFloat(); // leitura vinda do Python

    // Verifica os thresholds com base no gráfico
    if (az > -0.8) {
      moveServoSmoothly(currentAngle, angulo_subir);
      currentAngle = angulo_descer;
    }
    else if (az < -1.2) {
      moveServoSmoothly(currentAngle, angulo_descer);
      currentAngle = angulo_subir;
    }
    // Opcional: mantém posição atual se estiver entre os thresholds
  }
}

void moveServoSmoothly(int startAngle, int endAngle) {
  int stepDelay = 10; // ms entre cada passo
  int stepSize = 1;   // graus por passo

  if (startAngle == endAngle) return; // já está na posição

  if (startAngle < endAngle) {
    for (int angle = startAngle; angle <= endAngle; angle += stepSize) {
      servo.write(angle);
      delay(stepDelay);
    }
  } else {
    for (int angle = startAngle; angle >= endAngle; angle -= stepSize) {
      servo.write(angle);
      delay(stepDelay);
    }
  }
}