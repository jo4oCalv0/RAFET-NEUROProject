#include <Servo.h>

Servo servo;
int angle = 90;

void setup() {
  Serial.begin(9600);        // Inicia comunicação serial a 9600 bauds
  servo.attach(5);           // Liga o servo ao pino digital 9
  servo.write(angle);        // Define o ângulo inicial do servo (90° = centro)
}

void loop() {
  if (Serial.available()) {                          // Verifica se há dados recebidos na Serial
    String input = Serial.readStringUntil('\n');     // Lê até receber um "newline"
    float az = input.toFloat();                      // Converte o texto recebido para número decimal (float)
    
    // Mapeia o valor de az (-1.0 a +1.0 g esperado) para ângulo de 0 a 180
    int mapped = map(az * 100, -100, 100, 0, 180);    // az * 100 → converte para int entre -100 a 100
    mapped = constrain(mapped, 0, 180);              // Garante que o valor está entre 0 e 180 (limite do servo)
    
    servo.write(mapped);                             // Move o servo para o ângulo correspondente
  }
}