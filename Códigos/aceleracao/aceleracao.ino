#define pin_ch3 6 // Canal 3 aceleração = pino 6  pwm
#define pin_motor1 10 // Motor a ser acelerado
#define angulo 0
#include<Servo.h>

Servo motor1;
int leitura1,leitura2; // Guarda as leituras do canal 3
int media = 0;
int cal = 0;
int cont = 0;

void setup() {
  Serial.begin(9600);
  pinMode(pin_ch3, INPUT); // Entrada de dados do canal 3
  motor1.attach(pin_motor1); // Motor 1 no pino 10
  motor1.writeMicroseconds(1000); // Aceleração inicial do motor
  leitura1 = pulseIn(pin_ch3, HIGH); // Primeira Leitura do canal aceleração

}

void loop() {
  leitura2 = pulseIn(pin_ch3, HIGH); // Segunda Leitura para comparação
  int diferenca = (leitura2 > leitura1 ? leitura2 - leitura1 : leitura1 - leitura2) ; // a diferenca recebe sempre um valor positivo entre o valor lido no setup e o lido agora
  if (diferenca > 10) { // diferença grande
    cont++;
    if (cont == 10) {
      motor1.writeMicroseconds(leitura2);
      leitura1 = leitura2;
      cont = 0;
    }

  } else {
    cont = 0;
  }
  Serial.println(leitura2);
  //aceleracao = leitura2;
}

