#include <PID_v1.h>
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <Servo.h>


/*Canais do Rádio Controle - Turnigy 9X*/
#define pin_ch1 4 // 
#define pin_ch2 5 //
#define pin_ch3 6 // Trhottle
#define pin_ch4 7 // Não Sei

/*ESCs*/
#define esc1 8 // Esquerda Frente
#define esc2 9 // Direita Frente
#define esc3 10 // Esquerda Tras.
#define esc4 11 // Direita Tras.

Servo motor1; // Esquerda Frente
Servo motor2; // Direita Frente
Servo motor3; // Esquerda Tras.
Servo motor4; // Direita Tras.

Kalman kalmanX; // Instancias para o filtro de Kalman
Kalman kalmanY;
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

/*Variáveis de controle do Rádio*/
double throttle = 0,  //Canal 1   Eixo
       roll = 0,      //Canal 2   Eixo
       pitch = 0,     //Canal 3   Aceleração
       yaw = 0;       //Canal 4   Eixo


/* Variáveis para o MPU-6050 */
double accX, accY, accZ; //Valores lidos do acelerometro
double gyroX, gyroY, gyroZ; //Valores lidos do giroscópio
double kalAngleX, kalAngleY; // Angulos calculados usando o filtro de Kalman

/*Calculos do PID*/
double anguloLidoX = 0;
double anguloLidoY = 0;
double ang_desejado = 0;

uint8_t i2cData[14]; // Buffer para dados protocolo I2C
uint32_t timer, temp_Pass = 0, temp_Prev;

/*Calculos do PID*/
double pid,
       aceleracaoM1,
       aceleracaoM2,
       aceleracaoM3,
       aceleracaoM4;

double p = 0;
double i = 0;
double d = 0;

/*Constantes PID:*/
double Kp = 0.35; // 0.35
double Ki = 0.07; // 0.03
double Kd = 0.17; // 0.05


PID pidRoll(&anguloLidoX, &pid, &ang_desejado, Kp, Ki, Kd, DIRECT);
PID pidPitch(&anguloLidoY, &pid, &ang_desejado, Kp, Ki, Kd, DIRECT);


void setup() {
  Serial.begin(115200);
  Wire.begin();

  /*Pinos dos*/
  motor1.attach(esc1); // Esquerda Frente
  motor2.attach(esc2); // Direita Frente
  motor3.attach(esc3); // Esquerda Tras.
  motor4.attach(esc4); // Direita Tras.

  motor1.writeMicroseconds(1000); //Valor inicial para acionamento dos motores
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);

  /*Pinos de leitura do receptor rádio*/
  pinMode(pin_ch1, INPUT); // entrada canal 1
  pinMode(pin_ch2, INPUT); // entrada canal 2
  pinMode(pin_ch3, INPUT); // entrada canal 3
  pinMode(pin_ch4, INPUT); // entrada canal 4

  roll = calibrarStick(pulseIn(pin_ch1, HIGH));
  pitch = calibrarStick(pulseIn(pin_ch2, HIGH));
  yaw = calibrarStick(pulseIn(pin_ch4, HIGH));
  throttle = aceleracaoThrottle(pulseIn(pin_ch3, HIGH));

  pidRoll.SetMode(AUTOMATIC);
  pidPitch.SetMode(AUTOMATIC);
  pidRoll.SetSampleTime(5); // tempo do loop
  pidPitch.SetSampleTime(5); // tempo do loop


  /************** Protocolo I2C***************************/
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Altera a frequencia I2C para 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Altera a frequencia I2C para 400kHz
#endif

  i2cData[0] = 7; // Altera a taxa de amostragem para 1000Hz - 8kHz/(7+1) = 1000Hz  (Define a frequencia de transmissao de dados do SDA)
  i2cData[1] = 0x00; // Desabilita FSYNC e define a filtragem do acelerometro em 260 Hz, 256 Hz para o filtro do gisroscópio, 8 KHz de amostragem.
  i2cData[2] = 0x00; // Altera a faixa completa de escala do giroscópio para ±250deg/s
  i2cData[3] = 0x00; // Altera a faixa completa de escala do Acelerometro para ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Escreve em todos os registradores de uma vez
  while (i2cWrite(0x6B, 0x01, true)); // PLL com eixo X do gyroscopio referencia e desabilita o modo sleep

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize
  /******************************************************************/

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Atribui os ângulos iniciais
  kalmanY.setAngle(pitch);

  timer = micros(); // Pega o tempo de funcionamento da placa, para uso do Filtro.

  delay(4000); // Aguarda os motores calibrarem (Bips)
  //tempo = millis(); // Inicia a contagem para a Derivada do PID
}

void loop() {


  /* Atualiza os Valores do sensor */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calcula o delta tempo
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // Conversão de radioanos para graus
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Conversão para deg/s
  double gyroYrate = gyroY / 131.0; // Conversão para deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    kalAngleX = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    kalAngleY = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  /*Controlador PID***********************************************************************************************/

  //angulo lido do MPU-6050
  anguloLidoX = kalAngleX;


  //Corrigindo aceleracao com o PID
  if ( anguloLidoX <= 0) { // se o angulo for negativo
    pidRoll.SetControllerDirection(DIRECT);
    pidRoll.Compute();
    aceleracaoM1 = throttle + pid;
    aceleracaoM2 = throttle - (pid);

  } else {
    pidRoll.SetControllerDirection(REVERSE);
    pidRoll.Compute();
    aceleracaoM2 = throttle + pid;
    aceleracaoM1 = throttle - (pid);
  }

  motor1.writeMicroseconds(aceleracaoM1);
  motor2.writeMicroseconds(aceleracaoM2 + 10);


  /* Print Data */
//
//  Serial.print("Angulo: ");
//  Serial.print(kalAngleX);
//  Serial.print("\t");
//
//  Serial.print("PID: ");
//  Serial.print(pid);
//  Serial.print("\t");
//
//  Serial.print("Esq M1: "); Serial.print(aceleracaoM1); Serial.print("\t");
//  Serial.print("\t");
//  Serial.print("Dir M2: "); Serial.print(aceleracaoM2); Serial.println("\t");
}


/**/
double calibrarStick(unsigned long leitura_ch) {
  if (leitura_ch != 0) {

  }

  return 0;
}

double acelerarThrottle(unsigned long leitura) {

  unsigned long aceleracao;
  int diferenca = 0;




  if (diferenca > 10) { // diferença grande
    for (int i = 0; i < 10; i++) {
      unsigned long leitura2 = pulseIn(pin_ch3, HIGH); // Segunda Leitura para comparação
      diferenca = (leitura2 > leitura ? leitura2 - leitura : leitura - leitura2) ; // a diferenca recebe sempre um valor positivo entre o valor lido no setup e o lido agora
    }
    
    
      aceleracao = leitura2;
      return aceleracao;
    }

  } else {
    cont = 0;
    return leitura;
  }
}
