#include <PID_v1.h>
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <Servo.h>

/*Canais do Rádio Controle - Turnigy 9X: MODE 2 */
#define PIN_CH1 4 // Roll
#define PIN_CH2 5 // Pitch
#define PIN_CH3 6 // Trhottle
#define PIN_CH4 7 // Yaw

/*ESCs*/
#define ESC1 8 // Esquerda Frente
#define ESC2 9 // Direita Frente
#define ESC3 10 // Esquerda Tras.
#define ESC4 11 // Direita Tras.

Servo motor1; // Esquerda Frente
Servo motor2; // Direita Frente
Servo motor3; // Esquerda Tras.
Servo motor4; // Direita Tras.

Kalman kalmanX; // Instâncias para o filtro de Kalman
Kalman kalmanY;
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

/*Variáveis de controle do Rádio*/
double  roll = 0,     //Canal 1   Eixo X
        pitch = 0,    //Canal 1   Eixo Y
        throttle = 1300, //Canal 1   Aceleração
        yaw = 0;      //Canal 4   Eixo Z


/* Variáveis para cálculos com o MPU-6050 */
double accX, accY, accZ; //Valores lidos do acelerometro
double gyroX, gyroY, gyroZ; //Valores lidos do giroscópio

/*Angulos calculados usando o filtro de Kalman
  usadas para receber a leitura do MPU*/
double kalAngleX, // Esquerda e Direita
       kalAngleY, // Frente a trás
       ang_desejado = 0; //Valor Constante!

uint8_t i2cData[14]; // Buffer para dados protocolo I2C
uint32_t timer;

/*variáveis que aplicam aceleração aos motores individualmente*/
double aceleracaoM1,
       aceleracaoM2,
       aceleracaoM3,
       aceleracaoM4;

/*Constantes PID:*/
double Kp = 1.00; // 0.35
double Ki = 0.0005; // 0.03
double Kd = 0.05; // 0.05
double pidX, pidY, erroX = 0, erroY, erroAnteriorX = 0, erroAnteriorY = 0;
double P = 0;
double inte = 0;
double D = 0;
double pidDt = 0.08; // Variação de tempo/Tempo do Loop

/*Objetos para Cálculo do PID*/
//PID pidRoll(&kalAngleX, &pidX, &ang_desejado, Kp, Ki, Kd, DIRECT);
//PID pidPitch(&kalAngleY, &pidY, &ang_desejado, Kp, Ki, Kd, DIRECT);


void setup() {
  /*Monitoramento*/
  Serial.begin(115200);
  Wire.begin();

  /*Pinos dos motores*/
  motor1.attach(ESC1); // Esquerda Frente
  motor2.attach(ESC2); // Direita Frente
  motor3.attach(ESC3); // Esquerda Tras.
  motor4.attach(ESC4); // Direita Tras.
  /*Pinos de leitura do receptor rádio*/
  pinMode(PIN_CH1, INPUT); // entrada Roll
  pinMode(PIN_CH2, INPUT); // entrada Pitch
  pinMode(PIN_CH3, INPUT); // entrada Trhottle
  pinMode(PIN_CH4, INPUT); // entrada Yaw

  /*Aplicar aceleração inicial nos ESCs*/
  initMotores();
  /*Métodos da Biblioteca PID*/
  //  pidRoll.SetMode(AUTOMATIC); // Modo de trabalho
  //  pidPitch.SetMode(AUTOMATIC);
  //  pidRoll.SetSampleTime(5); // tempo do loop
  //  pidPitch.SetSampleTime(5);

  /************** Protocolo I2C***************************/
  iniciarMPU();
  /******************************************************************/

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  /*Leituras do acelerometro no MPU*/
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees

  // Se RESTRICT_PITCH foi definido, compila essa parte do cálculo:
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  //  kalmanX.setAngle(roll); // Filtra o sinal dos ângulos iniciais
  //  kalmanY.setAngle(pitch);
  timer = micros(); // Pega o tempo de funcionamento da placa, para uso do Filtro.

  delay(4000); // Aguarda os motores calibrarem (Bips)
}

void loop() {


  /* Atualiza os Valores do acelerometro */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  /* Atualiza os Valores do giroscópio */
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calcula a variação do tempo
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


  /*****************************************************************************************/
  //Rádio controle:





  /*Controlador PID***********************************************************************************************/

  //           ________________________________
  //          |            MPU-6050            |
  //          |            ________            |
  //          |           |________|           |
  //          |            _________           |
  //          |           |  Acel   | ^        |
  //          |           |  Giro   | | X      |
  //          |           |__Temp___|          |
  //          |               <---             |
  //          |                Y               |
  //          | +   -  scl sda xda xcl ado int |
  //          | 0   0   0   0   0   0   0   0  |
  //          |________________________________|
  //





  erroX = ang_desejado - kalAngleX;


  // Proporcional
  P = Kp * erroX;

  //Derivativo
  D = (erroX - erroAnteriorX) / pidDt;


// Anti Wind-up - Integração condicional
  if (abs(pidX) >= 5.0 && (((erroX >= 0) && (inte >= 0)) || ((erroX < 0) && (inte < 0)))) {
    Serial.print("ENTROU!!!!!");
    inte = inte;                  // Mantem o valor de I
  } else {
    inte += inte + (Ki*(erroX * dt));   //Integrando
  }

  

  //Soma dos termos
  pidX = P + inte + (Kd * D);

  

  erroAnteriorX = erroX;


  //Corrigindo aceleracao com o PID

  aceleracaoM1 = throttle - (pidX / 2);
  aceleracaoM2 = throttle + (pidX / 2);

  if (aceleracaoM1 > 1500) {      //
    aceleracaoM1 = 1500;

  }

  if (aceleracaoM2 > 1500) {      //
    aceleracaoM2 = 1500;

  }

  motor1.writeMicroseconds(aceleracaoM1);
  motor2.writeMicroseconds(aceleracaoM2);




  /* Print Data */

  Serial.print("Angulo: ");
  Serial.print(kalAngleX);
  Serial.print("\t");
  Serial.print("\t");

  Serial.print("Erro: ");
  Serial.print(erroX);
  Serial.print("\t");
  Serial.print("\t");

  Serial.print("P: ");
  Serial.print(P);
  Serial.print("\t");
  Serial.print("\t");

  Serial.print("I: ");
  Serial.print(inte);
  Serial.print("\t");
  Serial.print("\t");

  Serial.print("D: ");
  Serial.print(D);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("\t");

  Serial.print("PID: ");
  Serial.print(pidX);
  Serial.print("\t");
  Serial.print("\t");

  Serial.print("Esq M1: "); Serial.print(aceleracaoM1); Serial.print("\t");
  Serial.print("\t");
  Serial.print("Dir M2: "); Serial.print(aceleracaoM2); Serial.println("\t");





}














/*Método Chamado no Setup() para ajustar comunicação
  do Protocolo I2C e modos de leitura do MPU 6050.
  Verificando se a comunicação foi feita com sucesso*/
void iniciarMPU() {
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
}


void initMotores() {
  //Valor inicial para acionamento dos motores
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
}
