// BIBLIOTECAS:
#include <Wire.h> // COmunicação através de protocolo I2C
#include <Kalman.h> // Filtro de Kalman Source: https://github.com/TKJElectronics/KalmanFilter
#include <Servo.h> // Controle dos motores Brushless

//PINOS:
/*ESCs*/
#define ESC1 8 // Esquerda Frente
#define ESC2 9 // Direita Frente
/*Potenciometros de ajuste PID*/
#define P_pot A10 // FIO BRANCO
#define I_pot A9  // FIO ROXO
#define D_pot A8  // FIO AZUL

//OBJETOS:
Servo motor1; // Esquerda Frente
Servo motor2; // Direita Frente
Kalman kalmanX; // Instâncias para o filtro de Kalman
Kalman kalmanY;

//VARIÁVEIS
/*Potenciometros*/
int leitura_P = 0;
int leitura_I = 0;
int leitura_D = 0;
/*Aceleração padrão*/
double  throttle = 1300;


/* Eixos do MPU-6050 */
double accX, accY, accZ; //Valores lidos do acelerometro
double gyroX, gyroY, gyroZ; //Valores lidos do giroscópio

/*Angulos calculados usando o filtro de Kalman
  usadas para receber a leitura do MPU*/
double kalAngleX, // Esquerda e Direita
       kalAngleY,
       ang_desejado = 0; //Valor Constante!

uint8_t i2cData[14]; // Buffer para dados protocolo I2C
uint32_t timer;
double pidDt, tempo, tempoAnterior = 0;

/*variáveis que aplicam aceleração aos motores individualmente*/
double aceleracaoM1,  //esquerdo
       aceleracaoM2 = 0;  //direito

/*Constantes PID:                   Atuação*/
double Kp = 0;  //0.6; // 1.85   0.6        Força
double Ki = 0;  //0.7; // 0.3    0.7        Precisão
double Kd = 0;  //0.76; //0.46 a 0.76     Velocidade

double pidX, erroX = 0, erroAnteriorX = 0;
double P = 0;
double I = 0;
double D = 0;


void setup() {
  /*Monitoramento*/
  Serial.begin(9600);
  Wire.begin();

  /*Pinos dos motores*/
  motor1.attach(ESC1); // Esquerda Frente
  motor2.attach(ESC2); // Direita Frente
  /*Pinos dos potenciometros de ajuste PID*/
  pinMode(P_pot, INPUT);
  pinMode(I_pot, INPUT);
  pinMode(D_pot, INPUT);
  /*Aplicar aceleração inicial nos ESCs*/
  initMotores();

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

  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  kalmanX.setAngle(roll); // Filtra o sinal dos ângulos iniciais
  kalmanY.setAngle(pitch);

  timer = micros(); // Pega o tempo de funcionamento da placa, para uso do Filtro.
  tempo = millis(); // Inicia o contador da variação de tempo do PID
  delay(4000); // Aguarda os motores calibrarem (Bips)
}




void loop() {

  /*Calculo da Variação de tempo para derivativa PID*/
  tempoAnterior = tempo;
  tempo = millis();
  pidDt = (tempo - tempoAnterior) / 1000;

  /* Atualiza os Valores do acelerometro */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  /* Atualiza os Valores do giroscópio */
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calcula a variação do tempo para o MPU
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // Conversão de radianos para graus

  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  double gyroXrate = gyroX / 131.0; // Conversão para deg/s
  double gyroYrate = gyroY / 131.0; // Conversão para deg/s

  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    kalAngleX = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

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


  leitura_P = analogRead(P_pot);
  leitura_I = analogRead(I_pot);
  leitura_D = analogRead(D_pot);
  Kp = map (leitura_P, 0, 1024, 0, 50 );
  Kp = (Kp / 100);
  Ki = map (leitura_I, 0, 1024, 0, 50 );
  Ki = (Ki / 100);
  Kd = map (leitura_D, 0, 1024, 0, 50 );
  Kd = (Kd / 100);


  //Salva o erro anterior e calcula o atual

  erroX = kalAngleX - ang_desejado;

  // Proporcional
  P = Kp * erroX;

  //Derivativo
  D = Kd * ((erroX - erroAnteriorX) / pidDt);

  //  Integral com Anti Wind-up condicional
  if (abs(pidX) >= 3 &&
      (((erroX >= 0) && (I >= 0)) ||
       ((erroX < 0) && (I < 0)))) {

    I = I;  // Mantem o valor de I

  } else {  //Integra normalmente

    I = I + (Ki * (erroX * pidDt));

  }

  //Soma os termos
  pidX = P + I + D;


  //Corrigindo aceleracao com o PID

  // Observou-se que se o PID é positivo,
  // o angulo lido é negativo e vice versa!
  aceleracaoM1 = throttle - pidX;
  aceleracaoM2 = throttle + pidX;


  // Segurança na aceleração:
  if (aceleracaoM1 > 1500) {      //
    aceleracaoM1 = 1500;
  }
  if (aceleracaoM2 > 1500) {      //
    aceleracaoM2 = 1500;
  }

  // Escrever correção
  motor1.writeMicroseconds(aceleracaoM1);
  motor2.writeMicroseconds(aceleracaoM2);


  // Print Data
  Serial.print("Angulo:");
  Serial.print(kalAngleX);
  //    Serial.print("\t");
  //    Serial.print("\t");

  //    Serial.print("Erro: ");
  //    Serial.print(erroX);
  //    Serial.print("\t");
  //    Serial.print("\tn");

  Serial.print("P: ");
  Serial.print(leitura_P);
  //    Serial.print("\t");
  //    Serial.print("\t");

  Serial.print("I: ");
  Serial.print(leitura_I);
  //    Serial.print("\t");
  //    Serial.print("\t");

  Serial.print("D: ");
  Serial.print(leitura_D);
  Serial.println("\t");
  //    Serial.print("\t");
  //    Serial.println("  ");

  //    Serial.print("PID: ");
  //    Serial.print(pidX);
  //    Serial.print("\t");
  //    Serial.println("\t");

  //  Serial.print("Esq M1: "); Serial.print(aceleracaoM1); Serial.print("\t");
  //  Serial.print("\t");
  //  Serial.print("Dir M2: "); Serial.print(aceleracaoM2); Serial.println("\t");


  erroAnteriorX = erroX;
} // Fim do Loop



/*Método Chamado no Setup() para ajustar comunicação
  do Protocolo I2C e modos de leitura do MPU 6050.
  Verifica se a comunicação foi feita com sucesso*/
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
    //Serial.print(F("Error reading sensor"));
    while (1);
  }
  delay(100); // Wait for sensor to stabilize
}


void initMotores() {
  //Valor inicial para acionamento dos motores
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
}
