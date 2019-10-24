#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
unsigned long previousMillis,dt;
MPU6050 mpu;
//defini apenas para teste a leitura dos eixos
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2
// MPU controles e status de variaveis
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

//variaveis de projeto
int angYaw = 0;
int angRoll = 0;
int angPitch = 0;
float error, p,i,d,pid;
float kp,ki,kd;
//motores de propulsão:motor1 e 2;motores de submersão 3 e 4;
int motor1f = 3, motor1r = 5, motor2f = 6, motor2r = 9, motor3=10, motor4 = 11;
// variaveis de movimento e orientação


Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float pitch,roll,yaw;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = true;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    //inicia o i2c, pois o mesmo não é instanciado automaticamente
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. 
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(115200);
    //inicia o dispositivo.
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    
    //inicialização
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    //carrega as config
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    //seta os offset do sensor(mudança necessaria)
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

        if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void loop() {
//-----------------------------------------------------------------
//-----------------testes com serial ------------------------------
//-----------------------------------------------------------------
//a partir de um comando no serial será executado a rotina de movimentação do submarino
  if(Serial.available())
  {
    switch(Serial.read())
    {
      
      case 'ff':
        ff();
      break;
      
      case 'fe':
        fe();
      break;
      
      case 'fd':
        fd();
      break;

//      case 'rr':
//        rr();
//      break;
      default:
        //não implementado
        break;
    }
    receberParametros();
  }
//de acordo com o comando será executado uma rotina diferente.
//Ex: para frente os dois motores vão ter um pwm igual, se for dif o yaw ele vai corrigir um motor
//falta adicionar o pid para trabalhar com os dados
//falta adicionar os controles de direção que conterão os setpoints respectivos da movimentação
//serão implementados quatro rotinas(frente-esquerda{fe},frente-direita{fd},frente{ff},ré{rr})
//as rotinas direita,esquerda,esquerda-ré e direta-ré serão implementadas posteriormente.
//implementar a rotina de segurança para superaquecimento do sistema.
//falta adicionar o filtro complementar, para isso foi implementado o millis que será o dt(verificar se há a necessidade)
}


void receberParametros(){
     if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
    }

    mpuInterrupt = true;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
  if(fifoCount < packetSize){
          //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
      // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  }
    // check for overflow (this should never happen unless our code is too inefficient)
    else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {

        // read a packet from FIFO
  while(fifoCount >= packetSize){ // Lets catch up to NOW, someone is using the dreaded delay()!
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
  }

  }

unsigned long currentMillis = millis(); //VARIÁVEL RECEBE O TEMPO ATUAL EM MILISSEGUNDOS
//[0] é yaw,[1] é pitch e [2] é roll.
  if (currentMillis - previousMillis > 500) {
    //long dt = (currentMillis - previousMillis)/1000;
    previousMillis = currentMillis;
                // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yaw = ypr[0] * (180/M_PI);
            pitch = ypr[1] * (180/M_PI);
            roll = ypr[2] * (180/M_PI);
            Serial.print("ypr\t");
            Serial.print(yaw);
            Serial.print("\t");
            Serial.print(pitch);
            Serial.print("\t");
            Serial.println(roll);
  }
} 
void ff(){
  //a função ff tem como objetivo manter os dois motores de propulsão e mergulho no mesmo angulo que já estavam.
  //monitorar se o botao ainda está pressionado
  //se sim vou ver se o angulo de yaw e roll não se pitch não se alteraram
  while(Serial.read() == 'ff')
  {
    receberParametros();
    if(angYaw != yaw) // propulsão
    {
      //vai ver qual motor precisa girar mais que o outro   
      if(yaw > 0)
      {
          error = yaw - angYaw;
          p = kp*error;
          i = ki * error*dt;
      }
      if(yaw < 0)
      {
          //aumentar o da esquerda
      }
    }
    if(angPitch != pitch) // submersão
    {
      //vai ver qual motor precisa girar mais que o outro   
      if(pitch > 0)
      {
          //aumentar os dois de submersão da esquerda
      }
      if(pitch < 0)
      {
          //diminuir o da esquerda
      }
    }
  }
}

void fe(){
    //vou monitorar se o botão ainda não mudou;
    //vou ter um roll de 10º
    //vou variando o yaw
    //o pitch irei manter constante
    //para variar o yaw irei precisar girar mais o propulsor direito e quando eu parar vou salvar o angyaw = yaw que será minha referência;
    while(Serial.read() == 'fe')
      {
        if(roll < 15){
          //aumentar a rotação do motor direito
          //aqui vai ter pid
        }
        //colocar uma tensão de 8v no motor esquerdo e 10 no direito até soltar o botão
        //aqui não vai ter pid porque não terei referencia.
      }
} 
void fd(){
    //vou monitorar se o botão ainda não mudou;
    //vou ter um roll de 10º
    //vou variando o yaw
    //o pitch irei manter constante
    //para variar o yaw irei precisar girar mais o propulsor direito e quando eu parar vou salvar o angyaw = yaw que será minha referência;
    while(Serial.read() == 'fe')
      {
        if(roll < 15){
          //aumentar a rotação do motor esquerdo
          //aqui vai ter pid
        }
        //colocar uma tensão de 8v no motor direito e 10 no esquerdo até soltar o botão
        //aqui não vai ter pid porque não terei referencia.
      }
} 

void rr(){
  //a função ff tem como objetivo manter os dois motores de propulsão e mergulho no mesmo angulo que já estavam.
  //monitorar se o botao ainda está pressionado
  //se sim vou ver se o angulo de yaw e roll não se pitch não se alteraram
  while(Serial.read() == 'rr')
  {
    receberParametros();
    if(angYaw != yaw) // propulsão
    {
      //coloca um pwm igual nos dois motores.
      //vai ver qual motor precisa girar mais que o outro   
      if(yaw > 0)
      {
          //diminuir o da esquerda
      }
      if(yaw < 0)
      {
          //aumentar o da esquerda
      }
    }
    if(angPitch != pitch) // submersão
    {
      //coloca uma propulsão igual  
      //vai ver qual motor precisa girar mais que o outro   
      if(pitch > 0)
      {
          //aumentar os dois de submersão da esquerda
      }
      if(pitch < 0)
      {
          //diminuir o da esquerda
      }
    }
  }
}
