#include <FastPID.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#define CE_PIN 9
#define CSN_PIN 10

#define MISO_PIN 12
#define MOSI_PIN 11
#define SCK_PIN 13

#define LED A5

#define MotorA_PWM 	5
#define MotorB_PWM 	6

#define MotorA_Fwd 	3
#define MotorA_Bkw 	4
#define MotorB_Fwd 	7
#define MotorB_Bkw 	8

#define MotorA_EncA A0
#define MotorA_EncB A1
#define MotorB_EncA A3
#define MotorB_EncB A2

#define MotorSTBY	0	// Deshabilitado

// 60 unidades por vuelta, diametro de llanta 66.5mm
Encoder EncMA(MotorA_EncA, MotorA_EncB);
Encoder EncMB(MotorB_EncA, MotorB_EncB);

int32_t EncMA_count = 0;
int32_t EncMB_count = 0;

int32_t EncMA_prevCount = 0;
int32_t EncMB_prevCount = 0;

int32_t EncMA_countSetpoint = 0;
int32_t EncMB_countSetpoint = 0;

float hz=60.0;
float kp=2.0, ki=0.05, kd=0.1;
float kpS=0.04, kiS=0.1, kdS=0.0;

FastPID PIDposMA (kp, ki, kd, hz, 16, true);
FastPID PIDposMB (kp, ki, kd, hz, 16, true);

FastPID PIDspdMA (kpS, kiS, kdS, hz, 16, true);
FastPID PIDspdMB (kpS, kiS, kdS, hz, 16, true);

void inline encProcess();
void inline controlProcess();
void inline spdCalcProcess();
void inline PIDPosProcess();
void inline PIDSpdProcess();

void setup()
{
  pinMode(LED, OUTPUT);

  pinMode(MotorA_PWM, OUTPUT);
  pinMode(MotorB_PWM, OUTPUT);

  pinMode(MotorA_Fwd, OUTPUT);
  pinMode(MotorA_Bkw, OUTPUT);
  pinMode(MotorB_Fwd, OUTPUT);
  pinMode(MotorB_Bkw, OUTPUT);

  pinMode(MotorA_EncA, INPUT);
  pinMode(MotorA_EncB, INPUT);
  pinMode(MotorB_EncA, INPUT);
  pinMode(MotorB_EncB, INPUT);

  EncMA.read();
  EncMB.read();

  Serial.begin(115200);

  PIDposMA.clear();
  PIDposMB.clear();
  PIDspdMA.clear();
  PIDspdMB.clear();
  PIDposMA.setOutputRange(-128, 128);
  PIDposMB.setOutputRange(-128, 128);
  PIDspdMA.setOutputRange(-25, 25);
  PIDspdMB.setOutputRange(-25, 25);

  if (PIDposMA.err())
  {
    Serial.println("PIDposMA: Conf Error");
    while(1) {}
  }
  if (PIDspdMA.err())
  {
    Serial.println("PIDspdMA: Conf Error");
    while(1) {}
  }
}

void loop()
{
  EncMA_countSetpoint = 250;
  EncMB_countSetpoint = -250;

  encProcess();
  controlProcess();
  // motorControl(128, 128);
}

uint32_t lastTimePulseA = 0;
uint32_t lastTimePulseB = 0;
int32_t deltaArrayA[3];
uint8_t indexArrayA = 0;
int32_t deltaArrayB[3];
uint8_t indexArrayB = 0;
void inline encProcess()
{
  EncMA_count = EncMA.read();
  EncMB_count = EncMB.read();
  uint32_t currTimePulseA = micros();
  uint32_t currTimePulseB = micros();

  // Calculo para Encoder A
  if (EncMA_count != EncMA_prevCount)
  {
    uint32_t deltaTimeA = currTimePulseA - lastTimePulseA;
    if (deltaTimeA > 1000)
    {
      if (EncMA_count > EncMA_prevCount)
      {
        deltaArrayA[indexArrayA] = deltaTimeA;
      }
      else
      {
        deltaArrayA[indexArrayA] = -(deltaTimeA);
      }
      indexArrayA++;
      if (indexArrayA > (sizeof(deltaArrayA) / sizeof(uint32_t))) indexArrayA = 0;
    }
    EncMA_prevCount = EncMA_count;
    lastTimePulseA = currTimePulseA;
  }

  // Calculo para Encoder B
  if (EncMB_count != EncMB_prevCount)
  {
    uint32_t deltaTimeB = currTimePulseB - lastTimePulseB;
    if (deltaTimeB > 1000)
    {
      if (EncMB_count > EncMB_prevCount)
      {
        deltaArrayB[indexArrayA] = deltaTimeB;
      }
      else
      {
        deltaArrayB[indexArrayB] = -(deltaTimeB);
      }
      indexArrayB++;
      if (indexArrayB > (sizeof(deltaArrayB) / sizeof(uint32_t))) indexArrayB = 0;
    }
    EncMB_prevCount = EncMB_count;
    lastTimePulseB = currTimePulseB;
  }
}

const uint32_t periodControl = 1000000 / hz;
uint32_t lastTimeControl = 0;
int16_t spdSetpointA = 0;
int16_t spdSetpointB = 0;
int16_t avgSpeedA = 0;
int16_t avgSpeedB = 0;
int16_t motorSpdA = 0;
int16_t motorSpdB = 0;
void inline controlProcess()
{
  if((micros() - lastTimeControl) > periodControl)
  {
    lastTimeControl = micros();

    spdCalcProcess();
    PIDPosProcess();
    // spdSetpointA = 100;
    // spdSetpointB = 100;
    // motorControl(spdSetpointA, spdSetpointB);
    PIDSpdProcess();
  }
}

const uint32_t periodStop = 50000;
void inline spdCalcProcess()
{
  // uint32_t benchPrevTime = micros();

  uint32_t currTimeCalc = lastTimeControl;

  // Serial.println(currTimeCalc - lastTimePulseA);
  // Calculo para Encoder A
  if((currTimeCalc - lastTimePulseA) > periodStop)
  {
    for (uint8_t i = 0; i < (sizeof(deltaArrayA) / sizeof(uint32_t)); i++)
    {
      deltaArrayA[i] = 0;
    }
    avgSpeedA = 0;
  }
  else
  {
    int32_t avgTimeA = 0;
    for (uint8_t i = 0; i < (sizeof(deltaArrayA) / sizeof(uint32_t)); i++)
    {
      avgTimeA += deltaArrayA[i];
    }
    avgSpeedA = 3145728 / (avgTimeA | 1);
  }

  // Calculo para Encoder B
  if((currTimeCalc - lastTimePulseB) > periodStop)
  {
    for (uint8_t i = 0; i < (sizeof(deltaArrayB) / sizeof(uint32_t)); i++)
    {
      deltaArrayB[i] = 0;
    }
    avgSpeedB = 0;
  }
  else
  {
    int32_t avgTimeB = 0;
    for (uint8_t i = 0; i < (sizeof(deltaArrayB) / sizeof(uint32_t)); i++)
    {
      avgTimeB += deltaArrayB[i];
    }
    avgSpeedB = 3145728 / (avgTimeB | 1);
  }
  
  Serial.print("spdA: ");
  Serial.print(avgSpeedA);
  Serial.print("\t");
  Serial.print("spdB: ");
  Serial.println(avgSpeedB);


  // uint32_t benchCurrTime = micros();
  // Serial.println(benchCurrTime - benchPrevTime);
  // benchPrevTime = benchCurrTime;
}

void inline PIDPosProcess()
{
  int16_t errorA = EncMA_count - EncMA_countSetpoint;
  int16_t errorB = EncMB_count - EncMB_countSetpoint;

  if (abs(errorA) < 1)
  // if (errorA == 0)
  {
    PIDposMA.clear();
    spdSetpointA = 0;
    motorSpdA = 0;
  }
  else
  {
    spdSetpointA = PIDposMA.step(EncMA_countSetpoint, EncMA_count);
  }

  if (abs(errorB) < 1)
  // if(errorB == 0)
  {
    PIDposMB.clear();
    spdSetpointB = 0;
    motorSpdB = 0;
  }
  else
  {
    spdSetpointB = PIDposMB.step(EncMB_countSetpoint, EncMB_count);
  }

  // Serial.print("encA: ");
  // Serial.print(EncMA_count);
  // Serial.print("\t");
  // Serial.print("spdSetpointA: ");
  // Serial.println(spdSetpointA);
  // Serial.print("encB: ");
  // Serial.print(EncMB_count);
  // Serial.print("\t");
  // Serial.print("spdSetpointB: ");
  // Serial.println(spdSetpointB);
}

const int16_t motorMaxSpd = 150;
void inline PIDSpdProcess()
{
  int16_t accelA = PIDspdMA.step(spdSetpointA, avgSpeedA);
  int16_t accelB = PIDspdMB.step(spdSetpointB, avgSpeedB);

  // motorSpdA = spdSetpointA + accelA;
  // motorSpdB = spdSetpointB + accelB;

  motorSpdA += accelA;
  motorSpdB += accelB;

  if (motorSpdA > motorMaxSpd) motorSpdA = motorMaxSpd;
  else if (motorSpdA < -motorMaxSpd) motorSpdA = -motorMaxSpd;

  if (motorSpdB > motorMaxSpd) motorSpdB = motorMaxSpd;
  else if (motorSpdB < -motorMaxSpd) motorSpdB = -motorMaxSpd;

  motorControl(motorSpdA, motorSpdB);
}

void motorControl(int16_t velA, int16_t velB)
{
  uint8_t pwmA;
  uint8_t pwmB;

  if (velA < 0)
  {
    pwmA = -velA;
    digitalWrite(MotorA_Fwd, LOW);
    digitalWrite(MotorA_Bkw, HIGH);
  }
  else
  {
    pwmA = velA;
    digitalWrite(MotorA_Fwd, HIGH);
    digitalWrite(MotorA_Bkw, LOW);
  }

  if (velB < 0)
  {
    pwmB = -velB;
    digitalWrite(MotorB_Fwd, LOW);
    digitalWrite(MotorB_Bkw, HIGH);
  }
  else
  {
    pwmB = velB;
    digitalWrite(MotorB_Fwd, HIGH);
    digitalWrite(MotorB_Bkw, LOW);
  }

  analogWrite(MotorA_PWM, pwmA);
  analogWrite(MotorB_PWM, pwmB);
}