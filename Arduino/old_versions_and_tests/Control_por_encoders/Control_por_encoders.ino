#include <FastPID.h>

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
#define MotorB_EncA A2
#define MotorB_EncB A3

#define MotorSTBY	0	// Deshabilitado

// 30 unidades por vuelta, diametro de llanta 66.5mm
int16_t EncMA_count = 0;
int16_t EncMB_count = 0;

int16_t EncMA_countSetpoint = 0;
int16_t EncMB_countSetpoint = 0;

bool EncMA_lastState = 0;
bool EncMB_lastState = 0;

float kp=6.0, ki=0.5, kd=0.0, hz=30.0;

FastPID PIDMA (kp, ki, kd, hz, 16, true);
FastPID PIDMB (kp, ki, kd, hz, 16, true);

void inline encProcess();

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

  EncMA_lastState = digitalRead(MotorA_EncA);
  EncMB_lastState = digitalRead(MotorB_EncA);

  Serial.begin(115200);

  PIDMA.clear();
  PIDMB.clear();
  PIDMA.setOutputRange(-128, 128);
  PIDMB.setOutputRange(-128, 128);

  if (PIDMA.err())
  {
    Serial.println("PIDMA: Conf Error");
    while(1) {}
  }
}

const uint32_t periodPID = 1000000 / hz;
uint32_t lastTimePID = 0;
void loop()
{
  encProcess();

  EncMA_countSetpoint = 60;
  EncMB_countSetpoint = 60;

  if((micros() - lastTimePID) > periodPID)
  {
    int16_t velA;
    int16_t velB;

    lastTimePID = micros();
    
    int16_t errorA = EncMA_count - EncMA_countSetpoint;
    int16_t errorB = EncMB_count - EncMB_countSetpoint;
    // if (abs(errorA) < 1)
    if (errorA == 0)
    {
      PIDMA.clear();
      velA = 0;
    }
    else
    {
      velA = PIDMA.step(EncMA_countSetpoint, EncMA_count);
    }
    // if (abs(errorB) < 1)
    if(errorB == 0)
    {
      PIDMB.clear();
      velB = 0;
    }
    else
    {
      velB = PIDMB.step(EncMB_countSetpoint, EncMB_count);
    }

    motorControl(velA, velB);

    Serial.print("encA: ");
    Serial.print(EncMA_count);
    Serial.print("\t");
    Serial.print("velA: ");
    Serial.println(velA);
    Serial.print("encB: ");
    Serial.print(EncMB_count);
    Serial.print("\t");
    Serial.print("velB: ");
    Serial.println(velB);
  }
}

void inline encProcess()
{
  bool EncMA_state = digitalRead(MotorA_EncA);
  bool EncMB_state = digitalRead(MotorB_EncA);

  if (EncMA_state != EncMA_lastState)
  {
    if (digitalRead(MotorA_EncB) != EncMA_lastState)
    {
      EncMA_count++;
    }
    else
    {
      EncMA_count--;
    }
    Serial.print("EncMA: ");
    Serial.println(EncMA_count);
  }

  if (EncMB_state != EncMB_lastState)
  {
    if (digitalRead(MotorB_EncB) != EncMB_lastState)
    {
      EncMB_count--;
    }
    else
    {
      EncMB_count++;
    }
    Serial.print("EncMB: ");
    Serial.println(EncMB_count);
  }

  EncMA_lastState = EncMA_state;
  EncMB_lastState = EncMB_state;
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