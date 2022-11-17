#include <SPI.h>
#include "printf.h"
#include "RF24.h"

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

// #define MotorA_EncA A0
// #define MotorA_EncB A1
// #define MotorB_EncA A2
// #define MotorB_EncB A3

#define MotorSTBY	0	// Deshabilitado

RF24 radio(CE_PIN, CSN_PIN);

const uint16_t maxTimeBetweenPackets = 500;   // Tiempo (ms) maximo entre paquetes antes de detener el robot
const uint8_t addressRX[] = "1rPON";
//uint8_t addressTX[] = "0rPON";
const uint8_t magicPacketStartKey = 'R';
const uint8_t idRobot = 3;      // Cambialo de acuerdo al robot

uint8_t payload[6];

void setup() {
  pinMode(LED, OUTPUT);

  pinMode(MotorA_PWM, OUTPUT);
  pinMode(MotorB_PWM, OUTPUT);

  pinMode(MotorA_Fwd, OUTPUT);
  pinMode(MotorA_Bkw, OUTPUT);
  pinMode(MotorB_Fwd, OUTPUT);
  pinMode(MotorB_Bkw, OUTPUT);

  if (!radio.begin()) {
    while (1) {
      digitalWrite(LED, HIGH);
      delay(50);
      digitalWrite(LED, LOW);
      delay(50);
    }  // hold in infinite loop
  }

  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

  // save on transmission time by setting the radio to only transmit the
  // number of bytes we need to transmit
  radio.setPayloadSize(sizeof(payload));

  //radio.openWritingPipe(addressTX);
  radio.openReadingPipe(1, addressRX);

  //radio.stopListening();  // put radio in TX mode
  radio.startListening();  // put radio in RX mode
}

uint32_t lastPacketTime = 0;
int16_t speed = 0;
int16_t steer = 0;
void loop() {
  uint8_t pipe;
  if (radio.available(&pipe))
  {
    uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
    radio.read(&payload, bytes);             // fetch payload from FIFO

    if (bytes == 6)
    {
      if (payload[0] == magicPacketStartKey && payload[1] == idRobot)    
      {
        speed = payload[2] | (payload[3] << 8);
        steer = payload[4] | (payload[5] << 8);

        lastPacketTime = millis();
      }
    }
  }

  if ((millis() - lastPacketTime) > maxTimeBetweenPackets)
  {
    speed = 0;
    steer = 0;
  }

  controlMotores(speed, steer);
}

// Funcion para controlar la direccion y velocidad de los motores
// la variable de direccion acepta valores desde -255 a 255 y la de velocidad de -255 a 255
void controlMotores(int16_t velocidad, int16_t direccion)
{
	int16_t velocidadMotorA = velocidad + direccion;
	int16_t velocidadMotorB = velocidad - direccion;

	if(velocidadMotorA < 0)
	{
		velocidadMotorA = -velocidadMotorA;
		//velocidadMotorA = 0;
		digitalWrite(MotorA_Fwd, LOW);
		digitalWrite(MotorA_Bkw, HIGH);
	}
	else
	{
		digitalWrite(MotorA_Fwd, HIGH);
		digitalWrite(MotorA_Bkw, LOW);
	}
	
	if(velocidadMotorB < 0)
	{
		velocidadMotorB = -velocidadMotorB;
		//velocidadMotorB = 0;
		digitalWrite(MotorB_Fwd, LOW);
		digitalWrite(MotorB_Bkw, HIGH);
	}
	else
	{
		digitalWrite(MotorB_Fwd, HIGH);
		digitalWrite(MotorB_Bkw, LOW);
	}
	
	if(velocidadMotorA > 255) velocidadMotorA = 255;
	if(velocidadMotorB > 255) velocidadMotorB = 255;
	
	analogWrite(MotorA_PWM, velocidadMotorA);
	analogWrite(MotorB_PWM, velocidadMotorB);
}

void apagarMotores()
{
	analogWrite(MotorA_PWM, 0);
	analogWrite(MotorB_PWM, 0);
}

// void enviarUInt(uint16_t &entero)
// {
// 	char enteroLow = entero & 0xff;
// 	char enteroHigh = (entero >> 8) & 0xff;
	
// 	Serial1.write(enteroLow);
// 	Serial1.write(enteroHigh);
// }

// void enviarInt(int16_t &entero)
// {
// 	char enteroLow = entero & 0xff;
// 	char enteroHigh = (entero >> 8) & 0xff;
	
// 	Serial1.write(enteroLow);
// 	Serial1.write(enteroHigh);
// }

// // Control de los motores por el puerto serial, recibe datos de tipo "DX___,Y___"
// // para la direccion (solo eje X), "VX___,Y___" para la velocidad (solo eje Y)
// // y un comando para desactivar el controlManual
// void controlManual()
// {
// 	int32_t direccion = 0;
// 	int32_t velocidad = 0;
	
// 	uint32_t tiempoPrevio1 = 0;
	
// 	const uint16_t tiempoLimite1 = 1000;		// Para evitar perder el control del seguidor en caso de desconexion
	
// 	while(flagManualControl == 1)
// 	{
// 		while(Serial1.available() > 0)
// 		{
// 			tiempoPrevio1 = millis();
// 			switch(Serial1.read())
// 			{
// 				case 'D':
// 					direccion = Serial1.parseInt();
// 					Serial1.parseInt();
// 					break;
// 				case 'V':
// 					Serial1.parseInt();
// 					velocidad = Serial1.parseInt();
// 					break;
// 				case '6':
// 					switch(Serial1.parseInt())
// 					{
// 						case 9:
// 							flagManualControl = Serial1.parseInt();
// 							serialFlush();
// 							break;
// 						case 6:
// 							flagPrintMotors = Serial1.parseInt();
// 							serialFlush();
// 							break;
// 					}
// 				default:
// 					serialFlush();
// 			}
// 		}
		
// 		if((millis() - tiempoPrevio1) > tiempoLimite1)
// 		{
// 			apagarMotores();
// 		}
// 		else
// 		{
// 			controlMotores(direccion, velocidad);
// 		}
// 	}
// 	apagarMotores();
// 	serialFlush();
// }