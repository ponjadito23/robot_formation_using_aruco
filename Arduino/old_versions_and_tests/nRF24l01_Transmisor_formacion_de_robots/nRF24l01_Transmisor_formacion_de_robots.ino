#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define CE_PIN 9
#define CSN_PIN 10

#define MISO_PIN 12
#define MOSI_PIN 11
#define SCK_PIN 13

#define LED A5

// #define LED_delay 150

RF24 radio(CE_PIN, CSN_PIN);

uint8_t addressTX[] = "1rPON";
uint8_t addressRX[] = "0rPON";
const uint8_t magicPacketStartKey = 'R';

uint8_t payload[6];

void setup() {
  pinMode(LED, OUTPUT);

  Serial.begin(230400);

  if (!radio.begin())
  {
    Serial.println("Error: Couldn't initialize radio");
    while (1)
    {
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

  radio.openWritingPipe(addressTX);
  //radio.openReadingPipe(1, addressRX)

  radio.stopListening();  // put radio in TX mode

  payload[0] = magicPacketStartKey;
}

bool report = 0;
void loop()
{
  // digitalWrite(LED, HIGH);
  // payload = 1;
  // report = radio.write(&payload, sizeof(uint8_t));  // transmit & save the report
  // delay(LED_delay);
  // digitalWrite(LED, LOW);
  // payload = 2;
  // report = radio.write(&payload, sizeof(uint8_t));  // transmit & save the report
  // delay(LED_delay);
  // payload = 3;
  // report = radio.write(&payload, sizeof(uint8_t));  // transmit & save the report
  // delay(LED_delay);

  if (Serial.available() >= sizeof(payload) && Serial.read() == magicPacketStartKey)
  {
    Serial.readBytes(&payload[1], sizeof(payload) - 1);

    report = radio.write(payload, sizeof(payload));
  }
}
