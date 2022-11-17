#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define CE_PIN 9
#define CSN_PIN 10

#define LED_delay 150

RF24 radio(CE_PIN, CSN_PIN);

uint8_t addressTX[] = "1rPON";
uint8_t addressRX[] = "0rPON";

uint8_t payload = 0;

void setup() {
  // put your setup code here, to run once:

  pinMode(A5, OUTPUT);

  if (!radio.begin()) {
    while (1) {
      digitalWrite(A5, HIGH);
      delay(50);
      digitalWrite(A5, LOW);
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
}
bool report = 0;
void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(A5, HIGH);
  payload = 1;
  report = radio.write(&payload, sizeof(uint8_t));  // transmit & save the report
  delay(LED_delay);
  digitalWrite(A5, LOW);
  payload = 2;
  report = radio.write(&payload, sizeof(uint8_t));  // transmit & save the report
  delay(LED_delay);
  payload = 3;
  report = radio.write(&payload, sizeof(uint8_t));  // transmit & save the report
  delay(LED_delay);
}
