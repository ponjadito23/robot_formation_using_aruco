#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define CE_PIN 9
#define CSN_PIN 10

#define LED A5

RF24 radio(CE_PIN, CSN_PIN);

uint8_t addressRX[] = "1rPON";
//uint8_t addressTX[] = "0rPON";

uint8_t payload = 0;

void setup() {
  // put your setup code here, to run once:

  pinMode(LED, OUTPUT);

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

bool report = 0;
void loop() {
  uint8_t pipe;
  if (radio.available(&pipe))
  {
    uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
    radio.read(&payload, bytes);             // fetch payload from FIFO

    if (payload == 3)
    {
        digitalWrite(LED, HIGH);
    }
    else
    {
        digitalWrite(LED, LOW);
    }
  }
}
