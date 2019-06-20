#include <SPI.h>

#include "RF24.h"

//SCK -> 13//MISO -> 12//MOSI -> 11//CSN -> 7//CE -> 8

RF24 radio(7,8);

const uint64_t pipe = 0xE8E8F0F0E1LL;

int msg[1];

void setup()

{
radio.begin();

radio.openReadingPipe(1,pipe);

radio.startListening();

Serial.begin(9600);
}
void loop()
{
if (radio.available())
{
bool done = false;
while (!done)
{
  radio.read(msg, 1);
  Serial.println(msg[0]);
}
}
}
