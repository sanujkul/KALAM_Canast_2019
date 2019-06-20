#include <SPI.h>

#include <RF24.h>

int msg[1]; //SCK -> 13//MISO -> 12//MOSI -> 11//CSN -> 7//CE -> 8

RF24 radio(7, 8);

const uint64_t pipe = 0xE8E8F0F0E1LL;

int potpin = 0;

int val;

void setup(void)

{

  radio.begin();

  radio.openWritingPipe(pipe);

}

void loop(void)

{

  val = 0;

  msg[0] = val;

  radio.write(msg, 1);

}
