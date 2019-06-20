#include "LowPower.h"

// Pin 0 used for external interrupt
const int pin = 0;
bool state = true;
void setup()
{
  attachInterrupt(pin, blink, LOW);
  LowPower.standby();
}

void loop() 
{
  detachInterrupt(pin);
}

void blink(void)
{
 // Serial printing code   
}
