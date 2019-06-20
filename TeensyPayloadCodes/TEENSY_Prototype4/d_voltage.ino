#define R1 20000
#define R2 10000
#define ADC_MAX 4095 //12 bit ADC
#define VCC 3.3 //Max voltage read by SAMD21
#define VOLTAGE_DIV (R1+R2)/R2 //Factor for calculating battery voltage


void initBatteryVoltage() {
  // initialize battery voltage function
  pinMode(voltagePin,INPUT);
  analogReadResolution(12); // Set analog input resolution to max, 12-bits
}

float getBatteryVoltage() {
  int voltageDigital = analogRead(voltagePin);
  float voltageBattery = (voltageDigital)*(VCC/ADC_MAX)*VOLTAGE_DIV;
  return voltageBattery;
}
