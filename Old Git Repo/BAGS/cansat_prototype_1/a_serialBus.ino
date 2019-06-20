

void SERCOM4_Handler()
{
  zigbee.IrqHandler();
}

void initSerial() {
  //Serial1 is initialised by default

  //zigbee
  pinPeripheral(UART1_RX, PIO_SERCOM_ALT);
  pinPeripheral(UART1_TX, PIO_SERCOM_ALT);

  //I2C
  pinPeripheral(SDA, PIO_SERCOM_ALT);
  pinPeripheral(SCL, PIO_SERCOM_ALT);

  //SPI is initialised by default
}
