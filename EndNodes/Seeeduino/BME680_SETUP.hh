#ifndef BME680_SETUP_HH
#define BME680_SETUP_HH

void setupBME680(BME680_Class *BME680) {
    while (!BME680->begin(I2C_STANDARD_MODE)) {  
        delay(5000);
    }
    BME680->setOversampling(TemperatureSensor, Oversample16);  // Use enumerated type values
    BME680->setOversampling(HumiditySensor, Oversample16);     
    BME680->setOversampling(PressureSensor, Oversample16);
    BME680->setIIRFilter(IIR4);  
    BME680->setGas(320, 150);
}

#endif
