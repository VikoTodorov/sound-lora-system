#ifndef BME680_SETUP_HH
#define BME680_SETUP_HH

void setupBME680(Adafruit_BME680 *bme680) {
    while (!bme680->begin(0x76)) {  
        delay(5000);
    }
    bme680->setTemperatureOversampling(BME680_OS_4X);
    bme680->setHumidityOversampling(BME680_OS_2X);
    bme680->setPressureOversampling(BME680_OS_4X);
    bme680->setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme680->setGasHeater(320, 150); // 320*C for 150 ms
}

#endif
