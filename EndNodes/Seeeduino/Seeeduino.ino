#include <LoRaWan.h>
#include "lora_secrets.h" 

#include "Adafruit_ZeroFFT.h"
#include "mic.hh"

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "BME680_SETUP.hh"

#define sampleRate 12500     //sample rate of ADC
#define dataSize 1024        //used to set number of samples
#define dataHalfSize 512                
#define gClk 3               //used to define which generic clock we will use for ADC
#define intPri 0             //used to set interrupt priority for ADC
#define cDiv 1               //divide factor for generic clock

char buffer[256]; // lora init buffer

volatile int16_t aDCVal[dataSize];       //array to hold ADC samples
volatile uint16_t sampleCounter = 0;  //tracks how many samples we have collected
uint8_t countFFT = 0;
uint16_t indexFFT = 0;
typedef struct {
    int32_t frequency;
    int32_t amplitude;
} soundMeasurement_t;
soundMeasurement_t soundMeasurement = {0, 0};
soundMeasurement_t prevSoundMeasurement = {0, 0};

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme680; // I2C
typedef struct {
    int32_t temp;
    int32_t humidity;
    int32_t pressure;
    int32_t gas;
} bmeMeasurement_t;
bmeMeasurement_t bmeMeasurement = {0, 0, 0, 0};
bmeMeasurement_t prevBmeMeasurement = {0, 0, 0, 0};
bool firstLoop = true;   // skip it because is incorrect
bool secondLoop = true;  // start the time checking
long prevBMEmeasurement = 0;

unsigned char packetBuffer[48]; 
bool tempFlag = false;
bool humFlag = false;
bool pressFlag = false;
bool gasFlag = false;
bool freqFlag = false;
bool ampFlag = false;
bool testFlag = false;

uint8_t TEMP_HEAD = 6;
uint8_t HUM_HEAD = 5;
uint8_t PRESS_HEAD = 4;
uint8_t GAS_HEAD = 3;
uint8_t FREQ_HEAD = 2;
uint8_t AMP_HEAD = 1;
uint8_t TEST_HEAD = 0;

//bool result = false;

bool newMessage = false;
char str[254];

long prevMessage = 0; // used for tests

void ADC_Handler();
void setup(void) {     
 
    SerialUSB.begin(115200);
    while(!SerialUSB);
 
    lora.init();
 
    memset(buffer, 0, 256);
    lora.getVersion(buffer, 256, 1);
    SerialUSB.print(buffer); 
 
    memset(buffer, 0, 256);
    lora.getId(buffer, 256, 1);
    SerialUSB.print(buffer);
 
    lora.setKey(NET_SESSION_KEY, APP_SESSION_KEY, NULL);
 
    lora.setDeciveMode(LWABP);
    lora.setDataRate(DR5, EU868);
 
    lora.setChannel(0, 868.1);
    lora.setChannel(1, 868.1);
    lora.setChannel(2, 868.1);
 
    lora.setReceiceWindowFirst(0, 868.1);
    lora.setReceiceWindowSecond(869.5, DR5);
 
    lora.setDutyCycle(false);
    lora.setJoinDutyCycle(false);
 
    lora.setPower(14);
    delay(1000);
    
    noInterrupts();
    clearRegisters();              // disable ADC and clear CTRLA and CTRLB
    portSetup();                   // setup the ports or pin to make ADC measurement
    genericClockSetup(gClk, cDiv); // setup generic clock and routed it to ADC
    aDCSetup();                    // set up registers for ADC, input argument sets ADC reference
    setUpInterrupt(intPri);        // sets up interrupt for ADC and argument assigns priority
    aDCSWTrigger();                // trigger ADC to start free run mode
    interrupts();
    
    setupBME680(&bme680);
}

uint16_t testCounter = 0;

void loop(void) {   
    uint8_t offset = 1;
    
    if (millis() - prevBMEmeasurement >= 60000 || firstLoop || secondLoop) {
        NVIC_DisableIRQ(ADC_IRQn); // stop ADC_IRQ to read from I2C
            if (bme680.performReading()) { 
                bmeMeasurement.temp = bme680.temperature * 100;
                bmeMeasurement.humidity = bme680.humidity * 100;
                bmeMeasurement.pressure = bme680.pressure;
                bmeMeasurement.gas = bme680.gas_resistance;
            }
        NVIC_EnableIRQ(ADC_IRQn);  // enable ADC_IRQ again
        if (!firstLoop) {
            if (prevBmeMeasurement.temp - bmeMeasurement.temp >= 100
                    || prevBmeMeasurement.temp - bmeMeasurement.temp <= -100) { // difference in 1 degree
                prevBmeMeasurement.temp = bmeMeasurement.temp;
                tempFlag = true;
            }
            if (prevBmeMeasurement.pressure - bmeMeasurement.pressure >= 1000
                    || prevBmeMeasurement.pressure - bmeMeasurement.pressure <= -1000) { // difference in 10 pa
                prevBmeMeasurement.pressure = bmeMeasurement.pressure;
                pressFlag = true;
            }
            if (prevBmeMeasurement.humidity - bmeMeasurement.humidity >= 1000 
                    || prevBmeMeasurement.humidity -bmeMeasurement.humidity <= -1000) { // difference in 1 %
                prevBmeMeasurement.humidity = bmeMeasurement.humidity;
                humFlag = true;
            }
            if (prevBmeMeasurement.gas - bmeMeasurement.gas >= 1000 
                    || prevBmeMeasurement.gas - bmeMeasurement.gas <= -1000) { 
                prevBmeMeasurement.gas = bmeMeasurement.gas;
                gasFlag = true;
            }
            if (secondLoop) {
                secondLoop = false;
            }
            prevBMEmeasurement = millis();
        }
        else {
            firstLoop = false;
        }
    }

    testFlag = true; 
    if (sampleCounter == dataSize) { 
        uint16_t amp = 0;
        NVIC_DisableIRQ(ADC_IRQn); // stop ADC_IRQ to fill values in array
        removeDCOffset(aDCVal, dataSize);
        for (int i=0; i < dataSize; i++){
            amp += abs(aDCVal[i]);
        }
        soundMeasurement.amplitude += amp / dataHalfSize;
        ZeroFFT((q15_t*)aDCVal, dataSize);
        for (int i=0; i < dataHalfSize; i++){
            if (aDCVal[i] > aDCVal[indexFFT]) {
                indexFFT = i;
            }
        }
        soundMeasurement.frequency += (uint32_t)FFT_BIN(indexFFT, sampleRate, dataSize);
        //SerialUSB.println(FFT_BIN(indexFFT, sampleRate, dataSize)); //showing not good behaviour
        //SerialUSB.println(soundMeasurement.frequency); // possible bug
        countFFT++;
        
        if (countFFT == 50) {
            soundMeasurement.amplitude /= countFFT;
            soundMeasurement.frequency /= countFFT;
            //SerialUSB.print(soundMeasurement.amplitude);
            //SerialUSB.print(" ");
            //SerialUSB.println(soundMeasurement.frequency); // fourier need corection :-(
            if (soundMeasurement.amplitude - prevSoundMeasurement.amplitude >= 10
                    || soundMeasurement.amplitude - prevSoundMeasurement.amplitude <= -10) {
                //SerialUSB.println(soundMeasurement.amplitude - prevSoundMeasurement.amplitude);
                prevSoundMeasurement.amplitude = soundMeasurement.amplitude;
                ampFlag = true;
            }
            if (soundMeasurement.frequency - prevSoundMeasurement.frequency >= 45
                    || soundMeasurement.frequency - prevSoundMeasurement.frequency <= -45) {
                //SerialUSB.println(soundMeasurement.frequency - prevSoundMeasurement.frequency);
                prevSoundMeasurement.frequency = soundMeasurement.frequency;
                freqFlag = true;
            }
            soundMeasurement.frequency = 0;
            soundMeasurement.amplitude = 0;
            countFFT = 0;
        }
        
        indexFFT = 0;
        sampleCounter = 0;
        NVIC_EnableIRQ(ADC_IRQn);  // enable ADC_IRQ again
    }
    
    if (tempFlag) {
        packetBuffer[0] |= (1 << TEMP_HEAD);
        memmove(packetBuffer+offset, &bmeMeasurement.temp, sizeof(bmeMeasurement.temp));
        tempFlag = false;
        offset += 4;
        newMessage = true;
        //SerialUSB.println("temp");
    }
    if (humFlag) {
        packetBuffer[0] |= (1 << HUM_HEAD);
        memmove(packetBuffer+offset, &bmeMeasurement.humidity, sizeof(bmeMeasurement.humidity));
        humFlag = false;
        offset += 4;
        newMessage = true;
        //SerialUSB.println("hum");
    }
    if (pressFlag) {
        packetBuffer[0] |= (1 << PRESS_HEAD);
        memmove(packetBuffer+offset, &bmeMeasurement.pressure, sizeof(bmeMeasurement.pressure));
        pressFlag = false;
        offset += 4;
        newMessage = true;
        //SerialUSB.println("press");
    }
    if (gasFlag) {
        packetBuffer[0] |= (1 << GAS_HEAD);
        memmove(packetBuffer+offset, &bmeMeasurement.gas, sizeof(bmeMeasurement.gas));
        gasFlag = false;
        offset += 4;
        newMessage = true;
        //SerialUSB.println("gas");
    }
    
    if (freqFlag) {
        packetBuffer[0] |= (1 << FREQ_HEAD);
        memmove(packetBuffer+offset, &prevSoundMeasurement.frequency, sizeof(prevSoundMeasurement.frequency));
        freqFlag = false;
        offset += 4;
        newMessage = true;
        //SerialUSB.println("freq");
    }
    if (ampFlag) {
        packetBuffer[0] |= (1 << AMP_HEAD);
        memmove(packetBuffer+offset, &prevSoundMeasurement.amplitude, sizeof(prevSoundMeasurement.amplitude));
        ampFlag = false;
        offset += 4;
        newMessage = true;
        //SerialUSB.println("amp");
    }
    if (testFlag) {
        packetBuffer[0] |= (1 << TEST_HEAD);
        memmove(packetBuffer+offset, &testCounter, sizeof(testCounter));
        testFlag = false;
        //SerialUSB.println("test");
    }

    bool result = false;
    //if (millis() - prevMessage >= 10000) {
    if (newMessage) { 
        snprintf((char*)str, 254, "t%d, h%d, p%d, g%d, f %d, a%d, c%d", 
                bmeMeasurement.temp, bmeMeasurement.humidity, bmeMeasurement.pressure, bmeMeasurement.gas, prevSoundMeasurement.frequency, prevSoundMeasurement.amplitude, testCounter);
        
        SerialUSB.println(str);
        newMessage = false;
        //prevMessage = millis();
        testCounter++;
        //NVIC_DisableIRQ(ADC_IRQn); // stop ADC_IRQ while sending messages
        //result = lora.transferPacket(packetBuffer, 100);
        //NVIC_EnableIRQ(ADC_IRQn);  // enable ADC_IRQ again
        memset(packetBuffer, 0, 48);
    }
 
   /* if(result)
    {
        short length;
        short rssi;
 
        memset(buffer, 0, 256);
        length = lora.receivePacket(buffer, 256, &rssi);
 
        if(length)
        {
            SerialUSB.print("Length is: ");
            SerialUSB.println(length);
            SerialUSB.print("RSSI is: ");
            SerialUSB.println(rssi);
            SerialUSB.print("Data is: ");
            for(unsigned char i = 0; i < length; i ++)
            {
                SerialUSB.print("0x");
                SerialUSB.print(buffer[i], HEX);
                SerialUSB.print(" ");
            }
            SerialUSB.println();
        }
    }*/
}

// This ISR is called each time ADC makes a reading
void ADC_Handler() {
    if(sampleCounter < dataSize) {
      aDCVal[sampleCounter] = REG_ADC_RESULT;
      sampleCounter++;
    }
    // Need to reset interrupt
    ADC->INTFLAG.reg = ADC_INTENSET_RESRDY;
}
