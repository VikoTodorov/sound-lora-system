#include "Zanshin_BME680.h"          // Include the BME680 Sensor library
#include <LoRaWan.h>
#include "lora_secrets.h" 
#include "Adafruit_ZeroFFT.h"
#include "BME680_SETUP.hh"
#include "mic.hh"

#define sRate 12500                   //sample rate of ADC

const uint16_t dSize = 1024;          //used to set number of samples
const uint16_t dHalfSize = 512;
const byte gClk = 3;                  //used to define which generic clock we will use for ADC
const byte intPri = 0;                //used to set interrupt priority for ADC
const uint8_t cDiv = 1;               //divide factor for generic clock

volatile int16_t aDCVal[dSize];       //array to hold ADC samples
volatile uint16_t sampleCounter = 0;  //tracks how many samples we have collected

uint8_t countFFT = 0;
uint16_t indexFFT = 0;
uint32_t frequency = 0;
uint32_t amplitude = 0;
uint32_t amp = 0; 
uint32_t freq = 0;

char buffer[256];

BME680_Class BME680;                           // Create an instance of the BME680 class
static int32_t temp, humidity, pressure, gas;  // BME readings
// used in trigger events logic
int32_t prevTemp = 0;   
int32_t prevPressure = 0;   
int32_t prevHumidity = 0;   
int32_t prevGas = 0;   

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

long prevMessage = 0;

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
    
    setupBME680(&BME680); 

    noInterrupts();
    clearRegisters();              // disable ADC and clear CTRLA and CTRLB
    portSetup();                   // setup the ports or pin to make ADC measurement
    genericClockSetup(gClk, cDiv); // setup generic clock and routed it to ADC
    aDCSetup();                    // set up registers for ADC, input argument sets ADC reference
    setUpInterrupt(intPri);        // sets up interrupt for ADC and argument assigns priority
    aDCSWTrigger();                // trigger ADC to start free run mode
    interrupts();
}

uint16_t testCounter = 0;

bool firstLoop = true;
bool secondLoop = true;
void loop(void) {   
    uint8_t offset = 1;
    
    if (millis() - prevBMEmeasurement >= 60000 || firstLoop || secondLoop) {
        NVIC_DisableIRQ(ADC_IRQn); // stop ADC_IRQ to read from I2C
        BME680.getSensorData(temp, humidity, pressure, gas);  // Get readings
        NVIC_EnableIRQ(ADC_IRQn);  // enable ADC_IRQ again
        if (!firstLoop) {
            if (prevTemp - temp >= 100 || prevTemp - temp <= -100) { // difference in 1 degree
                prevTemp = temp;
                tempFlag = true;
            }
            if (prevPressure - pressure >= 500 || prevPressure - pressure <= -500) { // difference in 10 pa
                prevPressure = pressure;
                pressFlag = true;
            }
            if (prevHumidity - humidity >= 1000 || prevHumidity - humidity <= -1000) { // difference in 1 %
                prevHumidity = humidity;
                humFlag = true;
            }
            if (prevGas - gas >= 6000 || prevGas - gas <= -6000) { 
                prevGas = gas;
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

    if (sampleCounter == dSize) { 
        uint32_t amp1 = 0;
        NVIC_DisableIRQ(ADC_IRQn);
        removeDCOffset(aDCVal, dSize);
        for (int i = 0; i < dSize; i++) {
           amp1 += abs(aDCVal[i]);
        }
        amp1 /= dHalfSize;
        
        ZeroFFT((int16_t*)aDCVal, dSize);
        for (int i=0; i < dHalfSize; i++){
            if (aDCVal[i] > aDCVal[indexFFT]) {
                indexFFT = i;
            }
        }

        amp += amp1;
        freq += FFT_BIN(indexFFT, sRate, dSize);
        countFFT++;
        
        if (countFFT == 50) {
            amplitude = amp;
            frequency = freq;
            amplitude /= 50;
            frequency /= 50;
            //SerialUSB.print(amplitude);
            //SerialUSB.print("amp  ");
            //SerialUSB.print(frequency);
            //SerialUSB.println("Hz"); 
            freq = 0;
            amp = 0;
            countFFT = 0;
        }
        
        indexFFT = 0;
        sampleCounter = 0;
        NVIC_EnableIRQ(ADC_IRQn); 
    }
    freqFlag = true;
    ampFlag = true;
    //testFlag = true; 
    /*if (countSamples == (dSize-1)) { 
        NVIC_DisableIRQ(ADC_IRQn); // stop ADC_IRQ to fill values in array
        for (int i=0; i < dSize; i++){
            amp += aDCVal[i];
        }
        amp /= dSize;
        removeDCOffset(aDCVal, dSize);
        ZeroFFT((int16_t*)aDCVal, dSize);
        for (int i=0; i < dHalfSize; i++){
            if (aDCVal[i] > aDCVal[indexFFT]) {
                indexFFT = i;
            }
        }
        freq += (int)FFT_BIN(indexFFT, sRate, dSize);
        // SerialUSB.println(FFT_BIN(indexFFT, sRate, dSize)); //showing not good behaviour
        // SerialUSB.println(freq); // possible bug
        countFFT++;
        
        if (countFFT == 100) {
            amp /= countFFT;
            freq /= countFFT;
            // SerialUSB.println(freq); // fourier need corection :-(
            if (amplitude - amp >= 1 || amplitude - amp <= -1) {
                amplitude = amp;
                ampFlag = true;
            }
            if (frequency - freq >= 45 || frequency - freq <= -45) {
                frequency = freq;
                freqFlag = true;
            }
            freq = 0;
            amp = 0;
            countFFT = 0;
        }
        
        indexFFT = 0;
        countSamples = 0;
        NVIC_EnableIRQ(ADC_IRQn);  // enable ADC_IRQ again
    }*/
    
    if (tempFlag) {
        packetBuffer[0] |= (1 << TEMP_HEAD);
        memmove(packetBuffer+offset, &temp, sizeof(temp));
        tempFlag = false;
        offset += 4;
        newMessage = true;
        // SerialUSB.println("temp");
    }
    if (humFlag) {
        packetBuffer[0] |= (1 << HUM_HEAD);
        memmove(packetBuffer+offset, &humidity, sizeof(humidity));
        humFlag = false;
        offset += 4;
        newMessage = true;
        // SerialUSB.println("hum");
    }
    if (pressFlag) {
        packetBuffer[0] |= (1 << PRESS_HEAD);
        memmove(packetBuffer+offset, &pressure, sizeof(pressure));
        pressFlag = false;
        offset += 4;
        newMessage = true;
        // SerialUSB.println("press");
    }
    if (gasFlag) {
        packetBuffer[0] |= (1 << GAS_HEAD);
        memmove(packetBuffer+offset, &gas, sizeof(gas));
        gasFlag = false;
        offset += 4;
        newMessage = true;
        // SerialUSB.println("gas");
    }
    
    if (freqFlag) {
        packetBuffer[0] |= (1 << FREQ_HEAD);
        memmove(packetBuffer+offset, &frequency, sizeof(frequency));
        freqFlag = false;
        offset += 4;
        newMessage = true;
        // SerialUSB.println("freq");
    }
    if (ampFlag) {
        packetBuffer[0] |= (1 << AMP_HEAD);
        memmove(packetBuffer+offset, &amplitude, sizeof(amplitude));
        ampFlag = false;
        offset += 4;
        newMessage = true;
        //SerialUSB.println("amp");
    }
    if (testFlag) {
        packetBuffer[0] |= (1 << TEST_HEAD);
        memmove(packetBuffer+offset, &testCounter, sizeof(testCounter));
        testFlag = false;
        // SerialUSB.println("test");
    }

    bool result = false;
    if (millis() - prevMessage >= 10000) {
    //if (newMessage) { 
        //snprintf((char*)str, 254, "t%d, h%d, p%d, g%d, f %d, a%d, c%d", 
                //temp, humidity, pressure, gas, frequency, amplitude, testCounter);
        
        //SerialUSB.println(str);
        newMessage = false;
        prevMessage = millis();
        testCounter++;
        NVIC_DisableIRQ(ADC_IRQn); // stop ADC_IRQ while sending messages
        result = lora.transferPacket(packetBuffer, 100);
        NVIC_EnableIRQ(ADC_IRQn);  // enable ADC_IRQ again
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
    if(sampleCounter < dSize) {
      aDCVal[sampleCounter] = REG_ADC_RESULT;
      sampleCounter++;
    }
    // Need to reset interrupt
    ADC->INTFLAG.reg = ADC_INTENSET_RESRDY;
}
