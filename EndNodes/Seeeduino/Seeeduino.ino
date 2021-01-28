#include "Zanshin_BME680.h"  // Include the BME680 Sensor library
#include <LoRaWan.h>
#include "lora_secrets.h" 

char buffer[256];

BME680_Class BME680;  // Create an instance of the BME680 class
static int32_t temp, humidity, pressure, gas;  // BME readings

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

bool result = false;

char str[128];
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
    
    #include "BME680_SETUP.hh"
        
}

uint16_t counter = 0;
void loop(void) {   
    uint16_t amplitude = 80;
    uint16_t frequency = 4440;

  	BME680.getSensorData(temp, humidity, pressure, gas);  // Get readings
    uint8_t offset = 1;

    tempFlag = true;
    humFlag = true;
    pressFlag = true;
    gasFlag = true;
    freqFlag = true;
    ampFlag = true;
    testFlag = true; 
    if (tempFlag) {
        packetBuffer[0] |= (1 << TEMP_HEAD);
        memmove(packetBuffer+offset, &temp, sizeof(temp));
        tempFlag = false;
        offset += 4;
    }
    if (humFlag) {
        packetBuffer[0] |= (1 << HUM_HEAD);
        memmove(packetBuffer+offset, &humidity, sizeof(humidity));
        humFlag = false;
        offset += 4;
    }
    if (pressFlag) {
        packetBuffer[0] |= (1 << PRESS_HEAD);
        memmove(packetBuffer+offset, &pressure, sizeof(pressure));
        pressFlag = false;
        offset += 4;
    }
    if (gasFlag) {
        packetBuffer[0] |= (1 << GAS_HEAD);
        memmove(packetBuffer+offset, &gas, sizeof(gas));
        gasFlag = false;
        offset += 4;
    }
    if (freqFlag) {
        packetBuffer[0] |= (1 << FREQ_HEAD);
        memmove(packetBuffer+offset, &frequency, sizeof(frequency));
        freqFlag = false;
        offset += 2;
    }
    if (ampFlag) {
        packetBuffer[0] |= (1 << AMP_HEAD);
        memmove(packetBuffer+offset, &amplitude, sizeof(amplitude));
        ampFlag = false;
        offset += 2;
    }
    if (testFlag) {
        packetBuffer[0] |= (1 << TEST_HEAD);
        memmove(packetBuffer+offset, &counter, sizeof(counter));
        testFlag = false;
    }

    result = false;
    
    //snprintf((char*)str, 128, "t%d, h%d, p%d, g%d, f%d, a%d, s%d", 
    		//temp, humidity, pressure, gas, frequency, amplitude, seconds);
    
    //SerialUSB.println(str);
    result = lora.transferPacket(packetBuffer, 100);
    //result = lora.transferPacket("t:2700,p:1700,h:5000,g:156,f:440,a:230", 10);
 
    if(result)
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
    }
    counter++;
    delay(10000);
}
