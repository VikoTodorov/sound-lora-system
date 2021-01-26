#include "Zanshin_BME680.h"  // Include the BME680 Sensor library
#include <LoRaWan.h>
#include "lora_secrets.h" 

char buffer[256];

BME680_Class BME680;  // Create an instance of the BME680 class
static int32_t  temp, humidity, pressure, gas;  // BME readings


int i = 0;
char packetBuffer[48]; // t19.11h43.39p931.14g397.29f8000a10s10 - ~37 characters

bool result = false;
void setup(void)
{     
 
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
    lora.setChannel(2, 868.1);
    lora.setChannel(2, 868.1);
 
    lora.setReceiceWindowFirst(0, 868.1);
    lora.setReceiceWindowSecond(869.5, DR5);
 
    lora.setDutyCycle(false);
    lora.setJoinDutyCycle(false);
 
    lora.setPower(14);
    delay(1000);
    
    #include "BME680_SETUP.hh"
        
}

void loop(void)
{   

  	BME680.getSensorData(temp, humidity, pressure, gas);  // Get readings 
    result = false;
    
    snprintf((char*)packetBuffer, 48, "t%d.%d,h%d.%d,p%d.%d,g%d.%d,f%d,a%d,s%d", 
    		(int8_t)(temp / 100), (uint8_t)(temp % 100),
    		(int8_t)(humidity / 1000), (uint16_t)((humidity % 1000) / 10),
    		(int16_t)(pressure / 100), (uint8_t)(pressure % 100),
    		(int16_t)(gas / 100), (uint8_t)(gas % 100),
    		8000, 10, 10);
    
    SerialUSB.println(packetBuffer);
    result = lora.transferPacket(packetBuffer, 10);
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
    i++;
    delay(1000);
}
