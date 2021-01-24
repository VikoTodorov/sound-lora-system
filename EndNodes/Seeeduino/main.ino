#include <LoRaWan.h>
#include "lora_secrets.h" 

char buffer[256];

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
    lora.setChannel(2, 868.3);
    lora.setChannel(2, 868.5);
 
    lora.setReceiceWindowFirst(0, 868.1);
    lora.setReceiceWindowSecond(869.5, DR5);
 
    lora.setDutyCycle(false);
    lora.setJoinDutyCycle(false);
 
    lora.setPower(14);
}

String str;
int i = 0;
char buff[20];
void loop(void)
{   
    bool result = false;
    delay(2000);
    i++;
    str = "Hello World! " + i;
    str.toCharArray(buff, 20);
    SerialUSB.println(str);
    result = lora.transferPacket("t:2700,p:1700,h:5000,g:156,f:440,a:230", 10);
 
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
    delay(1000);
}
