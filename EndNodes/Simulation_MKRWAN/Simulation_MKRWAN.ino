#include <MKRWAN.h>
#include "lora_secrets.hh" 


char buffer[256]; // lora init buffer

typedef struct {
    int32_t frequency;
    int32_t amplitude;
} soundMeasurement_t;

typedef struct {
    int32_t temp;
    int32_t humidity;
    int32_t pressure;
    int32_t gas;
} bmeMeasurement_t;

soundMeasurement_t soundMeasurement;
bmeMeasurement_t bmeMeasurement;


unsigned char packetBuffer[32]; 
bool tempFlag = false;
bool humFlag = false;
bool pressFlag = false;
bool gasFlag = false;
bool freqFlag = false;
bool ampFlag = false;
bool testFlag = false;

#define TEMP_HEAD 6
#define HUM_HEAD 5
#define PRESS_HEAD 4
#define GAS_HEAD 3
#define FREQ_HEAD 2
#define AMP_HEAD 1
#define TEST_HEAD 0

LoRaModem modem;
void setup() {
    soundMeasurement.amplitude = 17;
    soundMeasurement.frequency = 200;
    
    bmeMeasurement.temp = 2431;
    bmeMeasurement.humidity = 5025;
    bmeMeasurement.pressure = 6789;
    bmeMeasurement.gas = 45678;
    
    if (!modem.begin(EU868)) {
        pinMode(LED_BUILTIN, OUTPUT);
        digitalWrite(LED_BUILTIN, HIGH);
        while (1);
    };

    bool connected = modem.joinOTAA(APP_EUI, APP_KEY);
    delay(1000);
    
    modem.minPollInterval(5);
    if (!connected) {
        Serial.println("Something went wrong; are you indoor? Move near a window and retry");
        while (1) {}
    }
    modem.setADR(true);
    delay(1000);
}

unsigned long prevTime;
bool firstLoop = true;
bool newMessage = false;

void loop() {
    uint8_t offset = 1;
    
    if (firstLoop) {
        tempFlag = true;
        humFlag = true;
        pressFlag = true;
        gasFlag = true;
        ampFlag = true;
        freqFlag = true;
        prevTime = millis();
    }
    
    else if(millis() - prevTime >= 15000) {
        soundMeasurement.amplitude--;
        soundMeasurement.frequency += 200;
        tempFlag = true;
        humFlag = true;
        pressFlag = true;
        gasFlag = true;
        ampFlag = true;
        freqFlag = true;
        prevTime = millis();
    }
  
    if (tempFlag) {
        packetBuffer[0] |= (1 << TEMP_HEAD);
        memmove(packetBuffer+offset, &bmeMeasurement.temp, sizeof(bmeMeasurement.temp));
        tempFlag = false;
        offset += sizeof(bmeMeasurement.temp);
        newMessage = true;
    }
    if (humFlag) {
        packetBuffer[0] |= (1 << HUM_HEAD);
        memmove(packetBuffer+offset, &bmeMeasurement.humidity, sizeof(bmeMeasurement.humidity));
        humFlag = false;
        offset += sizeof(bmeMeasurement.humidity);
        newMessage = true;
    }
    if (pressFlag) {
        packetBuffer[0] |= (1 << PRESS_HEAD);
        memmove(packetBuffer+offset, &bmeMeasurement.pressure, sizeof(bmeMeasurement.pressure));
        pressFlag = false;
        offset += sizeof(bmeMeasurement.pressure);
        newMessage = true;
    }
    if (gasFlag) {
        packetBuffer[0] |= (1 << GAS_HEAD);
        memmove(packetBuffer+offset, &bmeMeasurement.gas, sizeof(bmeMeasurement.gas));
        gasFlag = false;
        offset += sizeof(bmeMeasurement.gas);
        newMessage = true;
    }
    if (freqFlag) {
        packetBuffer[0] |= (1 << FREQ_HEAD);
        memmove(packetBuffer+offset, &soundMeasurement.frequency, sizeof(soundMeasurement.frequency));
        freqFlag = false;
        offset += sizeof(soundMeasurement.frequency);
        newMessage = true;
    }
    if (ampFlag) {
        packetBuffer[0] |= (1 << AMP_HEAD);
        memmove(packetBuffer+offset, &soundMeasurement.amplitude, sizeof(soundMeasurement.amplitude));
        ampFlag = false;
        offset += sizeof(soundMeasurement.amplitude);
        newMessage = true;
    }

    bool result = false;
    if (newMessage) { 
        newMessage = false;
        modem.beginPacket();
        modem.write(packetBuffer, offset);
        result = modem.endPacket(true);
        memset(packetBuffer, 0, offset);
    }
    
    if (firstLoop) {
        delay(200000);
        firstLoop = false;
    }

}
