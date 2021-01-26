#include <AudioFrequencyMeter.h>

AudioFrequencyMeter meter;

long long prevTime = 0;

void setup() {

  // put your setup code here, to run once:

  Serial.begin(115200);

  Serial.println("started");

  meter.setBandwidth(70.00, 1500);    // Ignore frequency out of this range

  meter.begin(A0, 8000);             // Intialize A0 at sample rate of 45kHz
}

void loop() {

  // put your main code here, to run repeatedly:

  float frequency = meter.getFrequency();

  if (millis()-prevTime >= 1000)
  {

    Serial.print(frequency);

    Serial.println(" Hz");
    prevTime = millis();

  }
}
