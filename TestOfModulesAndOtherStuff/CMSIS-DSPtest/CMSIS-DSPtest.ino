//#include "Adafruit_ZeroFFT.h"
#include "signal.h"
#define ARM_MATH_CM0
#include <arm_math.h>

#define DATA_SIZE 1024

//the sample rate
#define FS 12500
q15_t buff[DATA_SIZE*2];
// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200);
  while(!Serial); //wait for serial to be ready
  arm_rfft_instance_q15 rfft_instance;
  arm_cfft_instance_q15 cfft_instance;
  //arm_cfft_init_q15(&cfft_instance, DATA_SIZE);   
  arm_rfft_init_q15(&rfft_instance, DATA_SIZE, 0, 1);
  arm_rfft_q15(&rfft_instance, fftSignal, buff);

  //data is only meaningful up to sample rate/2, discard the other half
  for(int i=0; i<DATA_SIZE; i++){
    //print the corresponding FFT output
    buff[i] <<= 8;
    Serial.println(buff[i]);
  }
}

void loop() {
  //don't even do anything
}
