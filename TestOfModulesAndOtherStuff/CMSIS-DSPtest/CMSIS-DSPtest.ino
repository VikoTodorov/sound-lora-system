#include "signal.h"
#define ARM_MATH_CM0
#include <arm_math.h>

#define DATA_SIZE 512

//the sample rate
#define FS 16666
q15_t buff[DATA_SIZE*2];
//q15_t buff1[DATA_SIZE*2];
//float buff[DATA_SIZE];



arm_rfft_instance_q15 rfft_instance;
//arm_rfft_fast_instance_f32 rfft_instance;
// the setup routine runs once when you press reset:
q15_t maxValue;
uint32_t testIndex;
void setup() {
  Serial.begin(115200);
  while(!Serial); //wait for serial to be ready 
  /*arm_rfft_fast_init_f32(&rfft_instance, DATA_SIZE);
  arm_rfft_fast_f32(&rfft_instance, fftSignal_220, buff, 0);   
  //arm_cmplx_mag_f32(buff, buff, DATA_SIZE);
  arm_max_f32(buff, DATA_SIZE, &maxValue, &testIndex);*/
  
  arm_rfft_init_q15(&rfft_instance, DATA_SIZE, 0, 1);
  arm_rfft_q15(&rfft_instance, fftSignal_220, buff);
  for(int i=0; i < DATA_SIZE; i++){
    buff[i] <<= 8;
  }
  //arm_cmplx_mag_q15(buff, buff, DATA_SIZE);
  arm_max_q15(buff, 1024, &maxValue, &testIndex);
  
  //arm_abs_q15(buff, buff, DATA_SIZE);

  //arm_max_f32(buff, DATA_SIZE, &maxValue, &testIndex);
  for(int i=0; i<DATA_SIZE; i++){
    Serial.println(buff[i]);
  }
  Serial.println(maxValue);
  Serial.println(testIndex);
}


void loop() {
  //don't even do anything
}
