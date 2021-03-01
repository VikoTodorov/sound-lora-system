/*
 * FILENAME: EXAMPLE01_basic.ino
 * AUTHOR:   Orlando S. Hoilett
 * CONTACT:  orlandohoilett@gmail.com
 * VERSION:  3.0.0
 * 
 * 
 * AFFILIATIONS
 * Linnes Lab, Weldon School of Biomedical Engineering,
 * Purdue University, West Lafayette, IN 47907
 * 
 * 
 * DESCRIPTION
 * Basic test of the KickFFT class to calculate the Discrete Fourier Transform
 * 
 * The input signal is a simulated photoplethysmogram used to
 * measure heart rate. Each major peak is a new heart beat. The
 * smaller peak is a dicrotic notch which is generated when
 * the aortic valve closes.
 * 
 * 
 * UPDATES
 * Version 1.0.0
 * 2020/03/23:1424>
 *           - Updated comments section.
 *           - Added column headings "Freq(Hz),Magnitude"
 * 2020/07/16:0815>
 *           - Updated comments a bit.
 * Version 2.0.0
 * 2020/08/21:1847>
 *           - Moved to a templated class.
 * Version 3.0.0
 * 2020/08/23:0406> (UTC-5)
 *           - changed magnitude types to uint32_t to
 *           match isqrt function.
 * 
 * 
 * DISCLAIMER
 * Linnes Lab code, firmware, and software is released under the
 * MIT License (http://opensource.org/licenses/MIT).
 * 
 * The MIT License (MIT)
 * 
 * Copyright (c) 2020 Linnes Lab, Purdue University
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 */


#include "KickFFT.h"


const uint16_t samples = 1024;
//data is sampled every 42 milliseconds
//the input signal is a simulated photoplethysmogram used to measure
//heart rate. Each major peak is a new heart beat. The smaller peak is a
//dicrotic notch which is generated when the aortic valve closes.
const int16_t input[] = {304, 216, 158, 96, 37, -42, -98, -157, -207, -254, -291, -317, -340, -342, -359, -355, -331, -302, -266, -228, -184, -141, -101, -58, -14, 27, 66, 114, 148, 189, 234, 283, 323, 355, 379, 392, 397, 390, 356, 319, 266, 205, 159, 62, -16, -68, -130, -179, -224, -267, -303, -332, -347, -355, -351, -340, -318, -277, -266, -219, -176, -130, -87, -48, -5, 38, 85, 127, 171, 214, 260, 310, 334, 361, 381, 395, 380, 371, 345, 299, 245, 184, 113, 40, -23, -85, -128, -210, -253, -293, -319, -342, -352, -361, -349, -333, -311, -277, -239, -195, -154, -98, -84, -34, 3, 46, 98, 138, 186, 232, 275, 318, 353, 370, 394, 416, 383, 358, 322, 265, 220, 151, 83, 20, -43, -110, -167, -216, -258, -301, -311, -355, -363, -366, -352, -324, -296, -263, -219, -184, -143, -101, -56, -10, 22, 79, 107, 149, 195, 243, 290, 329, 357, 383, 394, 392, 379, 353, 310, 264, 193, 125, 57, -11, -70, -130, -183, -232, -272, -308, -332, -350, -353, -346, -328, -327, -294, -251, -211, -163, -124, -84, -33, -5, 43, 88, 134, 168, 219, 271, 295, 338, 362, 383, 397, 391, 368, 341, 297, 235, 174, 103, 36, -27, -99, -163, -210, -258, -290, -322, -342, -360, -360, -352, -333, -310, -276, -235, -176, -170, -118, -83, -35, 12, 55, 101, 148, 197, 236, 279, 316, 351, 374, 398, 381, 373, 349, 323, 267, 217, 144, 81, 10, -54, -117, -174, -223, -263, -312, -337, -359, -365, -365, -349, -330, -298, -265, -222, -180, -140, -94, -50, 15, 7, 62, 108, 153, 201, 249, 291, 331, 359, 382, 391, 389, 376, 348, 313, 240, 185, 111, 50, -16, -83, -141, -190, -238, -280, -310, -336, -352, -360, -361, -349, -324, -291, -250, -209, -165, -122, -78, -40, -1, 41, 93, 131, 204, 212, 261, 302, 341, 366, 389, 391, 390, 366, 334, 284, 208, 164, 97, 31, -30, -111, -165, -210, -254, -292, -327, -347, -353, -355, -346, -331, -305, -269, -238, -197, -155, -115, -64, -26, 19, 64, 109, 151, 198, 240, 282, 328, 385, 369, 388, 389, 389, 359, 321, 272, 214, 144, 73, 7, -56, -124, -176, -216, -278, -309, -333, -359, -359, -355, -340, -323, -289, -253, -209, -166, -128, -80, -47, -16, 32, 79, 127, 172, 214, 256, 307, 339, 361, 390, 398, 392, 397, 338, 298, 244, 186, 115, 47, -20, -80, -141, -195, -244, -282, -310, -335, -346, -358, -354, -340, -312, -279, -236, -197, -158, -116, -73, -27, 4, 50, 99, 126, 169, 224, 260, 293, 342, 372, 386, 393, 388, 367, 330, 283, 226, 184, 77, 13, -50, -111, -163, -213, -257, -294, -325, -347, -357, -358, -349, -323, -295, -277, -226, -191, -146, -101, -57, -20, 29, 68, 110, 158, 203, 244, 294, 319, 352, 370, 385, 398, 376, 354, 316, 264, 201, 132, 65, -1, -65, -108, -191, -238, -282, -313, -337, -352, -360, -351, -338, -321, -289, -251, -207, -173, -124, -92, -56, -14, 32, 79, 122, 169, 220, 261, 302, 338, 384, 385, 402, 383, 359, 333, 287, 243, 176, 109, 33, -26, -91, -148, -201, -248, -284, -302, -354, -362, -369, -355, -336, -310, -277, -235, -195, -157, -117, -71, -33, 6, 56, 87, 132, 175, 223, 274, 310, 345, 374, 385, 389, 375, 353, 320, 281, 201, 134, 68, 2, -60, -131, -175, -223, -267, -301, -334, -357, -364, -362, -336, -345, -307, -273, -231, -189, -146, -108, -61, -18, 12, 61, 106, 154, 192, 247, 270, 314, 339, 368, 385, 384, 369, 341, 300, 242, 187, 115, 46, -10, -92, -150, -206, -247, -286, -324, -344, -366, -362, -364, -347, -325, -294, -249, -194, -186, -137, -98, -52, -9, 34, 77, 124, 164, 207, 258, 300, 335, 359, 382, 377, 375, 356, 325, 288, 228, 162, 100, 28, -39, -99, -159, -212, -254, -304, -333, -358, -365, -364, -357, -340, -310, -277, -235, -201, -156, -112, -71, -6, 1, 50, 92, 133, 180, 231, 272, 315, 355, 372, 383, 387, 378, 350, 316, 253, 192, 131, 63, 3, -66, -132, -177, -227, -271, -311, -337, -357, -360, -368, -356, -338, -307, -267, -223, -180, -141, -96, -54, -11, 28, 69, 114, 183, 194, 247, 284, 330, 358, 378, 389, 391, 375, 341, 297, 246, 185, 114, 50, -27, -89, -149, -196, -240, -280, -318, -336, -351, -357, -354, -331, -315, -277, -247, -208, -169, -123, -79, -36, 6, 48, 91, 138, 182, 229, 271, 314, 380, 359, 384, 390, 381, 361, 334, 285, 226, 168, 95, 27, -42, -102, -158, -205, -259, -307, -331, -345, -355, -352, -347, -331, -299, -266, -225, -183, -144, -93, -67, -25, 15, 61, 108, 154, 197, 241, 289, 330, 360, 380, 393, 397, 408, 345, 315, 259, 201, 135, 73, 3, -63, -126, -180, -230, -274, -306, -335, -348, -364, -362, -350, -325, -289, -252, -213, -168, -132, -89, -49, -7, 35, 89, 110, 156, 201, 252, 303, 336, 363, 385, 389, 385, 366, 339, 297, 242, 198, 98, 35, -41, -95, -150, -199, -244, -284, -312, -339, -354, -358, -350, -333, -310, -288, -248, -200, -164, -118, -80, -37, 13, 47, 92, 138, 184, 227, 284, 304, 335, 360, 376, 373, 382, 355, 321, 274, 216, 143, 83, 11, -54, -92, -184, -227, -276, -307, -338, -357, -366, -355, -352, -327, -305, -268, -227, -187, -142, -114, -68, -29, 12, 61, 112, 149, 201, 242, 283, 325, 356, 376, 409, 375, 367, 337, 296, 249, 192, 121, 54, -10, -76, -136, -194, -237, -278, -292, -348, -360, -371, -356, -338, -320, -292, -248, -209, -164, -133, -87, -42, -3, 37, 76, 120, 166, 214, 261, 318, 336, 373, 387, 393, 389, 372, 335, 305, 227, 165, 96, 25, -38, -98, -159, -203, -248, -287, -325, -342, -355, -357, -332, -346, -314, -280, -233, -190, -151, -109, -67, -26, 16, 56, 103, 143, 191, 236, 270, 316, 346, 372, 389, 391, 379, 364, 324, 272, 215, 147, 74, 18, -67, -121, -180, -224, -269, -303, -328, -350, -361, -356, -342, -325, -298, -263, -202, -184, -142, -101, -59, -19, 24, 68, 113, 159, 205, 249, 289, 322};
const float Fs = 16666; //Hz


void setup()
{
  Serial.begin(9600); //Use SerialUSB for SparkFun SAMD21 boards
  while(!Serial); //will not run until Serial Monitor is open

  uint32_t mag[samples] = {0};
  uint16_t startIndex = 0;
  uint16_t endIndex = 0;
  KickFFT<int16_t>::fft(Fs, 0, Fs/2, samples, input, mag, startIndex, endIndex);


  //Print to Serial Monitor and copy and paste
  //into a .csv file to display in Excel
  Serial.println("Freq(Hz),Magnitude"); //Use SerialUSB for SparkFun SAMD21 boards
  for(uint16_t i = startIndex; i < endIndex; i++)
  {
    //Peak should be around 1.3 Hz or so
    
    Serial.print(i*Fs/samples); //Use SerialUSB for SparkFun SAMD21 boards
    Serial.print(","); //Use SerialUSB for SparkFun SAMD21 boards
    Serial.print(mag[i]); //Use SerialUSB for SparkFun SAMD21 boards
    Serial.println(); //Use SerialUSB for SparkFun SAMD21 boards
  }

  //Print the Raw Signal
  Serial.println("Time(ms),PPG"); //Use SerialUSB for SparkFun SAMD21 boards
  for(uint16_t i = 0; i < samples; i++)
  {
    Serial.print(i*1/Fs); //Use SerialUSB for SparkFun SAMD21 boards
    Serial.print(","); //Use SerialUSB for SparkFun SAMD21 boards
    Serial.print(input[i]); //Use SerialUSB for SparkFun SAMD21 boards
    Serial.println(); //Use SerialUSB for SparkFun SAMD21 boards
  }
  
}


void loop()
{
}
