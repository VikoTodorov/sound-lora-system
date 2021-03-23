#define sRate 16000           // sample rate of ADC

#define dSize 1024     // used to set number of samples
#define dHalfSize 512
#define gClk 3             // used to define which generic clock for ADC
#define intPri 0           // used to set interrupt priority for ADC
#define cDiv 1              //divide factor for generic clock

volatile int16_t ADCVal[dSize];   //array to hold ADC samples
volatile uint16_t sampleCounter = 0;          //tracks how many samples we have collected

uint8_t countFFT = 0;
uint16_t indexFFT = 0;
uint32_t freq = 0;
int32_t amplitude = 0;

void removeDCOffset(volatile int16_t, uint16_t);
void setup() {
    SerialUSB.begin(115200);
    while(!SerialUSB); 
    //analogReadResolution(8);
    noInterrupts();
    clearRegisters();
    portSetup();                   // setup the ports or pin to make ADC measurement
	  genericClockSetup(gClk, cDiv); // setup generic clock and routed it to ADC
    aDCSetup();                    // set up registers for ADC, input argument sets ADC reference
    setUpInterrupt(intPri);        // sets up interrupt for ADC and argument assigns priority
  	aDCSWTrigger();                // trigger ADC to start free run mode
    interrupts();
    
    while(sampleCounter != dSize);
    NVIC_DisableIRQ(ADC_IRQn);
    int maxMeasurement, minMeasurement, avr = 0;
    removeDCOffset(ADCVal, dSize);
    //maxMeasurement = ADCVal[0];
    //minMeasurement = ADCVal[0];
    for (int j = 0; j < dSize; j++) {
        if (j < dSize - 1) {
            SerialUSB.print(ADCVal[j]);
            SerialUSB.print(" ");
        }
        else {
            SerialUSB.print(ADCVal[j]);
            SerialUSB.println();
        }
        //if (maxMeasurement < ADCVal[j]) maxMeasurement = ADCVal[j];
        //if (minMeasurement > ADCVal[j]) minMeasurement = ADCVal[j];
        //avr += ADCVal[j];
    }
    //SerialUSB.print("Max ");
    //SerialUSB.println(maxMeasurement);
    //SerialUSB.print("Min ");
    //SerialUSB.println(minMeasurement);
    //avr >>= 10;
    //SerialUSB.println(avr);
    sampleCounter = 0;
    NVIC_EnableIRQ(ADC_IRQn); 
}

void loop() {
}

void clearRegisters() {
    // clear/disable adc if enabled
    ADC->CTRLA.reg = 0; 	
    while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY);
    // clear/disable default arduino ide settings for ctrlb 
	  ADC->CTRLB.reg = 0; 	
    while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY);
}

// function for configuring ports or pins, note that this will not use the same pin numbering scheme as Arduino, it will use Pxy groups
// x is the group and y is the number or id 
void portSetup() {
  	// Input pin for ADC Arduino A0/PA02
  	REG_PORT_DIRCLR1 = PORT_PA02;

  	// Enable multiplexing on PA02_AIN0 and PA03/ADC_VREFA
  	PORT->Group[0].PINCFG[2].bit.PMUXEN = 1;
  	PORT->Group[0].PINCFG[3].bit.PMUXEN = 1;
  	PORT->Group[0].PMUX[1].reg = PORT_PMUX_PMUXE_B | PORT_PMUX_PMUXO_B;
}

// this function sets up the generic clock that will be used for the ADC unit
// by default it uses the 48M system clock, input arguments set divide factor for generic clock and choose which generic clock
void genericClockSetup(int clk, int dFactor) {
  	// Enable the APBC clock for the ADC
  	REG_PM_APBCMASK |= PM_APBCMASK_ADC;
  
  	// This allows you to setup a div factor for the selected clock certain clocks allow certain division factors: Generic clock generators 3 - 8 8 division factor bits - DIV[7:0]
     GCLK->GENDIV.reg |= GCLK_GENDIV_ID(clk)| GCLK_GENDIV_DIV(dFactor);
     while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
     
  	// configure the generator of the generic clock with 8 MHz clock
  	// GCLK_GENCTRL_DIVSEL stay unset, it makes divide based on power of two
    GCLK->GENCTRL.reg |= GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(clk);
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
  
  	// enable clock, set gen clock number, and ID to where the clock goes (30 is ADC)
    GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(clk) | GCLK_CLKCTRL_ID(30);
    while (GCLK->STATUS.bit.SYNCBUSY);
}

void aDCSetup() {
  	// set vref for ADC to VCC
  	REG_ADC_REFCTRL = ADC_REFCTRL_REFSEL_INTVCC1;

  	// average control 1 sample
    // samplen = 8
    REG_ADC_AVGCTRL |= ADC_AVGCTRL_SAMPLENUM_8 | ADC_AVGCTRL_ADJRES(3);
  	REG_ADC_SAMPCTRL = ADC_SAMPCTRL_SAMPLEN(5); 
  
  	// Input control and input scan, gain 0, positive to A0, negative to gnd
  	REG_ADC_INPUTCTRL |= ADC_INPUTCTRL_GAIN_DIV2 | ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS_PIN0;
  	while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY);

    // set the divide factor, 16 bit resolution and freerun mode
  	ADC->CTRLB.reg |= ADC_CTRLB_RESSEL_16BIT | ADC_CTRLB_PRESCALER_DIV32 | ADC_CTRLB_FREERUN; 
    while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY);
    
    //ADC->OFFSETCORR.reg = ADC_OFFSETCORR_OFFSETCORR(2010);
    //while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY);
    //ADC->GAINCORR.reg = ADC_GAINCORR_GAINCORR(128);
    //while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY);

    //ADC->CTRLB.bit.CORREN = true;
    //while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY);

    // Disable window monitor mode
    ADC->WINCTRL.reg = ADC_WINCTRL_WINMODE_DISABLE; 
    while(ADC->STATUS.bit.SYNCBUSY);

    // start ADC when event occurs
    ADC->EVCTRL.reg |= ADC_EVCTRL_STARTEI; 
    while (ADC->STATUS.bit.SYNCBUSY);

    // enable and set ADC to run in standby
    ADC->CTRLA.reg |= ADC_CTRLA_ENABLE;     
    while (ADC->STATUS.bit.SYNCBUSY);
}

// This function sets up an ADC interrupt that is triggered 
// when an ADC value is out of range of the window
// input argument is priority of interrupt (0 is highest priority, except RESET, -2 and -1)
void setUpInterrupt(byte priority) {
  
    // enable ADC ready interrupt
    ADC->INTENSET.reg |= ADC_INTENSET_RESRDY; 
    while (ADC->STATUS.bit.SYNCBUSY);

    // enable ADC interrupts
    // set priority of the interrupt
    NVIC_EnableIRQ(ADC_IRQn); 
    NVIC_SetPriority(ADC_IRQn, priority); 
}

// software trigger to start ADC in free run
void aDCSWTrigger() {
    ADC->SWTRIG.reg |= ADC_SWTRIG_START;
}

// This ISR is called each time ADC makes a reading
void ADC_Handler() {
    if(sampleCounter < dSize) {
      ADCVal[sampleCounter] = REG_ADC_RESULT;
      sampleCounter++;
    }
    // Need to reset interrupt
    ADC->INTFLAG.reg = ADC_INTENSET_RESRDY;
}


// This function takes out DC offset of AC signal, it assumes that the offset brings signal to zero volts
// input arguments: array with measured points and size of measurement
void removeDCOffset (volatile int16_t valueADC[], uint16_t aSize) {
    // get number of levels in ADC measurement and divide it to aSize to get 0 level
    int avrADC = 0; 
    for (int i = 0; i < aSize; i++) {
        avrADC += valueADC[i];
    }
    avrADC /= aSize;
    for(int i = 0; i < aSize; i++) {
        // take out offset
        valueADC[i] = valueADC[i] - avrADC;
    }
}
