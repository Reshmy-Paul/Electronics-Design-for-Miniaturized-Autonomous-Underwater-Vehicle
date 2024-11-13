void setup() 
{
 Serial.begin(9600);

 SYSCTRL->DFLLCTRL.bit.ENABLE=0x0002;      //enable DFLL48MHz ADC i/p clock frequency

 GCLK->GENDIV.reg= GCLK_GENDIV_DIV(0) |    // Divide the 48MHz system clock by 8 = 6MHz
 GCLK_GENDIV_ID(2);                        // Set division on Generic Clock Generator 2 (GCLK2)

 GCLK->GENCTRL.reg|= GCLK_GENCTRL_GENEN |            //Enable Generic Clock Generator GCLK2
                     GCLK_GENCTRL_SRC_DFLL48M |      //Set the 48MHz clock source
                     GCLK_GENCTRL_ID(2);             //Select GCLK2

 GCLK->CLKCTRL.reg|= GCLK_CLKCTRL_CLKEN |        // Enable Generic Clock
                     GCLK_CLKCTRL_GEN_GCLK2 |    // Select the 48MHz GCLK2
                     GCLK_CLKCTRL_ID_ADC;        // Set GCLK2 as a clock source for ADC
 
 ADC->CTRLA.bit.ENABLE = 0x01;                    // Enable the ADC

 ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV512 |    // Divide Clock ADC GCLK by 512 (48MHz/512 = 93.7kHz)
                  ADC_CTRLB_RESSEL_12BIT;         // Set ADC resolution to 12 bits
}

void loop() 
{
  ADC->CTRLB.bit.DIFFMODE = 1;        // Set differential mode
  ADC->CTRLB.bit.FREERUN = 1;         // Enable Free running mode 

  ADC->INPUTCTRL.bit.MUXPOS = 0x00;   //PIN0 (ADC AIN0 pin)
  ADC->INPUTCTRL.bit.MUXNEG = 0x01;   //PIN1 (ADC AIN1 pin)

  ADC->SWTRIG.bit.START=1;                         // ADC start conversion
  while(ADC->STATUS.bit.SYNCBUSY);                 // Wait for write synchronization
  while(!ADC->INTFLAG.bit.RESRDY);                 // Wait for the conversion to complete
  ADC->INTFLAG.bit.RESRDY = 1;                     // Clear the result ready (RESRDY) interrupt flag
  while(ADC->STATUS.bit.SYNCBUSY);                 // Wait for read synchronization
  float result = ADC->RESULT.reg;                  // Read the ADC result

  Serial.println(result);
}