#include <SD.h>             // include the SD library
#include <SPI.h>

// Use the SAMD21's DMAC to transfer ADC results to buffer array in memory
// ADC Clock: 48MHz / 64 = 750kHz (ADCs typically require slower clocks than the main system clock to allow accurate conversions.)
// Conversion/Sample Rate: ((12-bit resolution/2) + 1 delay gain) = 7cycles. 
// (7cycles / 750kHz) = 9.3µs = 107.142kHz = 107,142 Sps

//#define SAMPLE_NO 512                                                           // Define the number of ADC samples
#define SAMPLE_NO 1024

uint16_t adcResult[SAMPLE_NO] = {};                                               // Store ADC values

typedef struct                                                                    // DMAC descriptor structure
{
  uint16_t btctrl;
  uint16_t btcnt;
  uint32_t srcaddr;
  uint32_t dstaddr;
  uint32_t descaddr;
} dmacdescriptor ;

volatile dmacdescriptor wrb[DMAC_CH_NUM] __attribute__ ((aligned (16)));          // Write back DMAC descriptors. Declares an array 'wrb' of 'dmacdescriptor' structures with size 'DMAC_CH_NUM', aligned to a 16-byte boundary, and marked as volatile because its values can change unexpectedly. Stores write-back descriptors to track completed DMA transfers. 
  //The __attribute__((aligned(16))) ensures that this array is aligned to a 16-byte boundary in memory. 

dmacdescriptor descriptor_section[DMAC_CH_NUM] __attribute__ ((aligned (16)));    // DMAC channel descriptors. Declares another array 'descriptor_section' of 'dmacdescriptor' structures, aligned to a 16-byte boundary. Stores active descriptors for each DMA channel, which the DMAC uses to know where to transfer data.
dmacdescriptor descriptor __attribute__ ((aligned (16)));                         // Place holder descriptor. 


void setup() {
  
  //Serial.begin(115200);

  Serial.begin(115200);                                                        // Start the native USB port 
  while(!Serial);                                                              // Wait for the console to open
  Serial.println("Setup");

  DMAC->BASEADDR.reg = (uint32_t) descriptor_section;                             // Specify the location of the descriptors. This is the register in the DMAC that holds the base address (location in memory) where the DMAC descriptors are stored.
  DMAC->WRBADDR.reg = (uint32_t) wrb;                                             // Specify the location of the write back descriptors, which are used to track the status of each completed transfer.
  DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);                    // Enable the DMAC peripheral. DMAC_CTRL_LVLEN(0xf): This enables all four priority levels (LVL0, LVL1, LVL2, and LVL3). These levels can be used to prioritize transfers.

  DMAC->CHID.reg = DMAC_CHID_ID(0);                                               // Select DMAC channel 0. 'DMAC->CHID.reg:'- This register lets you select which DMAC channel you want to configure. 'DMAC_CHID_ID(0)' macro selects channel 0.
  DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL(0) | DMAC_CHCTRLB_TRIGSRC(ADC_DMAC_ID_RESRDY) | DMAC_CHCTRLB_TRIGACT_BEAT;        // configures how and when DMAC channel 0 should move data. It sets three key things: priority level, trigger source, and the action to perform on each trigger.   
  //DMAC_CHCTRLB_TRIGACT_BEAT: This macro sets the action to BEAT mode. A "beat" refers to one unit of data (in this case, a 16-bit value from the ADC).

  descriptor.descaddr = (uint32_t)0;                                              // Set up descriptor, means this is the only transfer that will occur. Once the transfer completes, the DMAC will stop.
  descriptor.srcaddr = (uint32_t)&ADC->RESULT.reg;                                // Take the result from the ADC RESULT register.  This sets the source address to the ADC’s result register, which is where the ADC stores the result of each conversion.
  descriptor.dstaddr = (uint32_t)&adcResult[0] + sizeof(uint16_t) * SAMPLE_NO;    // Place it in the adcResult array. This sets the destination address to the location where the ADC data will be stored—in this case, the adcResult array.
  descriptor.btcnt = SAMPLE_NO;                                                   // Beat count is SAMPLE_NO
  descriptor.btctrl = DMAC_BTCTRL_BEATSIZE_HWORD |                                // Beat size is HWORD (16-bits). This specifies the size of each data transfer unit (beat) as a half-word, which is 16 bits.
                      DMAC_BTCTRL_DSTINC |                                        // Increment the destination address. This tells the DMAC to increment the destination address after each data transfer.
                      DMAC_BTCTRL_VALID;                                          // Descriptor is valid. This marks the descriptor as valid, meaning it can be used by the DMAC.

  memcpy(&descriptor_section[0], &descriptor, sizeof(descriptor));                // Copy the descriptor to the descriptor section array

  Serial.println("setup DMAC done");

  ADC->INPUTCTRL.bit.MUXPOS = 0x0;                   // Set the analog input to A0
  while(ADC->STATUS.bit.SYNCBUSY);                   // Wait for synchronization

  ADC->SAMPCTRL.bit.SAMPLEN = 0x00;                  // Set max Sampling Time Length to half divided ADC clock pulse (2.66µs)
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV64 |       // Divide Clock ADC GCLK by 64 (48MHz/64 = 750kHz)
                   ADC_CTRLB_RESSEL_12BIT |          // Set the ADC resolution to 12 bits
                   ADC_CTRLB_FREERUN;                // Set the ADC to free run
  while(ADC->STATUS.bit.SYNCBUSY);                   // Wait for synchronization  

  ADC->CTRLA.bit.ENABLE = 1;                         // Enable the ADC
  while(ADC->STATUS.bit.SYNCBUSY);                   // Wait for synchronization
  ADC->SWTRIG.bit.START = 1;                         // Initiate a software trigger to start an ADC conversion
  while(ADC->STATUS.bit.SYNCBUSY);                   // Wait for synchronization

  Serial.println("setup done");

  Serial.println("Initializing SD card...");

  if (!SD.begin(SDCARD_SS_PIN)) {                     // ChipSelect to establish connection with the SD card 
    Serial.println("Initialization failed!");
    while (1);
  }
  Serial.println("Initialization done.");
  //Serial.end();
}


void loop() {
  // put your main code here, to run repeatedly:
  DMAC->CHID.reg = DMAC_CHID_ID(0);                  // Select DMAC channel 0
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;          // Enable the selected DMAC channel
  while(!DMAC->CHINTFLAG.bit.TCMPL);                 // Wait for the DMAC to transfer complete(TCMPL) interrupt flag
  DMAC->CHINTFLAG.bit.TCMPL = 1;                     // Clear the DMA transfer complete (TCMPL) interrupt flag
   
  int total_sample_time = 10000;                      // 10000ms = 10s
   int n_iterations      = 1000;
 // int n_iterations      = total_sample_time * 107.14 * 1000.0 / SAMPLE_NO;
  Serial.println(n_iterations);

//uint16_t adcResult[SAMPLE_NO] = {};                 // Store ADC values
//byte testArray[2*SAMPLE_NO];
byte * testArray; // bytes is an 8 bit unsigned datatype equivalent to uint8_t
                  // //testArray is a pointer that can point to the address of a byte
            

testArray = (byte *)adcResult;// cast the adcResult array (which is of type uint16_t) to a pointer to byte.
                              // This allows testArray to point to the same memory location as adcResult, but treat each 16-bit value in adcResult as a series of bytes (2 bytes for each uint16_t value).


//SD.write(testArray,128);

 File dataFile = SD.open("data.txt", FILE_WRITE); //If this file already exists in the sd card it will open, else new file with this name is created.
                                                 

//dataFile.write(testArray,128);
  Serial.println("Start recording");

for (uint16_t indx = 0; indx<n_iterations; indx++) {

if (indx % 10 == 0)
 Serial.println(indx);
 
  if (dataFile)                          // if the datafile was opened properly
  {
    dataFile.write(testArray,2*SAMPLE_NO);             // Write whole ADC result to the file

   
  // Serial.println("Write to SD card.");
  //for (uint16_t i = 0; i<SAMPLE_NO; i++) { 
    // WRITE TO SD card here!!!!!
   //dataFile.println(adcResult[i]);             // Write each ADC result to the file
    //Serial.println(adcResult[i]);
   //}
  
   //Serial.println("Data written to SD card.");
  }

  else{
  Serial.println("Error opening file.");  // this command gets executed if data.txt was not properly opened and prints the error message
  }
  
}
  Serial.println("n_iterations done");
 
  // WRITE sd-card to UART
  /*byte readArray[2*SAMPLE_NO];
  dataFile.read(testArray,2*SAMPLE_NO);
  */

 
    dataFile.close();                             // Close the file
 
  delay(100);     
  //Write to UART here!!!
  //Serial.end();
}