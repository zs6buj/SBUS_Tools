
/*
 *       SBus Splitter by ZS6BUJ 1 November 2019
 *       Written for Teensy 3.2
 * 
 *       Library by Bolder_Flight_Systems: https://github.com/bolderflight/SBUS
 * 
 *        SBus Input 1 Pin 7
 *        SBus Input 2 Pin 9
 *        SBUS Output  Pin 8
 *        
 *        switchPin 14 Normally high = Input 1 Pin 7
 *                     Grounded low  = Input 2 Pin 9
 *         
 *       Timing is as follows: 1bit = 10 uS. Byte = start + 8 bits + parity + stop + extra stop = 120uS
 *
 *       1 Header byte 00001111b (0x0F)
 *       16 x 11 bit channels -> 22 bytes
 *       1 Byte with two digital channels (channel 17 and 18) and "frame lost" and "failsafe" flags
 *       1 Footer byte 00000000b (0x00)
 *       FRAME TOTAL = 25 BYTES
 *       
 */
  
#include "SBUS.h"
 
#define Debug_Input
//#define Test_Output

// SBus Input 1 Pin 7
// SBus Input 2 Pin 9
// SBUS Output  Pin 8

#define Debug Serial     // USB
SBUS SB1(Serial3);       // UART3 Input_1 RX3-pin7, Output TX3=pin8
SBUS SB2(Serial2);       // UART2 Input_2 RX2-pin9, TX not used 

#define  switchPin 14    // A0
uint8_t switchPinState = 0;

enum swState { Port1 , Port2 };
swState inPort = Port1;

volatile int16_t ch[18];   // pwm
volatile uint8_t SB[25];   // byte  
 
bool failSafe;
bool lostFrame;

void setup() {
  #if defined  Debug_Input || defined Test_Output 
    Debug.begin(115200);
    delay(3500);
    Debug.println("Starting .... ");   
  #endif 
   
  pinMode(switchPin, INPUT_PULLUP);
  
  SB1.begin();                              // UART3 RX3-pin 7, TX3=pin 8
  SB2.begin();                              // UART2 RX2-pin 9
}

void loop() {
  
  switchPinState = digitalRead(switchPin);  // normally high

  if ((inPort == 0) && (switchPinState == LOW)) { 
    Debug.print("inPort ="); Debug.println(inPort);
    inPort = Port2;
    delay(2000);  // debounce
    }
    
   if ((inPort == 1) && (switchPinState == HIGH)) { 
        Debug.print("inPort ="); Debug.println(inPort);
    inPort = Port1;
    delay(2000);   ;
    }
    
  uint8_t result = 0;
  if (inPort == Port1) {    
     result = SB1.read(&ch[0], &failSafe, &lostFrame);
  } else 
  if (inPort == Port2) {
    result = SB2.read(&ch[0], &failSafe, &lostFrame);
  }
  
  if (result){
    
    SB1.write(&ch[0]);       // SBUS out Serial1 TX
    
    #ifdef Debug_Input
      Debug.printf("Port= %d", inPort+1);
      for (int i=0 ; i<16 ; i++) {   // 16 PWM channels
       Debug.printf(" %4d", ch[i]);
      }
      
      Debug.printf(" failsafe = %d  LostFrame = %d\n", failSafe, lostFrame );
    #endif  

 

   /*
    ChannelsToBytes();       // Channels to SBUS array - purely for testing

    #ifdef Test_Output
      BytesToChannels();
      for (int i=0 ; i<16 ; i++) {   // 16 PWM channels
       Debug.printf(">%4d", ch[i]);
      }
      Debug.printf("\n");
    #endif     
  */

  
  }
}

void ChannelsToBytes(){  //  16 PWM channels to 22 byte array
    uint8_t i;
    uint8_t SBbyte;
    uint8_t SBbit;
    uint8_t Servobit; 
        
    // flush SB array
    for (i=0; i<24; i++) {
      SB[i] = 0;
    } 
    
    SB[0] =0x0f;  //  Header byte

    uint8_t j = 0;
    Servobit = 0;
    SBbyte = 1;
    SBbit = 0;
    
    for (i=0; i<176; i++) {
      if (ch[j] & (1<<Servobit)) {      
        SB[SBbyte] |= (1<<SBbit);
      }
      SBbit++;
      Servobit++;
      if (SBbit == 8) {
        SBbit = 0;
        SBbyte++;
      }
      if (Servobit == 11) {
        Servobit =0;
        j++;
      }
    }
    
    // digi ch 1
    if (ch[16] == 1) {
      SB[23] |= (1<<0);
    }
    // digi ch 2
    if (ch[17] == 1) {
      SB[23] |= (1<<1);
    }
    
}
//**************************************************************
void BytesToChannels() {

  ch[0]  = ((SB[1]|SB[2]<< 8) & 0x07FF);
  ch[1]  = ((SB[2]>>3|SB[3]<<5) & 0x07FF);
  ch[2]  = ((SB[3]>>6|SB[4]<<2|SB[5]<<10) & 0x07FF);
  ch[3]  = ((SB[5]>>1|SB[6]<<7) & 0x07FF);
  ch[4]  = ((SB[6]>>4|SB[7]<<4) & 0x07FF);
  ch[5]  = ((SB[7]>>7|SB[8]<<1|SB[9]<<9) & 0x07FF);
  ch[6]  = ((SB[9]>>2|SB[10]<<6) & 0x07FF);
  ch[7]  = ((SB[10]>>5|SB[11]<<3) & 0x07FF); 
  ch[8]  = ((SB[12]|SB[13]<< 8) & 0x07FF);
  ch[9]  = ((SB[13]>>3|SB[14]<<5) & 0x07FF);
  ch[10] = ((SB[14]>>6|SB[15]<<2|SB[16]<<10) & 0x07FF);
  ch[11] = ((SB[16]>>1|SB[17]<<7) & 0x07FF);
  ch[12] = ((SB[17]>>4|SB[18]<<4) & 0x07FF);
  ch[13] = ((SB[18]>>7|SB[19]<<1|SB[20]<<9) & 0x07FF);
  ch[14] = ((SB[20]>>2|SB[21]<<6) & 0x07FF);
  ch[15] = ((SB[21]>>5|SB[22]<<3) & 0x07FF);

  // digi ch 1
  if (SB[23] & (1<<0)) {
    ch[16] = 1;
  }
  else{
    ch[16] = 0;
  }
  // digi ch 2
  if (SB[23] & (1<<1)) {
    ch[17] = 1;
  }
  else{
    ch[17] = 0;
  }
}

//***********************************************************

void WriteBytesSerial2() {   //  NOT USED - here for information

  elapsedMicros byteDuration=0;
  uint8_t i = 0;
  while (i < 25) {   //  22 bytes plus digi byte = 16 channels plus ch 17 digi channel               
    // Sending one byte of data takes 11bit in 8E1 which = 110us at 100000 baud, but we need one stopbit longer so we start the next sending after 120us not 110
    if (byteDuration >= 124){
      byteDuration = 0;
      Serial2.write(SB[i]); 
      i++; 
    }
  }
}
//***********************************************************

void WriteBytesSerial3() {

  elapsedMicros byteDuration=0;
  uint8_t i = 0;
  while (i < 25) {   //  22 bytes plus digi byte = 16 channels plus ch 17 digi channel               
    // Sending one byte of data takes 11bit in 8E1 which = 110us at 100000 baud, but we need one stopbit longer so we start the next sending after 120us not 110
    if (byteDuration >= 124){
      byteDuration = 0;
      Serial3.write(SB[i]); 
      i++; 
    }
  }
}
