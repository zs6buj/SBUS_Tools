
/*
 *       SBus Splitter by ZS6BUJ October 2019
 *       Written for Teensy 3.2
 * 
 *       Library by Bolder_Flight_Systems: https://github.com/bolderflight/SBUS
 * 
 *       SBus in on RX pin of Serial1   pin  0
 *       SBus out1 on TX pin of Serial1 pin  1
 *       SBus out2 on TX pin of Serial2 pin 10
 *       
 *       Timing is as follows: 1bit = 10 uS. Byte = start + 8 bits + parity + stop + extra stop = 120uS
 *
 *       1 Header byte 00001111b (0x0F)
 *       16 x 11 bit channels -> 22 bytes
 *       1 Byte with two digital channels (channel 17 and 18) and "frame lost" and "failsafe" flags
 *       1 Footer byte 00000000b (0x00)
 *       FRAME TOTAL = 25 BYTES
 *       
 2019-10-14 v05 Added output test. Convert bytes back to pwm and compare.
  */
  
#include "SBUS.h"

#define Print_Input
//#define Test_Output

SBUS SB1(Serial1);

volatile int16_t ch[18];   // pwm
volatile uint8_t SB[25];   // byte  
 
bool failSafe;
bool lostFrame;

void setup() {
  #if defined  Print_Input || defined Test_Output
    Serial.begin(115200);
    delay(3500);
    Serial.println("Starting .... ");   
  #endif  

  SB1.begin();
  Serial2.begin(100000, SERIAL_8E1_TXINV);
  Serial3.begin(100000, SERIAL_8E1_TXINV);

}

void loop() {

  if (SB1.read(&ch[0], &failSafe, &lostFrame)){
    #ifdef Print_Input
      for (int i=0 ; i<16 ; i++) {   // 16 PWM channels
       Serial.printf(" %4d", ch[i]);
      }
      
      Serial.printf(" failsafe = %d  LostFrame = %d\n", failSafe, lostFrame );
    #endif  

    SB1.write(&ch[0]);       // SBUS out Serial1 TX

    ChannelsToBytes();       // Channels to SBUS array

    #ifdef Test_Output
      BytesToChannels();
      for (int i=0 ; i<16 ; i++) {   // 16 PWM channels
       Serial.printf(">%4d", ch[i]);
      }
      Serial.printf("\n");
    #endif  
    
    WriteBytesSerial2();

   // WriteBytesSerial3();
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

void WriteBytesSerial2() {

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
