
/*
 *       SBus Automatice Switch by ZS6BUJ 1 November 2019
 *       Written for Teensy 3.2
 * 
 *       Library by Bolder_Flight_Systems: https://github.com/bolderflight/SBUS
 * 
 *        SBus input 1 Pin 7
 *        SBus input 2 Pin 9
 *        SBUS output  Pin 8
 *        
 *        Auto-switching. Primary = Input 1
 *      
 *        STATUS LED    Off = no signals,  solid on = input1 active,   blinking = input 2 active
 *        
 *        
 *       
 */
  
#include "SBUS.h"

#define StatusLed  13

#define Debug_Input


// SBus Input 1 Pin 7
// SBus Input 2 Pin 9
// SBUS Output  Pin 8

#define Debug Serial     // USB
SBUS SB1(Serial3);       // UART3 Input_1 RX3-pin7, Output TX3=pin8
SBUS SB2(Serial2);       // UART2 Input_2 RX2-pin9, TX not used 

struct sbus_t {
      uint16_t    ch[18];   // pwm values
      bool        failSafe;
      bool        lostFrame;
      uint8_t     result;
     };
sbus_t            sb[3];   // one extra for readability
uint8_t           activeSB = 0;
uint32_t          active_millis = 0;
uint32_t          blink_millis = 0;
uint8_t           ledStatus;
bool              vld;

void setup() {
  #if defined  Debug_Input || defined Test_Output 
    Debug.begin(115200);
    delay(3500);
    Debug.println("Starting .... ");   
  #endif 
   
  pinMode(StatusLed, OUTPUT); 
  digitalWrite(StatusLed, LOW); 
  
  SB1.begin();                              // UART3 RX3-pin 7, TX3=pin 8
  SB2.begin();                              // UART2 RX2-pin 9
  
  blink_millis = millis();                  // for LED 

}

void loop() {

  if (millis() - active_millis  > 500) {  //  Timeout
    activeSB = 0;
  }

  sb[1].result = 0;
  sb[2].result = 0;
  
  sb[1].result = SB1.read(&sb[1].ch[0], &sb[1].failSafe, &sb[1].lostFrame);
  sb[2].result = SB2.read(&sb[2].ch[0], &sb[2].failSafe, &sb[2].lostFrame);
  
  if (sb[1].result) { 
    if (validPWM(1)) { 
      activeSB = 1; 
      active_millis = millis();  
      SB1.write(&sb[1].ch[0]);        // SBUS1 out Serial1 TX
      printSBus(1);
      digitalWrite(StatusLed, HIGH); 
    }} else { 
    
    if (sb[2].result) { 
      if (validPWM(2)) { 
        activeSB = 2;
        active_millis = millis();
        SB1.write(&sb[2].ch[0]);      // SBUS2 out Serial1 TX  
        printSBus(2);
    }} 
  } 

  serviceLED();

}

void printSBus(uint8_t sbIn) {
     #ifdef Debug_Input
      Debug.printf("Input Port=%d", sbIn);
      for (int i=0 ; i<16 ; i++) {   // 16 PWM channels
       Debug.printf(" %4d", sb[sbIn].ch[i]);
      }
      
      Debug.printf(" failsafe = %d  LostFrame = %d\n", sb[sbIn].failSafe, sb[sbIn].lostFrame );
    #endif   
}

bool validPWM(uint8_t sbIn) {  // check the first 8 only
  bool valid = true;
  for (int i=0 ; i < 8 ; i++)  {

    if ((sb[sbIn].ch[i] < 100) || (sb[sbIn].ch[i] > 2500)) {
      valid = false;
    }
   // Debug.printf("valid %d ch %4d\n", valid, sb[sbIn].ch[i]);
  }
  return valid;
}

void serviceLED() {

  if (activeSB == 0) {
    digitalWrite(StatusLed, LOW);
  } else {
    if (activeSB == 01) {
      digitalWrite(StatusLed, HIGH);
    } else {
      if (activeSB == 02) {
        digitalWrite(StatusLed, ledStatus);      
        if (millis() - blink_millis > 200) {
          blink_millis = millis(); 
          if (ledStatus == LOW) {
            ledStatus = HIGH;
          } else {
            ledStatus = LOW;
          }
        }  
      }
    }
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
