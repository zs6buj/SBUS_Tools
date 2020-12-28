
/*
 *       SBus Automatice Switch1 by ZS6BUJ 1 November 2019
 *       Written for Teensy 3.2
 * 
 *       Library by Bolder_Flight_Systems: https://github.com/bolderflight/SBUS
 * 
 *        SBus input 1 Pin 0
 *        SBus input 2 Pin 9
 *        SBUS output  Pin 8
 *        
 *        Auto-switch1ing. Primary = Input 1
 *      
 *        STATUS LED    Off = no signals,  solid on = input1 active,   blinking = input 2 active
 *        
 *        
 *       
 */
  
#include "SBUS.h"

#define StatusLed  13

#define Debug_Input
#define Test_Integrity

// SBus Input 1 Pin 7
// SBus Input 2 Pin 9
// SBUS Output  Pin 8

#define Debug Serial     // USB

SBUS SB1(Serial1);       // Input_1 RX1-pin0, TX not used 
SBUS SB2(Serial2);       // Input_2 RX2-pin9, TX not used 

  uint16_t    ch1[18];   // pwm values from SB1
  bool        failSafe1;
  bool        lostFrame1;
  uint8_t     result1;

  uint16_t    ch2[18];   // pwm values from SB2
  bool        failSafe2;
  bool        lostFrame2;
  uint8_t     result2;

  uint16_t    ch3[18];   // pwm values for check only
  bool        failSafe3;
  bool        lostFrame3;
  uint8_t     result3;
  
  uint8_t           activeSB = 0;
  uint32_t          active_millis = 0;
  uint32_t          blink_millis = 0;
  uint8_t           ledStatus;
  bool              vld;

  volatile uint8_t Byts[25];   // byte  

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
  Serial3.begin(100000, SERIAL_8E1_TXINV);
  
  blink_millis = millis();                  // for LED 

}

void loop() {

  if (millis() - active_millis  > 500) {  //  Timeout
    activeSB = 0;
  }

  result1 = 0;

  
  result1 = SB1.read(&ch1[0], &failSafe1, &lostFrame1);
  result2 = SB2.read(&ch2[0], &failSafe2, &lostFrame2);

  
  if (result1) { 
    if (validPWM(1)) { 
      activeSB = 1; 
      active_millis = millis(); 

      SB1_To_Byts();             // Channels to byte array
      #ifdef Test_Integrity
        Test_Integrity1();
      #endif 
      
      WriteBytsSB1();
       
      printSBus(1);
      digitalWrite(StatusLed, HIGH); 
    }}else { 
    
    if (result2) { 
      if (validPWM(2)) { 
        activeSB = 2;
        active_millis = millis();
        
        SB2_To_Byts();            // Channels to byte array
        #ifdef Test_Integrity
          Test_Integrity2();
        #endif 
        
        WriteBytsSB2();
        
        printSBus(2);
    }} 
  } 

  serviceLED();

}
//***********************************************************
void printSBus(uint8_t sbIn) {
     #ifdef Debug_Input
      Debug.printf("Input Port=%d", sbIn);
      for (int i=0 ; i<16 ; i++) {   // 16 PWM ch1annels
        if (sbIn == 1) {
          Debug.printf(" %4d", ch1[i]);
        } else {
          Debug.printf(" %4d", ch2[i]);
        }
      }
      if (sbIn == 1) {
        Debug.printf(" failSafe1 = %d  lostFrame1 = %d\n", failSafe1, lostFrame1 );
        } else {
        Debug.printf(" failSafe2 = %d  lostFrame2 = %d\n", failSafe2, lostFrame2 );
        } 
    #endif   
}
//***********************************************************
bool validPWM(uint8_t sbIn) {  // ch1eck the first 8 only
  bool valid = true;
  for (int i=0 ; i < 8 ; i++)  {

    if ((ch1[i] < 100) || (ch1[i] > 2500)) {
      valid = false;
    }
   // Debug.printf("valid %d ch1 %4d\n", valid, ch1[i]);
  }
  return valid;
}
//***********************************************************
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
//***********************************************************
void SB1_To_Byts(){  //  16 PWM channels to 22 byte array
    uint8_t i;
    uint8_t SBbyte;
    uint8_t SBbit;
    uint8_t Servobit; 
        
    // flush Byt array
    for (i=0; i<24; i++) {
      Byts[i] = 0;
    } 
    
    Byts[0] =0x0f;  //  Header byte

    uint8_t j = 0;
    Servobit = 0;
    SBbyte = 1;
    SBbit = 0;
    
    for (i=0; i<176; i++) {
      if (ch1[j] & (1<<Servobit)) {      
        Byts[SBbyte] |= (1<<SBbit);
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
    if (ch1[16] == 1) {
      Byts[23] |= (1<<0);
    }
    // digi ch 2
    if (ch1[17] == 1) {
      Byts[23] |= (1<<1);
    }
    
}
//***********************************************************
void SB2_To_Byts(){  //  16 PWM channels to 22 byte array
    uint8_t i;
    uint8_t SBbyte;
    uint8_t SBbit;
    uint8_t Servobit; 
        
    // flush Byts array
    for (i=0; i<24; i++) {
      Byts[i] = 0;
    } 
    
    Byts[0] =0x0f;  //  Header byte

    uint8_t j = 0;
    Servobit = 0;
    SBbyte = 1;
    SBbit = 0;
    
    for (i=0; i<176; i++) {
      if (ch2[j] & (1<<Servobit)) {      
        Byts[SBbyte] |= (1<<SBbit);
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
    if (ch2[16] == 1) {
      Byts[23] |= (1<<0);
    }
    // digi ch 2
    if (ch2[17] == 1) {
      Byts[23] |= (1<<1);
    }
    
}
//**************************************************************
void BytesToChannels() {

  ch3[0]  = ((Byts[1]|Byts[2]<< 8) & 0x07FF);
  ch3[1]  = ((Byts[2]>>3|Byts[3]<<5) & 0x07FF);
  ch3[2]  = ((Byts[3]>>6|Byts[4]<<2|Byts[5]<<10) & 0x07FF);
  ch3[3]  = ((Byts[5]>>1|Byts[6]<<7) & 0x07FF);
  ch3[4]  = ((Byts[6]>>4|Byts[7]<<4) & 0x07FF);
  ch3[5]  = ((Byts[7]>>7|Byts[8]<<1|Byts[9]<<9) & 0x07FF);
  ch3[6]  = ((Byts[9]>>2|Byts[10]<<6) & 0x07FF);
  ch3[7]  = ((Byts[10]>>5|Byts[11]<<3) & 0x07FF); 
  ch3[8]  = ((Byts[12]|Byts[13]<< 8) & 0x07FF);
  ch3[9]  = ((Byts[13]>>3|Byts[14]<<5) & 0x07FF);
  ch3[10] = ((Byts[14]>>6|Byts[15]<<2|Byts[16]<<10) & 0x07FF);
  ch3[11] = ((Byts[16]>>1|Byts[17]<<7) & 0x07FF);
  ch3[12] = ((Byts[17]>>4|Byts[18]<<4) & 0x07FF);
  ch3[13] = ((Byts[18]>>7|Byts[19]<<1|Byts[20]<<9) & 0x07FF);
  ch3[14] = ((Byts[20]>>2|Byts[21]<<6) & 0x07FF);
  ch3[15] = ((Byts[21]>>5|Byts[22]<<3) & 0x07FF);

  // digi ch 1
  if (Byts[23] & (1<<0)) {
    ch3[16] = 1;
  }
  else{
    ch3[16] = 0;
  }
  // digi ch 2
  if (Byts[23] & (1<<1)) {
    ch3[17] = 1;
  }
  else{
    ch3[17] = 0;
  }
}

//***********************************************************

void WriteBytsSB1() {

  elapsedMicros byteDuration=0;
  uint8_t i = 0;
  while (i < 25) {   //  22 bytes plus digi byte = 16 channels plus ch 17 digi channel               
    // Sending one byte of data takes 11bit in 8E1 which = 110us at 100000 baud, but we need one stopbit longer so we start the next sending after 120us not 110
    if (byteDuration >= 124){
      byteDuration = 0;
      Serial3.write(Byts[i]); 
      i++; 
    }
  }
}
//***********************************************************

void WriteBytsSB2() {

  elapsedMicros byteDuration=0;
  uint8_t i = 0;          
  while (i < 25) {    
    if (byteDuration >= 124){
      byteDuration = 0;
      Serial3.write(Byts[i]); 
      i++; 
    }
  }
}
//***********************************************************
bool Test_Integrity1() {
  bool integ = true;
  BytesToChannels();
  for (int i=0 ; i<16 ; i++) {   // 16 PWM channels
   if (ch1[i] != ch3[i]) integ = false;
   Debug.printf(">%4d", ch3[i]);
  }
  Debug.printf("Integrity=%1d \n", integ);
  return integ;
}      
//***********************************************************
bool Test_Integrity2() {
  bool integ = true;
  BytesToChannels();
  for (int i=0 ; i<16 ; i++) {   // 16 PWM channels
   if (ch2[i] != ch3[i]) integ = false;
   Debug.printf(">%4d", ch3[i]);
  }
  Debug.printf("Integrity=%1d \n", integ);
  return integ;
}  
