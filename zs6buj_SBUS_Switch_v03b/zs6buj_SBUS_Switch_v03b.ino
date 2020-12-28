
/*
 *       SBus Automatice Switch1 by ZS6BUJ 1 November 2019
 *       Written for Teensy 3.2
 * 
 *       Library by Bolder_Flight_Systems: https://github.com/bolderflight/SBUS
 * 
 *        SBus input 1 Pin 7
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


// SBus Input 1 Pin 7
// SBus Input 2 Pin 9
// SBUS Output  Pin 8

#define Debug Serial     // USB
SBUS SB1(Serial3);       // UART3 Input_1 RX3-pin7, Output TX3=pin8
//SBUS SB2(Serial2);       // UART2 Input_2 RX2-pin9, TX not used 


  uint16_t    ch1[18];   // pwm values
  bool        failSafe1;
  bool        lostFrame1;
  uint8_t     result1;

  uint16_t    ch2[18];   // pwm values
  bool        failSafe2;
  bool        lostFrame2;
  uint8_t     result2;


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
 // SB2.begin();                              // UART2 RX2-pin 9
  
  blink_millis = millis();                  // for LED 

}

void loop() {

  if (millis() - active_millis  > 500) {  //  Timeout
    activeSB = 0;
  }

  result1 = 0;

  
  result1 = SB1.read(&ch1[0], &failSafe1, &lostFrame1);
//  sb[2].result1 = SB2.read(&sb[2].ch1[0], &sb[2].failSafe1, &sb[2].lostFrame1);
  
  if (result1) { 
    if (validPWM(1)) { 
      activeSB = 1; 
      active_millis = millis();  
      SB1.write(&ch1[0]);        // SBUS1 out Serial1 TX
      printSBus(1);
      digitalWrite(StatusLed, HIGH); 
    }}

  serviceLED();

}

void printSBus(uint8_t sbIn) {
     #ifdef Debug_Input
      Debug.printf("Input Port=%d", sbIn);
      for (int i=0 ; i<16 ; i++) {   // 16 PWM ch1annels
       Debug.printf(" %4d", ch1[i]);
      }
      
      Debug.printf(" failSafe1 = %d  lostFrame1 = %d\n", failSafe1, lostFrame1 );
    #endif   
}

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
