
/*
 *       SBus to Mavlink RC override by ZS6BUJ December 2020
 *       Written for Teensy 3.2
 * 
 *       Library by Bolder_Flight_Systems: https://github.com/bolderflight/SBUS
 * 
 *       SBus in on RX pin of Serial1   pin  0   TX not used
 *       Mavlink RC out on RX/TX pins of Serial2 9 and 10
 *       
 *       SBus timing is as follows: 1bit = 10 uS. Byte = start + 8 bits + parity + stop + extra stop = 120uS
 *
 *       1 Header byte 00001111b (0x0F)
 *       16 x 11 bit channels -> 22 bytes
 *       1 Byte with two digital channels (channel 17 and 18) and "frame lost" and "failsafe" flags
 *       1 Footer byte 00000000b (0x00)
 *       FRAME TOTAL = 25 BYTES
 *       
 */
  
#include "SBUS.h"
#include <mavlink_types.h>
#include <ardupilotmega/mavlink.h>
#include <ardupilotmega/ardupilotmega.h>

#define Print_Input

#define Log                 Serial         // USB monitor

SBUS SBus(Serial1);

#define mvSerial            Serial2   
#define mav_baud            57600
#define mav_rxPin           9
#define mav_txPin           10

#define Device_sysid     251                     // Our Mavlink Identity - APM FC is 1, Mission Planner is 255, QGC default is 0 
#define Device_compid    MAV_COMP_ID_PERIPHERAL  // 158 Generic autopilot peripheral - APM FC is 1, MP is 190, QGC is  https://mavlink.io/en/messages/common.html
        
volatile int16_t ch[18];   // pwm 
 
bool failSafe;
bool lostFrame;

uint32_t  hb_count=0;
uint32_t  hb_millis=0;
uint32_t  sb_count=0;
uint32_t  sb_millis=0;
bool      sbGood = false;
bool      mavGood = false;
uint16_t  len;            // must be long !

mavlink_message_t   msg;
uint8_t             FCbuf[300];

// Mavlink Header
uint8_t    ap_sysid;                        // Sending system, return_to
uint8_t    ap_compid;
uint8_t    ap_targsys;                      // Target system 
uint8_t    ap_targcomp;

// Message #0  HEARTHBEAT 
uint8_t    ap_type_tmp = 0;              
uint8_t    ap_type = 0;
uint8_t    ap_autopilot = 0;
uint8_t    ap_base_mode = 0;
uint32_t   ap_custom_mode = 0;
uint8_t    ap_system_status = 0;
uint8_t    ap_mavlink_version = 0;

// Message #70 RC_Channels_Override
uint16_t  ap70_ch_raw[18];       // 16 + 2 channels, [0] thru [17] 

void setup() {

  Log.begin(115200);
  delay(2000);
  Log.println("Starting .... ");    

  SBus.begin();
  
  mvSerial.begin(mav_baud);    
  Log.printf("Mavlink serial on pins rx = %d and tx = %d\n", mav_rxPin, mav_txPin); 

}

void loop() {

  if (SBus.read(&ch[0], &failSafe, &lostFrame)){

    sb_count++;         
    if(!sbGood) {
      Log.printf("sb_count=%d\n", sb_count);
      if(sb_count >= 3) {        
        sbGood=true;
        sb_millis = millis();
        Log.println("SBus good!");  
      }
    }
    
    // remap values to regular uS range
    for (int i = 0 ; i < 18 ; i++) {
      if (*(ch+i) > 0) {
        *(ch+i) = map(*(ch+i), 172, 1811, 982, 2006);     
      }
    }    

    if ( (mavGood) && (sbGood) ){

      #if defined  Print_Input 
        for (int i=0 ; i<16 ; i++) {   // 16 PWM channels
         Log.printf(" %4d", ch[i]);
        }      
        Log.printf(" failsafe = %d  LostFrame = %d\n", failSafe, lostFrame );
      #endif 
      
      Write_Mavlink_RC_Ch_Override();
    }
  }

  Read_FC_And_Parse();  // just checking for good Mavlink heartbeat

  if(mavGood && (millis() - hb_millis) > 10000)  {   // if no heartbeat from APM for 10s then assume FC mav disconnected
    mavGood=false;
    Log.println("Heartbeat from FC timed out! FC not connected");      
    hb_count = 0;
   } 

  if(sbGood && (millis() - sb_millis) > 5000)  {   // if no good SB for 5s then assume SBUS disconnected
    sbGood=false;
    Log.println("SBUS timed out! SBUS not connected");      
   } 
  
}

//================================================================================================= 

bool Read_FC_And_Parse() {

    mavlink_status_t status;

    while(mvSerial.available()) { 
      byte c = mvSerial.read();
      if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {  // Read a frame             
        DecodeOneMavFrame();  
      }
    }
    return true;  
}

//================================================================================================= 
void Write_Mavlink_RC_Ch_Override() {  // #70
  ap_sysid = Device_sysid;                  
  ap_compid = Device_compid;                
  ap_targsys = 1;                             // APM FC
  ap_targcomp = 1;                            // APM FC

  // static inline uint16_t mavlink_msg_rc_channels_override_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
  //                             uint8_t target_system, uint8_t target_component, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint16_t chan13_raw, uint16_t chan14_raw, uint16_t chan15_raw, uint16_t chan16_raw, uint16_t chan17_raw, uint16_t chan18_raw)

 for (int i = 0 ; i < 18 ; i++) {
  ap70_ch_raw[i] = ch[i];           // from SBus buffer to #70 buffer
 }
  
  mavlink_msg_rc_channels_override_pack(ap_sysid, ap_compid, &msg, ap_targsys, ap_targcomp,
                   ap70_ch_raw[0], ap70_ch_raw[1], ap70_ch_raw[2], ap70_ch_raw[3],            
                   ap70_ch_raw[4], ap70_ch_raw[5], ap70_ch_raw[6], ap70_ch_raw[7],
                   ap70_ch_raw[8], ap70_ch_raw[9], ap70_ch_raw[10], ap70_ch_raw[11],  
                   ap70_ch_raw[12], ap70_ch_raw[13], ap70_ch_raw[14], ap70_ch_raw[15],   
                   ap70_ch_raw[16], ap70_ch_raw[17]);                                                                

  len = mavlink_msg_to_send_buffer(FCbuf, &msg);
  mvSerial.write(FCbuf,len);  
                    
 }

//================================================================================================= 

void DecodeOneMavFrame() {
  
   #if defined Mav_Print_All_Msgid
     uint16_t sz = sizeof(msg);
     Log.printf("FC  msgid = %3d Msg size =%3d\n",  msg.msgid, sz);
   #endif

   ap_sysid = msg.sysid;
   ap_compid = msg.compid;  

   switch(msg.msgid) {
    
        case MAVLINK_MSG_ID_HEARTBEAT:    // #0   https://mavlink.io/en/messages/common.html
          ap_type_tmp = mavlink_msg_heartbeat_get_type(&msg);   // Alex - don't contaminate the ap_type variable
          if (ap_type_tmp == 5 || ap_type_tmp == 6 || ap_type_tmp == 18 || ap_type_tmp == 26 || ap_type_tmp == 27) break;      
          // Ignore heartbeats from GCS (6) or Ant Trackers(5) or Onboard_Controllers(18) or Gremsy Gimbal(26) or ADSB (27))
          ap_type = ap_type_tmp;
          ap_autopilot = mavlink_msg_heartbeat_get_autopilot(&msg);
          ap_base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
          ap_custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
               
          ap_system_status = mavlink_msg_heartbeat_get_system_status(&msg);
          ap_mavlink_version = mavlink_msg_heartbeat_get_mavlink_version(&msg);
          hb_millis=millis(); 

          hb_count++; 
          
          if(!mavGood) {
            Log.printf("hb_count=%d\n", hb_count);

            if(hb_count >= 3) {        // If  3 heartbeats from MavLink then we are connected
              mavGood=true;
              Log.println("Mavlink good!");  
              }
            }
           
          #if defined Mav_Debug_FC_Heartbeat
            Log.print("Mavlink from FC #0 Heartbeat: ");  
            Log.print("ap_sysid="); Log.print(ap_sysid);   
            Log.print("  ap_compid="); Log.print(ap_compid);                      
            Log.print("  ap_type(frame)="); Log.print(ap_type);   
            Log.print("  ap_autopilot="); Log.print(ap_autopilot); 
            Log.print("  ap_base_mode="); Log.print(ap_base_mode); 
            Log.print(" ap_custom_mode="); Log.print(ap_custom_mode);
            Log.print("  ap_system_status="); Log.print(ap_system_status); 
            Log.print("  ap_mavlink_version="); Log.print(ap_mavlink_version);                    
            Log.println();
          #endif

          
          break;
                               
        default:
          if (!mavGood) break;
          #if defined Mav_Debug_All 
            Log.print("Mavlink from FC: ");
            Log.print("ID #");
            Log.print(msg.msgid);
            Log.println(" Ignored"); 
          #endif

          break;
      }
}
