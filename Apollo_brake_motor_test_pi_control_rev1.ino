/////////////////////////////////////////////////////////////
//   Hankyoung National University                         //
//   School of ICT, Robotics & Mechanical Engineering      //
//   Team Apollo 2025                                      //
/////////////////////////////////////////////////////////////
//                                                         //
//              STEER ECU CODE for Testweek                //
//                                        2025.09.20       //
/////////////////////////////////////////////////////////////
// Managed by                                              //
//   Gwangrok Lee                                          //
//   loks96@hknu.ac.kr                                     //
//   010-8371-7760                                         //
//                                                         //
// Source code developer                                   //
//   Hohyeon Jeong                                         //
//   gulimpan@hknu.ac.kr                                   //
//   010-7277-7436                                         //
//   &                                                     //
//   Junwoo Jang                                           //
//   junu0715@naver.com                                    //
//   010-3250-8086                                         //
/////////////////////////////////////////////////////////////

//============================================================//
//           1. Libraries                                     //
//           2. Pin settings                                  //
//           3. CAN ID define                                 //
//           4. Global variables                              //
//                                                            //
//           5. Void setup (for arduinoPINs / mcp2515)        //
// ★★★★★ 6. Void loop                                     //
//                                                            //
//    ★★★ 7. LSC derivation function (c++ Method)           //
//    ★★★ 8. TVC Drive function (c++ Method)                //
//           9. Mode bitReadRange function (c++ Method)       //
//============================================================//



//============================================================//
//           1. Libraries                                     //
//============================================================//
#include <SPI.h>         //for arduino SPI interfacing (mcp2515 module)
#include <mcp_can.h>     //Library(.zip) : https://github.com/Flori1989/MCP2515_lib

#include <EEPROM.h>      //for arduino EEPROM : Gain value save in eeprom (Permanent storage space of arduino)

#include <SoftwareSerial.h>  // to use arduino digitalPin as TTL signal port (for "TTL to RS485" module)
#include "CRC16.h"           // Library : "CRC" (ver 0.1.2) by "Rob Tillaart"   ---How to Use---> (youtube #344, "1:19:30" https://www.youtube.com/watch?v=Xp1cToOwmCQ&list=PLj5NnUk28LOeUCGXlJz1MuNssLDhALhYL)
#include "CRC.h"             // Library : "CRC" (ver 0.1.2) by "Rob Tillaart"   ---How to Use---> (youtube #344, "1:19:30" https://www.youtube.com/watch?v=Xp1cToOwmCQ&list=PLj5NnUk28LOeUCGXlJz1MuNssLDhALhYL)
// CRC is "Cyclic Redundancy Check" to check RS485 modbus msg (Like "check sum")



//============================================================//
//           2. Pin settings                                  //
//============================================================//
#define SPI_CS_PIN 10  // SPI CS pin  (for "SPI to CAN" module : mcp2515)
#define TTL_Rx_PIN 7       // TTL Receive pin  (for "TTL to RS485" module)

#define TTL_Tx_PIN 8       // TTL Transmit pin   (for "TTL to RS485" module)

# define vr_pin A1

/*
#define brk_pwm_pin 6        
#define brk_activation_pin 7      // <-- need check!!!!!!!!!!!!!
#define brk_disactivation_pin 8   // <-- need check!!!!!!!!!!!!!
#define vess_pwm_pin 9

#define pdl_pin A7                     // from acc pedal
#define force_greenlight_btn_pin A1    // force greenlight button
#define change_seg_btn_pin A2          // change segment button
#define TVC_btn_pin A3                 // Torque Vectoring Contol mode change button
#define LSC_btn_pin A4                 // Limit Speed Contol ON/OFF button
#define minus_btn_pin A5               // gain tuning button (-)
#define plus_btn_pin A6                // gain tuning button (+)
#define redlight_led_pin A0                // redlight LED
*/


//============================================================//
//           3. CAN ID define                                 //
//============================================================//
#define id_mainECU 0x100    // Enter SAME VALUE into all ECUs !!! //
#define id_strECU 0x101     // Enter SAME VALUE into all ECUs !!! //
#define id_imuECU 0x102     // Enter SAME VALUE into all ECUs !!! //
#define id_rpmECU 0x103     // Enter SAME VALUE into all ECUs !!! //



//============================================================//
//           4. Global variables                              //
//============================================================//
// APOLLO ECUs Common variables
const float str_scale_factor = 20.0;      // Enter SAME VALUE into "TRACTION ECU" !!! ///// <- need to be checked
const float gain_str_scale_factor = 10.0; // Enter SAME VALUE into "SEG&LCD ECU" !!! ///// <- need to be checked

// Vehicle & control Parameters
const float steer_ratio = 1 ;  ///// <- need to be checked !!!!!!!!!!!!!!!!!!!!!!!!
const float max_deg = 25.0 ;        ///// <- need to be checked
const float min_deg = -25.0 ;       ///// <- need to be checked
const float Kp = 5 ;
const float Ki = 1 ;
const float Kd = 0.1 ;
const float compensation_steer_DEG  = 0 ;

// Object creation
MCP_CAN CAN0(SPI_CS_PIN);
SoftwareSerial RS485(TTL_Rx_PIN, TTL_Tx_PIN);  //Works as (Rx_pin, Tx_pin)
CRC16 crc;

// for Parse Received CAN message 
// [ 8.Void loop -> (2) ]
unsigned long rxId;
byte len;
byte rxBuf[8];   //length of rxBuf needs to be "8" for Expansion ability


// for Received CAN data save    // <- need to be checked
// [ 8.Void loop -> (2) ]
byte modes_of_car = 0;
byte ref_acc = 0;
byte ref_regen = 0;
byte gain = 0;
byte ref_str_High = 0;
byte ref_str_Low = 0;

// for Gain Tuning
// [ 6.Void loop -> (5) ]
const int AdressShift = 0 ;  //0, 1, 2, 3, 4, 5, ...
int seg_mode_pre = 0 ;
int gain_pre = 0 ;

// for Transmit CAN message data frame
// [ 8.Void loop -> (7) ]
const uint8_t can_dlc = 2;    //Data Length Code(Bytes)
byte data[can_dlc] = {0}; 

// for CAN Transmit period 
// [ 8.Void loop -> (7) ]
unsigned long lastSend = 0;
const unsigned long sendInterval = 50; // [ms]

unsigned long timer_value;
unsigned long timer_old;
float dt;

float errorOld = 0.0;
float errorSum = 0.0;
int ref_str_deg;
int flag = 0;
int count_push;
int lastButtonState = 0;

//============================================================//
//           5. Void setup (for arduinoPINs / RS485 / mcp2515)//
//============================================================//
void setup(){
  pinMode(12, INPUT);
  RS485.begin(115200);
  Serial.begin(9600);
  while (RS485.available()) {
    RS485.read();    // Buffer clear
  }

 // while (CAN0.begin(CAN_500KBPS, MCP_8MHz) != CAN_OK) {
 //   delay(100);
 // }
//  APOLLO_set_CAN_id_Filter_for_STEERecu() ;
  unsigned long timer_old = millis();

}



//============================================================//
// ★★★★★ 6. Void loop                                     //
//============================================================//
void loop(){
  timer_value = millis();
  dt = (timer_value - timer_old)*0.001;

  
  
  int buttonState = digitalRead(12);
  
  if (buttonState != lastButtonState && buttonState == HIGH) {
     count_push++;
     
    
     
     delay(50);
  }
  
   
  if(count_push%2 == 1){
    ref_str_deg = 20;
  }
  else if(count_push%2 == 0){
    ref_str_deg = -20;
  }
  
      
  
  // 4-2) Steer Process - Steering angle derivation [deg]
  float str_deg = (-1) *( analogRead(vr_pin)-512 ) *(300.0/1023.0) *steer_ratio + compensation_steer_DEG ;
  
  // 4-3) Steer Process - Proportional control : "steer Error" based "steer motor RPM" derivation
  //float e = ref_str_deg - str_deg ;
 // float str_rpm = Kp*e;


  float str_rpm1 = speedPIControl(dt,str_deg,ref_str_deg,Kp,Ki,Kd); 

  
  float str_rpm = constrain(str_rpm1,-2500,2500);
  
  // 5) RS485modbus msg tramsmit(to steer motor) + CAN msg transmit
  unsigned long now = millis();
  if (now - lastSend >= sendInterval) {
    APOLLO_strMotor_Startup();
    delay (10);

    APOLLO_strMotor_rpmWrite(str_rpm);

    uint16_t str_scaled = str_deg*str_scale_factor;
    byte str_High = str_scaled >> 8;
    byte str_Low = str_scaled & 0xFF;
    data[0] = str_High;
    data[1] = str_Low ;
   // CAN0.sendMsgBuf(id_strECU, 0, can_dlc, data);
    
    lastSend = now;
  }

   
  timer_old = timer_value;  
  lastButtonState = buttonState;
  Serial.print("simtime : ");
  Serial.print(timer_value);
  Serial.print("  Ref_deg : ");
  Serial.print(ref_str_deg);
  Serial.print("  Cur_deg : ");
  Serial.print(str_deg);
  Serial.print("  str_rpm : ");
  Serial.print(str_rpm);
  Serial.print("  but : ");
  Serial.print(buttonState);
  Serial.print("  flag : ");
  Serial.println(count_push);
}

float speedPIControl(float DT, float input, float setPoint,  float Kp,float Ki, float Kd)
{
  float error;
  float output;
  
  error = setPoint-input;
  float error_dot = (error-errorOld)/DT;
  
  errorSum = errorSum + error*DT;

  output = Kp*error + Ki*errorSum + Kd*error_dot;
  errorOld = error;
  return(output);
}




/*

//============================================================//
//          9. Mode bitReadRange function (c++ Method)        //
//============================================================//     //LSB is Bit[0] (ex [bit7 bit6 bit5 bit4 bit3 bit2 bit1 bit0])
uint8_t APOLLO_bitReadRange(byte data, uint8_t startBit, uint8_t endBit) {  
  uint8_t length = endBit - startBit + 1;
  uint8_t mask = (1 << length) - 1;                      //Make serial mask by length (ex 00000111)
  uint8_t ans = (data >> startBit) & mask;
  return ans;
}
*/

//============================================================//
//      12. Str motor RPMcmd transmit function (c++ Method)   //  (for "TTL to RS485" module)
//============================================================//
void APOLLO_strMotor_rpmWrite(float str_vel) {
  while (RS485.available()) {
    RS485.read();    // Buffer clear
  }
  int32_t vel_cmd = round(str_vel);
  byte cmd_H = vel_cmd >> 8;       // Split vel_cmd into 2 bytes (High byte)
  byte cmd_L = vel_cmd & 0xFF;     // Split vel_cmd into 2 bytes (Low byte)
  //byte request[8] = { 0x01, 0x06, 0x00, 0x79, cmd_H, cmd_L, 0x00, 0x00 };
  byte request[] = { 0x01, 0x06, 0x00, 0x79, cmd_H, cmd_L };// +{, request_crc_low, request_crc_high}

  uint16_t request_crc = crc16(request, sizeof(request), 0x8005, 0xFFFF, 0, true, true);  //Derivation CRC of request array
  byte request_crc_high = request_crc >> 8;    // Split CRC into 2 bytes (High byte)
  byte request_crc_low = request_crc & 0xFF;   // Split CRC into 2 bytes (Low byte)
  request[6] = request_crc_low;
  request[7] = request_crc_high;

  RS485.write(request, 8);
  RS485.flush();

  delay(5);
}

void APOLLO_strMotor_Startup() {
  while (RS485.available()) {
    RS485.read();    // Buffer clear
  }
  byte request[] = { 0x01, 0x06, 0x00, 0x78, 0x00, 0x01 };// +{, request_crc_low, request_crc_high}

  uint16_t request_crc = crc16(request, sizeof(request), 0x8005, 0xFFFF, 0, true, true);
  byte request_crc_high = request_crc >> 8;    // Split CRC into 2 bytes (High byte)
  byte request_crc_low = request_crc & 0xFF;   // Split CRC into 2 bytes (Low byte)
  request[6] = request_crc_low;  // Split CRC into 2 bytes (High byte)
  request[7] = request_crc_high; // Split CRC into 2 bytes (Low byte)

  RS485.write(request, 8);
  RS485.flush();

  delay(5);
}

/*

//============================================================//
//       ★ 12. setup for CAN id Filter function(c++ Method)  //  //(for Standard id : 11bit [0x7FF])     --> APOLLO uses Standard Frame (Default)
//============================================================//  //     Extended id : 29bit [0x1FFFFFFF]
void APOLLO_set_CAN_id_Filter_for_STEERecu() {
  //CAN0.setMode(CAN_CONFIGURATION_MODE);
    CAN0.init_Mask(0, 0, 0x7FF);  // Mask 0 (Standard frame)  : RxBuf0                             //MCP2515 has 2 RxBuffer. Buffer0 can be seted 0,1 fillter
      CAN0.init_Filt(0, 0, id_mainECU);        // Allow to fill Buffer0 with CAN msg has id_mainECU
      //CAN0.init_Filt(1, 0, id_mainECU);
    CAN0.init_Mask(1, 0, 0x7FF);  // Mask 1 (Standard frame)  : RxBuf1                             //MCP2515 has 2 RxBuffer. Buffer1 can be seted 2,3,4,5 fillter
      CAN0.init_Filt(2, 0, id_mainECU);        // Allow to fill Buffer1 with CAN msg has id_mainECU
      //CAN0.init_Filt(3, 0, id_mainECU);
      //CAN0.init_Filt(4, 0, id_mainECU);
      //CAN0.init_Filt(5, 0, id_mainECU);
  //CAN0.setMode(CAN_NORMAL_MODE);
}*/
