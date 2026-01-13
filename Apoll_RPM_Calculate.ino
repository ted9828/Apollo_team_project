#include <SPI.h> 
#include <mcp2515.h>
MCP2515 mcp2515(10);

#define slave1 0x01
#define job1 0x00 

struct can_frame canMsg1;

float rev_R = 0;
unsigned int rpm_R;
float rev_L = 0;
unsigned int rpm_L;
unsigned long pretime;
unsigned long deltatime;
int interruptPin_R = 3;
int interruptPin_L = 2;

void isr_R()
{
rev_R++;
}

void isr_L()
{
rev_L++;
}

void setup()
{
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();
  Serial.begin(9600);
}



void loop()
{ 

  attachInterrupt(digitalPinToInterrupt(interruptPin_R),isr_R,CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin_L),isr_L,CHANGE);
  delay(1000);
  detachInterrupt(digitalPinToInterrupt(interruptPin_R));
  detachInterrupt(digitalPinToInterrupt(interruptPin_L));
  deltatime = micros()-pretime;
  rpm_R=((rev_R/32)/deltatime)*60000000;
  rpm_L=((rev_L/32)/deltatime)*60000000;
  pretime = micros();
  rev_R=0;
  Serial.println(rpm_L);
  rev_L=0;


  uint16_t Motor_RPM_R = rpm_R;
  uint16_t Motor_RPM_L = rpm_L;
  canMsg1.can_id  = slave1;
  canMsg1.can_dlc = 4;
  canMsg1.data[0] = 0x00;
  canMsg1.data[1] = 0x00;
  canMsg1.data[2] = 0x00;
  canMsg1.data[3] = 0x00;
  byte Motor_RPM_R_high = Motor_RPM_R >> 8;
  byte Motor_RPM_R_low = Motor_RPM_R & 0xFF;
  byte Motor_RPM_L_high = Motor_RPM_L >> 8;
  byte Motor_RPM_L_low = Motor_RPM_L & 0xFF;
  canMsg1.data[0] = Motor_RPM_R_high;
  canMsg1.data[1] = Motor_RPM_R_low;
  canMsg1.data[2] = Motor_RPM_L_high;
  canMsg1.data[3] = Motor_RPM_L_low;
  mcp2515.sendMessage(&canMsg1);
  
}