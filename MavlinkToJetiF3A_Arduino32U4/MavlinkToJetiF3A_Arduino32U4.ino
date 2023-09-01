/* 
  Jeti Sensor EX Telemetry C++ Library
  
  Simple Main program
  --------------------------------------------------------------------
  
  Copyright (C) 2023 Ruud Kroes

V2  added pitch and roll
    corrected yaw and alt value (/10 less)
    commented out delta time (test value for Mavlink speed)
    
**************************************************************/
#include <SoftwareSerial.h>
#include <mavlink.h>
#include "JetiExProtocol.h"

SoftwareSerial mySerial(10, 11); // RX, TX

JetiExProtocol jetiEx;
//unsigned long mstime = 0;
//unsigned long mstimeold = 0;
//unsigned long delta = 0;
enum
{
  ID_LAT = 1,
  ID_LON,
  ID_YAW,
  ID_ALT,
//  ID_DELTA,
  ID_PITCH,
  ID_ROLL,
};

// id from 1..15
JETISENSOR_CONST sensors[] PROGMEM =
{
  // id            name              unit         data type             precision 0->0, 1->0.0, 2->0.00
  { ID_LAT,        "F3A_LAT",        "",         JetiSensor::TYPE_30b, 0 },
  { ID_LON,        "F3A_LON",        "",         JetiSensor::TYPE_30b, 0 },
  { ID_YAW,        "F3A_YAW",        "",         JetiSensor::TYPE_14b, 1 }, 
  { ID_ALT,        "F3A_ALT",        "m",        JetiSensor::TYPE_14b, 1 },
//  { ID_DELTA,      "F3A_DELTA",      "ms",       JetiSensor::TYPE_14b, 0 },
  { ID_PITCH,      "F3A_PITCH",      "deg",      JetiSensor::TYPE_14b, 0 },
  { ID_ROLL,       "F3A_ROLL",       "deg",      JetiSensor::TYPE_14b, 0 },
  0 // end of array
};

void setup()
{
  //Serial.begin(115200); // debug poort 
  mySerial.begin(115200);  // software serial voor mavlink
  jetiEx.Start( "F3A", sensors );

}

void loop()
{
  //mstime = micros();
  MavLink_receive();
  jetiEx.DoJetiSend(); 
}



//function called by arduino to read any MAVlink messages sent by serial communication from flight controller to arduino
void MavLink_receive()
  { 
  mavlink_message_t msg;
  mavlink_status_t status;
 
  while(mySerial.available())
  {
    uint8_t c= mySerial.read();
    //Serial.println(c);
    //Get new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
 
    //Handle new message from autopilot
      switch(msg.msgid)
      {
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        {
          mavlink_global_position_int_t packet;
          mavlink_msg_global_position_int_decode(&msg, &packet);
          jetiEx.SetSensorValue( ID_LAT,  packet.lat);
          jetiEx.SetSensorValue( ID_LON,  packet.lon);
          jetiEx.SetSensorValue( ID_YAW,  packet.hdg /100);
          jetiEx.SetSensorValue( ID_ALT,  packet.alt /1000);
          //delta = mstime - mstimeold;
          //jetiEx.SetSensorValue( ID_DELTA,  delta /1000);
          //Serial.println (delta);
          //mstimeold = mstime;
          break;
        }
  
        case MAVLINK_MSG_ID_ATTITUDE:
        {
          mavlink_attitude_t packet;
          mavlink_msg_attitude_decode(&msg, &packet);
          jetiEx.SetSensorValue( ID_PITCH, ((packet.pitch * 4068) / 71));
          jetiEx.SetSensorValue( ID_ROLL, ((packet.roll * 4068) / 71));
          break;
        }
        break;
      }
    }
  }
}
