
/* 
  Jeti EX Telemetry sensor decoder C++ Library
  
  RxJetiEx.ino - Example printing telemetry data to Serial
                 Use with Arduino Micro Pro, Leonardo or Teensy 3.x

                 For Teensy: uncomment #define SERIAL_9BIT_SUPPORT in
                 ...\Arduino\hardware\teensy\avr\cores\teensy3\HardwareSerial.h
  -------------------------------------------------------------
  
  Copyright (C) 2022 Bernd Wokoeck
  
  Version history:
  0.99   02/09/2022  created

**************************************************************/

// spiffs: https://github.com/espressif/arduino-esp32/tree/master/libraries/SPIFFS

#include "RxJetiExDecode.h"

RxJetiDecode jetiDecode;

void setup()
{
  Serial.begin(19200);
  jetiDecode.Start( RxJetiDecode::SERIAL1 );  // for devices with more than one UART (i.e. Teensy): jetiDecode.Start( RxJetiDecode::SERIAL1..3 );
}

void loop()
{
  RxJetiExPacket * pPacket;

  while( ( pPacket = jetiDecode.GetPacket() ) != NULL ) 
  {
    switch( pPacket->GetPacketType() )
    {
    case RxJetiExPacket::PACKET_NAME:
      {
        RxJetiExPacketName * pName = (RxJetiExPacketName *)pPacket;
        PrintName( pName );
      }
      break;
    case RxJetiExPacket::PACKET_LABEL:
      {
        RxJetiExPacketLabel * pLabel = (RxJetiExPacketLabel *)pPacket;
        PrintLabel( pLabel );
      }
      break;
    case RxJetiExPacket::PACKET_VALUE:
      {
        RxJetiExPacketValue * pValue = (RxJetiExPacketValue *)pPacket;
        PrintValue( pValue );
      }
      break;
    case RxJetiExPacket::PACKET_ALARM:
      {
        RxJetiPacketAlarm  * pAlarm = (RxJetiPacketAlarm *)pPacket;
        PrintAlarm( pAlarm );
      }
      break;
    case RxJetiExPacket::PACKET_TEXT:
      {
        RxJetiPacketText * pText = (RxJetiPacketText *)pPacket;
        Serial.println( pText->m_textBuffer );
      }
      break;
    case RxJetiExPacket::PACKET_ERROR:
      Serial.println( "Invalid CRC  -----------------------" ); 
      break;
    }
  }
  // delay( 10 ); <-- don't put a delay here, because you will get buffer overruns 
}

void PrintName( RxJetiExPacketName * pName )
{
  char buf[50];
  sprintf(buf, "Sensor - Serial: %08lx", pName->GetSerialId() ); Serial.println( buf ); 
  sprintf(buf, "Name: %s", pName->GetName() ); Serial.println( buf ); 
  Serial.println( "!!!!!!!!!!!" );
}

void PrintLabel( RxJetiExPacketLabel * pLabel )
{
  char buf[50];
  sprintf(buf, "Label from %s, Serial: %08lx/%d", pLabel->GetName(), pLabel->GetSerialId(), pLabel->GetId() ); Serial.println( buf ); 
  sprintf(buf, "Label - %s, Unit: %s", pLabel->GetLabel(), pLabel->GetUnit() ); Serial.println( buf ); 
  Serial.println( "++++++++++" );
}

void PrintValue( RxJetiExPacketValue * pValue )
{
  char buf[50];
  float fValue;
  uint8_t day;  uint8_t month;  uint16_t year;
  uint8_t hour; uint8_t minute; uint8_t second;

  sprintf(buf, "Value from %s, Serial: %08lx/Id: %d", pValue->GetName(), pValue->GetSerialId(), pValue->GetId() ); Serial.println( buf ); 
  sprintf(buf, "Val - %s: ", pValue->GetLabel() ); Serial.print( buf ); 
  if( pValue->GetFloat( &fValue ) )
  {
    Serial.print( fValue );
  }
  else if( pValue->GetLatitude( &fValue ) )
  {
    Serial.print( fValue, 5 );
  }
  else if( pValue->GetLongitude( &fValue ) )
  {
    Serial.print( fValue, 5 );
  }
  else if( pValue->GetDate( &day, &month, &year ) )
  {
    sprintf( buf, "%d.%d.%d", day, month, year ); Serial.print( buf );
  }
  else if( pValue->GetTime( &hour, &minute, &second ) )
  {
    sprintf( buf, "%d:%d:%d", hour, minute, second ); Serial.print( buf );
  }
  else
    Serial.print( pValue->GetRawValue() );
  
  sprintf( buf, " %s Type: %s/%d", pValue->GetUnit(), GetDataTypeString( pValue->GetExType() ), pValue->GetExType() ); Serial.println( buf ); 
  Serial.println( "---------" );
}

void PrintAlarm( RxJetiPacketAlarm * pAlarm )
{
  char buf[50];
  sprintf(buf, "Alarm ! Code: %d, Sound: %d", pAlarm->GetCode(), pAlarm->GetSound() ); Serial.println( buf ); 
  Serial.println( "***********" );
}

const char * GetDataTypeString( uint8_t dataType )
{
  static const char * typeBuf[] = { "6b", "14b", "?", "?", "22b", "DT", "?", "?", "30b", "GPS" };
  return typeBuf[ dataType < 10 ? dataType : 2 ];
}
