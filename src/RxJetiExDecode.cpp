/* 
  Jeti EX Telemetry sensor decoder C++ Library
  
  RxJetiExSerial - Jeti EX sensor protocol decoder
  -------------------------------------------------------------------
  
  Copyright (C) 2022 Bernd Wokoeck
  
  Version history:
  0.99   02/09/2022  created

  Permission is hereby granted, free of charge, to any person obtaining
  a copy of this software and associated documentation files (the "Software"),
  to deal in the Software without restriction, including without limitation
  the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the
  Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
  IN THE SOFTWARE.

**************************************************************/

#include "RxJetiExDecode.h"

void  RxJetiDecode::Start( enComPort comPort )
{
  // init serial port 
  m_pSerial = RxJetiExSerial::CreatePort( comPort );
  m_pSerial->Init(); 
}

RxJetiExPacket * RxJetiDecode::GetPacket()
{
  if( millis() > m_tiTimeout )
  {
    m_tiTimeout = millis() + 1000;
    m_state = WAIT_STARTOFPACKET; 
    return NULL;
  }
  
  // process existing ex buffer witch values
  if( m_state == WAIT_NEXTVALUE )
  {
    RxJetiExPacket * pPacket = DecodeValue();
    if( pPacket == NULL )
      m_state = WAIT_STARTOFPACKET; 
    return pPacket;
  }

  // process next character
  uint16_t c =  m_pSerial->Getchar();
  if( c )
  {
    m_tiTimeout = millis() + 1000;
    
    // DumpSerial( 1, c );
    // char buf[32];
    // sprintf( buf, "0x%x\n", c ); Serial.print( buf );

    if( m_state == WAIT_STARTOFPACKET )
    {
      if( c == 0x007e )
      {
        // Serial.println( "Start" ); 
        m_state = WAIT_EX_BYTE;
      }
      else if( c == 0x00FE )
      {
         // Serial.println( "Simple text" ); 
         m_nBytes = 0;
         m_state = WAIT_ENDOFTEXT;
      }
    }
    else if( m_state == WAIT_EX_BYTE )
    {
      if( ( c & 0x000F ) == 0x000F )
      {
        // Serial.println( "Is EX packet" ); 
        m_state = WAIT_LEN;
      }
      else if( ( c & 0x0002 ) == 0x0002 )
      {
        // Serial.println( "Alarm" ); 
        m_nBytes = 0;
        m_state = WAIT_ENDOFALARM; 
      }
      else
      {
        // Serial.println( "Unhandled packet type" ); 
        m_state = WAIT_STARTOFPACKET; // unhandled message
        return &m_error;
      }
    }
    else if( m_state == WAIT_LEN )
    {
      m_enMsgType  = (((uint8_t)c >> 6) & 0x03);
      m_nPacketLen = (uint8_t)c & 0x1F;
      m_nBytes     = 0;
      memset( m_exBuffer, 0, sizeof( m_exBuffer ) );

     // char buf[32];
     // sprintf( buf, "msgytpe: %d\n", m_enMsgType ); Serial.print( buf );

      if( m_nPacketLen > 5 ) // must at least contain serial# and id
      {
        m_state = WAIT_ENDOFEXPACKET;
      }
      else
      {
        m_state = WAIT_STARTOFPACKET; // invalid length
        return &m_error;
      }

      // Serial.print( "Packet len: " ); 
      // Serial.println( m_nPacketLen ); 
    }
    else if( m_state == WAIT_ENDOFEXPACKET )
    {
      m_exBuffer[ m_nBytes++ ] = (uint8_t)c;
        
      if( m_nBytes == m_nPacketLen )
      {
        if( crcCheck() )
        {
          uint8_t key = m_exBuffer[4];          
          decrypt( key, m_exBuffer, m_nPacketLen );

          // DumpBuffer( m_exBuffer, m_nPacketLen );

          // sensor name
          if( m_exBuffer[ 5 ] == 0 && (m_enMsgType == MSGTYPE_TEXT) )
          {
            m_state = WAIT_STARTOFPACKET;
            return DecodeName();
          }
          // sensor label 
          else if( m_exBuffer[ 5 ] != 0 && (m_enMsgType == MSGTYPE_TEXT) )
          {
            m_state = WAIT_STARTOFPACKET;
            return DecodeLabel();
          }
          // message
          else if( m_enMsgType == MSGTYPE_MSG )
          {
            // todo
            memcpy( &m_value.m_serialId, &m_exBuffer[0], 4 );
            m_nBytes = 5; // place index on first value
            m_state = WAIT_NEXTVALUE;
            return NULL; 
          }
          else
          {
            // DumpBuffer( m_exBuffer, m_nPacketLen );
            memcpy( &m_value.m_serialId, &m_exBuffer[0], 4 );
            m_nBytes = 5; // place index on first value
            m_state = WAIT_NEXTVALUE;
            return DecodeValue();
          }
        }
        else
        {
          // invalid crc
          // Serial.println( "Crc error" ); 
          m_state = WAIT_STARTOFPACKET;
          return &m_error;
        }
      }
    }
    else if( m_state == WAIT_ENDOFALARM )
    {
      m_exBuffer[ m_nBytes++ ] = (uint8_t)c;
      if( m_nBytes == 2 )
      {
         m_alarm.m_bSound = m_exBuffer[ 0 ] & 1;
         m_alarm.m_code   = m_exBuffer[ 1 ];
         m_state = WAIT_STARTOFPACKET; 
         return &m_alarm;
      }
    }
    else if( m_state == WAIT_ENDOFTEXT )
    {
      if( c == 0x00FF )
      {
         m_state = WAIT_STARTOFPACKET; 
         memcpy( &m_text.m_textBuffer, &m_exBuffer[0], m_nBytes );
         m_text.m_textBuffer[m_nBytes] = '\0';
         return &m_text;
      }
      else if( m_nBytes > 32 )
      {
        m_state = WAIT_STARTOFPACKET; // invalid length
        return &m_error;
      }
      else
        m_exBuffer[ m_nBytes++ ] = (uint8_t)c;
    }
  }

  return NULL;
}

RxJetiExPacket * RxJetiDecode::DecodeName()
{
   uint32_t serialId;
   memcpy( &serialId, &m_exBuffer[0], 4 );

  if( m_nPacketLen < 8 ) 
    return NULL;

   // already present
   RxJetiExPacketName * pName = FindName( serialId );
   if( pName )
   {
     if( pName->m_pstrName == NULL ) // dummy name element, generated by AppendLabel ?
       pName->m_pstrName = NewName();
     return pName;
   }

   // new sensor
   pName = new RxJetiExPacketName;
   pName->m_serialId = serialId;

  // get sensor name
  pName->m_pstrName = NewName();

  // append to list
  AppendName( pName );

  DumpOutput( pName );

  return pName;
}

RxJetiExPacket * RxJetiDecode::DecodeLabel()
{
  uint32_t serialId;
  memcpy( &serialId, &m_exBuffer[0], 4 );
  uint8_t id = m_exBuffer[ 5 ];

  if( m_nPacketLen < 9 ) 
    return NULL;

  // already present
  RxJetiExPacketLabel * pLabel = FindLabel( serialId, id );
  if( pLabel )
    return pLabel;

  // new label
  pLabel = new RxJetiExPacketLabel;
  pLabel->m_serialId = serialId;
  pLabel->m_id = id;

  // get label name and unit
  pLabel->m_pstrLabel = NewName();
  pLabel->m_pstrUnit = NewUnit();

  // append to list
  AppendLabel( pLabel );

  DumpOutput( pLabel );

  return pLabel;
}

char * RxJetiDecode::NewName()
{ 
  int n = 6;
  // get sensor name or label
  int len = (m_exBuffer[n] >> 3) & 0x1F;
  char * p = new char[ len + 1 ];
  memcpy( p, &m_exBuffer[ n + 1 ], len );
  p[len]  = '\0';

  return p;
}

char * RxJetiDecode::NewUnit()
{
  int n = 6;
  int len1 = (m_exBuffer[n] >> 3) & 0x1F;
  int len2 = m_exBuffer[n] & 0x07;
  char * p = new char[ len2 + 1 ];
  memcpy( p, &m_exBuffer[(n+1) + len1], len2 );
  p[len2] = '\0';

  // replace degree symbol
  for( int i = 0; i < len2; i++ )
    if( p[i] == '\xB0' )
      p[i] = '�';

  return p;
}

char * RxJetiDecode::NewString( const char * pStr )
{
  size_t l = strlen( pStr ) + 1;
  char * c = new char[ l ];
  strcpy( c, pStr );
  c[l] = '\0';

  return c;
}

// decode sensor value from jeti ex format
RxJetiExPacket * RxJetiDecode::DecodeValue()
{
  if( m_nBytes >= m_nPacketLen - 3 ) // minimum length: packetLen - 1byte crc - 1 byte id - 1 byte data 
    return NULL;

  // type and id
  m_value.m_exType = m_exBuffer[ m_nBytes ] & 0x0F;
  m_value.m_id     = m_exBuffer[ m_nBytes++ ] >> 4;
  if( m_value.m_id == 0 )
    m_value.m_id = m_exBuffer[ m_nBytes++ ];

  switch( m_value.m_exType )
  {
  case RxJetiExPacket::TYPE_6b:
    {
      uint8_t val1       = m_exBuffer[ m_nBytes++ ];
      m_value.m_value    = val1 & 0x1F;
      m_value.m_value    = (val1 & 0x80) ? -m_value.m_value : m_value.m_value;
      m_value.m_exponent = (val1>>5) & 0x03;
    }
    break;

  case RxJetiExPacket::TYPE_14b:
    {
      uint8_t val1       = m_exBuffer[ m_nBytes++ ];
      uint8_t val2       = m_exBuffer[ m_nBytes++ ];
      m_value.m_value    = val1;
      m_value.m_value   |= (int32_t)(val2 & 0x1F) << 8;
      m_value.m_value    = (val2 & 0x80) ? -m_value.m_value : m_value.m_value;
      m_value.m_exponent = (val2>>5) & 0x03;
    }
    break;

  case RxJetiExPacket::TYPE_22b:
    {
      uint8_t val1       = m_exBuffer[ m_nBytes++ ];
      uint8_t val2       = m_exBuffer[ m_nBytes++ ];
      uint8_t val3       = m_exBuffer[ m_nBytes++ ];
      m_value.m_value    = val1;
      m_value.m_value   |= (int32_t)val2<<8;
      m_value.m_value   |= (int32_t)(val3 & 0x1F) << 16;
      m_value.m_value    = (val3 & 0x80) ? -m_value.m_value : m_value.m_value;
      m_value.m_exponent = (val3>>5) & 0x03;
    }
    break;

  case RxJetiExPacket::TYPE_30b:
    {
      uint8_t val1 = m_exBuffer[ m_nBytes++ ];
      uint8_t val2 = m_exBuffer[ m_nBytes++ ];
      uint8_t val3 = m_exBuffer[ m_nBytes++ ];
      uint8_t val4 = m_exBuffer[ m_nBytes++ ];
      m_value.m_value  = val1;
      m_value.m_value |= (int32_t)val2<<8;
      m_value.m_value |= (int32_t)val3<<16;
      m_value.m_value |= (int32_t)(val4 & 0x1F) << 24;
      m_value.m_value  = (val4 & 0x80) ? -m_value.m_value : m_value.m_value;
      m_value.m_exponent = (val4>>5) & 0x03;
    }
    break;

  case RxJetiExPacket::TYPE_DT:
    m_value.m_value  = m_exBuffer[ m_nBytes++ ];
    m_value.m_value |= (int32_t)(m_exBuffer[ m_nBytes++ ]) << 8;
    m_value.m_value |= (int32_t)(m_exBuffer[ m_nBytes++ ]) << 16;
    m_value.m_exType = RxJetiExPacket::TYPE_DT;
    m_value.m_exponent = 0;
    break;

  case RxJetiExPacket::TYPE_GPS:
    m_value.m_value  = m_exBuffer[ m_nBytes++ ];
    m_value.m_value |= (int32_t)(m_exBuffer[ m_nBytes++ ]) << 8;
    m_value.m_value |= (int32_t)(m_exBuffer[ m_nBytes++ ]) << 16;
    m_value.m_value |= (int32_t)(m_exBuffer[ m_nBytes++ ]) << 24;
    m_value.m_exType = RxJetiExPacket::TYPE_GPS;
    m_value.m_exponent = 0;
    break;
  }

  // link value with label 
  m_value.m_pLabel = FindLabel( m_value.m_serialId, m_value.m_id );

  DumpOutput( &m_value );

  return &m_value;
}

RxJetiExPacketName * RxJetiDecode::FindName( uint32_t serialId )
{
  RxJetiExPacketName * p = m_pSensorList;
  while( p )
  {
    if( p->m_serialId == serialId )
      return p;
    p = p->m_pNext; 
  }
  return 0;
}

RxJetiExPacketLabel * RxJetiDecode::FindLabel( uint32_t serialId, uint8_t id )
{
  RxJetiExPacketLabel * pL = NULL;
  RxJetiExPacketName  * pN = FindName( serialId );
  if( pN )
  {
    pL = pN->m_pFirstLabel; 
    while( pL )
    {
      if( pL->m_serialId == serialId && pL->m_id == id )
        return pL;
      pL = pL->m_pNext; 
    }
  }
  return 0;
}

void RxJetiDecode::AppendName( RxJetiExPacketName * pName )
{
  if( m_pSensorList == NULL )
  {
    m_pSensorList = pName;
    return;
  }

  RxJetiExPacketName * p = m_pSensorList;
  while( p )
  {
    if( p->m_pNext == NULL )
    {
      p->m_pNext = pName;
      return;
    }
    p = p->m_pNext; 
  }
  return;
}

void RxJetiDecode::AppendLabel( RxJetiExPacketLabel *pLabel )
{
  RxJetiExPacketLabel * pL = NULL;
  RxJetiExPacketName  * pN = FindName( pLabel->m_serialId );
  if( pN == NULL )
  {
    pN = new RxJetiExPacketName;
    pN->m_serialId = pLabel->m_serialId;
    AppendName( pN );
    pN->m_pFirstLabel = pLabel;
    pLabel->m_pName = pN;
    return;
  }

  // name found w/o finding a label before
  if( pN->m_pFirstLabel == NULL )
  {
    pN->m_pFirstLabel = pLabel;
    pLabel->m_pName = pN;
    return;
  }

  // at least one label already exists
  pL = pN->m_pFirstLabel; // name found w/o finding a label before
  while( pL )
  {
    if( pL->m_pNext == NULL )
    {
      pL->m_pNext = pLabel;
      pLabel->m_pName = pN;
      return;
    }
    pL = pL->m_pNext; 
  }
  return;
}

// optionally add missing label, unit and name data to current value
bool RxJetiDecode::CompleteValue( RxJetiExPacketValue * pValue, const char * pstrName, const char * pstrLabel, const char * pstrUnit )
{
  RxJetiExPacketLabel * pLabel = FindLabel( pValue->m_serialId, pValue->m_id );
  if( pLabel )
    return false; // nothing to do
  
  // new label
  pLabel = new RxJetiExPacketLabel;
  pLabel->m_serialId  = pValue->m_serialId;
  pLabel->m_id        = pValue->m_id;
  pLabel->m_pstrLabel = NewString( pstrLabel );
  pLabel->m_pstrUnit  = NewString( pstrUnit );

  // append to list
  AppendLabel( pLabel );

  // set sensor name
  RxJetiExPacketName * pName = FindName( pValue->m_serialId );
  if( pName && pName->m_pstrName == NULL )
    pName->m_pstrName = NewString( pstrName );

  return true;
}


// packet methods
////////////////

// string for unknown name, labels and units
const char * RxJetiExPacket::m_strUnknown = "?";

bool RxJetiExPacketValue::GetFloat( float * pValue )
{
  if( IsNumeric() && pValue )
  {
    *pValue = m_value;
    if( m_exponent == 1 )
      *pValue /= 10.0;
    else if( m_exponent == 2 )
      *pValue /= 100.0;
    return true;
  }
  return false;
}

bool  RxJetiExPacketValue::GetLatitude( float * pLatitude )
{
  bool bLongitude;
  if( GetGPS( &bLongitude, pLatitude ) && !bLongitude )
    return true;
  return false;
}

bool  RxJetiExPacketValue::GetLongitude( float * pLongitude )
{
  bool bLongitude;
  if( GetGPS( &bLongitude, pLongitude ) && bLongitude )
    return true;
  return false;
}

bool RxJetiExPacketValue::GetGPS( bool * pbLongitude, float * pCoord )
{
  if( m_exType == TYPE_GPS )
  {
    i2b.vInt = m_value;
    *pbLongitude = i2b.vBytes[3] & 0x20;
    uint16_t deg16  = (i2b.vBytes[3] & 0x01) << 8;
             deg16 +=  i2b.vBytes[2];
    uint16_t min16  =  i2b.vBytes[1] << 8;
             min16 +=  i2b.vBytes[0];

    float frac  = min16 / 0.60000f / 100000.0f;
    float coord = deg16 + frac;

    *pCoord = ( i2b.vBytes[3] & 0x40 ) ? -coord : coord;
    return true;
  }
  return false;
}

bool RxJetiExPacketValue::GetDate( uint8_t * pDay, uint8_t * pMonth, uint16_t * pYear )
{
  if( m_exType == TYPE_DT )
  {
    i2b.vInt = m_value;
    if( i2b.vBytes[2] & 0x20 )
    {
      *pDay   = i2b.vBytes[2] & 0x1F;
      *pMonth = i2b.vBytes[1];
      *pYear  = i2b.vBytes[0] + 2000;
      return true;
    }
  }
  return false;
}

bool RxJetiExPacketValue::GetTime( uint8_t * pHour, uint8_t * pMinute, uint8_t * pSecond )
{
  if( m_exType == TYPE_DT )
  {
    i2b.vInt = m_value;
    if( (i2b.vBytes[2] & 0x20) == 0 )
    {
      *pHour    = i2b.vBytes[2] & 0x1F;
      *pMinute  = i2b.vBytes[1];
      *pSecond  = i2b.vBytes[0];
      return true;
    }
  }
  return false;
}

inline bool RxJetiExPacketValue::IsNumeric()
{
  const uint16_t bitNumeric = 0x113; // 100010011
  if( m_exType <= TYPE_GPS )
    return (bitNumeric & (1<<m_exType)) ? true : false;
  return false;
}

// check if label, unit and sensor name exist
bool RxJetiExPacketValue::IsValueComplete()
{
  if( m_pLabel != NULL && m_pLabel->m_pstrUnit != NULL && m_pLabel->m_pName != NULL )
    if( m_pLabel->m_pName->m_pstrName != NULL )
      return true;
  return false;
}


// Jeti helpers
///////////////

// Published in "JETI Telemetry Protocol EN V1.06"
//* Jeti EX Protocol: Calculate 8-bit CRC polynomial X^8 + X^2 + X + 1
uint8_t RxJetiDecode::update_crc (uint8_t crc, uint8_t crc_seed)
{
  const int POLY = 7;
  unsigned char crc_u;
  unsigned char i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for (i=0; i<8; i++)
    crc_u = ( crc_u & 0x80 ) ? POLY ^ ( crc_u << 1 ) : ( crc_u << 1 );
  return (crc_u);
}

//* Calculate CRC8 Checksum over EX-Frame, Original code by Jeti
bool RxJetiDecode::crcCheck()
{
  uint8_t crc = 0;
  uint8_t c;

  uint8_t lenByte = m_nPacketLen | (m_enMsgType << 6);
  crc = update_crc( lenByte, crc );

  for( c = 0; c < m_nPacketLen - 1; c++ )
    crc = update_crc( m_exBuffer[c], crc );

  // Serial.print( "crc: " ); Serial.print( crc ); Serial.print( "/" ); Serial.println( m_exBuffer[ m_nPacketLen-1 ] );

  return( crc == m_exBuffer[ m_nPacketLen - 1] );
}

//
// ********************** taken from Jeti-Duplex-EX code by H.Stoecklein ******************
//
// Prepare EX data frame for transfer (Frameheader, Crypting, Länge, CRC)
void RxJetiDecode::decrypt( uint8_t key, uint8_t * exbuf, unsigned char n )
{
  unsigned char i;
  uint8_t cryptcode[4] = { 0x52,0x1C,0x6C,0x23 };
  int o = 3; // buffer offset since bytes 0-2 are omitted

  if( key == 0 ) // not encrypted
    return;

  exbuf[8-o] = exbuf[8-o] ^ key ^ 0x6D; // telemetry value id
  if (!(m_nPacketLen & 0x02))       // something fishy at Byte 8...
    exbuf[8-o]^=0x3F;

  exbuf[9-o] ^= 32;         // Roberts Tipp (tnx!)

  for( i = 9; i < n+o; i++ ) // Now decode frame starting at byte 9
  {
    exbuf[i-o] = exbuf[i-o] ^ key ^ (cryptcode[i%4] + ((i%2)?((i-8)&0xFC):0));
    if( key & 0x02 )
      exbuf[i-o] ^= 0x3F;
  }
}

// Debug output
///////////////
#ifdef RXJETIEX_DECODE_DEBUG
void RxJetiDecode::DumpOutput( RxJetiExPacket * pPacket )
{
  char buf[50];
  RxJetiExPacketName  * pName  = NULL;
  RxJetiExPacketLabel * pLabel = NULL;
  RxJetiExPacketValue * pValue = NULL;

  switch( pPacket->GetPacketType() )
  {
  case RxJetiExPacket::PACKET_NAME:
    pName = (RxJetiExPacketName *)pPacket;
    sprintf(buf, "Sensor - Serial: %08lx", pName->GetSerialId() ); Serial.println( buf ); 
    sprintf(buf, "Name: %s", pName->GetName() ); Serial.println( buf ); 
    Serial.println( "!!!!!!!!!!!" );
    break;
  case RxJetiExPacket::PACKET_LABEL:
    pLabel = (RxJetiExPacketLabel *)pPacket;
    sprintf(buf, "Label from %s, Serial: %08lx/%d", pLabel->GetName(), pLabel->GetSerialId(), pLabel->GetId() ); Serial.println( buf ); 
    sprintf(buf, "Label - %s, Unit: %s", pLabel->GetLabel(), pLabel->GetUnit() ); Serial.println( buf ); 
    Serial.println( "++++++++++" );
    break;
  case RxJetiExPacket::PACKET_VALUE:
    pValue = (RxJetiExPacketValue *)pPacket;
    Serial.print( "Value - " ); sprintf(buf, "%08lx", pValue->m_serialId ); Serial.println( buf ); 
    Serial.print( "Id: " );     Serial.print( pValue->m_id );               Serial.print( ", "); 
    Serial.print( "Val: " );    Serial.print( pValue->m_value );            Serial.print( ", "); 
    Serial.print( "Type: " );   Serial.print( GetDataTypeString( pValue->m_exType ) ); Serial.print( "/"); Serial.println( pValue->m_exType ); 
    Serial.print( "nBytes: " ); Serial.print( m_nBytes ); Serial.print( "/" );  Serial.println( m_nPacketLen ); 
    Serial.println( "++++++++++" );
    break;
  }
}

void RxJetiDecode::DumpBuffer( uint8_t * buffer, uint8_t nChars )
{
  Serial.println( "Buffer Dump" ); 
  for( int i = 0; i < nChars; i++ )
  {
    DumpSerial( 1, buffer[i] | 0xFF00, false );
    Serial.print( " " );
  }
  Serial.println( "" );
}

void RxJetiDecode::DumpSerial( int numChar, uint16_t sChar, bool bLf  )
{
   char buf[ 64 ];
   int idx = 0;

   numChar = min( 8, numChar );
   for( int i = 0; i < numChar; i++ )
   {
     if( (sChar & 0xFF00 ) == 0xFF00 )
     {
       buf[ idx++ ] = '+';
     }
     else if( sChar <= 255 ) // command bytes
     {
       buf[ idx++ ] = '-';
       buf[ idx++ ] = '-';
     }

     char c = sChar;
     {
       buf[ idx++ ] = '0';
       buf[ idx++ ] = 'x';
       itoa( ((int)c) & 0x00FF, &buf[idx], 16 );
       idx += 2;
       buf[ idx++ ] = ' ';
     }
   }
   if( idx > 0 )
     buf[ idx++ ] = '\0';
   
   if( bLf )
     Serial.println( buf );
   else
     Serial.print( buf );
}

const char * RxJetiDecode::GetDataTypeString( uint8_t dataType )
{
  static const char * typeBuf[] = { "6b", "14b", "?", "?", "22b", "DT", "?", "?", "30b", "GPS" };
  return typeBuf[ dataType < 10 ? dataType : 2 ];
}
#endif // RXJETIEX_DECODE_DEBUG

/*
 EX data packet structure

 Header
 Offset len   meaning
 0      1Byte 0x7E Separator of the message 
 1      1Byte 0xNF Distinct identification of an EX packet, N could be an arbitrary number.
 2      2Bit  Type (0-3) Packet type; 1 � Data protocol, 0 - Text protocol
 2      6Bit  Length (0-31) Length of a packet (number of bytes following)
 3-4    2Byte Serial Number,  Upper part of a serial number, Manufacturer ID (Little Endian)
 5-6    2Byte Serial Number, Lower part of a serial number, Device ID (Little Endian)
 7      1Byte Crypto key

 EX Text message (value name and unit)
 8      1Byte Identifier (0-255)
 9      5Bit  Length of the description 
 9      3Bit  Length of unit's description 
 ?      1Byte CRC8 Cyclic redundancy check 

 EX data message
 8      4Bit Identifier (0-15) 
 8      4Bit Data type (0-15)
 9      xByte Data with length according to jeti data type
 ... and so on, up to length of packet
 ?      1Byte CRC8 Cyclic redundancy check
*/
