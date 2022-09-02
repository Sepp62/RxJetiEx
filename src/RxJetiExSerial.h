/* 
  Jeti EX Telemetry sensor decoder C++ Library
  
  RxJetiExSerial - Sensor telemetry EX serial implementation
  -----------------------------------------------------------
  
  Copyright (C) 2012 Bernd Wokoeck
  
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

#ifndef RXJETIEXSERIAL_H
#define RXJETIEXSERIAL_H

#ifdef ESP32 
  #define RXJETIEX_ARDUINO_UART  // use 8 bit HardwareSerial
#endif 

#if ARDUINO >= 100
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

class RxJetiExSerial
{
public:
  static RxJetiExSerial * CreatePort( int comPort ); // comPort: 0=default, Teensy: 1..3

  virtual void     Init() = 0;
  virtual uint16_t Getchar(void) = 0;
};

// Teensy
/////////
#if defined (CORE_TEENSY) 

  class RxJetiExTeensySerial : public RxJetiExSerial
  {
  public:
    RxJetiExTeensySerial( int comPort );
    virtual void Init();
    virtual uint16_t Getchar(void);
  protected:
    HardwareSerial * m_pSerial;
  };

#elif defined (RXJETIEX_ARDUINO_UART)

  class RxJetiExArduinoSerial : public RxJetiExSerial
  {
  public:
    RxJetiExArduinoSerial( int comPort );
    virtual void Init();
    virtual uint16_t Getchar(void);
  protected:
    HardwareSerial * m_pSerial;
    uint16_t c_minus1;
    uint16_t c_minus2;
    uint16_t c_minus3;
  };

#else

  #if defined (__AVR_ATmega32U4__)
    #define _BV(bit) (1 << (bit))
    #define UCSRA UCSR1A
    #define UCSRB UCSR1B
    #define UCSRC UCSR1C

    #define UCSZ2 UCSZ12
    #define RXEN RXEN1
    #define TXEN TXEN1
    #define UCSZ0 UCSZ10
    #define UCSZ1 UCSZ11
    #define UCSZ2 UCSZ12

    #define UPM0 UPM10
    #define UPM1 UPM11

    #define UBRRH UBRR1H
    #define UBRRL UBRR1L

    #define RXCIE RXCIE1
    #define UDRIE UDRIE1
    #define TXCIE TXCIE1

    #define TXB8 TXB81
    #define RXB8 RXB81

    #define UDR UDR1
    #define UDRIE UDRIE1

    #define USART_RX_vect USART1_RX_vect
    #define USART_TX_vect USART1_TX_vect
    #define USART_UDRE_vect USART1_UDRE_vect
  #else
    #define UCSRA UCSR0A
    #define UCSRB UCSR0B
    #define UCSRC UCSR0C

    #define RXEN RXEN0
    #define TXEN TXEN0
    #define UCSZ0 UCSZ00
    #define UCSZ1 UCSZ01
    #define UCSZ2 UCSZ02

    #define UPM0 UPM00
    #define UPM1 UPM01

    #define UBRRH UBRR0H
    #define UBRRL UBRR0L

    #define RXCIE RXCIE0
    #define UDRIE UDRIE0
    #define TXCIE TXCIE0

    #define TXB8 TXB80
    #define RXB8 RXB80
    #define UDR UDR0
    #define UDRIE UDRIE0
  #endif 

  // ATMega
  //////////
  class RxJetiExAtMegaSerial : public RxJetiExSerial
  {
  public:
    virtual void Init();
  };

  // interrupt driven transmission
  // ->  low CPU usage (~1ms per frame), slightly higher latency
  ////////////////////////////////

  extern "C" void USART_RX_vect(void) __attribute__ ((signal)); // make C++ class accessible for ISR

  class RxJetiExHardwareSerialInt : public RxJetiExAtMegaSerial
  {
    friend void USART_RX_vect(void);

  public:
    virtual void Init();
    virtual uint16_t Getchar(void);

  protected:
    enum
    {
      RX_RINGBUF_SIZE = 64,
    };

    // rx buffer
    volatile uint16_t   m_rxBuf[ RX_RINGBUF_SIZE ]; 
    volatile uint16_t * m_rxHeadPtr;
    volatile uint16_t * m_rxTailPtr;
    volatile uint16_t   m_rxNumChar;
    volatile uint16_t * IncBufPtr( volatile uint16_t * ptr, volatile uint16_t * ringBuf, size_t bufSize );
  };
  
#endif // CORE_TEENSY

#endif // RXJETIEXSERIAL_H
