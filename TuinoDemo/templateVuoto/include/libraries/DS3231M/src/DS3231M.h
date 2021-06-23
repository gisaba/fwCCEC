
#include "Arduino.h"                                                          // Arduino data type definitions    //
#include <Wire.h>                                                             // Standard I2C "Wire" library      //
#ifndef DS3231M_h                                                             // Guard code definition            //
 #define DS3231M_h                                                           // Define the name inside guard code//
  /*****************************************************************************************************************
  ** Declare enumerated types used in the class                                                                   **
  *****************************************************************************************************************/
  enum alarmTypes {everySecond,secondsMatch,secondsMinutesMatch,              // Enumeration of the types of      //
       secondsMinutesHoursMatch,secondsMinutesHoursDateMatch,                 // alarm that can be set            //
       secondsMinutesHoursDayMatch,everyMinute,minutesMatch,minutesHoursMatch,//                                  //
       minutesHoursDateMatch,minutesHoursDayMatch,UnknownAlarm};              //                                  //
  /*****************************************************************************************************************
  ** Declare classes used in within the class                                                                     **
  *****************************************************************************************************************/
  class TimeSpan;                                                             //                                  //
  /*****************************************************************************************************************
  ** Declare constants used in the class                                                                          **
  *****************************************************************************************************************/
  #ifndef I2C_MODES                                                           // I2C related constants            //
    #define I2C_MODES                                                         // Guard code to prevent multiple   //
    const uint32_t I2C_STANDARD_MODE              =  100000;                  // Default normal I2C 100KHz speed  //
    const uint32_t I2C_FAST_MODE                  =  400000;                  // Fast mode                        //
  #endif                                                                      //----------------------------------//
  const uint32_t SECONDS_PER_DAY           =     86400;                       // 60 secs * 60 mins * 24 hours     //
  const uint32_t SECONDS_FROM_1970_TO_2000 = 946684800;                       //                                  //
  const uint8_t  DS3231M_ADDRESS           =      0x68;                       // Fixed I2C Address for DS3231M    //
  const uint8_t  DS3231M_RTCSEC            =      0x00;                       // Register names and addresses,    //
  const uint8_t  DS3231M_RTCMIN            =      0x01;                       // declare all possible registers   //
  const uint8_t  DS3231M_RTCHOUR           =      0x02;                       // and if the variables are unused  //
  const uint8_t  DS3231M_RTCWKDAY          =      0x03;                       // no space will be allocated by    //
  const uint8_t  DS3231M_RTCDATE           =      0x04;                       // the compiler                     //
  const uint8_t  DS3231M_RTCMTH            =      0x05;                       //                                  //
  const uint8_t  DS3231M_RTCYEAR           =      0x06;                       //                                  //
  const uint8_t  DS3231M_ALM1SEC           =      0x07;                       //                                  //
  const uint8_t  DS3231M_ALM1MIN           =      0x08;                       //                                  //
  const uint8_t  DS3231M_ALM1HOUR          =      0x09;                       //                                  //
  const uint8_t  DS3231M_ALM1DATE          =      0x0A;                       //                                  //
  const uint8_t  DS3231M_ALM2MIN           =      0x0B;                       //                                  //
  const uint8_t  DS3231M_ALM2HOUR          =      0x0C;                       //                                  //
  const uint8_t  DS3231M_ALM2DATE          =      0x0D;                       //                                  //
  const uint8_t  DS3231M_CONTROL           =      0x0E;                       //                                  //
  const uint8_t  DS3231M_STATUS            =      0x0F;                       //                                  //
  const uint8_t  DS3231M_AGING             =      0x10;                       //                                  //
  const uint8_t  DS3231M_TEMPERATURE       =      0x11;                       //                                  //
  /*****************************************************************************************************************
  ** Simple general-purpose date/time class (no TZ / DST / leap second handling). Copied from RTClib. For further **
  ** information on this implementation see https://github.com/SV-Zanshin/DS3231M/wiki/DateTimeClass              **
  *****************************************************************************************************************/
  class DateTime {                                                            //                                  //
    public:                                                                   //----------------------------------//
      DateTime (uint32_t t=0);                                                // Constructor                      //
      DateTime (uint16_t year,uint8_t month,uint8_t day,uint8_t hour=0,       // Overloaded Constructors          //
                uint8_t min=0,uint8_t sec=0);                                 //                                  //
      DateTime (const DateTime& copy);                                        //                                  //
      DateTime (const char* date, const char* time);                          //                                  //
      DateTime (const __FlashStringHelper* date,                              //                                  //
                const __FlashStringHelper* time);                             //                                  //
      uint16_t year()         const { return 2000 + yOff; }                   // Return the year                  //
      uint8_t  month()        const { return m; }                             // Return the month                 //
      uint8_t  day()          const { return d; }                             // Return the day                   //
      uint8_t  hour()         const { return hh; }                            // Return the hour                  //
      uint8_t  minute()       const { return mm; }                            // Return the minute                //
      uint8_t  second()       const { return ss; }                            // Return the second                //
      uint8_t  dayOfTheWeek() const;                                          // Return the DOW                   //
      long     secondstime()  const;                                          // times as seconds since 1/1/2000  //
      uint32_t unixtime(void) const;                                          // times as seconds since 1/1/1970  //
      DateTime operator+(const TimeSpan& span);                               // addition                         //
      DateTime operator-(const TimeSpan& span);                               // subtraction                      //
      TimeSpan operator-(const DateTime& right);                              // subtraction                      //
    protected:                                                                //----------------------------------//
      uint8_t yOff, m, d, hh, mm, ss;                                         // private variables                //
  }; // of class DateTime definition                                          //                                  //
  /*****************************************************************************************************************
  ** Timespan class which can represent changes in time with seconds accuracy. Copied from RTClib. For further    **
  ** information see ** https://github.com/SV-Zanshin/DS3231M/wiki/TimeSpanClass                                  **
  *****************************************************************************************************************/
  class TimeSpan {                                                            //                                  //
    public:                                                                   //----------------------------------//
      TimeSpan (int32_t seconds = 0);                                         //                                  //
      TimeSpan (int16_t days, int8_t hours, int8_t minutes, int8_t seconds);  //                                  //
      TimeSpan (const TimeSpan& copy);                                        //                                  //
      int16_t  days() const         { return _seconds / 86400L; }             //                                  //
      int8_t   hours() const        { return _seconds / 3600 % 24; }          //                                  //
      int8_t   minutes() const      { return _seconds / 60 % 60; }            //                                  //
      int8_t   seconds() const      { return _seconds % 60; }                 //                                  //
      int32_t  totalseconds() const { return _seconds; }                      //                                  //
      TimeSpan operator+(const TimeSpan& right);                              //                                  //
      TimeSpan operator-(const TimeSpan& right);                              //                                  //
    protected:                                                                //----------------------------------//
      int32_t _seconds;                                                       // internal seconds variable        //
  }; // of class TimeSpan definition                                          //                                  //
  /*****************************************************************************************************************
  ** Main DS3231M class for the Real-Time clock                                                                   **
  *****************************************************************************************************************/
  class DS3231M_Class {                                                       // Class definition                 //
    public:                                                                   // Publicly visible methods         //
      DS3231M_Class();                                                        // Class constructor                //
      ~DS3231M_Class();                                                       // Class destructor                 //
      bool     begin(const uint32_t i2cSpeed = I2C_STANDARD_MODE);            // Start I2C Communications         //
      void     adjust();                                                      // Set the date and time to compile //
      void     adjust(const DateTime& dt);                                    // Set the date and time            //
      DateTime now();                                                         // return time                      //
      int32_t  temperature();                                                 // return clock temp in 100x °C     //
    private:                                                                  // Private methods                  //
      uint8_t  readByte(const uint8_t addr);                                  // Read 1 byte from address on I2C  //
      void     writeByte(const uint8_t addr, const uint8_t data);             // Write 1 byte at address to I2C   //
      uint8_t  bcd2int(const uint8_t bcd);                                    // convert BCD digits to integer    //
      uint8_t  int2bcd(const uint8_t dec);                                    // convert integer to BCD           //
      uint8_t  _TransmissionStatus = 0;                                       // Status of I2C transmission       //
      uint32_t _SetUnixTime        = 0;                                       // UNIX time when clock last set    //
      uint8_t  _ss,_mm,_hh,_d,_m;                                             // Define date components           //
      uint16_t _y;                                                            // Define date components           //
  }; // of DS3231M class definition                                           //                                  //
#endif                                                                        //----------------------------------//