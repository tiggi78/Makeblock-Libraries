/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * \class MeUltrasonicSensor
 * \brief   Driver for Me ultrasonic sensor device.
 * @file    MeUltrasonicSensor.cpp
 * @author  MakeBlock
 * @version V1.0.2
 * @date    2016/06/25
 * @brief   Driver for Me ultrasonic sensor device.
 *
 * \par Copyright
 * This software is Copyright (C), 2012-2016, MakeBlock. Use is subject to license \n
 * conditions. The main licensing options available are GPL V2 or Commercial: \n
 *
 * \par Open Source Licensing GPL V2
 * This is the appropriate option if you want to share the source code of your \n
 * application with everyone you distribute it to, and you also want to give them \n
 * the right to share who uses it. If you wish to use this software under Open \n
 * Source Licensing, you must contribute all your source code to the open source \n
 * community in accordance with the GPL Version 2 when your application is \n
 * distributed. See http://www.gnu.org/copyleft/gpl.html
 *
 * \par Description
 * This file is a drive for Me ultrasonic sensor device, It supports ultrasonic sensor
 * V3.0 provided by the MakeBlock.
 *
 * \par Method List:
 *
 *    1. void MeUltrasonicSensor::setpin(uint8_t SignalPin)
 *    2. double MeUltrasonicSensor::distanceCm(uint16_t MAXcm)
 *    3. double MeUltrasonicSensor::distanceInch(uint16_t MAXinch)
 *    4. long MeUltrasonicSensor::measure(unsigned long timeout)
 *
 * \par History:
 * <pre>
 * `<Author>`         `<Time>`        `<Version>`        `<Descr>`
 * Mark Yan         2015/09/04     1.0.0            Rebuild the old lib.
 * Mark Yan         2015/11/16     1.0.1            Increase 100us delay, avoid ultrasonic read exception.
 * Mark Yan         2016/06/25     1.0.2            Modify Read mechanism of ultrasonic waves.
 * Vincent He       2019/03/28     1.0.3            Fix the bug that ultrasonic module ranging can only measure the maximum of 376cm.
 * </pre>
 *
 * @example UltrasonicSensorTest.ino
 */
#include "MeUltrasonicSensor.h"
#include "YetAnotherPcInt.h"
#include <util/atomic.h>

#ifdef ME_PORT_DEFINED
/**
 * Alternate Constructor which can call your own function to map the ultrasonic sensor to arduino port,
 * no pins are used or initialized here.
 * \param[in]
 *   None
 */
MeUltrasonicSensor::MeUltrasonicSensor(void) : MePort(0)
{
  _lastEnterTime = 0;
  _measureValue = 0;
}

/**
 * Alternate Constructor which can call your own function to map the ultrasonic Sensor to arduino port
 * \param[in]
 *   port - RJ25 port from PORT_1 to M2
 */
MeUltrasonicSensor::MeUltrasonicSensor(uint8_t port) : MePort(port)
{
  _lastEnterTime = 0 ;
  _measureValue = 0;
  _SignalPin = s2;
  _pcmskSignalPin = digitalPinToPCMSK(_SignalPin);
  uint8_t portBit = digitalPinToPCMSKbit(_SignalPin);
  _portBitMaskSignalPin = _BV(portBit);
  
  _bitSignalPin = digitalPinToBitMask(_SignalPin);
  uint8_t portPin = digitalPinToPort(_SignalPin);
  _regSignalPin = portModeRegister(portPin);
  _outSignalPin = portOutputRegister(portPin);
  
  PcInt::attachInterrupt( _SignalPin, measurePulse, this );

}
#else // ME_PORT_DEFINED
/**
 * Alternate Constructor which can call your own function to map the ultrasonic Sensor to arduino port,
 * it will assigned the signal pin.
 * \param[in]
 *   port - arduino port(should analog pin)
 */
MeUltrasonicSensor::MeUltrasonicSensor(uint8_t port)
{
  _SignalPin = port;
  _lastEnterTime = millis();
  _measureValue = 0;
  
}
#endif // ME_PORT_DEFINED

/**
 * \par Function
 *   setpin
 * \par Description
 *   Reset the ultrasonic Sensor available PIN by its arduino port.
 * \param[in]
 *   SignalPin - arduino port for sensor read(should analog pin)
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   None
 */
void MeUltrasonicSensor::setpin(uint8_t SignalPin)
{
  PcInt::detachInterrupt( _SignalPin );
  _SignalPin = SignalPin;
  _pcmskSignalPin = digitalPinToPCMSK(_SignalPin);
  uint8_t portBit = digitalPinToPCMSKbit(_SignalPin);
  _portBitMaskSignalPin = _BV(portBit);
  _bitSignalPin = digitalPinToBitMask(_SignalPin);
  uint8_t port = digitalPinToPort(_SignalPin);
  _regSignalPin = portModeRegister(port);
  _outSignalPin = portOutputRegister(port);
  
  _lastEnterTime = 0;
  _measureValue = 0;
#ifdef ME_PORT_DEFINED
  s2 = _SignalPin;
#endif // ME_PORT_DEFINED
  PcInt::attachInterrupt( _SignalPin, measurePulse, this );
}

/**
 * \par Function
 *   distanceCm
 * \par Description
 *   Centimeters return the distance
 * \param[in]
 *   MAXcm - The Max centimeters can be measured, the default value is 400.
 * \par Output
 *   None
 * \return
 *   The distance measurement in centimeters
 * \par Others
 *   None
 */
double MeUltrasonicSensor::distanceCm( bool triggerNew )
{
  double distance = 0.0;
  /* Get sound TOF (Time of Flight)*/
  if( triggerNew )
  {
    distance = (double)measure();
  }
  else
  {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      distance = _measureValue;
    }
  }
  /*
   * Soundspeed 346 m/s  (air, 25 deg C)
   * 2*distance [m] = 346 [m/s] * time [s]
   * distance [cm] = (346 [m/s] * time [us] / 2) / 10e6 [s to us] * 10e2 [m to cm] = 
   * = time [us] * 346 / 20000 = time [us] / 57.8 
   *  
  */
  distance /= 58.0;
  
  if( ( distance >= 400.0 ) || ( distance == 0.0 ) )
  {
    return 400.0;
  }
  else
  {
    return distance;
  }
}

/**
 * \par Function
 *   distanceInch
 * \par Description
 *   Inch return the distance
 * \param[in]
 *   MAXinch - The Max inch can be measured, the default value is 180.
 * \par Output
 *   None
 * \return
 *   The distance measurement in inch
 * \par Others
 *   None
 */
double MeUltrasonicSensor::distanceInch(uint16_t MAXinch)
{
  long distance = measure();
  if((((double)distance / 148.0) >= 400.0) || (distance == 0))
  {
    return( (double)180.0);//MAXinch
  }
  else
  {
    return( (double)distance / 148.0);
  }
}

/**
 * \par Function
 *   measure
 * \par Description
 *   To get the duration of the ultrasonic sensor
 * \param[in]
 *   timeout - This value is used to define the measurement range, The
 *   default value is 30000.
 * \par Output
 *   None
 * \return
 *   The duration value associated with distance
 * \par Others
 *   None
 */
long MeUltrasonicSensor::measure(unsigned long timeout)
{
  unsigned long duration = 0;
  trigger();
  /* pulseIn should have interrupt disabled, otherwise interrupts (e.g TIMER0) make pulseIn
   * mesure less than real impulse
   */
  /*
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    unsigned long duration = pulseIn( s2, HIGH, timeout);
  }    
  */
  /* Ultrasonic sensors has a timeout of 30 ms if it cannot measure the distance.
   * Wait for the measurement to be taken by PCINT interrupts */
  delay( ULTRA_TIMEOUT );
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    duration = _measureValue;
  }    
   
  return duration;
}

#pragma GCC push_options
#pragma GCC optimize ("-O3")
/* 
 * Trigger measure by pulsing pin
 */
void MeUltrasonicSensor::triggerTask ( void* arg )
 {
    MeUltrasonicSensor* sensor = static_cast<MeUltrasonicSensor*>(arg);
    sensor->trigger();
 }

void MeUltrasonicSensor::trigger()
{
  /* Do not trigger if previos measure isn't finished */
    uint32_t elapsed = millis();
    if( ( elapsed - _lastEnterTime ) > MIN_MEAS_TIME)
  {
    _lastEnterTime = elapsed;
    /* Disable PCINT interrupt otherwise it will measure following pulse. */
    *_pcmskSignalPin &= ~_portBitMaskSignalPin;
    
    //pinMode(_SignalPin, OUTPUT);  
    *_regSignalPin |= _bitSignalPin;
 
    //digitalWrite(_SignalPin, LOW);
    *_outSignalPin &= ~_bitSignalPin;

    //digitalWrite(_SignalPin, HIGH);
    *_outSignalPin |= _bitSignalPin;
   
    delayMicroseconds(5);
    
    //digitalWrite(_SignalPin, LOW);
    *_outSignalPin &= ~_bitSignalPin;

    //pinMode(_SignalPin, INPUT);
    *_regSignalPin &= ~_bitSignalPin;

    /* re-enable PCINT interrupt to measure the pulse. */
    *_pcmskSignalPin |= _portBitMaskSignalPin;

    //PcInt::attachInterrupt( _SignalPin, measurePulse, this );
  }
}

void MeUltrasonicSensor::measurePulse(void *userdata, bool pinstate)
{
    unsigned long currentTime = micros();
    
    MeUltrasonicSensor* sensor = static_cast<MeUltrasonicSensor*>(userdata);
    /* RISING edge: start of ultrasonic impulse*/
    if( pinstate )
    {
      sensor->_impulseStart = currentTime;
    }
    /* RISING edge: echo received*/
    if( !pinstate )
    {
      sensor->_measureValue = currentTime - sensor->_impulseStart;
    }
}
#pragma GCC pop_options
