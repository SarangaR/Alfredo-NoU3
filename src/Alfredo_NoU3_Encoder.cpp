#include "Alfredo_NoU3_Encoder.h"

RotaryEncoder::RotaryEncoder( uint8_t encoderPinA, uint8_t encoderPinB)
{
  this->encoderPinA = encoderPinA;
  this->encoderPinB = encoderPinB;
}

RotaryEncoder::~RotaryEncoder()
{
  detachInterrupts();
}

void RotaryEncoder::setEncoderType( EncoderType type )
{
  switch( type )
  {
    case FLOATING:
      encoderPinMode = INPUT_PULLUP;
    break;

    case HAS_PULLUP:
      encoderPinMode = INPUT;
    break;

    case SW_FLOAT:
      encoderPinMode = INPUT;
    break;

    default:
      return;
  }

}

void RotaryEncoder::setBoundaries( long minValue, long maxValue )
{
  this->minEncoderValue = minValue;
  this->maxEncoderValue = maxValue;
}

void RotaryEncoder::attachInterrupts()
{
  #if defined( BOARD_HAS_PIN_REMAP ) && ( ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3,0,0) )
    /**
     * The io_pin_remap.h in Arduino-ESP32 cores of the 2.0.x family
     * (since 2.0.10) define an `attachInterrupt()` macro that folds-in
     * a call to `digitalPinToGPIONumber()`, but FunctionalInterrupt.cpp
     * does this too, so we actually don't need the macro at all.
     * Since 3.x the call inside the function was removed, so the wrapping
     * macro is useful again.
     */
    #undef attachInterrupt
  #endif

  attachInterrupt( encoderPinA, std::bind( &RotaryEncoder::_encoder_ISR, this ), CHANGE );
  attachInterrupt( encoderPinB, std::bind( &RotaryEncoder::_encoder_ISR, this ), CHANGE );
}

void RotaryEncoder::detachInterrupts()
{
  detachInterrupt( encoderPinA );
  detachInterrupt( encoderPinB );
}

void RotaryEncoder::begin( )
{
  resetEncoderValue();

  pinMode( encoderPinA, encoderPinMode );
  pinMode( encoderPinB, encoderPinMode );

  delay( 20 );
  attachInterrupts();
}

bool RotaryEncoder::isEnabled()
{
  return _isEnabled;
}

void RotaryEncoder::enable()
{
  if( _isEnabled )
    return;

  attachInterrupts();

  _isEnabled = true;
}

void RotaryEncoder::disable()
{
  if( !_isEnabled )
    return;

  detachInterrupts();

  _isEnabled = false;
}

long RotaryEncoder::getEncoderValue()
{
  constrainValue();

  return currentValue;
}

void RotaryEncoder::constrainValue()
{
  if( currentValue < minEncoderValue )
    currentValue = maxEncoderValue;

  else if( currentValue > maxEncoderValue )
    currentValue = minEncoderValue;
}

void RotaryEncoder::setEncoderValue( long newValue )
{
  currentValue = newValue;
  constrainValue();
}

void ARDUINO_ISR_ATTR RotaryEncoder::_encoder_ISR()
{
  /**
   * Almost all of this came from a blog post by Garry on GarrysBlog.com:
   * https://garrysblog.com/2021/03/20/reliably-debouncing-rotary-encoders-with-arduino-and-esp32/
   *
   * Read more about how this works here:
   * https://www.best-microcontroller-projects.com/rotary-encoder.html
   */

  static uint8_t _previousAB = 3;
  static int8_t _encoderPosition = 0;

  bool valueChanged = false;

  _previousAB <<=2;  // Remember previous state

  if( digitalRead( encoderPinA ) ) _previousAB |= 0x02; // Add current state of pin A
  if( digitalRead( encoderPinB ) ) _previousAB |= 0x01; // Add current state of pin B

  _encoderPosition += encoderStates[( _previousAB & 0x0f )];

  /**
   * Update counter if encoder has rotated a full detent
   */

  if( _encoderPosition > 3 )        // Four steps forward
  {
    this->currentValue += 1;
    valueChanged = true;
  }
  else if( _encoderPosition < -3 )  // Four steps backwards
  {
    this->currentValue -= 1;
    valueChanged = true;
  }

  if( valueChanged )
  {
    // Reset our "step counter"
    _encoderPosition = 0;

  }
}
