
#ifndef ALFREDO_NOU3_ENCODER_H
#define ALFREDO_NOU3_ENCODER_H

#if defined( ARDUINO ) && ARDUINO >= 100
  #include <Arduino.h>

#elif defined( WIRING )
  #include <Wiring.h>

#else
  #include <WProgram.h>
  #include <pins_arduino.h>

#endif

#if defined( ESP32 )
  #define RE_ISR_ATTR IRAM_ATTR

  #ifdef ARDUINO_ISR_ATTR
    #undef ARDUINO_ISR_ATTR
    #define ARDUINO_ISR_ATTR IRAM_ATTR
  #endif

  #if defined( ESP_ARDUINO_VERSION ) && ( ESP_ARDUINO_VERSION == ESP_ARDUINO_VERSION_VAL(2,0,10) )
    /**
     * BUG ALERT!
     *
     * With Arduino-ESP32 core 2.0.10, the #include statement below
     * fails to compile due to a bug.
     * Also see `attachInterrupts()` in ESP32RotaryEncoder.cpp for
     * the note about the `attachInterrupt()` macro in 2.x cores.
     */
    #error Please upgrade the Arduino-ESP32 core to use this library.
  #else
    #include <FunctionalInterrupt.h>
  #endif
#endif

#define RE_LOOP_INTERVAL 100000U  // 0.1 seconds

typedef enum {
  FLOATING,
  HAS_PULLUP,
  SW_FLOAT
} EncoderType;

class RotaryEncoder {

  protected:

    #if defined( ESP32 )
      typedef std::function<void(long)> EncoderCallback;
    #else
      typedef void (*EncoderCallback)(long);
    #endif


  public:

    /**
     * @brief Construct a new Rotary Encoder instance
     *
     * @param encoderPinA        The A pin on the encoder, sometimes marked "CLK"
     * @param encoderPinB        The B pin on the encoder, sometimes marked "DT"

     */
    RotaryEncoder(
      uint8_t encoderPinA,
      uint8_t encoderPinB
    );

    /**
     * @brief Responsible for detaching interrupts and clearing the loop timer
     *
     */
    ~RotaryEncoder();

    /**
     * @brief Specifies whether the encoder pins need to use the internal pull-up resistors.
     *
     * @note Call this in `setup()`.
     *
     * @param type  FLOATING if you're using a raw encoder not mounted to a PCB (internal pull-ups will be used);
     *              HAS_PULLUP if your encoder is a module that has pull-up resistors, (internal pull-ups will not be used);
     *              SW_FLOAT your encoder is a module that has pull-up resistors, but the resistor for the switch is missing (internal pull-up will be used for switch input only)
     */
    void setEncoderType( EncoderType type );

    /**
     * @brief Set the minimum and maximum values that the encoder will return.
     *
     * @note This is a convenience function that calls `setMinValue()`, `setMaxValue()`, and `setCircular()`
     *
     * @param minValue      Minimum value (e.g. 0)
     * @param maxValue      Maximum value (e.g. 10)
     */
    void setBoundaries( long minValue, long maxValue);

    /**
     * @brief Sets up the GPIO pins specified in the constructor and attaches the ISR callback for the encoder.
     *
     * @note Call this in `setup()` after other "set" methods.
     *
     */
    void begin();

    /**
     * @brief Enables the encoder knob and pushbutton if `disable()` was previously used.
     *
     */
    void enable();

    /**
     * @brief Disables the encoder knob and pushbutton.
     *
     * Knob rotation and button presses will have no effect until after `enable()` is called
     *
     */
    void disable();

    /**
     * @brief Confirms whether the encoder knob and pushbutton have been disabled.
     *
     */
    bool isEnabled();


    /**
     * @brief Get the current value tracked by the encoder.
     *
     * @return A value between the minimum and maximum configured by `setBoundaries()`
     */
    long getEncoderValue();

    /**
     * @brief Override the value tracked by the encoder.
     *
     * @note If the new value is outside the minimum or maximum configured
     * by `setBoundaries()`, it will be adjusted accordingly
     *
     * @param newValue
     */
    void setEncoderValue( long newValue );

    /**
     * @brief Reset the value tracked by the encoder.
     *
     * @note This will try to set the value to 0, but if the minimum and maximum configured
     * by `setBoundaries()` does not include 0, then the minimum or maximum will be
     * used instead
     *
     */
    void resetEncoderValue() { setEncoderValue( 0 ); }

  private:

    const char *LOG_TAG = "ESP32RotaryEncoder";

    typedef enum {
        LEFT  = -1,
        STILL =  0,
        RIGHT =  1
    } Rotation;

    Rotation encoderStates[16] = {
      STILL, LEFT,  RIGHT, STILL,
      RIGHT, STILL, STILL, LEFT,
      LEFT,  STILL, STILL, RIGHT,
      STILL, RIGHT, LEFT,  STILL
    };

    int encoderPinMode = INPUT;

    uint8_t encoderPinA;
    uint8_t encoderPinB;

    /**
     * @brief Determines whether knob turns or button presses will be ignored.  ISRs still fire,
     *
     * Set by `enable()` and `disable()`.
     *
     */
    bool _isEnabled = true;

    /**
     * @brief Sets the minimum and maximum values of `currentValue`.
     *
     * Set in `setBoundaries()` and used in `constrainValue()`.
     *
     */
    long minEncoderValue = -1; long maxEncoderValue = 1;


    /**
     * @brief The value tracked by `encoder_ISR()` when the encoder knob is turned.
     *
     * This value can be overwritten in `constrainValue()` whenever
     * `getEncoderValue()` or `setEncoderValue()` are called.
     *
     */
    volatile long currentValue;


    /**
     * @brief Constrains the value set by `encoder_ISR()` and `setEncoderValue()`
     * to be in the range set by `setBoundaries()`.
     *
     */
    void constrainValue();

    /**
     * @brief Attaches ISRs to encoder and button pins.
     *
     * Used in `begin()` and `enable()`.
     *
     */
    void attachInterrupts();

    /**
     * @brief Detaches ISRs from encoder and button pins.
     *
     * Used in the destructor and in `disable()`.
     *
     */
    void detachInterrupts();



    void ARDUINO_ISR_ATTR _encoder_ISR();

};

#endif
