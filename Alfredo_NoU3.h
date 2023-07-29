#ifndef ALFREDO_NOU3_H
#define ALFREDO_NOU3_H

#include <inttypes.h>

// Pins (TODO: These should be static variables?)
#define PIN_MOTOR_STORE 17
#define PIN_MOTOR_CLOCK 18
#define PIN_MOTOR_DATA 21
#define PIN_MOTOR_NSLEEP -1 //(TODO: TBD)

#define PIN_SERVO1 7
#define PIN_SERVO2 6
#define PIN_SERVO3 5
#define PIN_SERVO4 4

#define RSL_PIN 41 // Same as built-in LED

// PWM Channels  (TODO: This should be an enum?)
#define CHANNEL_SERVO1 0
#define CHANNEL_SERVO2 1
#define CHANNEL_SERVO3 2
#define CHANNEL_SERVO4 3
#define RSL_CHANNEL 4

// PWM Configuration (TODO: These should be static variables?)
#define MOTOR_PWM_RES 10 // bits
#define MOTOR_PWM_FREQ 39000 // Based on AF Motor Shield, uses 39kHz for DC Motors
#define SERVO_PWM_RES 16 // bits
#define SERVO_PWM_FREQ 50 // Hz
#define RSL_PWM_RES 10 // bits
#define RSL_PWM_FREQ 1000 // Hz

// Motor states (TODO: This should be an enum?)
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4

// RSL states (TODO: This should be an enum?)
#define RSL_OFF 0
#define RSL_ON 1
#define RSL_DISABLED 2
#define RSL_ENABLED 3

// Drivetrain configurations (TODO: This should be an enum?)
#define TWO_MOTORS 0
#define FOUR_MOTORS 1

class NoU_SpiAgent {
    public:
        void begin();
        uint8_t update();
    private:
        volatile uint8_t speed = 0x80;
        volatile uint16_t spiDataOn = 0x5555;
        static constexpr uint16_t spiDataOff = 0x0000;
};

class NoU_Motor {
    public:
        NoU_Motor(uint8_t motorPort);
        void set(float output);
        void setState(uint8_t state);
        void setPower(uint16_t power);
        void setInverted(boolean isInverted);
        boolean isInverted();
        void setMinimumOutput(float minimumOutput);
        void setMaximumOutput(float maximumOutput);
        void setExponent(float exponent);
        void setDeadband(float deadband);
        float getOutput();
    private:
        float applyCurve(float output);
        uint8_t aPin;
        uint8_t bPin;
        uint8_t channel;
        boolean inverted;
        float output;
        uint8_t state;
        float minimumOutput = 0;
        float maximumOutput = 1;
        float exponent = 1;
        float deadband = 0;
};

class NoU_Servo {
    public:
        NoU_Servo(uint8_t servoPort, uint16_t minPulse = 540, uint16_t maxPulse = 2300);
        void write(float degrees);
        void writeMicroseconds(uint16_t pulseLength);
        void setMinimumPulse(uint16_t minPulse);
        void setMaximumPulse(uint16_t maxPulse);
        uint16_t getMicroseconds();
        float getDegrees();
    private:
        uint8_t pin;
        uint8_t channel;
        uint16_t minPulse;
        uint16_t maxPulse;
        uint16_t pulse;
};

class NoU_Drivetrain {
    public:
        NoU_Drivetrain(NoU_Motor* leftMotor, NoU_Motor* rightMotor);
        NoU_Drivetrain(NoU_Motor* frontLeftMotor, NoU_Motor* frontRightMotor,
                        NoU_Motor* rearLeftMotor, NoU_Motor* rearRightMotor);
        void tankDrive(float leftPower, float rightPower);
        void arcadeDrive(float throttle, float rotation, boolean invertedReverse = false);
        void curvatureDrive(float throttle, float rotation, boolean isQuickTurn = true);
        void holonomicDrive(float xVelocity, float yVelocity, float rotation);
        void setMinimumOutput(float minimumOutput);
        void setMaximumOutput(float maximumOutput);
        void setInputExponent(float inputExponent);
        void setInputDeadband(float inputDeadband);
    private:
        void setMotors(float frontLeftPower, float frontRightPower, float rearLeftPower, float rearRightPower);
        float applyInputCurve(float input);
        float applyOutputCurve(float output);
        NoU_Motor *frontLeftMotor;
        NoU_Motor *frontRightMotor;
        NoU_Motor *rearLeftMotor;
        NoU_Motor *rearRightMotor;
        uint8_t drivetrainType;
        float quickStopThreshold = 0.2;
        float quickStopAlpha = 0.1;
        float quickStopAccumulator;
        float minimumOutput = 0;
        float maximumOutput = 1;
        float inputExponent = 1;
        float inputDeadband = 0;
};

class RSL {
    public:
        static void initialize();
        static void setState(uint8_t state);
        static void update();
    private:
        static uint8_t state;
};

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_NOU3)
extern NoU_SpiAgent NoU3;
#endif

#endif
