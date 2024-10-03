#ifndef ALFREDO_NOU3_H
#define ALFREDO_NOU3_H

#include "Alfredo_NoU3_PCA9.h"
#include "Alfredo_NoU3_Encoder.h"
#include "Alfredo_NoU3_LSM6.h"
#include "Alfredo_NoU3_MMC5.h"

#include <inttypes.h>

#define PIN_SNS_VERSION  1
#define PIN_SNS_VIN      2

#define PIN_SERVO_1 4
#define PIN_SERVO_2 5
#define PIN_SERVO_3 6
#define PIN_SERVO_4 7

#define RSL_PIN 45

// PWM Channels  (TODO: This should be an enum?)
// Channel 0 and 1 are already in use?
#define CHANNEL_SERVO_1 2
#define CHANNEL_SERVO_2 3
#define CHANNEL_SERVO_3 4
#define CHANNEL_SERVO_4 5
#define RSL_CHANNEL 6

// PWM Configuration
const int SERVO_PWM_RES = 13; // bits
const int SERVO_PWM_FREQ = 50; // Hz
const int RSL_PWM_RES = 10; // bits
const int RSL_PWM_FREQ = 1000; // Hz

// RSL states (TODO: This should be an enum?)
#define RSL_OFF 0
#define RSL_ON 1
#define RSL_DISABLED 2
#define RSL_ENABLED 3

// Drivetrain configurations (TODO: This should be an enum?)
#define TWO_MOTORS 0
#define FOUR_MOTORS 1


class NoU_Agent {
    public:
		void begin(){ NoU3_PCA9_Begin(); };
		float getBatteryVoltage(){ return analogReadMilliVolts(PIN_SNS_VIN) * 0.001 * 7.818; };
		float getVersionVoltage(){ return analogReadMilliVolts(PIN_SNS_VERSION) * 0.001 ; };
};

class NoU_Motor {
    public:
        NoU_Motor(uint8_t motorPort);
        void set(float output);
        void setInverted(boolean isInverted);
        void setMinimumOutput(float minimumOutput);
        void setMaximumOutput(float maximumOutput);
        void setExponent(float exponent);
        void setDeadband(float deadband);
    private:
        float applyCurve(float output);
        uint8_t motorPort;
        boolean inverted = false;
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
extern NoU_Agent NoU3;
#endif

#endif
