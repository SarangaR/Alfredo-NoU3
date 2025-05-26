/**
 * Example code for an encodered motor controlled with PestoLink: https://pestol.ink
 * The NoU3 documentation and tutorials can be found at https://alfredo-nou3.readthedocs.io/
 */

#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>

NoU_Motor encoderedMotor(2);

void setup() {
    Wire.begin(PIN_I2C_SDA_QWIIC, PIN_I2C_SCL_QWIIC, 400000);

    Wire1.begin(PIN_I2C_SDA_IMU, PIN_I2C_SCL_IMU, 400000);
    NoU3.beginMotors();

    pinMode(RSL_PIN, OUTPUT);
    
    PestoLink.begin("NoU3_Encodered_Motor");
    Serial.begin(115200);

    beginEncoder();
}

unsigned long lastPrintTime = 0;

void loop() {
    if (lastPrintTime + 100 < millis()){
        Serial.println(getPosition());
        lastPrintTime = millis();
    }
    
    // Here we decide what the throttle and rotation direction will be based on gamepad inputs   
    if (PestoLink.update()) {
        float throttle = PestoLink.getAxis(1);
        
        encoderedMotor.set(throttle);

        //NoU3.setServiceLight(LIGHT_ENABLED);
    } else {
        //NoU3.setServiceLight(LIGHT_DISABLED);
    }

    // No need to mess with this code
    //NoU3.updateServiceLight();
}

uint8_t pinA = 18;
uint8_t pinB = 17;

volatile int32_t position = 0;
volatile uint8_t prevState = 0;

void beginEncoder() {
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pinA), updateISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinB), updateISR, CHANGE);
}

int32_t getPosition() {
    noInterrupts();
    int32_t pos = position;
    interrupts();
    return pos;
}

void updateISR() {
    uint8_t state = (digitalRead(pinA) << 1) | digitalRead(pinB);
    int8_t transition = (prevState << 2) | state;

    // Lookup table for quadrature direction
    const int8_t dirLookup[16] = {0, -1, 1, 0,
                                  1, 0, 0, -1,
                                 -1, 0, 0, 1,
                                  0, 1, -1, 0};

    position += dirLookup[transition];
    prevState = state;

    digitalWrite(RSL_PIN, !digitalRead(RSL_PIN));
}