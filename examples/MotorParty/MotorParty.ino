#include <Alfredo_NoU3.h>

NoU_Motor motor1(1);
NoU_Motor motor2(2);
NoU_Motor motor3(3);
NoU_Motor motor4(4);
NoU_Motor motor5(5);
NoU_Motor motor6(6);
NoU_Motor motor7(7);
NoU_Motor motor8(8);

NoU_Servo servo1(1);
NoU_Servo servo2(2);
NoU_Servo servo3(3);
NoU_Servo servo4(4);

void setup() {
    NoU3.begin();
    Serial.begin(115200);
    RSL::initialize();
    RSL::setState(RSL_ENABLED);
}

void loop() {
    for (float i = -1; i < 1; i += 0.01) {
        motor1.set(i);
        motor2.set(i);
        motor3.set(i);
        motor4.set(i);
        motor5.set(i);
        motor6.set(i);
		    motor7.set(i);
        motor8.set(i);
		
        servo1.write(map(i*100, -100, 100, 0, 180));
        servo2.write(map(i*100, -100, 100, 0, 180));
        servo3.write(map(i*100, -100, 100, 0, 180));
        servo4.write(map(i*100, -100, 100, 0, 180));
		
        RSL::update();
        delay(1);
        Serial.println(i);
    }
    for (float i = 1; i > -1; i -= 0.01) {
        motor1.set(i);
        motor2.set(i);
        motor3.set(i);
        motor4.set(i);
        motor5.set(i);
        motor6.set(i);
		    motor7.set(i);
        motor8.set(i);
		
        servo1.write(map(i*100, -100, 100, 0, 180));
        servo2.write(map(i*100, -100, 100, 0, 180));
        servo3.write(map(i*100, -100, 100, 0, 180));
        servo4.write(map(i*100, -100, 100, 0, 180));
		
        RSL::update();
        delay(1);
        Serial.println(i);
    }
}