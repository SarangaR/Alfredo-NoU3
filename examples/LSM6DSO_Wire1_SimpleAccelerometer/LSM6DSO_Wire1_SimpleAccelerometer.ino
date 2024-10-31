#include <Alfredo_NoU3.h>

int interruptPinLSM6 = 48;

float acceleration_x, acceleration_y, acceleration_z;
float gyroscope_x, gyroscope_y, gyroscope_z;

volatile bool newDataAvailable = true;

void setup() {

  NoU3.begin();
  Serial.begin(115200);

  if (LSM6.begin(Wire1) == false) {
    Serial.println("LSM6 did not respond - check your wiring. Freezing.");
    while (true) {};
  }
  pinMode(interruptPinLSM6, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPinLSM6), interruptRoutine, RISING);
  LSM6.enableInterrupt();

  Serial.print("Accelerometer sample rate = ");
  Serial.print(LSM6.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("X\tY\tZ\tX\tY\tZ");

  newDataAvailable = true;
}

void loop() {
  if (newDataAvailable == true) {
    newDataAvailable = false;
    if (LSM6.accelerationAvailable()) {
      LSM6.readAcceleration(&acceleration_x, &acceleration_y, &acceleration_z);  // Results are in g (earth gravity).
    }

    if (LSM6.gyroscopeAvailable()) {
      LSM6.readGyroscope(&gyroscope_x, &gyroscope_y, &gyroscope_z);  // Results are in degrees/second.
    }

    printState();
  }
}

void interruptRoutine() {
  newDataAvailable = true;
}

void printState() {
  Serial.print(acceleration_x);
  Serial.print('\t');
  Serial.print(acceleration_y);
  Serial.print('\t');
  Serial.print(acceleration_z);
  Serial.print('\t');
  Serial.print(gyroscope_x);
  Serial.print('\t');
  Serial.print(gyroscope_y);
  Serial.print('\t');
  Serial.println(gyroscope_z);
}