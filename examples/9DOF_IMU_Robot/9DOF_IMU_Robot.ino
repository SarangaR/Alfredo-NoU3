#include <Alfredo_NoU3.h>

int interruptPinLSM6 = 48;
int interruptPinMMC5 = 47;

float acceleration_x, acceleration_y, acceleration_z;
float gyroscope_x, gyroscope_y, gyroscope_z;
float magnetometer_X, magnetometer_Y, magnetometer_Z;

volatile bool newDataAvailableLSM6 = true;
volatile bool newDataAvailableMMC5 = true;

void setup() {
  NoU3.begin();
  Serial.begin(115200);

  // Initialize LSM6
  if (LSM6.begin(Wire1) == false) {
    Serial.println("LSM6 did not respond - check your wiring. Freezing.");
    while (true) {};
  }
  pinMode(interruptPinLSM6, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPinLSM6), interruptRoutineLSM6, RISING);
  LSM6.enableInterrupt();

  // Initialize MMC5
  if (MMC5.begin(Wire1) == false) {
    Serial.println("MMC5983MA did not respond - check your wiring. Freezing.");
    while (true) {};
  }
  MMC5.softReset();
  MMC5.setFilterBandwidth(800);
  MMC5.setContinuousModeFrequency(100);
  MMC5.enableAutomaticSetReset();
  MMC5.enableContinuousMode();
  pinMode(interruptPinMMC5, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPinMMC5), interruptRoutineMMC5, RISING);
  MMC5.enableInterrupt();

  Serial.println();
  Serial.println("Accel_X\tAccel_Y\tAccel_Z\tGyro_X\tGyro_Y\tGyro_Z\tMag_X\tMag_Y\tMag_Z");
}

void loop() {
  // Check LSM6 for new data
  if (newDataAvailableLSM6) {
    newDataAvailableLSM6 = false;

    if (LSM6.accelerationAvailable()) {
      LSM6.readAcceleration(&acceleration_x, &acceleration_y, &acceleration_z);
    }

    if (LSM6.gyroscopeAvailable()) {
      LSM6.readGyroscope(&gyroscope_x, &gyroscope_y, &gyroscope_z);
    }
  }

  // Check MMC5983MA for new data
  if (newDataAvailableMMC5) {
    newDataAvailableMMC5 = false;
    MMC5.clearMeasDoneInterrupt();

    MMC5.readAccelerometer(&magnetometer_X, &magnetometer_Y, &magnetometer_Z);  // Results in µT (microteslas).
  }

  formatPrint(acceleration_x);
  formatPrint(acceleration_y);
  formatPrint(acceleration_z);
  formatPrint(gyroscope_x);
  formatPrint(gyroscope_y);
  formatPrint(gyroscope_z);
  formatPrint(magnetometer_X);
  formatPrint(magnetometer_Y);
  formatPrint(magnetometer_Z);
  Serial.println('\t');
}

void interruptRoutineLSM6() {
  newDataAvailableLSM6 = true;
}

void interruptRoutineMMC5() {
  newDataAvailableMMC5 = true;
}

void formatPrint(float num) {
  char buffer[11];
  snprintf(buffer, sizeof(buffer), "%7.3f   ", num);
  Serial.print(buffer);
}
