#include <Alfredo_NoU3.h>

void setup() {
  Serial.begin(115200);
  NoU3.beginMotors();
  NoU3.beginIMUs();

  Serial.println();
  Serial.println("Accel_X\tAccel_Y\tAccel_Z\tGyro_X\tGyro_Y\tGyro_Z\tMag_X\tMag_Y\tMag_Z");
}

long lastPrintMs = 0;  // Stores the last time the function was called

void loop() {

  NoU3.updateIMUs();

  if (millis() - lastPrintMs >= 200) {
    lastPrintMs = millis();  // Update the last time the function was called
    formatPrint(NoU3.acceleration_x);
    formatPrint(NoU3.acceleration_y);
    formatPrint(NoU3.acceleration_z);
    formatPrint(NoU3.gyroscope_x);
    formatPrint(NoU3.gyroscope_y);
    formatPrint(NoU3.gyroscope_z);
    formatPrint(NoU3.magnetometer_X);
    formatPrint(NoU3.magnetometer_Y);
    formatPrint(NoU3.magnetometer_Z);
    Serial.println('\t');
  }
}

void formatPrint(float num) {
  char buffer[11];
  snprintf(buffer, sizeof(buffer), "%7.3f   ", num);
  Serial.print(buffer);
}
