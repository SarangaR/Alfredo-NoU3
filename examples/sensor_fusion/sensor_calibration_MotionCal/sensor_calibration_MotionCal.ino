#include <Alfredo_NoU3.h>

void setup() {
  Serial.begin(115200);
  NoU3.beginMotors();
  NoU3.beginIMUs();

  Serial.println("Accel_X\tAccel_Y\tAccel_Z\tGyro_X\tGyro_Y\tGyro_Z\tMag_X\tMag_Y\tMag_Z");
}

void loop() {
  NoU3.updateIMUs();

  if(NoU3.checkDataIMU() == true){
    printRaw();
  }
}

void printRaw(){
  // 'Raw' values to match expectation of MotionCal
  Serial.print("Raw:");
  Serial.print(int(NoU3.acceleration_x*8192.0)); Serial.print(",");
  Serial.print(int(NoU3.acceleration_y*8192.0)); Serial.print(",");
  Serial.print(int(NoU3.acceleration_z*8192.0)); Serial.print(",");
  Serial.print(int(NoU3.gyroscope_x*10.0)); Serial.print(",");
  Serial.print(int(NoU3.gyroscope_y*10.0)); Serial.print(",");
  Serial.print(int(NoU3.gyroscope_z*10.0)); Serial.print(",");
  Serial.print(int(NoU3.magnetometer_x*10.0)); Serial.print(",");
  Serial.print(int(NoU3.magnetometer_y*10.0)); Serial.print(",");
  Serial.print(int(NoU3.magnetometer_z*10.0)); Serial.println("");
}