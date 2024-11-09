#include <Alfredo_NoU3.h>

const int numMeasurements = 2000;
float acceleration_x[numMeasurements];
float acceleration_y[numMeasurements];
float acceleration_z[numMeasurements];
float gyroscope_x[numMeasurements];
float gyroscope_y[numMeasurements];
float gyroscope_z[numMeasurements];

int currentIndex = 0;
bool measurementsComplete = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait here until the Serial Monitor is opened
  }

  NoU3.beginMotors();
  NoU3.beginIMUs();

  Serial.println("Serial connection established. Starting data collection...");
}

void loop() {
  NoU3.updateIMUs();
  
  // Check if data is available and measurements are not complete
  if (NoU3.checkDataIMU() && !measurementsComplete) {
    // Store measurements in arrays
    acceleration_x[currentIndex] = NoU3.acceleration_x;
    acceleration_y[currentIndex] = NoU3.acceleration_y;
    acceleration_z[currentIndex] = NoU3.acceleration_z;
    gyroscope_x[currentIndex] = NoU3.gyroscope_x;
    gyroscope_y[currentIndex] = NoU3.gyroscope_y;
    gyroscope_z[currentIndex] = NoU3.gyroscope_z;

    currentIndex++;

    // Check if we've reached the target number of measurements
    if (currentIndex >= numMeasurements) {
      measurementsComplete = true;
      calculateAndPrintAverages();
    }
  }
}

void calculateAndPrintAverages() {
  float sumAx = 0, sumAy = 0, sumAz = 0;
  float sumGx = 0, sumGy = 0, sumGz = 0;

  // Sum all values for each variable
  for (int i = 0; i < numMeasurements; i++) {
    sumAx += acceleration_x[i];
    sumAy += acceleration_y[i];
    sumAz += acceleration_z[i];
    sumGx += gyroscope_x[i];
    sumGy += gyroscope_y[i];
    sumGz += gyroscope_z[i];
  }

  // Calculate averages
  float avgAx = sumAx / numMeasurements;
  float avgAy = sumAy / numMeasurements;
  float avgAz = sumAz / numMeasurements;
  float avgGx = sumGx / numMeasurements;
  float avgGy = sumGy / numMeasurements;
  float avgGz = sumGz / numMeasurements;

  // Print averages
  Serial.println("Average values after 2000 measurements:");
  Serial.print("Acceleration X: "); Serial.println(avgAx);
  Serial.print("Acceleration Y: "); Serial.println(avgAy);
  Serial.print("Acceleration Z: "); Serial.println(avgAz);
  Serial.print("Gyroscope X: "); Serial.println(avgGx);
  Serial.print("Gyroscope Y: "); Serial.println(avgGy);
  Serial.print("Gyroscope Z: "); Serial.println(avgGz);
}