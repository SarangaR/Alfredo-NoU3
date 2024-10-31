#include <Alfredo_NoU3.h>

int interruptPinMMC5 = 47;

float magnetometer_X = 0, magnetometer_Y = 0, magnetometer_Z = 0;

volatile bool newDataAvailable = true;

void setup()
{
  NoU3.begin();
  Serial.begin(115200);

  if (MMC5.begin(Wire1) == false){
    Serial.println("MMC5983MA did not respond - check your wiring. Freezing.");
    while (true){};
  }
  MMC5.softReset();
  MMC5.setFilterBandwidth(800);
  MMC5.setContinuousModeFrequency(100);
  MMC5.enableAutomaticSetReset();
  MMC5.enableContinuousMode();
  pinMode(interruptPinMMC5, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPinMMC5), interruptRoutine, RISING);
  MMC5.enableInterrupt();

  newDataAvailable = true;
}

void loop()
{
  if (newDataAvailable == true){
    newDataAvailable = false;
    MMC5.clearMeasDoneInterrupt();

    MMC5.readAccelerometer(&magnetometer_X, &magnetometer_Y, &magnetometer_Z); // Results are in uT (microteslas).

    Serial.print("X: ");
    Serial.print(magnetometer_X, 3);

    Serial.print("   Y: ");
    Serial.print(magnetometer_Y, 3);
    
    Serial.print("   Z: ");
    Serial.println(magnetometer_Z, 3);
  }
}

void interruptRoutine() {
    newDataAvailable = true;
}


