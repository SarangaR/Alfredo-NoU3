
#include <Alfredo_NoU3.h>

SFE_MMC5983MA myMag;

int interruptPin = 34;

volatile bool newDataAvailable = true;
uint32_t rawValueX = 0;
uint32_t rawValueY = 0;
uint32_t rawValueZ = 0;
double scaledX = 0;
double scaledY = 0;
double scaledZ = 0;
double heading = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println("MMC5983MA Example");
  
  Wire1.begin(35, 36);

  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), interruptRoutine, RISING);

  if (myMag.begin(Wire1) == false){
    Serial.println("MMC5983MA did not respond - check your wiring. Freezing.");
    while (true);
  }

  myMag.softReset();
  myMag.setFilterBandwidth(800);
  myMag.setContinuousModeFrequency(100);
  myMag.enableAutomaticSetReset();
  myMag.enableContinuousMode();
  myMag.enableInterrupt();

  newDataAvailable = true;
}

void loop()
{
  if (newDataAvailable == true){
    newDataAvailable = false;
    myMag.clearMeasDoneInterrupt();

    myMag.readFieldsXYZ(&rawValueX, &rawValueY, &rawValueZ);

    scaledX = (double)rawValueX - 131072.0;
    scaledX /= 131072.0;

    scaledY = (double)rawValueY - 131072.0;
    scaledY /= 131072.0;

    scaledZ = (double)rawValueZ - 131072.0;
    scaledZ /= 131072.0;

    Serial.print("X: ");
    Serial.print(scaledX * 800.0, 3);

    Serial.print("   Y: ");
    Serial.print(scaledY * 800.0, 3);
    
    Serial.print("   Z: ");
    Serial.println(scaledZ * 800.0, 3);
  }
}

void interruptRoutine() {
    newDataAvailable = true;
}
