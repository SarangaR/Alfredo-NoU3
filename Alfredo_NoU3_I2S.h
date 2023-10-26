#include <Arduino.h>

#ifndef H_ALFREDO_NOU3_I2S
#define H_ALFREDO_NOU3_I2S

void NoU_I2S_Begin();
void NoU_I2S_Update();
void NoU_I2S_SetMotor(uint8_t motorPort, int16_t motorPower);

#endif // H_ALFREDO_NOU3_I2S