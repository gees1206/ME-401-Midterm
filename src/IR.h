#pragma once

#include <Arduino.h>
#include "dc_motors.h"
#include "MeanFilterLib.h"
void IRINIT(void);
void IRScan( void * parameter);
bool isObstical(void);
int getAvoidLeft(void);
int getAvoidRight(void);