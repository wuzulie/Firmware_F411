#ifndef __LEDRING12_H__
#define __LEDRING12_H__
#include "sys.h"

void ledring12Init(void);
void ledringPowerControl(bool state);
void setLedringEffect(u8 set);

#endif //__LEDRING12_H__
