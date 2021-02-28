/*
 * lcdDriverInterface.h
 *
 *  Created on: Feb 23, 2021
 *      Author: neset
 */

#ifndef LCDDRIVERINTERFACE_H_
#define LCDDRIVERINTERFACE_H_

#include "lcd_i2cModule.h"
#include "../motorDriverInterface/motorDriverInterface.h"
void lcd_Init(void);
void printToLcdDrv1Receive(mD_interface gets);
void printToLcdDrv1Transmit(mD_interface sends);
void printToLcdDrv2Receive(mD_interface gets);
void printToLcdDrv2Transmit(mD_interface sends);
void cleanTheLcd(void);

#endif /* LCDDRIVERINTERFACE_H_ */
