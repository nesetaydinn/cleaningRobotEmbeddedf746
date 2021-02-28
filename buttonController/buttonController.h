/*
 * buttonController.h
 *
 *  Created on: Feb 17, 2021
 *      Author: neset
 */

#ifndef BUTTONCONTROLLER_H_
#define BUTTONCONTROLLER_H_

#include "main.h"
#include "stdbool.h"
#include "../motorDriverInterface/motorDriverInterface.h"

#define BTNCOUNTER htim2
extern TIM_HandleTypeDef BTNCOUNTER;

/* @brief You must give labels when create in cubemx project pins
 * @param select -> select pin
 * @param plus -> plus pin
 * @param minus -> minus pin
 * */
#define READ_SELECT_BTN HAL_GPIO_ReadPin(select_GPIO_Port,select_Pin)
#define READ_PLUS_BTN HAL_GPIO_ReadPin(plus_GPIO_Port,plus_Pin)
#define READ_MINUS_BTN HAL_GPIO_ReadPin(minus_GPIO_Port,minus_Pin)

#define ITEM_MAX 3 //items maximum index
#define ITEM_MIN 0 //items minumum index

#define BTN_SHORT_PRESS_TIME 10
#define BTN_LONG_PRESS_TIME 500

void buttonCounterIncrease(void);
void buttonController(void);
void btnParameterInit(void);
void setDriver1AngleValue(uint16_t angle);
void setDriver2AngleValue(uint16_t angle);
void transmissionDriver1(void);
void transmissionDriver2(void);

mD_interface getDriver1TransmitVal(void);
mD_interface getDriver2TransmitVal(void);
uint8_t getSelectedLcdVal(void);
uint8_t getSelectedLcdItemVal(void);
void lcdController(void);

#endif /* BUTTONCONTROLLER_H_ */
