/*
 * analogValuesController.h
 *
 *  Created on: Feb 16, 2021
 *      Author: neset
 */

#ifndef ANALOGVALUESCONTROLLER_H_
#define ANALOGVALUESCONTROLLER_H_


#include "main.h"

#define ANALOGINPUTVAL 2
#define ADCTIMEOUT 5

#if ANALOGINPUTVAL==1
uint16_t readAndGetAnalog1Value(ADC_HandleTypeDef *adc);
uint16_t getAnalogValue1(void);
#elif ANALOGINPUTVAL==2
void readAnalog2Values(ADC_HandleTypeDef *adc1,ADC_HandleTypeDef *adc2);
uint16_t getAnalogValue1(void);
uint16_t getAnalogValue2(void);
#endif


uint16_t valuesMap(uint16_t inValue, uint16_t inMin, uint16_t inMax, uint16_t outMin, uint16_t outMax);


#endif /* ANALOGVALUESCONTROLLER_H_ */
