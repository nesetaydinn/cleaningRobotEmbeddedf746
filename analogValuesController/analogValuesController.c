/*
 * analogValuesController.c
 *
 *  Created on: Feb 16, 2021
 *      Author: neset
 */

#include "analogValuesController.h"
#if ANALOGINPUTVAL==1
uint16_t analogValue1;
uint16_t readAndGetAnalog1Value(ADC_HandleTypeDef *adc){
	  HAL_ADC_Start(adc);
	  HAL_ADC_PollForConversion(adc, ADCTIMEOUT);
	  analogValue1 = HAL_ADC_GetValue(adc);
	  HAL_ADC_Stop (adc);
}
uint16_t getAnalogValue1(void){return analogValue1;}
#elif ANALOGINPUTVAL==2
uint16_t analogValue1;
uint16_t analogValue2;
void readAnalog2Values(ADC_HandleTypeDef *adc1,ADC_HandleTypeDef *adc2){
	  HAL_ADC_Start(adc1);
	  HAL_ADC_PollForConversion(adc1, ADCTIMEOUT);
	  analogValue1 = HAL_ADC_GetValue(adc1);
	  HAL_ADC_Stop (adc1);
	  HAL_ADC_Start(adc2);
	  HAL_ADC_PollForConversion(adc2, ADCTIMEOUT);
	  analogValue2 = HAL_ADC_GetValue(adc2);
	  HAL_ADC_Stop (adc2);
}
uint16_t getAnalogValue1(void){return analogValue1;}
uint16_t getAnalogValue2(void){return analogValue2;}
#endif



/**
 * @brief Creating mapping values for input values
 * @param inValue -> our input value
 * @param inMin -> input interval minumum value
 * @param inMax -> input interval maximum value
 * @param outMin -> output interval minumum value
 * @param outMax -> output interval maximum value
 * @return output value
 */
uint16_t valuesMap(uint16_t inValue, uint16_t inMin, uint16_t inMax, uint16_t outMin, uint16_t outMax) {
	return (inValue - inMin)*(outMax - outMin) / (inMax - inMin) + outMin;
}




