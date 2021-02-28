/*
 * taskManagerInterface.h
 *
 *  Created on: Feb 25, 2021
 *      Author: neset
 */

#ifndef TASKMANAGERINTERFACE_H_
#define TASKMANAGERINTERFACE_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "main.h"

#define ADCREADCH1 hadc1
extern ADC_HandleTypeDef ADCREADCH1;

#define ADCREADCH2 hadc3
extern ADC_HandleTypeDef ADCREADCH2;

void tasks_init(void);

#endif /* TASKMANAGERINTERFACE_H_ */
