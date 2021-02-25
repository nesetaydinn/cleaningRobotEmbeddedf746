/*
 * motorDriverInterface.h
 *
 *  Created on: Feb 15, 2021
 *      Author: neset
 */

#ifndef MOTORDRIVERINTERFACE_H_
#define MOTORDRIVERINTERFACE_H_

#include "main.h"


#define DRIVERCHANNEL 2 //how many are there driver
//Note: uart timeout value when be long then data transmission is safe
#define TIMEOUTVAL 20 //uart transmission timeout preiod

typedef struct{
    uint8_t pid_kd;
    uint8_t pid_ki;
    uint8_t pid_kp;
    uint8_t factor;
    uint16_t angle;
}mD_interface;

#if DRIVERCHANNEL == 1
#define MDI_channel1 huart1
extern UART_HandleTypeDef MDI_channel1;

uint8_t getDriver1kd(void);
uint8_t getDriver1ki(void);
uint8_t getDriver1kp(void);
uint8_t getDriver1factor(void);
uint16_t getDriver1angle(void);

void MDI_sendDataChannel1(uint16_t angleVal,uint8_t kd,uint8_t ki,uint8_t kp,uint8_t rate );
void MDI_getDataChannel1(void);
mD_interface getDriver1ReceiveVal(void);
#elif DRIVERCHANNEL == 2
#define MDI_channel1 huart6
extern UART_HandleTypeDef MDI_channel1;

#define MDI_channel2 huart7 //uart channel
extern UART_HandleTypeDef MDI_channel2;

uint8_t getDriver1kd(void);
uint8_t getDriver1ki(void);
uint8_t getDriver1kp(void);
uint8_t getDriver1factor(void);
uint16_t getDriver1angle(void);

uint8_t getDriver2kd(void);
uint8_t getDriver2ki(void);
uint8_t getDriver2kp(void);
uint8_t getDriver2factor(void);
uint16_t getDriver2angle(void);

void MDI_sendDataChannel1(uint16_t angleVal,uint8_t kd,uint8_t ki,uint8_t kp,uint8_t rate );
void MDI_getDataChannel1(void);
void MDI_getDataChannel1Ver2(void);
void MDI_sendDataChannel2(uint16_t angleVal,uint8_t kd,uint8_t ki,uint8_t kp,uint8_t rate );
void MDI_getDataChannel2(void);
void MDI_getDataChannel2Ver2(void);


mD_interface getDriver1ReceiveVal(void);
mD_interface getDriver2ReceiveVal(void);

#endif




#endif /* MOTORDRIVERINTERFACE_H_ */
