/*
 * motorDriverInterface.h
 *
 *  Created on: Feb 15, 2021
 *      Author: neset
 */

#ifndef MOTORDRIVERINTERFACE_H_
#define MOTORDRIVERINTERFACE_H_

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#define DRIVERCHANNEL 2 //how many are there driver
//Note: uart timeout value when be long then data transmission is safe
#define TIMEOUTVAL 10 //uart transmission timeout preiod

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

void MDI_sendDataChannel1(uint16_t angleVal,uint8_t kp,uint8_t ki,uint8_t kd,uint8_t rate );
void MDI_sendDataChannel1Ver2(uint16_t angleVal,uint8_t kp,uint8_t ki,uint8_t kd,uint8_t rate );
void MDI_getDataChannel1(void);
void MDI_getDataChannel1Ver2(void);
void MDI_getDataChannel1Ver3(void);
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

void MDI_sendDataChannel1(uint16_t angleVal,uint8_t kp,uint8_t ki,uint8_t kd,uint8_t rate );
void MDI_sendDataChannel1Ver2(uint16_t angleVal,uint8_t kp,uint8_t ki,uint8_t kd,uint8_t rate );
void MDI_getDataChannel1(void);
void MDI_getDataChannel1Ver2(void);
void MDI_getDataChannel1Ver3(void);
void MDI_getDataChannel1Ver4(void);
void MDI_sendDataChannel2(uint16_t angleVal,uint8_t kp,uint8_t ki,uint8_t kd,uint8_t rate );
void MDI_sendDataChannel2Ver2(uint16_t angleVal,uint8_t kp,uint8_t ki,uint8_t kd,uint8_t rate );
void MDI_getDataChannel2(void);
void MDI_getDataChannel2Ver2(void);
void MDI_getDataChannel2Ver3(void);
void MDI_getDataChannel2Ver4(void);

mD_interface getDriver1ReceiveVal(void);
mD_interface getDriver2ReceiveVal(void);

#endif

//"%d - %d - %d - %d - %d - %d - %d - %d - %d\n",rec1Buff[0],rec1Buff[1],rec1Buff[2],rec1Buff[3],rec1Buff[4],rec1Buff[5],rec1Buff[6],rec1Buff[7],rec1Buff[8]
//"DRV 1: angle: %d - kd: %d - ki: %d - kp: %d - factor: %d\n",driver1.angle,driver1.pid_kd,driver1.pid_ki,driver1.pid_kp,driver1.factor
//"%d - %d - %d - %d - %d - %d - %d - %d - %d\n",rec1Buff[0],rec1Buff[1],rec1Buff[2],rec1Buff[3],rec1Buff[4],rec1Buff[5],rec1Buff[6],rec1Buff[7],rec1Buff[8]
//"DRV 1: angle: %d - kd: %d - ki: %d - kp: %d - factor: %d\n",driver1.angle,driver1.pid_kd,driver1.pid_ki,driver1.pid_kp,driver1.factor
//HAL_UART_Transmit(&MDI_channel2,(uint8_t*)&sendBuff,sizeof(sendBuff),TIMEOUTVAL);
//"%d - %d - %d - %d - %d - %d - %d - %d - %d - %d\n",sendBuff[0],sendBuff[1],sendBuff[2],sendBuff[3],sendBuff[4],sendBuff[5],sendBuff[6],sendBuff[7],sendBuff[8],sendBuff[9]
//HAL_UART_Transmit(&MDI_channel2,(uint8_t*)&sendBuff,sizeof(sendBuff),TIMEOUTVAL);
	//"%d - %d - %d - %d - %d - %d - %d - %d - %d - %d\n",sendBuff[0],sendBuff[1],sendBuff[2],sendBuff[3],sendBuff[4],sendBuff[5],sendBuff[6],sendBuff[7],sendBuff[8],sendBuff[9]
//"DRV 2: angle: %d - kd: %d - ki: %d - kp: %d - factor: %d\n",driver2.angle,driver2.pid_kd,driver2.pid_ki,driver2.pid_kp,driver2.factor
//"DRV 2: angle: %d - kd: %d - ki: %d - kp: %d - factor: %d\n",driver2.angle,driver2.pid_kd,driver2.pid_ki,driver2.pid_kp,driver2.factor
//"DRV 2: angle: %d - kd: %d - ki: %d - kp: %d - factor: %d\n",driver2.angle,driver2.pid_kd,driver2.pid_ki,driver2.pid_kp,driver2.factor



#endif /* MOTORDRIVERINTERFACE_H_ */
