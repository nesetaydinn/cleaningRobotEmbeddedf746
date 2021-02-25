/*
 * lcdDriverInterface.c
 *
 *  Created on: Feb 23, 2021
 *      Author: neset
 */

#include "lcdDriverInterface.h"
#include "../buttonController/buttonController.h"
char lcdBuff[20];

/* @brief lcd module initalize
 * @param none
 * @return none
 * */
void lcd_Init(void){
	LCD_i2cDeviceCheck();
	LCD_Init();
	LCD_BackLight(LCD_BL_ON);
	LCD_SetCursor(0,0);

	LCD_Send_String("TEST",STR_NOSLIDE);
}
void printToLcdDrv1Receive(mD_interface gets){
	LCD_SetCursor(1,1);
	sprintf(lcdBuff,"Gln1:a:%04d d:%02x",gets.angle,gets.pid_kd);
	LCD_Send_String(lcdBuff,STR_NOSLIDE);
	LCD_SetCursor(2,1);
	sprintf(lcdBuff,"i:%02x p:%02x f:%02x",gets.pid_ki,gets.pid_kp,gets.factor);
	LCD_Send_String(lcdBuff,STR_NOSLIDE);
}
void printToLcdDrv1Transmit(mD_interface sends){
	LCD_SetCursor(1,1);
	sprintf(lcdBuff,"Gdn1:a:%04d d:%02x",sends.angle,sends.pid_kd);
	LCD_Send_String(lcdBuff,STR_NOSLIDE);
	LCD_SetCursor(2,1);
	sprintf(lcdBuff,"i:%02x p:%02x f:%02x",sends.pid_ki,sends.pid_kp,sends.factor);
	LCD_Send_String(lcdBuff,STR_NOSLIDE);
}
void printToLcdDrv2Receive(mD_interface gets){
	LCD_SetCursor(1,1);
	sprintf(lcdBuff,"Gln2:a:%04d d:%02x",gets.angle,gets.pid_kd);
	LCD_Send_String(lcdBuff,STR_NOSLIDE);
	LCD_SetCursor(2,1);
	sprintf(lcdBuff,"i:%02x p:%02x f:%02x",gets.pid_ki,gets.pid_kp,gets.factor);
	LCD_Send_String(lcdBuff,STR_NOSLIDE);
}
void printToLcdDrv2Transmit(mD_interface sends){
	LCD_SetCursor(1,1);
	sprintf(lcdBuff,"Gdn2:a:%04d d:%02x",sends.angle,sends.pid_kd);
	LCD_Send_String(lcdBuff,STR_NOSLIDE);
	LCD_SetCursor(2,1);
	sprintf(lcdBuff,"i:%02x p:%02x f:%02x",sends.pid_ki,sends.pid_kp,sends.factor);
	LCD_Send_String(lcdBuff,STR_NOSLIDE);
}
void cleanTheLcd(void){
	LCD_Clear();
}
void lcdController(void){
	static mD_interface temp;
	static uint8_t itemVal=0,statuVal=0;
	if(0==statuVal){
		if(0==itemVal){
			temp=getDriver1TransmitVal();
			printToLcdDrv1Transmit(temp);

		}else{
			temp=getDriver1ReceiveVal();
			printToLcdDrv1Receive(temp);
		}
	}
	else{
		if(0==itemVal){
			temp=getDriver2TransmitVal();
			printToLcdDrv2Transmit(temp);

		}else{
			temp=getDriver2ReceiveVal();
			printToLcdDrv2Receive(temp);
		}
	}
	statuVal=getSelectedLcdVal();
	itemVal=getSelectedLcdItemVal();
}
