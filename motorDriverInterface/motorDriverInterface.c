/*
 * motorDriverInterface.c
 *
 *  Created on: Feb 15, 2021
 *      Author: neset
 */

#include "motorDriverInterface.h"

#if DRIVERCHANNEL == 1
mD_interface driver1;
uint8_t rec1Buff[9];
#elif DRIVERCHANNEL == 2
mD_interface driver1;
mD_interface driver2;

uint8_t rec1Buff[20];
uint8_t rec2Buff[20];
#endif
/**
 * @brief Write command to Motor Driver
 * @param uartChannel -> get uart channel
 * @param cmd -> command to write
 * @return none
 */
static void MDI_writeCommand(UART_HandleTypeDef *uartChannel, uint8_t cmd) {
	HAL_UART_Transmit(uartChannel, (uint8_t*) &cmd, sizeof(cmd), TIMEOUTVAL);
}
/**
 * @brief Write small data to Motor Driver
 * @param uartChannel -> get uart channel
 * @param data -> data to write
 * @return none
 */
static void MDI_writeSmallData(UART_HandleTypeDef *uartChannel, uint8_t data) {
	HAL_UART_Transmit(uartChannel, (uint8_t*) &data, sizeof(data), TIMEOUTVAL);
}
/**
 * @brief Write big data to Motor Driver
 * @param uartChannel -> get uart channel
 * @param buff -> get data array
 * @param buff_size -> get data array size
 * @return none
 */
static void MDI_writeBigData(UART_HandleTypeDef *uartChannel, uint8_t *buff,
		size_t buff_size) {
	while (buff_size > 0) {
		uint16_t chunk_size = buff_size > 65535 ? 65535 : buff_size;
		HAL_UART_Transmit(uartChannel, buff, chunk_size, TIMEOUTVAL);
		buff += chunk_size;
		buff_size -= chunk_size;
	}
}
/**
 * @brief Write  2 byte data to Motor Driver
 * @param uartChannel -> get uart channel
 * @param data -> get 2 byte data
 * @return none
 */
void MDI_2byteWriteData(UART_HandleTypeDef *uartChannel, uint16_t data) {
	uint8_t arrTmp[] = { data >> 8, data & 0xFF };
	MDI_writeBigData(uartChannel, arrTmp, sizeof(arrTmp));

}
#if DRIVERCHANNEL == 1
/**
 * @brief drive to Motor Driver 1
 * @param angleVal -> get motor angle value
 * @param kp -> get Pid kp value
 * @param ki -> get Pid ki value
 * @param kd -> get Pid kd value
 * @param factor -> get factor of Pid elements
 * @return none
 */
void MDI_sendDataChannel1(uint16_t angleVal, uint8_t kd, uint8_t ki, uint8_t kp,
		uint8_t factor) {
	uint16_t checksumTmp = 0;
	MDI_writeCommand(&MDI_channel1, 0xFF);
	MDI_writeCommand(&MDI_channel1, 0xFF); //Data transmission started
	MDI_2byteWriteData(&MDI_channel1, angleVal);
	uint8_t tmpArr[] = { angleVal >> 8, angleVal & 0xFF };
	checksumTmp += tmpArr[0];
	checksumTmp += tmpArr[1]; //2 byte angle val sended
	MDI_writeSmallData(&MDI_channel1, kd);
	checksumTmp += kd; //writed kd
	MDI_writeSmallData(&MDI_channel1, ki);
	checksumTmp += ki; //writed ki
	MDI_writeSmallData(&MDI_channel1, kp);
	checksumTmp += kp; //writed kp
	MDI_writeSmallData(&MDI_channel1, factor);
	checksumTmp += factor; //writed factor
	uint8_t tmp = checksumTmp % 256;
	MDI_writeSmallData(&MDI_channel1, tmp); //checksum first byte
	uint8_t tmpComp = ~tmp;
	MDI_writeSmallData(&MDI_channel1, tmpComp); //checksum second byte

}
/**
 * @brief drive to Motor Driver 1 version 2 (will use for pic drive)
 * @param angleVal -> get motor angle value
 * @param kp -> get Pid kp value
 * @param ki -> get Pid ki value
 * @param kd -> get Pid kd value
 * @param factor -> get factor of Pid elements
 * @return none
 */
void MDI_getDataChannel1Ver2(void) {
	HAL_UART_Receive(&MDI_channel1, (uint8_t*) rec1Buff, 10, TIMEOUTVAL * 10);
	if (0xFF == rec1Buff[0] && 0xFF == rec1Buff[1]) {
		uint16_t checksumTmp = 0;
		for (uint8_t c = 2; c < 8; c++)
			checksumTmp += rec1Buff[c];
		uint8_t tmp = checksumTmp % 256;
		uint8_t tmpComp = ~tmp;
		if (tmp == rec1Buff[8] && tmpComp == rec1Buff[9]) {
			driver1.angle = ((uint16_t) rec1Buff[2] << 8) | rec1Buff[3];
			driver1.pid_kp = rec1Buff[4];
			driver1.pid_ki = rec1Buff[5];
			driver1.pid_kd = rec1Buff[6];
			driver1.factor = rec1Buff[7];
		}

	}
}
/**
 * @brief get to Motor Driver 1 values
 * @return none
 */
void MDI_getDataChannel1(void){
	HAL_UART_Receive(&MDI_channel1,(uint8_t*)rec1Buff,10,50);
	if(0xFF==rec1Buff[0] && 0xFF==rec1Buff[1]){
		uint16_t checksumTmp=0;
		for(uint8_t c=2;c<8;c++)checksumTmp+=rec1Buff[c];
		uint8_t tmp =checksumTmp%256;
		uint8_t tmpComp =~tmp;
		if(tmp == rec1Buff[8] && tmpComp == rec1Buff[9]){
			driver1.angle=((uint16_t)rec1Buff[2] << 8) | rec1Buff[3];
			driver1.pid_kd=rec1Buff[4];
			driver1.pid_ki=rec1Buff[5];
			driver1.pid_kp=rec1Buff[6];
			driver1.factor=rec1Buff[7];
		}
	}
	for(uint8_t c=2;c<8;c++)rec1Buff[c]=0;
	//"angle: %d - kd: %d - ki: %d - kp: %d - factor: %d\n",driver1.angle,driver1.pid_kd,driver1.pid_ki,driver1.pid_kp,driver1.factor
}
#elif DRIVERCHANNEL == 2
/**
 * @brief drive to Motor Driver 1
 * @param angleVal -> get motor angle value
 * @param kp -> get Pid kp value
 * @param ki -> get Pid ki value
 * @param kd -> get Pid kd value
 * @param factor -> get factor of Pid elements
 * @return none
 */
void MDI_sendDataChannel1(uint16_t angleVal, uint8_t kd, uint8_t ki, uint8_t kp,
		uint8_t factor) {
	uint16_t checksumTmp = 0;
	MDI_writeCommand(&MDI_channel1, 0xFF);
	MDI_writeCommand(&MDI_channel1, 0xFF); //Data transmission started
	MDI_2byteWriteData(&MDI_channel1, angleVal);
	uint8_t tmpArr[] = { angleVal >> 8, angleVal & 0xFF };
	checksumTmp += tmpArr[0];
	checksumTmp += tmpArr[1]; //2 byte angle val sended
	MDI_writeSmallData(&MDI_channel1, kd);
	checksumTmp += kd; //writed kd
	MDI_writeSmallData(&MDI_channel1, ki);
	checksumTmp += ki; //writed ki
	MDI_writeSmallData(&MDI_channel1, kp);
	checksumTmp += kp; //writed kp
	MDI_writeSmallData(&MDI_channel1, factor);
	checksumTmp += factor; //writed factor
	uint8_t tmp = checksumTmp % 256;
	MDI_writeSmallData(&MDI_channel1, tmp); //checksum first byte
	uint8_t tmpComp = ~tmp;
	MDI_writeSmallData(&MDI_channel1, tmpComp); //checksum second byte

}
/**
 * @brief drive to Motor Driver 1 version 2 (will use for pic drive)
 * @param angleVal -> get motor angle value
 * @param kp -> get Pid kp value
 * @param ki -> get Pid ki value
 * @param kd -> get Pid kd value
 * @param factor -> get factor of Pid elements
 * @return none
 */
void MDI_sendDataChannel1Ver2(uint16_t angleVal, uint8_t kp, uint8_t ki,
		uint8_t kd, uint8_t factor) {
	uint16_t checksumTmp = 0;
	uint8_t tmpArr[] = { angleVal >> 8, angleVal & 0xFF };
	checksumTmp += tmpArr[0];
	checksumTmp += tmpArr[1];
	checksumTmp += kp;
	checksumTmp += ki;
	checksumTmp += kd;
	checksumTmp += factor;
	uint8_t tmp = checksumTmp % 256;
	uint8_t tmpComp = ~tmp;
	uint8_t sendBuff[10] = { 0XFF, 0XFF, tmpArr[0], tmpArr[1], kp, ki, kd,
			factor, tmp, tmpComp };
	for (uint8_t counter = 0; counter < 10; counter++) {
		MDI_writeSmallData(&MDI_channel1, sendBuff[counter]);
		vTaskDelay(50);
	}
}

/**
 * @brief get to Motor Driver 1 values
 * @return none
 */
uint8_t getFirstData1;
void MDI_getDataChannel1(void) {
	HAL_UART_Receive(&MDI_channel1, (uint8_t*) &getFirstData1, 1, TIMEOUTVAL);
	if (0xFF == getFirstData1) {
		HAL_UART_Receive(&MDI_channel1, (uint8_t*) rec1Buff, 9, TIMEOUTVAL * 9);
		if (0xFF == rec1Buff[0]) {
			uint16_t checksumTmp = 0;
			for (uint8_t c = 1; c < 7; c++)
				checksumTmp += rec1Buff[c];
			uint8_t tmp = checksumTmp % 256;
			uint8_t tmpComp = ~tmp;
			if (tmp == rec1Buff[7] && tmpComp == rec1Buff[8]) {
				driver1.angle = ((uint16_t) rec1Buff[1] << 8) | rec1Buff[2];
				driver1.pid_kp = rec1Buff[3];
				driver1.pid_ki = rec1Buff[4];
				driver1.pid_kd = rec1Buff[5];
				driver1.factor = rec1Buff[6];
			}
		}
		for (uint8_t c = 0; c < 9; c++)
			rec1Buff[c] = 0;
	}
	getFirstData1 = 0;
}
/**
 * @brief get to Motor Driver 1 values
 * @return none
 */
void MDI_getDataChannel1Ver2(void) {
	HAL_UART_Receive(&MDI_channel1, (uint8_t*) rec1Buff, 10, TIMEOUTVAL * 10);
	if (0xFF == rec1Buff[0] && 0xFF == rec1Buff[1]) {
		uint16_t checksumTmp = 0;
		for (uint8_t c = 2; c < 8; c++)
			checksumTmp += rec1Buff[c];
		uint8_t tmp = checksumTmp % 256;
		uint8_t tmpComp = ~tmp;
		if (tmp == rec1Buff[8] && tmpComp == rec1Buff[9]) {
			driver1.angle = ((uint16_t) rec1Buff[2] << 8) | rec1Buff[3];
			driver1.pid_kp = rec1Buff[4];
			driver1.pid_ki = rec1Buff[5];
			driver1.pid_kd = rec1Buff[6];
			driver1.factor = rec1Buff[7];
		}

	}
}
/**
 * @brief get to Motor Driver 1 values
 * @return none
 */
void MDI_getDataChannel1Ver3(void) {
	static uint8_t tmpArr[10];
	static uint8_t counter=0,getTmp=0,getTmpBeff=0;
	if((MDI_channel1.Instance->ISR & USART_ISR_RXNE) != 0){
		getTmp = MDI_channel1.Instance->RDR;
		if(0xFF ==getTmp && 0xFF ==getTmpBeff){
			tmpArr[0]=0xFF;
			tmpArr[1]=0xFF;
			counter=1;
		}
		tmpArr[counter]=getTmp;
		getTmpBeff=getTmp;
		counter++;
		if(counter>9){
			counter=0;
			 uint16_t checksumTmp = 0;
			for (uint8_t c = 2; c < 8; c++)checksumTmp += tmpArr[c];
			uint8_t tmp = checksumTmp % 256;
			uint8_t tmpComp = ~tmp;
			if (tmp == tmpArr[8] && tmpComp == tmpArr[9]) {
			driver1.angle = ((uint16_t) tmpArr[2] << 8) | tmpArr[3];
			driver1.pid_kp = tmpArr[4];
			driver1.pid_ki = tmpArr[5];
			driver1.pid_kd = tmpArr[6];
			driver1.factor = tmpArr[7];
		}
		}
	}
	__disable_irq();
	MDI_channel1.Instance->RDR=0;
	MDI_channel1.RxState=HAL_UART_STATE_READY;
	 __HAL_UART_CLEAR_FLAG(&MDI_channel1, UART_CLEAR_RTOF);
	 __enable_irq();

}
/**
 * @brief get to Motor Driver 1 values
 * @return none
 */
void MDI_getDataChannel1Ver4(void) {
	HAL_UART_Receive(&MDI_channel1, (uint8_t*) rec1Buff, 20, TIMEOUTVAL * 150);
	for (uint8_t counter = 0; counter < sizeof(rec1Buff); counter++) {
		if (rec1Buff[counter] == 0xFF && rec1Buff[counter + 1] == 0xFF) {
			uint16_t checksumTmp = 0;
			uint8_t tmpArr[10];
			for (uint8_t counter2 = 0; counter2 < sizeof(tmpArr); counter2++) {
				tmpArr[counter2] = rec1Buff[counter + counter2];
			}
			for (uint8_t c = 2; c < 8; c++)
				checksumTmp += tmpArr[c];
			uint8_t tmp = checksumTmp % 256;
			uint8_t tmpComp = ~tmp;
			if (tmp == tmpArr[8] && tmpComp == tmpArr[9]) {
				driver1.angle = ((uint16_t) tmpArr[2] << 8) | tmpArr[3];
				driver1.pid_kp = tmpArr[4];
				driver1.pid_ki = tmpArr[5];
				driver1.pid_kd = tmpArr[6];
				driver1.factor = tmpArr[7];
			}
			return;
		}
	}
}
/**
 * @brief drive to Motor Driver 2
 * @param angleVal -> get motor angle value
 * @param kp -> get Pid kp value
 * @param ki -> get Pid ki value
 * @param kd -> get Pid kd value
 * @param factor -> get factor of Pid elements
 * @return none
 */
void MDI_sendDataChannel2(uint16_t angleVal, uint8_t kp, uint8_t ki, uint8_t kd,
		uint8_t factor) {
	uint16_t checksumTmp = 0;
	MDI_writeCommand(&MDI_channel2, 0xFF);
	MDI_writeCommand(&MDI_channel2, 0xFF); //Data transmission started
	MDI_2byteWriteData(&MDI_channel2, angleVal);
	uint8_t tmpArr[] = { angleVal >> 8, angleVal & 0xFF };
	checksumTmp += tmpArr[0];
	checksumTmp += tmpArr[1]; //2 byte angle val sended
	MDI_writeSmallData(&MDI_channel2, kp);
	checksumTmp += kp; //writed kp
	MDI_writeSmallData(&MDI_channel2, ki);
	checksumTmp += ki; //writed ki
	MDI_writeSmallData(&MDI_channel2, kd);
	checksumTmp += kd; //writed kd
	MDI_writeSmallData(&MDI_channel2, factor);
	checksumTmp += factor; //writed factor
	uint8_t tmp = checksumTmp % 256;
	MDI_writeSmallData(&MDI_channel2, tmp); //checksum first byte
	uint8_t tmpComp = ~tmp;
	MDI_writeSmallData(&MDI_channel2, tmpComp); //checksum second byte
}
/**
 * @brief drive to Motor Driver 2 version 2 (will use for pic drive)
 * @param angleVal -> get motor angle value
 * @param kp -> get Pid kp value
 * @param ki -> get Pid ki value
 * @param kd -> get Pid kd value
 * @param factor -> get factor of Pid elements
 * @return none
 */
void MDI_sendDataChannel2Ver2(uint16_t angleVal, uint8_t kp, uint8_t ki,
		uint8_t kd, uint8_t factor) {
	uint16_t checksumTmp = 0;
	uint8_t tmpArr[] = { angleVal >> 8, angleVal & 0xFF };
	checksumTmp += tmpArr[0];
	checksumTmp += tmpArr[1];
	checksumTmp += kp;
	checksumTmp += ki;
	checksumTmp += kd;
	checksumTmp += factor;
	uint8_t tmp = checksumTmp % 256;
	uint8_t tmpComp = ~tmp;
	uint8_t sendBuff[10] = { 0XFF, 0XFF, tmpArr[0], tmpArr[1], kp, ki, kd,
			factor, tmp, tmpComp };
	for (uint8_t counter = 0; counter < 10; counter++) {
		MDI_writeSmallData(&MDI_channel2, sendBuff[counter]);
		vTaskDelay(50);
	}
}
/**
 * @brief get to Motor Driver 2 values
 * @return none
 */
uint8_t getFirstData2;
void MDI_getDataChannel2(void) {
	HAL_UART_Receive(&MDI_channel2, (uint8_t*) &getFirstData2, 1, TIMEOUTVAL);
	if (0xFF == getFirstData2) {
		HAL_UART_Receive(&MDI_channel2, (uint8_t*) rec2Buff, 9, TIMEOUTVAL * 9);
		if (0xFF == rec2Buff[0]) {
			uint16_t checksumTmp = 0;
			for (uint8_t c = 1; c < 7; c++)
				checksumTmp += rec2Buff[c];
			uint8_t tmp = checksumTmp % 256;
			uint8_t tmpComp = ~tmp;
			if (tmp == rec2Buff[7] && tmpComp == rec2Buff[8]) {
				driver2.angle = ((uint16_t) rec2Buff[1] << 8) | rec2Buff[2];
				driver2.pid_kp = rec2Buff[3];
				driver2.pid_ki = rec2Buff[4];
				driver2.pid_kd = rec2Buff[5];
				driver2.factor = rec2Buff[6];
			}
		}
		for (uint8_t c = 0; c < 9; c++)
			rec2Buff[c] = 0;
	}
	getFirstData2 = 0;
}
/**
 * @brief get to Motor Driver 2 values
 * @return none
 */
void MDI_getDataChannel2Ver2(void) {

	HAL_UART_Receive(&MDI_channel2, (uint8_t*) rec2Buff, 10, TIMEOUTVAL * 10);
	if (0xFF == rec2Buff[0] && 0xFF == rec2Buff[1]) {
		uint16_t checksumTmp = 0;
		for (uint8_t c = 2; c < 8; c++)
			checksumTmp += rec2Buff[c];
		uint8_t tmp = checksumTmp % 256;
		uint8_t tmpComp = ~tmp;
		if (tmp == rec2Buff[8] && tmpComp == rec2Buff[9]) {
			driver2.angle = ((uint16_t) rec2Buff[2] << 8) | rec2Buff[3];
			driver2.pid_kp = rec2Buff[4];
			driver2.pid_ki = rec2Buff[5];
			driver2.pid_kd = rec2Buff[6];
			driver2.factor = rec2Buff[7];
		}
	}
}
/**
 * @brief get to Motor Driver 2 values
 * @return none
 */
void MDI_getDataChannel2Ver3(void) {
	static uint8_t tmpArr[10];
	static uint8_t counter=0,getTmp=0,getTmpBeff=0;
	if((MDI_channel2.Instance->ISR & USART_ISR_RXNE) != 0){
		getTmp = MDI_channel2.Instance->RDR;
		if(0xFF ==getTmp && 0xFF ==getTmpBeff){
			tmpArr[0]=0xFF;
			tmpArr[1]=0xFF;
			counter=1;
		}
		tmpArr[counter]=getTmp;
		getTmpBeff=getTmp;
		counter++;
		if(counter>9){
			counter=0;
			 uint16_t checksumTmp = 0;
			for (uint8_t c = 2; c < 8; c++)checksumTmp += tmpArr[c];
			uint8_t tmp = checksumTmp % 256;
			uint8_t tmpComp = ~tmp;
			if (tmp == tmpArr[8] && tmpComp == tmpArr[9]) {
			driver2.angle = ((uint16_t) tmpArr[2] << 8) | tmpArr[3];
			driver2.pid_kp = tmpArr[4];
			driver2.pid_ki = tmpArr[5];
			driver2.pid_kd = tmpArr[6];
			driver2.factor = tmpArr[7];
		}
		}


	}
	__disable_irq();
	MDI_channel2.Instance->RDR=0;
	MDI_channel2.RxState=HAL_UART_STATE_READY;
	 __HAL_UART_CLEAR_FLAG(&MDI_channel2, UART_CLEAR_RTOF);
	 __enable_irq();
}
/**
 * @brief get to Motor Driver 2 values
 * @return none
 */
void MDI_getDataChannel2Ver4(void){
		HAL_UART_Receive(&MDI_channel2, (uint8_t*) &rec2Buff, 20, TIMEOUTVAL*150);
		for (uint8_t counter = 0; counter < sizeof(rec2Buff); counter++) {
		if (rec2Buff[counter] == 0xFF && rec2Buff[counter + 1] == 0xFF) {
			uint16_t checksumTmp = 0;
			uint8_t tmpArr[10];
			for (uint8_t counter2 = 0; counter2 < sizeof(tmpArr); counter2++) {
				tmpArr[counter2] = rec2Buff[counter + counter2];
			}
			for (uint8_t c = 2; c < 8; c++)
				checksumTmp += tmpArr[c];
			uint8_t tmp = checksumTmp % 256;
			uint8_t tmpComp = ~tmp;
			if (tmp == tmpArr[8] && tmpComp == tmpArr[9]) {
				driver2.angle = ((uint16_t) tmpArr[2] << 8) | tmpArr[3];
				driver2.pid_kp = tmpArr[4];
				driver2.pid_ki = tmpArr[5];
				driver2.pid_kd = tmpArr[6];
				driver2.factor = tmpArr[7];
			}

		}
	}

}
#endif

#if DRIVERCHANNEL == 1
uint8_t getDriver1kd(void){ return driver1.angle;}
uint8_t getDriver1ki(void){ return driver1.pid_kd;}
uint8_t getDriver1kp(void){ return driver1.pid_ki;}
uint8_t getDriver1factor(void){ return driver1.pid_kp;}
uint16_t getDriver1angle(void){return  driver1.angle;}
mD_interface getDriver1ReceiveVal(void){return driver1;}
#elif DRIVERCHANNEL == 2
uint8_t getDriver1kd(void) {
	return driver1.angle;
}
uint8_t getDriver1ki(void) {
	return driver1.pid_kd;
}
uint8_t getDriver1kp(void) {
	return driver1.pid_ki;
}
uint8_t getDriver1factor(void) {
	return driver1.pid_kp;
}
uint16_t getDriver1angle(void) {
	return driver1.angle;
}

uint8_t getDriver2kd(void) {
	return driver2.angle;
}
uint8_t getDriver2ki(void) {
	return driver2.pid_kd;
}
uint8_t getDriver2kp(void) {
	return driver2.pid_ki;
}
uint8_t getDriver2factor(void) {
	return driver2.pid_kp;
}
uint16_t getDriver2angle(void) {
	return driver2.factor;
}

mD_interface getDriver1ReceiveVal(void) {
	return driver1;
}
mD_interface getDriver2ReceiveVal(void) {
	return driver2;
}
#endif

