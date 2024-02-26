/*
 * CANFilter.h
 *
 *  Created on: Feb 21, 2024
 *      Author: kmehta
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "stm32f1xx_hal.h"
#include "main.h"
#include "temp.h"

extern CAN_HandleTypeDef hcan;

void TS_ECU_SYNC_RX1_FilterConfig(void);
void TS_ECU_SYNC_RX2_FilterConfig(void);
void TS_ECU_ChargingStateTrigFilterConfig(void);
void TS_ECU1_SendDiagnosticData(TS_ECU1_TX1_t* _tx1_t);
void TS_ECU1_SendTemperatures(int* _tempArray);

#endif /* INC_CAN_H_ */
