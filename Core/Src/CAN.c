/*
 * CANFilter.h
 *
 *  Created on: Feb 21, 2024
 *      Author: kmehta
 */

#include <CAN.h>

void TS_ECU_SYNC_RX1_FilterConfig(void)
{
	CAN_FilterTypeDef filter;

	filter.FilterActivation = CAN_FILTER_ENABLE;
	filter.FilterBank = 0;
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterIdHigh = (TS_ECU_SYNC_RX1_CANID << 5);
	filter.FilterIdLow = (0x000U);
	filter.FilterMaskIdHigh = (TS_ECU_SYNC_RX1_CANID << 5);
	filter.FilterMaskIdLow = (0x000U);
	filter.FilterMode = CAN_FILTERMODE_IDLIST;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.SlaveStartFilterBank = 15;

	if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
	{
		Error_Handler();
	}
}

void TS_ECU_SYNC_RX2_FilterConfig(void)
{
	CAN_FilterTypeDef filter;

	filter.FilterActivation = CAN_FILTER_ENABLE;
	filter.FilterBank = 1;
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterIdHigh = (TS_ECU_SYNC_RX2_CANID << 5);
	filter.FilterIdLow = (0x000U);
	filter.FilterMaskIdHigh = (TS_ECU_SYNC_RX2_CANID << 5);
	filter.FilterMaskIdLow = (0x000U);
	filter.FilterMode = CAN_FILTERMODE_IDLIST;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.SlaveStartFilterBank = 15;

	if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
	{
		Error_Handler();
	}
}

void TS_ECU_ChargingStateTrigFilterConfig(void)
{
	CAN_FilterTypeDef filter;

	filter.FilterActivation = CAN_FILTER_ENABLE;
	filter.FilterBank = 2;
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterIdHigh = (TS_ECU_ChargingStateTrigger_CANID << 5);
	filter.FilterIdLow = (0x000U);
	filter.FilterMaskIdHigh = (TS_ECU_ChargingStateTrigger_CANID << 5);
	filter.FilterMaskIdLow = (0x000U);
	filter.FilterMode = CAN_FILTERMODE_IDLIST;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.SlaveStartFilterBank = 15;

	if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
	{
		Error_Handler();
	}
}

void TS_ECU_SendDiagnosticData(TS_ECU1_TX1_t* _tx1_t)
{
	CAN_TxHeaderTypeDef txHeader;
	uint8_t data[5];
	uint32_t mailbox;
	uint8_t dlc, ide;

	Pack_TS_ECU1_TX1_Temp(_tx1_t, data, &dlc, &ide);

	txHeader.DLC = TS_ECU1_TX1_DLC;
	txHeader.ExtId = DISABLE;
	txHeader.IDE = TS_ECU1_TX1_IDE;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.StdId = TS_ECU1_TX1_CANID;
	txHeader.TransmitGlobalTime = DISABLE;

	if (HAL_CAN_AddTxMessage(&hcan, &txHeader, data, &mailbox) != HAL_OK)
	{
		Error_Handler();
	}

	while (HAL_CAN_IsTxMessagePending(&hcan, mailbox));

}

void TS_ECU_SendTemperatures(int* _tempArray)
{
	CAN_TxHeaderTypeDef txHeaderOne, txHeaderTwo, txHeaderThree;
	uint8_t dataTX2[8], dataTX3[8], dataTX4[8];
	uint32_t mailbox;

    for(int i = 0; i < 8; i++) {
        dataTX2[i] = ((_tempArray[i] & (0xFFU)) + 40);
        dataTX3[i] = ((_tempArray[i + 8] & (0xFFU)) + 40);
        dataTX4[i] = ((_tempArray[i + 16] & (0xFFU)) + 40);
    }

    txHeaderOne.DLC = TS_ECU1_TX2_DLC;
    txHeaderOne.ExtId = DISABLE;
    txHeaderOne.IDE = TS_ECU1_TX2_IDE;
    txHeaderOne.RTR = CAN_RTR_DATA;
    txHeaderOne.StdId = TS_ECU1_TX2_CANID;
    txHeaderOne.TransmitGlobalTime = DISABLE;

    txHeaderTwo.DLC = TS_ECU1_TX3_DLC;
    txHeaderTwo.ExtId = DISABLE;
    txHeaderTwo.IDE = TS_ECU1_TX3_IDE;
    txHeaderTwo.RTR = CAN_RTR_DATA;
    txHeaderTwo.StdId = TS_ECU1_TX3_CANID;
    txHeaderTwo.TransmitGlobalTime = DISABLE;

    txHeaderThree.DLC = TS_ECU1_TX4_DLC;
    txHeaderThree.ExtId = DISABLE;
    txHeaderThree.IDE = TS_ECU1_TX4_IDE;
    txHeaderThree.RTR = CAN_RTR_DATA;
    txHeaderThree.StdId = TS_ECU1_TX4_CANID;
    txHeaderThree.TransmitGlobalTime = DISABLE;

	if (HAL_CAN_AddTxMessage(&hcan, &txHeaderOne, dataTX2, &mailbox) != HAL_OK)
	{
		Error_Handler();
	}

	while (HAL_CAN_IsTxMessagePending(&hcan, mailbox));

	if (HAL_CAN_AddTxMessage(&hcan, &txHeaderTwo, dataTX3, &mailbox) != HAL_OK)
	{
		Error_Handler();
	}

	while (HAL_CAN_IsTxMessagePending(&hcan, mailbox));

	if (HAL_CAN_AddTxMessage(&hcan, &txHeaderThree, dataTX4, &mailbox) != HAL_OK)
	{
		Error_Handler();
	}

	while (HAL_CAN_IsTxMessagePending(&hcan, mailbox));
}
