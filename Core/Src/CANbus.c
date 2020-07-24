/*
 * CANbus.c
 *
 *  Created on: Nov 8, 2019
 *      Author: Moh Hisyam Faiz
 */
#include "stm32f1xx_hal.h"
#include "main.h"
#include "can.h"
#include "string.h"

uint32_t BMS_UID=1001;   //BMS_UID antara 0-1048575

extern CAN_HandleTypeDef	hcan;
CAN_TxHeaderTypeDef			Tx_Header;
CAN_RxHeaderTypeDef 		Rx_Header;

uint32_t 					panjang,lebar,z;
uint8_t               		Tx_data[8];
uint32_t              		TxMailbox;
uint8_t 					Rx_data[8];
uint8_t						Data_id1[8],
							Data_id2[8];
int oke=0, nyantol = 0;

uint8_t						charge_state=1,
							discharge_state=1,
							sleep_state=0;

extern uint8_t 		flag_trip_overtemperature,
					flag_trip_undertemperature,
					flag_trip_SOCOverDischarge,
					flag_trip_SOCOverCharge,			//di tiada kan..!
					flag_trip_undervoltage,
					flag_trip_overvoltage,
					flag_trip_overcurrentdischarge,
					flag_trip_overcurrentcharge,
					flag_trip_shortcircuit,
					flag_trip_systemfailure,
					flag_trip_unbalance;

extern uint8_t Clear_Trip_undervoltage,
		Clear_Trip_overcurrentdischarge;

extern float VBATT, IBATT;

extern uint8_t flag_start_shutdown, BMS_mode;

extern float Pack_SOC, SOC_manipulasi, Delta_VCell,Bat_Pow_Out;
extern float Pack_Cap;
extern uint16_t LifeTime;
extern uint8_t BATT_State;

// Variabel Suhu
extern float Suhu_T1,Suhu_T2,Suhu_T3,Suhu_T4;
float Tmax;

union Float_byte {
	float    m_float;
	uint8_t  m_bytes[sizeof(float)];};

union uint16_byte {
	uint16_t m_uint16_t;
	uint8_t  m_bytes[sizeof(float)];};

union Float_byte 	data;
union uint16_byte 	Batt_voltage,
					Batt_current,
					Batt_SOC,
					Batt_temp,
					Batt_capacity,
					Batt_SOH,
					Batt_cycle,
					over_charge_current,
					over_discharge_current,
					over_temp_charge,
					over_temp_discharge,
					under_voltage,
					limit_capacity,
					over_charge,
					start_balancing;

union uint16_byte vcell_15databyte[15];

extern float vcell_15data[15];
extern uint16_t vcell_15data_digi[15];
extern float v_cell_tot;

extern float Delta_VCell,Bat_Pow_Out;
extern float Pack_Cap;
extern uint16_t LifeTime;
extern uint8_t BATT_State;

float				SOH_batt=90;

float				OCC_protection,
					OCD_protection,
					OTC_protection,
					OTD_protection,
					UV_protection,
					LC_protection,
					OC_protection,
					Balancing_protection;

uint16_t			Cycle_batt=125;
uint8_t				Handshaking = 0,
					identified = 0,
					Delay_Charger = 0,
					Ready_toCharge = 0,
					flag_Check_SOCawal = 0;

void BMS_CAN_Tx()
{
		int i;
		Batt_voltage.m_uint16_t=VBATT*100;
		Batt_current.m_uint16_t=(IBATT+50)*100;
		Batt_SOC.m_uint16_t=SOC_manipulasi*100;
//		Batt_SOC.m_uint16_t=(int)SOC_manipulasi;	default Data SOC

		Tmax=Suhu_T1;
		if(Tmax < Suhu_T2) Tmax = Suhu_T2;
		if(Tmax < Suhu_T3) Tmax = Suhu_T3;
		if(Tmax < Suhu_T4) Tmax = Suhu_T4;

		Batt_temp.m_uint16_t=(Tmax+40)*10;
		Batt_capacity.m_uint16_t=Pack_Cap*100;
		Batt_SOH.m_uint16_t=(int)SOH_batt;
		Batt_cycle.m_uint16_t=LifeTime;

		for(int mn=0;mn<15;mn++)
		{
			vcell_15databyte[mn].m_uint16_t=vcell_15data_digi[mn];
		}

	if(Handshaking==1){
		// CAN ID transmit #1
		Tx_Header.ExtId = (0x0B0<<20|UNIQUE_Code);  //7b1
		//CAN Data #1
		Tx_data[0] = Batt_voltage.m_bytes[0];
		Tx_data[1] = Batt_voltage.m_bytes[1];
		Tx_data[2] = Batt_current.m_bytes[0];
		Tx_data[3] = Batt_current.m_bytes[1];
		Tx_data[4] = Batt_SOC.m_bytes[0];
		Tx_data[5] = Batt_SOC.m_bytes[1];
		Tx_data[6] = Batt_temp.m_bytes[0];
		Tx_data[7] = Batt_temp.m_bytes[1];

		//CAN Tx message #1
		Tx_Header.DLC = 8;
		uint32_t delay_TICK1 = HAL_GetTick();
		while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan)){
			if(HAL_GetTick() - delay_TICK1 > 3000)
				break;
		}

		if(HAL_CAN_AddTxMessage(&hcan, &Tx_Header, Tx_data, &TxMailbox)!= HAL_OK) {
			HAL_CAN_AbortTxRequest(&hcan, TxMailbox);
			Error_Handler();
		}
		i=1000;
		while(i>1) i--;


		// CAN ID transmit #2
		Tx_Header.ExtId = (0x0B1<<20|UNIQUE_Code); //7b2
		//CAN Data #2
		Tx_data[0] = Batt_capacity.m_bytes[0];
		Tx_data[1] = Batt_capacity.m_bytes[1];
		Tx_data[2] = Batt_SOH.m_bytes[0];
		Tx_data[3] = Batt_SOH.m_bytes[1];
		Tx_data[4] = Batt_cycle.m_bytes[0];
		Tx_data[5] = Batt_cycle.m_bytes[1];
		Tx_data[6] = flag_trip_shortcircuit&0x01;
		Tx_data[6] |= (flag_trip_overcurrentdischarge&0x01)<<1;
		Tx_data[6] |= (flag_trip_overcurrentcharge&0x01)<<2;
		Tx_data[6] |= (flag_trip_overtemperature&0x01)<<3;
		Tx_data[6] |= (flag_trip_undertemperature&0x01)<<4;
		Tx_data[6] |= (flag_trip_overtemperature&0x01)<<5;
		Tx_data[6] |= (flag_trip_undertemperature&0x01)<<6;
		Tx_data[6] |= (flag_trip_unbalance&0x01)<<7;

		Tx_data[7] =  (flag_trip_undervoltage&0x01);
		Tx_data[7] |= (flag_trip_overvoltage&0x01)<<1;
		Tx_data[7] |= (flag_trip_SOCOverDischarge&0x01)<<2;
		Tx_data[7] |= (flag_trip_systemfailure&0x01)<<3;
		Tx_data[7] |= (charge_state&0x01)<<4;
		Tx_data[7] |= (discharge_state&0x01)<<5;
		Tx_data[7] |= (sleep_state&0x01)<<6;

		//CAN Tx message #2
		Tx_Header.DLC = 8;
		uint32_t delay_TICK2 = HAL_GetTick();
		while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan)){
			if(HAL_GetTick() - delay_TICK2 > 3000)
				break;
		}

		if(HAL_CAN_AddTxMessage(&hcan, &Tx_Header, Tx_data, &TxMailbox)!= HAL_OK) {
			HAL_CAN_AbortTxRequest(&hcan, TxMailbox);
			Error_Handler();
		}
		i=1000;
		while(i>1) i--;

/*
		// *********************** Start Cell  Voltage Data Send ******************************
		// CAN ID transmit #4
		Tx_Header.ExtId=(0x0B4<<20|UNIQUE_Code); //b4
		//CAN Data #4
		Tx_data[0]=vcell_15databyte[0].m_bytes[0];
		Tx_data[1]=vcell_15databyte[0].m_bytes[1];
		Tx_data[2]=vcell_15databyte[1].m_bytes[0];
		Tx_data[3]=vcell_15databyte[1].m_bytes[1];
		Tx_data[4]=vcell_15databyte[2].m_bytes[0];
		Tx_data[5]=vcell_15databyte[2].m_bytes[1];
		Tx_data[6]=vcell_15databyte[3].m_bytes[0];
		Tx_data[7]=vcell_15databyte[3].m_bytes[1];

		//CAN Tx message #4
		Tx_Header.DLC = 8;
		while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan));
		if(HAL_CAN_AddTxMessage(&hcan, &Tx_Header, Tx_data, &TxMailbox)!= HAL_OK) Error_Handler();
		i=1000;
		while(i>1) i--;

		// CAN ID transmit #5
		Tx_Header.ExtId=(0x0B5<<20|UNIQUE_Code);  //b5
		//CAN Data #4
		Tx_data[0]=vcell_15databyte[4].m_bytes[0];
		Tx_data[1]=vcell_15databyte[4].m_bytes[1];
		Tx_data[2]=vcell_15databyte[5].m_bytes[0];
		Tx_data[3]=vcell_15databyte[5].m_bytes[1];
		Tx_data[4]=vcell_15databyte[6].m_bytes[0];
		Tx_data[5]=vcell_15databyte[6].m_bytes[1];
		Tx_data[6]=vcell_15databyte[7].m_bytes[0];
		Tx_data[7]=vcell_15databyte[7].m_bytes[1];
		//CAN Tx message #4
		Tx_Header.DLC = 8;
		while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan));
		if(HAL_CAN_AddTxMessage(&hcan, &Tx_Header, Tx_data, &TxMailbox)!= HAL_OK) Error_Handler();
		i=1000;
		while(i>1) i--;

		// CAN ID transmit #6
		Tx_Header.ExtId=(0x0B6<<20|UNIQUE_Code); //b6
		//CAN Data #4
		Tx_data[0]=vcell_15databyte[8].m_bytes[0];
		Tx_data[1]=vcell_15databyte[8].m_bytes[1];
		Tx_data[2]=vcell_15databyte[9].m_bytes[0];
		Tx_data[3]=vcell_15databyte[9].m_bytes[1];
		Tx_data[4]=vcell_15databyte[10].m_bytes[0];
		Tx_data[5]=vcell_15databyte[10].m_bytes[1];
		Tx_data[6]=vcell_15databyte[11].m_bytes[0];
		Tx_data[7]=vcell_15databyte[11].m_bytes[1];
		//CAN Tx message #4
		Tx_Header.DLC = 8;
		while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan));
		if(HAL_CAN_AddTxMessage(&hcan, &Tx_Header, Tx_data, &TxMailbox)!= HAL_OK) Error_Handler();
		i=1000;
		while(i>1) i--;

		// CAN ID transmit #4
		Tx_Header.ExtId=(0x0B7<<20|UNIQUE_Code); //b7
		//CAN Data #4
		Tx_data[0]=vcell_15databyte[12].m_bytes[0];
		Tx_data[1]=vcell_15databyte[12].m_bytes[1];
		Tx_data[2]=vcell_15databyte[13].m_bytes[0];
		Tx_data[3]=vcell_15databyte[13].m_bytes[1];
		Tx_data[4]=vcell_15databyte[14].m_bytes[0];
		Tx_data[5]=vcell_15databyte[14].m_bytes[1];
		Tx_data[6]=vcell_15databyte[14].m_bytes[0];
		Tx_data[7]=vcell_15databyte[14].m_bytes[1];
		//CAN Tx message #4
		Tx_Header.DLC = 8;
		while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan));
		if(HAL_CAN_AddTxMessage(&hcan, &Tx_Header, Tx_data, &TxMailbox)!= HAL_OK) Error_Handler();
		i=1000;
		while(i>1) i--;
		// ******************************End Cell  Voltage Data Send**************************************
//	*/
	}
	else {
		// Handshaking with Charger
		Delay_Charger+=1;
		if(Handshaking == 0 && identified == 0 && Delay_Charger >= 50){
			Tx_Header.ExtId = (0x0E0<<20|UNIQUE_Code); //b7
			Tx_data[0] = 0;
			Tx_data[1] = 0;
			Tx_data[2] = 0;
			Tx_data[3] = 0;
			Tx_data[4] = 0;
			Tx_data[5] = 0;
			Tx_data[6] = 0x55;
			Tx_data[7] = 0;

			Tx_Header.DLC = 8;
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan));
			if(HAL_CAN_AddTxMessage(&hcan, &Tx_Header, Tx_data, &TxMailbox)!= HAL_OK) Error_Handler();
			i=1000;
			while(i>1) i--;
			Delay_Charger=0;
		}
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rx_Header, Rx_data)== HAL_OK){
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		int i;

		if(Rx_Header.StdId==0x0E2){	//Handshaking
			if(Rx_data[6]==0x55 && identified==0){
				identified = 1;
				Tx_Header.DLC = 8;
				Tx_data[6] = 0xAA;

				while(!HAL_CAN_GetTxMailboxesFreeLevel(hcan));
				if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, Tx_data, &TxMailbox)!= HAL_OK) Error_Handler();
				i=1000;
				while(i>1) i--;

				if(Pack_SOC < 90 && flag_Check_SOCawal != 1) {
					Ready_toCharge = 1;
					flag_Check_SOCawal = 1;
					oke=9;
				}
				else if(Pack_SOC >= 90 && flag_Check_SOCawal != 1) {
					Ready_toCharge = 0;
					flag_Check_SOCawal = 1;
					oke=8;
				}

			}

			if(Rx_data[6]==0xAA && identified==1){

				if(Ready_toCharge == 0 && flag_Check_SOCawal != 0){
					if(Pack_SOC < 70){
						Ready_toCharge = 1;
						oke=10;
					}
					oke=12;
				}
				Handshaking = 1;
				if(Ready_toCharge == 1) BMS_mode = 2;
			}
		}

		if(Rx_Header.StdId==0x1B2 && Handshaking == 0){	//activate BMS

			if((Rx_data[0]&0x01) == 1)  //without handshake
			{
				oke=3;
				flag_start_shutdown=1;
				BMS_mode=(Rx_data[0]>>1)&0x03;
				Handshaking=1; identified=1;
			}

			if((Rx_data[7]&0x01) == 1 && Rx_data[1]==0) //with hs
			{
				flag_start_shutdown=1;
				BMS_mode=0;
				oke=1;
			}
		}

		if(Rx_Header.ExtId == ((0x1B2<<20)|UNIQUE_Code) && Handshaking == 1 ){
			if(Rx_data[0]==0 && Rx_data[7]==0) {
				oke=2;
				Rx_Header.ExtId = 0;
				Handshaking=0;
				identified=0;
				flag_start_shutdown=0;
				BMS_mode=0;
				BATT_State=STATE_STANDBY;
			}

			Rx_Header.StdId=0;
			Clear_Trip_overcurrentdischarge=(Rx_data[0]>>3)&&0x01;
			Clear_Trip_undervoltage=(Rx_data[0]>>4)&&0x01;
		}
	}
	Rx_Header.ExtId = 0;
	Rx_Header.StdId = 0;
	memset(Rx_data, 0, 8*sizeof(Rx_data[0]));
}


void BMS_CAN_Config()
{
	/* Configure the CAN Filter */
	CAN_FilterTypeDef  sFilterConfig;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) Error_Handler();

	/* Start the CAN peripheral */
	if (HAL_CAN_Start(&hcan) != HAL_OK) Error_Handler();

	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

	/* Configure Transmission process */
	Tx_Header.TransmitGlobalTime = DISABLE;
	Tx_Header.RTR = CAN_RTR_DATA;
	Tx_Header.IDE = CAN_ID_EXT;
}
