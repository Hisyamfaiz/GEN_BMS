/*
 * CANbus.h
 *
 *  Created on: Nov 8, 2019
 *      Author: Moh Hisyam Faiz
 */

#ifndef INC_CANBUS_H_
#define INC_CANBUS_H_

#endif /* INC_CANBUS_H_ */
#include "stm32f1xx_hal.h"
#include "main.h"
#include "fonts.h"
#include "ssd1306.h"
#include "stdio.h"
#include "stdlib.h"
#include "can.h"

void BMS_CAN_Tx(void);
void BMS_CAN_Rx(void);
void BMS_CAN_Config(void);

extern CAN_TxHeaderTypeDef	Tx_Header;
extern CAN_RxHeaderTypeDef 	Rx_Header;

extern uint32_t 	panjang,lebar,z;
extern uint8_t    	Tx_data[8];
extern uint32_t   	TxMailbox;
extern uint8_t 		Rx_data[8];
extern uint8_t		Data_id1[8],
					Data_id2[8];

union 	Float_byte {
		float    m_float;
		uint8_t  m_bytes[sizeof(float)];};

union	uint16_byte {
		uint16_t m_uint16_t;
		uint8_t  m_bytes[sizeof(float)];};

extern union Float_byte 	data;
extern union uint16_byte 	Batt_voltage,
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

extern float		V_batt,
					I_batt,
					SOC_batt,
					Temp_batt,
					Capacity_batt,
					SOH_batt,
					UV_protection,
					LC_protection,
					OC_protection,
					Balancing_protection;

extern float		OCC_protection,
					OCD_protection,
					OTC_protection,
					OTD_protection;

extern uint16_t		Cycle_batt;

extern uint8_t		charge_state,
					discharge_state,
					sleep_state;
