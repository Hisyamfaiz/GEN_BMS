#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "Battery_Charge_Discharge.h"

extern float VBATT, IBATT;

extern float Pack_SOC, Delta_VCell,Bat_Pow_Out, Pack_Cap;
extern uint16_t LifeTime;
extern uint8_t BATT_State;

extern uint8_t				charge_state,
							discharge_state,
							sleep_state;

void Batt_Discharge_Mode(void)
{
	if(flag_trip_undervoltage==ON||
			flag_trip_overtemperature==ON||
			flag_trip_undertemperature==ON||
			flag_trip_overcurrentdischarge==ON||
			flag_trip_SOCOverDischarge==ON||
			flag_trip_shortcircuit==ON||
			flag_trip_unbalance==ON||
			flag_trip_systemfailure==ON)
	{
		Batt_Open_Mode();
	}
	else
	{
		HAL_GPIO_WritePin(BAT_CUT_P_GPIO_Port, BAT_CUT_P_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BAT_CUT_N_GPIO_Port, BAT_CUT_N_Pin, GPIO_PIN_SET);
		BATT_State=STATE_DISCHARGE;

		charge_state=0;
		discharge_state=1;
		sleep_state=0;
	}


}

void Batt_Charge_Mode(void)
{
	if(flag_trip_overvoltage==ON			||
			flag_trip_overtemperature==ON	||
			flag_trip_undertemperature==ON	||
			flag_trip_overcurrentcharge==ON	||
			flag_trip_SOCOverCharge==ON		||
			flag_trip_shortcircuit==ON		||
			flag_trip_systemfailure==ON		)
	{
		Batt_Open_Mode();
	}
	else
	{
		HAL_GPIO_WritePin(BAT_CUT_P_GPIO_Port, BAT_CUT_P_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BAT_CUT_N_GPIO_Port, BAT_CUT_N_Pin, GPIO_PIN_RESET);
		BATT_State=STATE_CHARGE;
		charge_state=1;
		discharge_state=0;
		sleep_state=0;
	}

}

void Batt_Full_CD_Mode(void)
{
	if(flag_trip_undervoltage==ON			||
			flag_trip_overvoltage==ON		||
			flag_trip_overtemperature==ON	||
			flag_trip_undertemperature==ON	||
			flag_trip_overcurrentdischarge==ON||
			flag_trip_overcurrentcharge==ON	||
			flag_trip_SOCOverDischarge==ON	||
			flag_trip_SOCOverCharge==ON		||
			flag_trip_shortcircuit==ON		||
			flag_trip_unbalance==ON			||
			flag_trip_systemfailure==ON		)
	{
		Batt_Open_Mode();
	}
	else
	{
		HAL_GPIO_WritePin(BAT_CUT_P_GPIO_Port, BAT_CUT_P_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BAT_CUT_N_GPIO_Port, BAT_CUT_N_Pin, GPIO_PIN_SET);
		BATT_State=STATE_FULL_CHARGE_DISCHARGE;
		charge_state=1;
		discharge_state=1;
		sleep_state=0;
	}
}

void Batt_Open_Mode(void)
{
	HAL_GPIO_WritePin(BAT_CUT_P_GPIO_Port, BAT_CUT_P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BAT_CUT_N_GPIO_Port, BAT_CUT_N_Pin, GPIO_PIN_RESET);
	BATT_State=STATE_STANDBY;
	charge_state=0;
	discharge_state=0;
	sleep_state=1;

	HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, 0);

	check_SOC_Based_OCV();
}

void check_SOC_Based_OCV(void)
{
	//Pack_SOC=0.226863411166458*VBATT*VBATT-18.618705166771*VBATT+378.560621625972;   //Persamaan Baterai INR 21700
	Pack_SOC=(147.471026094008*(VBATT/15.0) - 494.687746093127);  // Persamaan Baterai EVE ICR18650/26V

	if(Pack_SOC>130) Pack_SOC=100;
	else if(Pack_SOC>100) Pack_SOC=100;
	else if(Pack_SOC<0) Pack_SOC=0;
}
