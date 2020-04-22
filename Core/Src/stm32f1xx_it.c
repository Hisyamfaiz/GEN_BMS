/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gpio.h"
#include "Battery_Charge_Discharge.h"
#include "arm_math.h"
#include "math.h"
#include "CANbus.h"

#define maxdata 500

#define NTC_RES(adc_val)	((adc_val * 10000.0) / (4095.0 - adc_val))
#define NTC_TEMP(adc_ind)	(1.0 / ((logf(NTC_RES(adc_ind) / 10000.0) / 3434.0) + (1.0 / 298.15)) - 273.15)

#define max_hitung_suhu 10

uint8_t fault_code,last_fault_code;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern int test_tim2;
extern uint16_t adc_val[8];

int sumV,sumI,sumVn,sumRef,i;
float v_datadigi,i_datadigi,vn_datadigi,ref_datadigi;
uint16_t i_arrdata[maxdata],ref_arrdata[maxdata];
float VBATT, IBATT;

float sum_current;
float AH_Consumption;
uint16_t time_soc;

extern float Pack_SOC, Delta_VCell,Bat_Pow_Out;
extern float Pack_Cap;
extern uint16_t LifeTime;
extern uint8_t BATT_State;

// Variabel Suhu
float Res_T1,Res_T2,Res_T3,Res_T4;
float Suhu_T1,Suhu_T2,Suhu_T3,Suhu_T4;

int hitung_suhu;

//Variable Pengaman
float
		TMS=0.5;

float T_Under_trip,
	  T_trip_cycle;

uint8_t Clear_Trip_undervoltage,
		Clear_Trip_overcurrentdischarge;

float TMS_I_Over=0.5;

float T_I_Over_trip,
	  T_I_Over_trip_cycle;


float I_Over_Set=10.9,
	  I_Over_Set_Charge=7,
	  Temp_Over_Set=55,
	  Temp_Under_Set=15,
	  SOC_Under_Set=10,
	  SOC_Over_Set=120,
	  V_Under_Set=39.5,
	  V_Over_Set=65,
	  Persen_Imbalance_Set=30;

uint8_t 		flag_trip_overtemperature,
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

extern uint8_t flag_start_shutdown, BMS_mode;

extern uint8_t BATT_Start_Up;

extern float v_cell_tot;

extern float persen_imbalance;

int kl;

extern uint8_t				charge_state,
							discharge_state,
							sleep_state;

float OFFSET_SENSOR_ARUS,IBATT_for_offset_cal;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void Batt_Protection_when_discharge(void);
void Batt_Protection_when_charge(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles CAN RX1 interrupt.
  */
void CAN1_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX1_IRQn 0 */

  /* USER CODE END CAN1_RX1_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN CAN1_RX1_IRQn 1 */

  /* USER CODE END CAN1_RX1_IRQn 1 */
}

/**
  * @brief This function handles CAN SCE interrupt.
  */
void CAN1_SCE_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_SCE_IRQn 0 */

  /* USER CODE END CAN1_SCE_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN CAN1_SCE_IRQn 1 */

  /* USER CODE END CAN1_SCE_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  test_tim2++;
  hitung_suhu++;

  sumI=sumI-i_arrdata[i];
  sumRef=sumRef-ref_arrdata[i];

  i_arrdata[i]=adc_val[0];
  ref_arrdata[i]=adc_val[1];

  sumI=sumI+i_arrdata[i];
  sumRef=sumRef+ref_arrdata[i];

  i_datadigi=sumI/maxdata;
  ref_datadigi=sumRef/maxdata;


  // *************PROSES Konversi dari DATA ADC ke Data Real *******************************/////

  VBATT=v_cell_tot;
  if(VBATT<0) VBATT=-1;

//	IBATT = -0.06189346733668010*i_datadigi + 121.153903517579 - OFFSET_SENSOR_ARUS; //modul A
//  IBATT_for_offset_cal = -0.06189346733668010*i_datadigi + 121.153903517579;
//  IBATT=0.95556329728489100*IBATT + 0.06243330788446070;// Modul A Recalibrate


  IBATT=-0.0399633588118257*i_datadigi + 77.3576930186035- OFFSET_SENSOR_ARUS; // Modul B
  IBATT_for_offset_cal= -0.0399633588118257*i_datadigi + 77.3576930186035;

  if(hitung_suhu>=max_hitung_suhu)
  {
	  hitung_suhu=0;
	  Res_T1=adc_val[2]*10000/(3900-adc_val[2]); 	// 10000 => R1 , 3900 => Vcc dalam nilai digital
	  Suhu_T1= -24.05*log(Res_T1) + 246.41;			//1 / a + b (Ln RT / R25) + c b (Ln RT / R25)2
	  Res_T2=adc_val[3]*10000/(3900-adc_val[3]);
	  Suhu_T2= -24.05*log(Res_T2) + 246.41;			//1 / a + b (Ln RT / R25) + c b (Ln RT / R25)2
	  Res_T3=adc_val[4]*10000/(3900-adc_val[4]);
	  Suhu_T3= -24.05*log(Res_T3) + 246.41;			//1 / a + b (Ln RT / R25) + c b (Ln RT / R25)2
	  Res_T4=adc_val[5]*10000/(3900-adc_val[5]);
	  Suhu_T4= -24.05*log(Res_T4) + 246.41;			//1 / a + b (Ln RT / R25) + c b (Ln RT / R25)2
  }

  if(BATT_Start_Up==1)
  {
	  if(BATT_State==STATE_DISCHARGE)
	  {
		 Batt_Protection_when_discharge();
	  }

	  if(BATT_State==STATE_CHARGE)
	  {
		  Batt_Protection_when_charge();
	  }


	  //********************* Clearing protection status *****************************////
	  // ---> Clearing UnderVoltage
	  if(((Clear_Trip_undervoltage==1)||(VBATT>54))&&flag_trip_undervoltage==ON)
	  {
		  flag_trip_undervoltage=OFF;
		  Clear_Trip_undervoltage=0;
	  }
	  // ---> Clearing OverCurrent Discharge
	  if(flag_trip_overcurrentdischarge==ON && Clear_Trip_overcurrentdischarge==1)
	  {
		  flag_trip_overcurrentdischarge=OFF;
		  Clear_Trip_overcurrentdischarge=0;
	  }
	  // ---> Clearing OverTemperature
	  if(flag_trip_overtemperature==ON && ((Suhu_T1<40)||(Suhu_T2<40)||(Suhu_T3<40)||(Suhu_T4<40)))
	  {
		  flag_trip_overtemperature=OFF;
	  }
	  // ---> Clearing UnderTemperature
	  if(flag_trip_undertemperature==ON && ((Suhu_T1>20)||(Suhu_T2>20)||(Suhu_T3>20)||(Suhu_T4>20)))
	  {
		  flag_trip_undertemperature=OFF;
	  }
	  // ---> Clearing OverDischarge
	   if(flag_trip_SOCOverDischarge==ON && Pack_SOC>20)
	   {
		  flag_trip_SOCOverDischarge=OFF;
	   }
	   // ---> Clearing OverCharge
	   if(flag_trip_SOCOverCharge==ON && Pack_SOC<70)
	   {
		  flag_trip_SOCOverCharge=OFF;
	   }
  }
	  i++;
	  i=i%maxdata;

  //////////// Bagian Hitung SOC /////// SOC akan dihitung berdasarkan state baterai (Jika charge maupun discharge)
  if(BATT_State==STATE_CHARGE||BATT_State==STATE_DISCHARGE||BATT_State==STATE_FULL_CHARGE_DISCHARGE||BATT_State==STATE_STANDBY)
  {
	  time_soc++;
	  sum_current+=IBATT;
	  if(time_soc>999)
	  {
		  AH_Consumption = (-1*sum_current/1000*(1.0/3600.0))/Pack_Cap*100-(4e-5); //Konsumsi System 4e-5
		  Pack_SOC=Pack_SOC+AH_Consumption;
		  time_soc=0;
		  sum_current=0;
	  }
  }
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  if(flag_start_shutdown==1)
  {
	  BMS_CAN_Tx();
  }
  BMS_CAN_Rx();
  /* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void Batt_Protection_when_discharge(void)
{
	///////////////////// Short Circuit //////////////////////////////////////
			  	  if(IBATT>(VBATT/0.9))
	 		  	  {
	 		  		  fault_code=12;
	 		  		  Batt_Open_Mode();
	 		  		  flag_trip_shortcircuit=ON;
	 		  		  HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
	 		  	  }
			  //*****************Pengecekan Undervoltage dan Proteksi***********************************//////
			  	  else if(VBATT<V_Under_Set && flag_trip_undervoltage==OFF )   //Indikasi terjadi Undervoltage
			  	  {
			  		fault_code=1;
			  		T_Under_trip=TMS/(1-(VBATT/V_Under_Set));
			  		T_trip_cycle+=0.001;

			  		if(T_trip_cycle>T_Under_trip && flag_trip_undervoltage==OFF)
			  		{
			  			Batt_Open_Mode();
			  			T_trip_cycle=T_Under_trip;
			  			flag_trip_undervoltage=ON;
			  			HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
			  		}


			  		if(flag_trip_undervoltage==OFF)
			  		{
			  			if(T_Under_trip-T_trip_cycle>15)
			  			{
			  				if((test_tim2%1000)==0)
			  				{
			  					BUZZ_Toggle;
			  					test_tim2=0;
			  				}
			  			}
			  			else if(T_Under_trip-T_trip_cycle>10)
			  			{
			  				if((test_tim2%100)==0)
			  				{
			  					BUZZ_Toggle;
			  					test_tim2=0;
			  				}
			  			}
			  			else if(T_Under_trip-T_trip_cycle>1)
			  			{
			  				HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
			  			}
			  		}

			  	  }
			  	//**************Pengecekan OverCurrent Discharge **********************//
			  	  else if((IBATT-I_Over_Set)>0 && flag_trip_overcurrentdischarge==OFF)   //Indikasi terjadi Over Current
			  	  {
			  		fault_code=2;
			  		T_I_Over_trip=TMS_I_Over/((IBATT/10.9)-1);
			  		T_I_Over_trip_cycle+=0.001;

			  		if(T_I_Over_trip_cycle>T_I_Over_trip && flag_trip_overcurrentdischarge==OFF)
			  		{
			  			Batt_Open_Mode();
			  			T_I_Over_trip_cycle=T_I_Over_trip;
			  			flag_trip_overcurrentdischarge=ON;
			  			HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
			  		}
			  		if(flag_trip_overcurrentdischarge==OFF)
		  			{
		  				if(T_I_Over_trip-T_I_Over_trip_cycle>15)
		  				{
		  					if((test_tim2%1000)==0)
			  				{
			  					BUZZ_Toggle;
			  					test_tim2=0;
			  				}
			  			}
			  			else if(T_I_Over_trip-T_I_Over_trip_cycle>10)
			  			{
			  				if((test_tim2%100)==0)
			  				{
			  					BUZZ_Toggle;
			  					test_tim2=0;
			  				}
			  			}
			  			else if(T_I_Over_trip-T_I_Over_trip_cycle>1)
			  			{
			  				HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
			  			}
			  		}

			  	  }

			  	  //**************Pengecekan OverTemperature ****************************//
			  	  else if(((50-Suhu_T1<10)||(85-Suhu_T2<10)||(50-Suhu_T3<10)||(85-Suhu_T4<10)) && flag_trip_overtemperature==OFF)
			  	  {
			  		  fault_code=3;
			  		  if(Suhu_T1>Temp_Over_Set-10 && Suhu_T1<=Temp_Over_Set-5)
			  		  {
			  			  if((test_tim2%1000)==0)
			  			  {
			  				  BUZZ_Toggle;
			  				  test_tim2=0;
			  			  }
			  		  }
			  		  else if(Suhu_T1>Temp_Over_Set-5 && Suhu_T1<=Temp_Over_Set-2)
			  		  {
			  			  if((test_tim2%500)==0)
			  			  {
			  				  BUZZ_Toggle;
			  				  test_tim2=0;
			  			  }
			  		  }
			  		  else if(Suhu_T1>Temp_Over_Set-2 && Suhu_T1<=Temp_Over_Set)
			  		  {
			  			  if((test_tim2%500)==0)
			  			  {
			  				  BUZZ_Toggle;
			  				  test_tim2=0;
			  			  }
			  		  }
			  		  else if(Suhu_T1>50||Suhu_T2>85||Suhu_T3>50||Suhu_T4>85)
			  		  {
			  			  Batt_Open_Mode();
			  			  flag_trip_overtemperature=ON;
			  			  HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
			  		  }
			  	  }

			  	  //**************Pengecekan UnderTemperature ****************************//
			  	  else if((Suhu_T1-Temp_Under_Set<=10||Suhu_T2-Temp_Under_Set<=10||Suhu_T3-Temp_Under_Set<=10||Suhu_T4-Temp_Under_Set<=10) && flag_trip_undertemperature==OFF)
			  	  {
			  		  fault_code=4;
			  		  if(Suhu_T1<=Temp_Under_Set+10 && Suhu_T1>Temp_Under_Set+5)
			  		  {
			  			  if((test_tim2%1000)==0)
			  			  {
			  				  BUZZ_Toggle;
			  				  test_tim2=0;
			  			  }
			  		  }
			  		  else if(Suhu_T1<=Temp_Under_Set+5 && Suhu_T1>Temp_Under_Set+2)
			  		  {
			  			  if((test_tim2%500)==0)
			  			  {
			  				  BUZZ_Toggle;
			  				  test_tim2=0;
			  			  }
			  		  }
			  		  else if(Suhu_T1<Temp_Under_Set+2 && Suhu_T1>=Temp_Under_Set)
			  		  {
			  			  if((test_tim2%500)==0)
			  			  {
			  				  BUZZ_Toggle;
			  				  test_tim2=0;
			  			  }
			  		  }
			  		  else if(Suhu_T1<Temp_Under_Set||Suhu_T2<Temp_Under_Set||Suhu_T3<Temp_Under_Set||Suhu_T4<Temp_Under_Set)
			  		  {
			  			  Batt_Open_Mode();
			  			  flag_trip_undertemperature=ON;
			  			  HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
			  		  }
			  	  }

			  	 //**************Pengecekan SOC_OverDischarge****************************//
			  	  else if(Pack_SOC-SOC_Under_Set<=10 && flag_trip_SOCOverDischarge==OFF && BATT_State==STATE_DISCHARGE)
			  	  {
			  		  fault_code=5;
			  		  if(Pack_SOC<=SOC_Under_Set+10 && Pack_SOC>SOC_Under_Set+5)
			  		  {
			  			  if((test_tim2%1000)==0)
			  			  {
			  				  BUZZ_Toggle;
			  				  test_tim2=0;
			  			  }
			  		  }
			  		  else if(Pack_SOC<=SOC_Under_Set+5 && Pack_SOC>SOC_Under_Set+2)
			  		  {
			  			  if((test_tim2%500)==0)
			  			  {
			  				  BUZZ_Toggle;
			  				  test_tim2=0;
			  			  }
			  		  }
			  		  else if(Pack_SOC<SOC_Under_Set+2 && Pack_SOC>=SOC_Under_Set)
			  		  {
			  			  if((test_tim2%500)==0)
			  			  {
			  				  BUZZ_Toggle;
			  				  test_tim2=0;
			  			  }
			  		  }
			  		  else if(Pack_SOC<SOC_Under_Set)
			  		  {
			  			  Batt_Open_Mode();
			  			  flag_trip_SOCOverDischarge=ON;
			  			  HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
			  		  }
				  }

			  	///////// Imbalance Checking Status
			  	  else if(Persen_Imbalance_Set-persen_imbalance<10)
			  	  {
			  		  fault_code=9;
			  		  if(persen_imbalance>Persen_Imbalance_Set)
			  		  {
			  			flag_trip_unbalance=ON;
			  			Batt_Open_Mode();
			  		  }
			  	  }


			  	//Clearing when data status is normal before system trip
			  	  else
			  	  {
			  		  if(fault_code!=0) last_fault_code=fault_code;
			  		  fault_code=0;
			  		  HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
			  		  T_Under_trip=0;
			  		  T_trip_cycle=T_trip_cycle-0.001;
			  		  T_I_Over_trip_cycle-=0.001;
			  		  if(T_trip_cycle<0) T_trip_cycle=0;
			  		if(T_I_Over_trip_cycle<0) T_I_Over_trip_cycle=0;
			  	  }
}



void Batt_Protection_when_charge(void)
{
					///////////////////// Short Circuit //////////////////////////////////////
					if(fabs(IBATT)>VBATT)
				  	{
				  		fault_code=12;
				  		Batt_Open_Mode();
				  		flag_trip_shortcircuit=ON;
				  		HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
				  	}

					/////////**************Pengecekan OverCharge****************************//
					else if(SOC_Over_Set-Pack_SOC<=10 && flag_trip_SOCOverCharge==OFF)
					{
			  		  fault_code=6;
			  		  if(Pack_SOC>SOC_Over_Set)
			  		  {
			  				  Batt_Open_Mode();
			  				  flag_trip_SOCOverCharge=ON;
			  				  HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
			  		  }
			  	  	}

			  		//**************Pengecekan OverTemperature ****************************//
			  		else if(((45-Suhu_T1<5)||(75-Suhu_T2<10)||(45-Suhu_T3<5)||(75-Suhu_T4<10)) && (flag_trip_overtemperature==OFF)) // Warning Over Temperature Charge 40 65 40 65
			  		{
			  			  fault_code=7;
			  			  if((Suhu_T1>45)||(Suhu_T2>75)||(Suhu_T3>45)||(Suhu_T4>75))
			  			  {
			  				  	  Batt_Open_Mode();
			  				  	  flag_trip_overtemperature=ON;
			  				  	  HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
			  			  }
			  		}

			  		//**************Pengecekan UnderTemperature ****************************//
			  		else if((Suhu_T1-Temp_Under_Set<=10||Suhu_T2-Temp_Under_Set<=10||Suhu_T3-Temp_Under_Set<=10||Suhu_T4-Temp_Under_Set<=10) && flag_trip_undertemperature==OFF)
			  		{
			  			  fault_code=8;
			  			  if(Suhu_T1<=Temp_Under_Set+10 && Suhu_T1>Temp_Under_Set+5)
			  			  {
			  				  if((test_tim2%1000)==0)
			  				  {
			  					  BUZZ_Toggle;
			  					  test_tim2=0;
			  				  }
			  			  }
			  			  else if(Suhu_T1<=Temp_Under_Set+5 && Suhu_T1>Temp_Under_Set+2)
			  			  {
			  				  if((test_tim2%500)==0)
			  				  {
			  					  BUZZ_Toggle;
			  					  test_tim2=0;
			  				  }
			  			  }
			  			  else if(Suhu_T1<Temp_Under_Set+2 && Suhu_T1>=Temp_Under_Set)
			  			  {
			  				  if((test_tim2%500)==0)
			  				  {
			  					  BUZZ_Toggle;
			  					  test_tim2=0;
			  				  }
			  			  }
			  			  else if(Suhu_T1<Temp_Under_Set||Suhu_T2<Temp_Under_Set||Suhu_T3<Temp_Under_Set||Suhu_T4<Temp_Under_Set)
			  			  {
			  				  Batt_Open_Mode();
			  				  flag_trip_undertemperature=ON;
			  				  HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
			  			  }
			  		}

				    //**************Pengecekan OverCurrent Charge **********************//
			  		else if((fabs(IBATT)-I_Over_Set_Charge)>0 && flag_trip_overcurrentcharge==OFF)   //Indikasi terjadi Over Current
				    {
				    	fault_code=10;
				    	T_I_Over_trip=14.5/(((IBATT/6.9)*(IBATT/6.9))-1);
				    	T_I_Over_trip_cycle+=0.001;

	    		  		if(T_I_Over_trip_cycle>T_I_Over_trip && flag_trip_overcurrentcharge==OFF)
	    		  		{
	    		  			Batt_Open_Mode();
	    		  			T_I_Over_trip_cycle=T_I_Over_trip;
	    		  			flag_trip_overcurrentcharge=ON;
	    		  			HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
	    		  		}

	    		  		if(flag_trip_overcurrentcharge==OFF)
	    	  			{
				    		if(T_I_Over_trip-T_I_Over_trip_cycle>15)
				    		{
				    			if((test_tim2%1000)==0)
				    			{
				    				BUZZ_Toggle;
				    				test_tim2=0;
				    			}
				    		}
				    		else if(T_I_Over_trip-T_I_Over_trip_cycle>10)
				    		{
				    			if((test_tim2%100)==0)
				    			{
				    				BUZZ_Toggle;
				    				test_tim2=0;
				    			}
				    		}
				    		else if(T_I_Over_trip-T_I_Over_trip_cycle>1)
				    		{
				    			HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
				    		}
				    	}

		 		  	}

				    ///////////////////// Overvoltage Protection//////////////////////////////
			  		else if(VBATT>V_Over_Set)
			  		{
			  			fault_code=11;
			  			flag_trip_overvoltage=ON;
			  			Batt_Open_Mode();
			  		}

				    //Clearing when data status is normal before system trip
				    else
				    {
				    	  if(fault_code!=0) last_fault_code=fault_code;
				    	  fault_code=0;
				    	  HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
				    	  T_Under_trip=0;
				    	  T_trip_cycle=T_trip_cycle-0.001;
				    	  T_I_Over_trip_cycle-=0.001;
				    	  if(T_trip_cycle<0) T_trip_cycle=0;
				    	  if(T_I_Over_trip_cycle<0) T_I_Over_trip_cycle=0;
				     }
}


/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
