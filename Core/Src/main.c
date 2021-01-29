/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include <ctype.h>
#include "Battery_Charge_Discharge.h"
#include "CANbus.h"
#include "powermeter_ade7880.h"
#include "fram.h"
#include "eeprom.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adc_val[8];

char pesan[20], lower_UNIQUE_Code[20], UPPER_UNIQUE_Code[20];

int test_tim2;
extern float v_datadigi,i_datadigi,vn_datadigi,ref_datadigi;
extern float VBATT, IBATT;

float Pack_SOC,	SOC_manipulasi, Delta_VCell,Bat_Pow_Out;
float Pack_Cap=21.116;//hasil
uint16_t LifeTime;
uint8_t BATT_State;
float persen_imbalance;
//test git
extern float AH_Consumption, AH_Total;

extern float Suhu_T1,Suhu_T2,Suhu_T3,Suhu_T4;

extern float T_Under_trip, T_trip_cycle;
extern uint8_t flag_trip_undervoltage;

extern uint8_t flag_trip_overtemperature;

extern float T_I_Over_trip,
			T_I_Over_trip_cycle,
			Isc,Vsc;
extern float SOC_manipulasi;
extern uint32_t cek_CC;

char buffer_uart[512];

uint8_t BATT_Start_Up=0;

uint16_t vcell_15data_digi[15];
float vcell_15data[15];
float v_cell_tot;
extern uint16_t balance_status;

int ij;

extern uint8_t fault_code,last_fault_code;
uint8_t datain_fram=119, dataout_fram;

uint8_t Flag_Battery_Locked_for_Ship,Flag_Force_Balance;

uint8_t flag_start_shutdown, BMS_mode;

uint8_t cmd4[2];

extern float minus_offset[15];

extern float OFFSET_SENSOR_ARUS,IBATT_for_offset_cal;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void BMS_ScreenMode_Standby(void);
void BMS_ScreenMode_RUN(void);
void BMS_ScreenMode_ForceBalance(void);
void BMS_ScreenMode_Locked_Ship(void);
void Calc_vcell_tot(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  //BMS_CAN_Config();
   HAL_FLASH_Unlock();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_IWDG_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //USART3 untuk RS485, tidak digunakan untuk BOARD v1.

  ltc6812_GPIO_Config();
  ltc6812_SPIInit();

  HAL_Delay(10);
  read_v_15cell(vcell_15data_digi, vcell_15data);
  for(ij=0,v_cell_tot=0;ij<15;ij++)
  {
 	v_cell_tot+=vcell_15data[ij];
  }
  HAL_Delay(10);

  BMS_CAN_Config();

  BMS_ON_InitBeep();

  BATT_Start_Up=1;

  HAL_Delay(1000);

  flag_start_shutdown=0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(DIP1 && !DIP2)			BMS_ScreenMode_ForceBalance();		//Mode Force Balancing
	  else if(!DIP1 && DIP2) 	BMS_ScreenMode_Locked_Ship();		//Mode Shipping
	  else if(DIP1 && DIP2)		BMS_ScreenMode_RUN();				//MODE RUN
	  else						BMS_ScreenMode_Standby();			//Mode Standby

	  HAL_IWDG_Refresh(&hiwdg);
	  read_v_15cell(vcell_15data_digi, vcell_15data);

	  if(flag_start_shutdown==1)
	  {
		  	  //Read voltage of each cell (15cell)
		  	  if(BATT_State==STATE_CHARGE)
		  	  {
		  		 // LTC681x_MUTE_UNMUTE_emul(1);
//		  		  read_v_15cell(vcell_15data_digi, vcell_15data);
		  		  //LTC681x_MUTE_UNMUTE_emul(0);
		  	  }
		  	  else read_v_15cell(vcell_15data_digi, vcell_15data);

		  	  //Get Balancing Data
		  	  get_balance_status(vcell_15data);

		  	  // Balancing Process
		  	  if((IBATT<-0.1 && (v_cell_tot>VBATT_BALANCE_START)) || Flag_Force_Balance==1)     //arus charging 0.1 tidak perlu di balancing
		  	  {
		  		  LTC681x_balance_cell(2052);
		  	  }
		  	  else
		  	  {
		  			balance_status=0;
		  			LTC681x_balance_cell(0x0000);
		  	  }

		  	  //Calculate total Battery Voltage
		  	  Calc_vcell_tot();

			  //Mark that System operating
			  HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
	  }

	  HAL_IWDG_Refresh(&hiwdg);
	  HAL_Delay(1);

//	  flag_start_shutdown=1;
//	  BMS_mode=1;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void BMS_ON_InitBeep(void)
{
	itoa(UNIQUE_Code, lower_UNIQUE_Code, 16);
	int ii=0;
	while(ii<6){
		UPPER_UNIQUE_Code[ii] = toupper(lower_UNIQUE_Code[ii]);
		ii++;
	}
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) &adc_val, 6);

	SSD1306_Init();
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	SSD1306_UpdateScreen();

	HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_TogglePin(BUZZ_GPIO_Port, BUZZ_Pin);
	HAL_Delay(500);
	HAL_GPIO_TogglePin(BUZZ_GPIO_Port, BUZZ_Pin);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(BUZZ_GPIO_Port, BUZZ_Pin);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(BUZZ_GPIO_Port, BUZZ_Pin);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(BUZZ_GPIO_Port, BUZZ_Pin);
	HAL_Delay(100);

	sprintf(pesan,"BMS - Gen.V2");
	SSD1306_GotoXY(5,0);
	SSD1306_Puts(pesan, &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();

	HAL_TIM_Base_Start_IT(&htim2);

	HAL_TIM_Base_Start_IT(&htim3);

	HAL_Delay(1000);

	BATT_State=STATE_STANDBY;
	Batt_Open_Mode();
	AH_Total=0;


}

void BMS_ScreenMode_Standby(void)
{
	 Flag_Battery_Locked_for_Ship=0;
	 Flag_Force_Balance=0;


	 SSD1306_Fill(SSD1306_COLOR_BLACK);
	 sprintf(pesan,"BMS - Gen.V2");
	 SSD1306_GotoXY(20,28);
	 SSD1306_Puts(pesan, &Font_7x10, SSD1306_COLOR_WHITE);
	 SSD1306_UpdateScreen();
	 Batt_Open_Mode();

	 HAL_Delay(1);
}

void BMS_ScreenMode_RUN(void)
{
	Flag_Battery_Locked_for_Ship=0;
	Flag_Force_Balance=0;

	if(flag_start_shutdown==0)
	{
		 SSD1306_Fill(SSD1306_COLOR_BLACK);
			 sprintf(pesan,"BMS - Gen.V2 - RUN");
			 SSD1306_GotoXY(0,18);
			 SSD1306_Puts(pesan, &Font_7x10, SSD1306_COLOR_WHITE);

			 sprintf(pesan,"SLEEP_STATE");
			 SSD1306_GotoXY(0,38);
			 SSD1306_Puts(pesan, &Font_7x10, SSD1306_COLOR_WHITE);
			 SSD1306_UpdateScreen();
			 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
			 Batt_Open_Mode();
			 flag_trip_overtemperature=OFF;
			 flag_trip_undertemperature=OFF;
			 flag_trip_SOCOverDischarge=OFF;
			 flag_trip_SOCOverCharge=OFF;			//di tiada kan..!
			 flag_trip_undervoltage=OFF;
			 flag_trip_overvoltage=OFF;
			 flag_trip_overcurrentdischarge=OFF;
			 flag_trip_overcurrentcharge=OFF;
			 flag_trip_shortcircuit=OFF;
			 flag_trip_systemfailure=OFF;
			 flag_trip_unbalance=OFF;
			 OFFSET_SENSOR_ARUS=IBATT_for_offset_cal;
	}
	else
	{
		SSD1306_Fill(SSD1306_COLOR_BLACK);
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		if(BATT_State==STATE_CHARGE)
		{
			sprintf(pesan,"RUN (C) - %s", UPPER_UNIQUE_Code);
			SSD1306_GotoXY(0,0);
			SSD1306_Puts(pesan, &Font_7x10, SSD1306_COLOR_WHITE);
		}
		else if(BATT_State==STATE_DISCHARGE)
		{
			sprintf(pesan,"RUN (D) - %s", UPPER_UNIQUE_Code);
			SSD1306_GotoXY(0,0);
			SSD1306_Puts(pesan, &Font_7x10, SSD1306_COLOR_WHITE);
		}
		else if(BATT_State==STATE_FULL_CHARGE_DISCHARGE)
		{
			sprintf(pesan,"BMS-RUN (Full C/D)");
			SSD1306_GotoXY(0,0);
			SSD1306_Puts(pesan, &Font_7x10, SSD1306_COLOR_WHITE);
		}
		else if(BATT_State==STATE_STANDBY)
		{
			sprintf(pesan,"BMS-RUN (Open)");
			SSD1306_GotoXY(0,0);
			SSD1306_Puts(pesan, &Font_7x10, SSD1306_COLOR_WHITE);
		}

		sprintf(pesan,"V=%6.2f I=%6.2f",VBATT,IBATT);
		SSD1306_GotoXY(0,10);
		SSD1306_Puts(pesan, &Font_7x10, SSD1306_COLOR_WHITE);
		sprintf(pesan,"T=%3.0f|%3.0f|%3.0f|%3.0f",Suhu_T1,Suhu_T2,Suhu_T3,Suhu_T4);
		SSD1306_GotoXY(0,20);
		SSD1306_Puts(pesan, &Font_7x10, SSD1306_COLOR_WHITE);
		sprintf(pesan,"C=%5.1f%%--%5.1f%%",Pack_SOC,SOC_manipulasi);
		SSD1306_GotoXY(0,30);
		SSD1306_Puts(pesan, &Font_7x10, SSD1306_COLOR_WHITE);
		sprintf(pesan,"B=%5d, %4.1f-%4.2f",balance_status,persen_imbalance, OFFSET_SENSOR_ARUS);
		SSD1306_GotoXY(0,40);
		SSD1306_Puts(pesan, &Font_7x10, SSD1306_COLOR_WHITE);


		sprintf(pesan,"%d-%d--%4.2f| %5.0f",fault_code,last_fault_code,Isc, AH_Total);
		SSD1306_GotoXY(0,50);
		SSD1306_Puts(pesan, &Font_7x10, SSD1306_COLOR_WHITE);


		SSD1306_UpdateScreen();

		if(BMS_mode==0) Batt_Open_Mode();
		else if(BMS_mode==1) Batt_Discharge_Mode();
		else if(BMS_mode==2) Batt_Charge_Mode();
		else if(BMS_mode==3) Batt_Full_CD_Mode();

	}

	HAL_Delay(1);
}

void BMS_ScreenMode_Locked_Ship(void)
{
	 Flag_Battery_Locked_for_Ship=1;
	 Flag_Force_Balance=0;

	 SSD1306_Fill(SSD1306_COLOR_BLACK);
	 sprintf(pesan,"BMS - Gen.V2");
	 SSD1306_GotoXY(20,18);
	 SSD1306_Puts(pesan, &Font_7x10, SSD1306_COLOR_WHITE);

	 sprintf(pesan,"Pack Ready Shipped");
	 SSD1306_GotoXY(0,38);
	 SSD1306_Puts(pesan, &Font_7x10, SSD1306_COLOR_WHITE);
	 SSD1306_UpdateScreen();
	 Batt_Open_Mode();
}

void BMS_ScreenMode_ForceBalance(void)
{
	Flag_Battery_Locked_for_Ship=0;
	Flag_Force_Balance=1;

	SSD1306_Fill(SSD1306_COLOR_BLACK);
	sprintf(pesan,"BMS - Gen.V2");
	SSD1306_GotoXY(20,0);
	SSD1306_Puts(pesan, &Font_7x10, SSD1306_COLOR_WHITE);

	sprintf(pesan,"Force Balance");
	SSD1306_GotoXY(0,10);
	SSD1306_Puts(pesan, &Font_7x10, SSD1306_COLOR_WHITE);

	sprintf(pesan,"B=%5d",balance_status);
	SSD1306_GotoXY(0,40);
	SSD1306_Puts(pesan, &Font_7x10, SSD1306_COLOR_WHITE);

	SSD1306_UpdateScreen();
	Batt_Open_Mode();
}

void Calc_vcell_tot(void)
{
	float v_cell_temporary;
	for(ij=0;ij<15;ij++)
	{
		v_cell_temporary+=vcell_15data[ij];
	}

	if(v_cell_temporary>10)
	{
		v_cell_tot=v_cell_temporary;
	}

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
