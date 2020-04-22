/*
 * ltc6812.h
 *
 *  Created on: 20 Mei 2016
 *      Author: SUPPORT 4
 */

#ifndef ltc6812_INC_ltc6812_H_
#define ltc6812_INC_ltc6812_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"


#define ltc6812_RESET_PIN  	GPIO_PIN_8//GPIO_Pin_12
#define ltc6812_RESET_PORT	GPIOC

#define ltc6812_CS_PIN      GPIO_PIN_0//GPIO_Pin_4
#define ltc6812_CS_PORT			GPIOB
#define ltc6812_MOSI_PIN    GPIO_PIN_5//GPIO_Pin_6
#define ltc6812_MOSI_PORT		GPIOB
#define ltc6812_MISO_PIN    GPIO_PIN_4//GPIO_Pin_5
#define ltc6812_MISO_PORT		GPIOB
#define ltc6812_SCK_PIN	    GPIO_PIN_3//GPIO_Pin_2
#define ltc6812_SCK_PORT		GPIOB
//#define ltc6812_CF1_PIN		GPIO_Pin_11
//#define ltc6812_CF2_PIN		GPIO_Pin_13
//#define ltc6812_CF3_PIN		GPIO_Pin_14

#define ltc6812_SCK_SET		HAL_GPIO_WritePin(ltc6812_SCK_PORT, ltc6812_SCK_PIN, GPIO_PIN_SET)
#define ltc6812_SCK_RESET	HAL_GPIO_WritePin(ltc6812_SCK_PORT, ltc6812_SCK_PIN, GPIO_PIN_RESET)

#define ltc6812_MOSI_SET	HAL_GPIO_WritePin(ltc6812_MOSI_PORT, ltc6812_MOSI_PIN, GPIO_PIN_SET)
#define ltc6812_MOSI_RESET	HAL_GPIO_WritePin(ltc6812_MOSI_PORT, ltc6812_MOSI_PIN, GPIO_PIN_RESET)

#define ltc6812_CS_SET(x)	HAL_GPIO_WritePin(ltc6812_CS_PORT, x, GPIO_PIN_SET)//D10_Port->BSRRL = x;
#define ltc6812_CS_RESET(x)	HAL_GPIO_WritePin(ltc6812_CS_PORT, x, GPIO_PIN_RESET)//D10_Port->BSRRH = x;

#define ltc6812_MISO		HAL_GPIO_ReadPin(ltc6812_MISO_PORT, ltc6812_MISO_PIN)

#define ltc6812_RESET_HIGH	HAL_GPIO_WritePin(ltc6812_RESET_PORT, ltc6812_RESET_PIN, GPIO_PIN_SET);//GPIOE->BSRRL = ltc6812_RESET_PIN;
#define ltc6812_RESET_LOW	HAL_GPIO_WritePin(ltc6812_RESET_PORT, ltc6812_RESET_PIN, GPIO_PIN_RESET);//GPIOE->BSRRH = ltc6812_RESET_PIN;

#define VBATT_BALANCE_START 55
#define VCELL_BALANCE_PERMITTED 3.4

void ltc6812_Config(void);
void ltc6812_getData(float *Vrms_R, float *Vrms_S, float *Vrms_T, float *Irms_R, float *Irms_S, float *Irms_T, float *Irms_N);
void ltc6812_getDataPFf(float *pf_R,float *pf_S,float *pf_T,float *freq_R,float *freq_S, float *freq_T);
void ltc6812_getDataPOW(float *P_R,float *P_S, float *P_T);
void ltc6812_getDataVA(float *VA_R,float *VA_S, float *VA_T);

void ltc6812_GPIO_Config(void);
void ltc6812_SPIInit(void);
void ltc6812_Delay(volatile uint32_t nCount);;

void ltc6812_Write8(uint8_t out);
void ltc6812_Write16(uint16_t out);
void ltc6812_Write32(uint32_t out);
uint8_t ltc6812_Read8(void);
uint16_t ltc6812_Read16(void);
uint32_t ltc6812_Read32(void);

void cmd_68_emul(uint8_t tx_cmd[2]);
uint16_t pec15_calc_emul(uint8_t len, //Number of bytes that will be used to calculate a PEC
                    uint8_t *data //Array of data that will be used to calculate  a PEC
                   );

extern const uint16_t crc15Table[256];
void LTC681x_adcv_emul( uint8_t MD, //ADC Mode
				   uint8_t DCP, //Discharge Permit
				   uint8_t CH //Cell Channels to be measured
                 );

void LTC681x_rdcv_reg_emul(uint8_t reg, //Determines which cell voltage register is read back
                      uint8_t total_ic, //the number of ICs in the
                      uint8_t data_out[6] //An array of the unparsed cell codes
                     );
void convert_to_cell_data(uint8_t data_in[8],
													float cell_data[3]);

void LTC681x_rdcfga_reg_emul( uint8_t data_out[8] //An array of the unparsed cell codes
                     );

void LTC681x_rdsctrl_reg_emul( uint8_t data_out[8]
                     );

void LTC681x_rdpwm_reg_emul( uint8_t data_out[8]
                     );

void LTC681x_wrcfga_reg_emul( uint8_t CFGA4, uint8_t CFGA5
                     );

void LTC681x_wrsctrl_reg_emul( uint8_t data_in[8]
                     );

void LTC681x_wrpwm_reg_emul( uint8_t data_in[8]
                     );

void read_v_15cell(uint16_t v_cell_digi[15],float vcell_data[15]);

void LTC681x_balance_cell(uint16_t Cell_to_balance);

void get_balance_status(float Cell_Voltage_15data[15]);

void LTC681x_MUTE_UNMUTE_emul( uint8_t mute_unmute);


#define AIGAIN 		0x4380

#ifdef __cplusplus
}
#endif

#endif /* ltc6812_INC_ltc6812_H_ */
