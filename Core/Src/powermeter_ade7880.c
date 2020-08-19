/*
 * ltc6812.c
 *
 *  Created on: 20 Mei 2016
 *      Author: SUPPORT 4
 */

#include <stdio.h>
#include <string.h>
#include "powermeter_ade7880.h"
#include "math.h"
#include "main.h"

#if (UNIQUE_Code == 0xAAAA1)
float minus_offset[15];
//float minus_offset[15]={512-70,512+140,512-700,512-600,512-700,512-100,512,512-500,512-500,512-500,512,512+140,512-560,512-560,512-600}; //modul A
#else
float minus_offset[15]={500,840,-40,-40,-100,540,830,20,-130,-120,560,880,-30,-110,-140}; //modul B
#endif

extern uint8_t cmd1[4];
uint8_t cmd2[2];
extern uint8_t cmd_out[8];

extern uint32_t cmd32;
const uint16_t crc15Table[256] = {0x0,0xc599, 0xceab, 0xb32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,  //!<precomputed CRC15 Table
                                0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1,
                                0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e,
                                0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b,
                                0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd,
                                0x2544, 0x2be, 0xc727, 0xcc15, 0x98c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c,
                                0x3d6e, 0xf8f7,0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x7c2, 0xc25b, 0xc969, 0xcf0, 0xdf0d,
                                0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
                                0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640,
                                0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba,
                                0x4a88, 0x8f11, 0x57c, 0xc0e5, 0xcbd7, 0xe4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b,
                                0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921,
                                0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070,
                                0x85e9, 0xf84, 0xca1d, 0xc12f, 0x4b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a, 0x3528,
                                0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59,
                                0x2ac0, 0xd3a, 0xc8a3, 0xc391, 0x608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7, 0x54aa, 0x9133, 0x9a01,
                                0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9,
                                0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a,
                                0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25,
                                0x2fbc, 0x846, 0xcddf, 0xc6ed, 0x374, 0xd089, 0x1510, 0x1e22, 0xdbbb, 0xaf8, 0xcf61, 0xc453,
                                0x1ca, 0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b,
                                0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3,
                                0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095
                               };
uint8_t CFGAR4=0x00,
			CFGAR5=0x00,
			CFGBR0=0x00;

extern float v_cell_tot;
extern float persen_imbalance;
uint16_t balance_status;
float delta_vbatt[15];

float Cell_Voltage_Lowest;
void ltc6812_GPIO_Config(void);
void ltc6812_SPIInit(void);
void ltc6812_Delay(volatile uint32_t nCount);


void ltc6812_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIO clocks */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitStructure.Pin       = ltc6812_CS_PIN;
	GPIO_InitStructure.Mode      = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull      = GPIO_NOPULL;
	GPIO_InitStructure.Speed     = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(ltc6812_CS_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = ltc6812_SCK_PIN;
	HAL_GPIO_Init(ltc6812_SCK_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = ltc6812_MOSI_PIN;
	HAL_GPIO_Init(ltc6812_MOSI_PORT, &GPIO_InitStructure);

	

	GPIO_InitStructure.Pin       = ltc6812_MISO_PIN;
	GPIO_InitStructure.Mode      = GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull      = GPIO_NOPULL;
	GPIO_InitStructure.Speed     = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(ltc6812_MISO_PORT, &GPIO_InitStructure);
}

void ltc6812_SPIInit(void)
{
	ltc6812_CS_SET(ltc6812_CS_PIN);
	ltc6812_RESET_HIGH;
	ltc6812_Delay(10);
	ltc6812_RESET_LOW;
	ltc6812_Delay(2000);
	ltc6812_RESET_HIGH;
	ltc6812_Delay(10);

	//toggle CS 3 times to enter SPI Mode
	ltc6812_CS_RESET(ltc6812_CS_PIN);
	ltc6812_Delay(100);
	ltc6812_CS_SET(ltc6812_CS_PIN);
	ltc6812_Delay(100);
	ltc6812_CS_RESET(ltc6812_CS_PIN);
	ltc6812_Delay(100);
	ltc6812_CS_SET(ltc6812_CS_PIN);
	ltc6812_Delay(100);
	ltc6812_CS_RESET(ltc6812_CS_PIN);
	ltc6812_Delay(100);
	ltc6812_CS_SET(ltc6812_CS_PIN);
	ltc6812_Delay(100);

	ltc6812_Delay(20000);
}

void ltc6812_Delay(volatile uint32_t nCount)
{
	while(nCount > 0) { nCount--; }
}

void ltc6812_Write8(uint8_t out)
{
	uint8_t i;

	//ltc6812_CS_SET(CS_Pin);
	//ltc6812_MOSI_RESET;
	ltc6812_SCK_RESET;
	//ltc6812_CS_RESET(CS_Pin);
	//ltc6812_Delay(10);
	for (i = 0; i < 8; i++) {
		if ((out >> (7-i)) & 0x01) {
			ltc6812_MOSI_SET;
		} else {
			ltc6812_MOSI_RESET;
		}
		ltc6812_Delay(10);
		ltc6812_SCK_SET;
		ltc6812_Delay(10);
		ltc6812_SCK_RESET;
	}
}

void ltc6812_Write16(uint16_t out)
{
	uint8_t i;

	//ltc6812_CS_SET(CS_Pin);
	//ltc6812_MOSI_RESET;
	//ltc6812_SCK_RESET;
	//ltc6812_CS_RESET(CS_Pin);
  //ltc6812_Delay(10);
	for (i = 0; i < 16; i++) {
		if ((out >> (15-i)) & 0x01) {
			ltc6812_MOSI_SET;
		} else {
			ltc6812_MOSI_RESET;
		}
		ltc6812_Delay(10);
		ltc6812_SCK_RESET;
		ltc6812_Delay(10);
		ltc6812_SCK_SET;
	}
}

void ltc6812_Write32(uint32_t out)
{
	uint8_t i;

	//ltc6812_CS_SET(CS_Pin);
	//ltc6812_MOSI_RESET;
	//ltc6812_SCK_RESET;
	//ltc6812_CS_RESET(CS_Pin);
	//ltc6812_Delay(10);
	for (i = 0; i < 32; i++) {
		if ((out >> (31-i)) & 0x01) {
			ltc6812_MOSI_SET;
		} else {
			ltc6812_MOSI_RESET;
		}
		ltc6812_Delay(10);
		ltc6812_SCK_RESET;
		ltc6812_Delay(10);
		ltc6812_SCK_SET;
	}
}

uint8_t ltc6812_Read8()
{
	uint8_t i;
	uint8_t temp = 0;
	//ltc6812_Delay(10);
	//ltc6812_CS_RESET(CS_Pin);
	ltc6812_MOSI_RESET;
	ltc6812_SCK_RESET;
	for (i = 0; i < 8; i++) {
		ltc6812_Delay(10);
		ltc6812_SCK_SET;
		ltc6812_Delay(10);
		if (ltc6812_MISO == GPIO_PIN_SET) {
			temp |= (1 << (7-i));
		}
		ltc6812_Delay(10);
		ltc6812_SCK_RESET;
	}
	//ltc6812_CS_SET(CS_Pin);

	return temp;
}

uint16_t ltc6812_Read16()
{
	uint8_t i;
	uint16_t temp = 0;
	//ltc6812_Delay(10);
	//ltc6812_CS_RESET(CS_Pin);
	ltc6812_MOSI_RESET;
	ltc6812_SCK_SET;
	for (i = 0; i < 16; i++) {
		ltc6812_Delay(20);
		ltc6812_SCK_RESET;
		ltc6812_Delay(20);
		if (ltc6812_MISO == GPIO_PIN_SET) {
			temp |= (1 << (15-i));
		}
		ltc6812_Delay(20);
		ltc6812_SCK_SET;
	}
	//ltc6812_CS_SET(CS_Pin);

	return temp;
}

uint32_t ltc6812_Read32()
{
	uint8_t i;
	uint32_t temp = 0;
	//ltc6812_Delay(10);
	//ltc6812_CS_RESET(CS_Pin);
	ltc6812_MOSI_RESET;
	ltc6812_SCK_SET;
	for (i = 0; i < 32; i++) {
		ltc6812_Delay(20);
		ltc6812_SCK_RESET;
		ltc6812_Delay(20);
		if (ltc6812_MISO == GPIO_PIN_SET) {
			temp |= (1 << (31-i));
		}
		ltc6812_Delay(20);
		ltc6812_SCK_SET;
	}
	//ltc6812_CS_SET(CS_Pin);

	return temp;
}


void LTC681x_adcv_emul( uint8_t MD, //ADC Mode
				   uint8_t DCP, //Discharge Permit
				   uint8_t CH //Cell Channels to be measured
                 )
{
	uint8_t cmd[2];
	uint8_t md_bits;
	
	md_bits = (MD & 0x02) >> 1;
	cmd[0] = md_bits + 0x02;
	md_bits = (MD & 0x01) << 7;
	cmd[1] =  md_bits + 0x60 + (DCP<<4) + CH;
	
	cmd_68_emul(cmd);
}

void cmd_68_emul(uint8_t tx_cmd[2])
{
    uint8_t cmd[4];
    uint16_t cmd_pec;
//    uint8_t md_bits;

    cmd[0] = tx_cmd[0];
    cmd[1] =  tx_cmd[1];
    cmd_pec = pec15_calc_emul(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);
    ltc6812_CS_RESET(ltc6812_CS_PIN);
    ltc6812_Write8(cmd[0]);
		ltc6812_Write8(cmd[1]);
		ltc6812_Write8(cmd[2]);
		ltc6812_Write8(cmd[3]);
    ltc6812_CS_SET(ltc6812_CS_PIN);

}



uint16_t pec15_calc_emul(uint8_t len, //Number of bytes that will be used to calculate a PEC
                    uint8_t *data //Array of data that will be used to calculate  a PEC
                   )
{
    uint16_t remainder,addr;

    remainder = 16;//initialize the PEC
    for (uint8_t i = 0; i<len; i++) { // loops for each byte in data array
        addr = ((remainder>>7)^data[i])&0xff;//calculate PEC table address
//#ifdef MBED
        remainder = (remainder<<8)^crc15Table[addr];
//#else
//        remainder = (remainder<<8)^pgm_read_word_near(crc15Table+addr);
//#endif
    }
    return(remainder*2);//The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}

/* Writes the command and reads the raw cell voltage register data */
void LTC681x_rdcv_reg_emul(uint8_t reg, //Determines which cell voltage register is read back
                      uint8_t total_ic, //the number of ICs in the
                      uint8_t data_out[8] //An array of the unparsed cell codes
                     )
{
	uint8_t cmd[4];
	uint16_t cmd_pec;

	if (reg == 1)     //1: RDCVA
	{
		cmd[1] = 0x04;  //alamat asli 0x04
		cmd[0] = 0x00;
	}
	else if (reg == 2) //2: RDCVB
	{
		cmd[1] = 0x06;
		cmd[0] = 0x00;
	}
	else if (reg == 3) //3: RDCVC
	{
		cmd[1] = 0x08;
		cmd[0] = 0x00;
	}
	else if (reg == 4) //4: RDCVD
	{
		cmd[1] = 0x0A;
		cmd[0] = 0x00;
	}
	else if (reg == 5) //4: RDCVE
	{
		cmd[1] = 0x09;
		cmd[0] = 0x00;
	}
	else if (reg == 6) //4: RDCVF
	{
		cmd[1] = 0x0B;
		cmd[0] = 0x00;
	}

	cmd_pec = pec15_calc_emul(2, cmd);
	cmd[2] = (uint8_t)(cmd_pec >> 8);
	cmd[3] = (uint8_t)(cmd_pec);

	ltc6812_CS_RESET(ltc6812_CS_PIN);
  ltc6812_Write8(cmd[0]);
	ltc6812_Write8(cmd[1]);
	ltc6812_Write8(cmd[2]);
	ltc6812_Write8(cmd[3]);
	
	data_out[0]= ltc6812_Read8();
	data_out[1]= ltc6812_Read8();
	data_out[2]= ltc6812_Read8();
	data_out[3]= ltc6812_Read8();
	data_out[4]= ltc6812_Read8();
	data_out[5]= ltc6812_Read8();
	data_out[6]= ltc6812_Read8();
	data_out[7]= ltc6812_Read8();
	
  ltc6812_CS_SET(ltc6812_CS_PIN);
	
}

void convert_to_cell_data(uint8_t data_in[8],
													float cell_data[3])
{
	uint16_t v_cell_digi;
	uint8_t ccc=0;
	
	v_cell_digi=data_in[ccc+1]<<8|data_in[ccc];
	cell_data[ccc]=v_cell_digi*0.0001;
	ccc++;
	
	v_cell_digi=data_in[2*ccc+1]<<8|data_in[2*ccc];
	cell_data[ccc]=v_cell_digi*0.0001;
	ccc++;
	
	v_cell_digi=data_in[ccc+1]<<8|data_in[ccc];
	cell_data[ccc]=v_cell_digi*0.0001;
	
}

void read_v_15cell(uint16_t v_cell_digi[15],float vcell_data[15])
{
		uint8_t cmd_v[8];
		uint8_t cc=0;
		
		HAL_GPIO_WritePin(CS_SPI1_GPIO_Port,CS_SPI1_Pin,GPIO_PIN_RESET);
		cmd2[0]=0x07;
		cmd2[1]=0x23;
		cmd_68_emul(cmd2);	
		HAL_Delay(1);
		LTC681x_adcv_emul(0,1,0);
		HAL_Delay(700);
	
		LTC681x_rdcv_reg_emul(1,1,cmd_v);
		
		v_cell_digi[cc]=(cmd_v[2*cc+1]<<8|cmd_v[2*cc])-minus_offset[cc];
		vcell_data[cc]=v_cell_digi[cc]*0.0001;
		cc++;
	
		v_cell_digi[cc]=(cmd_v[2*cc+1]<<8|cmd_v[2*cc])-minus_offset[cc];
		vcell_data[cc]=v_cell_digi[cc]*0.0001;
		cc++;
	
		v_cell_digi[cc]=(cmd_v[2*cc+1]<<8|cmd_v[2*cc])-minus_offset[cc];
		vcell_data[cc]=v_cell_digi[cc]*0.0001;
		cc++;
		
		LTC681x_rdcv_reg_emul(2,1,cmd_v);
		cc=0;
		
		v_cell_digi[cc+3]=(cmd_v[2*cc+1]<<8|cmd_v[2*cc])-minus_offset[cc+3];
		vcell_data[cc+3]=v_cell_digi[cc+3]*0.0001;
		cc++;
	
		v_cell_digi[cc+3]=(cmd_v[2*cc+1]<<8|cmd_v[2*cc])-minus_offset[cc+3];
		vcell_data[cc+3]=v_cell_digi[cc+3]*0.0001;
		cc++;
	
		v_cell_digi[cc+3]=(cmd_v[2*cc+1]<<8|cmd_v[2*cc])-minus_offset[cc+3];
		vcell_data[cc+3]=v_cell_digi[cc+3]*0.0001;
		cc++;
		
		LTC681x_rdcv_reg_emul(3,1,cmd_v);
		cc=0;
		
		v_cell_digi[cc+6]=(cmd_v[2*cc+1]<<8|cmd_v[2*cc])-minus_offset[cc+6];
		vcell_data[cc+6]=v_cell_digi[cc+6]*0.0001;
		cc++;
	
		v_cell_digi[cc+6]=(cmd_v[2*cc+1]<<8|cmd_v[2*cc])-minus_offset[cc+6];
		vcell_data[cc+6]=v_cell_digi[cc+6]*0.0001;
		cc++;
	
		v_cell_digi[cc+6]=(cmd_v[2*cc+1]<<8|cmd_v[2*cc])-minus_offset[cc+6];
		vcell_data[cc+6]=v_cell_digi[cc+6]*0.0001;
		cc++;
		
		LTC681x_rdcv_reg_emul(4,1,cmd_v);
		cc=0;
		
		v_cell_digi[cc+9]=(cmd_v[2*cc+1]<<8|cmd_v[2*cc])-minus_offset[cc+9];
		vcell_data[cc+9]=v_cell_digi[cc+9]*0.0001;
		cc++;
	
		v_cell_digi[cc+9]=(cmd_v[2*cc+1]<<8|cmd_v[2*cc])-minus_offset[cc+9];
		vcell_data[cc+9]=v_cell_digi[cc+9]*0.0001;
		cc++;
	
		v_cell_digi[cc+9]=(cmd_v[2*cc+1]<<8|cmd_v[2*cc])-minus_offset[cc+9];
		vcell_data[cc+9]=v_cell_digi[cc+9]*0.0001;
		cc++;
		
		LTC681x_rdcv_reg_emul(5,1,cmd_v);
		cc=0;
		
		v_cell_digi[cc+12]=(cmd_v[2*cc+1]<<8|cmd_v[2*cc])-minus_offset[cc+12];
		vcell_data[cc+12]=v_cell_digi[cc+12]*0.0001;
		cc++;
	
		v_cell_digi[cc+12]=(cmd_v[2*cc+1]<<8|cmd_v[2*cc])-minus_offset[cc+12];
		vcell_data[cc+12]=v_cell_digi[cc+12]*0.0001;
		cc++;
	
		v_cell_digi[cc+12]=(cmd_v[2*cc+1]<<8|cmd_v[2*cc])-minus_offset[cc+12];
		vcell_data[cc+12]=v_cell_digi[cc+12]*0.0001;
		cc++;
		
		HAL_GPIO_WritePin(CS_SPI1_GPIO_Port,CS_SPI1_Pin,GPIO_PIN_SET);
}

void LTC681x_rdcfga_reg_emul( uint8_t data_out[8]
                     )
{
	uint8_t cmd[4];
	uint16_t cmd_pec;

	cmd[1] = 0x02;  //RDCFGA
	cmd[0] = 0x00;
	
	cmd_pec = pec15_calc_emul(2, cmd);
	cmd[2] = (uint8_t)(cmd_pec >> 8);
	cmd[3] = (uint8_t)(cmd_pec);

	ltc6812_CS_RESET(ltc6812_CS_PIN);
  ltc6812_Write8(cmd[0]);
	ltc6812_Write8(cmd[1]);
	ltc6812_Write8(cmd[2]);
	ltc6812_Write8(cmd[3]);
	
	data_out[0]= ltc6812_Read8();
	data_out[1]= ltc6812_Read8();
	data_out[2]= ltc6812_Read8();
	data_out[3]= ltc6812_Read8();
	data_out[4]= ltc6812_Read8();
	data_out[5]= ltc6812_Read8();
	data_out[6]= ltc6812_Read8();
	data_out[7]= ltc6812_Read8();
	
  ltc6812_CS_SET(ltc6812_CS_PIN);
	
}

void LTC681x_rdsctrl_reg_emul( uint8_t data_out[8]
                     )
{

	uint8_t cmd[4];
	uint16_t cmd_pec;

	cmd[1] = 0x16;  //RDSCTRL
	cmd[0] = 0x00;
	
	cmd_pec = pec15_calc_emul(2, cmd);
	cmd[2] = (uint8_t)(cmd_pec >> 8);
	cmd[3] = (uint8_t)(cmd_pec);

	ltc6812_CS_RESET(ltc6812_CS_PIN);
  ltc6812_Write8(cmd[0]);
	ltc6812_Write8(cmd[1]);
	ltc6812_Write8(cmd[2]);
	ltc6812_Write8(cmd[3]);
	
	data_out[0]= ltc6812_Read8();
	data_out[1]= ltc6812_Read8();
	data_out[2]= ltc6812_Read8();
	data_out[3]= ltc6812_Read8();
	data_out[4]= ltc6812_Read8();
	data_out[5]= ltc6812_Read8();
	data_out[6]= ltc6812_Read8();
	data_out[7]= ltc6812_Read8();
	
  ltc6812_CS_SET(ltc6812_CS_PIN);
	
}

void LTC681x_rdpwm_reg_emul( uint8_t data_out[8]
                     )
{
	uint8_t cmd[4];
	uint16_t cmd_pec;

	cmd[1] = 0x22;  //RDPWM
	cmd[0] = 0x00;
	
	cmd_pec = pec15_calc_emul(2, cmd);
	cmd[2] = (uint8_t)(cmd_pec >> 8);
	cmd[3] = (uint8_t)(cmd_pec);

	ltc6812_CS_RESET(ltc6812_CS_PIN);
  ltc6812_Write8(cmd[0]);
	ltc6812_Write8(cmd[1]);
	ltc6812_Write8(cmd[2]);
	ltc6812_Write8(cmd[3]);
	
	data_out[0]= ltc6812_Read8();
	data_out[1]= ltc6812_Read8();
	data_out[2]= ltc6812_Read8();
	data_out[3]= ltc6812_Read8();
	data_out[4]= ltc6812_Read8();
	data_out[5]= ltc6812_Read8();
	data_out[6]= ltc6812_Read8();
	data_out[7]= ltc6812_Read8();
	
  ltc6812_CS_SET(ltc6812_CS_PIN);
	
}

void LTC681x_wrcfga_reg_emul( uint8_t CFGA4, uint8_t CFGA5
                     )
{
	uint8_t cmd[4];
	uint16_t cmd_pec;
	uint8_t data_in[8];

	cmd[1] = 0x01;  //WRCFGA
	cmd[0] = 0x00;
	
	cmd_pec = pec15_calc_emul(2, cmd);
	cmd[2] = (uint8_t)(cmd_pec >> 8);
	cmd[3] = (uint8_t)(cmd_pec);

	ltc6812_CS_RESET(ltc6812_CS_PIN);
  ltc6812_Write8(cmd[0]);
	ltc6812_Write8(cmd[1]);
	ltc6812_Write8(cmd[2]);
	ltc6812_Write8(cmd[3]);
	
	data_in[0]= 0x00;
	data_in[1]= 0x00;
	data_in[2]= 0x00;
	data_in[3]= 0x00;
	data_in[4]= CFGA4;
	data_in[5]= CFGA5;
	cmd_pec=pec15_calc_emul(6,data_in);
	data_in[6]= (uint8_t)(cmd_pec >> 8);
	data_in[7]= (uint8_t)(cmd_pec);
	
	for(int kl=0;kl<8;kl++)
	{
		ltc6812_Write8(data_in[kl]);
	}
	
  ltc6812_CS_SET(ltc6812_CS_PIN);
	
}

void LTC681x_wrcfgb_reg_emul( uint8_t CFGB0
                     )
{
	uint8_t cmd[4];
	uint16_t cmd_pec;
	uint8_t data_in[8];

	cmd[1] = 0x24;  //WRCFGB
	cmd[0] = 0x00;

	cmd_pec = pec15_calc_emul(2, cmd);
	cmd[2] = (uint8_t)(cmd_pec >> 8);
	cmd[3] = (uint8_t)(cmd_pec);

	ltc6812_CS_RESET(ltc6812_CS_PIN);
  ltc6812_Write8(cmd[0]);
	ltc6812_Write8(cmd[1]);
	ltc6812_Write8(cmd[2]);
	ltc6812_Write8(cmd[3]);

	data_in[0]= CFGB0;
	data_in[1]= 0x00;
	data_in[2]= 0x00;
	data_in[3]= 0x00;
	data_in[4]= 0x00;
	data_in[5]= 0x00;
	cmd_pec=pec15_calc_emul(6,data_in);
	data_in[6]= (uint8_t)(cmd_pec >> 8);
	data_in[7]= (uint8_t)(cmd_pec);

	for(int kl=0;kl<8;kl++)
	{
		ltc6812_Write8(data_in[kl]);
	}

  ltc6812_CS_SET(ltc6812_CS_PIN);

}

void LTC681x_wrsctrl_reg_emul( uint8_t data_in[8]
                     )
{
	uint8_t cmd[4];
	uint16_t cmd_pec;

	cmd[1] = 0x14;  //WRSCTRL
	cmd[0] = 0x00;
	
	cmd_pec = pec15_calc_emul(2, cmd);
	cmd[2] = (uint8_t)(cmd_pec >> 8);
	cmd[3] = (uint8_t)(cmd_pec);

	ltc6812_CS_RESET(ltc6812_CS_PIN);
  ltc6812_Write8(cmd[0]);
	ltc6812_Write8(cmd[1]);
	ltc6812_Write8(cmd[2]);
	ltc6812_Write8(cmd[3]);
	
	data_in[0]= 0x00;
	data_in[1]= 0x00;
	data_in[2]= 0x00;
	data_in[3]= 0x00;
	data_in[4]= 0x00;
	data_in[5]= 0x00;
	cmd_pec=pec15_calc_emul(6,data_in);
	data_in[6]= (uint8_t)(cmd_pec >> 8);
	data_in[7]= (uint8_t)(cmd_pec);
	
	for(int kl=0;kl<8;kl++)
	{
		ltc6812_Write8(data_in[kl]);
	}
	
  ltc6812_CS_SET(ltc6812_CS_PIN);
	
}

void LTC681x_wrpwm_reg_emul( uint8_t data_in[8]
                     )
{
	uint8_t cmd[4];
	uint16_t cmd_pec;

	cmd[1] = 0x20;  //WRpwm
	cmd[0] = 0x00;
	
	cmd_pec = pec15_calc_emul(2, cmd);
	cmd[2] = (uint8_t)(cmd_pec >> 8);
	cmd[3] = (uint8_t)(cmd_pec);

	ltc6812_CS_RESET(ltc6812_CS_PIN);
	ltc6812_Write8(cmd[0]);
	ltc6812_Write8(cmd[1]);
	ltc6812_Write8(cmd[2]);
	ltc6812_Write8(cmd[3]);
	
	data_in[0]= 0xFF;
	data_in[1]= 0xFF;
	data_in[2]= 0xFF;
	data_in[3]= 0xFF;
	data_in[4]= 0xFF;
	data_in[5]= 0xFF
	;
	cmd_pec=pec15_calc_emul(6,data_in);
	data_in[6]= (uint8_t)(cmd_pec >> 8);
	data_in[7]= (uint8_t)(cmd_pec);
	
	for(int kl=0;kl<8;kl++)
	{
		ltc6812_Write8(data_in[kl]);
	}
	
  ltc6812_CS_SET(ltc6812_CS_PIN);
	
}

void LTC681x_balance_cell(uint16_t Cell_to_balance)
{
	uint8_t cell_balance_status;
	uint16_t temp_var;
	CFGAR4=0x00;
	CFGAR5=0x00;
	CFGBR0=0x00;

	for(int lm=0;lm<15;lm++)
	{
		cell_balance_status=Cell_to_balance>>lm & 0x0001;
		if(lm<8)
		{
			temp_var=cell_balance_status<<lm;
			CFGAR4+=temp_var;
		}
		else if(lm<12)
		{
			temp_var=cell_balance_status<<(lm-8);
			CFGAR5+=temp_var;
		}
		else
		{
			temp_var=cell_balance_status<<(lm-8);
			CFGBR0+=temp_var;
		}
		temp_var=0;
	}

	LTC681x_wrcfga_reg_emul(CFGAR4, CFGAR5);
	LTC681x_wrcfgb_reg_emul(CFGBR0);

}

void get_balance_status(float Cell_Voltage_15data[15])
{
	Cell_Voltage_Lowest=4.2;
	balance_status=0x0000;
	uint16_t temp_dat;
	float buffer_imbalance;


		for(int ik=0;ik<15;ik++)
		{
			if(Cell_Voltage_15data[ik]<Cell_Voltage_Lowest) Cell_Voltage_Lowest=Cell_Voltage_15data[ik];
		}

		for(int ik=0;ik<15;ik++)
		{
		   delta_vbatt[ik] = Cell_Voltage_15data[ik] - Cell_Voltage_Lowest;

		   buffer_imbalance+=delta_vbatt[ik];

		   if(delta_vbatt[ik]> 0.025 && Cell_Voltage_15data[ik]>VCELL_BALANCE_PERMITTED)
		   {
			   temp_dat = 0x01;
			   temp_dat = temp_dat << ik;
			   balance_status= balance_status+temp_dat;
		   }

		}
		persen_imbalance=buffer_imbalance*100/14.0/1.2;


}
void LTC681x_MUTE_UNMUTE_emul( uint8_t mute_unmute)
{
	uint8_t cmd[4];
	uint16_t cmd_pec;
//	uint8_t data_in[8];
	if(mute_unmute==1)
	{
		cmd[1] = 0x28;  //MUTE
		cmd[0] = 0x00;
	}
	else
	{
		cmd[1] = 0x29;  //UNMUTE
		cmd[0] = 0x00;
	}

	cmd_pec = pec15_calc_emul(2, cmd);
	cmd[2] = (uint8_t)(cmd_pec >> 8);
	cmd[3] = (uint8_t)(cmd_pec);

	ltc6812_CS_RESET(ltc6812_CS_PIN);
  ltc6812_Write8(cmd[0]);
	ltc6812_Write8(cmd[1]);
	ltc6812_Write8(cmd[2]);
	ltc6812_Write8(cmd[3]);
}
