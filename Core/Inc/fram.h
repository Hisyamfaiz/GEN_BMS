#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include "main.h"
#include "stm32f1xx_hal.h"
#include "i2c.h"
//#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "addr_fram.h"

#define EEPROM_ADDRESS 0xA0

extern void WriteData_FRAM(int addr, uint8_t data);
extern uint8_t ReadData_FRAM(int addr);
extern void WriteChar_FRAM(int addr, char data_char[]);
extern uint8_t * ReadChar_FRAM(int addr0, int addrn, uint8_t data_read_char[]);
extern void WritemByte_FRAM(int addr0, uint8_t mByte[]);
extern void ReadmByte_FRAM(int addr0, uint8_t mByte[]);
extern float Read_latitude(void);
extern float Read_longitude(void);

#ifdef __cplusplus
}
#endif
