#ifndef SI7021.H

#define SI7021.H

//*Includes*/
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_gpio.h"

/*Defines*/

#define SI7021_SLAVE_ADDR				0x40

// SI7021 Sensor Commands
#define TRIGGER_TEMP_MEASURE_HOLD		0xE3
#define TRIGGER_HUMD_MEASURE_HOLD  		0xE5
#define TRIGGER_TEMP_MEASURE_NOHOLD  	0xF3
#define TRIGGER_HUMD_MEASURE_NOHOLD  	0xF5
#define WRITE_USER_REG  				0xE6
#define READ_USER_REG  					0xE7
#define SOFT_RESET  					0xFE

// SI7021 Driver Error Codes
#define SI7021_ERR_OK					"Operation returned OK."
#define SI7021_ERR_CONFIG				"Error on configration."
#define SI7021_ERR_INSTALL				"Error on installation."
#define SI7021_ERR_NOTFOUND				"Error - NOT FOUND"
#define SI7021_ERR_INVALID_ARG			"Error - Invalid Argument"
#define SI7021_ERR_FAIL		 			"Error - Failed"
#define SI7021_ERR_INVALID_STATE		"Error - Invalid State"
#define SI7021_ERR_TIMEOUT	 			"Error - Time-Out"

#define MASTER_CLOCK_SPEED 				100000


typedef struct {
	I2C_InitTypeDef i2c_struct;
	GPIO_InitTypeDef gpio_struct;
} DRV_TEMP_Config_t;

void DRV_TEMP_Init(DRV_TEMP_Config_t *cfg);
void DRV_TEMP_Deinit(I2C_TypeDef* i2cType,GPIO_TypeDef* gpi0Type);
float DRV_TEMP_Read();
float DRV_HUM_Read();
//uint16_t read_value(uint8_t command);
//int is_crc_valid(uint16_t value, uint8_t crc);
void I2C_stop(I2C_TypeDef* I2Cx);
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx);
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx);
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data);
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);


#endif
