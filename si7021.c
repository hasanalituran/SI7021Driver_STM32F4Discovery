#include "si7021.h"
#include "stm32f4xx_rcc.h"

void DRV_TEMP_Init(DRV_TEMP_Config_t *cfg)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;

	I2C_InitStruct = cfg->i2c_struct;
	GPIO_InitStruct = cfg->gpio_struct;

	// enable APB1 peripheral clock for I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	// enable clock for SCL and SDA pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);


	/* setup SCL and SDA pins
		 * You can connect I2C1 to two different
		 * pairs of pins:
		 * 1. SCL on PB6 and SDA on PB7
		 * 2. SCL on PB8 and SDA on PB9
	*/

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_Init(GPIOB,&GPIO_InitStruct);

	// Connect I2C1 pins to AF
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);	// SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA

	I2C_InitStruct.I2C_ClockSpeed = 100000;
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C1, &I2C_InitStruct);

	I2C_Cmd(I2C1, ENABLE);

}

void DRV_TEMP_Deinit(I2C_TypeDef* i2cType,GPIO_TypeDef* gpioType)
{
	I2C_DeInit(i2cType);
	I2C_SoftwareResetCmd(i2cType,ENABLE);

	GPIO_DeInit(gpioType);

}


float DRV_TEMP_Read()
{
	uint8_t raw_temprature[3];

	I2C_start(I2C1, SI7021_SLAVE_ADDR<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2C1, TRIGGER_HUMD_MEASURE_NOHOLD); // write one byte to the slave
	//I2C_write(I2C1, 0x03); // write another byte to the slave
	I2C_stop(I2C1); // stop the transmission

	delay_ms(50);

	I2C_start(I2C1, SI7021_SLAVE_ADDR<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
	raw_temprature[0] = I2C_read_ack(I2C1); 					   //MSB Byte
	raw_temprature[1] = I2C_read_ack(I2C1); 					   //LSB Byte
	raw_temprature[2] = I2C_read_nack(I2C1);					   //CRC Byte

	uint16_t raw_value = ((uint16_t) raw_temprature[0] << 8) | (uint16_t) raw_temprature[1];
	if(raw_value == 0)
	{
		return -404;
	}

	// return the actual value according to the formula in the doc.
	return (raw_value * 175.72 / 65536.0) - 46.85;
}


float DRV_HUM_Read()
{
	uint8_t raw_humidity[3];

	I2C_start(I2C1, SI7021_SLAVE_ADDR<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2C1, TRIGGER_HUMD_MEASURE_NOHOLD); // write one byte to the slave
	//I2C_write(I2C1, 0x03); // write another byte to the slave
	I2C_stop(I2C1); // stop the transmission

	delay_ms(50);

	I2C_start(I2C1, SI7021_SLAVE_ADDR<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
	raw_humidity[0] = I2C_read_ack(I2C1); 							//MSB Byte
	raw_humidity[1] = I2C_read_ack(I2C1); 							//LSB Byte
	raw_humidity[2] = I2C_read_nack(I2C1);							//CRC Byte

	uint16_t raw_value = ((uint16_t) raw_humidity[0] << 8) | (uint16_t) raw_humidity[1];
	if(raw_value == 0)
	{
		return -404;
	}

	// return the actual value according to the formula in the doc.
	return (raw_value * 125.0 / 65536.0) - 6.0;
}



/* This function issues a start condition and
 * transmits the slave address + R/W bit
 *
 * Parameters:
 * 		I2Cx --> the I2C peripheral e.g. I2C1
 * 		address --> the 7 bit slave address
 * 		direction --> the transmission direction can be:
 * 						I2C_Direction_Tranmitter for Master transmitter mode
 * 						I2C_Direction_Receiver for Master receiver
 */
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction){
	// wait until I2C1 is not busy anymore
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

	// Send I2C1 START condition
	I2C_GenerateSTART(I2Cx, ENABLE);

	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

	// Send slave Address for write
	I2C_Send7bitAddress(I2Cx, address, direction);

	/* wait for I2C1 EV6, check if
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */
	if(direction == I2C_Direction_Transmitter){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	}
	else if(direction == I2C_Direction_Receiver){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}
}

/* This function transmits one byte to the slave device
 * Parameters:
 *		I2Cx --> the I2C peripheral e.g. I2C1
 *		data --> the data byte to be transmitted
 */
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
	I2C_SendData(I2Cx, data);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

/* This function reads one byte from the slave device
 * and acknowledges the byte (requests another byte)
 */
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx){
	// enable acknowledge of recieved data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

/* This function reads one byte from the slave device
 * and doesn't acknowledge the recieved data
 */
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx){
	// disabe acknowledge of received data
	// nack also generates stop condition after last byte received
	// see reference manual for more info
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	I2C_GenerateSTOP(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

/* This function issues a stop condition and therefore
 * releases the bus
 */
void I2C_stop(I2C_TypeDef* I2Cx){
	// Send I2C1 STOP Condition
	I2C_GenerateSTOP(I2Cx, ENABLE);
}
