/***********************************************************************************
  * @file    mcp3564.c
  * @brief   C Library for configuring the MCP3561/2/4 1/2/4 channel
  *          24 Bit sigma-delta ADC on STM32
  * @version 0.1
  * @date    2021-11-29
  * @license Apache 2.0
  * @author  Simon Burkhardt
  *
  *          FHNW University of Applied Sciences and Arts Northwestern Switzerland
  *          https://www.fhnw.ch/ise/
  *          https://github.com/fhnw-ise-qcrypt/mcp3564
  *
  *          GAP Quantum Technologies University of Geneva
  *          https://www.unige.ch/gap/qic/qtech/
  *
  * @see     https://www.microchip.com/en-us/product/MCP3561
************************************************************************************
*/

#include "main.h"
#include "mcp3564.h"

void _MCP3561_write(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t size){
	// manually operate the !CS signal, because the STM32 hardware NSS signal is (sadly) useless
	HAL_GPIO_WritePin(MCP3561_CHIP_SELECT_GPIO_Port, MCP3561_CHIP_SELECT_GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, pData, size, MCP3561_HAL_TIMEOUT);
	HAL_GPIO_WritePin(MCP3561_CHIP_SELECT_GPIO_Port, MCP3561_CHIP_SELECT_GPIO_Pin, GPIO_PIN_SET);
}

uint8_t _MCP3561_sread(SPI_HandleTypeDef *hspi, uint8_t *cmd){
	uint8_t reg8[2];
	// manually operate the !CS signal, because the STM32 hardware NSS signal is (sadly) useless
	HAL_GPIO_WritePin(MCP3561_CHIP_SELECT_GPIO_Port, MCP3561_CHIP_SELECT_GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(hspi, cmd, reg8, 2, MCP3561_HAL_TIMEOUT);
	HAL_GPIO_WritePin(MCP3561_CHIP_SELECT_GPIO_Port, MCP3561_CHIP_SELECT_GPIO_Pin, GPIO_PIN_SET);
	return reg8[1];
}

/**
 * @brief  Initializes the MCP356x chip according to user config
 * @note   must be edited by the user
 */
void MCP3561_Init(SPI_HandleTypeDef *hspi){
	uint8_t cmd[4] = {0,0,0,0};

	// be careful with the bitwise or operator "|"
	cmd[0]  = MCP3561_CONFIG0_WRITE;
	cmd[1]  = MCP3561_CONFIG0_CLK_SEL_EXT;   // clock selection
	cmd[1] |= MCP3561_CONFIG0_ADC_MODE_CONV; // standby or converting
	cmd[1] |= MCP3561_CONFIG0_CS_SEL_NONE;   // input current
	_MCP3561_write(hspi, cmd, 2);

	cmd[0]  = MCP3561_CONFIG1_WRITE;
	cmd[1]  = MCP3561_CONFIG1_OSR_512;       // over sampling rate
	cmd[1] |= MCP3561_CONFIG1_AMCLK_DIV8;    // sampling clock prescaler
	_MCP3561_write(hspi, cmd, 2);

	cmd[0]  = MCP3561_CONFIG2_WRITE;
	cmd[1]  = MCP3561_CONFIG2_BOOST_x1;   // Boost
	cmd[1] |= MCP3561_CONFIG2_GAIN_x1;    // Gain
	cmd[1] |= MCP3561_CONFIG2_AZ_MUX_OFF; // offset cancellation algorithm
	cmd[1] += 3; // last two bytes must always be '11'
	_MCP3561_write(hspi, cmd, 2);

	cmd[0]  = MCP3561_CONFIG3_WRITE;
	cmd[1]  = MCP3561_CONFIG3_CONV_MODE_CONTINUOUS; // conversion mode
	cmd[1] |= MCP3561_CONFIG3_DATA_FORMAT_32BIT;    // SPI output data format, (32 and 24 bit available)
	cmd[1] |= MCP3561_CONFIG3_CRCCOM_OFF;           // CRC
	cmd[1] |= MCP3561_CONFIG3_GAINCAL_OFF;          // gain calibration
	cmd[1] |= MCP3561_CONFIG3_OFFCAL_OFF;           // offset calibration
	_MCP3561_write(hspi, cmd, 2);

	cmd[0]  = MCP3561_IRQ_WRITE;
	cmd[1]  = MCP3561_IRQ_MODE_IRQ_HIGH;  // IRQ default pin state
	cmd[1] |= MCP3561_IRQ_FASTCMD_ON;     // fast commands
	cmd[1] |= MCP3561_IRQ_STP_ON;         // start of conversion IRQ
	_MCP3561_write(hspi, cmd, 2);

	cmd[0]  = MCP3561_MUX_WRITE;
	cmd[1]  = (MCP3561_MUX_CH0 << 4) | MCP3561_MUX_CH1;   // [7..4] VIN+ / [3..0] VIN-
	_MCP3561_write(hspi, cmd, 2);

	// configure SCAN mode to automatically cycle through channels
	// only available for MCP3562 and MCP356^4, and only for certain input combinations
	// @see Datasheet Table 5-14 on p. 54

	/*
	cmd[0] = MCP3561_SCAN_WRITE;
	cmd[1] = MCP3561_SCAN_DLY_NONE;
	cmd[2] = (1<<0) | (1<<1) | (1<<2) | (1<<3);  // enable all differential channels in SCAN mode. bits: 8,9,10,11
	cmd[3] = 0;
	_MCP3561_write(hspi, cmd, 4);
	*/


}

/**
 * @brief prints the configuration registers content
 */
void MCP3561_PrintRegisters(SPI_HandleTypeDef *hspi){
	uint8_t reg8 = 0;
	uint8_t cmd [5] = {0,0,0,0,0};

	cmd[0] = MCP3561_CONFIG0_SREAD;
	reg8 = _MCP3561_sread(hspi, cmd);
	printf("CONF0: %02x\n", reg8);

	cmd[0] = MCP3561_CONFIG1_SREAD;
	reg8 = _MCP3561_sread(hspi, cmd);
	printf("CONF1: %02x\n", reg8);

	cmd[0] = MCP3561_CONFIG2_SREAD;
	reg8 = _MCP3561_sread(hspi, cmd);
	printf("CONF2: %02x\n", reg8);

	cmd[0] = MCP3561_CONFIG3_SREAD;
	reg8 = _MCP3561_sread(hspi, cmd);
	printf("CONF3: %02x\n", reg8);

	cmd[0] = MCP3561_IRQ_SREAD;
	reg8 = _MCP3561_sread(hspi, cmd);
	printf("IRQ  : %02x\n", reg8);

	cmd[0] = MCP3561_MUX_SREAD;
	reg8 = _MCP3561_sread(hspi, cmd);
	printf("MUX  : %02x\n", reg8);

	/* @todo all the remaining registers */
}

/**
 * @brief resets the configuration to the default values
 * @todo  test this function
 */
void MCP3561_Reset(SPI_HandleTypeDef *hspi){
	uint8_t cmd;
	cmd = DEVICE_RESET_COMMAND;
	HAL_SPI_Transmit(hspi, &cmd, 1, 10);
}


/**
 * @brief read 24 Bit left justified ADC register
 * @todo  how to read from other data formats?
 */
uint32_t MCP3561_ReadADCData(SPI_HandleTypeDef *hspi){
	uint8_t val[5] = {0,0,0,0,0};
	uint8_t cmd[5] = {0,0,0,0,0};
	cmd[0] = MCP3561_SREAD_DATA_COMMAND;
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 0);
	HAL_SPI_TransmitReceive(hspi, cmd, val, 5, 10);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 1);
	uint32_t value = (val[1] << 16) | (val[2] << 8) | val[3];
	return value;
}

/*
 * @bug this does not work because it will skip the transaction and write to the chip select pin
 */
uint32_t MCP3561_ReadADCData_IT(SPI_HandleTypeDef *hspi){
	uint8_t val[5] = {0,0,0,0,0};
	uint8_t cmd[5] = {0,0,0,0,0};
	cmd[0] = MCP3561_SREAD_DATA_COMMAND;

	// manually operate the !CS signal, because the STM32 hardware NSS signal is (sadly) useless
	HAL_GPIO_WritePin(MCP3561_CHIP_SELECT_GPIO_Port, MCP3561_CHIP_SELECT_GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(hspi, cmd, val, 5, MCP3561_HAL_TIMEOUT);
	HAL_GPIO_WritePin(MCP3561_CHIP_SELECT_GPIO_Port, MCP3561_CHIP_SELECT_GPIO_Pin, GPIO_PIN_SET);

	uint32_t value = (val[1] << 16) | (val[2] << 8) | val[3];
	return value;
}

