/***********************************************************************************
  * @file    mcp3564_conf.h
  * @brief   C Library for configuring the MCP3561/2/4 1/2/4 channel
  *          24 Bit sigma-delta ADC on STM32
  * @version 0.1
  * @date    2022-08-02
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

#ifndef INC_MCP3564_CONF_H_
#define INC_MCP3564_CONF_H_




/*
//@note  4-channel, SCAN conversion at fs = 2.4kHz max.
// ( clock selection | ADC mode | input current selection )
#define MCP3561_USERCONF_REG0 (MCP3561_CONFIG0_CLK_SEL_EXT | MCP3561_CONFIG0_ADC_MODE_CONV | MCP3561_CONFIG0_CS_SEL_NONE)
// ( oversampling rate | sample clock prescaler )
#define MCP3561_USERCONF_REG1 (MCP3561_CONFIG1_OSR_4096 | MCP3561_CONFIG1_AMCLK_DIV0)
// ( boost | gain | offset cancellation algorithm )
#define MCP3561_USERCONF_REG2 (MCP3561_CONFIG2_BOOST_x1 | MCP3561_CONFIG2_GAIN_x1 | MCP3561_CONFIG2_AZ_MUX_OFF)
// ( conversion mode | SPI output data format 32 or 24 bit | CRC | gain calibration | offset calibration )
#define MCP3561_USERCONF_REG3 (MCP3561_CONFIG3_CONV_MODE_CONTINUOUS | MCP3561_CONFIG3_DATA_FORMAT_24BIT | MCP3561_CONFIG3_CRCCOM_OFF | MCP3561_CONFIG3_GAINCAL_OFF | MCP3561_CONFIG3_OFFCAL_OFF)
// ( IRQ default pin state | fast commands | start of conversion IRQ )
#define MCP3561_USERCONF_IRQ_REG (MCP3561_IRQ_MODE_IRQ_HIGH | MCP3561_IRQ_FASTCMD_ON | MCP3561_IRQ_STP_ON)
#define MCP3561_USERCONF_SCAN_ENABLE // enable / disable SCAN conversion mode
#define MCP3561_USERCONF_SCAN_REG (MCP3561_SCAN_DLY_8 | MCP3561_SCAN_CH_DIFF_A | MCP3561_SCAN_CH_DIFF_B | MCP3561_SCAN_CH_DIFF_C | MCP3561_SCAN_CH_DIFF_D)
#define MCP3561_USERCONF_TIMER_VAL (1333)
*/

/*
//@note single channel, polling conversion at fs = 2.4kHz max.
//       conversion time is 4.2ms --> polling delay >=4.2ms
#define MCP3561_USERCONF_REG0 (MCP3561_CONFIG0_CLK_SEL_EXT | MCP3561_CONFIG0_ADC_MODE_CONV | MCP3561_CONFIG0_CS_SEL_NONE)
#define MCP3561_USERCONF_REG1 (MCP3561_CONFIG1_OSR_4096 | MCP3561_CONFIG1_AMCLK_DIV0)
#define MCP3561_USERCONF_REG2 (MCP3561_CONFIG2_BOOST_x1 | MCP3561_CONFIG2_GAIN_x1 | MCP3561_CONFIG2_AZ_MUX_OFF)
#define MCP3561_USERCONF_REG3 (MCP3561_CONFIG3_CONV_MODE_CONTINUOUS | MCP3561_CONFIG3_DATA_FORMAT_24BIT | MCP3561_CONFIG3_CRCCOM_OFF | MCP3561_CONFIG3_GAINCAL_OFF | MCP3561_CONFIG3_OFFCAL_OFF)
#define MCP3561_USERCONF_IRQ_REG (MCP3561_IRQ_MODE_IRQ_HIGH | MCP3561_IRQ_FASTCMD_ON | MCP3561_IRQ_STP_ON)
#undef MCP3561_USERCONF_SCAN_ENABLE // enable / disable SCAN conversion mode
*/

/*
//@note single channel, polling conversion at fs = 12.4 Hz max.
//       conversion time is 80.8ms --> polling delay >=80.8ms
#define MCP3561_USERCONF_REG0 (MCP3561_CONFIG0_CLK_SEL_EXT | MCP3561_CONFIG0_ADC_MODE_CONV | MCP3561_CONFIG0_CS_SEL_NONE)
#define MCP3561_USERCONF_REG1 (MCP3561_CONFIG1_OSR_81920 | MCP3561_CONFIG1_AMCLK_DIV0)
#define MCP3561_USERCONF_REG2 (MCP3561_CONFIG2_BOOST_x1 | MCP3561_CONFIG2_GAIN_x1 | MCP3561_CONFIG2_AZ_MUX_OFF)
#define MCP3561_USERCONF_REG3 (MCP3561_CONFIG3_CONV_MODE_CONTINUOUS | MCP3561_CONFIG3_DATA_FORMAT_24BIT | MCP3561_CONFIG3_CRCCOM_OFF | MCP3561_CONFIG3_GAINCAL_OFF | MCP3561_CONFIG3_OFFCAL_OFF)
#define MCP3561_USERCONF_IRQ_REG (MCP3561_IRQ_MODE_IRQ_HIGH | MCP3561_IRQ_FASTCMD_ON | MCP3561_IRQ_STP_ON)
#undef MCP3561_USERCONF_SCAN_ENABLE // enable / disable SCAN conversion mode
*/


//@note  4-channel, auto conversion in SCAN mode at fs = 110 Hz max.
//       32 bit data format with CHannel ID
#define MCP3561_USERCONF_REG0 (MCP3561_CONFIG0_CLK_SEL_EXT | MCP3561_CONFIG0_ADC_MODE_CONV | MCP3561_CONFIG0_CS_SEL_NONE)
#define MCP3561_USERCONF_REG1 (MCP3561_CONFIG1_OSR_4096 | MCP3561_CONFIG1_AMCLK_DIV0)
#define MCP3561_USERCONF_REG2 (MCP3561_CONFIG2_BOOST_x1 | MCP3561_CONFIG2_GAIN_x1 | MCP3561_CONFIG2_AZ_MUX_OFF)
#define MCP3561_USERCONF_REG3 (MCP3561_CONFIG3_CONV_MODE_CONTINUOUS | MCP3561_CONFIG3_DATA_FORMAT_32BIT_CHID_SGN | MCP3561_CONFIG3_CRCCOM_OFF | MCP3561_CONFIG3_GAINCAL_OFF | MCP3561_CONFIG3_OFFCAL_OFF)
#define MCP3561_USERCONF_IRQ_REG (MCP3561_IRQ_MODE_IRQ_HIGH | MCP3561_IRQ_FASTCMD_ON )
#define MCP3561_USERCONF_SCAN_ENABLE // enable / disable SCAN conversion mode
#define MCP3561_USERCONF_SCAN_REG (MCP3561_SCAN_DLY_512 | MCP3561_SCAN_CH_DIFF_A | MCP3561_SCAN_CH_DIFF_B | MCP3561_SCAN_CH_DIFF_C | MCP3561_SCAN_CH_DIFF_D)
#define MCP3561_USERCONF_TIMER_VAL (1206222)


#endif /* INC_MCP3564_CONF_H_ */
