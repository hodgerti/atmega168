/*============================================================
* Filename: lm73_functions.h
* Author: Roger Traylor
* Co-Author: Isaac Connor Hodgert
* Date: 11-11-2019
* Description: special defines and functions for the lm73 temperature sensor
*============================================================*/

#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#ifndef TRUE
	#define TRUE 1
#endif
#ifndef FALSE
	#define FALSE 0
#endif

#include <avr/io.h>
#include "twi_master.h"

#define LM73_ADDRESS 0b10010000              // LM73-0 float
#define LM73_WRITE (LM73_ADDRESS | TW_WRITE) // LSB is a zero to write
#define LM73_READ  (LM73_ADDRESS | TW_READ)  // LSB is a one to read
#define LM73_PTR_TEMP          0x00          // LM73 temperature register address
#define LM73_PTR_CONFIG        0x01          // LM73 configuration register address
#define LM73_UP_LIMIT          0x02          // LM73 upper temperature limit register address
#define LM73_LOW_LIMIT         0x03          // LM73 lower temperature limit register address
#define LM73_PTR_CTRL_STATUS   0x04          // LM73 control and status register address
#define LM73_ID                0x07          // LM73 ID register

#define LM73_CONFIG_VALUE      0b01001000    // no power down, enable alert active low, restart alert
#define LM73_STATUS_VALUE      0b11100000    // no timeout, max resolution

#define LM73_N_ALERT_PIN       PF0
#define LM73_ADDR_PIN          PF1

#define LM73_100_C             0x3200
#define LM73_0_C               0x0000

struct lm73_data
{
	int8_t sign;
	uint8_t integer_value;
	uint16_t decimal_value;
};

//*****************************************************************************
// Name: init_lm73                                   
// Description: initialize lm73. Includes initializing twi hardware.
//*****************************************************************************
void init_lm73( );

//*****************************************************************************
// Name: init_lm73                                   
// Description: returns TRUE if reading out of bounds temperature.
//*****************************************************************************
uint8_t check_lm73_alert( );

//*****************************************************************************
// Name: get_lm73_temp                                   
// Description: get lm73 data
//*****************************************************************************
struct lm73_data get_lm73_temp( );
  
