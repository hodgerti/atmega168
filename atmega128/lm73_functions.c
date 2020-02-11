// lm73_functions.c       
// Roger Traylor 11.28.10

#include <util/twi.h>
#include "lm73_functions.h"
#include <util/delay.h>

uint8_t lm73_wr_buf[ 3 ];
uint8_t lm73_rd_buf[ 3 ];

void init_lm73( )
{
	// set ADDR and enable NALERT
	DDRF  |= (1<<LM73_ADDR_PIN);
	PORTF |= (1<<LM73_ADDR_PIN);
	DDRF  &= ~(1<<LM73_N_ALERT_PIN);
	PORTF &= ~(1<<LM73_N_ALERT_PIN);
	_delay_us( 50 );
	
	// init twi hardware
	init_twi( );
	
	// start temperature mode
	twi_start_wr( LM73_ADDRESS, LM73_PTR_TEMP, 1 );
	/*
	// send configuration
	lm73_wr_buf[ 0 ] = LM73_PTR_CONFIG;
	lm73_wr_buf[ 1 ] = LM73_CONFIG_VALUE;
	twi_start_wr( LM73_ADDRESS, lm73_wr_buf, 2 );
	// send status
	lm73_wr_buf[ 0 ] = LM73_PTR_CTRL_STATUS;
	lm73_wr_buf[ 1 ] = LM73_STATUS_VALUE;
	twi_start_wr( LM73_ADDRESS, lm73_wr_buf, 2 );
	// send temperature upper limit
	lm73_wr_buf[ 0 ] = LM73_UP_LIMIT;
	lm73_wr_buf[ 1 ] = (uint8_t)(LM73_100_C>>8);
	lm73_wr_buf[ 2 ] = (uint8_t)(LM73_100_C);
	twi_start_wr( LM73_ADDRESS, lm73_wr_buf, 3 );
	// send temperature lower limit
	lm73_wr_buf[ 0 ] = LM73_LOW_LIMIT;
	lm73_wr_buf[ 1 ] = (uint8_t)(LM73_0_C>>8);
	lm73_wr_buf[ 2 ] = (uint8_t)(LM73_0_C & 8);
	twi_start_wr( LM73_ADDRESS, lm73_wr_buf, 3 );
	*/
}  //  init_lm73  /

uint8_t check_lm73_alert( )
{
	if( PORTF | (1<<LM73_N_ALERT_PIN) ) return 0;
	else return 1;
}  //  check_lm73_alert  /

struct lm73_data get_lm73_temp( )
{	
	// get raw data
	twi_start_rd( LM73_ADDRESS, lm73_rd_buf, 2 );
	uint16_t raw_result = (lm73_rd_buf[ 0 ]<<8)|(lm73_rd_buf[ 1 ]);
	// format data
	uint8_t integer_value = (uint8_t)((raw_result>>7)&0xFF);
	uint16_t decimal_value = ((raw_result>>2)&0b11111)*10000/0b11111;
	int8_t sign_value;
	if( raw_result&0xF000 ) sign_value = -1;
	else sign_value = 1;
	// apply data
	struct lm73_data result = { sign_value, integer_value, decimal_value };
	return result;
}  //  lm73_temp_convert  /
