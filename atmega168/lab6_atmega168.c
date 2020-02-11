/*============================================================
* Filename: lab5_atmega168.c
* Author: Isaac Connor Hodgert
* Date: 12-8-2019
* Description: Code for ECE 473 Lab 5 where I run UART and TWI on the atmega168
*============================================================*/

//******************************************* 
// Includes
//*******************************************
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "twi_master.h"
#include "uart_functions_m168p.h"

//******************************************* 
// Defines
//*******************************************
#ifndef F_CPU
#define F_CPU 16000000 // cpu speed in hertz 
#endif
#define TRUE 1
#define FALSE 0
#define EXTERNAL_TEMP_PROTOCOL 0

// uart
#define UART_SYNC_CHAR		0xAA
#define UART_GET_TEMP		0xF0
#define UART_GET_TEMP_ACK	0xE0
enum { UART_TEMP_TYPE };
#define UART_BUFF_SIZE 16
char uart_buff[ UART_BUFF_SIZE ];

// temperature
#define SH21_ADDRESS			0x80
#define SH21_TEMP_HOLD			0xE3
#define SH21_HUM_HOLD			0xE5
#define SH21_TEMP_NO_HOLD      	0xF3
#define SH21_HUM_NO_HOLD 		0x0F

float temperature = -1.0f;
uint8_t wr_buff[ 3 ];
uint8_t rd_buff[ 3 ];

void get_sh21( )
{
	wr_buff[ 0 ] = SH21_TEMP_HOLD;
	twi_start_wr( SH21_ADDRESS, wr_buff, 1 );
	twi_start_rd( SH21_ADDRESS, rd_buff, 3 );
	uint16_t raw_result = (rd_buff[ 0 ]<<8)|(rd_buff[ 1 ]);
	// taken from section 6.2 in the SH21 data sheet:
	temperature = -46.85f + ( 175.72f * (float)raw_result / 65536.0f );
}

int main( )
{
	// init
	sei( );
	uart_init( );
	init_twi( );
	
	// main loop
	while( TRUE )
	{
		_delay_ms( 10 );
		
		get_sh21( );
		
		#if ( EXTERNAL_TEMP_PROTOCOL )
		char sync_char = uart_getc( );
		if( sync_char == UART_SYNC_CHAR )
		{
			uart_putc( UART_SYNC_CHAR );
			char get_cmd = uart_getc( );
			if( get_cmd == UART_GET_TEMP )
			{
				uart_putc( UART_GET_TEMP_ACK );
				uint32_t raw = (uint32_t)temperature;
				uart_putc( (char)(raw>>24) ); 
				uart_putc( (char)(raw>>16) );
				uart_putc( (char)(raw>>8) );
				uart_putc( (char)(raw) );
			}
		}
		#else
		char buff[6];
		dtostrf( temperature, 4, 1, buff );
		uart_puts( buff );
		#endif
	}
	
	return 0;
}