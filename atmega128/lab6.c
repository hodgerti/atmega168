/*============================================================
* Filename: lab6.c
* Author: Isaac Connor Hodgert
* Date: 12-9-2019
* Description: Code for ECE 473 Lab 6
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
#include "songs.h"
#include "hd44780.h"
#include "lm73_functions.h"
#include "twi_master.h"
#include "uart_functions.h"
#include "si4734.h"

//******************************************* 
// Defines
//*******************************************
#ifndef F_CPU
	#define F_CPU 16000000 // cpu speed in hertz 
#endif
#define EXT_F_CLK 32768  // 32khz
#ifndef TRUE
	#define TRUE 1
#endif
#ifndef FALSE
	#define FALSE 0
#endif

//******************************************* 
// Types
//*******************************************
struct time
{
	  uint8_t   tick;
	  uint16_t  ms;
	  uint8_t   s;
	  uint8_t   m;
	  uint8_t   h;
};

struct lcd_data
{
	char top_row[NUM_LCD_CHARS];
	char bot_row[NUM_LCD_CHARS];
	uint8_t cursor_pos;
};

//******************************************* 
// Globals
//*******************************************
// pin naming
#define PHOTOCELL_PIN		PF7
#define SEG_ENABLE_PIN 		PC7
#define SEG_BRIGHTNESS_PIN	PB7
#define VOLUME_PIN			PD7
#define SPEAKER_L_PIN		PE3
#define SPEAKER_R_PIN		PE4
#define RADIO_INT_PIN		PE7
#define RADIO_N_RST_PIN		PE2
#define ENCODER_PIN			PD2

// buttons
enum { APPLY_FM_FREQ_BUTTON, TOGGLE_LCD_DISPLAY_MODE_BUTTON, UNUSED_BUTTON, ALARM_ARM_BUTTON, TIME_SET_BUTTON, ALARM_SET_BUTTON, SNOOZE_BUTTON, MUTE_BUTTON, BUTTON_COUNT };
uint16_t button_counts_A[ BUTTON_COUNT ];
uint8_t buttons_clicked_A;
uint8_t buttons_pressed_A;
uint8_t buttons_toggled_A;

// music
#define DEFAULT_VOLUME				  25
#define ROUNDABOUT_LENGTH			  52
uint16_t time_step					= 120;
uint16_t note_count					= 0;
uint16_t note_idx					= ROUNDABOUT_LENGTH;
const uint16_t ROUNDABOUT[ROUNDABOUT_LENGTH] = {E3, E3,E3,NA, G3, G3,G3,NA,NA,NA,NA,NA,NA,NA,NA,NA, B3, B3,B3,NA, A3,B3,A3,G3,F1, F1,F1,NA, A3, A3,A3,NA, E3, E3,E3,NA, G3, G3,G3,G3,NA,NA,NA,NA,NA,NA,NA,NA, B3, B3,NA,NA };

// lcd display modes
enum { LCD_DISPLAY_ALARM_AND_FREQUENCY, LCD_DISPLAY_TEMPS, LCD_DISPLAY_MODE_COUNT };

// states
uint8_t time_set_active = FALSE;
uint8_t alarm_set_active = FALSE;
uint8_t alarm_armed = FALSE;
uint8_t alarm_triggered = FALSE;
uint8_t mute_active = TRUE;
uint8_t snooze_active = FALSE;
uint8_t lcd_display_mode = LCD_DISPLAY_TEMPS;

// global variables
uint8_t encoder_data;
int8_t enc_0_dir	= 0x00;
int8_t enc_1_dir	= 0x00;
struct time default_time;
struct time alarm_time;
struct time snooze_time;
uint8_t volume;
struct lcd_data lcd_data_handle;
uint8_t brightness;
uint8_t default_time_changed;

// segment
enum { NUMSEG_0, NUMSEG_1, NUMSEG_2, NUMSEG_3, NUMSEG_4, NUMSEG_5, NUMSEG_6, NUMSEG_7, NUMSEG_8, NUMSEG_9, NUMSEG_A, NUMSEG_B, NUMSEG_C, NUMSEG_D, NUMSEG_E, NUMSEG_F, NUMSEG_COLON, NUMSEG_INDC, NUMSEG_UP_COLON, NUMSEG_DOWN_COLON, NUMSEG_NONE, NUMSEG_COUNT };
enum { DIGIT_1, DIGIT_2, DIGIT_3, DIGIT_4, DIGIT_LP, DIGIT_PUSH, DIGIT_NONE, DIGIT_COUNT };
uint8_t segment_data[5];
uint8_t segment_dictionary[NUMSEG_COUNT];
uint8_t digit_dictionary[DIGIT_COUNT];

// temperature
#define LM73_CHECK_PERIOD_MS 500
#define MAX2323_CHECK_PERIOD_MS 50
#define EXTERNAL_TEMP_PROTOCOL 0
#define UART_SYNC_CHAR		0xAA
#define UART_GET_TEMP		0xF0
#define UART_GET_TEMP_ACK	0xE0
enum { UART_TEMP_TYPE };
#define UART_BUFF_SIZE 16
char uart_buff[ UART_BUFF_SIZE ];
struct lm73_data internal_temp;
#define EXTERNAL_TEMP_STRING_SIZE 6
char external_temp_string[ EXTERNAL_TEMP_STRING_SIZE ];
uint8_t external_temp_rcv_rdy = FALSE;

// radio
#define DEFAULT_FM_FREQ 10630
#define MIN_FM_FREQ 8810
#define MAX_FM_FREQ 10790
uint8_t radio_on;

uint16_t current_fm_freq;
uint16_t current_am_freq;
uint16_t current_sw_freq;
uint8_t  current_volume;

enum radio_band{FM, AM, SW};
volatile enum radio_band current_radio_band;

volatile uint8_t STC_interrupt;  // flag bit to indicate tune or seek is done

//******************************************* 
// Functions
//*******************************************
// buttons PORT A
void init_buttons_A( );
void get_buttons_A( );
void handle_buttons_A( );
// segments
void init_segments( );
void clock_to_segments( );
void set_segments( );
// time
void init_5ms_clock( );
void handle_5ms_clock( );
void consume_5ms_clock( );
// alarm
void init_alarm_time( );
void check_alarm( );
void init_snooze( );
void snooze( );
// lcd
void my_lcd_init( );
void activate_lcd_send( );
void send_to_lcd( );
void set_lcd_alarm( );
void set_lcd_volume( );
void set_lcd_frequency( );
void set_lcd_internal_temp( );
void set_lcd_external_temp( );
void toggle_lcd_display_mode( );
// spi
void init_spi( );
uint8_t get_encoder_set_bargraph( uint8_t );
int8_t get_encoder_direction( uint8_t, uint8_t );
void apply_encoders( int8_t, int8_t );
// volume
void init_volume( );
void set_volume( );
// alarm music
void init_alarm_music( );
void alarm_music_engine( );
void start_alarm_music( );
void stop_alarm_music( );
void play_note( uint16_t );
// ADC
void init_photocell( );
void find_brightness( );
void init_brightness( );
void set_brightness( );
void read_brightness( );
// temperature
void apply_lm73( );
void apply_max2323( );
// radio
void init_radio( );
void turn_on_fm_radio( );
void turn_off_fm_radio( );
void radio_reset( );
void apply_radio_fm_freq( );
// utility
uint8_t bitmarch( uint8_t );

//*****************************************************************************
// Name: Timer0 compare interrupt                                    
// Description: Use this to adjust the time, get raw buttons, and set segments
//*****************************************************************************
ISR( TIMER0_COMP_vect )
{
  	handle_5ms_clock( );
	get_buttons_A( );
}

//******************************************************************************
// Name: ADC conversion complete interrupt                                   
// Description: find and set brightness
//*****************************************************************************
ISR( ADC_vect )
{
	find_brightness( );
	set_brightness( );
	ADCSRA &= ~(1<<ADSC);
}  //  ADC_vect  /

//*****************************************************************************
// Name: Timer1 overflow/compare A interrupt                                    
// Description: Use these to toggle volume bit
//*****************************************************************************
ISR( TIMER1_OVF_vect )		{ PORTD ^= (1<<VOLUME_PIN); }
ISR( TIMER1_COMPA_vect )	{ PORTD ^= (1<<VOLUME_PIN); }

//*****************************************************************************
// Name: INT 6 interrupt vector                                  
// Description: This indicates that GPO2 on the radio board had a rising edge
//				Indicates radio has finished changing.
//*****************************************************************************
ISR( INT7_vect )
{
	STC_interrupt = TRUE;
}  //  INT6_vect  /

//*****************************************************************************
// Name: USART                          
// Description: 
//*****************************************************************************
ISR( USART0_RX_vect )
{
	static uint8_t rdx;
	
  	char rx_char = UDR0;              				//get character
	if( rdx < EXTERNAL_TEMP_STRING_SIZE )
  		external_temp_string[ rdx++ ] = rx_char;  		//store in array 
 	//if entire string has arrived, set flag, reset index
  	if( rx_char == '\0' )
	{
		external_temp_rcv_rdy = TRUE;
		rdx = 0;
  	}
}

//*******************************************************
// Name: main                                    
// Description: this is the main processing loop
//*******************************************************
int main( )
{
	// init
	sei( );
	init_buttons_A( );
	init_5ms_clock( );
	init_alarm_time( );
	init_snooze( );
	init_spi( );
	my_lcd_init( );
	init_volume( );
	init_photocell( );
	init_brightness( );
	init_segments( );
	init_alarm_music( );
	init_lm73( );
	uart_init( );
	init_twi( );
	init_radio( );
	
	while( TRUE )
	{		
		_delay_us( 50 );
		
		uint8_t volume_rep = bitmarch( 8 * volume / 100 );
		encoder_data = get_encoder_set_bargraph( volume_rep );
		enc_0_dir = get_encoder_direction( 0, encoder_data );
		enc_1_dir = get_encoder_direction( 1, encoder_data );
		apply_encoders( enc_0_dir, enc_1_dir );
		send_to_lcd( );
		read_brightness( );
		check_alarm( );
		handle_buttons_A( );
		alarm_music_engine( );
		clock_to_segments( );
		set_segments( );
		
		consume_5ms_clock( );
	}
}

//*****************************************************************************
// Name: init_buttons_A
// Description: init buttons on PORTA
//*****************************************************************************
void init_buttons_A( )
{
	buttons_pressed_A = 0x00;
	buttons_clicked_A = 0x00;
	buttons_toggled_A = 0x00;
	for( uint8_t bdx = 0; bdx < BUTTON_COUNT; bdx++ )
	{
	  	button_counts_A[bdx] = 0;
	}
}  //  init_buttons_A  /

//*****************************************************************************
// Name: get_buttons_A
// Description: get input data from PORT A push buttons. Handles debouncing.
//*****************************************************************************
void get_buttons_A( )
{		
	DDRA = 0x00;
	PORTA = 0xFF;
	
	PORTB = digit_dictionary[DIGIT_PUSH] | ( PORTB & 0x0F );
	// check each pin
	uint8_t pdx;
	for( pdx = 0; pdx < BUTTON_COUNT; pdx++ )
	{
		// feed in 1's until button is pushed
    	button_counts_A[pdx] = (button_counts_A[pdx] << 1) | (! bit_is_clear(PINA, pdx)) | 0xE000;
    	if( button_counts_A[pdx] == 0xF000 ) buttons_clicked_A |= (1 << pdx);
		else                                 buttons_clicked_A &= ~(1 << pdx);
		if( button_counts_A[pdx] == 0xE000 ) buttons_pressed_A |= (1 << pdx);
		else                                 buttons_pressed_A &= ~(1 << pdx);
	}
  	buttons_toggled_A ^= buttons_clicked_A;
  	
	PORTA = 0xFF;
	DDRA = 0xFF;
} //  get_buttons_A  /

//*****************************************************************************
// Name: handle_buttons_A
// Description: handle state changes
//*****************************************************************************
void handle_buttons_A( )
{
	alarm_set_active = buttons_toggled_A & (1<<ALARM_SET_BUTTON);
	time_set_active = buttons_toggled_A & (1<<TIME_SET_BUTTON);
	alarm_armed = buttons_toggled_A & (1<<ALARM_ARM_BUTTON);
	if( default_time_changed && ( buttons_clicked_A & (1<<ALARM_ARM_BUTTON) ) )
	{
		lcd_display_mode = LCD_DISPLAY_ALARM_AND_FREQUENCY;
		set_lcd_alarm( );
		set_lcd_frequency( );
	}
	if( default_time_changed && ( buttons_clicked_A & (1<<MUTE_BUTTON) ) )
	{
		snooze_active = FALSE;
		alarm_triggered = FALSE;
		stop_alarm_music( );
	}
	if( default_time_changed && ( buttons_clicked_A & (1<<SNOOZE_BUTTON) ) )
	{
		snooze( );
		alarm_triggered = FALSE;
		stop_alarm_music( );
	}
	if( default_time_changed && ( default_time.ms%LM73_CHECK_PERIOD_MS == 0 ) )
	{
		apply_lm73( );
	}
	if( default_time.ms%MAX2323_CHECK_PERIOD_MS == 0 )
	{
		apply_max2323( );
	}
	if( default_time_changed && ( buttons_clicked_A & (1<<TOGGLE_LCD_DISPLAY_MODE_BUTTON) ) )
	{
		toggle_lcd_display_mode( );
	}
	if( default_time_changed && ( buttons_clicked_A & ( 1<<APPLY_FM_FREQ_BUTTON ) ) )
	{
		if( radio_on )
		{
			turn_off_fm_radio( );
		}
		else
		{
			snooze_active = FALSE;
			alarm_triggered = FALSE;
			stop_alarm_music( );
			apply_radio_fm_freq( );
		}
	}
}  //  handle_buttons_A  /

//*****************************************************************************
// Name: init_segments                                    
// Description: initialize segment variables and ports
//*****************************************************************************
void init_segments( )
{
	// initialize segment look ups per possible digit displayed
	segment_dictionary[NUMSEG_0]  = ~( (1<<PORT0)|(1<<PORT1)|(1<<PORT2)|(1<<PORT3)|(1<<PORT4)|(1<<PORT5) );
	segment_dictionary[NUMSEG_1]  = ~( (1<<PORT1)|(1<<PORT2) );
	segment_dictionary[NUMSEG_2]  = ~( (1<<PORT0)|(1<<PORT1)|(1<<PORT6)|(1<<PORT4)|(1<<PORT3) );
	segment_dictionary[NUMSEG_3]  = ~( (1<<PORT0)|(1<<PORT1)|(1<<PORT6)|(1<<PORT2)|(1<<PORT3) );
	segment_dictionary[NUMSEG_4]  = ~( (1<<PORT5)|(1<<PORT6)|(1<<PORT1)|(1<<PORT2) );
	segment_dictionary[NUMSEG_5]  = ~( (1<<PORT0)|(1<<PORT5)|(1<<PORT6)|(1<<PORT2)|(1<<PORT3) );
	segment_dictionary[NUMSEG_6]  = ~( (1<<PORT0)|(1<<PORT5)|(1<<PORT6)|(1<<PORT4)|(1<<PORT2)|(1<<PORT3) );
	segment_dictionary[NUMSEG_7]  = ~( (1<<PORT0)|(1<<PORT1)|(1<<PORT2) );
	segment_dictionary[NUMSEG_8]  = ~( (1<<PORT0)|(1<<PORT5)|(1<<PORT1)|(1<<PORT6)|(1<<PORT4)|(1<<PORT2)|(1<<PORT3) );
	segment_dictionary[NUMSEG_9]  = ~( (1<<PORT0)|(1<<PORT5)|(1<<PORT1)|(1<<PORT6)|(1<<PORT2)|(1<<PORT3) );
	segment_dictionary[NUMSEG_A]  = ~( (1<<PORT0)|(1<<PORT1)|(1<<PORT2)|(1<<PORT4)|(1<<PORT5)|(1<<PORT6) );
	segment_dictionary[NUMSEG_B]  = ~( (1<<PORT2)|(1<<PORT3)|(1<<PORT4)|(1<<PORT5)|(1<<PORT6) );
	segment_dictionary[NUMSEG_C]  = ~( (1<<PORT0)|(1<<PORT3)|(1<<PORT4)|(1<<PORT5) );
	segment_dictionary[NUMSEG_D]  = ~( (1<<PORT1)|(1<<PORT2)|(1<<PORT3)|(1<<PORT4)|(1<<PORT6) );
	segment_dictionary[NUMSEG_E]  = ~( (1<<PORT0)|(1<<PORT3)|(1<<PORT4)|(1<<PORT5)|(1<<PORT6) );
	segment_dictionary[NUMSEG_F]  = ~( (1<<PORT0)|(1<<PORT4)|(1<<PORT5)|(1<<PORT6) );
	segment_dictionary[NUMSEG_COLON]       = ~( (1<<PORT0)|(1<<PORT1) );
	segment_dictionary[NUMSEG_INDC]        = ~( (1<<PORT2) );
	segment_dictionary[NUMSEG_UP_COLON]    = ~( (1<<PORT1) );
	segment_dictionary[NUMSEG_DOWN_COLON]  = ~( (1<<PORT0) );
	segment_dictionary[NUMSEG_NONE]  = 0xFF;

	// initialize digit look ups
	digit_dictionary[DIGIT_1]     = (1<<PORT6);
	digit_dictionary[DIGIT_2]     = (1<<PORT5)|(1<<PORT4);
	digit_dictionary[DIGIT_3]     = (1<<PORT4);
	digit_dictionary[DIGIT_4]     = 0x00;
	digit_dictionary[DIGIT_LP]    = (1<<PORT5);
	digit_dictionary[DIGIT_PUSH]  = (1<<PORT6)|(1<<PORT5)|(1<<PORT4);
	digit_dictionary[DIGIT_NONE]  = (1<<PORT7);

	// initialize the segment data to starting values
	segment_data[DIGIT_1]   = NUMSEG_0;
	segment_data[DIGIT_2]   = NUMSEG_NONE;
	segment_data[DIGIT_3]   = NUMSEG_NONE;
	segment_data[DIGIT_4]   = NUMSEG_NONE;
	segment_data[DIGIT_LP]  = NUMSEG_NONE;

	//set port bits 4-7 B as outputs
	DDRB |= (1<<PORT4)|(1<<PORT5)|(1<<PORT6)|(1<<PORT7);
	// set to outputs
	PORTA = 0xFF;
	DDRA = 0xFF;
	// enable seg
	DDRC  |= (1<<SEG_ENABLE_PIN);
	PORTC |= (1<<SEG_ENABLE_PIN);
} //  init_segments  /

//******************************************************************************
// Name: clock_to_segments                                    
// Description: computes segment data using a given time
//*****************************************************************************
void clock_to_segments( )
{   
	static struct time this_time;
	
	if( alarm_set_active )
	{
		this_time = alarm_time;
		segment_data[DIGIT_LP] = NUMSEG_INDC;
	}
	else
	{
		this_time = default_time;	
	}
	segment_data[DIGIT_4] = this_time.m % 10;
	segment_data[DIGIT_3] = this_time.m / 10;
	segment_data[DIGIT_2] = this_time.h % 10;
	segment_data[DIGIT_1] = this_time.h / 10;
} //  clock_to_segments  /

//*****************************************************************************
// Name: set_segments
// Description: sets the 7 segment display
//*****************************************************************************
void set_segments( )
{
	static uint8_t digit = 0;
	
	digit %= 5;
	
	PORTB = digit_dictionary[DIGIT_NONE] | ( PORTB & 0x0F );	
	PORTA = segment_dictionary[ segment_data[ digit ] ];
	PORTB = digit_dictionary[ digit ] | ( PORTB & 0x0F );

	digit++;
} //  set_segments  /

//*****************************************************************************
// Name: init_5ms_clock
// Description: Set timer to CTC, 5ms period. 
//              This clock adjusts itself to count in perfect 5ms increments.
// OCR0 = (32MHz/1)*(5ms) - 1
// OCR0 = 162.84 = 162 21/25
#define CLOCK_OCR0_DEFAULT      162
#define CLOCK_ADJUST_AMOUNT     21
#define CLOCK_ADJUST_FREQUENCY  25
// tick-to-tick error = 0.09%
// long term error = 0.0%
//*****************************************************************************
void init_5ms_clock( )
{
	// variables
	default_time_changed = TRUE;
	default_time.tick  = 0;
	default_time.ms    = 0;
	default_time.s     = 0;
	default_time.m     = 0;
	default_time.h     = 12;
	// functionality
	ASSR  |= (1<<AS0);  // enable external oscillator
	TCCR0 |= (1<<WGM01); TCCR0 &= ~(1<<WGM00);  // set to CTC mode
	TIMSK |= (1<<OCIE0);  // set compare match interrupt
	TCCR0 |= (0<<CS02)|(0<<CS01)|(1<<CS00);  // set prescale to 1 and enable timer0
	OCR0 = CLOCK_OCR0_DEFAULT;
} //  init_5ms_clock  /

//*****************************************************************************
// Name: handle_5ms_clock
// Description: Adjust clock and handle events
//*****************************************************************************
void handle_5ms_clock( )
{
	// bound default time
	default_time.tick %= CLOCK_ADJUST_FREQUENCY;
	default_time.ms %= 1000;
	default_time.s %= 60;
	default_time.m %= 60;
	default_time.h %= 24;
	
	// increment default time
	default_time.tick++;
	default_time.ms += 5;
	default_time.s  += default_time.ms/1000;
	default_time.m  += default_time.s/60;
	default_time.h  += default_time.m/60;
	
	// adjust default time
	if( default_time.tick == 0) OCR0 = CLOCK_OCR0_DEFAULT - CLOCK_ADJUST_AMOUNT;
	else OCR0 = CLOCK_OCR0_DEFAULT;
	
	if( time_set_active )
	{
		// blink colon
		if( default_time.s % 2 ) segment_data[DIGIT_LP] = NUMSEG_INDC;
		else                     segment_data[DIGIT_LP] = NUMSEG_NONE;
	}
	else
	{
		// blink colon
		if( default_time.s % 2 ) segment_data[DIGIT_LP] = NUMSEG_COLON;
		else                     segment_data[DIGIT_LP] = NUMSEG_NONE;
	}

	default_time_changed = TRUE;
}  //  handle_5ms_clock  /

//*****************************************************************************
// Name: consume_5ms_clock
// Description: remove clock tick
//*****************************************************************************
void consume_5ms_clock( )
{
	default_time_changed = FALSE;	
}  //  consume_5ms_clock  /

//*****************************************************************************
// Name: init_alarm_time
// Description: init alarm time
//*****************************************************************************
void init_alarm_time( )
{
	alarm_time.tick = 0;
	alarm_time.ms   = 0;
	alarm_time.s    = 0;
	alarm_time.m    = 0;
	alarm_time.h    = 16;	
}  //  init_alarm_time  /

//*****************************************************************************
// Name: check_alarm
// Description: check the alam and activate events
//*****************************************************************************
void check_alarm( )
{	 
	if( snooze_active &&
	    ( (snooze_time.h==default_time.h) && (snooze_time.m==default_time.m) && (snooze_time.s==default_time.s)) )
	{
		alarm_triggered = TRUE;
		start_alarm_music( );
		if( radio_on ) turn_off_fm_radio( );
	}
	else if( alarm_armed &&
		( (alarm_time.h==default_time.h) && (alarm_time.m==default_time.m) && (alarm_time.s==default_time.s) ) )
	{
		alarm_triggered = TRUE;
		start_alarm_music( );
		if( radio_on ) turn_off_fm_radio( );
	}
}  //  check_alarm  /

//*****************************************************************************
// Name: init_snooze
// Description: init snooze
//*****************************************************************************
void init_snooze( )
{
	snooze_time.ms = 0;
	snooze_time.s = 0;
	snooze_time.m = 0;
	snooze_time.h = 0;
}  //  init_snooze  /

//*****************************************************************************
// Name: snooze
// Description: snooze the alarm
//*****************************************************************************
void snooze( )
{
	snooze_active = TRUE;
	
	snooze_time = default_time;
	snooze_time.s += 10;
	snooze_time.m += snooze_time.s/60;
	snooze_time.h += snooze_time.m/60;
	snooze_time.s %= 60;
	snooze_time.m %= 60;
	snooze_time.h %= 24;
}  //  snooze  /

//*****************************************************************************
// Name: my_lcd_init
// Description: init lcd
//*****************************************************************************
void my_lcd_init( )
{
	strncpy( lcd_data_handle.top_row, "                ", 16 );
	strncpy( lcd_data_handle.bot_row, "                ", 16 );
	lcd_data_handle.cursor_pos = 0;
		
	lcd_init( );
	
	if( lcd_display_mode == LCD_DISPLAY_ALARM_AND_FREQUENCY )
	{
		set_lcd_alarm( );
		set_lcd_frequency( );
	}
	else if( lcd_display_mode != LCD_DISPLAY_TEMPS )
	{
		set_lcd_internal_temp( );
		set_lcd_external_temp( );
	}
}  //  my_lcd_init  /

//*****************************************************************************
// Name: activate_lcd_send
// Description: activate lcd send
//*****************************************************************************
void activate_lcd_send( )
{
	lcd_data_handle.cursor_pos = 0;
}  //  activate_lcd_send  /

//*****************************************************************************
// Name: send_to_lcd
// Description: sends characters to LCD if applicable
//*****************************************************************************
void send_to_lcd( )
{
	if( lcd_data_handle.cursor_pos == 32 ) return;
	
	if( lcd_data_handle.cursor_pos == 0 )
		line1_col1( );
	else if( lcd_data_handle.cursor_pos == 16 )
		line2_col1( );
	
	if( lcd_data_handle.cursor_pos < 16 ) 
		char2lcd( lcd_data_handle.top_row[ lcd_data_handle.cursor_pos ] );
	else
		char2lcd( lcd_data_handle.bot_row[ lcd_data_handle.cursor_pos - 16 ] );
	
	lcd_data_handle.cursor_pos++;
	
}  //  send_to_lcd  /

//*****************************************************************************
// Name: set_lcd_alarm
// Description: set LCD top row to alarm
//*****************************************************************************
void set_lcd_alarm( )
{	
	if( alarm_armed )
	{
		strncpy( lcd_data_handle.top_row, "ALARM: ARMED    ", 16 );
	}
	else
	{
		strncpy( lcd_data_handle.top_row, "ALARM: NOT ARMED", 16 );
	}
	activate_lcd_send( );
}  //  set_lcd_alarm  /

//*****************************************************************************
// Name: set_lcd_volume
// Description: set top row as volume
//*****************************************************************************
void set_lcd_volume( )
{
	set_lcd_frequency( );
}  //  set_lcd_volume  /

//*****************************************************************************
// Name: set_lcd_frequency
// Description: set bot row as radio frequency
//*****************************************************************************
void set_lcd_frequency( )
{	
	char int_buff[5];
	char dec_buff[2];
	char vol_buff[3];
	itoa( current_fm_freq/100, int_buff, 10 );
	itoa( current_fm_freq%100, dec_buff, 10 );
	itoa( volume, vol_buff, 10 );
	uint8_t cnt = sprintf( lcd_data_handle.bot_row, "FM:%s.%s VOL:%s", int_buff, dec_buff, vol_buff );
	for( ; cnt < 16; cnt++ ) lcd_data_handle.bot_row[cnt] = ' ';
	activate_lcd_send( );
}

//*****************************************************************************
// Name: set_lcd_internal_temp
// Description: set LCD bot row to internal temp
//*****************************************************************************
void set_lcd_internal_temp( )
{	
	char int_buffer[3];
	char dec_buffer[5];
	itoa( internal_temp.integer_value, int_buffer, 10 );
	itoa( internal_temp.decimal_value, dec_buffer, 10 );
	uint8_t cnt = sprintf( lcd_data_handle.bot_row, "IT: %c%s.%s'c", internal_temp.sign == -1 ? '-' : ' ', int_buffer, dec_buffer );
	for( ; cnt < 16; cnt++ ) lcd_data_handle.bot_row[cnt] = ' ';
	activate_lcd_send( );
}  //  set_lcd_internal_temp  /
	

//*****************************************************************************
// Name: set_lcd_external_temp
// Description: set LCD top row to external temp
//*****************************************************************************
void set_lcd_external_temp( )
{
	uint8_t cnt = sprintf( lcd_data_handle.top_row, "OT: %.5s'c", external_temp_string );
	for( ; cnt < 16; cnt++ ) lcd_data_handle.top_row[cnt] = ' ';
	activate_lcd_send( );
}  //  set_lcd_external_temp  /

void toggle_lcd_display_mode( )
{
	lcd_display_mode++;
	lcd_display_mode %= LCD_DISPLAY_MODE_COUNT;
	if( lcd_display_mode == LCD_DISPLAY_ALARM_AND_FREQUENCY )
	{
		set_lcd_alarm( );
		set_lcd_frequency( );
	}
	else if( lcd_display_mode == LCD_DISPLAY_TEMPS )
	{
		set_lcd_internal_temp( );
		set_lcd_external_temp( );
	}		
}

//*****************************************************************************
// Name: init_spi
// Description: init spi for use with the bar graph and encoders
//*****************************************************************************
void init_spi( )
{	
	/* Run this code before attempting to write to the LCD.*/
	DDRF  |= 0x08;  // port F bit 3 is enable for LCD
	PORTF &= 0xF7;  // port F bit 3 is initially low
	
  	DDRB  |=   (1<<PB0)|(1<<PB1)|(1<<PB2)|(1<<PB3); // Turn on SS, SCLK, MOSI, MISO
  	SPCR  |=   (1<<SPE)|(1<<MSTR); // enable SPI, master mode 
  	SPSR  |=   (1<<SPI2X); // double speed operation
}  // init_spi  /

//*****************************************************************************
// Name: get_encoder_and_bargraph
// Description: set/get data using SPI
//*****************************************************************************
uint8_t get_encoder_set_bargraph( uint8_t in_data )
{
	// activate encoder shift in
	PORTB &= ~(1<<PB0) | 0xF0;
	PORTB |= (1<<PB0);
	// send data
	SPDR = in_data;
	while( bit_is_clear(SPSR, SPIF) ) { }
	// get data from encoder
	uint8_t spi_data = SPDR;
	// activate bar graph shift in
	PORTB |= (1<<PB0);
	PORTB &= ~(1<<PB0) | 0xF0;
	return spi_data;
} // get_encoder_set_bargraph /

//******************************************************************************
// Name: get_encoder_direction
// Description: This function processes up to 4 encoder datas and returns turn direction
//              enc = {0:3}
//              raw_data = 0b enc3:1 enc3:0  enc2:1 enc2:0  enc1:1 enc1:0  enc0:1 enc0:0
//              return {-1,0,1}
//*****************************************************************************
int8_t get_encoder_direction( uint8_t enc, uint8_t raw_data )
{
	static uint8_t last_data_enc_n      = 0x00;
	static uint8_t last_direction_enc_n = 0x00;
	// result
	int8_t result = 0;
	uint8_t shft = 2*enc;
	
	// find current ones
	uint8_t current_data = (raw_data>>shft)&(0b11);
	uint8_t last_current_data = (last_data_enc_n>>shft)&(0b11);
	
	// find direction
	if( last_current_data != current_data )
	{
	  	// get directions
	  	int8_t current_direction = -1;
	  	int8_t last_current_direction = (last_direction_enc_n>>shft)&(0b11);
	  	last_current_direction |= ~(0b11)*((last_current_direction&0b10)>>1);  // sign extend
	  	// process directions
	  	switch (last_current_data)
	  	{
		case 0:
			if( current_data == 1 ) current_direction = 1;
			break;
		case 1:
			if( current_data == 3 ) current_direction = 1;
			break;
		case 2:
			if( current_data == 0 ) current_direction = 1;
			break;
		case 3:
			if( current_data == 2 ) current_direction = 1;
			break;
	  	}
	  	// apply direction
	  	if( current_data == 0 && current_direction == last_current_direction ) result = current_direction;
	  	// set last direction
	  	last_direction_enc_n &= ~(0b11<<shft);  // clear bits
	  	last_direction_enc_n |= (current_direction&0b11)<<shft;  // set bits
	}
	// set last data
	last_data_enc_n &= ~(0b11<<shft);  // clear bits
	last_data_enc_n |= current_data<<shft;   // set bits
	
	return result;
}  //  get_encoder_direction  /

//******************************************************************************
// Name: apply_encoders
// Description: apply encoders
//*****************************************************************************
void apply_encoders( int8_t enc_0_dir, int8_t enc_1_dir )
{
	// escape if encoder did not do anything
	if( enc_0_dir == 0 && enc_1_dir == 0 ) return;
	
	// adjust alarm time
	if( alarm_set_active )
	{
		// hours
		if( alarm_time.h == 0 && enc_0_dir == -1 ) alarm_time.h = 23;
		else alarm_time.h += enc_0_dir;
		// minutes
		if( alarm_time.m == 0 && enc_1_dir == -1 ) alarm_time.m = 59;
	   	else alarm_time.m += enc_1_dir;
		// bound
		alarm_time.m %= 60;
		alarm_time.h %= 24;
	}
	// adjust clock time
	else if( time_set_active )
	{
		// hours
		if( default_time.h == 0 && enc_0_dir == -1 ) default_time.h = 23;
		else default_time.h += enc_0_dir;
		// minutes
		if( default_time.m == 0 && enc_1_dir == -1 ) default_time.m = 59;
	   	else default_time.m += enc_1_dir;
	}
	// adjust volume and FM frequency
	else
	{
		// adjust frequency with encoder 0 (the left one)
		if( enc_0_dir != 0 )
	    {
			current_fm_freq += enc_0_dir*20;
			// clamp
			if( current_fm_freq < MIN_FM_FREQ ) current_fm_freq = MIN_FM_FREQ;
			else if ( current_fm_freq > MAX_FM_FREQ ) current_fm_freq = MAX_FM_FREQ;
			apply_radio_fm_freq( );
	    }	
		// adjust volume with encoder 1 (the right one)
		if( enc_1_dir != 0 )
	    {
			volume += enc_1_dir;
			// clamp
			if( volume <= 0 ) volume = 1;
			else if ( volume >= 100 ) volume = 99;
			set_volume( volume );
	    }	
	}
}  //  apply_encoders  /

//*****************************************************************************
// Name: init_volume
// Description: use Timer1 to control speaker volume
//*****************************************************************************
void init_volume( )
{
	volume = DEFAULT_VOLUME;
	
	DDRD |= (1<<VOLUME_PIN);
	
	TCCR1A &= ~((1<<COM3A1)|(1<<COM3A0)|(1<<COM3B1)|(1<<COM3B0)|(1<<COM3C1)|(1<<COM3C0));  // disable OC3n's
	TCCR1B |= (1<<WGM32);  TCCR1A |= (1<<WGM30);  // set Fast PWM mode 8-bit (0x00FF TOP)	
	TIMSK  |= (1<<OCIE1A)|(1<<TOIE1);  // enable overflow and compare match intterupts
	TCCR1B &= ~(1<<ICNC1);	// disable noise canceling
	
	TCCR1B |= (1<<CS31)|(1<<CS30);  // start clock with 64 prescalar
	
	set_volume( );
}  //  init_volume  /

//*****************************************************************************
// Name: set_volume
// Description: set volume
//*****************************************************************************
void set_volume( )
{
	OCR1A = 0x00FF * volume / 100;
	set_lcd_volume( );
}  // set_volume  /

//*****************************************************************************
// Name: init_alarm_music
// Description: init music
//*****************************************************************************
void init_alarm_music( )
{
	// functionality
	DDRE |= (1<<SPEAKER_L_PIN)|(1<<SPEAKER_R_PIN);
	
	TCCR3A	|= (1<<COM3A0)|(1<<COM3B0);  // set toggle OC3A/OC3B
	TCCR3B |= (1<<WGM33)|(1<<WGM32);  TCCR3A |= (1<<WGM31)|(1<<WGM30);  // set Fast PWM mode with OCRnA top	
	TCCR3B &= ~(1<<ICNC3);	// disable noise canceling
	play_note( NA );
}  //  init_alarm_music  /
	
//*****************************************************************************
// Name: alarm_music_engine
// Description: handle music playing
//*****************************************************************************
void alarm_music_engine( )
{	
	if( mute_active )
	{
		play_note( NA );
	}
	else
	{
		if( note_idx >= ROUNDABOUT_LENGTH )
		{
			if( alarm_triggered )
			{
				start_alarm_music( );
			}
			else
			{
				stop_alarm_music( );	
			}
		}
		// play next note
		else if( default_time_changed && (default_time.ms%time_step) < 5 )
		{
			play_note( ROUNDABOUT[ note_idx ] );
			note_idx++;
		}	
	}
}  //  alarm_music_engine  /
		 
//*****************************************************************************
// Name: start_alarm_music
// Description: start music
//*****************************************************************************
void start_alarm_music( )
{
	turn_off_fm_radio( );
	mute_active = FALSE;
	note_idx = 0;
}  //  start_alarm_music  /

//*****************************************************************************
// Name: stop_alarm_music
// Description: stop music
//*****************************************************************************
void stop_alarm_music( )
{
	mute_active = TRUE;
	note_idx = ROUNDABOUT_LENGTH;
}  //  stop_alarm_music  /
		 
//*****************************************************************************
// Name: play_note
// Description: use Timer3 to set frequency of speakers
//*****************************************************************************
void play_note( uint16_t freq )
{
	static uint16_t last_freq = NA;
	
	if( freq != NA )
	{
		if( freq != last_freq )
		{
			OCR3A  = ( (F_CPU/(1024)) / (2*freq) ) - 1;  // left/right speaker
			TCCR3B |= (1<<CS32)|(1<<CS30);  // start clock with 1024 prescalar
		}
	}
	else
	{
		TCCR3B &= ~((1<<CS32)|(1<<CS31)|(1<<CS30));  // turn off clock
	}
	
	last_freq = freq;
}  //  play_note  /

//*****************************************************************************
// Name: init_photocell
// Description: init photocell
//*****************************************************************************
void init_photocell( )
{
	DDRF  &= ~(1<<PHOTOCELL_PIN);  // make input 
	PORTF &= ~(1<<PHOTOCELL_PIN);  // disable pullup
	ADMUX |= (1<<REFS0)|(0b00111);  // single-ended, ADC7, right adjusted, 10 bit resolution
	ADCSRA |= (1<<ADEN)|(1<<ADIE);  // with interrupt enabled
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // div 128
	
}  //  init_photocell  /

//*****************************************************************************
// Name: init_brightness
// Description: use Timer2
//*****************************************************************************
void init_brightness( )
{
	brightness = 50;
	
	DDRB |= (1<<SEG_BRIGHTNESS_PIN);
	
	TCCR2 |= (1<<COM21);  // toggle OC2
	TCCR2 |= (1<<WGM21)|(1<<WGM20);  // set Fast PWM mode
	TCCR2 |= (1<<CS21);  // start clock with 8 prescalar
}  //  init_brightness  /

//*****************************************************************************
// Name: find_brightness
// Description: get photocell brightness percent
//*****************************************************************************
void find_brightness( )
{
	brightness = 100 - ( 100 * ADC / 1024 );
}  //  find_brightness  /

//*****************************************************************************
// Name: set_brightness
// Description: use Timer2 to set segment brightness
//*****************************************************************************
void set_brightness( )
{
	OCR2 = 0xFF * brightness / 100;
}  //  set_brightness  /

//*****************************************************************************
// Name: read_brightness
// Description: poke ADC
//*****************************************************************************
void read_brightness( )
{
	ADCSRA |= (1<<ADSC);
}  //  read_brightness  /

//*****************************************************************************
// Name: apply_lm73
// Description: 
//*****************************************************************************
void apply_lm73( )
{
	internal_temp = get_lm73_temp( );
	if( lcd_display_mode == LCD_DISPLAY_TEMPS ) set_lcd_internal_temp( );
}  //  apply_lm73  /

//*****************************************************************************
// Name: apply_max2323
// Description: communicate with the external temperature sensor via a protocol
//*****************************************************************************
void apply_max2323( )
{
	#if ( EXTERNAL_TEMP_PROTOCOL )
	uart_putc( UART_SYNC_CHAR );
	char sync_char = uart_getc( );
	if( sync_char == UART_SYNC_CHAR )
	{
		uart_putc( UART_GET_TEMP );
		char temp_ack = uart_getc( );
		if( temp_ack == UART_GET_TEMP_ACK )
		{
			uint32_t raw = ((uint32_t)uart_getc( )<<24)|((uint32_t)uart_getc( )<<16)|((uint32_t)uart_getc( )<<8)|((uint32_t)uart_getc( ));
			external_temp = (float)raw;
		}
	}
	#else
	#endif
	
	if( external_temp_rcv_rdy && lcd_display_mode == LCD_DISPLAY_TEMPS )
	{
		external_temp_rcv_rdy = FALSE;
		set_lcd_external_temp( );
	}
}  //  apply_max2323  /

//*****************************************************************************
// Name: init_radio                                    
// Description: init radio
//*****************************************************************************
void init_radio( )
{
	// reset
	PORTE &= ~(1<<RADIO_N_RST_PIN);
	DDRE |= (1<<RADIO_N_RST_PIN); 
	
	// int
	PORTE &= ~(1<<RADIO_INT_PIN);
	DDRE &= ~(1<<RADIO_INT_PIN); 
	
	EICRB |= (1<<ISC71)|(1<<ISC70);  // rising edge
	EIMSK |= (1<<RADIO_INT_PIN);
	
	current_radio_band = FM;
	current_fm_freq = DEFAULT_FM_FREQ;
}  //  init_radio  /

//*****************************************************************************
// Name: turn_on_fm_radio                                    
// Description: turn on fm radio. Spam it baby.
//*****************************************************************************
void turn_on_fm_radio( )
{
	for( uint8_t idx = 0; idx < 2; idx++ )
	{
		radio_reset( );
		fm_pwr_up();  
	}
	radio_on = TRUE;
}  //  turn_on_fm_radio  /

//*****************************************************************************
// Name: turn_off_fm_radio                                    
// Description: turn off fm radio. Spam it baby.
//*****************************************************************************
void turn_off_fm_radio( )
{
	radio_pwr_dwn( );
	radio_reset( );
	radio_on = FALSE;
}  //  turn_off_fm_radio  /

//*****************************************************************************
// Name: radio_reset                                    
// Description: reset the radio
//*****************************************************************************
void radio_reset( )
{	
	DDRE  |= (1 << PE2); //Port E bit 2 is active high reset for radio 
	PORTE |= (1 << PE2); //radio reset is on at powerup (active high)

	//hardware reset of Si4734
	PORTE &= ~(1<<PE7); //int2 initially low to sense TWI mode
	DDRE  |= 0x80;      //turn on Port E bit 7 to drive it low
	PORTE |=  (1<<PE2); //hardware reset Si4734 
	_delay_us(200);     //hold for 200us, 100us by spec         
	PORTE &= ~(1<<PE2); //release reset 
	_delay_us(30);      //5us required because of my slow I2C translators I suspect
	//Si code in "low" has 30us delay...no explaination
	DDRE  &= ~(0x80);   //now Port E bit 7 becomes input from the radio interrupt
}  //  radio_reset  /

//*****************************************************************************
// Name: apply_radio_fm_freq                                    
// Description:
//*****************************************************************************
void apply_radio_fm_freq( )
{
	if( !radio_on )
	{
		turn_on_fm_radio( );
	}
	
	fm_tune_freq( );
	
	lcd_display_mode = LCD_DISPLAY_ALARM_AND_FREQUENCY;
	set_lcd_alarm( );
	set_lcd_frequency( );
}  //  apply_radio_fm_freq  /

//*****************************************************************************
// Name: bitmarch                                    
// Description: sets n number of sequential bits to 1
//*****************************************************************************
uint8_t bitmarch( uint8_t n_b )
{
  if( n_b == 8 ) return -((uint8_t)1);
  else           return (((uint8_t)1)<< n_b) - 1;
}  //  bitmarch  /