#define SYNTHESISER_H_

#include <stdint.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>        
#include <avr/sleep.h>
#include "i2cmaster.h"
#define F_CPU 16000000UL

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <math.h>

//Our Defines
#define AD5241 0b01011000 //our chip address
#define USART_BAUDRATE 19200
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) -1)
#define SAMPLE_POINTS 48
#define WAVE_TYPE sinTable[waveIndex]
#define SINE_WAVEFORM 0
#define SQUARE_WAVEFORM 1
#define TRIANGLE_WAVEFORM 2
#define SAWTOOTH_WAVEFORM 3

/*
 *Function Prototypes
 */

//Runs the main loops of our program.
void main_loop(void);

//Runs the i2c, reading off pot on PINA4 and to update VCF variable resistor.
void run_i2c(uint8_t pot);

//Does an analogue read from a pot on pin A4, returns the integer value
uint8_t get_pot_value(void);

//Register buttons presses and adjustment of OCRA counter value.
void generate_notes(void);

//Initialises the USART
void initialise_USART(void) ;

// Initalise watch dog timer
void initialise_wdt(void);

// Initalise button frequency
void initialise_button_freq(void);

// A function for updating prescale of timer 0.
void update_timer0_pre (int value);

// A function for updating prescale of timer 1.
void update_time1_pre (int value);

// A function for updating prescale of timer 2.
void update_time2_pre (int value);

// A function for the press of a keyboard key.
void play_keyboard(int value);

// A function for the release of a keyboard key.
void stop_keyboard (int value);

// A function for the updating of button notes.
void update_button_freq(int receValue, int buttonIndex);

// A function for running sleep mode.
void run_sleep(void);

// Start timer interrupt to generate wave.
void start_wave(int button);

// Stop timer interrupt to stop wave.
void stop_wave(int button);

// A function for the update of wave type.
void update_wave_type(void);

// A struture for the value of active notes
typedef struct
{
	int value;	//0 - 7; or keyboard, if no occupied, -1 means is available
}ActiveQuene;

ActiveQuene buttonQueue[3];
ActiveQuene keyboardQueue[3];

// A structure for the ocr value and prescale of frequency.
typedef struct
{
	uint8_t OCR;
	int PRE;
}Freq;

// Frequency array of 84 notes.
int freqOCRArray[84] = {153,144,136,128,121,211,108,102,96,91,85,81,76,72,68,64,61,57,54,51,48,45,
	43,40,38,36,34,32,243,229,216,204,192,182,172,162,153,144,136,128,121,114,108,102,96,91,86,81,
	76,72,68,64,60,57,54,51,48,45,43,40,38,36,34,32,243,229,216,204,192,182,171,161,153,144,136,128,
121,114,108,102,96,91,86,81};
int freqPREArray[84] = { 64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,
	64,64,64,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,1,1,1,1,1,1,1,
1,1,1,1,1,1,1,1,1,1,1,1,1};

// Frequency array of initial ocr and pre value of 8 button notes.
int initOCRArray[8] = {91,80,72,64,57,51,48,43};
int initPREArray[8] = {8,8,8,8,8,8,8,8};

// sine look up table.
const int sinTable[]= {
	128,144,160,176,191,205,218,229,
	238,245,251,254,255,254,251,245,
	238,229,218,205,191,176,160,144,
	128,111,95,79,64,50,37,26,
	17,10,4,1,0,1,4,10,
	17,26,37,50,64,79,95,111
};

// triangle look up table.
const int triTable[] = {
	11,21,32,43,53,64,74,85,
	96,106,117,128,138,149,159,170,
	181,191,202,213,223,234,244,255,
	244,234,223,213,202,191,181,170,
	159,149,138,128,117,106,96,85,
	74,64,53,43,32,21,11,0
};

// sawtooth look up table
const int sawTable[] = {
	5,11,16,21,27,32,37,43,
	48,53,58,64,69,74,80,85,
	90,96,101,106,112,117,122,128,
	133,138,143,149,154,159,165,170,
	175,181,186,191,197,202,207,213,
	218,223,228,234,239,244,250,255
};

// square look up table.
const int squTable[] = {
	0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,
	255,255,255,255,255,255,255,255,
	255,255,255,255,255,255,255,255,
	255,255,255,255,255,255,255,255
};