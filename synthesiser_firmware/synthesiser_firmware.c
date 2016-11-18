/*
 * synthesiser.c
 *
 */

#include "synthesiser.h"

//Global variables
volatile unsigned int value;
volatile int disableSleep = -1;
int i = 0;
int timerNum = 0, buttonIndex = 0;
// potentiometer value
int p_val;
// Button press variable
int buttonAReading = 0, buttonCReading = 0;
int lastButtonAReading = 0, lastButtonCReading = 0;
// Four pole switch variable
const int *wave_table;
int switch_reading, last_switch_reading;
// Sleep mode variable
int  sleepflag = 0, light_delay, sleepCounter = 313;
// Timer interrupt counter
int timer1Index = 0, timer0Index = 0, timer2Index = 0;
// Frequency variable
Freq KeySound[84];
Freq ButtonSound[8];

/* 
 * A function for initialising three timer. The reason to have three timer open is to
 * receive three different frequency signal when three button is pressed all the time.
 *
 */
void initialise_timer (void) {
	TCCR0A = (0<<COM0A1) | (0<<COM0A0) | (1<<WGM01); 
	TCCR0B = _BV(CS00);
	TCCR1A = (0<<COM1A1) | (0<<COM1A0); 
	TCCR1B = _BV(CS10) | _BV(WGM12);
	TCCR2A = (0<<COM2A1) | (0<<COM2A0) | (1<<WGM21); 
	TCCR2B = _BV(CS20);
	TCNT1 = 0;
	TCNT0 = 0;
	TCNT2 = 0;
}

/*
 * A function for initialising USART connection to receive the notes and scale
 * change the software send to synthesiser board.
 *
 */
void initialise_USART(void) {
	UBRR0L = BAUD_PRESCALE;
	UBRR0H = (BAUD_PRESCALE >> 8);
	UCSR0B = ((1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0));
}

/*
 * A function for initialising adc channel to receiving the cutoff frequency
 * signal which sent from the potentiometer to voltage control filter. 
 *
 */
void initialise_adc_channel (void) {
	ADCSRA = (1<<ADEN) | (1<<ADPS1) | (1<<ADPS2);
	ADMUX  = _BV(MUX2) | _BV(ADLAR);
}

/*  
 * A function for initialising watch dog timer.
 */
void initialise_wdt(void)
{
	wdt_reset();
	WDTCSR = (1<<WDCE)|(1<<WDE);
	WDTCSR = (1<<WDIE)|(1<<WDP0);
}

/*
 *A function for initialising all hardware settings.
 */
void initialise_hardware() {
	initialise_timer(); // Initialise timer
	initialise_adc_channel(); // Initialise adc channel
	initialise_USART();  // Initialise USART
	initialise_i2c(); //initialise i2c
	initialise_wdt(); //initialise watchdog
}

/*
 * A function for initialising data direction register.
 */
void initialise_DDR() {
	DDRC = 0x04;
	DDRB  = 0xFF;
	DDRA = 0x00;
	DDRD = 0xFF;
}

/*
 * A function for initialising default button frequency 
 * in case the software doesn't send default note to synth.
 */
void initialise_button_freq(void) {
	for (i = 0; i < 84; i++) {
		KeySound[i].OCR = freqOCRArray[i];
		KeySound[i].PRE = freqPREArray[i];
	}
	for (i = 0; i < 8; i++) {
		ButtonSound[i].OCR = initOCRArray[i];
		ButtonSound[i].PRE = initPREArray[i];
	}
}

/*
 * This is the main function to start with.
 */
int main(void)
{
	initialise_DDR();
	initialise_hardware();
	initialise_button_freq();
	sei();
	wave_table = sinTable;
	buttonQueue[0].value = -1;
	buttonQueue[1].value = -1;
	buttonQueue[2].value = -1;
	keyboardQueue[0].value = -1;
	keyboardQueue[1].value = -1;
	keyboardQueue[2].value = -1;
	last_switch_reading = switch_reading = 0;
	p_val = get_pot_value();
	run_i2c(p_val);
	disableSleep = -1;
	sleepflag = 0;
	while(1) {
		main_loop();
	}
}

/*  
 * This is the main loop function.
 */
void main_loop(void){
	PORTD &= ~ (_BV(PORTD7) | _BV(PORTD3));  
	PORTC &= ~_BV(PORTC2);
	if (disableSleep != -1)
	{
		WDTCSR &= ~(1<<WDIE);
	}
	if (sleepflag == 1) {
		run_sleep();
	} else {
		PORTD |= _BV(PORTD4);
	}
	p_val = get_pot_value();
	run_i2c(p_val);
	switch_reading = PINA;
	if(switch_reading != last_switch_reading) {
		update_wave_type();
	}
	generate_notes();
}

/* 
 * Register button presses and turn on LED to indicate to user.
 */
void generate_notes(void) {

	buttonAReading = PINA;
	buttonCReading = PINC;
	
	// The first button is pressed
	if ((buttonCReading & 0x08) && !(lastButtonCReading & 0x08))	{
		start_wave(0);
	} 
	// The first button is released
	else if (!(buttonCReading & 0x08) && (lastButtonCReading & 0x08))	{
		stop_wave(0);
	}
	
	// The second button is pressed
	if ((buttonCReading & 0x10) && !(lastButtonCReading & 0x10))	{
		start_wave(1);
	}  
	// The second button is released
	else if (!(buttonCReading & 0x10) && (lastButtonCReading & 0x10))	{
		stop_wave(1);
	}
	// The third button is pressed
	if ((buttonCReading & 0x20) && !(lastButtonCReading & 0x20))	{
		start_wave(2);
	} 
	// The third button is released
	else if (!(buttonCReading & 0x20) && (lastButtonCReading & 0x20))	{
		stop_wave(2);

	}
	// The fourth button is pressed
	if ((buttonCReading & 0x40) && !(lastButtonCReading & 0x40))	{
		start_wave(3);
	} 
	// The fourth button is released
	else if (!(buttonCReading & 0x40) && (lastButtonCReading & 0x40))	{
		stop_wave(3);
	}
	// The fifth button is pressed
	if ((buttonCReading & 0x80) && !(lastButtonCReading & 0x80))	{
		start_wave(4);
	}
	// The fifth button is released
	else if (!(buttonCReading & 0x80) && (lastButtonCReading & 0x80))	{
		stop_wave(4);
	}
	// The sixth button is pressed
	if ((buttonAReading & 0x80) && !(lastButtonAReading & 0x80))	{
		start_wave(5);
	}
	// The sixth button is released
	else if (!(buttonAReading & 0x80) && (lastButtonAReading & 0x80))	{
		stop_wave(5);
	}
	// The seventh button is pressed
	if ((buttonAReading & 0x40) && !(lastButtonAReading & 0x40))	{
		start_wave(6);
	}
	// The seventh button is released
	else if (!(buttonAReading & 0x40) && (lastButtonAReading & 0x40))	{
		stop_wave(6);
	}
	// The eighth button is pressed
	if ((buttonAReading & 0x20) && !(lastButtonAReading & 0x20))	{
		start_wave(7);
	}
	// The eighth button is released
	else if (!(buttonAReading & 0x20) && (lastButtonAReading & 0x20))	{
		stop_wave(7);
	}	
	
	if(!(buttonCReading & 0xf8) && !(buttonAReading & 0xE0)) {
		buttonQueue[0].value = -1;
		buttonQueue[1].value = -1;
		buttonQueue[2].value = -1;		
	}
	else {
		sleepCounter = 313;
		PORTD |= (1<<PORTD5);
	}
	lastButtonAReading = buttonAReading;
	lastButtonCReading = buttonCReading;
}

/*
 * A function for the release of a keyboard key.
 */
void stop_keyboard (int value) {
	int i = 0;
	for(i=0; i<3; i++) {
		if(keyboardQueue[i].value == value - 84) {
			if(i==0) {
				TIMSK0 = 0;
			}
			if(i==1) {
				TIMSK1 = 0;
			}
			if(i==2) {
				TIMSK2 = 0;
			}
			keyboardQueue[i].value = -1;
		}
	}
	if (TIMSK0 == 0 && TIMSK1 == 0 && TIMSK2 == 0)
	{
		PORTD &= ~((1 << PORTD5) | (1 << PORTD6));
	}
		
}

/*
 * A function for the press of a keyboard key.
 */
void play_keyboard(int value) {
	int i;
	PORTD |= (1<<PORTD5) | (1<<PORTD6);
	for (i = 0; i < 3; i++)
	{
		if (keyboardQueue[i].value == -1)
		{
			keyboardQueue[i].value = value;
			if(i==0) {
				OCR0A = KeySound[value].OCR;
				update_timer0_pre (KeySound[value].PRE);
				TIMSK0 |= _BV(OCIE0A);
				return;
			}
			if(i==1) {
				OCR1A = KeySound[value].OCR;
				update_time1_pre(KeySound[value].PRE);
				TIMSK1 |= _BV(OCIE1A);
				return;
			}
			if(i==2) {
				OCR2A = KeySound[value].OCR;
				update_time2_pre(KeySound[value].PRE);
				TIMSK2 |= _BV(OCIE2A);
				return;
			}
		}
					
	}
	OCR0A =KeySound[value].OCR;
	update_timer0_pre(KeySound[value].PRE);
	TIMSK0 |= _BV(OCIE0A);
	buttonQueue[0].value = KeySound[value].OCR;
}

/*
 * A function for the updating of button notes
 */
void update_button_freq(int receValue, int buttonIndex) {
	PORTD |= (1<<PORTD6);
	int valueIndex;
	valueIndex = receValue - 168;
	ButtonSound[buttonIndex] = KeySound[valueIndex];
	
}

/*
 * An interrupt for the USART communication.
 */
ISR(USART0_RX_vect) {
	value = UDR0;
	disableSleep = value;
	if (value >167) {
		update_button_freq(value, buttonIndex);
		buttonIndex++;
		
	} else if (value > 83 && value <= 167) {
		stop_keyboard(value);
	} else if (value <= 83) {
		play_keyboard(value);
	}
	if (buttonIndex == 8)
	{
		buttonIndex = 0;
		PORTD &= ~(_BV(PORTD6));
	}
}

/*
 * A function for getting the potention value from ADC.
 *
 * @return int - potentiometer value
 */
uint8_t get_pot_value(void) {
	uint8_t potentiometer_val;
	ADMUX  = _BV(MUX2) | _BV(ADLAR);
	ADCSRA |= (1<<ADSC);
	while(ADCSRA & (1<<ADSC)) {
		;  //Wait until conversion finished 
	}
	potentiometer_val = ADCH; 
	return potentiometer_val;
}

/*
 * An interrupt for timer 0 to send to the flipflop 1.
 */
ISR (TIMER0_COMPA_vect) {
	PORTD &= ~_BV(PORTD3);
	switch(timer0Index) {
		case SAMPLE_POINTS:
		timer0Index = 0;
		default:
		PORTB =	wave_table[timer0Index];
		timer0Index++;
		PORTD |= _BV(PORTD3);
	}
}

/*
 * An interrupt for timer 1 to send to the flipflop 2.
 */
ISR (TIMER1_COMPA_vect) {
    PORTD &= ~_BV(PORTD7);
	switch(timer1Index) {
		case SAMPLE_POINTS:
			timer1Index = 0;
		default:
			PORTB =	wave_table[timer1Index];
			timer1Index++;
			PORTD |= _BV(PORTD7);
	}
}

/*
 * An interrupt for timer 2 to send to the flipflop 3.
 */
ISR (TIMER2_COMPA_vect) {
	PORTC &= ~_BV(PORTC2);
	switch(timer2Index) {
		case SAMPLE_POINTS:
			timer2Index = 0;
		default:
			PORTB =	wave_table[timer2Index];
			timer2Index++;
			PORTC |= _BV(PORTC2);
	}
}


/*
 * A function for update prescale of timer 0.
 */
void update_timer0_pre (int value) {
	if (value == 1) {
		TCCR0B = (0<<CS02) | (0<<CS01) | (1<<CS00);
		} else if (value == 8) {
		TCCR0B = (0<<CS02) | (1<<CS01) | (0<<CS00);
		} else if (value == 64) {
		TCCR0B = (0<<CS02) | (1<<CS01) | (1<<CS00);
	}
}


/*
 * A function for updating prescale of timer 1.
 */
void update_time1_pre (int value) {
	if (value == 1) {
		TCCR1B = (0<<CS12) | (0<<CS11) | (1<<CS10) | (1<<WGM12);
		} else if (value == 8) {
		TCCR1B = (0<<CS12) | (1<<CS11) | (0<<CS10) | (1<<WGM12);
		} else if (value == 64) {
		TCCR1B = (0<<CS12) | (1<<CS11) | (1<<CS10) | (1<<WGM12);
	}
}

/*
 * A function for updating prescale of timer 2.
 */
void update_time2_pre (int value) {
	if (value == 1) {
		TCCR2B = (0<<CS22) | (0<<CS21) | (1<<CS20);
		} else if (value == 8) {
		TCCR2B = (0<<CS22) | (1<<CS21) | (0<<CS20);
		} else if (value == 64) {
		TCCR2B = (0<<CS22) | (1<<CS21) | (1<<CS20);
	}
}


/* 
 * The interrupt for the watch dog timer.
 */
ISR(WDT_vect)
{
	//Burst of fice 0.1Hz pulses
	if (sleepCounter != 0)
	{
		sleepflag = 0;
		sleepCounter--;
	} else {
		if (sleepflag == 0)
		{
			light_delay = 156;
			PORTD &= ~(1<<PORTD4);
		}
		sleepflag = 1;

		if(light_delay) {
			light_delay--;
		} else {
			light_delay = 156;
			PORTD &= ~(1<<PORTD4);
		}
		if(light_delay <= 1) {
			PORTD = (1<<PORTD4);
		}
	}
}

/*  
 * A function for running sleep mode.
 */
void run_sleep(void) {
	set_sleep_mode(SLEEP_MODE_IDLE);
	cli();
	sleep_enable();
	sei();
	sleep_cpu();
	sleep_disable();
	sei();
}

/*  
 * A function for the update of wave type.
 */
void update_wave_type(void) {
	sleepCounter = 313;
	if((switch_reading & 0x01) == 0) {
		wave_table = sawTable;
	}
	if((switch_reading & 0x02) == 0) {
		wave_table = triTable;
	}
	if((switch_reading & 0x04) == 0) {
		wave_table = squTable;
	}
	if((switch_reading & 0x08) == 0) {
		wave_table = sinTable;
	}
	last_switch_reading = switch_reading;
}


/* 
 * Starts i2c, if successful sends a value passed into the function 
 * to the variables resistor.
 *
 */
void run_i2c(uint8_t pot) {
	unsigned char ret;
	ret = i2c_start(AD5241);      
	if (ret) {
		i2c_stop();
		} else {
		ret  = i2c_write(0b00000000);   
		ret = i2c_write(pot);
		i2c_stop();					
	}
}

/*
 * Start timer interrupt to generate wave
 *
 */
void start_wave(int button) {
	int i;
	for(i=0; i<3; i++) {
		if(buttonQueue[i].value == -1) {
			buttonQueue[i].value = button;
			if(i==0) {
				OCR0A = ButtonSound[button].OCR;
				update_timer0_pre(ButtonSound[button].PRE);
				TIMSK0 |= _BV(OCIE0A);
				return;
			}
			if(i==1) {
				OCR1A = ButtonSound[button].OCR;
				update_time1_pre(ButtonSound[button].PRE);
				
				TIMSK1 |= _BV(OCIE1A);
				return;
			}
			if(i==2) {
				OCR2A = ButtonSound[button].OCR;
				update_time2_pre(ButtonSound[button].PRE);
				TIMSK2 |= _BV(OCIE2A);
				return;
			}
		}
	}
	
	OCR0A = ButtonSound[button].OCR;
	update_time2_pre(ButtonSound[button].PRE);
	TIMSK0 |= _BV(OCIE0A);
	buttonQueue[0].value = button;
	
}

/*
 * Stop timer interrupt to stop wave
 *
 */
void stop_wave(int button) {
	int i;
	PORTD &= ~(_BV(PORTD5));
	for(i=0; i<3; i++) {
		if(buttonQueue[i].value == button) {
			if(i==0) {
				TIMSK0 = 0;
			}
			if(i==1) {
				TIMSK1 = 0;
			}
			if(i==2) {
				TIMSK2 = 0;
			}
			buttonQueue[i].value = -1;
		}
	}	
}


