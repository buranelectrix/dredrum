/*

+----------------------------------------------------------------------+
|                                                                      |
|                        D  R  E  D  R  U  M                           |
|                                                                      |
|                               v 1.0                                  |
|                                                                      |
|          8-bit 3 oscillator percussion/drone PWM generator           |
|                                                                      |
|                  target: Atmel ATmega 48P @ 20MHz                    |
|                                                                      |
+----------------------------------------------------------------------+
|                     Copyright 2014 Jan Cumpelik                      |
|                                                                      |
| This program is free software: you can redistribute it and/or modify |
| it under the terms of the GNU General Public License as published by |
| the Free Software Foundation, either version 3 of the License, or    |
| (at your option) any later version.                                  |
|                                                                      |
| This program is distributed in the hope that it will be useful,      |
| but WITHOUT ANY WARRANTY; without even the implied warranty of       |
| MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the        |
| GNU General Public License for more details.                         |
|                                                                      |
| You should have received a copy of the GNU General Public License    |
| along with this program.  If not, see <http://www.gnu.org/licenses/>.|
+----------------------------------------------------------------------+

*/

 
#include <avr/io.h>
#define F_CPU 20000000L
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "./8bit_q_sine.h"

// macros
	#define BUTTRIG		(1 << 0)
	#define BUTWAVE		(1 << 1)
	#define BUTVOICE1		(1 << 2)
	#define BUTVOICE2		(1 << 3)
	#define BUTVOICE3		(1 << 4)
	#define TRIGIN 		(1 << 5)

	#define SIGNAL_BUFFER_SIZE 16

	#define V1 	0
	#define V2 	1
	#define V3 	2

	#define PITCH		3 	//	PITCH
	#define PITCHENV		2 	//	PITCH DROP	
	#define DECAY		1 	//	DECAY
	#define LEVEL		0 	//	LEVEL
	#define WAVE		4 	//	WAVE

	#define NEW			0
	#define STORE		1
	#define UPDATE 		2

	#define ALLOWED 		0
	#define NOT_ALLOWED	1


// trigger becomes 1 when rising edge is detected on trig input
volatile uint8_t trigger = 0;

// audio data buffer structure
typedef struct {
	uint8_t 	data[SIGNAL_BUFFER_SIZE] ;
	uint8_t 	head;
	uint8_t 	tail;
	uint8_t 	fill;
} bufferstructure;
volatile bufferstructure buffer;

// structure for storing button states
typedef struct {
	uint8_t 	scanned;
	uint8_t 	last;
	uint8_t 	notused;
} buttonsstructure;	

//  structure with oscillator parameters
typedef struct {
	uint8_t	playing;
	uint32_t 	phase_acc;
	uint16_t 	level_acc;
} oscillator;


// setup hardware routine
void setup_basedrum() {
		
	DDRD = 0x00;
	PORTD = 0x00;

	DDRD &= ~(1 << PD0);	// trig in input
	DDRD |=  (1 << PD1);	// led voice 1 output
	DDRD &= ~(1 << PD2);	// voice 1 button input
	PORTD |= (1 << PD2);	// pullup on
	DDRD |=  (1 << PD3); 	// led voice 2 output
	DDRD &= ~(1 << PD4); 	// trig button input
	PORTD |= (1 << PD4);	// pullup on
	DDRD &= ~(1 << PD5); 	// voice 2 button input
	PORTD |= (1 << PD5);	// pullup on
	DDRD |=  (1 << PD6); 	// led voice 3 output
	DDRD |=  (1 << PD7);	// led play output

	DDRB = 0x00;
	PORTB = 0x00;
	DDRB &= ~(1 << PB0);	// boice 3 button input
	PORTB |= (1 << PB0); 	// pullup on
	DDRB |= (1 << PB1);	// pwm out OC1A
	DDRB |= (1 << PB2);	// leds wave select
	DDRB |= (1 << PB3);
	DDRB |= (1 << PB4);
	DDRB |= (1 << PB5);

	DDRC = 0x00;		// all inputs
	PORTC = 0x00;
	PORTC |= (1 << PC1); 	// wave button pullup

	// internal ADC setup
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); 	//  prescaler 128
	ADMUX = (1<<ADLAR) | (1 << REFS0); 					// left adjust, AVcc ref
	ADCSRA |= (1 << ADEN); 							// enable ADC
	ADCSRA |= (1 << ADSC); 							// start single conversion to initialize ADC

	// interrupts setup 
		cli();								// global int disable
	// TIMER 0 - 8bit - audio out interrupt
		TCCR0A = 0;								// reset register
		TCCR0A |= (1 << WGM01);						// CTC
		TCCR0B = 0;
		TCCR0B |= (1 << CS01);						// prescaler 8
		TIMSK0 = (1 << OCIE0A);						// output compare enable
		OCR0A = 130;							// 15625 Hz
	// TIMER 2 - 8bit - general purpose
		TCCR2A	=	0;						// normal operation
		TCCR2B	=	(1 << CS20) | (1 << CS21) | (1 << CS22);	// 1024 prescaler (19531Hz)
		TCNT2	=	0;
	// TIMER 1 - 10bit PWM
		// set timer to fast pwm mode
		TCCR1B	=	(1 << WGM12);
		TCCR1A	=	(1 << WGM10);				// 8bit pwm
		// prescaler /1
		TCCR1B	|= 	(1 << CS10);
		// clear OC1A pin on compare match, set at bottom
		TCCR1A	|= (1 << COM1A1);
		sei();		
}

// waverenderer returns amplitude value being given position in time and type of wave
uint8_t waverender(uint16_t wave_pointer, uint8_t wave_type) {
	uint8_t wave = 127;
	switch(wave_type) {
		// sine wave ----------------------------------------------------------------
		case 0:
			// load from table
			if (wave_pointer > 0xff) {
				if (wave_pointer > 0x017f) {
					// 4. quarter
					wave_pointer &= 0xff;
					wave_pointer = 0xff - wave_pointer;
					wave = pgm_read_byte(&q_sine_table[wave_pointer]);
				} else {
					// 3. quarter
					wave_pointer &= 0xff;
					wave = pgm_read_byte(&q_sine_table[wave_pointer]);
				}
				wave = 0x7f - wave;
			} else {
				if (wave_pointer > 0x7f) {
					// 2. quarter
					wave_pointer = 0xff - wave_pointer;
					wave = pgm_read_byte(&q_sine_table[wave_pointer]);
				} else {
					// 1. quarter
					wave = pgm_read_byte(&q_sine_table[wave_pointer]);
				}
				wave += 0x7f;
			}
		break;
		// square wave ----------------------------------------------------------------
		case 1:
			if (wave_pointer > 0xff) {
				wave = 0xff;
			} else {
				wave = 0x00;
			}
		break;
		// saw wave ----------------------------------------------------------------
		case 2:
			wave_pointer >>= 1;
			wave = 0xff - wave_pointer;
		break;
		// noise ----------------------------------------------------------------
		// Dredrum's unique digital noise is created by reading program data ;)
		case 3:
			wave = pgm_read_byte(&q_sine_table[0xff + wave_pointer]);
		break;
	}
	return wave;
}


// storing unit settings to memory
// settings are recalled when power up
void store_to_EEPROM(uint8_t array[][5]) {

	//disable interrupts
	cli();

	// write to eeprom, addr 10, lenght 15
	eeprom_write_block( (const void*)array, (void*) 10, 15);

	//done, flash leds
	uint8_t c;
	uint32_t d;
	for (c = 1; c < 7; c++) {
		for (d = 0; d < 100000; d++) {
			if (c & 0x01) {
				PORTD |= (1 << PD1) | (1 << PD3) | (1 << PD6);
			} else {
				PORTD &= ~(1 << PD1);
				PORTD &= ~(1 << PD3);
				PORTD &= ~(1 << PD6);
			}
		}
	}

	//enable interrupts
	sei();
}



//////////////////////////////////////////////////
//
// MAIN ROUTINE
//
//////////////////////////////////////////////////

int main() {

	cli();

	uint8_t audiodata = 0;
	uint8_t voice_params[3][5];	// [voice][parameter]

	uint8_t pots[4][3];		// [pot][new/store/updateflag]

	buttonsstructure buttons;
	buttons.scanned = 0x00;
	buttons.last = 0x00;
	buttons.notused = 0x00;
	uint8_t voice = 0;
	uint8_t last_voice = 0;
	oscillator v1;
	oscillator v2;
	oscillator v3;

	uint8_t click_mode = 0;
			
	v1.playing = 0;
	v1.phase_acc = 0;
	v1.level_acc = 0;
	v2.playing = 0;
	v2.phase_acc = 0;
	v2.level_acc = 0;
	v3.playing = 0;
	v3.phase_acc = 0;
	v3.level_acc = 0;

	// run HW setup routine
	setup_basedrum();

	//initialize arrays
	uint8_t i;
	for (i = 0; i < SIGNAL_BUFFER_SIZE; i++) {
		buffer.data[i]= 127;
	}

	// load sound settings from memory
	eeprom_read_block( (void*)voice_params, (const void*) 10, 15 );

	for (i = 0; i < 4; i++) {
		pots[i][NEW] = 0;
		pots[i][STORE] = 0;
		pots[i][UPDATE] = ALLOWED;
	}

	sei();


	//////////////////////////////////////////////////
	//
	// MAIN ROUTINE LOOP
	//
	//////////////////////////////////////////////////	

	while(1) {

		// when "trigger" is 1
		if (trigger) {
			
			trigger = 0;
			
			v1.playing = 1;
			v1.phase_acc = 0;
			v1.level_acc = 0;

			v2.playing = 1;
			v2.phase_acc = 0;
			v2.level_acc = 0;

			v3.playing = 1;
			v3.phase_acc = 0;
			v3.level_acc = 0;
		}

		while (buffer.fill < SIGNAL_BUFFER_SIZE) {	// keep audio buffer filled

			int16_t v1_signed = 0;
			int16_t v2_signed = 0;
			int16_t v3_signed = 0;
		
			// voice 1 level accumulator - decay
			v1.level_acc += (( 0xff - voice_params[V1][DECAY] ) >> 3);
			// elapsed time runs from 0 to 127
			uint8_t v1_elapsed_time = v1.level_acc >> 9;
			if (v1_elapsed_time >= 127) v1.playing = 0; // end of sound
			// voice 1 pitch accumulator - pitch
			v1.phase_acc += (( voice_params[V1][PITCH] << 4) + 1);
			v1.phase_acc += ((127 - v1_elapsed_time) * (voice_params[V1][PITCHENV])) >> 4 ;	
			// v1_wave_pointer gives the "time" value for waverenderer
			uint16_t v1_wave_pointer = v1.phase_acc >> 8;
			v1_wave_pointer &= 0x01ff;

			if (v1.playing) {
				// call waverenderer
				uint8_t v1_wave = waverender(v1_wave_pointer, voice_params[V1][WAVE]);
				// voice 1 volume calculation
				uint16_t v1_volume = voice_params[V1][LEVEL];	// 0..255
				v1_volume *= (127 - v1_elapsed_time);		// 0..32358
				v1_volume >>= 8;						// 0..126
				// voice 1 volume application on wave amplitude
				v1_signed = (int16_t)v1_wave; 			// 0..255
				v1_signed -= 128;						// -128..127
				v1_signed *= (int16_t)v1_volume;			// -16256..16129
				v1_signed >>= 7;						// -128..127
			} 

			// voice 2 level accumulator - decay
			v2.level_acc += (( 0xff - voice_params[V2][DECAY] ) >> 3);
			// elapsed time runs from 0 to 127
			uint8_t v2_elapsed_time = v2.level_acc >> 9;
			if (v2_elapsed_time >= 127) v2.playing = 0; // end of sound
			// voice 2 pitch accumulator - pitch
			v2.phase_acc += (( voice_params[V2][PITCH] << 4) + 1);
			v2.phase_acc += ((127 - v2_elapsed_time) * (voice_params[V2][PITCHENV])) >> 4;
			// v2_wave_pointer gives the "time" value for waverenderer
			uint16_t v2_wave_pointer = v2.phase_acc >> 8;
			v2_wave_pointer &= 0x01ff;

			if (v2.playing) {
				// call waverenderer
				uint8_t v2_wave = waverender(v2_wave_pointer, voice_params[V2][WAVE]);
				// voice 2 volume calculation
				uint16_t v2_volume = voice_params[V2][LEVEL];	// 0..255
				v2_volume *= (127 - v2_elapsed_time);		// 0..32358
				v2_volume >>= 8;						// 0..126
				// voice 2 volume application on wave amplitude
				v2_signed = (int16_t)v2_wave; 			// 0..255
				v2_signed -= 128;						// -128..127
				v2_signed *= (int16_t)v2_volume;			// -16256..16129
				v2_signed >>= 7;						// -128..127
			} 

			// voice 3 level accumulator - decay
			v3.level_acc += (( 0xff - voice_params[V3][DECAY] ) >> 3);
			// elapsed time runs from 0 to 127
			uint8_t v3_elapsed_time = v3.level_acc >> 9;
			if (v3_elapsed_time >= 127) v3.playing = 0; // end of sound
			// voice 3 pitch accumulator - pitch
			v3.phase_acc += (( voice_params[V3][PITCH] << 4) + 1);
			v3.phase_acc += ((127 - v3_elapsed_time) * (voice_params[V3][PITCHENV])) >> 4;
			// v3_wave_pointer gives the "time" value for waverenderer
			uint16_t v3_wave_pointer = v3.phase_acc >> 8;
			// end of sound after one amplitude if click mode is on
			if (click_mode) {
				if (v3_wave_pointer >= 0x01ff) v3.playing = 0;
			} 

			if (v3.playing) {
				v3_wave_pointer &= 0x01ff;
				// call waverenderer
				uint8_t v3_wave = waverender(v3_wave_pointer, voice_params[V3][WAVE]);
				// voice 3 volume calculation
				uint16_t v3_volume = voice_params[V3][LEVEL];	// 0..255
				v3_volume *= (127 - v3_elapsed_time);		// 0..32358
				v3_volume >>= 8;						// 0..126
				// voice 3 volume application on wave amplitude
				v3_signed = (int16_t)v3_wave; 			// 0..255
				v3_signed -= 128;						// -128..127
				v3_signed *= (int16_t)v3_volume;			// -16256..16129
				v3_signed >>= 7;						// -128..127
			} 

			// mix all voices together
			int16_t mix = v1_signed + v2_signed + v3_signed;
			if (mix < -127) mix = -127;
			if (mix > 127)  mix = 127;
			mix += 127;
			audiodata = mix;
			// insert audio data to buffer
			++ buffer.tail;
			if (buffer.tail >= SIGNAL_BUFFER_SIZE) buffer.tail = 0;
			buffer.data[buffer.tail] = audiodata;
			++ buffer.fill;

		}

		// LEDS output
		if (voice == 0) {
			PORTD |= (1 << PD1);
			PORTD &= ~(1 << PD3);
			PORTD &= ~(1 << PD6);
		} else if (voice == 1) {
			PORTD &= ~(1 << PD1);
			PORTD |= (1 << PD3);
			PORTD &= ~(1 << PD6);
		} else if (voice == 2) {
			PORTD &= ~(1 << PD1);
			PORTD &= ~(1 << PD3);
			PORTD |= (1 << PD6);
		}

		uint8_t waveleds = 0;
		static uint8_t last_waveleds = 0;
		waveleds = (1 << (5 - (voice_params[voice][WAVE])));
		if (waveleds == last_waveleds) {
		} else {
			PORTB &= ~(0b00111100);
			PORTB |= waveleds;
		}
		last_waveleds = waveleds;

		if ( (v1.playing) || (v2.playing) || (v3.playing) ) {
			PORTD |= (1 << PD7);
		} else {
			PORTD &= ~(1 << PD7);
		}

		// scan and process buttons
			if (TCNT2 > 195) { 			// cca 100Hz
			TCNT2 = 0;

			if (PINC & (1 << PC0)) {
				click_mode = 0;
			} else {
				click_mode = 1;
			}

			buttons.scanned = 0x00;
			if (PIND & (1 << PD4)) buttons.scanned |= (BUTTRIG);
			if (PINC & (1 << PC1)) buttons.scanned |= (BUTWAVE);
			if (PIND & (1 << PD2)) buttons.scanned |= (BUTVOICE1);
			if (PIND & (1 << PD5)) buttons.scanned |= (BUTVOICE2);
			if (PINB & (1 << PB0)) buttons.scanned |= (BUTVOICE3);

			buttons.scanned ^= 0xff;
			buttons.notused |= buttons.scanned & ~(buttons.last);

			uint8_t freshbutt = buttons.scanned & buttons.notused;

			if ( (buttons.scanned & BUTVOICE1) && (buttons.scanned & BUTVOICE3) ) {
				if ((buttons.notused & BUTVOICE1) || (buttons.notused & BUTVOICE3)) {
					store_to_EEPROM(voice_params);
					buttons.notused &= ~(BUTVOICE1);
					buttons.notused &= ~(BUTVOICE3);
				}
			}

			if (freshbutt & BUTVOICE1) {
				voice = 0;
				buttons.notused &= ~(BUTVOICE1);
			}
			if (freshbutt & BUTVOICE2) {
				voice = 1;
				buttons.notused &= ~(BUTVOICE2);
			}
			if (freshbutt & BUTVOICE3) {
				voice = 2;
				buttons.notused &= ~(BUTVOICE3);
			}
			if (freshbutt & BUTWAVE) {
				++ voice_params[voice][WAVE];
				if (voice_params[voice][WAVE] > 3) voice_params[voice][WAVE] = 0;
				buttons.notused &= ~(BUTWAVE);
			}
			if (freshbutt & BUTTRIG) {
				trigger = 1;
				buttons.notused &= ~(BUTTRIG);
			}
			buttons.last = buttons.scanned;
		}

		// read pots
		static uint8_t current_pot = 0;
		// ADIF is set when previous conversion is complete, so pick conversion result
		if (ADCSRA & (1 << ADIF)) {
			// clear flag
			ADCSRA |= (1 << ADIF);
			// read ADC result
			pots[current_pot][NEW] = ADCH;

			if (last_voice != voice) {
				pots[0][UPDATE] = NOT_ALLOWED;
				pots[1][UPDATE] = NOT_ALLOWED;
				pots[2][UPDATE] = NOT_ALLOWED;
				pots[3][UPDATE] = NOT_ALLOWED;
				pots[0][STORE] = pots[0][NEW];
				pots[1][STORE] = pots[1][NEW];
				pots[2][STORE] = pots[2][NEW];
				pots[3][STORE] = pots[3][NEW];	
			}

			if (pots[current_pot][UPDATE] == NOT_ALLOWED) {
				if ( (pots[current_pot][NEW] < (pots[current_pot][STORE]- 6)) ||
					(pots[current_pot][NEW] > (pots[current_pot][STORE] + 6)) ) {
					pots[current_pot][UPDATE] = ALLOWED;
				}
			}

			if (pots[current_pot][UPDATE] == ALLOWED) {
				voice_params[voice][current_pot] = pots[current_pot][NEW];
			}

			last_voice = voice;

			// select new pot for new conversion
			++ current_pot;
			current_pot &= 0x03;
			ADMUX = current_pot + 2;
			ADMUX |= (1<<ADLAR) | (1 << REFS0); // left adjust, AVcc ref
			// start new single conversion
			ADCSRA |= (1 << ADSC); 
		}
	}
}


//////////////////////////////////////////////////
//
// INTERRUPT ROUTINE
//
//////////////////////////////////////////////////

ISR(TIMER0_COMPA_vect) {

	++ buffer.head;
	if (buffer.head >= SIGNAL_BUFFER_SIZE) buffer.head = 0;
	OCR1A = buffer.data[buffer.head];
	-- buffer.fill;

	static uint8_t triginput = 0;
	triginput <<= 1;
	if (PIND & (1 << PD0)) {} else {triginput |= 0x01;}
	if ((triginput & 0x03) == 0x01) {trigger = 1;}
}
