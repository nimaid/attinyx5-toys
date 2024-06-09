#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdbool.h>

#define DATA_PIN PB2
#define CLOCK_PIN PB0
#define LATCH_PIN PB1

#define UPDATE_DELAY 2500
//#define SHIFT_DELAY 1000

uint8_t invader1[] = {
	0b01110000,
	0b00011000,
	0b11111101,
	0b10110110,
	0b00111100,
	0b00111100,
	0b00111100,
	0b10110110,
	0b11111101,
	0b00011000,
	0b01110000
};
uint8_t invader2[] = {
	0b00001110,
	0b00011000,
	0b10111101,
	0b01110110,
	0b00111100,
	0b00111100,
	0b00111100,
	0b01110110,
	0b10111101,
	0b00011000,
	0b00001110
};


void shift_out(uint8_t data_out) {
	PORTB &= ~(1<<LATCH_PIN);
	
	for(int i = 0; i < 8; i++) {
		PORTB &= ~(1<<CLOCK_PIN);
		if(data_out & (1<<i)) {
			PORTB |= (1<<DATA_PIN);
		} else {
			PORTB &= ~(1<<DATA_PIN);
		}
		#ifdef SHIFT_DELAY
		_delay_us(SHIFT_DELAY);
		#endif
		
		PORTB |= (1<<CLOCK_PIN);
		#ifdef SHIFT_DELAY
		_delay_us(SHIFT_DELAY);
		#endif
	}
	
	PORTB |= (1<<LATCH_PIN);
}

void display_pattern(const uint8_t *pattern_in, int pattern_width, bool space) {
	for(int i = 0; i < pattern_width; i++) {
		shift_out(pattern_in[i]);
		_delay_us(UPDATE_DELAY);
	}
	
	if(space) {
		shift_out(0);
		_delay_us(UPDATE_DELAY);
	}
}

int main(void) {
	// Setup outputs
	DDRB = (1<<DATA_PIN) + (1<<CLOCK_PIN) + (1<<LATCH_PIN);
	
	// Main Loop
	while(1) {
		display_pattern(invader1, sizeof(invader1), true);
		display_pattern(invader2, sizeof(invader2), true);
	}
}