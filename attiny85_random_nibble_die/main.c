#include <stdbool.h>

#include <avr/io.h>
#include <util/delay.h>

#define BUTTON_BIT PB4
const uint8_t led_bits[] = {PB3, PB0, PB1, PB2};

#define LOOP_DELAY_TIME 1

uint8_t button_reads = 0;
#define PRESS_CONDITION (1<<7)     // Falling edge
#define RELEASE_CONDITION ~(1<<7)  // Rising edge

void display_nibble(uint8_t nibble_in) {
	for(int i = 0; i < 4; i++) {
		if(nibble_in & (1<<i)) {
			PORTB |= (1<<led_bits[i]);
		}
		else {
			PORTB &= ~(1<<led_bits[i]);
		}
	}
}

int main(void) {
	// Setup LEDs as outputs
	for(int i = 0; i < 4; i++) {
		DDRB |= (1<<led_bits[i]);
	}
	
	// Setup button as input (pull-up)
	DDRB |= (1<<BUTTON_BIT);
	PORTB |= (1<<BUTTON_BIT);
	
	// Declare variables
	uint8_t count = 0;
	
	// Display initial count
	display_nibble(count);
	
	// Main Loop
	while(1) {
		// On press
		if(button_reads == PRESS_CONDITION) {
			count++;
			count %= 0b10000;
			
			display_nibble(count);
		}
		
		// Update button reads
		button_reads <<= 1;
		if(PINB & (1<<BUTTON_BIT)) {
			button_reads |= 1;
		}
		
		// Delay
		#ifdef LOOP_DELAY_TIME
		_delay_ms(LOOP_DELAY_TIME);
		#endif
	}
	
	return 0;
}