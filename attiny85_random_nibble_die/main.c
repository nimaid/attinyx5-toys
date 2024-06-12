#include <stdbool.h>

#include <avr/io.h>
#include <util/delay.h>

#define BUTTON_BIT PB4
const uint8_t led_bits[] = {PB3, PB0, PB1, PB2};

#define DELAY_TIME 1000

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
	bool button_pressed = false;
	
	// Display initial count
	display_nibble(count);
	
	// Main Loop
	while(1) {
		// If the button is not pressed (high)
		if(PINB & (1<<BUTTON_BIT)) {
			button_pressed = false;
		}
		// If the button is pressed (low)
		else {
			// On press (falling edge)
			if(!button_pressed) {
				count++;
				count %= 0b10000;
				
				display_nibble(count);
			}
			
			button_pressed = true;
		}
	}
	
	return 0;
}