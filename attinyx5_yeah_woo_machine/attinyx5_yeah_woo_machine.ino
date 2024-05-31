/* Tiny YEAH! WOO! Machine
 * Author: Ella Jameson
 * 
 * This program is free software: you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, or 
 * (at your option) any later version. 
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * GNU General Public License for more details. 
 * 
 * 
 * 
 * Clock Setting: Internal 1 MHz
 * 
 * Schematic:
 *          ┌───U───┐
 *     ─────┤ ○     ├───── Vcc
 *     ─3(D)┤       ├2(D)─ Yeah
 *     ─4(D)┤       ├1(D)───10KΩ─┬─ Out
 * Gnd ─────┤       ├0(D)─ Woo   ╪ 0.1uF
 *          └───────┘            ⏚
 * For line level output, add this to the above schematic:
 *         10uF
 * Out ───┬─┤(── Line Out
 *    1KΩ ⌇
 *        ⏚
 * 
 * Pin Functions:
 * • Pitch:
 *   ◦ An analog input that varies the sample playback speed for the
 *     entire chip.
 *   ◦ At 5v, the chip will play back at full speed. Lower voltages will
 *     result in slower playback, and therefore a lower overall pitch.
 * • Out:
 *   ◦ A high-speed PWM "analog" audio output.
 *   ◦ True analog output is realized via the RC lowpass filter.
 *
 * Make new drum kits: http://synthworks.eu/attiny85-drum-creator/
 * Export your audio as raw mono unsigned 8-bit 4000 Hz
 */


#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#include "samples.h"  // The samples to use

// Analog pins are hard-coded to be read using optimized code below
// Output pin is also hard-coded for performance
#define YEAH_PIN 2
#define WOO_PIN 0

#define YEAH_SAMPLE 0
#define WOO_SAMPLE 1

// Tweak this between 0 and 1023 to get the correct pitch for your voltage
#define PITCH 925

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


//---------- Ringbuf parameters -----------
uint8_t Ringbuffer[256];
uint8_t RingWrite=0;
uint8_t RingRead=0;
volatile uint8_t RingCount=0;
//-----------------------------------------

// An indexable array of the sample pointers in drums.h
const uint8_t *drum[] =
{
  drum0,
  drum1,
  drum2,
  drum3
};

// sizeof() can't work with the above array of pointers, so here is a helper array
const uint16_t sizeof_drum[] =
{
  sizeof(drum0),
  sizeof(drum1),
  sizeof(drum2),
  sizeof(drum3)
};

uint16_t samplecnt[4];
uint16_t samplepnt[4];


void append_ring_buffer(uint8_t sample) {
  cli();
  Ringbuffer[RingWrite]=sample;
  RingWrite++;
  RingCount++;
  sei();
}

void update_ring_buffer() {
  int16_t total=0;
  for(int i=0; i<4; i++) {
    if (samplecnt[i]) {
      total+=(pgm_read_byte_near(drum[i] + samplepnt[i]++)-128);
      samplecnt[i]--;
    }
  }
  total>>=1;
  total+=128;  
  if (total>255) total=255;
  append_ring_buffer(total);
}

void play_sample(uint8_t sample_num) {
  samplepnt[sample_num]=0;
  samplecnt[sample_num]=sizeof_drum[sample_num];
}


// 0-1023, 1023 is full
void set_playback_speed(uint16_t playback_speed) {
  OCR0A = 49+((127- (playback_speed>>3) ));
}



void setup() {
  OSCCAL=255;
  // Enable 64 MHz PLL and use as the source for Timer1
  PLLCSR = 1<<PCKE | 1<<PLLE;     

  // Set up Timer/Counter1 for PWM output
  TIMSK = 0;                                // Timer interrupts OFF
  TCCR1 = 1<<PWM1A | 2<<COM1A0 | 1<<CS10;   // PWM A, clear on match, 1:1 prescale
  //GTCCR = 1<<PWM1B | 2<<COM1B0;             // PWM B, clear on match
  OCR1A = 128; //OCR1B = 128;               // 50% duty at start

  pinMode(1, OUTPUT);  // Enable PWM output pin

  pinMode(YEAH_PIN, INPUT_PULLUP);
  pinMode(WOO_PIN, INPUT_PULLUP);

  //Set up Timer/Counter0 for 20kHz interrupt to output samples.
  TCCR0A = 3<<WGM00;             // Fast PWM
  TCCR0B = 1<<WGM02 | 2<<CS00;   // 1/8 prescale
  TIMSK = 1<<OCIE0A;             // Enable compare match, disable overflow
  OCR0A = 49;                    // Divide by 400

  set_playback_speed(PITCH);
}

bool yeah_trigger;
bool woo_trigger;

void loop() {
  if (RingCount<32) {  // If there is space in ringbuffer
    update_ring_buffer();

    // Detect the YEAH! input
    if (digitalRead(YEAH_PIN) != yeah_trigger) {  // Detect yeah_trigger state change
      yeah_trigger = !yeah_trigger; // Toggle state
      if (!yeah_trigger) {  // If on a falling edge, trigger the sample
        play_sample(YEAH_SAMPLE);
      }
    }

    // Detect the WOO! input
    if (digitalRead(WOO_PIN) != woo_trigger) {  // Detect woo_trigger state change
      woo_trigger = !woo_trigger; // Toggle state
      if (!woo_trigger) {  // If on a falling edge, trigger the sample
        play_sample(WOO_SAMPLE);
      }
    }
  }
}


ISR(TIMER0_COMPA_vect) {
  //-------------------  Ringbuffer handler -------------------------
    
    if (RingCount) {                            // If entry in FIFO
      OCR1A = Ringbuffer[(RingRead++)];         // Output 8-bit DAC
      RingCount--;
    }

  //-----------------------------------------------------------------
}
