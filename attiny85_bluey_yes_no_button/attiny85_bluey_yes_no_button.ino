/* ATtiny85 Yes/No Button (from Bluey)
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
 * Clock Setting: Internal 8 MHz
 * 
 * Schematic:
 *       ┌───U───┐
 *     ──┤ ○     ├── Vcc
 * Yes ─3┤       ├2─  10KΩ
 *  No ─4┤       ├1───~~~~─┬─ Out
 * Gnd ──┤       ├0─       ╪ 0.1uF
 *       └───────┘         ⏚
 * For line level output, add this to the above schematic:
 *         100uF
 * Out ───┬─┤(── Line Out
 *  2.7KΩ ⌇ 
 *        ⏚
 * The above resistor may need to be made smaller to make it quieter
 * It can also be made larger or even ommitted to make it louder
 * 
 * Pin Functions:
 * • Yes / No:
 *   ◦ A digital input that triggers the respective sample on the falling edge.
 *   ◦ Uses internal pullup resistors, so you only have to connect
 *     a button between the pin and ground.
 * • Out:
 *   ◦ A high-speed PWM "analog" audio output.
 *   ◦ True analog output is realized via the RC lowpass filter.
 *
 * ~~~~~~ NOTE: ~~~~~~
 * The following section will eventually be replaced with a special Python program.
 * ~~~~~~~~~~~~~~~~~~~
 * Make new samples: http://synthworks.eu/attiny85-drum-creator/
 * Export your audio as raw mono unsigned 8-bit, around 7000 Hz.
 * Use the highest sample rate possible while still fitting it all in memory.
 * If you use a different sample rate, tweak the PITCH and ISR_SKIP values.
 */

// Tweak this between 0 and 1023 to get the correct pitch for your samples
// Higher is faster/higher pitched
#define PITCH 850
// If you need the pitch even lower, this is a coarse adjustment
// Higher is slower/lower pitched
#define ISR_SKIP 3



#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#include "samples.h"  // The sample audio data

// Output pin is hard-coded as 1 for performance
#define A_PIN 3
#define B_PIN 4

#define A_SAMPLE 0
#define B_SAMPLE 1

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

// An indexable array of the sample pointers in samples.h
const uint8_t *sample[] =
{
  sample0,
  sample1
};

// sizeof() can't work with the above array of pointers, so here is a helper array
const uint16_t sizeof_sample[] =
{
  sizeof(sample0),
  sizeof(sample1)
};

uint16_t samplecnt[2];
uint16_t samplepnt[2];


void append_ring_buffer(uint8_t sample) {
  cli();
  Ringbuffer[RingWrite]=sample;
  RingWrite++;
  RingCount++;
  sei();
}

void update_ring_buffer() {
  int16_t total=0;
  for(int i=0; i<2; i++) {
    if (samplecnt[i]) {
      total+=(pgm_read_byte_near(sample[i] + samplepnt[i]++)-128);
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
  samplecnt[sample_num]=sizeof_sample[sample_num];
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

  pinMode(A_PIN, INPUT_PULLUP);
  pinMode(B_PIN, INPUT_PULLUP);

  //Set up Timer/Counter0 for 20kHz interrupt to output samples.
  TCCR0A = 3<<WGM00;             // Fast PWM
  TCCR0B = 1<<WGM02 | 2<<CS00;   // 1/8 prescale
  TIMSK = 1<<OCIE0A;             // Enable compare match, disable overflow
  OCR0A = 49;                    // Divide by 400

  set_playback_speed(PITCH);
}

bool a_trigger;
bool b_trigger;

void loop() {
  if (RingCount<32) {  // If there is space in ringbuffer
    update_ring_buffer();

    // Detect the A input
    if (digitalRead(A_PIN) != a_trigger) {  // Detect a_trigger state change
      a_trigger = !a_trigger; // Toggle state
      if (!a_trigger) {  // If on a falling edge, trigger the sample
        play_sample(A_SAMPLE);
      }
    }

    // Detect the B input
    if (digitalRead(B_PIN) != b_trigger) {  // Detect b_trigger state change
      b_trigger = !b_trigger; // Toggle state
      if (!b_trigger) {  // If on a falling edge, trigger the sample
        play_sample(B_SAMPLE);
      }
    }
  }
}


uint8_t isr_run = 0;

ISR(TIMER0_COMPA_vect) {
  //-------------------  Ringbuffer handler -------------------------
    if(isr_run == ISR_SKIP) {
      if (RingCount) {                            // If entry in FIFO
        OCR1A = Ringbuffer[(RingRead++)];         // Output 8-bit DAC
        RingCount--;
      }

      isr_run = 0;
    }
    else {
      isr_run++;
    }
  //-----------------------------------------------------------------
}
