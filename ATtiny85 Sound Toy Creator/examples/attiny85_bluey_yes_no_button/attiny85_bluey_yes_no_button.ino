/* ATtiny85 Sound Toy w/ Advanced Power Management
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
 * ~~~~~~ Board Settings (Arduino) ~~~~~~
 * Internal 8 MHz
 * Disable millis()/micros() to save space
 * Disable B.O.D. to save power
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * ~~~~~~ Fuses (avrdude) ~~~~~~
 * efuse: 0xFF
 * hfuse: 0xD7
 * lfuse: 0xE2
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 
 * ~~~~~~ Schematic ~~~~~~
 *      ┌───U───┐
 *    ──┤ ○     ├── Vcc
 * S0 ─3┤       ├2─ !AMP_SHUTDOWN
 * S1 ─4┤       ├1────~~──┬─ Out
 *   ┌──┤       ├0─  10KΩ ╪ 0.1uF
 *   ⏚  └───────┘         ⏚
 * 
 * Key:
 * • S0 / S1:
 *   ◦ A digital input that triggers the respective sample on the falling edge.
 *   ◦ Uses internal pullup resistors, so you only have to connect
 *     a button between the pin and ground.
 * • !AMP_SHUTDOWN:
 *   ◦ A digital output that is designed to be connected to the shutdown pin on the PAM8302/PAM8403.
 *   ◦ Is LOW when the controller is asleep, and HIGH IMPEDANCE (disconnected) when it's awake
 * • Out:
 *   ◦ A high-speed PWM "analog" audio output.
 *   ◦ True analog output is realized via the RC lowpass filter.
 *
 * Good practice is to always include a 0.1uF ceramic capacitor between VCC and GND (called a decoupling capacitor).
 * Make sure this is connected DIRECTLY to the ATtiny85 power pin, and make the wires/traces as short as possible.
 *
 * While you could connect the buttons directly between the pins and ground, you will get false presses due to button bouncing.
 * To save flash memory, no software debouncing is used. Instead, you can fix the problem with more RC filters, like so:
 *   _T_  10kΩ
 * ┌─○ ○───~~─┬─ Pin
 * ⏚          ╪ 0.1uF
 *            ⏚
 *
 * For line level output, add this to the above schematic:
 *         10uF
 * Out ───┬─┤(── Line Out
 *  2.7KΩ ⌇
 *        ⏚
 * The above resistor may need to be made smaller to make it quieter
 * It can also be made larger or even ommitted to make it louder
 * If connecting to a PAM8302 or PAM8403, you can ommit the cap and just use the suggested circuit from the datasheet.
 * ~~~~~~~~~~~~~~~~~~~~~~~
 * 
 * ~~~~~~ Current Draw ~~~~~~
 * Measurements do not include a speaker load, but do include a PAM8403.
 * Awake: 10.9 mA
 * Asleep: 0.3 uA
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 
 * ~~~~~~ Making New Sample Kits ~~~~~~
 * NOTE: This section will eventually be replaced with a special Python program.
 * Make new samples: http://synthworks.eu/attiny85-drum-creator/
 * Export your audio as raw mono unsigned 8-bit, around 7000 Hz.
 * Sample Rate = (Free Bytes - 2) / Total Audio Length (seconds)
 * At 7000 Hz with 6698 free bytes, you get 956.6 ms of audio total
 * Use the highest sample rate possible while still fitting it all in memory.
 * It's suggested to aggressively trim and fade the clips to make them shorter, allowing higher sample rates.
 * If you use a different sample rate, tweak the PITCH and ISR_SKIP_SAMPLES values.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 
 * ~~~~~~ Technical Notes ~~~~~~
 * https://ww1.microchip.com/downloads/en/devicedoc/atmel-2586-avr-8-bit-microcontroller-attiny25-attiny45-attiny85_datasheet.pdf
 * http://synthworks.eu/attiny85-drum-creator/
 * https://www.gadgetronicx.com/attiny85-sleep-modes-tutorial/
 * https://thewanderingengineer.com/2014/08/11/arduino-pin-change-interrupts/
 * https://thewanderingengineer.com/2014/08/09/avr-arduino-default-isr-resetting-pin-change-interrupt-problem/
 * http://www.gammon.com.au/power
 * https://www.mouser.com/datasheet/2/115/PAM8403-247318.pdf
 * https://www.diodes.com/assets/Datasheets/PAM8302A.pdf
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

// Sample Kit: Bluey Yes/No Button
// Sample Mapping:
// ┌───┬────────┐
// │ # │ Sample │
// ├───┼────────┤
// │ 0 │   Yes  │
// │ 1 │   No   │
// └───┴────────┘

// Tweak this between 0 and 1023 to get the correct pitch for your samples
// Higher is faster/higher pitched
#define PITCH 840
// If you need the pitch even lower, this is a coarse adjustment
// Higher is slower/lower pitched
#define ISR_SKIP_SAMPLES 3



//#define FAST_SLEEP  // Used to make the chip sleep after only 5 seconds of inactivity
//#define USE_BLANK_SAMPLES // Used to compile without samples to get a baseline sketch size



#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#ifdef USE_BLANK_SAMPLES
#include "blank_samples.h"  // Empty samples that take up no flash
#else
#include "samples.h"  // The sample audio data
#endif
#define NUM_SAMPLES 2

// Which pins will trigger each sample, tied to hard-coded register settings for wakeup interrupts
uint8_t trigger_pins[] = {3, 4};

// Which pin is used for shutting down the amp (could also be an "awake" indicator LED)
#define AMP_SHUTDOWN_PIN 2

// Hacky way to measure time, used for the sleep mode timeout and the wakeup period
// millis() doesn't work if I use Timer0 myself, so I had to roll my own solution
// I'll be honest, I can't be bothered to do the math to get accurate time
// This value gives something very *close* to milliseconds, but the clock runs a little slow
#define ISR_SKIP_CLOCK 1<<5 // How many ISR cycles before the clock is incremented
volatile uint32_t clock = 0;  // This is effectively the "time" in "ISR_SKIP_CLOCKs", not milliseconds

#ifdef FAST_SLEEP
#define SLEEP_TIMEOUT 5000  // How long to wait before sleeping in debug mode (5 seconds)
#else
#define SLEEP_TIMEOUT 150000  // How long to wait before sleeping in production mode (2.5 minutes)
#endif
#define WAKEUP_WINDOW 400  // How long to wait before playing samples after waking up (PWM and amp startup time)

bool sample_queue[NUM_SAMPLES] = {false};

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
const uint8_t *sample[NUM_SAMPLES] =
{
  sample0,
  sample1
};

// sizeof() can't work with the above array of pointers, so here is a helper array
const uint16_t sizeof_sample[NUM_SAMPLES] =
{
  sizeof(sample0),
  sizeof(sample1)
};

uint16_t samplecnt[NUM_SAMPLES];
uint16_t samplepnt[NUM_SAMPLES];

bool pin_state[NUM_SAMPLES] = {HIGH};
uint32_t isr_run_sample = 0;
uint32_t isr_run_clock = 0;

volatile uint32_t last_button_press_time = 0;
volatile uint32_t last_wakeup_time = 0;
volatile bool still_waking_up = false;

uint8_t i;



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

  // Set up Timer/Counter0 for 20kHz interrupt to output samples.
  TCCR0A = 3<<WGM00;             // Fast PWM
  TCCR0B = 1<<WGM02 | 2<<CS00;   // 1/8 prescale
  TIMSK = 1<<OCIE0A;             // Enable compare match, disable overflow
  OCR0A = 49;                    // Divide by 400

  set_playback_speed(PITCH);

  // Setup pins as inputs
  for(i=0; i<NUM_SAMPLES; i++) {
    pinMode(trigger_pins[i], INPUT_PULLUP);
  }

  pinMode(AMP_SHUTDOWN_PIN, INPUT);  // High impedance (disconnected)

  disable_unused_peripherals();

  disable_pin_interrupts();
}

void loop() {
  // If there is space in ringbuffer, update it
  if(RingCount < 32) {
    update_ring_buffer();
  }

  // Detect the edge-triggered inputs
  for(i=0; i<NUM_SAMPLES; i++) {
    // Detect this pin state change
    if (digitalRead(trigger_pins[i]) != pin_state[i]) {  
      pin_state[i] = !pin_state[i];  // Toggle state
      // If on a falling edge, trigger the sample
      if (!pin_state[i]) {
        last_button_press_time = clock;

        // If still waking up, queue instead of playing
        if(still_waking_up) {
          sample_queue[i] = true;
        }
        else {
          play_sample(i);
        }
      }
    }
  }

  // If still_waking_up and out of the time window, play queue, clear queue, and reset still_waking_up
  if(still_waking_up && (clock - last_wakeup_time >= WAKEUP_WINDOW)) {
    still_waking_up = false;

    for(i=0; i<NUM_SAMPLES; i++) {
      if(sample_queue[i]) {
        play_sample(i);
      }
        sample_queue[i] = false;
    }
  }

  // Sleep if enough time has passed
  if(clock - last_button_press_time > SLEEP_TIMEOUT) {
    sleep();
  }
}



ISR(TIMER0_COMPA_vect) {
  //-------------------  Ringbuffer handler -------------------------
    if(isr_run_sample == ISR_SKIP_SAMPLES) {
      isr_run_sample = 0;

      if(RingCount) {                            // If entry in FIFO
        OCR1A = Ringbuffer[(RingRead++)];        // Output 8-bit DAC
        RingCount--;
      }
    }
    else {
      isr_run_sample++;
    }
  //-----------------------------------------------------------------

  //---------------------  Clock handler ----------------------------
    if(isr_run_clock == ISR_SKIP_CLOCK) {
      isr_run_clock = 0;

      clock++;
    }
    else {
      isr_run_clock++;
    }
  //-----------------------------------------------------------------
}

// Wakeup handler
ISR(PCINT0_vect) {  // Handles ALL pins on PORTB (on the ATtiny85, that's every pin!)
  wake_up();
}

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

void enable_pin_iterrupts() {
  GIMSK |= (1 << PCIE);  // Enable extrnal interrupt on all enabled pins
  PCMSK |= (1 << PCINT3) | (1 << PCINT4);  // Enable wakeup interrupts
  //MCUCR |= (1 << ISC01);  // Change mode to falling edge detection
}

void disable_pin_interrupts() {
  GIMSK &= ~(1 << PCIE);  // Disable extrnal interrupt on all enabled pins
  PCMSK &= ~(1 << PCINT3) & ~(1 << PCINT4);  // Disable wakeup interrupts
}

void sleep()
{ 
  pinMode(AMP_SHUTDOWN_PIN, OUTPUT);
  digitalWrite(AMP_SHUTDOWN_PIN, LOW);

  enable_pin_iterrupts();

  MCUCR |= (1 << SM1);      // Enabling sleep mode and powerdown sleep mode
  MCUCR |= (1 << SE);       // Enabling sleep enable bit

  __asm__ __volatile__ ( "sleep" "\n\t" :: );  // Sleep instruction to put controller to sleep
}

void wake_up() {
  disable_pin_interrupts();

  pinMode(AMP_SHUTDOWN_PIN, INPUT);  // High impedance (disconnected)

  MCUCR &= ~(1 << SM1);      // Disabling sleep mode and powerdown sleep mode
  MCUCR &= ~(1 << SE);       // Disabling sleep enable bit

  // Reset the clock to prevent an eventual overflow (that would take WEEKS of being awake but still)
  // We can get away with this because we only ever care about time relative to the last wake up
  clock = 0;

  // Set flags and timestamps
  still_waking_up = true;
  last_button_press_time = clock;
  last_wakeup_time = clock;
}

// Turn off unused peripherals to save lots of power, especially when sleeping
void disable_unused_peripherals() {
  // Disable and shut down the ADC (unused)
  ADCSRA = 0;  // This single line improves the sleeping battery life by about 1000x
  PRR |= (1 << PRADC);

  // Disable the Universal Serial Interface (unused)
  PRR |= (1 << PRUSI);
}

/*
ISR(BADISR_vect)
{
  // Do nothing, overwritting default reset behaviors
}
*/