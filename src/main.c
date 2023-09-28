/*****************************************************************************/
/*                                                                           */
/* Filename: main.c                                                          */
/* Begin:    21.05.2018                                                      */
/* Author:   Kertész Csaba-Zoltán                                            */
/* E-mail:   csaba.kertesz@unitbv.ro                                         */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Description                                                               */
/*   - source file containing main system routines                           */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Change history:                                                           */
/*                                                                           */
/*   21.05.2018: - first implementation                                      */
/*   2022.05.06: - converted to programmable frequency generator             */
/*                                                                           */
/*****************************************************************************/

/**** include files **********************************************************/

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdint.h>

#include "config.h"

/**** local function prototypes **********************************************/
void init_uc(void);

/**** constants **************************************************************/

// code points
const PROGMEM uint8_t CODE_MO[6]  = { 0xee, 0x3b, 0xb8, 0 }; // 1110 1110 0011 1011 1011 1
const PROGMEM uint8_t CODE_MOE[6] = { 0xee, 0x3b, 0xb8, 0x80, 0 }; // 1110 1110 0011 1011 1011 1000 1
const PROGMEM uint8_t CODE_MOI[6] = { 0xee, 0x3b, 0xb8, 0xa0, 0 }; // 1110 1110 0011 1011 1011 1000 101
const PROGMEM uint8_t CODE_MOS[6] = { 0xee, 0x3b, 0xb8, 0xa8, 0 }; // 1110 1110 0011 1011 1011 1000 1010 1
const PROGMEM uint8_t CODE_MOH[6] = { 0xee, 0x3b, 0xb8, 0xaa, 0 }; // 1110 1110 0011 1011 1011 1000 1010 101
const PROGMEM uint8_t CODE_MO5[6] = { 0xee, 0x3b, 0xb8, 0xaa, 0x80, 0 }; // 1110 1110 0011 1011 1011 1000 1010 1010 1
const PROGMEM uint8_t CODE_S[6]   = { 0xa8, 0 }; // 1010 1

#define CODE_MO_LEN 21
#define CODE_MOE_LEN 25
#define CODE_MOI_LEN 27
#define CODE_MOS_LEN 29
#define CODE_MOH_LEN 31
#define CODE_MO5_LEN 23
#define CODE_S_LEN 5

// TODO: I have to find a better solution for this
const PROGMEM uint8_t space_adjust[32] = {
  10, 7, 8, 4,
  6, 17, 4, 7,
  11, 15, 10, 5,
  9, 13, 10, 13,
  7, 11, 8, 11,
  5, 9, 6, 9,
  9, 5, 5, 7,
  10, 7, 8, 4,
};

#define SPACE_ADJUST(len,wpm,i) \
  ((INTERVAL_COUNT(i) + (7 * TICKS_PER_SIGN(wpm))) % (((len) + 7) * TICKS_PER_SIGN(wpm))) / \
  (TICKS_PER_SIGN(wpm)) / \
  ((INTERVAL_COUNT(i) + (7 * TICKS_PER_SIGN(wpm))) / (((len) + 7) * TICKS_PER_SIGN(wpm)))

#ifdef USE_PROG_FREQ
const PROGMEM uint16_t frequencies[8] = {803, 810, 813, 820, 824, 827, 834, 837};
#endif

/**** global variables *******************************************************/

uint8_t code[6];
uint8_t* code_ptr;
uint8_t ticks_per_sign;
uint16_t enable_period;
uint16_t interval;
uint8_t output_set;
uint8_t output;
uint16_t interval_ticks;
uint8_t key_ticks;
uint8_t space_count;
uint8_t bit;
uint8_t space;
#ifdef USE_LED
uint16_t led_ticks;
#endif
#ifdef USE_PROG_FREQ
uint16_t frequency;
#endif

/**** local functions ********************************************************/

/*===========================================================================*/
/*  Function: TIMER0_COMPA                                                   */
/*  Module:   main                                                           */
/*===========================================================================*/
/*  Parameters:                                                              */
/*        - none                                                             */
/*  Return value:                                                            */
/*        - none                                                             */
/*===========================================================================*/
/*  Description:                                                             */
/*    Interrupt service routine for Timer0 Compare A module                  */
/*    interrupt is executed periodically at every 8ms                        */
/*                                                                           */
/*    ATtiny261 does not have CTC functionality for 16bit mode, so only the  */
/*    8 bit timer is used with CTC, so there is not enough resolution to     */
/*    have a period equal to the morse code signs, these must be delayed     */
/*    by software, counting enough periods for each sign                     */
/*    when enough ticks are passed the next sign is determined and the       */
/*    keying will be either turned on or turned off                          */
/*    the periods are also counted for the interval timer, turning on/off    */
/*    the whole transmitter                                                  */
/*                                                                           */
/*===========================================================================*/
ISR(TIMER0_COMPA_vect)
{
  // check if output is enabled
  if (!interval || interval_ticks++ < enable_period)
  {
    // set output enable pin if necessary
    output |= OUTPUT_ENABLE;

    // in interval mode, key the transmitter for the last 2 seconds
    if (interval && interval_ticks > enable_period - TXOFF_TICKS)
    {
      output |= OUTPUT_KEY;
    }
    if (!key_ticks--)
    {
      key_ticks = ticks_per_sign - 1;

      if ( space_count >= space )
      {
        code_ptr = code;
        bit = 0x80;
        space_count = 0;
      }
      if ( *code_ptr & bit )
      {
        output |= OUTPUT_KEY;
        space_count = 0;
      }
      else
      {
        output &= ~OUTPUT_KEY;
        space_count++;
      }
      bit >>= 1;
      if ( !bit )
      {
        if ( *code_ptr )
          code_ptr++;
        bit = 0x80;
      }
    }

#ifdef USE_LED
    if (led_ticks >= ENABLED_LED_TICKS)
      led_ticks = 0;
#endif
  }
  else
  {
    output &= ~(OUTPUT_ENABLE | OUTPUT_KEY);
    space_count = space + 1;
    key_ticks = 0;

#ifdef USE_LED
    if (led_ticks >= enable_period)
      led_ticks = 0;
#endif
  }

#ifdef USE_LED
  if (led_ticks++ != 0)
  {
    LED_OFF();
  }
  else
  {
    LED_ON();
  }
#endif

  OUTPUT(output ^ output_set);
}


/*===========================================================================*/
/*  Function: init_uc                                                        */
/*  Module:   main                                                           */
/*===========================================================================*/
/*  Parameters:                                                              */
/*        - none                                                             */
/*  Return value:                                                            */
/*        - none                                                             */
/*===========================================================================*/
/*  Description:                                                             */
/*    initialize microcontroller                                             */
/*    input ports for the dip-switches, output for keying and LED            */
/*    period timer for 8ms interrupt                                         */
/*    also read in the dip-switches, to determine the timing configuration   */
/*===========================================================================*/
void init_uc(void)
{
  const uint8_t * ptr;
  uint8_t * dst = code;
  register uint8_t intervals;
  int i;

  // analog comparator is not used, disable to reduce power
  ACSRA = (1 << ACD);
  // also allow reducing power for all but Timer0 (and USI if needed)
  PRR = (1 << PRTIM1) |
#ifndef USE_SPI
        (1 << PRUSI) |
#endif
        (1 << PRADC);

  // setup ports:
  // porta all input with pullup
  PORTA = 0xFF;
  DDRA = 0x00;
  // some PORTB pins have alternate functionality, some have output pins
  // and some have config bits on them, including some multiple use pins
  // initially set only the inputs, to determine the default states
  PORTB = PORTB_DIP_PINS;
  DDRB = 0x00;

  // read dip-switch settings
  // code speed
  if (DIP(SPEED) != 0)
    ticks_per_sign = TICKS_PER_SIGN(CODE_SPEED_FAST);
  else
    ticks_per_sign = TICKS_PER_SIGN(CODE_SPEED_SLOW);

  // interval period short/long
  if (DIP(INTERVAL_LENGTH) != 0)
  {
    enable_period = INTERVAL_COUNT(INTERVAL_SHORT);
  }
  else
  {
    enable_period = INTERVAL_COUNT(INTERVAL_LONG);
  }

  // interval mode
  SET_INTERVAL_VALUE(interval, enable_period);

  if (interval)
  {
    // adjust the interword spacing we get full words in an interval
    space = pgm_read_byte(&space_adjust[DIP(INTERVAL) & 0x03]);
  }
  else
  {
    enable_period = 0;
    space = 7;
  }

    // D1-3 (PA7, PA6, PA5) code
  switch ( DIP(CODE) )
  {
    case DIP_CODE_MOE:  ptr = CODE_MOE; intervals = 0; break;
    case DIP_CODE_MOI:  ptr = CODE_MOI; intervals = 1; break;
    case DIP_CODE_MOS:  ptr = CODE_MOS; intervals = 2; break;
    case DIP_CODE_MOH:  ptr = CODE_MOH; intervals = 3; break;
    case DIP_CODE_MO5:  ptr = CODE_MO5; intervals = 4; break;
    case DIP_CODE_S:    ptr = CODE_S;   intervals = 0; break;
    default: ptr = CODE_MO;  intervals = 0;
  }
  for (i = 0; i < 6; i++)
    *dst++ = pgm_read_byte(ptr++);

  if (intervals)
  {
    interval_ticks = interval;
    do
    {
      interval_ticks -= enable_period;
    } while (intervals-- && interval_ticks > enable_period);
  }

  // frequency setting
#ifdef USE_PROG_FREQ
  frequency = pgm_read_word(&frequencies[DIP(FREQ)]);
#endif

  // if we write back the dip-switch settings to the port pin
  // we disable the pullups on switches which are already connected
  // to the ground
  // normally during functioning, the dip switches are not altered
  // so we can reduce the current through the pullups on these
  // switches
  // the off switches shall preserve the pullup, to have the PIN on
  // a stable level
  // this reduces power consumption by 100uA for every turned on switch
  PORTA = PINA;

  // key level, enable level
#ifdef USE_LEVEL_SETTING
  output_set = (DIP(KEY_LEVEL) != 0 ? 0 : OUTPUT_KEY) | (DIP(ENABLE_LEVEL) != 0 ? 0 : OUTPUT_ENABLE);
#endif
  PORTB = output_set |
#ifdef USE_SPI
          USI_DO | OSC_SEN |
#endif
          (PINB & PORTB_DIP_PINS);

#ifdef USE_LTC6309
  DDRB = USI_DO | USI_SCK | OSC_SEN;

  // configure oscillator using SPI interface
  // high byte (OCT3:0 DAC9:6) OCT = 11, DAC from frequency
  PORTB &= ~OSC_SEN;
  USIDR = (11 << 4) | (frequency >> 6);
  USISR = 1 << USIOIF;

  do
  {
    USICR = (1 << USIWM0) | (1 << USICS1) | (1 << USICLK) | (1 << USITC);
  } while (!(USISR & (1 << USIOIF)));
  // low byte (DAC5:0 CNF1:0) DAC from frecuency, CNF 10 (CLKN disabled)
  USIDR = ((frequency & 0x3F) << 2) | 0x02;
  USISR = 1 << USIOIF;
  do
  {
    USICR = (1 << USIWM0) | (1 << USICS1) | (1 << USICLK) | (1 << USITC);
  } while (!(USISR & (1 << USIOIF)));
  PORTB |= OSC_SEN | USI_SCK | USI_DO;
  DDRB &= ~(USI_DO | USI_SCK | OSC_SEN);

  // disable USI, and set port to pullups where needed
  USICR = 0;
  PRR |= (1 << PRUSI);
  // clear pullup if USI_DO pin is pulled down
  if (!(PINB & USI_DO))
    PORTB &= ~USI_DO;
#endif

  DDRB |=
#ifdef USE_LED
          OUTPUT_LED |
#endif
          OUTPUT_ENABLE | OUTPUT_KEY;

  // setup timer: CTC interrupt at 8ms
  TCCR0A = 0x01;
  TCCR0B = 0x04;
  OCR0A = TICKS_PER_SECOND - 1;
  TIMSK = 1 << OCIE0A;
}


/**** global functions *******************************************************/

/*===========================================================================*/
/*  Function: main                                                           */
/*  Module:   main                                                           */
/*===========================================================================*/
/*  Parameters:                                                              */
/*        - none                                                             */
/*  Return value:                                                            */
/*        - none                                                             */
/*===========================================================================*/
/*  Description:                                                             */
/*    main program loop                                                      */
/*===========================================================================*/
int main(void)
{
  // initialize the system
  init_uc();

  // and from now on work only in the IRQ, so enable the interrupts
  sei();

  // then do nothing
  while (1)
  {
    sleep_mode();
  }
  

  return 0;
}


