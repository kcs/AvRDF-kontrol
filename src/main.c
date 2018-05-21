/*****************************************************************************/
/*                                                                           */
/* Filename: main.c                                                          */
/* Begin:    21.05.2018                                                      */
/* Author:   Kertész Csaba-Zoltán                                            */
/* E-mail:   csaba.kertesz@vega.unitbv.ro                                    */
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
/*                                                                           */
/*****************************************************************************/

/**** include files **********************************************************/

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdint.h>

/**** local function prototypes **********************************************/
void init_uc(void);

/**** constants **************************************************************/

// code speed settings
// normally 10 WPM for slow speed and 15 WPM for high speed
#define CODE_SPEED_SLOW 10
#define CODE_SPEED_FAST 15

// usual ON times (in seconds):
#define INTERVAL_SHORT 12
#define INTERVAL_LONG  60

// output bits
#define OUTPUT_PORT PORTB
#define OUTPUT_ENABLE_PIN 2
#define OUTPUT_ENABLE (1 << OUTPUT_ENABLE_PIN)
#define OUTPUT_KEY_PIN 3
#define OUTPUT_KEY (1 << OUTPUT_KEY_PIN)

// code points
const PROGMEM uint8_t CODE_MO[6]  = { 0xee, 0x3b, 0xb8, 0 }; // 1110 1110 0011 1011 1011 1
const PROGMEM uint8_t CODE_MOE[6] = { 0xee, 0x3b, 0xb8, 0x80, 0 }; // 1110 1110 0011 1011 1011 1000 1
const PROGMEM uint8_t CODE_MOI[6] = { 0xee, 0x3b, 0xb8, 0xa0, 0 }; // 1110 1110 0011 1011 1011 1000 101
const PROGMEM uint8_t CODE_MOS[6] = { 0xee, 0x3b, 0xb8, 0xa8, 0 }; // 1110 1110 0011 1011 1011 1000 1010 1
const PROGMEM uint8_t CODE_MOH[6] = { 0xee, 0x3b, 0xb8, 0xaa, 0 }; // 1110 1110 0011 1011 1011 1000 1010 101
const PROGMEM uint8_t CODE_MO5[6] = { 0xee, 0x3b, 0xb8, 0xaa, 0x80, 0 }; // 1110 1110 0011 1011 1011 1000 1010 1010 1
const PROGMEM uint8_t CODE_S[6]   = { 0xa8, 0 }; // 1010 1


/**** macros *****************************************************************/
#define TICKS_PER_SIGN(wpm) 150 / (wpm)

// interval count normal values:
// slow/short: 100, slow/long: 500
// fast/short: 150, fast/long: 750
#define INTERVAL_COUNT(WPM, I) (WPM) * (I) * 1000ul / 1200

#define OUTPUT(out) OUTPUT_PORT = (OUTPUT_PORT & ~(OUTPUT_ENABLE | OUTPUT_KEY)) | ((out) & (OUTPUT_ENABLE | OUTPUT_KEY))


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
  // check if on/off mode enabled
  if (interval)
  {
    if (interval_ticks++ < enable_period)
      output |= OUTPUT_ENABLE;
    else
      output &= ~OUTPUT_ENABLE;
    if (interval_ticks >= interval)
      interval_ticks = 0;
  }
  else
    output |= OUTPUT_ENABLE;

  if (output & OUTPUT_ENABLE)
  {
    if (key_ticks--)
    {
      key_ticks = ticks_per_sign - 1;

      if ( space_count >= 7 )
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
  }
  else
  {
    output &= ~OUTPUT_KEY;
    space_count = 8;
  }

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
  register uint8_t intervals;

  // setup ports:
  // porta all input with pullup
  PORTA = 0xFF;
  DDRA = 0x00;
  // portb 0,1 input with pullup, 2,3,6 push-pull output, others alternate
  // initially set only the inputs, to determine the default states
  PORTB = 0x03;
  DDRB = 0x00;

  // read dip-switch settings
  // D8 speed (PA0)
  if ( PINA & 1 )
    ticks_per_sign = TICKS_PER_SIGN(CODE_SPEED_FAST);
  else
    ticks_per_sign = TICKS_PER_SIGN(CODE_SPEED_SLOW);

  // D4 (PA4) continuous/interval
  if ( PINA & (1<<4) )
  {
    // interval
    // D5 (PA3) - short/long
    if ( PINA & (1<<3) )
    {
      if ( PINA & (1<<0) )
        enable_period = INTERVAL_COUNT(CODE_SPEED_FAST, INTERVAL_SHORT);
      else
        enable_period = INTERVAL_COUNT(CODE_SPEED_SLOW, INTERVAL_SHORT);
    }
    else
    {
      if ( PINA & (1<<0) )
        enable_period = INTERVAL_COUNT(CODE_SPEED_FAST, INTERVAL_LONG);
      else
        enable_period = INTERVAL_COUNT(CODE_SPEED_SLOW, INTERVAL_LONG);
    }
    // D6-D7 (PA2, PA1) number of intervals
    intervals = ((PINA >> 1) & 0x03);
    interval = enable_period + enable_period;
    while (intervals--)
      interval += enable_period;
  }

    // D1-3 (PA7, PA6, PA5) code
  switch ( ( PINA >> 5 ) & 0x07 )
  {
    case 1:  memcpy_P(code, (PGM_P)pgm_read_word(&CODE_MOE), 6); intervals = 0; break;
    case 2:  memcpy_P(code, (PGM_P)pgm_read_word(&CODE_MOI), 6); intervals = 1; break;
    case 3:  memcpy_P(code, (PGM_P)pgm_read_word(&CODE_MOS), 6); intervals = 2; break;
    case 4:  memcpy_P(code, (PGM_P)pgm_read_word(&CODE_MOH), 6); intervals = 3; break;
    case 5:  memcpy_P(code, (PGM_P)pgm_read_word(&CODE_MO5), 6); intervals = 4; break;
    case 6:  memcpy_P(code, (PGM_P)pgm_read_word(&CODE_S), 6);   intervals = 0; break;
    default: memcpy_P(code, (PGM_P)pgm_read_word(&CODE_MO), 6);  intervals = 0;
  }
  if (intervals)
  {
    interval_ticks = interval;
    do
    {
      interval_ticks -= enable_period;
    } while (intervals-- && interval_ticks > enable_period);
  }

  // D9 (PB0) key level, D10 (PB1) enable level
  output_set = ((PINB & (1 << 0)) << OUTPUT_KEY_PIN) | (((PINB & (1 << 1)) >> 1) << OUTPUT_ENABLE_PIN);
  OUTPUT(~output_set);
  DDRB |= OUTPUT_ENABLE | OUTPUT_KEY;

  // setup timer: CTC interrupt at 8ms
  TCCR0A = 0x01;
  TCCR0B = 0x04;
  OCR0A = 125 - 1;
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
    ;
  // here also some power management can be applied

  return 0;
}


