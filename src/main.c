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
#include <avr/sleep.h>
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
#define OUTPUT_LED_PIN 6
#define OUTPUT_LED (1 << OUTPUT_LED_PIN)

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

// in interval mode, mark end of transmission cycle with 2s tone
#define TXOFF_TICKS 250

/**** macros *****************************************************************/
#define TICKS_PER_SIGN(wpm) 150 / (wpm)

// interval count normal values:
// slow/short: 100, slow/long: 500
// fast/short: 150, fast/long: 750
#define INTERVAL_COUNT(I) (I) * 125

#define OUTPUT(out) OUTPUT_PORT = (OUTPUT_PORT & ~(OUTPUT_ENABLE | OUTPUT_KEY)) | ((out) & (OUTPUT_ENABLE | OUTPUT_KEY))
#define LED_ON() OUTPUT_PORT |= OUTPUT_LED
#define LED_OFF() OUTPUT_PORT &= ~OUTPUT_LED

#define ENABLED_LED_TICKS (125)
#define DISABLED_LED_TICKS (60 * 125)

#define SPACE_ADJUST(len,wpm,i) \
  ((INTERVAL_COUNT(i) + (7 * TICKS_PER_SIGN(wpm))) % (((len) + 7) * TICKS_PER_SIGN(wpm))) / \
  (TICKS_PER_SIGN(wpm)) / \
  ((INTERVAL_COUNT(i) + (7 * TICKS_PER_SIGN(wpm))) / (((len) + 7) * TICKS_PER_SIGN(wpm)))

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
uint16_t led_ticks;


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

    if (led_ticks >= ENABLED_LED_TICKS)
      led_ticks = 0;
  }
  else
  {
    output &= ~OUTPUT_KEY;
    space_count = space + 1;
    key_ticks = 0;

    if (led_ticks >= enable_period)
      led_ticks = 0;
  }

  if (led_ticks++)
    LED_OFF();
  else
    LED_ON();

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
  // also allow reducing power for all but Timer0
  PRR = (1 << PRTIM1) | (1 << PRUSI) | (1 << PRADC);

  // setup ports:
  // porta all input with pullup
  PORTA = 0xFF;
  DDRA = 0x00;
  // portb 0,1 input with pullup, 2,3,6 push-pull output, others alternate
  // initially set only the inputs, to determine the default states
  PORTB = 0x03;
  DDRB = 0x00;

  // read dip-switch settings
  // D8 speed (PA4)
  if ( PINA & (1<<4) )
    ticks_per_sign = TICKS_PER_SIGN(CODE_SPEED_FAST);
  else
    ticks_per_sign = TICKS_PER_SIGN(CODE_SPEED_SLOW);

  // D4 (PA3) continuous/interval
  if ( PINA & (1<<3) )
  {
    // interval
    // D5 (PA2) - short/long
    if ( PINA & (1<<2) )
    {
      enable_period = INTERVAL_COUNT(INTERVAL_SHORT);
    }
    else
    {
      enable_period = INTERVAL_COUNT(INTERVAL_LONG);
    }
    // D6-D7 (PA1, PA0) number of intervals
    interval = enable_period;
    for (i = 0; i <= (PINA & 0x03); i++)
      interval = interval + enable_period;

    // adjust the interword spacing we get full words in an interval
    space = pgm_read_byte(&space_adjust[((PINA >> 3) & 0x1E) | ((PINA >> 2) & 0x01)]);
  }
  else
  {
    space = 7;
  }

    // D1-3 (PA7, PA6, PA5) code
  switch ( ( PINA >> 5 ) & 0x07 )
  {
    case 1:  ptr = CODE_MOE; intervals = 0; break;
    case 2:  ptr = CODE_MOI; intervals = 1; break;
    case 3:  ptr = CODE_MOS; intervals = 2; break;
    case 4:  ptr = CODE_MOH; intervals = 3; break;
    case 5:  ptr = CODE_MO5; intervals = 4; break;
    case 6:  ptr = CODE_S;   intervals = 0; break;
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

  // D9 (PB0) key level, D10 (PB1) enable level
  output_set = ((PINB & (1 << 0)) << OUTPUT_KEY_PIN) | (((PINB & (1 << 1)) >> 1) << OUTPUT_ENABLE_PIN);
  output_set ^= OUTPUT_ENABLE | OUTPUT_KEY;
  PORTB = output_set | (PINB & 0x03);
  DDRB |= OUTPUT_ENABLE | OUTPUT_KEY | OUTPUT_LED;

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
  {
    sleep_mode();
  }
  

  return 0;
}


