/*****************************************************************************/
/*                                                                           */
/* Filename: config.h                                                        */
/* Begin:    2023-09-25                                                      */
/* Author:   Kertész Csaba-Zoltán                                            */
/* E-mail:   csaba.kertesz@unitbv.ro                                         */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Description                                                               */
/*   - configuration values for ARDF controller implementation               */
/*     this file allows the use of various different implementation types    */
/*     different schemes can be selected using Makefile constants:           */
/*       - PF_LTC6903: programmable frequency variant using LTC6903 chip     */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Change history:                                                           */
/*                                                                           */
/*   2023.09.25: - separated from main.c                                     */
/*                                                                           */
/*****************************************************************************/

#ifndef __CONFIG_H__
#define __CONFIG_H__

// board variants
#if BOARD_VERSION == 2
  #define USE_LTC6903
  #define USE_PROG_FREQ
  #define USE_SPI
#else
  #define USE_LED
  #define USE_LEVEL_SETTING
#endif

// timer ticks
#define TICKS_PER_SECOND 125

// code speed settings
// normally 10 WPM for slow speed and 15 WPM for high speed
#define CODE_SPEED_SLOW 10
#define CODE_SPEED_FAST 15

#define TICKS_PER_SIGN(wpm) (60 * TICKS_PER_SECOND / 50 / (wpm))

// usual ON times (in seconds):
#define INTERVAL_SHORT 12
#define INTERVAL_LONG  60

// interval count normal values:
// slow/short: 100, slow/long: 500
// fast/short: 150, fast/long: 750
#define INTERVAL_COUNT(I) ((I) * TICKS_PER_SECOND)

// in interval mode, mark end of transmission cycle with 2s tone
#define TXOFF_TICKS (2 * TICKS_PER_SECOND)


// output bits
#define OUTPUT_PORT PORTB

// pin selection depends on variants
// using LTC6903
#ifdef USE_LTC6903
    // chip is always working, no separate enable pin is used
    #define OUTPUT_ENABLE 0
    // code is keyed using key pin
    #define OUTPUT_KEY_PIN 0
    #define OUTPUT_KEY (1 << OUTPUT_KEY_PIN)

    // LTC6903 is connected to USI in SPI mode
    // define pin settings for this
    #define USI_DO_PIN 1
    #define USI_DO (1 << USI_DO_PIN)
    #define USI_SCK_PIN 2
    #define USI_SCK (1 << USI_SCK_PIN)
    #define OSC_SEN_PIN 3
    #define OSC_SEN (1 << OSC_SEN_PIN)


// basic fixed frequency variant
#else
    // oscillator is enabled by enable pin with configurable logic level
    #define OUTPUT_ENABLE_PIN 2
    #define OUTPUT_ENABLE (1 << OUTPUT_ENABLE_PIN)
    // code is keyed using key pin
    #define OUTPUT_KEY_PIN 3
    #define OUTPUT_KEY (1 << OUTPUT_KEY_PIN)

    // aditional LED is used to signal when TX is enabled
    #define OUTPUT_LED_PIN 6
    #define OUTPUT_LED (1 << OUTPUT_LED_PIN)
    #define LED_ON() OUTPUT_PORT |= OUTPUT_LED
    #define LED_OFF() OUTPUT_PORT &= ~OUTPUT_LED

#endif

#ifdef USE_LED
    #define ENABLED_LED_TICKS (TICKS_PER_SECOND)
    #define DISABLED_LED_TICKS (60 * TICKS_PER_SECOND)
#endif


#define OUTPUT(out) OUTPUT_PORT = (OUTPUT_PORT & ~(OUTPUT_ENABLE | OUTPUT_KEY)) | ((out) & (OUTPUT_ENABLE | OUTPUT_KEY))


// DIP switch settings
// this heavily depends on board variants

// v1 board, basic operation


#define DIP_BIT(port, pin) (PIN##port & (1 << pin))
#define DIP_BIT_EXP(port, pin) DIP_BIT(port, pin)
#define DIP_BIT_x(name, pos) ((DIP_BIT_EXP(DIP_##name##_BIT##pos##_PORT, DIP_##name##_BIT##pos##_PIN) >> DIP_##name##_BIT##pos##_PIN) << pos)
#define DIP_BITS_1(name) (DIP_BIT_x(name, 0))
#define DIP_BITS_2(name) (DIP_BIT_x(name, 1) | DIP_BIT_x(name, 0))
#define DIP_BITS_3(name) (DIP_BIT_x(name, 2) | DIP_BIT_x(name, 1) | DIP_BIT_x(name, 0))
#define DIP_BITS( count, name ) DIP_BITS_##count( name )
#define DIP_BITS_EXP(count, name) DIP_BITS(count, name)

#define DIP(name) DIP_BITS_EXP(DIP_##name##_BITS, name)

#if BOARD_VERSION == 2

#define DIP_CODE_BITS 3
#define DIP_CODE_BIT0_PORT A
#define DIP_CODE_BIT0_PIN  5
#define DIP_CODE_BIT1_PORT A
#define DIP_CODE_BIT1_PIN  6
#define DIP_CODE_BIT2_PORT A
#define DIP_CODE_BIT2_PIN  7

#define DIP_SPEED_BITS 1
#define DIP_SPEED_BIT0_PORT A
#define DIP_SPEED_BIT0_PIN  3

#define DIP_ENABLE_LEVEL_BITS 1
#define DIP_ENABLE_LEVEL_BIT0_PORT B
#define DIP_ENABLE_LEVEL_BIT0_PIN  1

#define DIP_KEY_LEVEL_BITS 1
#define DIP_KEY_LEVEL_BIT0_PORT B
#define DIP_KEY_LEVEL_BIT0_PIN  0

#define DIP_INTERVAL_LENGTH_BITS 1
#define DIP_INTERVAL_LENGTH_BIT0_PORT A
#define DIP_INTERVAL_LENGTH_BIT0_PIN  5

#define DIP_INTERVAL_BITS 2
#define DIP_INTERVAL_BIT0_PORT A
#define DIP_INTERVAL_BIT0_PIN  6
#define DIP_INTERVAL_BIT1_PORT A
#define DIP_INTERVAL_BIT1_PIN  7

#define SET_INTERVAL_VALUE(interval, period) \
  switch (DIP(INTERVAL))                     \
  {                                          \
    case 1: interval = 2 * period; break;    \
    case 2: interval = 3 * period; break;    \
    case 3: interval = 5 * period; break;    \
    default: interval = 0;                   \
  }

// frequency DAC settings
#define DIP_FREQ_BITS 3
#define DIP_FREQ_BIT2_PORT B
#define DIP_FREQ_BIT2_PIN  6
#define DIP_FREQ_BIT1_PORT B
#define DIP_FREQ_BIT1_PIN  1
#define DIP_FREQ_BIT0_PORT A
#define DIP_FREQ_BIT0_PIN  4

#define PORTB_DIP_PINS (( 1 << 1 ) | ( 1 << 6))

#else /* board version 1: basic fixed controller */

#define DIP_CODE_BITS 3
#define DIP_CODE_BIT0_PORT A
#define DIP_CODE_BIT0_PIN  5
#define DIP_CODE_BIT1_PORT A
#define DIP_CODE_BIT1_PIN  6
#define DIP_CODE_BIT2_PORT A
#define DIP_CODE_BIT2_PIN  7

#define DIP_SPEED_BITS 1
#define DIP_SPEED_BIT0_PORT A
#define DIP_SPEED_BIT0_PIN  4

#define DIP_ENABLE_LEVEL_BITS 1
#define DIP_ENABLE_LEVEL_BIT0_PORT B
#define DIP_ENABLE_LEVEL_BIT0_PIN  1

#define DIP_KEY_LEVEL_BITS 1
#define DIP_KEY_LEVEL_BIT0_PORT B
#define DIP_KEY_LEVEL_BIT0_PIN  0


#define DIP_INTERVAL_LENGTH_BITS 1
#define DIP_INTERVAL_LENGTH_BIT0_PORT A
#define DIP_INTERVAL_LENGTH_BIT0_PIN  2

#define DIP_INTERVAL_BITS 2
#define DIP_INTERVAL_BIT0_PORT A
#define DIP_INTERVAL_BIT0_PIN  0
#define DIP_INTERVAL_BIT1_PORT A
#define DIP_INTERVAL_BIT1_PIN  1
#define DIP_INTERVAL_BIT2_PORT A
#define DIP_INTERVAL_BIT2_PIN  3

#define SET_INTERVAL_VALUE(interval, period) \
  switch (DIP(INTERVAL))                     \
  {                                          \
    case 4: interval = 2 * period; break;    \
    case 5: interval = 3 * period; break;    \
    case 6: interval = 4 * period; break;    \
    case 7: interval = 5 * period; break;    \
    default: interval = 0;                   \
  }

#define PORTB_DIP_PINS (( 1 << 1 ) | ( 1 << 0))

#endif


typedef enum
{
    DIP_CODE_MO = 0,
    DIP_CODE_MOE,
    DIP_CODE_MOI,
    DIP_CODE_MOS,
    DIP_CODE_MOH,
    DIP_CODE_MO5,
    DIP_CODE_S
} DIP_CODE_VALUE;









#endif /*__CONFIG_H__*/
