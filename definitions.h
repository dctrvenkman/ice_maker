/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

#ifndef _DEFINITIONS_H_
#define	_DEFINITIONS_H_

#include <xc.h> 

/* Internal Oscillator w/ PLL enabled -> Sys Clock = 16MHz */
#define _XTAL_FREQ 16000000

/* Wrapper Macros */
#define SET_FAN_SPEED(x) (CCPR2L = x)
#define WATER_PUMP_IS_ON() (PORTAbits.RA3)
#define TURN_WATER_PUMP_ON() (PORTAbits.RA3 = 1)
#define TURN_WATER_PUMP_OFF() (PORTAbits.RA3 = 0)
#define COMPRESSOR_IS_ON() (PORTAbits.RA4)
#define TURN_COMPRESSOR_ON() (PORTAbits.RA4 = 1)
#define TURN_COMPRESSOR_OFF() (PORTAbits.RA4 = 0)
//#define TANK_IS_FULL() (PORTBbits.RB2 == 0)
#define TANK_IS_FULL() (water_level_v < 0x70)
#define RESERVOIR_VALVE_IS_OPEN() (PORTCbits.RC0)
#define OPEN_RESERVOIR_VALVE() (PORTCbits.RC0 = 1)
#define CLOSE_RESERVOIR_VALVE() (PORTCbits.RC0 = 0)
#define HEATER_VALVE_IS_OPEN() (PORTCbits.RC1)
#define OPEN_HEATER_VALVE() (PORTCbits.RC1 = 1)
#define CLOSE_HEATER_VALVE() (PORTCbits.RC1 = 0)
#define TURN_LOW_WATER_LED_ON() (PORTCbits.RC4 = 1)
#define TURN_LOW_WATER_LED_OFF() (PORTCbits.RC4 = 0)
#define TURN_ICE_FULL_LED_ON() (PORTCbits.RC3 = 1)
#define TURN_ICE_FULL_LED_OFF() (PORTCbits.RC3 = 0)
#define TURN_POWER_LED_ON() (PORTBbits.RB5 = 1)
#define TURN_POWER_LED_OFF() (PORTBbits.RB5 = 0)
#define ALL_OFF() do{ \
    PORTAbits.RA4 = 0; \
    PORTAbits.RA3 = 0; \
    CCPR2L = 0; \
    PORTCbits.RC0 = 0; \
    PORTCbits.RC1 = 1; \
    PORTCbits.RC4 = 0; \
    PORTCbits.RC3 = 0; \
    PORTBbits.RB5 = 0; \
}while(0)

// State machine states
typedef enum
{
    OFF,
    INIT,
    MAKING_ICE,
    RELEASING_ICE,
    OUT_OF_WATER,
    ICE_BIN_FULL,
}state_t;

#endif	/* _DEFINITIONS_H_ */

