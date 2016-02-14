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


/*
 * TODO:
 * NOTE: Rev 2 WATER_LEVEL changed from RA5 to pin RB2
 */

/* Wrapper Macros */
#define SET_FAN_SPEED(x) (CCPR1L = x)
#define IS_WATER_PUMP_ON() (PORTAbits.RA3)
#define TURN_WATER_PUMP_ON() (PORTAbits.RA3 = 1)
#define TURN_WATER_PUMP_OFF() (PORTAbits.RA3 = 0)
#define IS_COMPRESSOR_ON() (PORTAbits.RA4)
#define TURN_COMPRESSOR_ON() (PORTAbits.RA4 = 1)
#define TURN_COMPRESSOR_OFF() (PORTAbits.RA4 = 0)
#define TANK_IS_FULL() (PORTAbits.RA5 == 0)
#define IS_RESERVOIR_VALVE_OPEN() (PORTCbits.RC0)
#define TURN_ON_RESERVOIR_VALVE() (PORTCbits.RC0 = 1)
#define TURN_OFF_RESERVOIR_VALVE() (PORTCbits.RC0 = 0)
#define IS_HEATER_VALVE_OPEN() (PORTCbits.RC1)
#define TURN_HEATER_VALVE_ON() (PORTCbits.RC1 = 1)
#define TURN_HEATER_VALVE_OFF() (PORTCbits.RC1 = 0)
#define TURN_LOW_WATER_LED_ON() (PORTBbits.RB4 = 1)
#define TURN_LOW_WATER_LED_OFF() (PORTBbits.RB4 = 0)
#define TURN_ICE_FULL_LED_ON() (PORTCbits.RC3 = 1)
#define TURN_ICE_FULL_LED_OFF() (PORTCbits.RC3 = 0)
#define TURN_POWER_LED_ON() (PORTBbits.RB5 = 1)
#define TURN_POWER_LED_OFF() (PORTBbits.RB5 = 0)
#define ALL_OFF() do{ \
    PORTAbits.RA4 = 0; \
    PORTAbits.RA3 = 0; \
    CCPR1L = 0; \
    PORTCbits.RC0 = 0; \
    PORTCbits.RC1 = 0; \
    PORTBbits.RB4 = 0; \
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

