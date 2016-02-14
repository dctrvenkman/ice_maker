/*
 * File:   interrupts.c
 * Author: root
 *
 * Created on January 24, 2016, 10:41 AM
 */


#include <xc.h>
#include <stdbool.h>
#include "definitions.h"

static unsigned char t2_10ms_count = 0;
unsigned char elapsed_seconds = 0;
unsigned char elapsed_minutes = 0;
bool ice_fall_event = false;

// State machine state
extern state_t state;

/******************************************************************************/
/* Interrupt Routines                                                         */
/******************************************************************************/

/* Low-priority interrupt routine */
void interrupt low_isr(void)
{
	/* Check all pertinent interrupt flags */
	
	/* Timer2 Interrupt */
	if(PIR1bits.TMR2IF)
	{
		t2_10ms_count++;
		if(t2_10ms_count >= 100)
		{
			elapsed_seconds++;
			t2_10ms_count = 0;
		}
		if(elapsed_seconds >= 60)
		{
			elapsed_minutes++;
			elapsed_seconds = 0;
		}
	}
	
	/* PORTB interrupt on change */
	if(INTCONbits.RBIF)
	{
		/* Read PORTB register to clear the latch */
		char tmp = PORTB;
		/* Clear the interrupt */
		INTCONbits.RBIF = 0;
		
		/* Determine which events have occurred*/
		if(tmp & _PORTB_RB0_MASK) // Rising edge (flapper down)
			ice_fall_event = true;
		
		// Power button key down (Falling edge)
		if(!(tmp & _PORTB_RB3_MASK)) 
		{
			if(state == OFF)
			{
				TURN_POWER_LED_ON();
				state = INIT;
			}
			else
			{
				// Turn everything off
				ALL_OFF();
				state == OFF;
			}
		}
	}
	
	/* Clear all interrupt flags */
	PIR1 = 0; 
}
