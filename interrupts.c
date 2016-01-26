/*
 * File:   interrupts.c
 * Author: root
 *
 * Created on January 24, 2016, 10:41 AM
 */


#include <xc.h>

static unsigned char t2_10ms_count = 0;
unsigned char elapsed_seconds = 0;
unsigned char elapsed_minutes = 0;

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
	
	/* Clear all interrupt flags */
	PIR1 = 0; 
}
