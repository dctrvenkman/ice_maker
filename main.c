/*
 * File:   main.c
 * Author: Administrator
 *
 * Created on January 22, 2016, 8:58 PM
 */

/* 
 * Chip Wiring:
 * RA1 (Analog input) = Ambient temperature thermistor
 * RA2 (Analog input) = Low side compressor temperature thermistor
 * RA3 (Digital output) = Water pump (pumps water over evaporation tray)
 * RA4 (Digital output) = Compressor
 * RB0 (Digital input) = Ice fall lever
 * RB1 (Digital input) = ON/OFF Button
 * RB2 (Digital input) = Water level sensor
 * RB3 (Digital output / PWM) = Condenser Fan
 * RB5 (Digital output) = ON/OFF LED
 * RC0 (Digital output) = Reservoir drain valve (drains water from fill reservoir)
 * RC1 (Digital output) = Heater valve (directs high side to evaporation tray to release ice cubes)
 * RC3 (Digital output) = Ice full LED
 * RC4 (Digital output) = Add water LED
 * RC6 (Digital output) = UART TX
 * RC7 (Digital input) = UART RX
 */

#include <xc.h>
#include <stdio.h>
#include <stdbool.h>
#include "definitions.h"

/* Build Test Mode */
//#define TEST_MODE

/* Enable UART for debugging */
#define UART_ENABLE

/* Number of minutes to try filling the tank until it is
 * determined there is no more water in the reservoir 
 * NOTE: It takes approx 50 seconds to fill the tank from empty */
#define TANK_FILL_TIMEOUT 2

/* Number of minutes to delay the start of tank filling after
 * the first measurement in which the tank is no longer full */
#define TANK_FILL_START_DELAY 5


/* Maximum number of minutes to wait for the ice to drop
 * NOTE: This may take a while if the compressor is initially off */
#define ICE_DROP_TIMEOUT 10

/* Counters generated from Timer2 interrupt */
extern unsigned char elapsed_seconds;
extern unsigned char elapsed_minutes;
/* Events detected by interrupt */
extern bool ice_fall_event;

/* Time to start filling the tank */
unsigned char tank_fill_start_time = 0;

/* State machine current state */
state_t state = OFF;

/* Active ice making cycle time
 * NOTE: The longer the duration the larger the ice. */
unsigned char ice_making_duration = 30;


#ifdef UART_ENABLE
/* NOTE: The UART output function can't be wrapped any nicer do to: 
 * 1: XC8 lacking variadic macro support 
 * 2: XC8 not supporting vsprintf
 * Thanks for that.
 */
void UartPrint(unsigned char* data)
{
    unsigned char i = 0;
    while(0 != data[i])
    {
		/* Wait for anything buffered to be sent */
        while(!PIR1bits.TXIF)
            ;
        TXREG = data[i];
		
		if(data[i] == '\n')
		{
			while(!PIR1bits.TXIF)
				;
			TXREG = '\r';
		}
		i++;
    }
}
#else
#define UartPrint() ;
#define sprintf() ;
#endif


void main(void)
{
    unsigned char low_side_temp_v;
    unsigned char water_level_v;
    char tmpString[24];
 #ifdef MEASURE_AMBIENT_TEMP
    unsigned char ambient_temp_v;
#endif
	
    /*
     * Configure System Clock for 16MHz
     */
    OSCCONbits.IRCF = 0b11;
    
    /*
     * Initialize Peripherals and I/O ports 
     */
    // A1 (Ambient Temp) analog input
    // A2 (Low Side Temp) analog inputs
    ANSELA = 0x06;
    // B2 (Water Level) analog input
    ANSELB = 0x04;
    // A1 (Ambient Temp) input
    // A2 (Low Side Temp) input
    // A3 (Water Pump) output
    // A4 (Compressor) output
    TRISA = 0xE7;
    // B0 (Ice fall lever) input
    // B1 (Power button) input
    // B2 (Water Level) input
    // B3 (Fan PWM) output
    // B5 (Power LED) output
    TRISB = 0xD7;
	// Enable weak pull-up option on PORTB
	OPTION_REGbits.nRBPU = 0;
	// Enable weak pull-ups on B0 and B1
	WPUB = 0x03;
    /* Enable interrupt on change for pin B0 and B1 */
	IOCB = 0x03;
    // C0 (Reservoir drain valve) output
    // C1 (Heater Valve) output
    // C3 (Ice LED) output
    // C4 (Water LED) output
	TRISC = 0xE4;

	/* Enable ADC fixed voltage reference and set it to 4.096V */
	FVRCON = 0x43;
    /* Wait for fixed voltage reference to be ready */
    while(!FVRCONbits.FVRRDY)
        ;
	/* Set ADC clock to Fosc / 16 and use internal fixed reference */
	ADCON1 = 0x53;
    /* Enable ADC Module */
    ADCON0bits.ADON = 1;
	
	/* Configure PWM on pin B3 (Condenser Fan) */
	/* Set B3 pin as CCP2 */
	APFCONbits.CCP2SEL = 1;
	/* Set CCP2 as PWM and clear 2 LSBs */
	CCP2CON = 0x0C;
	
    /* Configure Timer2 */
	/* Fosc/4 -> /16 -> /250 -> /10 = 100Hz */
	/* Set prescaler to 1:16 and postscaler to 1:10, enable Timer2 */
	T2CON = 0x4F;
	/* Enable Timer2 interrupt */
	PIE1bits.TMR2IE = 1;
	/* Set PR2 threshold register */
	PR2	= 250;
	/* Clear TMR2 counter register */
	TMR2 = 0;
		
	
#ifdef UART_ENABLE
    /* UART Config */
    /* Set high speed baud rate select */
    TXSTAbits.BRGH = 1;
    /* SPBRG = (Fosc / (16 * baud_rate)) - 1 */
    SPBRG = 8;
    /* Serial port enable */
    RCSTAbits.SPEN = 1;
    /* Enable receiver */
    RCSTAbits.CREN = 1;
    /* Enable transmitter */
    TXSTAbits.TXEN = 1;
#endif
	
	/* Start with everything off */
	ALL_OFF();
	
	/* If Power button is held during power on start in RELEASING_ICE state */
    /* NOTE: The read of PORTB will set the IOC latch */
    if(~PORTB & _PORTB_RB1_MASK)
    {
        state = RELEASING_ICE;
		TURN_POWER_LED_ON();
		TURN_LOW_WATER_LED_ON();
		TURN_ICE_FULL_LED_ON();
    }
	else
	{
		/* Enable global interrupts, PORTB IOC, and peripheral interrupts  */
		INTCON = 0xC8;
	}
	
	/* Main program loop */
	while(1)
	{  
		/* Select ADC Channel 1 (AN1) */
		ADCON0bits.CHS = 1;
		/* Start ADC */
		ADCON0bits.GO = 1;
		/* Wait for ADC to complete */
		while(ADCON0bits.nDONE)
			;
		/* Clear ADC interrupt MAY NOT BE NEEDED */
		PIR1bits.ADIF = 0;
		/* Read low side temp thermistor voltage */
		low_side_temp_v = ADRES;
		
#ifdef MEASURE_AMBIENT_TEMP
		/* Select ADC Channel 2 (AN2) */
		ADCON0bits.CHS = 2;
		/* Start ADC */
		ADCON0bits.GO = 1;
		/* Wait for ADC to complete */
		while(ADCON0bits.nDONE)
			;
		/* Clear ADC interrupt MAY NOT BE NEEDED */
		PIR1bits.ADIF = 0;
		/* Read ambient temp thermistor voltage */
		ambient_temp_v = ADRES;
#endif
		
        
        /* Select ADC Channel 8 (AN8) */
		ADCON0bits.CHS = 8;
		/* Start ADC */
		ADCON0bits.GO = 1;
		/* Wait for ADC to complete */
		while(ADCON0bits.nDONE)
			;
		/* Clear ADC interrupt MAY NOT BE NEEDED */
		PIR1bits.ADIF = 0;
		/* Read low side temp thermistor voltage */
		water_level_v = ADRES;
        
        
		// Clear console
		UartPrint((unsigned char*)"\x1B[2J\x1B[H");
		
        /* Check for and clear UART overrun */
		if(RCSTAbits.OERR)
		{
			RCSTAbits.CREN = 0;
			RCSTAbits.CREN = 1;
		}
		/* Get input from UART */
        if(PIR1bits.RCIF)
		{
			unsigned char rx_byte = RCREG;
            if(rx_byte == '[')
                ice_making_duration--;
            if(rx_byte == ']')
                ice_making_duration++;

#ifdef TEST_MODE
		/*
		 * Test Mode Commands
		 * ----------------------------
		 * q - Toggle Power LED
		 * w - Toggle Water LED
		 * e - Toggle Ice LED
		 * r - Fan Off
		 * t - Fan 50%
		 * y - Fan 100%
		 * 
		 * a - Toggle Water Pump
		 * s - Toggle Compressor
		 * d - Toggle Reservoir Valve
		 * f - Toggle Heater Valve
		 * 
		 * <space> - Turn everything off
		 */
			switch(rx_byte)
			{
				case 'q':
					if(PORTBbits.RB5)
						TURN_POWER_LED_OFF();
					else
						TURN_POWER_LED_ON();
					break;
				case 'w':
					if(PORTCbits.RC4)
						TURN_LOW_WATER_LED_OFF();
					else
						TURN_LOW_WATER_LED_ON();
					break;
				case 'e':
					if(PORTCbits.RC3)
						TURN_ICE_FULL_LED_OFF();
					else
						TURN_ICE_FULL_LED_ON();
					break;
				case 'r':
					SET_FAN_SPEED(0x00);
					break;
				case 't':
					SET_FAN_SPEED(0x80);
					break;
				case 'y':
					SET_FAN_SPEED(0xff);
					break;
				case 'a':
					if(WATER_PUMP_IS_ON())
						TURN_WATER_PUMP_OFF();
					else
						TURN_WATER_PUMP_ON();
					break;
				case 's':
					if(COMPRESSOR_IS_ON())
						TURN_COMPRESSOR_OFF();
					else
						TURN_COMPRESSOR_ON();
					break;
				case 'd':
					if(RESERVOIR_VALVE_IS_OPEN())
						CLOSE_RESERVOIR_VALVE();
					else
						OPEN_RESERVOIR_VALVE();
					break;
				case 'f':
					if(HEATER_VALVE_IS_OPEN())
						CLOSE_HEATER_VALVE();
					else
						OPEN_HEATER_VALVE();
					break;
				case ' ':
					ALL_OFF();
					break;
			}
		}

		if(ice_fall_event)
		{
			ice_fall_event = false;
			UartPrint((unsigned char*)"Ice fall event.\n");
		}
		
		if(state == INIT)
		{
		    state = OFF;
		    UartPrint((unsigned char*)"Power button pressed.\n");
		}
#else
        }
        
        switch(state)
        {
            case OFF:
				/* Nothing to be done as the interrupt handles it */
				/* Don't keep time in the off state */
				elapsed_minutes = 0;
				elapsed_seconds = 0;
                sprintf(tmpString, "Off\n");
                break;
            case INIT:
                /* Wait for the tank to fill */
				if(TANK_IS_FULL())
				{
					/* Reset timer */
					elapsed_minutes = 0;
					elapsed_seconds = 0;
					/* Move on to making ice */
					state = MAKING_ICE;
				}
                sprintf(tmpString, "Init\n");
                break;
            case MAKING_ICE:
				if(elapsed_minutes < ice_making_duration)
				{
					/* Don't start timer until we reach freezing
					 * This should help with the first run */
					if(low_side_temp_v > 0x50)
						elapsed_minutes = 0;
					
					/* Close heat valve and turn on condenser fan, compressor, and water pump */
					CLOSE_HEATER_VALVE();
					SET_FAN_SPEED(0xff);
					TURN_COMPRESSOR_ON();
					TURN_WATER_PUMP_ON();
				}
				else
				{
					/* Turn off fan, compressor, and water pump */
					SET_FAN_SPEED(0);
					TURN_COMPRESSOR_OFF();
					TURN_WATER_PUMP_OFF();
					/* Reset timer */
					elapsed_minutes = 0;
					elapsed_seconds = 0;
					/* Release the ice */
					state = RELEASING_ICE;
				}	
                sprintf(tmpString, "Making Ice\n");
                break;
            case RELEASING_ICE:
				if(ice_fall_event || (elapsed_minutes >= ICE_DROP_TIMEOUT))
				{
					CLOSE_HEATER_VALVE();
					/* Clear ice fall event */
					ice_fall_event = false;
					/* Reset timer */
					elapsed_minutes = 0;
					elapsed_seconds = 0;
					/* Start making ice again */
					state = MAKING_ICE;
				}
				else
				{
					/*  Open the heater valve and wait for the ice to fall */
					OPEN_HEATER_VALVE();
                    SET_FAN_SPEED(0xff);
					TURN_COMPRESSOR_ON();
				}
                sprintf(tmpString, "Releasing Ice\n");
                break;
			case OUT_OF_WATER:
                /* TODO: Do something */
				ALL_OFF();
				TURN_LOW_WATER_LED_ON();
				sprintf(tmpString, "Out of water\n");
				break;
			case ICE_BIN_FULL:
                /* TODO: Implement */
				ALL_OFF();
				TURN_ICE_FULL_LED_ON();
				sprintf(tmpString, "Ice bin full\n");
				break;
            default:
                sprintf(tmpString, "UNKNOWN\n");
        }
		
        /* Common code for all states which need the water tank to be filled */
        if(state == INIT || state == MAKING_ICE )
        {
            /* Maintain a full tank by pulling from the reservoir as needed */
            if(TANK_IS_FULL())
            {
                CLOSE_RESERVOIR_VALVE();
                /* Reset the tank fill start time */
                tank_fill_start_time = elapsed_minutes + TANK_FILL_START_DELAY;
            }
            else
            {
				/* Check for tank fill timeout (i.e. out of water) */
                if(elapsed_minutes >= (tank_fill_start_time + TANK_FILL_TIMEOUT))
                {
                    /* Timed out so move to OUT_OF_WATER state */
                    CLOSE_RESERVOIR_VALVE();
                    /* Reset timer */
                    elapsed_minutes = 0;
                    elapsed_seconds = 0;
                    /* Out of water */
                    state = OUT_OF_WATER;
                }
                else if(elapsed_minutes >= tank_fill_start_time)
                {
                    /* Fill the tank by draining water from the reservoir */
                    OPEN_RESERVOIR_VALVE();
                }
            }
        }
        
		UartPrint((unsigned char*)"State: ");
        UartPrint(tmpString);
#endif
		
        sprintf(tmpString, "Time: %0.2d:%0.2d (%d min)\n", elapsed_minutes, elapsed_seconds, ice_making_duration);
		UartPrint(tmpString);
		sprintf(tmpString, "Tank fill start: %d min\n", tank_fill_start_time);
		UartPrint(tmpString);
#ifdef MEASURE_AMBIENT_TEMP
        sprintf(tmpString, "A Temp: 0x%x\n", ambient_temp_v);
		UartPrint(tmpString);
#endif
		sprintf(tmpString, "LS Temp: 0x%x\n", low_side_temp_v);
        UartPrint(tmpString);
#if 0
        sprintf(tmpString, "Fan: 0x%x\n", CCPR2L);
        UartPrint(tmpString);
#endif
        sprintf(tmpString, "Pump: %s\n", WATER_PUMP_IS_ON() ? "On" : "Off");
        UartPrint(tmpString);
        sprintf(tmpString, "Comp: %s\n", COMPRESSOR_IS_ON() ? "On" : "Off");
        UartPrint(tmpString);
        sprintf(tmpString, "Tank: %s (0x%x)\n", TANK_IS_FULL() ? "Full" : "Not Full", water_level_v);
        UartPrint(tmpString);
        sprintf(tmpString, "W Valve: %s\n", RESERVOIR_VALVE_IS_OPEN() ? "Open" : "Closed");
        UartPrint(tmpString);
        sprintf(tmpString, "H Valve: %s\n", HEATER_VALVE_IS_OPEN() ? "Open" : "Closed");
        UartPrint(tmpString);
		sprintf(tmpString, "\n\n");
        UartPrint(tmpString);
		
        __delay_ms(500);
    }

    return;
}
