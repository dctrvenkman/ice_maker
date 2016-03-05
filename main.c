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

/* UART used for debugging */
#define UART_ENABLE

/* Number of seconds to try filling the tank until it is
 * determined there is no more water in the reservoir */
#define TANK_FILL_TIMEOUT 2

/* Counters generated from Timer2 interrupt */
extern unsigned char elapsed_seconds;
extern unsigned char elapsed_minutes;
/* Events detected by interrupt */
extern bool ice_fall_event;

// State machine state
state_t state = OFF;

/*
 * Used to determine the duration of active ice making
 * The longer the duration the larger the ice.
 */
unsigned char ice_making_duration = 2;


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
        TXREG = data[i++];
		
		if(data[i-1] == '\n')
		{
			while(!PIR1bits.TXIF)
				;
			TXREG = '\r';
		}
    }
}
#else
#define UartPrint() ;
#define sprintf() ;
#endif


void main(void)
{
    unsigned char low_side_temp_v;
    unsigned char ambient_temp_v;
	unsigned char reservoir_fill_timer;
    char tmpString[24];
 	
    /*
     * Configure System Clock for 16MHz
     */
    OSCCONbits.IRCF = 0b11;
    
    /*
     * Initialize Peripherals and I/O ports 
     */
    /* Set all pins to digital */
    ANSELA = 0;
    ANSELB = 0;
	/* Enable weak pull-up option on PORTB */
	OPTION_REGbits.nRBPU = 0;
	/* Disable all weak pull-ups */
	WPUB = 0;
	
    /* RA1 = Analog input (Ambient temp) */
    TRISAbits.TRISA1 = 1;
    ANSELAbits.ANSA1 = 1;
    /* RA2 = Analog input (Low side temp) */
    TRISAbits.TRISA2 = 1;
    ANSELAbits.ANSA2 = 1;
    /* ADFVR[1..0] Sets ADC internal fixed voltage reference:
     * 00 = off
     * 01 = 1.024V
     * 10 = 2.048V
     * 11 = 4.096V
     */
    FVRCONbits.ADFVR1 = 1;
    FVRCONbits.ADFVR0 = 1;
    /* Enable fixed voltage reference */
    FVRCONbits.FVREN = 1;
    /* Wait for fixed voltage reference to be ready */
    while(!FVRCONbits.FVRRDY)
        ;
    /* Use internal fixed voltage reference */
    ADCON1bits.ADREF = 0b11;
    /* Set ADC clock to Fosc / 16 */
    ADCON1bits.ADCS = 0b101;
    /* Enable ADC Module */
    ADCON0bits.ADON = 1;
	
	/* Configure PWM on pin B3 (Condenser Fan) */
	/* Set B3 pin as CCP2 */
	APFCONbits.CCP2SEL = 1;
	/* Set CCP2 as PWM */
	CCP2CONbits.CCP2M = 0b1100; 
	/* Set PWM MSBs */
	CCPR2L = 0x00;
	/* Set PWM 2 LSBs */
	CCP2CONbits.DC2B = 0b00;
	
    /* Configure Timer2 */
	/* Fosc/4 -> /16 -> /250 -> /10 = 100Hz */
    /* Set prescaler to 1:16 */
    T2CONbits.T2CKPS = 0b11;
    /* Set postscaler to 1:10 */
    T2CONbits.TOUTPS = 0b1001;
	/* Enable Timer2 interrupt */
	PIE1bits.TMR2IE = 1;
	/* Enable Timer2 */
    T2CONbits.TMR2ON = 1;
	/* Set PR2 threshold register */
	PR2	= 250;
	/* Clear TMR2 counter register */
	TMR2 = 0;
    
	
	/* Set pin A3 as output (Water Pump) */
	TRISAbits.TRISA3 = 0;
	/* Set pin A4 as output (Compressor) */
	TRISAbits.TRISA4 = 0;
		
	/* Set pin B0 as input (Ice fall lever) */
	//TRISBbits.TRISB0 = 1;
	/* Enable weak pull-up on B0 */
	WPUBbits.WPUB0 = 1;
	/* Enable interrupt on change for pin B0 */
	IOCBbits.IOCB0 = 1;
	/* Set pin B1 as input (Power button) */
	//TRISBbits.TRISB1 = 1;
	/* Enable weak pull-up on B1 */
	WPUBbits.WPUB1 = 1;
	/* Enable interrupt on change for pin B1 */
	IOCBbits.IOCB1 = 1;
	/* Set pin B2 as input (Water level sensor) */		
    //TRISBbits.TRISB2 = 1;
	/* Set pin B3 as output (Fan PWM) */
	TRISBbits.TRISB3 = 0;
	/* Set pin B5 as output (Power LED) */
	TRISBbits.TRISB5 = 0;
	
	/* Set pin C0 as output (Reservoir drain valve) */
	TRISCbits.TRISC0 = 0;
	/* Set pin C1 as output (Heater valve) */
	TRISCbits.TRISC1 = 0;
	/* Set pin C3 as output (Ice LED) */
	TRISCbits.TRISC3 = 0;
	/* Set pin C4 as output (Water LED) */
	TRISCbits.TRISC4 = 0;
		
	
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

	/* Enable PORTB interrupt on change */
	INTCONbits.RBIE = 1;
	/* Enable peripheral interrupts */
	INTCONbits.PEIE = 1;
	/* Enable global interrupts */
	ei();
	
	/* NOTE: Cast added to get rid of warnings */
	UartPrint((unsigned char*)"\x1B[2J\x1B[H");
	UartPrint((unsigned char*)"Controller ");
	UartPrint((unsigned char*)"initialized.\n");
	
	/* Start with everything off */
	ALL_OFF();
	
	
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
		
		// Clear console
		UartPrint((unsigned char*)"\x1B[2J\x1B[H");
		
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
		
		unsigned char rx_byte;
		if(PIR1bits.RCIF)
		{
			rx_byte = RCREG;
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
				if(TANK_IS_FULL())
				{
					CLOSE_RESERVOIR_VALVE();
					/* Reset timer */
					elapsed_minutes = 0;
					elapsed_seconds = 0;
					/* Move on to making ice */
					state = MAKING_ICE;
				}
				else
				{
					if(elapsed_minutes < TANK_FILL_TIMEOUT)
					{
						/* Fill the tank by draining water from the reservoir */
						OPEN_RESERVOIR_VALVE();
					}
					else
					{
						CLOSE_RESERVOIR_VALVE();
						/* Reset timer */
						elapsed_minutes = 0;
						elapsed_seconds = 0;
						/* Out of water */
						state = OUT_OF_WATER;
					}
				}
                sprintf(tmpString, "Init\n");
                break;
            case MAKING_ICE:
				if(elapsed_minutes < ice_making_duration)
				{
					/* Maintain a full tank by pulling from the reservoir as needed */
					if(TANK_IS_FULL())
						CLOSE_RESERVOIR_VALVE();
					else
						if(elapsed_minutes < TANK_FILL_TIMEOUT)
						{
							/* Fill the tank by draining water from the reservoir */
							OPEN_RESERVOIR_VALVE();
						}
						else
						{
							CLOSE_RESERVOIR_VALVE();
							/* Reset timer */
							elapsed_minutes = 0;
							elapsed_seconds = 0;
							/* Out of water */
							state = OUT_OF_WATER;
						}

					/* Close heat valve and turn on condenser fan, compressor, and water pump */
					CLOSE_HEATER_VALVE();
					SET_FAN_SPEED(0xff);
					TURN_COMPRESSOR_ON();
					TURN_WATER_PUMP_ON();

					// TODO: Check if low side temp is below freezing
					/* Freezing If low side temp ADC value < 0x50 */
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
				if(ice_fall_event)
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
				}
                sprintf(tmpString, "Releasing Ice\n");
                break;
			case OUT_OF_WATER:
				ALL_OFF();
				TURN_LOW_WATER_LED_ON();
				sprintf(tmpString, "Out of water\n");
				break;
			case ICE_BIN_FULL:
				ALL_OFF();
				TURN_ICE_FULL_LED_ON();
				sprintf(tmpString, "Ice bin full\n");
				break;
            default:
                sprintf(tmpString, "UNKNOWN\n");
        }
		
		UartPrint((unsigned char*)"State: ");
        UartPrint(tmpString);
#endif
		
		sprintf(tmpString, "Time: %0.2d:%0.2d\n", elapsed_minutes, elapsed_seconds);
		UartPrint(tmpString);
        sprintf(tmpString, "Ambient Temp: 0x%x\n", ambient_temp_v);
		UartPrint(tmpString);
		sprintf(tmpString, "Low Side Temp: 0x%x\n", low_side_temp_v);
        UartPrint(tmpString);
        sprintf(tmpString, "Fan Speed: 0x%x\n", CCPR2L);
        UartPrint(tmpString);
        sprintf(tmpString, "Water pump: %s\n", WATER_PUMP_IS_ON() ? "On" : "Off");
        UartPrint(tmpString);
        sprintf(tmpString, "Compressor: %s\n", COMPRESSOR_IS_ON() ? "On" : "Off");
        UartPrint(tmpString);
        sprintf(tmpString, "Tank: %s\n", TANK_IS_FULL() ? "Full" : "Not Full");
        UartPrint(tmpString);
        sprintf(tmpString, "Water Valve: %s\n", RESERVOIR_VALVE_IS_OPEN() ? "Open" : "Closed");
        UartPrint(tmpString);
        sprintf(tmpString, "Heat Valve: %s\n", HEATER_VALVE_IS_OPEN() ? "Open" : "Closed");
        UartPrint(tmpString);
		sprintf(tmpString, "\n");
        UartPrint(tmpString);
		
        __delay_ms(1000);
    }
    return;
}
