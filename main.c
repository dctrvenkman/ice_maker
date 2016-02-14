/*
 * File:   main.c
 * Author: Administrator
 *
 * Created on January 22, 2016, 8:58 PM
 */


/* TODO: UPDATE FOR NEW BOARD
 * Chip Wiring:
 * RA1 (Analog input) = Ambient temperature thermistor
 * RA2 (Analog input) = Low side compressor temperature thermistor
 * RA3 (Digital output) = Water pump (pumps water over evaporation tray)
 * RA4 (Digital output) = Compressor
 * RA5 (Digital input) = Water level sensor
 * RB0 (Digital input) = Ice fall lever
 * RB3 (Digital input) = ON/OFF Button
 * RB4 (Digital output) = Add water LED
 * RB5 (Digital output) = ON/OFF LED
 * RC0 (Digital output) = Reservoir drain valve (drains water from fill reservoir)
 * RC1 (Digital output) = Heater valve (directs high side to evaporation tray to release ice cubes)
 * RC2 (Digital output / PWM) = Condenser Fan
 * RC3 (Digital output) = Ice full LED
 * RC6 (Digital output) = UART TX
 * RC7 (Digital output) = UART RX
 */

/*
 * 10k Thermistor Values (found online)
 * deg C = Resistance
 * -10 = 55340
 *   0 = 32660
 *  10 = 19900
 *  20 = 12490
 *  25 = 10000
 *  30 =  8056
 *  40 =  5326
 *  50 =  3602
 *  60 =  2489
 *  70 =  1753
 *  80 =  1258
 *  90 =   917
 * 100 =   679
 * 
 * Measured
 * 72.5 =5.53k
 * 73 = 5k
 * 76 =04.8k
 * 
 * 
 */


#include <xc.h>
#include <stdio.h>
#include <stdbool.h>
#include "definitions.h"

/* UART used for debugging */
#define UART_ENABLE

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
unsigned char minutes_for_ice_making = 30;


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
    char tmpString[16];
  
	
	ANSELB = 0;
	TRISBbits.TRISB5 = 0;
	TURN_POWER_LED_ON();
	
	
	
	
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
	
    /* RA1 = Analong input (Ambient temp) */
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
    FVRCONbits.ADFVR0 = 0;
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
	
	/* Configure PWM on pin C2 (Condenser Fan) */
	/* Set CCP1 as PWM */
	CCP1CONbits.CCP1M = 0b1100; 
	/* Set PWM MSBs */
	CCPR1L = 0x00;
	/* Set PWM 2 LSBs */
	CCP1CONbits.DC1B = 0b00;
	
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
	/* Set pin A5 as input (Water level sensor) */		
    //TRISAbits.TRISA5 = 1;
	
	/* Set pin B0 as input (Ice fall lever) */
	//TRISBbits.TRISB0 = 1;
	/* Enable weak pull-up on B0 */
	WPUBbits.WPUB0 = 1;
	/* Enable interrupt on change for pin B0 */
	IOCBbits.IOCB0 = 1;
	/* Set pin B3 as input (Power button) */
	//TRISBbits.TRISB3 = 1;
	/* Enable interrupt on change for pin B3 */
	IOCBbits.IOCB3 = 1;
	/* Set pin B4 as output (Water LED) */
	TRISBbits.TRISB4 = 0;
	/* Set pin B5 as output (Power LED) */
	TRISBbits.TRISB5 = 0;
	
	/* Set pin C0 as output (Reservoir drain valve) */
	TRISCbits.TRISC0 = 0;
	/* Set pin C1 as output (Heater valve) */
	TRISCbits.TRISC1 = 0;
	/* Set pin C2 as output (Fan PWM) */
	TRISCbits.TRISC2 = 0;
	/* Set pin C3 as output (Ice LED) */
	TRISCbits.TRISC3 = 0;
		
	
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
	
	
	
	
	
	
	
	
	
	UartPrint((unsigned char*)"q - Power LED toggle\n");
	UartPrint((unsigned char*)"w - Water LED toggle\n");
	UartPrint((unsigned char*)"e - Ice LED toggle\n");
	UartPrint((unsigned char*)"r - Fan Off\n");
	UartPrint((unsigned char*)"t - Fan 50%\n");
	UartPrint((unsigned char*)"y - Fan 100%\n");
	UartPrint((unsigned char*)"a - Measure temps\n");
	UartPrint((unsigned char*)"s - Water pump on\n");
	UartPrint((unsigned char*)"d - Water pump off\n");
	UartPrint((unsigned char*)"f - Compressor on\n");
	UartPrint((unsigned char*)"g - Compressor off\n");
	UartPrint((unsigned char*)"h - Reservoir valve on\n");
	UartPrint((unsigned char*)"j - Reservoir valve off\n");
	UartPrint((unsigned char*)"z - Tank full\n");
	UartPrint((unsigned char*)"x - Heater valve on\n");
	UartPrint((unsigned char*)"c - Heater valve off\n");
	UartPrint((unsigned char*)"space - ALL OFF\n");
	
	while(1)
	{
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
					if(PORTBbits.RB4)
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
					/* Select ADC Channel 1 (AN1) */
					ADCON0bits.CHS = 1;
					/* Start ADC */
					ADCON0bits.GO = 1;
					/* Wait for ADC to complete */
					while(ADCON0bits.nDONE)
						;
					/* Clear ADC interrupt MAY NOT BE NEEDED */
					PIR1bits.ADIF = 0;
					/* Read ambient temp thermistor voltage */
					ambient_temp_v = ADRES;

					/* Select ADC Channel 2 (AN2) */
					ADCON0bits.CHS = 2;
					/* Start ADC */
					ADCON0bits.GO = 1;
					/* Wait for ADC to complete */
					while(ADCON0bits.nDONE)
						;
					/* Clear ADC interrupt MAY NOT BE NEEDED */
					PIR1bits.ADIF = 0;
					/* Read low side temp thermistor voltage */
					low_side_temp_v = ADRES;

					sprintf(tmpString, "Ambient: 0x%x\n", ambient_temp_v);
					UartPrint(tmpString);
					sprintf(tmpString, "Low Side: 0x%x\n", low_side_temp_v);
					UartPrint(tmpString);
					break;
				case 's':
					TURN_WATER_PUMP_ON();
					break;
				case 'd':
					TURN_WATER_PUMP_OFF();
					break;
				case 'f':
					TURN_COMPRESSOR_ON();
					break;
				case 'g':
					TURN_COMPRESSOR_OFF();
					break;
				case 'h':
					TURN_ON_RESERVOIR_VALVE();
					break;
				case 'j':
					TURN_OFF_RESERVOIR_VALVE();
					break;
				case 'z':
					if(TANK_IS_FULL())
						UartPrint((unsigned char*)"Tank full\n");
					else
						UartPrint((unsigned char*)"Tank not full\n");
					break;
				case 'x':
					TURN_HEATER_VALVE_ON();
					break;
				case 'c':
					TURN_HEATER_VALVE_OFF();
					break;
				case ' ':
					ALL_OFF();
					break;
			}
		}
		
		if(ice_fall_event)
		{
			ice_fall_event = false;
			UartPrint((unsigned char*)"Ice fall trigger\n");
		}
		
		if(state == OFF)
			TURN_POWER_LED_OFF();
		else
			TURN_POWER_LED_ON();
	}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
#if 0
	
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
		/* Read ambient temp thermistor voltage */
        ambient_temp_v = ADRES;
        
        /* Select ADC Channel 2 (AN2) */
        ADCON0bits.CHS = 2;
        /* Start ADC */
        ADCON0bits.GO = 1;
        /* Wait for ADC to complete */
        while(ADCON0bits.nDONE)
            ;
        /* Clear ADC interrupt MAY NOT BE NEEDED */
        PIR1bits.ADIF = 0;
		/* Read low side temp thermistor voltage */
        low_side_temp_v = ADRES;
       
		// Clear console
		UartPrint((unsigned char*)"\x1B[2J\x1B[H");
        UartPrint((unsigned char*)"State: ");
        switch(state)
        {
            case OFF:
				/* Nothing to be done as the interrupt handles it */
                sprintf(tmpString, "Off\n");
                break;
            case INIT:
				/* Fill the tank by draining water from the reservoir */
				if(TANK_IS_FULL())
				{
					TURN_OFF_RESERVOIR_VALVE();
					state = MAKING_ICE;
				}
				else
					TURN_ON_RESERVOIR_VALVE();
                sprintf(tmpString, "Init\n");
                break;
            case MAKING_ICE:
				/* Maintain a full tank by pulling from the reservoir as needed */
				if(TANK_IS_FULL())
					TURN_OFF_RESERVOIR_VALVE();
				else
					TURN_ON_RESERVOIR_VALVE();
				
				
                sprintf(tmpString, "Making Ice\n");
                break;
            case RELEASING_ICE:
                sprintf(tmpString, "Releasing Ice\n");
                break;
			case OUT_OF_WATER:
				ALL_OFF();
				TURN_LOW_WATER_LED_ON();
				break;
			case ICE_BIN_FULL:
				ALL_OFF();
				TURN_ICE_FULL_LED_ON();
				break;
            default:
                sprintf(tmpString, "UNKNOWN\n");
        }
        UartPrint(tmpString);
		sprintf(tmpString, "Time: %0.2d:%0.2d\n", elapsed_minutes, elapsed_seconds);
		UartPrint(tmpString);
        sprintf(tmpString, "Ambient: 0x%x\n", ambient_temp_v);
		UartPrint(tmpString);
		sprintf(tmpString, "Low Side: 0x%x\n", low_side_temp_v);
        UartPrint(tmpString);
		sprintf(tmpString, "\n");
        UartPrint(tmpString);
		
		
        __delay_ms(1000);
    }
#endif
    return;
}
