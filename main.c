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
unsigned char tank_fill_timeout_time;

/* Maximum number of minutes to wait for the ice to drop */
#define ICE_DROP_TIMEOUT 5

/* Counters generated from Timer2 interrupt */
extern unsigned char elapsed_seconds;
extern unsigned char elapsed_minutes;
/* Events detected by interrupt */
extern bool ice_fall_event;

/* State machine current state */
state_t state = OFF;

/* Active ice making cycle time
 * NOTE: The longer the duration the larger the ice. */
unsigned char ice_making_duration = 45;


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
    unsigned char water_level_v;
	unsigned char reservoir_fill_timer;
    char tmpString[24];
 	
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

    /* If Power button is held during power on start in RELEASING_ICE state */
    /* NOTE: The read of PORTB will set the IOC latch */
    if(~PORTB & _PORTB_RB1_MASK)
    {
        state = RELEASING_ICE;
    }
    
	/* Enable PORTB interrupt on change */
	INTCONbits.RBIE = 1;
	/* Enable peripheral interrupts */
	INTCONbits.PEIE = 1;
	/* Enable global interrupts */
	ei();
	
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
                    {
						CLOSE_RESERVOIR_VALVE();
                        /* Reset the tank fill timeout time */
                        tank_fill_timeout_time = elapsed_minutes + TANK_FILL_TIMEOUT;
                    }
					else
						if(elapsed_minutes < tank_fill_timeout_time)
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
        sprintf(tmpString, "tIceMaking: %d min\n", ice_making_duration);
        UartPrint(tmpString);
#ifdef MEASURE_AMBIENT_TEMP
        sprintf(tmpString, "A Temp: 0x%x\n", ambient_temp_v);
		UartPrint(tmpString);
#endif
		sprintf(tmpString, "LS Temp: 0x%x\n", low_side_temp_v);
        UartPrint(tmpString);
        sprintf(tmpString, "Fan: 0x%x\n", CCPR2L);
        UartPrint(tmpString);
        sprintf(tmpString, "Pump: %s\n", WATER_PUMP_IS_ON() ? "On" : "Off");
        UartPrint(tmpString);
        sprintf(tmpString, "Comp: %s\n", COMPRESSOR_IS_ON() ? "On" : "Off");
        UartPrint(tmpString);
        sprintf(tmpString, "Tank: %s\n", TANK_IS_FULL() ? "Full" : "Not Full");
        UartPrint(tmpString);
        sprintf(tmpString, "Water: 0x%x\n", water_level_v);
        UartPrint(tmpString);
        sprintf(tmpString, "W Valve: %s\n", RESERVOIR_VALVE_IS_OPEN() ? "Open" : "Closed");
        UartPrint(tmpString);
        sprintf(tmpString, "H Valve: %s\n", HEATER_VALVE_IS_OPEN() ? "Open" : "Closed");
        UartPrint(tmpString);
		sprintf(tmpString, "\n");
        UartPrint(tmpString);
		
        __delay_ms(1000);
    }

    /* Should never reach */
    return;
}
