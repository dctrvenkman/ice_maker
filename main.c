/*
 * File:   main.c
 * Author: Administrator
 *
 * Created on January 22, 2016, 8:58 PM
 */

/*
 * WIRING:
 * RA0 = Vcap (.1uF - 1uF)
 * RA1 (Analog input) = ROOM SR
 * RA2 (Analog input) = LNSR
 * 
 * (Digital input) = ON/OFF Button
 * (Digital output) = ON/OFF LED
 * (Digital output) = ADD WATER LED
 * (Digital output) = FULL ICE LED
 * 
 * RC2 (Digital output / PWM) = FAN
 * RA3 (Digital output) = PUMP
 * RA4 (Digital output) = COMP
 * 
 * RA5 (Digital input???) = WATER
 * RC0 (Digital output) = WATERV
 * RC1 (Digital output) = VALVE
 * RB0 (Digital input) need pullup? = CKKG
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
 */


#include <xc.h>
#include <stdio.h>

/* UART used for debugging */
#define UART_ENABLE

/* Internal Oscillator w/ PLL enabled -> Sys Clock = 16MHz */
#define _XTAL_FREQ 16000000

/* Wrapper Macros */
#define SET_FAN_SPEED(x) CCPR1L = x
#define TURN_PUMP_ON() PORTAbits.RA3 = 1
#define TURN_PUMP_OFF() PORTAbits.RA3 = 0
#define TURN_COMPRESSOR_ON() PORTAbits.RA4 = 1
#define TURN_COMPRESSOR_OFF() PORTAbits.RA4 = 0
#define TURN_WATERV_ON() PORTCbits.RC0 = 1
#define TURN_WATERV_OFF() PORTCbits.RC0 = 0
#define TURN_VALVE_ON() PORTCbits.RC1 = 1
#define TURN_VALVE_OFF() PORTCbits.RC1 = 0
#if 0
#define TURN_LOW_WATER_LED_ON() PORTBbits.RB0 = 1
#define TURN_LOW_WATER_LED_OFF() PORTBbits.RB0 = 0
#define TURN_ICE_FULL_LED_ON() PORTBbits.RB0 = 1
#define TURN_ICE_FULL_LED_OFF() PORTBbits.RB0 = 0
#define TURN_POWER_LED_ON() PORTBbits.RB0 = 1
#define POWER_LED_OFF() PORTBbits.RB0 = 0
#endif

typedef enum
{
    OFF,
    INIT,
    MAKING_ICE,
    RELEASING_ICE,
    FAIL
}state_t;

/* Counters generated from Timer2 interrupt */
extern unsigned char elapsed_seconds;
extern unsigned char elapsed_minutes;

/*
 * Used to determing the duration of active ice making
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
    }
}
#else
#define UartPrint() ;
#define sprintf() ;
#endif


void main(void)
{
    unsigned char LNSR_V;
    unsigned char ROOMSR_V;
    char tmpString[16];
    state_t state = OFF;
  
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
    /* RA1 = Analong input */
    TRISAbits.TRISA1 = 1;
    ANSELAbits.ANSA1 = 1;
    /* RA2 = Analog input */
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
    
    
	/* Configure PWM on pin C2 */
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
    
	/* Set pin C2 as output (FAN PWM) */
	TRISCbits.TRISC2 = 0;
	/* Set pin A3 as output (PUMP) */
	TRISAbits.TRISA3 = 0;
	/* Set pin A4 as output (COMPRESSOR) */
	TRISAbits.TRISA4 = 0;
	/* Set pin C0 as output (WATERV) */
	TRISCbits.TRISC0 = 0;
	/* Set pin C1 as output (VALVE) */
	TRISCbits.TRISC1 = 0;
	/* TODO: Determine direction A5 (WATER) */		
    //TRISAbits.TRISA4 = 0;
	/* TODO: Determine if B0 needs pullup (CKKG) */
	
	
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

	/* Enable peripheral interrupts */
	INTCONbits.PEIE = 1;
	/* Enable global interrupts */
	ei();
	
	/* NOTE: Cast added to get rid of warnings */
	UartPrint((unsigned char*)"Controller ");
	UartPrint((unsigned char*)"initialized.\n");
	
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
		/* Read ROOM ST thermistor */
        ROOMSR_V = ADRES;
        
        
        /* Select ADC Channel 2 (AN2) */
        ADCON0bits.CHS = 2;
        /* Start ADC */
        ADCON0bits.GO = 1;
        /* Wait for ADC to complete */
        while(ADCON0bits.nDONE)
            ;
        /* Clear ADC interrupt MAY NOT BE NEEDED */
        PIR1bits.ADIF = 0;
		/* Read LNSR thermistor */
        LNSR_V = ADRES;
       

		//UartPrint("\x1B[2J");
        UartPrint((unsigned char*)"STATE: ");
        switch(state)
        {
            case OFF:
                sprintf(tmpString, "OFF\n");
                break;
            case INIT:
                sprintf(tmpString, "Init\n");
                break;
            case MAKING_ICE:
                sprintf(tmpString, "Making Ice\n");
                break;
            case RELEASING_ICE:
                sprintf(tmpString, "Releaseing Ice\n");
                break;
            case FAIL:
                sprintf(tmpString, "Failure\n");
                break;
            default:
                sprintf(tmpString, "UNKNOWN\n");
        }
        UartPrint(tmpString);
		sprintf(tmpString, "Time: %0.2d:%0.2d\n", elapsed_minutes, elapsed_seconds);
		UartPrint(tmpString);
        sprintf(tmpString, "LNSR: 0x%x\n", LNSR_V);
        UartPrint(tmpString);
        sprintf(tmpString, "ROOMSR: 0x%x\n", ROOMSR_V);
        UartPrint(tmpString);
		sprintf(tmpString, "\n");
        UartPrint(tmpString);

		
        __delay_ms(1000);
    }
    
    return;
}
