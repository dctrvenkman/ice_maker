/*
 * File:   configuration_bits.c
 * Author: Administrator
 *
 * Created on January 23, 2016, 1:04 PM
 */


#include <xc.h>

/* Refer to documentation at xc8_install\docs\chips\16f722a.html */
#pragma config WDTE = OFF // Watchdog Timer disabled
#pragma config PWRTE = OFF // Power-up Timer disabled
#pragma config CP = OFF // Program memory code protection is disabled
#pragma config BOREN = OFF // Brown-out Reset disabled
#pragma config BORV = 19 // Brown-out Reset Voltage 1.9V
#pragma config DEBUG = ON // In-circuit debugger enabled, RB6/ICSPCLK and RB7/ICSPDAT are dedicated to the debugger
#pragma config MCLRE = ON // RE3/MCLR pin function is MCLR
#pragma config FOSC = INTOSCIO // Internal oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN
#pragma config PLLEN = ON // INTOSC Frequency is 16MHz (32x)
#pragma config VCAPEN = RA0 // VCAP functionality is enabled on RA0
