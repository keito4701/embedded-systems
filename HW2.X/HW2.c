/*******************************
 * Name: Nathan Reagan
 * Student ID#: 1001069209
 * CSE 3442/5442 - Embedded Systems 1
 * Homework 2
 * 
 * Homework 2 Description:
 *  	Use PIC18F452 and XC8 for this assignment
 * 
 *  	Use CCP1's Capture Mode and Interrupts
 *  	to determine the frequency of a signal
 *  	coming through PIN RC2 (CCP1)
 * 
 *  	printf() the calculated frequency continuously
 * 
 *  	Use Clock Stimulus to generate the signal
 *  	within MPLAB's Simulator (50Hz to 500Hz)
 ********************************/ 

// PIC18F452 Configuration Bit Settings
#pragma config OSC = HS         // Oscillator Selection bits (HS oscillator)
#pragma config OSCS = OFF       // Oscillator System Clock Switch Enable bit (Oscillator system clock switch option is disabled (main oscillator is source))
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = OFF        // Brown-out Reset Enable bit (Brown-out Reset disabled)
#pragma config BORV = 20        // Brown-out Reset Voltage bits (VBOR set to 2.0V)
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 128      // Watchdog Timer Postscale Select bits (1:128)
#pragma config CCP2MUX = ON     // CCP2 Mux bit (CCP2 input/output is multiplexed with RC1)
#pragma config STVR = OFF       // Stack Full/Underflow Reset Enable bit (Stack Full/Underflow will not cause RESET)
#pragma config LVP = OFF        // Low Voltage ICSP Enable bit (Low Voltage ICSP disabled)

#include <xc.h>
#include <stdio.h> // To use the printf() statement for Simulator's USART Output Window
                   // printf XC8 details: http://microchipdeveloper.com/tls2101:printf

void putch(unsigned char data);
void interrupt timer(void);
void show_freq();
void enable_timer();

//prev time, to be used to calculate frequency
unsigned int t1 = 0;
unsigned int t2 = 0;
double f = 0;

#define _XTAL_FREQ 4000000 // Fosc = 4MHz, required for __delay_ms() and __delay_us() functions

// To allow regular console printing within MPLAB's Simulator (override the putch function)
// Go to File > Project Properties > Simulator > Option Categories > Uart1 IO Options > Enable Uart IO (check the box)
// For full info go here: http://microchipdeveloper.com/xc8:console-printing
void main(void) 
{
    // Needed for printing in MPLAB's UART Console window of the Simulator
    TXSTAbits.TXEN = 1; // enable transmitter
    RCSTAbits.SPEN = 1; // enable serial port
	
	//other settings and main routine...
    CCP1CON = 0b00000101;   // Initialize CCP1CON for capture (every rising edge)
    TRISCbits.RC2 = 1;      // input for CCP1 (RC2)    
    T3CON = 0b01000000;     // prescaler at 1, two 8-bit operations
    PIE1bits.CCP1IE = 1;    // External CCP1E enabled
    
    IPR1bits.CCP1IP = 1;    // Enable for high priority
    INTCONbits.PEIE = 1;    // Enable for LOW interrupts
    INTCONbits.GIE = 1;     // Enable (release the hold) for HIGH interrupts (that have been indiv. enabled)
    
    enable_timer();
    
    while(1) {
        //xd
        printf("f = %0.3f Hz\n", f);
    }
}

void putch(unsigned char data) {
    while(!PIR1bits.TXIF) // wait until the transmitter is ready
        continue;
    
    TXREG = data;   // send one character
}

void interrupt Timer(void) {
    if( (PIE1bits.CCP1IE == 1) && (PIR1bits.CCP1IF == 1) ){
        show_freq();
        PIR1bits.CCP1IF = 0;
    }
}

void show_freq() {
    t2 = (CCPR1H << 8) + CCPR1L;
    
    f = 1000000/(t2-t1);
    t1 = t2;
}

//enables timer for use
void enable_timer() {
    T3CONbits.TMR3ON = 1;
}
