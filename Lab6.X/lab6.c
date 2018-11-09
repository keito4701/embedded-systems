/*******************************
 * Name: Nathan Reagan
 * Student ID#: 1001069209
 * Lab Day:
 * CSE 3442/5442 - Embedded Systems 1
 * Lab 5: DAC
 ********************************/

/*
    B3: allows for incrementing (B2) to happen         :SW2
    B2: increments clock quickly                       :PB2
    B1: pauses the clock from incrementing             :SW1
    B0: reset button (also flashes LED)                :PB1
*/


#include <math.h>
#include <xc.h> // For the XC8 Compiler (automatically finds/includes the specific PIC18F452.h header file)
#define _XTAL_FREQ  10000000     // Running at 10MHz (external oscillator/crystal), REQUIRED for delay functions

#define LED_LEFT    PORTAbits.RA3  // QwikFlash red LED (left) to toggle
#define LED_CENTER  PORTAbits.RA2  // QwikFlash red LED (center) to toggle
#define LED_RIGHT   PORTAbits.RA1  // QwikFlash red LED (right) to toggle

// PIC18F452 Configuration Bit Settings
#pragma config OSC     = HS     // Oscillator Selection bits (HS oscillator)
#pragma config OSCS    = OFF    // Oscillator System Clock Switch Enable bit (Oscillator system clock switch option is disabled (main oscillator is source))
#pragma config PWRT    = OFF    // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR     = OFF    // Brown-out Reset Enable bit (Brown-out Reset disabled)
#pragma config WDT     = OFF    // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config CCP2MUX = ON     // CCP2 Mux bit (CCP2 input/output is multiplexed with RC1)
#pragma config STVR    = ON     // Stack Full/Underflow Reset Enable bit (Stack Full/Underflow will cause RESET)
#pragma config LVP     = OFF    // Low Voltage ICSP Enable bit (Low Voltage ICSP disabled)

/*******************************
 * Global Vars
 * 
 * Strings for LCD Initialization and general use
 * For stability reasons, each array must have 10 total elements (no less, no more)
 ********************************/
const char LCDstr[]  = {0x33,0x32,0x28,0x01,0x0c,0x06,0x00,0x00}; // LCD Initialization string (do not change)

//Never change element [0] or [9] of the following char arrays
//You may only change the middle 8 elements for displaying on the LCD
char Str_1[] = {0x80,' ',' ',' ',' ',' ',' ',' ',' ',0};    // First line of LCD
char Str_2[] = {0xC0,'h',' ',' ',' ',' ',' ',' ',' ',0};    // Second line of LCD

const char Clear1[] = {0x80,' ',' ',' ',' ',' ',' ',' ',' ',0};  // Clear first line of LCD
const char Clear2[] = {0xC0,' ',' ',' ',' ',' ',' ',' ',' ',0};  // Clear second line of LCD


/*******************************
 * Function prototypes
 ********************************/
void Initialize_PIC(void);
void Initialize_LCD(void);
void Print_To_LCD(const char *);
void Toggle_LEDs(void);

void interrupt My_ISR_High(void);
void interrupt low_priority timer(void);

void high_prio();
void timer_display();
void initialize_timer();
void increment_time();

unsigned char h, m, s;

void main(void)
{
    Initialize_PIC();       //Initialize all settings required for general QwikFlash and LCD operation
	Print_To_LCD(Clear1);   //Clear the LCD line 1 to start your program
	Print_To_LCD(Clear2);   //Clear the LCD line 2 to start your program
	
	//other settings and main routine...
    TRISB = 0b00001111;  // inputs for B3:B0
    TRISA = 0b00010000;  // output for LED
    
    //setup timer settings
    initialize_timer();
    
    // Interrupt settings
    INTCONbits.TMR0IE = 1;  // timer0 enabled
    INTCONbits.INT0IE = 1;   // External INT0 enabled (pin B0)
    INTCON2 = 0b11110000;
    INTCON3 = 0b00001000;
    
    RCONbits.IPEN = 1;      // Allow HIGH and LOW priority interrupts (instead of all HIGH)
    INTCONbits.PEIE = 1;    // Enable for LOW interrupts
    INTCONbits.GIE = 1;     // Enable (release the hold) for HIGH interrupts (that have been indiv. enabled)
    
    //initialize clock to 0, set string for format
    h = 0, m = 0, s = 0;
    Str_1[3] = ':';
    Str_1[6] = ':';
    
    //print starting time
    Str_1[1] = h/10 + '0';
    Str_1[2] = h%10 + '0';
    Str_1[4] = m/10 + '0';
    Str_1[5] = m%10 + '0';
    Str_1[7] = s/10 + '0';
    Str_1[8] = s%10 + '0';
    
    Print_To_LCD(Str_1);
    
    //flash middle LED
    PORTAbits.RA1 = 0;
    PORTAbits.RA2 = 1;
    PORTAbits.RA3 = 0;
    
    // start timer
    T0CONbits.TMR0ON = 1;
    
    while(1) {
        //xd
    }
}

//HIGH PRIORITY: b0 activated (reset switch)
void interrupt My_ISR_High(void) {
    // Was the triggered interrupt EXTERNAL INTERRUPT 1? (pin B0)
    // If the interrupt IS ENABLED and its FLAG IS SET
    if((INTCONbits.INT0E == 1) && (INTCONbits.INT0F == 1)) {
        high_prio();
        INTCONbits.INT0IF = 0;  // Clear INT0 Flag bit ("reset" it)
    }
}

//only perform interrupt if switch (B2) is off
void interrupt low_priority Timer(void) {
    if( (INTCONbits.TMR0IE == 1) && (INTCONbits.TMR0IF == 1) ){
        timer_display();
        INTCONbits.TMR0IF = 0;
    }
}

//reset timer
void high_prio(){
    s = 0;
    m = 0;
    h = 0;
    
    //change LED while button is pressed
    while(PORTBbits.RB0 == 1){
        PORTAbits.RA1 = 1;
        PORTAbits.RA2 = 0;
        PORTAbits.RA3 = 1;
        
        __delay_ms(5);
    }
    
    PORTAbits.RA1 = 0;
    PORTAbits.RA2 = 1;
    PORTAbits.RA3 = 0;
    
    initialize_timer();
    
    Str_1[1] = h/10 + '0';
    Str_1[2] = h%10 + '0';
    Str_1[4] = m/10 + '0';
    Str_1[5] = m%10 + '0';
    Str_1[7] = s/10 + '0';
    Str_1[8] = s%10 + '0';
    
    Print_To_LCD(Str_1);
}

//Increment second counter, adjust other values accordingly, display
void timer_display(){
    //pause while switch is flipped
    while(PORTBbits.RB1 == 1){
        
        //if PB2 (B2) is held down AND SW2 (B3) is active, increment clock quickly
        if(PORTBbits.RB3 == 1 && PORTBbits.RB2 == 1) {
            increment_time();
            __delay_ms(50);
        }
    }
    
    //once pause switch is let go, resume time like normal
    increment_time();
    
    initialize_timer();
}

//increments the second counter, updates the time on LCD
void increment_time(){
    s += 1;
    
    if (s > 60){
        s = 0;
        m += 1;
    }
    
    if (m > 60) {
        m = 0;
        h += 1;
    }
    
    if (h > 24)
        h = 0;
    
    Str_1[1] = h/10 + '0';
    Str_1[2] = h%10 + '0';
    Str_1[4] = m/10 + '0';
    Str_1[5] = m%10 + '0';
    Str_1[7] = s/10 + '0';
    Str_1[8] = s%10 + '0';
    
    Print_To_LCD(Str_1);
}

/*******************************
 * Initialize_PIC(void)
 *
 * This function performs all initializations of variables and registers
 * for the PIC18F452 when specifically on the QwikFlash board.
 *
 * DO NOT CHANGE ANYTHING IN THIS FUNCTION
 ********************************/
void Initialize_PIC(void)
{
    // Reference the QwikFlash schematic (pdf) to understand analog/digital IO decisions
    
    ADCON1 = 0b10001110;    // Enable PORTA & PORTE digital I/O pins
    TRISA  = 0b11100001;    // Set I/O for PORTA
    TRISB  = 0b11011100;    // Set I/O for PORTB
    TRISC  = 0b11010000;    // Set I/0 for PORTC
    TRISD  = 0b00001111;    // Set I/O for PORTD
    TRISE  = 0b00000000;    // Set I/O for PORTE
    PORTA  = 0b00010000;    // Turn off all four LEDs driven from PORTA    
    LED_LEFT    = 0;        // All LEDs initially OFF
    LED_CENTER  = 0;        // All LEDs initially OFF
    LED_RIGHT   = 0;        // All LEDs initially OFF
    Initialize_LCD();       // Initialize LCD
}

/*******************************
 * Initialize_LCD(void)
 *
 * Initialize the Optrex 8x2 character LCD.
 * First wait for 0.1 second, to get past the displays power-on reset time.
 *
 * DO NOT CHANGE ANYTHING IN THIS FUNCTION
 *******************************/
void Initialize_LCD(void)
{
    char currentChar;
    char *tempPtr;

    __delay_ms(100);                // wait 0.1 sec (100 ms)

    PORTEbits.RE0 = 0;              // RS=0 for command
    tempPtr = LCDstr;

    while (*tempPtr)                // if the byte is not zero (end of string)
    {
        currentChar = *tempPtr;
        PORTEbits.RE1 = 1;          // Drive E pin high
        PORTD = currentChar;        // Send upper nibble
        PORTEbits.RE1 = 0;          // Drive E pin low so LCD will accept nibble          
        __delay_ms(10);             // wait 10 ms
        currentChar <<= 4;          // Shift lower nibble to upper nibble
        PORTEbits.RE1 = 1;          // Drive E pin high again
        PORTD = currentChar;        // Write lower nibble
        PORTEbits.RE1 = 0;          // Drive E pin low so LCD will process byte        
        __delay_ms(10);             // wait 10 ms	
        tempPtr++;                  // Increment pointer to next character
    }
}

/*******************************
 * Print_To_LCD(const char * tempPtr) 
 *
 * This function is called with the passing in of an array of a constant
 * display string.  It sends the bytes of the string to the LCD.  The first
 * byte sets the cursor position.  The remaining bytes are displayed, beginning
 * at that position.
 * This function expects a normal one-byte cursor-positioning code, 0xhh, or
 * an occasionally used two-byte cursor-positioning code of the form 0x00hh.
 *
 * DO NOT CHANGE ANYTHING IN THIS FUNCTION
 ********************************/
void Print_To_LCD(const char * tempPtr)
{
	char currentChar;
    PORTEbits.RE0 = 0;          // Drive RS pin low for cursor-positioning code

    while (*tempPtr)            // if the byte is not zero (end of string)
    {
        currentChar = *tempPtr;
        PORTEbits.RE1 = 1;      // Drive E pin high
        PORTD = currentChar;    // Send upper nibble
        PORTEbits.RE1 = 0;      // Drive E pin low so LCD will accept nibble
        currentChar <<= 4;      // Shift lower nibble to upper nibble
        PORTEbits.RE1 = 1;      // Drive E pin high again
        PORTD = currentChar;    // Write lower nibble
        PORTEbits.RE1 = 0;      // Drive E pin low so LCD will process byte
        __delay_ms(10);         // wait 10 ms	
        PORTEbits.RE0 = 1;      // Drive RS pin high for displayable characters
        tempPtr++;              // Increment pointerto next character
    }
}

/*******************************
 * Toggle_LEDs(void)
 *
 * This function simply toggles the QwikFlash's red LEDs in a simple sequence
 * The LED_X defines are at the top of this .c file
 * 
 * You may alter this function if you like or directly manipulate LEDs in other functions
 ********************************/
void Toggle_LEDs(void)
{
    LED_LEFT ^= 1;      // regardless of the bit's previous state, this flips it to 1 or 0
    __delay_ms(100);
    
    LED_CENTER ^= 1;
    __delay_ms(100);
    
    LED_RIGHT ^= 1;
    __delay_ms(100);
}

void initialize_timer(){
    //timer0 settings
    T0CONbits.T08BIT = 0;       // 16-bit timer
    T0CONbits.T0CS = 0;         // internal clock
    T0CONbits.T0SE = 0;         // low-to-high transition
    T0CONbits.PSA = 0;          // enable prescaler
    T0CONbits.T0PS = 0b101;     // 64-bit prescaler
    
    //calculate load time, load into registers
    unsigned int x = (unsigned int)(2.5e6/64);
    unsigned int pre = 65535 - x;
    
    unsigned char high = (pre >> 8);
    unsigned char low = (pre << 8) >> 8;
    TMR0H = high;
    TMR0L = low;
}


// Always have at least 1 blank line at the end of the .c file


