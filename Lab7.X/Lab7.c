/*******************************
 * Name: Nathan Reagan
 * Student ID#: 1001069209
 * Lab Day: 
 * CSE 3442/5442 - Embedded Systems 1
 * Lab 7 (ABET): Building a PIC18F4520 Standalone Alarm System with EUSART Communication 
 ********************************/
 
#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>


// PIC18F452 Configuration Bit Settings
#pragma config OSC     = HS     // Oscillator Selection bits (HS oscillator)
#pragma config PWRT    = OFF    // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE   = ON     // Brown-out Reset Enable bit (Brown-out Reset enabled)
#pragma config WDT     = OFF    // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config LVP     = OFF    // Low Voltage ICSP Enable bit (Low Voltage ICSP disabled)

#define _XTAL_FREQ 20000000     //If using the External Crystal of 20 MHz

void transmit (unsigned char c);
unsigned char receive ();
unsigned char keypadReceive ();
unsigned char receiveFinal ();

void transmitString (unsigned char* s);
void receiveString (unsigned char* s, int size);
void receiveStringKeypad (unsigned char* s, int size);
void receiveStringFinal (unsigned char* s, int size);
void newLine();
void displayMenu();

void passcodeHandler();
void changePasscode();

void pirHandler();  //PIR functions
void disablePIR();
void enablePIR();

void tempHandler(); //TEMP sensor functions
void enableTemp();
void disableTemp();
void adjustTemp();
void initialize_timer();
void interrupt low_priority Timer(void);
int getTemp();
int tempLow;
int tempHigh;

void inputHandler();//INPUT functions
void enableKeyboard();
void enableKeypad();

void interrupt RB0_Interrupt(void);
void Red_LED();

void enabledDisabled(unsigned char c);
void validatePassword();
void EEPROM_WriteByte(unsigned char eepromAddress, unsigned char eepromData);
unsigned char EEPROM_ReadByte(unsigned char eepromAddress);

void EEPROM_WriteString(unsigned char* eepromAddress, unsigned char* eepromData, int size);
void EEPROM_ReadString(unsigned char* eepromAddress, unsigned char* eepromData, int size);

void putch(unsigned char data);
void clear();

unsigned char password[5];      //4 char password
unsigned char keypad = 0;       //enable/disable keypad (default disabled)
unsigned char tempSensor = 0;   //enable/disable temp sensor (default disabled)

unsigned char newAddress = 0x00;        //address to determine whether first boot
unsigned char passwordAddress[] = {0x01, 0x02, 0x03, 0x04};     //address for password storage
unsigned char pirAddress = 0x05;        //address for PIR enable/disable
unsigned char inputAddress = 0x06;      //address for input method
unsigned char tempAddress = 0x07;       //address for temp sensor
unsigned char lowAddress[] = {0x08, 0x09, 0x0A};
unsigned char highAddress[] = {0x0B, 0x0C, 0x0D};

int recentTemperature;
char printTemp[4];

int isValidEntry(unsigned char c);
void waitForEnter();


//*********** MAIN ***********
void main()
{
    TRISDbits.RD1 = 0;	//RD1 is an OUTPUT (Green LED)
    PORTDbits.RD1 = 0;  //RD1 (Red LED) is disabled
    TRISDbits.RD0 = 0;	//RD0 is an OUTPUT (Red LED)
    PORTDbits.RD0 = 0;  //RD0 (Red LED) is disabled
    TRISDbits.RD2 = 0;  //RD2 (Blue LED) is an OUTPUT
    TRISAbits.RA4 = 0;  //RA4 (yellow LED) is an OUTPUT
    
    TRISBbits.RB5 = 0;  //output (horizontal) pins for keypad
    TRISBbits.RB4 = 0;
    TRISBbits.RB3 = 0;
    TRISBbits.RB2 = 0;
    PORTBbits.RB5 = 0;  //output (horizontal) pins for keypad set to 0
    PORTBbits.RB4 = 0;
    PORTBbits.RB3 = 0;
    PORTBbits.RB2 = 0;
    
    TRISCbits.RC5 = 1;  //input (vertical) pins for keypad
    TRISCbits.RC4 = 1;
    TRISDbits.RD3 = 1;
    
    TRISAbits.RA1 = 1;  //input for temp sensor
    
    ADCON1bits.PCFG = 0b1101;   //AN12:AN2 = D, AN1:AN0 = A
    
    //GET INPUT METHOD
    unsigned char inputStatus = EEPROM_ReadByte(inputAddress);
    if(inputStatus == 0xFF)
        enableKeyboard();
    else
        enableKeypad();
    PORTDbits.RD2 = keypad;
    
    //GET INPUT METHOD
    unsigned char tempStatus = EEPROM_ReadByte(tempAddress);
    if(tempStatus == 0xFF)
        disableTemp();
    else
        enableTemp();
    
    unsigned char enterNew = EEPROM_ReadByte(newAddress);  //check whether first time boot
    
    clear();
    //IF FIRST TIME BOOT
    if(enterNew == 0xFF) {
        //Enter new password
        transmitString("Enter new password: ");  
        receiveStringFinal(password, 4);
        enterNew = 0;
        
        //store in EEPROM
        EEPROM_WriteByte(newAddress, enterNew);
        EEPROM_WriteString(passwordAddress, password, 4);
        
        //set temp ranges
        tempLow = 0;
        tempHigh = 100;
    } else {
        EEPROM_ReadString(passwordAddress, password, 4);    //read in existing password
        
        //read in temp
        tempLow = (EEPROM_ReadByte(lowAddress[0])-48) * 100;
        tempLow += (EEPROM_ReadByte(lowAddress[1])-48) * 10;
        tempLow += EEPROM_ReadByte(lowAddress[2]) - 48; 

        tempHigh = (EEPROM_ReadByte(highAddress[0])-48) * 100;
        tempHigh += (EEPROM_ReadByte(highAddress[1])-48) * 10;
        tempHigh += EEPROM_ReadByte(highAddress[2]) - 48;   
    }
    
    //Force user to login
    validatePassword();
    
    //*********** INTERRUPTS ***********
    //PIR INTERRUPT
    TRISBbits.RB0 = 1;        // RB0 (Motion) is an input
    
    //enable/disable PIR (based on EEPROM) for INTI0E
    unsigned char pirStatus = EEPROM_ReadByte(pirAddress);
    if(pirStatus == '0')
        enablePIR();
    else
        disablePIR();
    
    INTCON2 = 0b11110000;
    INTCON3 = 0b00001000;
    
    RCONbits.IPEN = 1;      // Allow HIGH and LOW priority interrupts (instead of all HIGH)
    INTCONbits.PEIE = 1;    // Enable for LOW interrupts
    INTCONbits.GIE = 1; // Enable (release the hold) for HIGH interrupts (that have been indiv. enabled)*/
            
    initialize_timer();
    //*********** MAIN WHILE LOOP ***********
	while(1)
	{   
        displayMenu();
        unsigned char option = 0;
        
        option = receiveFinal();
        newLine(); newLine();
        
        switch(option) {
            case '1' :
                passcodeHandler();
                break;
            case '2' :
                pirHandler();
                break;
            case '3' :
                tempHandler();
                break;
            case '4' :
                inputHandler();
                break;
            case '5' :
                newLine();
                exit(0);
            default :
                break;
        }
        
	} //end of while(1)
	
} //end of void main()

void passcodeHandler(){
    transmitString("1. Change passcode"); newLine();
    transmitString("2. Return to main menu"); newLine(); newLine();
    transmitString("> ");
    char option = receiveFinal(); newLine(); newLine();
    
    switch(option) {
        case '1':
            changePasscode();
            break;
        default:
            break;
    }
}

void changePasscode() {
    validatePassword();
    
    transmitString("Enter new password: ");  
    receiveStringFinal(password, 4);
    newLine();
    
    EEPROM_WriteString(passwordAddress, password, 4);
}

void pirHandler() {
    transmitString("1. Enable PIR sensor"); newLine();
    transmitString("2. Disable PIR sensor"); newLine(); 
    transmitString("3. Return to main menu"); newLine(); newLine();
    transmitString("> ");
    char option = receiveFinal(); newLine(); newLine();
    
    switch(option) {
        case '1':
            enablePIR();
            break;
        case '2':
            disablePIR();
            break;
        default:
            break;
    }
}

void enablePIR() {
    INTCONbits.INT0IE = 1;
    INTCONbits.INT0IF = 0;
    
    EEPROM_WriteByte(pirAddress, '0');
}

void disablePIR() {
    INTCONbits.INT0IE = 0;
    INTCONbits.INT0IF = 0;
    
    EEPROM_WriteByte(pirAddress, 0xFF);
}

void tempHandler() {
    char templ[4];
    templ[0] = (char)(tempLow/100 + '0');
    templ[1] = (char)(tempLow%100/10 + '0');
    templ[2] = (char)(tempLow%10 + '0');
    templ[3] = 0;
    
    char temph[4];
    temph[0] = (char)(tempHigh/100 + '0');
    temph[1] = (char)(tempHigh%100/10 + '0');
    temph[2] = (char)(tempHigh%10 + '0');
    temph[3] = 0;
    
    transmitString("1. Enable TEMP sensor"); newLine();
    transmitString("2. Disable TEMP sensor"); newLine(); 
    transmitString("3. Adjust TEMP sensor ("); 
    transmitString(templ); transmitString("-"); transmitString(temph); transmitString(")"); newLine(); 
    transmitString("4. Return to main menu"); newLine(); 
    transmitString(" (Current temp: "); transmitString(printTemp); transmitString (" F)"); newLine(); newLine();
    transmitString("> ");
    char option = receiveFinal(); newLine(); newLine();
    
    switch(option) {
        case '1':
            enableTemp();
            break;
        case '2':
            disableTemp();
            break;
        case '3':
            adjustTemp();
            break;
        default:
            break;
    }
    
}

void enableTemp(){
    tempSensor = 1;
    EEPROM_WriteByte(tempAddress, '0');
}

void disableTemp() {
    tempSensor = 0;
    EEPROM_WriteByte(tempAddress, 0xFF);
}

void adjustTemp() {
    char tempTemp[4];
    
    transmitString("Enter minimum temperature (F)"); newLine(); newLine();
    transmitString("> ");       
    receiveStringFinal(tempTemp, 3); newLine(); newLine();
    tempLow = atoi(tempTemp); 
    
    transmitString("Enter maximum temperature (F)"); newLine(); newLine();
    transmitString("> "); 
    receiveStringFinal(tempTemp, 3); newLine(); newLine();
    tempHigh = atoi(tempTemp);
    
    char templ[4];
    templ[0] = (char)(tempLow/100 + '0');
    templ[1] = (char)(tempLow%100/10 + '0');
    templ[2] = (char)(tempLow%10 + '0');
    templ[3] = 0;
    
    char temph[4];
    temph[0] = (char)(tempHigh/100 + '0');
    temph[1] = (char)(tempHigh%100/10 + '0');
    temph[2] = (char)(tempHigh%10 + '0');
    temph[3] = 0;
    
    EEPROM_WriteString(lowAddress, templ, 3);
    EEPROM_WriteString(highAddress, temph, 3);
}

void initialize_timer(){
    //timer0 settings
    T0CONbits.T08BIT = 0;       // 16-bit timer
    T0CONbits.T0CS = 0;         // internal clock
    T0CONbits.T0SE = 0;         // low-to-high transition
    T0CONbits.PSA = 0;          // enable prescaler
    T0CONbits.T0PS = 0b111;     // 256-bit prescaler
    
    //calculate load time, load into registers
    unsigned int x = (unsigned int)(10e6/256);
    unsigned int pre = 65535 - x;
    
    unsigned char high = (pre >> 8);
    unsigned char low = (pre << 8) >> 8;
    TMR0H = high;
    TMR0L = low;
    
    INTCONbits.TMR0IE = 1;  // timer0 enabled
    INTCONbits.TMR0IF = 0;  // timer0 enabled
}

//only perform interrupt if switch (B2) is off
void interrupt low_priority Timer(void) {
    if( (INTCONbits.TMR0IE == 1) && (INTCONbits.TMR0IF == 1) ){
        getTemp();
        initialize_timer();
    }
}

int getTemp() {
    //flash LED
    PORTAbits.RA4 = 1;
     
    ADCON1bits.VCFG = 0b00; //use Vss and Vdd ref, AN1 and AN2 analog inputs (already configured)
    ADCON0 = 0b00000101;    //AN1 input, GO disabled, AD on 
    ADCON2 = 0b10111010;    //right justified, 20 Tad acq time, Fosc/32
    
    ADCON0bits.GO = 1;
    while(ADCON0bits.DONE == 1) ;
        
    //combine into a single 10-bit number
    long adLow = ADRESL;
    long adHigh = ADRESH;
    
    PIR1bits.ADIF = 0;
    long fullAD = adHigh << 8;
    fullAD += adLow;
    
    //temp range -40C (.1 V) to 125C (1.75 V)
    //temp range -40F to 257F
    double temp1 = (fullAD*(5.0)/((1 << 10) - 1));     //range of .1 to 1.75 V
    int temp = (int)(temp1*180 - 58);   //convert to F

    
    recentTemperature = temp;
    
    printTemp[0] = (char)(temp/100 + '0');
    printTemp[1] = (char)(temp%100/10 + '0');
    printTemp[2] = (char)(temp%10 + '0');
    printTemp[3] = 0;
    
    if(tempSensor == 1 && (temp > tempHigh || temp < tempLow)) {
        clear();
    
        validatePassword(); //check for user password
        PORTAbits.RA4 = 0;  //RD0 (LED) is disabled

        transmitString("1. Adjust TEMP threshold"); newLine(); 
        transmitString("2. Return to main menu"); newLine(); newLine();
        transmitString("> ");
        char option = receiveFinal(); newLine(); newLine();

        switch(option) {
            case '1':
                adjustTemp();
                break;
            default:
                break;
        }
        
        displayMenu();
    }
        
    
    //transmitString(printTemp);
    
    //turn off LED
    PORTAbits.RA4 = 0;
}

void inputHandler() {
    transmitString("1. Keyboard Input"); newLine();
    transmitString("2. Keypad Input"); newLine(); newLine();
    transmitString("> ");
    char option = receiveFinal(); newLine(); newLine();
    
    switch(option) {
        case '1':
            enableKeyboard();
            break;
        case '2':
            enableKeypad();
            break;
        default:
            break;
    }
    
    PORTDbits.RD2 = keypad;
}

void enableKeyboard(){
    keypad = 0;
    EEPROM_WriteByte(inputAddress, 0xFF);
}

void enableKeypad() {
    keypad = 1;
    EEPROM_WriteByte(inputAddress, '0');
}

void interrupt RB0_Interrupt(void)
{
    // Was the triggered interrupt EXTERNAL INTERRUPT 1? (pin B0)
    // If the interrupt IS ENABLED and its FLAG IS SET
    if((INTCONbits.INT0IE == 1) && (INTCONbits.INT0IF == 1)) {
        Red_LED();
        INTCONbits.INT0IF = 0;  // Clear INT0 Flag bit ("reset" it)
    }
}

void Red_LED(){
    clear();
    
    PORTDbits.RD0 = 1;  //RD0 (LED) is enabled
    validatePassword(); //check for user password
    PORTDbits.RD0 = 0;  //RD0 (LED) is disabled
    
    transmitString("1. Disable PIR sensor"); newLine(); 
    transmitString("2. Return to main menu"); newLine(); newLine();
    transmitString("> ");
    char option = receiveFinal(); newLine(); newLine();
    
    switch(option) {
        case '1':
            disablePIR();
            break;
        default:
            break;
    }
    
    displayMenu();
}

//transmits char c FROM the pic TO the user
//Fosc = 20 MHz, KBAUD = 9.62, BAUD rate = 9.6 kbps, %error = .16
void transmit (unsigned char c) {
	TRISCbits.RC7 = 1;	//RX is INPUT
	SPBRG = 129;		//BAUD for Fosc = 20 MHz
	TXSTAbits.SYNC = 0;	//Async mode
	TXSTAbits.BRGH = 1;	//HIGH speed BAUD rate
	RCSTAbits.RX9 = 0;	//8-bit reception
	RCSTAbits.SPEN = 1;	//Serial port ENABLED (both RX and TX)
	TXSTAbits.TXEN = 1;	//Enable transmitter (turn ON)
	
	while(TXSTAbits.TRMT == 0);	//wait until transmission is done
	
	TXREG = c;					//send out byte
}

//receives input TO the pic FROM the user
//Fosc = 20 MHz, KBAUD = 9.62, BAUD rate = 9.6 kbps, %error = .16
unsigned char receive () {
    unsigned char c = 0;
    
    while(!isValidEntry(c)) {
        TRISCbits.RC7 = 1;	//RX is INPUT
        SPBRG = 129;		//BAUD for Fosc = 20 MHz
        TXSTAbits.SYNC = 0;	//Async mode
        TXSTAbits.BRGH = 1;	//HIGH speed BAUD rate
        RCSTAbits.RX9 = 0;	//8-bit reception
        RCSTAbits.SPEN = 1;	//Serial port ENABLED (both RX and TX)
        RCSTAbits.CREN = 1;	//Enable continuous receiver (turn ON)

        while(PIR1bits.RCIF == 0);	//wait for incoming data

        c = RCREG;	//reading RCREG clears RCIF flag
    }

    transmit(c);
    __delay_ms(10);
	return c;					//return received byte
}

unsigned char keypadReceive () {
    unsigned char option = 0;
    
    while(!isValidEntry(option)) {
        while(option == 0) {
            PORTBbits.RB5 = 1;  //enable first row
            if(PORTCbits.RC5 == 1) {
                option = '1'; 
                while(PORTCbits.RC5 == 1);
            }
            if(PORTCbits.RC4 == 1) {
                option = '2';   
                while(PORTCbits.RC4 == 1);   
            }
            if(PORTDbits.RD3 == 1) {
                option = '3';
                while(PORTDbits.RD3 == 1);
            }
            PORTBbits.RB5 = 0;  //disable first row

            PORTBbits.RB4 = 1;  //enable second row
            if(PORTCbits.RC5 == 1) {
                option = '4';
                while(PORTCbits.RC5 == 1);
            }
            if(PORTCbits.RC4 == 1) {
                option = '5';
                while(PORTCbits.RC4 == 1);   
            }
            if(PORTDbits.RD3 == 1) {
                option = '6';
                while(PORTDbits.RD3 == 1);   
            }
            PORTBbits.RB4 = 0;  //disable second row

            PORTBbits.RB3 = 1;  //enable third row
            if(PORTCbits.RC5 == 1) {
                option = '7';
                while(PORTCbits.RC5 == 1);
            }
            if(PORTCbits.RC4 == 1) {
                option = '8';
                while(PORTCbits.RC4 == 1);   
            }
            if(PORTDbits.RD3 == 1) {
                option = '9';
                while(PORTDbits.RD3 == 1);   
            }
            PORTBbits.RB3 = 0;  //disable third row

            PORTBbits.RB2 = 1;  //enable fourth row
            if(PORTCbits.RC4 == 1) {
                option = '0';
                while(PORTCbits.RC4 == 1);   
            }
            if(PORTDbits.RD3 == 1) {
                option = '#';
                while(PORTDbits.RD3 == 1);   
            }
            PORTBbits.RB2 = 0;  //disable fourth row
                __delay_ms(10);

        }
    }
    
    transmit(option);
    return option;
}

void keypadReceiveString (unsigned char* s, int size){
    for(int i = 0; i < size; i++) {
        s[i] = keypadReceive();
    }
    s[size] = '\0';
    
    waitForEnter();
    newLine();
}

//transmits string s FROM the user TO the PIC
void transmitString (unsigned char* s) {
    int size = strlen(s);
    
    for(int i = 0; i < size; i++)
        transmit(s[i]);
}

//receive a string of characters
void receiveString (unsigned char* s, int size){
    for(int i = 0; i < size; i++) {
        s[i] = receive();
    }
    
    s[size] = '\0';
    
    waitForEnter();
    
    newLine();
}

//checks for serial OR keypad input
unsigned char receiveFinal() {
    unsigned char c = 0;
    
    if(keypad == 0)
        c = receive();
    else c = keypadReceive();
    
    waitForEnter();
    
    return c;
}

//checks for serial OR keypad input
void receiveStringFinal(unsigned char* s, int size) {
    if(keypad == 0)
        receiveString(s, size);
    else
        keypadReceiveString(s, size);
}

void enabledDisabled(unsigned char c) {
    if(c == 0)
        transmitString(" (Disabled)");
    else if (c == 1)
        transmitString(" (Enabled)");
}

void validatePassword() {
    char validLogin = 0;
    while (validLogin == 0) {
        transmitString("Enter password: ");
        char attempt[5];
        receiveStringFinal(attempt, 4);

        if(strcmp(attempt, password) == 0) {
            newLine();          //display newline
            validLogin = 1;     //set login to valid
            
            PORTDbits.RD1 = 1;  //RD1 (green LED) is enabled
        }
    }
}

void newLine() {
    transmitString("\n\r\r");
}

//sends data FROM the user TO the EEPROM
void EEPROM_WriteByte(unsigned char eepromAddress, unsigned char eepromData)
{
    EEADR = eepromAddress;      // Write the address to EEADR.
    EEDATA = eepromData;        // load the 8-bit data value to be written in the EEDATA register.
    
    EECON1bits.EEPGD = 0;       // setup for writing
    EECON1bits.CFGS = 0;
    EECON1bits.WREN = 1;               
    
    EECON2 = 0x55;                // Execute the special instruction sequence
    EECON2 = 0xaa;                // Refer the datasheet for more info
    
    EECON1bits.WR = 1;          // Set the WR bit to trigger the eeprom write operation.
    while(EECON1bits.WR == 1);  //wait to clear to 0
}

//sends data TO the user FROM the EEPROM at a specified address
unsigned char EEPROM_ReadByte(unsigned char eepromAddress) {
    EEADR = eepromAddress;      // Write the address to EEADR
    
    EECON1bits.EEPGD = 0;       // setup for writing
    EECON1bits.CFGS = 0;
    EECON1bits.RD = 1;               
    
    char c = EEDATA;
    return c;
}

//sends data FROM the user TO the EEPROM
void EEPROM_WriteString(unsigned char* eepromAddress, unsigned char* eepromData, int size) {
    for(int i = 0; i < size; i++)
        EEPROM_WriteByte(eepromAddress[i], eepromData[i]);
}

//sends data TO the user FROM the EEPROM at a specified address
void EEPROM_ReadString(unsigned char* eepromAddress, unsigned char* eepromData, int size) {
    for(int i = 0; i < size; i++)
        eepromData[i] = EEPROM_ReadByte(eepromAddress[i]);
}

void putch(unsigned char data) {
    while(!PIR1bits.TXIF) // wait until the transmitter is ready
        continue;
    
    TXREG = data;   // send one character
}

void clear() {
    transmitString("\033[2J");
    transmitString("\033[0;0H");
}

void displayMenu() {
    transmitString("1. Passcode Options"); newLine();
    transmitString("2. PIR Sensor Alarm"); enabledDisabled(INTCONbits.INT0IE); newLine();
    transmitString("3. Temperature Sensor Alarm"); enabledDisabled(tempSensor); newLine();
    transmitString("4. Switch Input Method"); newLine();
    transmitString("5. Reset"); newLine(); newLine();

    transmitString("> ");
}

int isValidEntry(unsigned char c) {
    if(c == 13 || c == '#')
        return 1;
    if(c > 47 && c < 58)
        return 1;
    
    return 0;
}

void waitForEnter() {
    unsigned char e = 0;
    if(keypad)
        while(e != '#')
            e = keypadReceive();
    else
        while(e != 13)
            e = receive();
}
