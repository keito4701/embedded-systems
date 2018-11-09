; Embedded 1: Lecture 5 - PIC IO
; ASM examples
	
#include "p18f452.inc"

;;  block comment is CTRL /
;;  hotkeys: Tools > Options > Keymap

;;  To view the PORTB and TRISB pins in a tabular format
;;	Window > PIC Memory Views > SFRs
;;	Scroll to find the registers you wish to view
    
;;  To view the PORTB and TRISB pins in a graphical/logical format
;;	Window > Simulator > Analyzer
;;	Set a breakpoint and hit "Debug Main Project"
;;	To add pins click "Edit Pin Channel Definitions" in the Logic Analyzer
;;	Step Through your program to see changes (over time) in the Logic Analyzer
	
;;  To view the VAR_x "variable" stored in the File Registers
;;	Window > PIC Memory Views > File Registers
;;	Scroll to find the registers you wish to view
	
;;  To apply logic/stimulus to pins
;;	Window > Simulator > Stimulus
;;	Under Asynchronous tab
;;	    Pick "Pin" you wish to control
;;	    Pick "Action" you wish to take (HIGH, LOW, TOGGLE)
;;	    Debug the program, step through, and click the "Fire" button in Stimulus
;------------------------------------------------------------------------------
    
	VAR_1	EQU	0x20	;My 1st "variable" stored at File Register address 0x20
	VAR_2	EQU	0x21	;My 2nd "variable" stored at File Register address 0x21
	
	ORG		0x00
	
;OUTPUT Example - START--------------------------------------------------------
LOOP_1	MOVLW   0			;All 0?s to WREG (clear the Working Register)
		MOVWF   TRISB		;Make PORTB an OUTPUT for every pin
		MOVWF	PORTB		;Clear PORTB initially
		NOP
		NOP
		NOP
		MOVLW   B'10101010' ;Prepare number to write to PORTB pins
		MOVWF   PORTB	    ;Write 1/0 to PORTB pins
		NOP
		NOP
		GOTO	LOOP_1	    ;Go back to beginning (while(1))
		;Comment Out the above "GOTO" to continue to the next example below
;OUTPUT Example - END----------------------------------------------------------
	
		NOP
		NOP
		NOP
	
;INPUT Example - START---------------------------------------------------------
		MOVLW   0	    	;All 0?s to WREG (clear the Working Register)
		MOVWF	TRISB	    ;Clear TRISB initially
		MOVWF	PORTB	    ;Clear PORTB initially
		MOVLW	B'11111111' ;Make PORTB an INPUT for every pin
		MOVWF   TRISB	    ;Make PORTB an INPUT for every pin
		NOP
		NOP		    ;FIRE Stimulus around here when stepping through
		NOP
		NOP
		MOVF	PORTB, W    ;Copy PORTB's value into the Working Register
		MOVWF	VAR_1	    ;Copy the WREG's value into my "variable" 
		NOP
		NOP
		MOVFF	PORTB, VAR_2;Another method to copy a PORT into another 
		NOP		    		;File Register location without using the WREG
		NOP
		GOTO	LOOP_1	    ;Go back to beginning (while(1))
;INPUT Example - END-----------------------------------------------------------
	
END
    
	


