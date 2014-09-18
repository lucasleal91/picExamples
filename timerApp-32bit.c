// ******************************************************************************************* //
//
// File:         timerApp-class.c
// Date:         08-17-2013
// Authors:      Garrett Vanhoy
//
// ******************************************************************************************* //

// Include file for PIC24FJ64GA002 microcontroller. This include file defines
// MACROS for special function registers (SFR) and control bits within those
// registers.

#include "p24fj64ga002.h"
#include <stdio.h>

// ******************************************************************************************* //
// Configuration bits for CONFIG1 settings.
//
// Make sure "Configuration Bits set in code." option is checked in MPLAB.
// This option can be set by selecting "Configuration Bits..." under the Configure
// menu in MPLAB.
//
// These settings are appropriate for debugging the PIC microcontroller. If you need to
// program the PIC for standalone operation, change the COE_ON option to COE_OFF.

_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF &
          BKBUG_ON & COE_OFF & ICS_PGx1 &
          FWDTEN_OFF & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS32768 )

// ******************************************************************************************* //
// Configuration bits for CONFIG2 settings.
// Make sure "Configuration Bits set in code." option is checked in MPLAB.
// This option can be set by selecting "Configuration Bits..." under the Configure
// menu in MPLAB.

_CONFIG2( IESO_OFF & SOSCSEL_SOSC & WUTSEL_LEG & FNOSC_PRIPLL & FCKSM_CSDCMD & OSCIOFNC_OFF &
          IOL1WAY_OFF & I2C1SEL_PRI & POSCMOD_XT )

// ******************************************************************************************* //
// Defines to simply UART's baud rate generator (BRG) regiser
// given the osicllator freqeuncy and PLLMODE.

#define XTFREQ          7372800         	  // On-board Crystal frequency
#define PLLMODE         4               	  // On-chip PLL setting (Fosc)
#define FCY             (XTFREQ*PLLMODE)/2    // Instruction Cycle Frequency (Fosc/2)

#define BAUDRATE         115200
#define BRGVAL          ((FCY/BAUDRATE)/16)-1

// ******************************************************************************************* //
volatile int state = 0; //State 0 is when I am not timing. State 1 is when I am timing.
volatile unsigned long int time = 0;
volatile int printed = 1;
volatile float counter = 0;
// ******************************************************************************************* //


int main(void)
{

	RPINR18bits.U1RXR = 9;
	RPOR4bits.RP8R = 3;

        TMR4 = 0;
        TMR5 = 0;
        PR4 = 0b0101110111111111;
        PR5 = 0b11010;
        IFS1bits.T5IF = 0;
        IEC1bits.T5IE = 1;
        T4CONbits.T32 = 1;
        T4CONbits.TON = 1;
        T4CONbits.TCKPS0 = 1;
        T4CONbits.TCKPS1 = 1;

        //Enable CN for switch RB5
        //default digital and already is debounced.
//        TRISBbits.TRISB5 = 1;
        CNEN2bits.CN27IE = 1;
        IFS1bits.CNIF = 0;
        IEC1bits.CNIE = 1;

	// Set UART1's baud rate generator register (U1BRG) to the value calculated above.
	U1BRG  = BRGVAL;
	U1MODE = 0x8000;
	U1STA  = 0x0440; 		// Reset status register and enable TX & RX
	IFS0bits.U1RXIF = 0;

	// The main loop for your microcontroller should not exit (return), as
	// the program should run as long as the device is powered on.
	while(1)
	{
            //TODO: Create two states for button press 1 and two
            //TODO: Print when the button is pressed only once.
            //TODO: Calculate time elapsed.
            if(printed == 0){
                printf("Press %d\n", state);
                if(state == 0){
                    printf("Time in second: %f\n", (float)time/57599+ 30*counter);
                }
                printed = 1;
            }
	}
	return 0;
}

// ******************************************************************************************* //
// Defines an interrupt service routine that will execute whenever Timer 1's
// count reaches the specfied period value defined within the PR1 register.
//
//     _ISR and _ISRFAST are macros for specifying interrupts that
//     automatically inserts the proper interrupt into the interrupt vector
//     table
//
//     _T1Interrupt is a macro for specifying the interrupt for Timer 1
//
// The functionality defined in an interrupt should be a minimal as possible
// to ensure additional interrupts can be processed.
//void _ISR _T1Interrupt(void)

//TODO: Add a T5 interrupt
void __attribute__((interrupt,auto_psv)) _T5Interrupt(void){
    IFS1bits.T5IF = 0;
    counter = counter + 1;
}
//TODO: Add a CN Interrupt. Whenever the button is pressed and the state is 0, switch state
//Also, set the counter to 0. Take the timer to 0.
//Print something every time you enter this function.
void __attribute__((interrupt,auto_psv)) _CNInterrupt(void){
        IFS1bits.CNIF = 0;

        if(PORTBbits.RB5 == 1 && state == 1){
            state = 0;
            time = TMR5; //Storing a 16-bit integer into a 32-bit long int
            time = time << 16; //Shift by 16 makes PR5 the most significant bits
            time = time + TMR4; //Now add the least significant bits.
            printed = 0;
        }
        else if(PORTBbits.RB5 == 1 && state == 0){
            state = 1;
            TMR4 = 0;
            TMR5 = 0;
            counter = 0;
            printed = 0;
        }
}

// ******************************************************************************************* //
