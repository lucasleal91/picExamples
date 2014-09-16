// ******************************************************************************************* //
//
// File:         exercise7.c
// Date:         09-02-2014
// Authors:      Garrett Vanhoy
// ******************************************************************************************* //

// Include file for PIC24FJ64GA002 microcontroller. This include file defines
// MACROS for special function registers (SFR) and control bits within those
// registers.

#include "p24fj64ga002.h"
#include <stdio.h>

// These settings are appropriate for debugging the PIC microcontroller. If you need to
// program the PIC for standalone operation, change the COE_ON option to COE_OFF.

_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF &
          BKBUG_ON & COE_ON & ICS_PGx1 &
          FWDTEN_OFF & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS32768 )

/*_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF &  //for stand alone operation
          BKBUG_ON & COE_OFF & ICS_PGx1 &
          FWDTEN_OFF & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS32768 )*/

// ******************************************************************************************* //
// Configuration bits for CONFIG2 settings.
// Make sure "Configuration Bits set in code." option is checked in MPLAB.
// This option can be set by selecting "Configuration Bits..." under the Configure
// menu in MPLAB.

_CONFIG2( IESO_OFF & SOSCSEL_SOSC & WUTSEL_LEG & FNOSC_PRIPLL & FCKSM_CSDCMD & OSCIOFNC_OFF &
          IOL1WAY_OFF & I2C1SEL_PRI & POSCMOD_XT )

// ******************************************************************************************* //
volatile int count = 0;

int main(void)
{

    //TMR1 (bass) (.5 sec)
    TMR1 = 0;
    PR1 = 28800;
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
    T1CON = 0x8030;

    //TMR2 (snare) (.25 sec)
    TMR2 = 0;
    PR2 = 14400;
    IFS0bits.T2IF = 0;
    IEC0bits.T2IE = 1;
    T2CON = 0x8030;

    //TMR3 (guitar) (.1 sec)
    TMR3 = 0;
    PR3 = 5760;
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;
    T3CON = 0x8030;
    
    //TMR4-5
    TMR4 = 0;
    PR4 = 25856;
    PR5 = 4;
    IFS1bits.T5IF = 0;
    IEC1bits.T5IE = 1;
    T4CON = 0x8030;
    T4CONbits.T32 = 1;

    //SW1 Input
    TRISBbits.TRISB5 = 1;

    //LEDs are output
    TRISBbits.TRISB15 = 0;
    TRISBbits.TRISB14 = 0;
    TRISBbits.TRISB13 = 0;
    TRISBbits.TRISB12 = 0;

    //Default off
    LATBbits.LATB15 = 1;
    LATBbits.LATB14 = 1;
    LATBbits.LATB13 = 1;
    LATBbits.LATB12 = 1;

    //Enable CN for RB5 (SW1)
    CNEN2bits.CN27IE = 1;
    IFS1bits.CNIF = 0;
    IEC1bits.CNIE = 1;

    while(1){
    }
	return 0;
}

void __attribute__((interrupt)) _CNInterrupt(void){
    count = count + 1;
    if(count > 8){
        count = 0;
        LATBbits.LATB15 = 1;
        LATBbits.LATB14 = 1;
        LATBbits.LATB13 = 1;
        LATBbits.LATB12 = 1;
    }
    IFS1bits.CNIF = 0;
}

void __attribute__((interrupt)) _T1Interrupt(void){
    if(count > 1){
        LATBbits.LATB15 = !LATBbits.LATB15;
    }
    TMR1 = 0;
    IFS0bits.T1IF = 0;
}

void __attribute__((interrupt)) _T2Interrupt(void){
    if(count > 3){
        LATBbits.LATB14 = !LATBbits.LATB14;
    }
    TMR2 = 0;
    IFS0bits.T2IF = 0;
}

void __attribute__((interrupt)) _T3Interrupt(void){
    if(count > 5){
        LATBbits.LATB13 = !LATBbits.LATB13;
    }
    TMR3 = 0;
    IFS0bits.T3IF = 0;
}

void __attribute__((interrupt)) _T5Interrupt(void){
    if(count > 7){
        LATBbits.LATB12 = !LATBbits.LATB12;
    }
    TMR4 = 0;
    TMR5 = 0;
    IFS1bits.T5IF = 0;
}