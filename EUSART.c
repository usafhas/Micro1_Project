
#include <xc.h>
#include <stdio.h>
#include <stdlib.h>

#define BAUD 9600
#define FOSC 4000000L
#define DIVIDER ((int)(FOSC/(16UL * BAUD) -1))  //Should be 25 for 9600/4MhZ
#define NINE_BITS 0
#define SPEED 0x4
#define RX_PIN TRISC5
#define TX_PIN TRISC4
#define _XTAL_FREQ 4000000.0    /*for 4mhz*/
//  Function Prototypes:  Actual functions after main.  

    //OSCCON = 0x68;      //Sets CPU clock Fosc=4MHz
   // ANSELA = 0;
    //TRISA = 0;          //  Now turn on the PWM at the start
    //INTCON = 0;
    // TXCKSEL = 1;    // Note:!!!!!  OPPOSITE FOR 1825 VS 1829
   // RXDTSEL = 1;  /*makes RC4 & 5 TX & RX for USART (Allows ICSP)*/
   // ANSELA  = 0;
   // ANSELC  = 0;    /* all ADC pins to digital I/O */
   // TRISC   = 0;      /* set as output */
   // LATC    = 0x00;   /* zero out */




void setup_comms(void){

	RX_PIN = 1;
	TX_PIN = 1;
	SPBRG = DIVIDER;
	RCSTA = (NINE_BITS|0x90);
	TXSTA = (SPEED|NINE_BITS|0x20);
        TXEN=1;
        SYNC=0;
        SPEN = 1;
        BRGH=1;
}

void
putch(unsigned char byte)
{
	/* output one byte */
	while(!TXIF)	/* set when register is empty */
		continue;
	TXREG = byte;
}


void
putch_int(int byte)
{
	/* output one byte */
	while(!TXIF)	/* set when register is empty */
		continue;
	TXREG = byte;
}

unsigned char
getch() {
	/* retrieve one byte */
	while(!RCIF)	/* set when register is not empty */
		continue;
	return RCREG;
}

unsigned char
getche(void)
{
	unsigned char c;
	putch(c = getch());
	return c;
}