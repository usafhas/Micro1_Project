/*
 * File:  Micro_Project.c
 * Author:  Heath Spidle
 *          Michael Amaya
 *          Diana Dimitriu
 *
 * Created on March 8, 2016, 9:18 PM
 */
// PIC16F1829 Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1
#pragma config FOSC = INTOSC       // Oscillator Selection (ECH, External Clock, High Power Mode (4-32 MHz): device clock supplied to CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF        // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = OFF       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = ON        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)


#include <stdio.h>
#include <stdlib.h>

void delay(void);

//I2C Operation
unsigned char Receive(unsigned char addi2c);
unsigned char Receive_Reg(unsigned char addi2c, unsigned char Reg, unsigned char addi2cr);
void Transmit(unsigned char addi2c, unsigned char data);
void Transmit_Reg(unsigned char addi2c, unsigned char  Reg, unsigned char data);

//I2C Setup
void setup_i2c(void);
void setup_Baro();
void setup_Gyro();
void setup_Mag();
void setup_Accel();


//Barometer I2C
double pressuremb();
int temperature();
int altitudem();

//Magnometer
float compass();

int Aroll();
int Ayaw();
int Apitch();
int Gyaw();
int Groll();
int Gpitch();

unsigned char test;
unsigned char data;

////////////////////
void setup_comms(void);
void putch(unsigned char);
void putch_int(int byte);
unsigned char getch(void);
unsigned char getche(void);

unsigned int Tcount, Threshold;
int i,j,Touch[3], mode;


////////////////

void main(void) {
    //************ Setup all of the basic Parameters ************ //
    OSCCON = 0x68;      //Sets CPU clock Fosc=4MHz
    TRISA = 0X10;          //  Now turn on the PWM at the start
    INTCON = 0;
    TXCKSEL = 1;    // Note:!!!!!  OPPOSITE FOR 1825 VS 1829
    RXDTSEL = 1;  /*makes RC4 & 5 TX & RX for USART (Allows ICSP)*/
    ANSELA  = 0X10;
    ANSELC  = 0;    /* all ADC pins to digital I/O */
    TRISC   = 0;      /* set as output */
    LATC    = 0x00;   /* zero out */
    TRISC   = 0X88;
    //*************************************************************//
    
    /************************Touch Pads Program**************************/
    
   // first set up system stuff:  Fosc, T0 and t1
    Touch[0] = 3;  // RA4 pad CPS3
    Touch[1] = 7;  // RC3 pad CPS7
    Touch[2] = 9;  //RC7  pad CPS9

    int Temp;
    APFCON0 = 0x84; //This sets pins RC5 and RC4 as RX & TX on pic16f1829
    OPTION_REG =    0XC5;
    T1CON =         0XC1;
    T1GCON =        0X81;       //81 is without toggle mode A1 is toggle mode
    CPSCON1 =       0X03;
    CPSCON0 =       0X8C;
    PORTA   =       0;
    setup_comms();	// set up the USART - settings defined in usart.h
    LATA |=  (0X01 << 5);       //test flash led
    for (i=0; i< 0x7000; i++) continue;  //short delay
    LATA &= ~(1<<5);
    Threshold =     0x10F4; // ********************************* have to determine this value empirically
    
    // *************** Setup I2C / IMU registers *****************************//
    setup_i2c();
    setup_Baro();
    setup_Gyro();
    setup_Mag();
    setup_Accel();
    // **************  End Setup I2C / IMU Registers **********//
    
    int Gtemp  ,Galt;
    float Ghead;
   double Gpress;
    int Gr, Gp, Gy, Ar, Ap, Ay;
    
    
    while (1){  // Begin Infinite Loop
        mode=3; // if nothig pressed then case 3
//  Now initiliz the timer0 and 1 and scan 3 pads
       for (j=0; j<3; j++){ 
        CPSCON1 =  Touch[j];          
//  Nowinitiliz the timer0 and 1
            TMR1ON  =       0;
            TMR0    =       0;
            TMR1H   =       0;
            TMR1L   =       0;
            TMR1ON  =       1;
            TMR0IF  =       0;      //Clear the interrupt flag for Timer 1
            TMR0    =       0;
//  Now start the touch sensing part and UART output

        while (!TMR0IF) continue;   //wait here till Timer 0 overflows
            Tcount  =   (TMR1H<<8)+TMR1L;       //Save 16 bit count value in timer 1
            //printf("Tcount = %x, Threshold= %x \r\n", Tcount, Threshold);
            TMR0IF  =       0;      //Clear the interrupt flag for Timer 1
            if (Tcount < Threshold) 
                mode=j;
            } // end of For rotates through the 3 touch pads
        
        //THIS IS WHERE THE CASE SWITCH THING WOULD GO;     
            switch(mode){
                
                case 0:
                    
                    Gtemp =  temperature();
                    Gpress = pressuremb();
                    Galt = altitudem();
                    //Ghead = compass();
        
                    //double alt = altitudem();
                    printf("\nTemp is %dC \tPressure is %dmb\talttitude is %dm\r", Gtemp, Gpress, Galt);
                    break;
                
                case 1:
                    Gr = Groll();
                    Gp = Gpitch();
                    Gy = Gyaw();
                    printf("\nGyro reading \t\t%d roll \t\t%d pitch \t\t%d yaw\r", Gr, Gp, Gy);
                    break;
                    
                case 2:
                    Ar = Aroll();
                    Ap = Apitch(); 
                    Ay = Ayaw();
                   printf("\nAccel reading \t\t%d roll \t\t%d pitch \t\t%d yaw\r", Ar, Ap, Ay);
                    
                    break;
                    
                case 3:
                    printf("\33[2K Please Press a button \r");
                    delay();
                    
                    break;
                
                
            } // End Switch statement
  

  /*END OF TOUCH PORTION*/
            //delay();
    } // End of Infinite While
    
    return; //
} // End of Main


void   delay (void){
    int j;
    for (j=0; j< 0xFFF; j++) continue ;
    return;
 }

