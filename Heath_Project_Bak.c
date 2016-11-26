/*
 * File:   Heath_Project.c
 * Author: Heath
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
unsigned char pressuremb();
int temperature();
double altitudem();

//Magnometer
int compass();

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



////////////////

void main(void) {
    
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
    
   
    APFCON0 = 0x84; //This sets pins RC5 and RC4 as RX & TX on pic16f1829
   
    PORTA   =       0;
    setup_comms();	// set up the USART - settings defined in usart.h
    LATA =0;       //test flash led
    
 
 
    setup_i2c();
   
    setup_Baro();
    setup_Gyro();
    setup_Mag();
    setup_Accel();
    
    
    while(1){
        printf("w ");
               
        
        int temp_var = temperature();
        int temp_varp = pressuremb();
        
        double alt = altitudem();
        
        printf("Temp is %dC \tPressure is %dmb\talttitude is %dm\theading %d\n", temp_var, temp_varp, alt, compass());
        printf("Gyro reading\t%d roll\t%d pitch\t%d yaw\n", Groll(), Gpitch(), Gyaw());
        printf("Accel reading\t%d roll\t%d pitch\t%d yaw\n", Aroll(), Apitch(), Ayaw());
       
        delay();         // Delay a bit to make it slow
        //RA5 = 1;
        delay();
        //RA5 = 0;
           
    }
    
    return;
}


void   delay (void){
    int j;
    for (j=0; j< 0xFFF; j++) continue ;
    return;
 }

