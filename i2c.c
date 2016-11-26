#include <xc.h>
#include <stdio.h>
#include <stdlib.h>

char data;
void setup_i2c(void){
 
 //SCL1 RB6  Set as Inputs
 //SDA1  RB4  Set as Inputs
    TRISB = 0b01010000;
 //The hold time of the SDAx pin is selected by the SDAHT bit of the SSPxCON3
    SSP1CON3 = 0b00000000; //300ns  
  //Master mode is enabled by setting and clearing the appropriate SSPM bits in the SSPxCON1 register and by setting the SSPEN bit
    SSP1CON1 = 0b00101000; //5=sspen 1000 = i2c Master FSOC/4
    //SSP1CON2 = 0x00;
    
    //Baud Rate Generator
    SSP1ADD = 0x09; //4MHz Clock
            
        return;
}

void wait(){
    //while(( SSP1CON2 & 0x1F ) || ( SSPSTAT & 0x04 ));
    
    //while(( SSP1CON2 & 0x1F ) || ( SSP1STAT & 0x04 ) || ( !SSP1IF) || !(SSP1CON2 | 0x60)); not 6
    
    while(( SSP1CON2 & 0x1F ) || ( SSP1STAT & 0x04 ) || ( !SSP1IF));
   
}
void waitr(){
    //while(( SSP1CON2 & 0x1F ) || ( SSPSTAT & 0x04 ));
    
    while(( SSP1CON2 & 0x1F ) || ( SSP1STAT & 0x04 ) || !(SSP1CON2 | 0x60));
    
    //while(( SSP1CON2 & 0x1F ) || ( SSP1STAT & 0x04 ) || ( !SSP1IF));
   
}

void Transmit(unsigned char addi2c,unsigned char data){
   
    //Start condition
    SSP1CON2 = 0x01;
    //load slave address
    SSP1BUF = addi2c; // variable for address
    SSP1BUF = data; //variable for data to send
    //Stop Data
    SSP1CON2 = 0x04;
    return;
}

void Transmit_Reg(unsigned char addi2c, unsigned char Reg, unsigned char data){
    //Start condition
    SSP1CON2 = 0x01; //Send Start Bit
    while(!SSP1IF);//wait for start bit interrupt to be set
    SSP1IF = 0; //clear the interrupt flag
    //load slave address
    wait();
    SSP1BUF = addi2c; // variable for address
    wait(); //While buffer is full wait
    //while SSPSTAT & 0x01
    //restart bit
    ///SSP1CON2 = 0x02; // restart enable bit
    //while(!SSP1IF); //wait for restart condition to clear
    //SSP1IF = 0;
    wait();
    SSP1BUF = Reg;// write the registery address to send
    wait();
    
    
    //SSP1CON2 = 0x02; // restart enable bit
    //while(!SSP1IF); //wait for restart condition to clear
    //SSP1IF = 0;
    wait();
    SSP1BUF = data; //variable for data to send to registry
    wait();
    //Stop Data
    SSP1CON2 = 0x04;
    wait();
    return;
}

unsigned char Receive(unsigned char addi2c){
   
    //Start condition
    SSP1CON2 = 0x01;
    SSP1BUF = addi2c; // variable for address
    //program to receive
    SSP1CON2 = 0x08; //RCEN
    data = SSP1BUF;
    //Stop Data
    SSP1CON2 = 0x04;
    
    return data;
}

unsigned char Receive_Reg(unsigned char addi2c, unsigned char Reg, unsigned char addi2cr){
    
    //Start condition
    SSP1CON2 = 0x01;
    while(!SSP1IF); //wait for start condition to clear
    SSP1IF = 0;
    waitr();
    SSP1BUF = addi2c; // variable for i2c address
    waitr();
    
   // SSP1CON2 = 0x02; // restart enable bit
    //while(!SSP1IF); //wait for restart condition to clear
   // SSP1IF = 0;
    waitr();
    SSP1BUF = Reg; //register to receive from
    waitr();
    
    SSP1CON2 = 0x02; // restart enable bit
    while(!SSP1IF); //wait for restart condition to clear
    SSP1IF = 0;
    waitr();
    SSP1BUF = addi2cr; // variable for address with read bit set
    waitr();
    
    //program to receive
    SSP1CON2 = 0x08; //RCEN
    waitr(); //wait for Buffer to be full
    data = SSP1BUF;  //receive data
    //set ackbit
    SSP1CON2 = SSP1CON2 | 0x20;
    SSP1CON2 = SSP1CON2 | 0x10; // ack bit
    while(!SSP1IF);
    SSP1IF = 0;
    //Stop Data
    // need ack bit
    SSP1CON2 = 0x04;// stop bit
    while(!SSP1IF);
    waitr();
    SSP1IF = 0;
    
    return data;
}



//From Data Sheet
/*Master mode is enabled by setting and clearing the
appropriate SSPM bits in the SSPxCON1 register and
by setting the SSPEN bit. In Master mode, the SDAx
and SCKx pins must be configured as inputs */

/*A Baud Rate Generator is used to set the clock
frequency output on SCLx. See Section 25.7 ?Baud
Rate Generator? for more detail.*/

/*To initiate a Start condition (Figure 25-26), the user
sets the Start Enable bit, SEN bit of the SSPxCON2
register. If the SDAx*/

/*Transmission of a data byte, a 7-bit address or the
other half of a 10-bit address is accomplished by simply
writing a value to the SSPxBUF register*/

/*25.6.6.4 Typical Transmit Sequence:
1. The user generates a Start condition by setting
the SEN bit of the SSPxCON2 register.
2. SSPxIF is set by hardware on completion of the
Start.
3. SSPxIF is cleared by software.
4. The MSSPx module will wait the required start
time before any other operation takes place.
5. The user loads the SSPxBUF with the slave
address to transmit.
6. Address is shifted out the SDAx pin until all eight
bits are transmitted. Transmission begins as
soon as SSPxBUF is written to.
7. The MSSPx module shifts in the ACK bit from
the slave device and writes its value into the
ACKSTAT bit of the SSPxCON2 register.
8. The MSSPx module generates an interrupt at
the end of the ninth clock cycle by setting the
SSPxIF bit.
9. The user loads the SSPxBUF with eight bits of
data.
10. Data is shifted out the SDAx pin until all eight
bits are transmitted.
11. The MSSPx module shifts in the ACK bit from
the slave device and writes its value into the
ACKSTAT bit of the SSPxCON2 register.
12. Steps 8-11 are repeated for all transmitted data
bytes.
13. The user generates a Stop or Restart condition
by setting the PEN or RSEN bits of the
SSPxCON2 register. Interrupt is generated once
the Stop/Restart condition is complete.*/

/*25.6.7.4 Typical Receive Sequence:
1. The user generates a Start condition by setting
the SEN bit of the SSPxCON2 register.
2. SSPxIF is set by hardware on completion of the
Start.
3. SSPxIF is cleared by software.
4. User writes SSPxBUF with the slave address to
transmit and the R/W bit set.
5. Address is shifted out the SDAx pin until all eight
bits are transmitted. Transmission begins as
soon as SSPxBUF is written to.
6. The MSSPx module shifts in the ACK bit from
the slave device and writes its value into the
ACKSTAT bit of the SSPxCON2 register.
7. The MSSPx module generates an interrupt at
the end of the ninth clock cycle by setting the
SSPxIF bit.
8. User sets the RCEN bit of the SSPxCON2
register and the Master clocks in a byte from the
slave.
9. After the 8th falling edge of SCLx, SSPxIF and
BF are set.
10. Master clears SSPxIF and reads the received
byte from SSPxUF, clears BF.
11. Master sets ACK value sent to slave in ACKDT
bit of the SSPxCON2 register and initiates the
ACK by setting the ACKEN bit.
12. Masters ACK is clocked out to the slave and
SSPxIF is set.
13. User clears SSPxIF.
14. Steps 8-13 are repeated for each received byte
from the slave.
15. Master sends a not ACK or Stop to end
communication.*/