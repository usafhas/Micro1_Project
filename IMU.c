#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*
https://www.pololu.com/product/2469
https://www.pololu.com/product/2469
AltIMU-10v  Sensor

Slave Address

Gyro (L3GD20) 1101011b
Accelerometer (LSM303DLHC) 0011001b   0100100b
Magnetometer (LSM303DLHC) 0011110b
Barometer (LPS331AP) 1011101b

6b =gyro
5b =barometer
1e =magnometer
19 =accel

*/
/*
write address
write register
write value
end
*/
/*
read register

write address
write register
read value
end
*/
/*
#define gyroADD 0x6b
#define baroADD 0x5b
#define magADD 0x1e
#define accelADD 0x19
*/
//write bit set
#define gyroADD 0xD6
#define baroADD 0xBA
#define magADD 0x3C
#define accelADD 0x32
//read bit set
#define gyroADDr 0xD7
#define baroADDr 0xBB
#define magADDr 0x3D
#define accelADDr 0x33


void Transmit_Reg(unsigned char addi2c, unsigned char Reg, unsigned char data);
unsigned char Receive_Reg(unsigned char addi2c, unsigned char Reg,unsigned char addi2cr);
char Receive(unsigned char addi2c);

void setup_Accel(){
    //i2c.write_byte_data(AAdd, ACCEL_CTRL_REG1_A, 0x27) #initialise the Accelerometer
    //i2c.write_byte_data(AAdd, ACCEL_CTRL_REG4_A, 0x40)
    //ACCEL_CTRL_REG1_A = 0x20 # 00000111 rw
    //ACCEL_CTRL_REG4_A = 0x23 # 00000000 rw
    
    Transmit_Reg(accelADD, 0x20, 0x67);
    Transmit_Reg(accelADD, 0x23, 0x00);
    Transmit_Reg(accelADD, 0x24, 0x60);
    
    return;
    
}

void setup_Mag(){
    //i2c.write_byte_data(MAdd, MAG_MR_REG_M, 0x00) #initialise the Magnetometer
    //MAG_MR_REG_M = 0x02
    Transmit_Reg(magADD, 0x00, 0x1C);//220 hz
    Transmit_Reg(magADD, 0x01, 0x20); // 1.3 mag field 1100 yx and 980z gauss
    Transmit_Reg(magADD, 0x02, 0x00);
    return;
}

void setup_Gyro(){
    //i2c.write_byte_data(GAdd, L3DG20_CTRL1, 0x0F) #0b01011111
    //i2c.write_byte_data(GAdd, L3DG20_CTRL4, 0x00) #2000 dps multipy by .07 for degrees
    //L3DG20_CTRL1 = 0x20
    //L3DG20_CTRL4 = 0X23
    Transmit_Reg(gyroADD, 0x20, 0x0F);
    Transmit_Reg(gyroADD, 0x23, 0x00);
    //xl/h = 28/29
    //yl/h = 2a/2b
    //zl/h = 2c/2d
    return;
}

void setup_Baro(){
    //i2c.write_byte_data(BAdd,LPS331AP_CTRL_REG1,0x00)
    //i2c.write_byte_data(BAdd,RES_ADDR,0x7A)
    //i2c.write_byte_data(BAdd,LPS331AP_CTRL_REG1,0x84)
    //i2c.write_byte_data(BAdd,LPS331AP_CTRL_REG2,0x01)
    //LPS331AP_CTRL_REG1 = 0x20
    //RES_ADDR = 0x10
    //LPS331AP_CTRL_REG2 = 0x21
    //
    //Transmit_Reg(baroADD, 0x20, 0x80);
    Transmit_Reg(baroADD, 0x20, 0x80);
    Transmit_Reg(baroADD, 0x10, 0x33);//8 bits resolution on each pressure and temp
    Transmit_Reg(baroADD, 0x21, 0x81);
    //read 28h PRESS_OUT_XL
    
    return;
}

double pressuremb(){
   int pressxl, pressl, pressh;
   float press;
    pressxl = Receive_Reg(baroADD, 0x28, baroADDr);
    pressl = Receive_Reg(baroADD, 0x29, baroADDr);
    pressh = Receive_Reg(baroADD, 0x2A, baroADDr);
    //press = ((pressh << 8)+(pressl));
    press = pressxl;
    //press = Receive(baroADD);
    press = press/4096.0;
   
    return press;
}

int altitudem(){
    int p = pressuremb();
    float tempa = pow((p/1013.25),0.190284);
    int altm = ((1-(tempa))*145366.45)*0.3048;
    return altm;
}

int temperature(){
   int temp_byte, t1, t2;
   t1 = Receive_Reg(baroADD, 0x2B, baroADDr);
   t2 = Receive_Reg(baroADD, 0x2C, baroADDr);
    temp_byte = ((t2<<8)+(t1));
    int degree =42.5+ temp_byte/480;
    return degree;
}

float compass(){
    //Xmag L/H 08h 09h
    //Ymag 0Ah 0Bh
    //Zmag 0Ch 0Dh
    //16 bit 2's comp 
    /*case LSM303_MAGGAIN_1_3:
      _lsm303Mag_Gauss_LSB_XY = 1100;
      _lsm303Mag_Gauss_LSB_Z  = 980;
      break;
     * LSM303_MAGGAIN_1_3                        = 0x20,  // +/- 1.3
     event->magnetic.x = _magData.x / _lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
  event->magnetic.y = _magData.y / _lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
  event->magnetic.z = _magData.z / _lsm303Mag_Gauss_LSB_Z * SENSORS_GAUSS_TO_MICROTESLA;
     1 gauss = .0001 tesla, so .0000000001*/
   unsigned char xL, xH, yL, yH, zL, zH;
   short int x, y ,z;
    int temp_comp;
    float xm, ym, zm;
    
    xL = Receive_Reg(magADD, 0x04, magADDr);
    xH = Receive_Reg(magADD, 0x03, magADDr);
    x = ((xH << 8)+xL);
    // xm = x/1100*0.00001;
    xm = x/1100*0.000001;
    yL = Receive_Reg(magADD, 0x08, magADDr);
    yH = Receive_Reg(magADD, 0x07, magADDr);
    y = ((yH << 8)+yL);
    ym = y/1100*0.000001;
    zL = Receive_Reg(magADD, 0x06, magADDr);
    zH = Receive_Reg(magADD, 0x05, magADDr);
    z = ((zH << 8)+zL);
    zm = z/1100*0.000001;
    
    //temp_comp = pow(pow(x,2)+pow(y,2)+pow(z,2),0.5);
    
    //temp_comp = atan2(y, x);
    //printf("x %d y %d z %d \r\n",x,y,z);
    //if(temp_comp < 0){ temp_comp+=360};
    temp_comp = (180/3.14) * atan2(y,x);
    
    return temp_comp;
    
}

int accelx(){
    //Accel X L/H 28h 29h
    //Accel Y 2Ah 2Bh
    //Accel Z 2Ch 2Dh
    //16 bit 2's comp
    
    int axL, axH, ax;
    
    axL = Receive_Reg(accelADD, 0x28, accelADDr);
    axH = Receive_Reg(accelADD, 0x29, accelADDr);
    ax = (axH << 8)|axL;
    return ax;
}
int accely(){
    int ayL, ayH, ay;
    ayL = Receive_Reg(accelADD, 0x2A, accelADDr);
    ayH = Receive_Reg(accelADD, 0x2B, accelADDr);
    ay = (ayH << 8)|ayL;
    return ay;
}
int accelz(){
    int azL, azH, az;
    azL = Receive_Reg(accelADD, 0x2C, accelADDr);
    azH = Receive_Reg(accelADD, 0x2D, accelADDr);
    az = (azH << 8)|azL;
    return az;
}

int gyrox(){
     int gxL, gxH, gyL, gyH, gzL, gzH, gx, gy ,gz;
    
    gxL = Receive_Reg(gyroADD, 0x28, gyroADDr);
    gxH = Receive_Reg(gyroADD, 0x29, gyroADDr);
    gx = (gxH << 8)|gxL;
    gx = gx*0.07;
    
    return gx;
}
int gyroy(){
    int gyL, gyH, gy;
    gyL = Receive_Reg(gyroADD, 0x2A, gyroADDr);
    gyH = Receive_Reg(gyroADD, 0x2B, gyroADDr);
    gy = (gyH << 8)|gyL;
    gy = gy*0.07;
    
    return gy;
}
int gyroz(){
    int gzL, gzH, gz;
    gzL = Receive_Reg(gyroADD, 0x2C, gyroADDr);
    gzH = Receive_Reg(gyroADD, 0x2D, gyroADDr);
    gz = ((gzH << 8)|gzL);
    gz = gz*0.07;
    return gz;
}  

    
    /*
     accelerationX = (signed int)(((signed int)rawData_X) * 3.9);
accelerationY = (signed int)(((signed int)rawData_Y) * 3.9);
accelerationZ = (signed int)(((signed int)rawData_Z) * 3.9);

roll = 180 * atan (accelerationY/sqrt(accelerationX*accelerationX + accelerationZ*accelerationZ))/M_PI;*/



/*//Convert Gyro raw to degrees per second
rate_gyr_x = (float) gyrRaw[0] * G_GAIN;
rate_gyr_y = (float) gyrRaw[1]  * G_GAIN;
rate_gyr_z = (float) gyrRaw[2]  * G_GAIN;*/
/*
 //Calculate the angles from the gyro
gyroXangle+=rate_gyr_x*DT;
gyroYangle+=rate_gyr_y*DT;
gyroZangle+=rate_gyr_z*DT;*/
int Gpitch(){
    int gpitch;
    //pitch = 180 * atan (accelerationX/sqrt(accelerationY*accelerationY + accelerationZ*accelerationZ))/M_PI;
    gpitch = (180/3.14) * atan(gyrox()/pow(pow(gyroy(),2)+pow(gyroz(),2),.5));
    return gpitch;
}
int Groll(){
    int groll;
    groll = (180/3.14) * atan(gyroy()/pow(pow(gyrox(),2)+pow(gyroz(),2),.5));
    return groll;
    
}
int Gyaw(){
    int gyaw;
    gyaw = (180/3.14) * atan(gyroz()/pow(pow(gyrox(),2)+pow(gyroy(),2),.5));
    return gyaw;
}


/*AccXangle = (float) (atan2(accRaw[1],accRaw[2])+M_PI)*RAD_TO_DEG;
AccYangle = (float) (atan2(accRaw[2],accRaw[0])+M_PI)*RAD_TO_DEG;*/
int Apitch(){
    int apitch;
    //pitch = 180 * atan (accelerationX/sqrt(accelerationY*accelerationY + accelerationZ*accelerationZ))/M_PI;
    apitch = (180/3.14) * atan(accelx()/pow(pow(accely(),2)+pow(accelz(),2),.5));
    return apitch;
}
int Aroll(){
    int aroll;
    //pitch = 180 * atan (accelerationX/sqrt(accelerationY*accelerationY + accelerationZ*accelerationZ))/M_PI;
    aroll = (180/3.14) * atan(accely()/pow(pow(accelx(),2)+pow(accelz(),2),.5));
    return aroll;
}
int Ayaw(){
    int ayaw;
    //pitch = 180 * atan (accelerationX/sqrt(accelerationY*accelerationY + accelerationZ*accelerationZ))/M_PI;
    ayaw = (180/3.14) * atan(accelz()/pow(pow(accely(),2)+pow(accelx(),2),.5));
    return ayaw;
}