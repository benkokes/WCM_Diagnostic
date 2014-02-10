/*
 * KXCJ9.c
 *
 * Created: 1/21/2014 10:40:56 PM
 *  Author: bkokes
 */ 
//#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <avr/io.h>
#include "twimaster.h"
#include "i2c_core.h"


#include "KXCJ9.h"


void SensorInitKXCJ9_int(void)
{
	unsigned char tempBuf[4];
	uint8_t error;
	//PORTB|=(1<<PORTB2);
	tempBuf[0] = 0x00; //set PC1 to 0 to enable changes
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x1B, tempBuf, 1))!=0)
	error=1;

	tempBuf[0] = (0x40 | 0x00); //12bit mode,DRDY int, ORd with accel gain of 2G's
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x1B, tempBuf, 1))!=0)
	error=1;

	tempBuf[0] = 0x30; //Enable External int, Active High
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x1E, tempBuf, 1))!=0)
	error=1;

	tempBuf[0] = 0x03;		//Output Data Rate; 100Hz, LPF Roll-Off: 50Hz
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x21, tempBuf, 1))!=0)
	error=1;

	//enter operating mode
	tempBuf[0] = (0x80 | 0x40 | 0x20 | 0x00); //Enter Operating mode, 12bit mode,DRDY int, ORd with accel gain of 2G's
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x1B, tempBuf, 1))!=0)
	error=1;

}

void ReadKXCJ9(sensordata *sensor_struct)
{
	uint8_t sensorBuf[8];
	uint8_t error;
	//Grab CJ9 Status
	if((i2c_read(KXCJ9_I2C_SLAVE_ADDRESS, 0x1A, sensorBuf, 1)) != 0) // Read Status Register bit
	error=1;

	//Grab CJ9 data
	if((i2c_read(KXCJ9_I2C_SLAVE_ADDRESS, 0x06, sensorBuf, 6)) != 0) // fetch accel data
	error=1;

	sensor_struct->Xaxis12= (((int)sensorBuf[1]<<8) | (int)sensorBuf[0])>>4;	//assemble accel words
	sensor_struct->Yaxis12= (((int)sensorBuf[3]<<8) | (int)sensorBuf[2])>>4;
	sensor_struct->Zaxis12= (((int)sensorBuf[5]<<8) | (int)sensorBuf[4])>>4;
	
}