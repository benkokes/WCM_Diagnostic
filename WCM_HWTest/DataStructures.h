/*
 * DATASTRUCTURES.h
 *
 * Created: 1/21/2014 10:58:25 PM
 *  Author: bkokes
 */ 


#ifndef DATASTRUCTURES_H_
#define DATASTRUCTURES_H_

typedef struct{
	unsigned char second;   //enter the current time, date, month, and year
	unsigned char minute;
	unsigned char hour;
	unsigned char date;
	unsigned char month;
	unsigned int year;
}time;


typedef struct {//numbers attached to variables are bit designators
	int Xaxis12;
	int Yaxis12;
	int Zaxis12;
	int32_t Pressure12BMP;
	double temperature12BMP;
	int humidity12SHT;
	int temperature12SHT;
	unsigned int lightLevel16;
	int BattLevel10;
}sensordata;




#endif /* DATASTRUCTURES_H_ */