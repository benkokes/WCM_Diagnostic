/*
Sensor I2C Addresses: 7-bit(8Bit)
KXCJ9: 0x0F(0x1E)
SHT21: 0x40(0x80)
BH1750:0x5C(0xB8)
BMP180:0x77(0xEE)

SPI Devices:
LCD Sharp LS013B7DH03
EEPROM Microchip 5AA1024 --131072 byte capacity
*/
#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#define UART_BAUD_RATE	38400

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <compat/ina90.h>
#include "twimaster.h"
#include "i2c_core.h"
#include "uart.h"
#include "spi.h"
#include "KXCJ9.h"
#include "bh1750.h"
#include "bmp180.h"
#include "sht21.h"
#include "sharplcd.h"
#include "eeprom.h"
#include "DataStructures.h"

/*Global Variables */
time t; //time structure used in SRAM
time starttime;

uint8_t sleepmode=0;

char not_leap(void)      //check for leap year
{
	if (!(t.year%100))
	return (char)(t.year%400);
	else
	return (char)(t.year%4);
}

	/***********************************
	*Analog-to-Digital Converter setup *
	***********************************/
void adc_init(void)
{	
ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); //128 divisor from main clock for ADC
ADCSRB |= (0<<ADTS2)|(0<<ADTS0)|(0<<ADTS0); //ADC free running mode (No trigger)
//DIDR0 = 0b00001001; // disable digital buffers for ADC lines 0 and 3
ADMUX |=(0<<REFS1)|(0<<REFS0);
ADMUX &= ~(1<<ADLAR);  //No Left adjust
ADCSRA |=(1<<ADEN); //Enable ADC	
}

int check_batt_level(void)
{

	unsigned int adc = 0;
	ADCSRA |=(1<<ADEN); //Enable ADC

	/* change the adc multiplexer channel */
	
	//ADMUX = 0b0000111; // Single Ended ADC7 read
	//ADMUX = 0b01011110;	 // AVCC as VREF and 1.1VDC bandgap read
	ADMUX = 0x4E; //AVCC reference to the 1.1v bandgap

	/* start a conversion */
	ADCSRA |= (1<<ADSC);
	
	/* wait until the conversion is finished */
	while (ADCSRA & (1<<ADSC));
	adc = ADCL;
	adc |= ADCH << 8;

	/*** Perform a second time to get correct values ***/

	/* start a conversion */
	ADCSRA |= (1<<ADSC);
	
	/* wait until the conversion is finished */
	while (ADCSRA & (1<<ADSC));
	adc = ADCL;
	adc |= ADCH << 8;

	ADCSRA &=~(1<<ADEN); //Disable ADC

	/* return 10 bit result */

	return adc;


}// End Check_batt...
void PortInit(void)
{
	
	/*************
	*Port B Setup*
	*************/
	DDRB |= (1<<DDB0)|(1<<DDB1)|(1<<DDB2)|(1<<DDB3)|(1<<DDB5); // SCK, MOSI are outputs. SS MUST be configured as Output.
	PORTB |= (1<<PORTB0)|(1<<PORTB1)|(1<<PORTB3)|(1<<PORTB4)|(1<<PORTB5); 
/*
oB0:EEPROM_Hold	oB1:LED1(eeprom side)
oB2:LED2(uCside)oB3:MOSI
iB4:MISO		oB5:SCK
oB6:OSC			oB7:OSC
*/

	/*************
	*Port C Setup*
	*************/
	DDRC |=(1<<DDC0)|(1<<DDC2)|(1<<DDC3);
	PORTC |=(1<<PORTC0)|(1<<PORTC3);
/*
oC0:LCDDispMode	iC1:SW2
oC2:GPIO1		oC3:LCD_CS
iC4:SDA			iC5:SCL
xC6:RESET		
*/

	/*************
	*Port D Setup*
	*************/
	DDRD  |= (1<<DDD1)|(1<<DDD3)|(1<<DDD4)|(1<<DDD6)|(1<<DDD7);
	PORTD |= (1<<PORTD1)|(1<<PORTD3)|(1<<PORTD4)|(1<<PORTD6)|(1<<PORTD7);
/*
iD0:UARTRX		oD1:UARTTX 
iD2:AccelInt    oD3:LCDDisp_EXT 
oD4:ALS_DVI		iD5:SW1
oD6:EEPROM_WP	oD7:EEPROM_CS
*/	
}

void RTCInit(void)
{
	//Disable timer2 interrupts
	TIMSK2  = 0;
	//Enable asynchronous mode
	ASSR  |= (1<<AS2);
	//set initial counter value
	TCNT2=0;
	//set prescaler 128
	TCCR2B |= (0<<CS21)|(1<<CS20);
	//wait for registers update
	while (ASSR & ((1<<TCN2UB)|(1<<TCR2BUB)));
	//clear interrupt flags
	TIFR2  |= (1<<TOV2);
	//enable TOV2 interrupt
	//TIMSK2  |= (1<<TOIE2);
}

void enablePWM_OC2B(void)
{
	TCCR2A |= (0<<COM2B1) | (1<<COM2B0);
}

void lcd_init(void)
{
	PORTC |= (1<<PORTC3); //LCD CS high.
	PORTC &= ~(1<<PORTC3); //LCD CS low.  deselect
	PORTC |= (1<<PORTC0); //LCD EXTMODE: 1=Hardware toggle
}

uint8_t eeprom_test(void)
{
	uint32_t i=0;
	uint8_t j=0;
	char eeprom_test[38];
	uint8_t testarray[255];
	uint8_t eeprom_read_var=0;
		
	for(i=0; i<254; i++, j++) //write numbers 0-9 to EEPROM
		{
			if(write_eeprom(i,j)){
				sprintf(eeprom_test, "EEGood wtpos:%2d\r\n", (int)i);
				uart_puts(eeprom_test);
			}else{
				sprintf(eeprom_test, "EEBad wtpos:%2d\r\n", (int)i);
				uart_puts(eeprom_test);
			}
			testarray[i]= (uint8_t)i;
		}
		
	for(i=0; i<254; i++)
		{
			eeprom_read_var=read_eeprom(i);
			sprintf(eeprom_test, "Pos:%2d, ReadVar:%2d, ArrayVar:%2d\r\n",(int)i, eeprom_read_var, testarray[i]);
			uart_puts(eeprom_test);
			
			if(testarray[i]-eeprom_read_var) //check for diff between read value and written val
				return 0; //test failed, probably not connected...
		}
		uart_puts("\r\n");	
	return 1; //Fortune smiles, EEPROM test passed!!
}

int main(void)
{
	char mainbuf[23];
	uint8_t i2cdeviceaddresses[5]={0,0,0,0,0};
	uint8_t i2cdevicesfound = 0; 
	uint8_t i=0, readerror=0;
	sensordata recent_measure;
	
	cli();
	
	//Power reduction register -- ensure SPI is on
	PRR &= (1<<PRSPI);
	//PowerSave State
	//SMCR|=(1<<SM1)|(1<<SM0); //Power-save
	
	//I/O Init
	PortInit();
	_delay_ms(10);
	
	//UART init
	uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU));

	//Initialize I2C Bus
	twi_init();
	
	//Initialize the Timer2
	RTCInit();
	enablePWM_OC2B(); //required to service VCOM of LCD, but convenient to init here.
	
	//SPI Initialization
	init_spi();
	
	//Analog to digital converter - for battey measurement
	adc_init();
	
	//Enable global interrupts
	sei();
	//LCD Init
	lcd_init();
	_delay_ms(10);
		
	//Fresh slate!
	clearlcd();	
	
	uart_puts("Startup!\r\n");
	
	//Check the Bus for.... stuff
	i2cdevicesfound = scani2c(i2cdeviceaddresses);
	
	sprintf(mainbuf, "Devices found: %3d    ", i2cdevicesfound);
	drawline_str(10,mainbuf);
	
	drawline_str(20,"I2C Addresses         ");
	
	sprintf(mainbuf, "0x%02x 0x%02x 0x%02x 0x%02x", i2cdeviceaddresses[0],i2cdeviceaddresses[1],i2cdeviceaddresses[2],i2cdeviceaddresses[3]);
	drawline_str(30,mainbuf);
					
	//Accelerometer Init
	SensorInitKXCJ9_int();
	uart_puts("Accelerometer Sensor Int Complete\r\n");
	
	//Pressure Sensor Init
	bmp180_init();
	uart_puts("Pressure Sensor Int Complete\r\n");
	
	//Humidity Sensor Init
	SHT2x_SoftReset();
	uart_puts("Humidity Sensor Int Complete\r\n");
	
	//Light Sensor Init
	bh1750_init();
	uart_puts("Light Sensor Int Complete\r\n");
	
	//EEPROM init
	drawline_str(50, "Testing EEPROM...     ");
	eeprom_init();
	
	//Test EEPROM Connection/comms
	if(eeprom_test()){
		drawline_str(60, "EEPROM Passed! :-)    ");
	}else{
		
		drawline_str(60, "EEPROM Failed! >:-(   ");
	}

	for(i=7;i>0; i--)
	{
		sprintf(mainbuf, "Closing in %2d seconds",i);
		drawline_str(80,mainbuf);
		_delay_ms(1000);
	}

	//Enable RTC
	TIMSK2  |= (1<<TOIE2);
		
	uart_puts("End Inits\r\n");
		
	drawline_str(1,  "Primo Victoria!      ");
	drawline_str(10, "Elapsed Time         ");
	drawline_str(30, "Accel Measure(X,Y,Z) ");
    drawline_str(50, "Press/Temp(Pa,C)   ");
	drawline_str(70, "Humid/Temp(RH,C)     ");
	drawline_str(90, "Light/Batt(lux,V)    ");
	drawline_str(110, "Switch State/LEDstate");

	while(1)
    {
		
	ReadKXCJ9(&recent_measure);
	recent_measure.Pressure12BMP = bmp180_getpressure();
	recent_measure.temperature12BMP = bmp180_gettemperature();
	SHT2x_MeasureHM(HUMIDITY,&recent_measure);
	SHT2x_MeasureHM(TEMP,&recent_measure);
	recent_measure.lightLevel16 = bh1750_getlux();
	recent_measure.BattLevel10 = check_batt_level();
	
	recent_measure.humidity12SHT= (int)SHT2x_CalcRH(recent_measure.humidity12SHT);
	recent_measure.temperature12SHT = (int)SHT2x_CalcTemperatureC(recent_measure.temperature12SHT);
	/*sprintf(mainbuf, "$,%3d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d\r\n",maincounter++, 
	recent_measure.Xaxis12, recent_measure.Yaxis12,	recent_measure.Zaxis12,\
	recent_measure.Pressure12bit,recent_measure.temperature12BMP, \
	recent_measure.humidity12, recent_measure.temperature12SHT,\
	recent_measure.lightLevel16,recent_measure.BattLevel10);
	
	uart_puts(mainbuf);
	*/
	PORTB^=(1<<PORTB1);
	PORTB^=(1<<PORTB2);
	_delay_ms(500);
	sprintf(mainbuf, "%4d,%2d,%2d,%2d:%2d:%2d ",t.year,t.month,t.date,t.hour,t.minute,t.second);
	drawline_str(20, mainbuf);
	sprintf(mainbuf, "%5d,%5d,%5d     ",recent_measure.Xaxis12, recent_measure.Yaxis12,	recent_measure.Zaxis12);
	drawline_str(40, mainbuf);
	sprintf(mainbuf, "%10u,%10d          ",(uint16_t)recent_measure.Pressure12BMP,(int)recent_measure.temperature12BMP*10);
	drawline_str(60, mainbuf);
	sprintf(mainbuf, "%10d,%10d          ",recent_measure.humidity12SHT, recent_measure.temperature12SHT);
	drawline_str(80, mainbuf);
	sprintf(mainbuf, "%10u,%10d          ",recent_measure.lightLevel16,recent_measure.BattLevel10);
	drawline_str(100, mainbuf);
	sprintf(mainbuf, "%4u,%4u,%4u,%4u   ",((PIND & 0x20)>>5),((PINC & 0x02)>>1), ((PORTB & 0x02)>>1),((PORTB & 0x04)>>2));
	drawline_str(120, mainbuf);
	_delay_ms(500);
/*
			SMCR|=(1<<SE);
			asm volatile("sleep"::);
			SMCR &= ~(1<<SE);
*/
	}
}

//Overflow ISR
ISR(TIMER2_OVF_vect)
{
	if (++t.second==60)        //keep track of time, date, month, and year
	{
		t.second=0;
		if (++t.minute==60)
		{
			t.minute=0;
			if (++t.hour==24)
			{
				t.hour=0;
				if (++t.date==32)
				{
					t.month++;
					t.date=1;
				}
				else if (t.date==31)
				{
					if ((t.month==4) || (t.month==6) || (t.month==9) || (t.month==11))
					{
						t.month++;
						t.date=1;
					}
				}
				else if (t.date==30)
				{
					if(t.month==2)
					{
						t.month++;
						t.date=1;
					}
				}
				else if (t.date==29)
				{
					if((t.month==2) && (not_leap()))
					{
						t.month++;
						t.date=1;
					}
				}
				if (t.month==13)
				{
					t.month=1;
					t.year++;
				}
			}
		}
	}

	//asm volatile("nop"::);	
}