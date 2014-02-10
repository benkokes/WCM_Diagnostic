/*
 * sharplcd.c
 *
 * Created: 1/30/2014 4:28:53 PM
 *  Author: bkokes
 */ 
#include <avr/io.h>
#include <inttypes.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "sharplcd.h"
#include "spi.h"
#include "glcdfont.h"
#include "uart.h"

void drawline_str(uint8_t rowaddr, char *textbuf)
{
	
	char lcdlinebuf[17] ={0};
	unsigned int fontcurrentbyte=0, currentcharbyteoffset=0;
	unsigned int fontbyteindex=0, lcdbyteindex=0, fontcharindex=0, lcdbitindex=0,fontrownum=0;
	uint8_t i=0;
	
	SPCR &= ~(1<<DORD);//MSB first
	PORTC |= (1<<PORTC3); //LCD CS high.
	_delay_us(6);
	spi_transfer(0b10000000);
		while(fontrownum<8)
		{
			//printf("FontCharIndex:%3d, FontByteIndex:%3d, LCDBitIndex:%3d, LCDByteIndex:%3d FontRowNum:%3d \r\n", fontcharindex,fontbyteindex,lcdbitindex,lcdbyteindex,fontrownum);
			currentcharbyteoffset = textbuf[fontcharindex]*5;
			//currentchar = pgm_read_byte(fontarray + (textbuf*5)+ fontcharindex);//currentchar = fontarray[textbuf[fontcharindex]*5];
			fontcurrentbyte = pgm_read_byte(fontarray+currentcharbyteoffset+fontbyteindex);//fontcurrentbyte = fontarray[currentcharbyteoffset+fontbyteindex];
			
			//lcdlinebuf[lcdbyteindex] |= ((fontarray[currentcharbyteoffset+fontbyteindex] & (1<<fontrownum))<<lcdbitindex);
			
			if(fontbyteindex==5){//inserts space between characters
				lcdlinebuf[lcdbyteindex] |= (1<<lcdbitindex); //white pixel
				//uart_puts(" ");
				}else{
				if(((fontcurrentbyte & (1<<fontrownum))<<lcdbitindex)){ //if(((fontarray[currentcharbyteoffset+fontbyteindex] & (1<<fontrownum))<<lcdbitindex)){
					lcdlinebuf[lcdbyteindex] &= ~(1<<lcdbitindex); //black pixel
					//uart_puts("*");
					}else{
					lcdlinebuf[lcdbyteindex] |= (1<<lcdbitindex); //white pixel
					//uart_puts(" ");
				}
			}

			if(fontbyteindex<5){ //Font bit & byte indexer
				fontbyteindex++;
				}else{
				fontbyteindex=0; //done with current character.
				fontcharindex++; //move onto next char in the text buffer
			}
			
			if(lcdbitindex<7){ //LCD Bit & Byte indexer
					lcdbitindex++;
				}else{
					lcdbitindex=0;
					lcdbyteindex++;
				
					if(lcdbyteindex>15){
					SPCR |= (1<<DORD);//LSB First
					spi_transfer(rowaddr+fontrownum); //Initial row address(static) + row offset
					//SPCR &= ~(1<<DORD);//MSB first
					for(i=0; i<16; i++){
						spi_transfer(lcdlinebuf[i]); //LCD Data
						}
					spi_transfer(0x00);//pad after the command+address+data package
					fontrownum++;
					fontbyteindex=0;
					fontcharindex=0;
					lcdbitindex=0;
					lcdbyteindex=0;
					//uart_puts("\r\n");
				}
		}//end else{...
	
	}//End while(fontrownum...
	spi_transfer(0x00);
	_delay_us(3);
	PORTC &= ~(1<<PORTC3); //LCD CS low.
}
void clearlcd(void)
{
	SPCR &= ~(1<<DORD);//MSB first
	PORTC |= (1<<PORTC3); //LCD CS high.
	_delay_us(3);
	spi_transfer(0b10100000);
	spi_transfer(0x00);
	_delay_us(3);
	PORTC &= ~(1<<PORTC3); //LCD CS low.
	_delay_ms(5);
	
}