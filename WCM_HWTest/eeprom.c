
/* ------------------------------------------------------------------------
**	 Module:	eeprom.c: 	
**	 Author:    Mike Hankey
**	 Hardware 	AVR ATmega328
**	 Software:	gcc 4.3.3 AVR Studio 4.18 Build 700
**
**	 DESCRIPTION: Handles eeprom communications
**						  
**    Version: 1.0
**
**    Copyright © 2010, Mike Hankey
**    All rights reserved. [BSD License]
**    http://www.JaxCoder.com/
** 
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
** 1. Redistributions of source code must retain the above copyright
**    notice, this list of conditions and the following disclaimer.
** 2. Redistributions in binary form must reproduce the above copyright
**    notice, this list of conditions and the following disclaimer in the
**    documentation and/or other materials provided with the distribution.
** 3. All advertising materials mentioning features or use of this software
**    must display the following acknowledgement:
**    This product includes software developed by the <organization>.
** 4. Neither the name of the <organization> nor the
**    names of its contributors may be used to endorse or promote products
**    derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY <COPYRIGHT HOLDER> ''AS IS'' AND ANY
** EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
** 
** ------------------------------------------------------------------------*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/common.h>
#include <util/delay.h>
#include <stdbool.h> //for TRUE FALSE
//#include "common.h"
#include "eeprom.h"

/* -- GetEEPROMStartPointer ----------------------------------------------
**
**	Description: Updates the avail pointer
**
**	Params:	None
**	Returns: word - start location in eeprom
** -----------------------------------------------------------------------*/
uint32_t GetEEPROMStartPointer()
{
	return eeprom_start;
}

/* -- GetEEPROMAvailPointer ----------------------------------------------
**
**	Description: Updates the avail pointer
**
**	Params:	None
**	Returns: word - Next available location in eeprom
** -----------------------------------------------------------------------*/
uint32_t GetEEPROMAvailPointer()
{
	return eeprom_avail;
}

/* -- UpdateEEPROMPointer ----------------------------------------------
**
**	Description: Updates the avail pointer
**
**	Params:	word - Offset
**	Returns: None
** -----------------------------------------------------------------------*/
void UpdateEEPROMPointer(uint32_t offset)
{
	eeprom_avail += offset;
}

/* -- eeprom_init --------------------------------------------------------
**
**	Description: Initialize/Reset eeprom and all pointers
**
**	Params:	None
**	Returns: None
** -----------------------------------------------------------------------*/
void eeprom_init()
{
	eeprom_start = 0;
	eeprom_avail = 0;
	eeprom_end = eeprom_start + EEPROM_SIZE;
	EEPROM_HOLD_DIS; //HOLD high (hold disabled)
	EEPROM_WP_DIS; //WP high (Write enabled)
}

/* -- read_eeprom -----------------------------------------------------------
**
**	Description: Read a byte from eeprom
**
**	Params:	word - address to read from
**	Returns: byte - data
** -----------------------------------------------------------------------*/
uint8_t read_eeprom(uint32_t address)
{
	int data;

	if (address >= EEPROM_SIZE)
		return false;//null;
	SPCR &= ~(1<<DORD);//MSB first
	EEPROM_CS_EN;//PORTB &= ~_BV(SLAVESELECT);
	//_delay_us(1);
	spi_transfer(READ); //transmit read opcode
	spi_transfer((char)(address>>16));  //send A24-A17 address byte first
	spi_transfer((char)(address>>8));   //send MSByte address byte next
	spi_transfer((char)(address));      //send LSByte address last
	data = spi_transfer(0xFF); //get data byte
	//_delay_us(1);
	EEPROM_CS_DIS;//PORTB = _BV(SLAVESELECT);

	return data;
}

/* -- read_page_eeprom --------------------------------------------------
**
**	Description: Read data from eeprom
**
**	Params:	byte* - pointer to data buffer
**			word - address to read from
**			word - length of data to read
**	Returns: bool - true on success false otherwise
** -----------------------------------------------------------------------*/
uint8_t read_page_eeprom(uint8_t* pdata, uint32_t address, uint32_t len)
{
	if (address >= EEPROM_SIZE || len > EEPROM_PAGE_SIZE)
		return false;
	
	EEPROM_CS_EN;//PORTB &= ~_BV(SLAVESELECT);
	SPCR &= ~(1<<DORD);//MSB first
	spi_transfer(READ); //transmit read opcode
	spi_transfer((char)(address>>16));  //send A24-A17 address byte first
	spi_transfer((char)(address>>8));   //send MSByte address byte next
	spi_transfer((char)(address));      //send LSByte address last

	for (int i = 0; i < len; i++)
		*pdata++ = spi_transfer(0xff);

	EEPROM_CS_DIS; //PORTB = _BV(SLAVESELECT);

	return true;
}

/* -- write_epprom -------------------------------------------------------
**
**	Description: Write data to EEPROM
**
**	Params:	byte* - pointer to data buffer
**			word - address to write data
**	Returns: bool - true on success false otherwise
** -----------------------------------------------------------------------*/
uint8_t write_eeprom(uint32_t address, uint8_t data)
{
	if (address >= EEPROM_SIZE)
		return false;

	write_enable_eeprom();

	EEPROM_CS_EN; //PORTB &= ~_BV(SLAVESELECT);
	SPCR &= ~(1<<DORD);//MSB first
	_delay_us(1);
	spi_transfer(WRITE); //write instruction
	spi_transfer((char)(address>>16));  //send A24-A17 address byte first
	spi_transfer((char)(address>>8));   //send MSByte address byte next
	spi_transfer((char)(address));      //send LSByte address last
  
    spi_transfer(data); //write data byte
	_delay_us(1);
	EEPROM_CS_DIS; //PORTB = _BV(SLAVESELECT);
_delay_ms(2);
	return true;
}

/* -- write_page_epprom --------------------------------------------------
**
**	Description: Write data to EEPROM
**
**	Params:	byte* - pointer to data buffer
**			word - address to write data
**			word - length of data
**	Returns: bool - true on success false otherwise
** -----------------------------------------------------------------------*/
uint8_t write_page_eeprom(uint32_t address, uint8_t* data, uint32_t len)
{
	if (address >= EEPROM_SIZE || len > EEPROM_PAGE_SIZE)
		return false;

	write_enable_eeprom();

	EEPROM_CS_EN; //PORTB &= ~_BV(SLAVESELECT);
	SPCR &= ~(1<<DORD);//MSB first
	spi_transfer(WRITE); //write instruction
	
	spi_transfer((char)(address>>16));  //send A24-A17 address byte first
	spi_transfer((char)(address>>8));   //send MSByte address byte next
  	spi_transfer((char)(address));      //send LSByte address last
  
  	for (int i = 0; i < len; i++)
	    spi_transfer(*(data + i)); //write data byte

	EEPROM_CS_DIS; //PORTB = _BV(SLAVESELECT);

	return true;
}

/* -- write_enable_eeprom ------------------------------------------------
**
**	Description: Enable writting to the EEPROM
**
**	Params:	None
**	Returns: None
** -----------------------------------------------------------------------*/
void write_enable_eeprom()
{
	SPCR &= ~(1<<DORD);//MSB first
	EEPROM_CS_EN; //PORTB &= ~_BV(SLAVESELECT);
	_delay_us(1);
	spi_transfer(WREN); //write enable
	_delay_us(1);
	EEPROM_CS_DIS; //PORTB = _BV(SLAVESELECT);
}

/* -- eeprom_copy ---------------------------------------------------------
**
**	Description: Copies len of eeprom to dest. memory
**
**	Params:	byte* - Destination address
**			word - eeprom source address
**			word - length of chunk to copy.
**	Returns: None
** -----------------------------------------------------------------------*/
uint8_t eeprom_copy(uint8_t* dest, uint32_t src, uint32_t len)
{
	if (src + len > EEPROM_SIZE || len > EEPROM_PAGE_SIZE)
		return false;

	read_page_eeprom(dest, src, len);

	return true;
}

/* -- spi_init -----------------------------------------------------------
**
**	Description: Initialize SPI interface
**
**	Params:	None
**	Returns: None
** -----------------------------------------------------------------------*/
/*void spi_init()
{
	DDRB = _BV(SLAVESELECT) | _BV(SPICLOCK) | _BV(DATAOUT);
	PORTB = _BV(SLAVESELECT);
	
	// SPCR = 01010000
	//interrupt disabled,spi enabled,msb 1st,master,clk low when idle,
	//sample on leading edge of clk,system clock/4 rate (fastest)
	SPCR = _BV(SPE) | _BV(MSTR);
}
*/
/* -- spi_transfer -----------------------------------------------------------
**
**	Description: Transfers one byte of data via SPI
**
**	Params:	uint8_t	byte to send
**	Returns: None
** -----------------------------------------------------------------------*/
/*
byte spi_transfer(byte data)
{
  SPDR = data;
  while (!(SPSR & (1<<SPIF)))     // Wait the end of the transmission
  {
  };
  return SPDR;                    // return the received byte
}
*/
