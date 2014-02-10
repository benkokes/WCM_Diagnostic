
#ifndef _EEPROM_H_
#define _EEPROM_H_ 1

#define EEPROM_SIZE 		131072
#define EEPROM_PAGE_SIZE	256

/*
#define SLAVESELECT PB2	//ss
#define DATAOUT PB3		//MOSI
#define DATAIN  PB4		//MISO 
#define SPICLOCK PB5	//sck
*/
#define EEPROM_CS_EN	(PORTD &= ~(1<<PORTD7))
#define EEPROM_CS_DIS	(PORTD |= (1<<PORTD7))

#define EEPROM_HOLD_EN	(PORTB &= ~(1<<PORTB0))
#define EEPROM_HOLD_DIS (PORTB |=(1<<PORTB0))

#define EEPROM_WP_EN	(PORTD &= ~(1<<PORTD6))
#define EEPROM_WP_DIS   (PORTD |= (1<<PORTD6))
  

//opcodes
#define WREN  6
#define WRDI  4
#define RDSR  5
#define WRSR  1
#define READ  3
#define WRITE 2

volatile uint32_t eeprom_start;
volatile uint32_t eeprom_avail;
volatile uint32_t eeprom_end;

uint32_t GetEEPROMStartPointer();
uint32_t GetEEPROMAvailPointer();
void UpdateEEPROMPointer(uint32_t);

void eeprom_init();
uint8_t read_eeprom(uint32_t);
uint8_t read_page_eeprom(uint8_t*, uint32_t, uint32_t);
uint8_t write_eeprom(uint32_t, uint8_t);
uint8_t write_page_eeprom(uint32_t, uint8_t*, uint32_t);
void write_enable_eeprom();
uint8_t eeprom_copy(uint8_t*, uint32_t, uint32_t);

void spi_init();
uint8_t spi_transfer(uint8_t data);

#endif //_EEPROM_H_
