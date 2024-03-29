/*
bh1750 lib 0x01

copyright (c) Davide Gironi, 2013

Released under GPLv3.
Please refer to LICENSE file for licensing information.
*/


#include <stdio.h>
#include <util/delay.h>

#include "bh1750.h"
#include "twimaster.h"
#include "i2c_core.h"
#include "uart.h"

/*
 * init bh1750
 */
void bh1750_init() {
/*
	#if BH1750_I2CINIT == 1
	//init i2c
	i2c_init();
	_delay_us(10);
	#endif
*/
	//write config
	twi_start_wait(BH1750_ADDR |TWI_WRITE);
	twi_write(BH1750_MODE);
	twi_stop();

}

/*
 * read lux value
 */
unsigned int bh1750_getlux() {
	unsigned int ret = 0;

	twi_start_wait(BH1750_ADDR | TWI_READ);
	ret = twi_readAck();
	ret |= (twi_readNak()<<8);
	twi_stop();
	return ret;
}
