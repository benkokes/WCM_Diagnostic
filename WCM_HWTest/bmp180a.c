#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "bmp180.h"
#include "twimaster.h"
#include "i2c_core.h"

/* sensor API **************************************************************/

const sensor_functions_t bmp180_fn = {
    .measure = bmp180_measure,
    .sleep = bmp180_sleep,
    .wake = bmp180_wake
};


/**
 * Initializes the BMP180 pressure sensor.  This makes sure that the i2c bus
 * is in a usable state, and reads the device-specific calibration 
 * information from the sensor and into local memory for later use.
 */
bmp180_status_t bmp180_setup(bmp180_t *bmp, i2c_bus_t *bus) {
    SENSOR(bmp)->fn = &bmp180_fn;
    SENSOR(bmp)->payload_size = 2 * sizeof(sensor_flags_t) +
        sizeof(int32_t) + sizeof(int16_t);
    bmp->bus = bus;
    bmp->conversion_complete = 0;

    /* read calibration data from sensor eeprom */
    bmp180_status_t ret = BMP180_SUCCESS;
    ret |= bmp180_read_reg16s(bmp, BMP180_REG_AC1, &bmp->calib.AC1);
    ret |= bmp180_read_reg16s(bmp, BMP180_REG_AC2, &bmp->calib.AC2);
    ret |= bmp180_read_reg16s(bmp, BMP180_REG_AC3, &bmp->calib.AC3);
    ret |= bmp180_read_reg16(bmp, BMP180_REG_AC4, &bmp->calib.AC4);
    ret |= bmp180_read_reg16(bmp, BMP180_REG_AC5, &bmp->calib.AC5);
    ret |= bmp180_read_reg16(bmp, BMP180_REG_AC6, &bmp->calib.AC6);
    ret |= bmp180_read_reg16s(bmp, BMP180_REG_B1, &bmp->calib.B1);
    ret |= bmp180_read_reg16s(bmp, BMP180_REG_B2, &bmp->calib.B2);
    ret |= bmp180_read_reg16s(bmp, BMP180_REG_MB, &bmp->calib.MB);
    ret |= bmp180_read_reg16s(bmp, BMP180_REG_MC, &bmp->calib.MC);
    ret |= bmp180_read_reg16s(bmp, BMP180_REG_MD, &bmp->calib.MD);

    if(ret != BMP180_SUCCESS) return BMP180_I2C_ERROR;

    /* read initial uncompensated pressure and temperature values */
    bmp180_start_conversion(bmp, BMP180_START_TEMPERATURE);
    bmp180_await_conversion_complete(bmp);
    ret = bmp180_read_reg16(bmp, BMP180_REG_RESULT, &bmp->utemp);
    if(ret != BMP180_SUCCESS) return ret;
    bmp180_start_conversion(bmp, BMP180_START_PRESSURE);
    bmp180_await_conversion_complete(bmp);
    ret = bmp180_read_reg24(bmp, BMP180_REG_RESULT, &bmp->upres);
    if(ret != BMP180_SUCCESS) return ret;
    bmp->upres = bmp->upres >> (8 - BMP180_OSS);

    return BMP180_SUCCESS;
}

void *bmp180_measure(sensor_t *sensor, void *msg) {
    bmp180_t *bmp = BMP180(sensor);
    
    /* read the pressure result */
    bmp180_read_reg24(bmp, BMP180_REG_RESULT, &bmp->upres);
    bmp->upres = bmp->upres >> (8 - BMP180_OSS);

    /* compute values */
    int16_t temperature;
    int32_t pressure;

    bmp180_compute_result(bmp, &pressure, &temperature);
    debug_json_sensor_dec2("bmp_pressure", pressure);
    debug_json_sensor_dec1("bmp_temperature", temperature);

    /* add results to message payload */
    msg = sensor_add_payload(msg, 
        SENSOR_TYPE_BMP_PRESSURE | SENSOR_SIZE_32BIT | SENSOR_SIGNED,
        &pressure);
    msg = sensor_add_payload(msg,
        SENSOR_TYPE_BMP_TEMPERATURE | SENSOR_SIZE_16BIT | SENSOR_SIGNED,
        &temperature);

    return msg;
}

void bmp180_sleep(sensor_t *sensor) {
    /* start temperature measurement when going to sleep */
    bmp180_start_conversion(BMP180(sensor), BMP180_START_TEMPERATURE);
}

void bmp180_wake(sensor_t *sensor) {
    /* on wake, read the temperature measurement completed during sleep, and
     * start a pressure measurement */
    bmp180_read_reg16(BMP180(sensor), BMP180_REG_RESULT,
        &BMP180(sensor)->utemp);
    bmp180_start_conversion(BMP180(sensor), BMP180_START_PRESSURE);
}

/**
 * Obtains a pointer to the calibration data in memory.
 * @return a pointer to the BMP180 device-specific calibration data
 */
bmp180_eeprom_data_t *bmp180_get_calib(bmp180_t *bmp) {
    return &bmp->calib;
}

/**
 * Starts a measurement on the pressure sensor.  This also arms the
 * interrupts to detect when the conversion is complete.
 * @param command the read command to send to the sensor, generally either
 *      #BMP180_START_TEMPERATURE or #BMP180_START_PRESSURE
 */
int bmp180_start_conversion(bmp180_t *bmp, uint8_t command) {
    bmp->conversion_complete = 0;

    int ret = i2c_start(bmp->bus, BMP180_ADDR, I2C_WRITE);
    if(ret != I2C_SUCCESS) return BMP180_I2C_ERROR;
    ret = i2c_write(bmp->bus, BMP180_REG_CONTROL, I2C_CONTINUE);
    if(ret != I2C_SUCCESS) return BMP180_I2C_ERROR;
    ret = i2c_write(bmp->bus, command, I2C_STOP);
    if(ret != I2C_SUCCESS) return BMP180_I2C_ERROR;

    return BMP180_SUCCESS;
}

/**
 * Puts the MCU to sleep until the BMP180 measurement is complete.  The
 * interrupt that fires when the measurement completes will wake the chip.
 */
void bmp180_await_conversion_complete(bmp180_t *bmp) {
    /* TODO: fixme */
}

/**
 * Reads a 16-bit register from the BMP180 sensor.
 * @param reg the register address to read
 * @return the 16-bit value that was read
 */
bmp180_status_t bmp180_read_reg16(bmp180_t *bmp, uint8_t reg, uint16_t *value) {
    uint8_t hi, lo;

    int ret = i2c_start(bmp->bus, BMP180_ADDR, I2C_WRITE);
    if(ret != I2C_SUCCESS) return BMP180_I2C_ERROR;
    ret = i2c_write(bmp->bus, reg, I2C_CONTINUE);
    if(ret != I2C_SUCCESS) return BMP180_I2C_ERROR;
    ret = i2c_start(bmp->bus, BMP180_ADDR, I2C_READ);
    if(ret != I2C_SUCCESS) return BMP180_I2C_ERROR;
    hi = i2c_read(bmp->bus, I2C_CONTINUE);
    lo = i2c_read(bmp->bus, I2C_STOP);

    *value = (hi << 8) | lo;
    return BMP180_SUCCESS;
}

/**
 * Reads a 24-bit register from the BMP180 sensor.
 * @param reg the register address to read
 * @return the 24-bit value that was read
 */
bmp180_status_t bmp180_read_reg24(bmp180_t *bmp, uint8_t reg, uint32_t *value) {
    uint8_t hi, lo, xlo;
    
    int ret = i2c_start(bmp->bus, BMP180_ADDR, I2C_WRITE);
    if(ret != I2C_SUCCESS) return BMP180_I2C_ERROR;
    ret = i2c_write(bmp->bus, reg, I2C_CONTINUE);
    if(ret != I2C_SUCCESS) return BMP180_I2C_ERROR;
    ret = i2c_start(bmp->bus, BMP180_ADDR, I2C_READ);
    if(ret != I2C_SUCCESS) return BMP180_I2C_ERROR;
    hi = i2c_read(bmp->bus, I2C_CONTINUE);
    lo = i2c_read(bmp->bus, I2C_CONTINUE);
    xlo = i2c_read(bmp->bus, I2C_STOP);

    *value = ((uint32_t)hi << 16) | ((uint32_t)lo <<8) | (uint32_t)xlo;
    return BMP180_SUCCESS;
}

void bmp180_compute_result(bmp180_t *bmp, int32_t *pressure, int16_t *temperature) {
    // temperature intermediates
    int32_t b5;

    // pressure intermediates
    int32_t b3, b6;
    uint32_t b4, b7;

    // temporaries
    int32_t x1, x2, x3;
    // calculate temperature
    x1 = (((int32_t)bmp->utemp - (int32_t)bmp->calib.AC6) * 
        (int32_t)bmp->calib.AC5) / 32768UL;
    x2 = ((int32_t)bmp->calib.MC * 2048L) / (x1 + (int32_t)bmp->calib.MD);
    b5 = x1 + x2;
    *temperature = (b5 + 8) / 16;
    //*temperature = x2;

    // calculate pressure
    b6 = b5 - 4000;
    x1 = ((int32_t)bmp->calib.B2 * (b6 * (b6 / 4096))) / 2048;
    x2 = ((int32_t)bmp->calib.AC2 * b6) / 2048;
    x3 = x1 + x2;
    b3 = ((((int32_t)bmp->calib.AC1 * 4 + x3) << BMP180_OSS) + 2) / 4;
    x1 = ((int32_t)bmp->calib.AC3 * b6) / 8192;
    x2 = ((int32_t)bmp->calib.B1 * ((b6 * b6) / 4096)) / 65536;
    x3 = ((x1 + x2) + 2) / 4;
    b4 = ((int32_t)bmp->calib.AC4 * (uint32_t)(x3 + 32768)) / 32768;
    b7 = ((uint32_t)bmp->upres - b3) * (50000 >> BMP180_OSS);
    if(b7 < 0x80000000) {
        *pressure = (b7 * 2) / b4;
    } else {
        *pressure = (b7 / b4) * 2;
    }
    x1 = (*pressure / 256) * (*pressure / 256);
    x1 = (x1 * 3038) / 65536;
    x2 = (-7357 * *pressure) / 65536;
    *pressure = *pressure + ((x1 + x2 + 3791) / 16);
}

