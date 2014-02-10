#ifndef __bmp805_h_
#define __bmp180_h_

#include <stdint.h>
#include "twimaster.h"
#include "i2c_core.h"

#define BMP180_ADDR 0x77

#define BMP180_REG_AC1 0xAA
#define BMP180_REG_AC2 0xAC
#define BMP180_REG_AC3 0xAE
#define BMP180_REG_AC4 0xB0
#define BMP180_REG_AC5 0xB2
#define BMP180_REG_AC6 0xB4
#define BMP180_REG_B1 0xB6
#define BMP180_REG_B2 0xB8
#define BMP180_REG_MB 0xBA
#define BMP180_REG_MC 0xBC
#define BMP180_REG_MD 0xBE

#define BMP180_REG_RESULT 0xF6

#define BMP180_REG_CONTROL 0xF4
#define BMP180_START_TEMPERATURE 0x2E
#define BMP180_START_PRESSURE0 0x34
#define BMP180_START_PRESSURE1 0x74
#define BMP180_START_PRESSURE2 0x84
#define BMP180_START_PRESSURE3 0xF4

#define BMP180_OSS 3
#if BMP180_OSS == 0
#   define BMP180_START_PRESSURE BMP180_START_PRESSURE0
#elif BMP180_OSS == 1
#   define BMP180_START_PRESSURE BMP180_START_PRESSURE1
#elif BMP180_OSS == 2
#   define BMP180_START_PRESSURE BMP180_START_PRESSURE2
#elif BMP180_OSS == 3
#   define BMP180_START_PRESSURE BMP180_START_PRESSURE3
#endif

typedef enum {
    BMP180_SUCCESS = 0,
    BMP180_I2C_ERROR
} bmp180_status_t;

typedef struct {
    int16_t AC1;
    int16_t AC2;
    int16_t AC3;
    uint16_t AC4;
    uint16_t AC5;
    uint16_t AC6;
    int16_t B1;
    int16_t B2;
    int16_t MB;
    int16_t MC;
    int16_t MD;
} bmp180_eeprom_data_t;

typedef struct {
    /* sensor API */
    sensor_t sensor;

    /* I/O configuration */
    i2c_bus_t *bus;

    /* internal status */
    bmp180_eeprom_data_t calib;
    volatile uint8_t conversion_complete;

    /* most recent uncompensated readings */
    uint32_t upres;
    uint16_t utemp;
} bmp180_t;

#define BMP180(x) ((bmp180_t*)x)

bmp180_status_t bmp180_setup(bmp180_t *bmp, i2c_bus_t *bus);

void *bmp180_measure(sensor_t *sensor, void *msg);
void bmp180_sleep(sensor_t *sensor);
void bmp180_wake(sensor_t *sensor);

bmp180_eeprom_data_t *bmp180_get_calib(bmp180_t *bmp);

int bmp180_start_conversion(bmp180_t *bmp, uint8_t command);
void bmp180_await_conversion_complete(bmp180_t *bmp);

bmp180_status_t bmp180_read_reg16(bmp180_t *bmp, uint8_t reg, uint16_t *value);
bmp180_status_t bmp180_read_reg24(bmp180_t *bmp, uint8_t reg, uint32_t *value);

#define bmp180_read_reg16s(bmp, reg, value) \
    bmp180_read_reg16(bmp, reg, (uint16_t*)value)

void bmp180_compute_result(bmp180_t *bmp, int32_t *pressure, int16_t *temperature);

#endif // __bmp180_h_

