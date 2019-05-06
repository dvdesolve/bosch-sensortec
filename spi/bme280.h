#ifndef _BME280_H
#define _BME280_H

#ifdef _BME280_DEFINITIONS_FILE
#include "bme280_definitions.h"
#endif

#ifndef BME280_PORT
#define BME280_PORT PORTA
#endif

#ifndef BME280_SS
#define BME280_SS   PA0
#endif

#define BME280_ID_ADDR                   0xD0
#define BME280_RESET_ADDR                0xE0
#define BME280_STATUS_ADDR               0xF3
#define BME280_CTRL_HUM_ADDR             0xF2
#define BME280_CTRL_MEAS_ADDR            0xF4
#define BME280_CONFIG_ADDR               0xF5
#define BME280_TP_CALIB_START_ADDR       0x88
#define BME280_HUM_H1_CALIB_START_ADDR   0xA1
#define BME280_HUM_REST_CALIB_START_ADDR 0xE1
#define BME280_PRESS_START_ADDR          0xF7
#define BME280_TEMP_START_ADDR           0xFA
#define BME280_HUM_START_ADDR            0xFD
#define BME280_TPH_START_ADDR            0xF7

#define BME280_RESET_CMD 0xB6
#define BME280_DUMB_CMD  0xFF

#define BME280_TP_CALIB_LENGTH       24
#define BME280_H_CALIB_LENGTH        8
#define BME280_HUM_REST_CALIB_LENGTH 7
#define BME280_PRESS_LENGTH          3
#define BME280_TEMP_LENGTH           3
#define BME280_HUM_LENGTH            2
#define BME280_TPH_LENGTH            8

#define BME280_ID   0x60

typedef unsigned char   (*spi_rxtx)(unsigned char);

extern spi_rxtx         BME280_spi;

extern unsigned char    BME280_read_ID(void);
extern void             BME280_reset(void);
extern char             BME280_measuring(void);
extern void             BME280_set_acquisition(const unsigned char os_t, const unsigned char os_p, const unsigned char os_h, const unsigned char mode);
extern void             BME280_set_config(const unsigned char t_sb, const unsigned char filter, const unsigned char spi3);
extern void             BME280_read_calibration(unsigned char *tp_coeff, unsigned char *h_coeff);
extern void             BME280_read_pressure(unsigned char *data);
extern void             BME280_read_temperature(unsigned char *data);
extern void             BME280_read_humidity(unsigned char *data);
extern void             BME280_read_TPH(unsigned char *data);

#endif
