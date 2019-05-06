#ifndef _BMP280_H
#define _BMP280_H

#ifdef _BMP280_DEFINITIONS_FILE
#include "bmp280_definitions.h"
#endif

#ifndef BMP280_PORT
#define BMP280_PORT PORTA
#endif

#ifndef BMP280_SS
#define BMP280_SS   PA0
#endif

#define BMP280_ID_ADDR          0xD0
#define BMP280_RESET_ADDR       0xE0
#define BMP280_STATUS_ADDR      0xF3
#define BMP280_CTRL_MEAS_ADDR   0xF4
#define BMP280_CONFIG_ADDR      0xF5
#define BMP280_CALIB_START_ADDR 0x88
#define BMP280_PRESS_START_ADDR 0xF7
#define BMP280_TEMP_START_ADDR  0xFA
#define BMP280_TP_START_ADDR    0xF7

#define BMP280_RESET_CMD    0xB6
#define BMP280_DUMB_CMD     0xFF

#define BMP280_CALIB_LENGTH 24
#define BMP280_PRESS_LENGTH 3
#define BMP280_TEMP_LENGTH  3
#define BMP280_TP_LENGTH    6

#define BMP280_ID   0x58

typedef unsigned char   (*spi_rxtx)(unsigned char);

extern spi_rxtx         BMP280_spi;

extern unsigned char    BMP280_read_ID(void);
extern void             BMP280_reset(void);
extern char             BMP280_measuring(void);
extern void             BMP280_set_acquisition(const unsigned char os_t, const unsigned char os_p, const unsigned char mode);
extern void             BMP280_set_config(const unsigned char t_sb, const unsigned char filter, const unsigned char spi3);
extern void             BMP280_read_calibration(unsigned char *coeff);
extern void             BMP280_read_pressure(unsigned char *data);
extern void             BMP280_read_temperature(unsigned char *data);
extern void             BMP280_read_TP(unsigned char *data);

#endif
