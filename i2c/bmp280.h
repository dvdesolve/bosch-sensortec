#ifndef _BMP280_H
#define _BMP280_H

#include <stdint.h>

#define BMP280_ID_ADDR          0xD0
#define BMP280_RESET_ADDR       0xE0
#define BMP280_CALIB_START_ADDR 0x88
#define BMP280_CTRL_MEAS_ADDR   0xF4
#define BMP280_STATUS_ADDR      0xF3
#define BMP280_TP_START_ADDR    0xF7
#define BMP280_PRESS_START_ADDR 0xF7
#define BMP280_TEMP_START_ADDR  0xFA

#define BMP280_CALIB_LENGTH 24
#define BMP280_TP_LENGTH    6
#define BMP280_PRESS_LENGTH 3
#define BMP280_TEMP_LENGTH  3

#define BMP280_ID           0x58
#define BMP280_I2C_ADDR     0x76
#define BMP280_RESET_CMD    0xB6

#define BMP280_TEMP_LOW     -4000
#define BMP280_TEMP_HIGH    8500
#define BMP280_PRESS_LOW    30000
#define BMP280_PRESS_HIGH   110000

typedef struct {
    uint16_t T1;
    int16_t T2;
    int16_t T3;

    uint16_t P1;
    int16_t P2;
    int16_t P3;
    int16_t P4;
    int16_t P5;
    int16_t P6;
    int16_t P7;
    int16_t P8;
    int16_t P9;
} BMP280_coeffs_t;

typedef struct {
    unsigned char t0;
    unsigned char t1;
    unsigned char t2;
    unsigned char p0;
    unsigned char p1;
    unsigned char p2;
} BMP280_data_t;

extern BMP280_coeffs_t BMP280_calib;
extern BMP280_data_t BMP280_tp;

extern uint8_t BMP280_read_id(unsigned char *id);
extern uint8_t BMP280_reset(void);
extern uint8_t BMP280_read_calibration(void);
extern uint8_t BMP280_set_acquisition(const unsigned char os_t, const unsigned char os_p, const unsigned char mode);
extern uint8_t BMP280_read_status(unsigned char *status);
extern uint8_t BMP280_read_TP(void);
extern void    BMP280_compensate(int16_t *final_T, uint32_t *final_P, unsigned char *alarm_T, unsigned char *alarm_P);

#endif
