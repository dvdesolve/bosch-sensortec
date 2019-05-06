#ifndef _BME280_H
#define _BME280_H

#include <stdint.h>

#define BME280_ID_ADDR                      0xD0
#define BME280_RESET_ADDR                   0xE0
#define BME280_TP_CALIB_START_ADDR          0x88
#define BME280_HUM_H1_CALIB_START_ADDR      0xA1
#define BME280_HUM_REST_CALIB_START_ADDR    0xE1
#define BME280_CTRL_HUM_ADDR                0xF2
#define BME280_CTRL_MEAS_ADDR               0xF4
#define BME280_STATUS_ADDR                  0xF3
#define BME280_TPH_START_ADDR               0xF7
#define BME280_PRESS_START_ADDR             0xF7
#define BME280_TEMP_START_ADDR              0xFA
#define BME280_HUM_START_ADDR               0xFD

#define BME280_TP_CALIB_LENGTH          24
#define BME280_H_CALIB_LENGTH           8
#define BME280_HUM_REST_CALIB_LENGTH    7
#define BME280_TPH_LENGTH               8
#define BME280_PRESS_LENGTH             3
#define BME280_TEMP_LENGTH              3
#define BME280_HUM_LENGTH               2

#define BME280_ID           0x60
#define BME280_I2C_ADDR     0x76
#define BME280_RESET_CMD    0xB6

#define BME280_TEMP_LOW     -4000
#define BME280_TEMP_HIGH    8500
#define BME280_PRESS_LOW    30000
#define BME280_PRESS_HIGH   110000
#define BME280_HUM_LOW      0
#define BME280_HUM_HIGH     102400

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

    unsigned char H1;
    int16_t H2;
    unsigned char H3;
    int16_t H4;
    int16_t H5;
    char H6;
} BME280_coeffs_t;

typedef struct {
    unsigned char t0;
    unsigned char t1;
    unsigned char t2;
    unsigned char p0;
    unsigned char p1;
    unsigned char p2;
    unsigned char h0;
    unsigned char h1;
} BME280_data_t;

extern BME280_coeffs_t BME280_calib;
extern BME280_data_t BME280_tph;

extern uint8_t BME280_read_id(unsigned char *id);
extern uint8_t BME280_reset(void);
extern uint8_t BME280_read_calibration(void);
extern uint8_t BME280_set_acquisition(const unsigned char os_t, const unsigned char os_p, const unsigned char os_h, const unsigned char mode);
extern uint8_t BME280_read_status(unsigned char *status);
extern uint8_t BME280_read_TPH(void);
extern void    BME280_compensate(int16_t *final_T, uint32_t *final_P, uint32_t *final_H, unsigned char *alarm_T, unsigned char *alarm_P, unsigned char *alarm_H);

#endif
