#include "bmp280.h"
#include "twi.h"

BMP280_coeffs_t BMP280_calib;
BMP280_data_t BMP280_tp;

uint8_t BMP280_read_id(unsigned char *id) {
    if (!TWI_start())
        return 0;

    if (!TWI_slave_write(BMP280_I2C_ADDR))
        return 0;

    if (!TWI_write_byte(BMP280_ID_ADDR))
        return 0;

    if (!TWI_rstart())
        return 0;

    if (!TWI_slave_read(BMP280_I2C_ADDR))
        return 0;

    if (!TWI_read_byte(id))
        return 0;

    TWI_stop();

    return 1;
}

uint8_t BMP280_reset(void) {
    if (!TWI_start())
        return 0;

    if (!TWI_slave_write(BMP280_I2C_ADDR))
        return 0;

    if (!TWI_write_byte(BMP280_RESET_ADDR))
        return 0;

    if (!TWI_write_byte(BMP280_RESET_CMD))
        return 0;

    TWI_stop();

    return 1;
}

uint8_t BMP280_read_calibration(void) {
    unsigned char coeffs[BMP280_CALIB_LENGTH];

    if (!TWI_start())
        return 0;

    if (!TWI_slave_write(BMP280_I2C_ADDR))
        return 0;

    if (!TWI_write_byte(BMP280_CALIB_START_ADDR))
        return 0;

    if (!TWI_rstart())
        return 0;

    if (!TWI_slave_read(BMP280_I2C_ADDR))
        return 0;

    if (!TWI_read_nbytes(coeffs, BMP280_CALIB_LENGTH))
        return 0;

    TWI_stop();

    BMP280_calib.T1 = (((uint16_t)coeffs[1]) << 8) | ((uint16_t)coeffs[0]);
    BMP280_calib.T2 = (((int16_t)coeffs[3]) << 8) | ((int16_t)coeffs[2]);
    BMP280_calib.T3 = (((int16_t)coeffs[5]) << 8) | ((int16_t)coeffs[4]);

    BMP280_calib.P1 = (((uint16_t)coeffs[7]) << 8) | ((uint16_t)coeffs[6]);
    BMP280_calib.P2 = (((int16_t)coeffs[9]) << 8) | ((int16_t)coeffs[8]);
    BMP280_calib.P3 = (((int16_t)coeffs[11]) << 8) | ((int16_t)coeffs[10]);
    BMP280_calib.P4 = (((int16_t)coeffs[13]) << 8) | ((int16_t)coeffs[12]);
    BMP280_calib.P5 = (((int16_t)coeffs[15]) << 8) | ((int16_t)coeffs[14]);
    BMP280_calib.P6 = (((int16_t)coeffs[17]) << 8) | ((int16_t)coeffs[16]);
    BMP280_calib.P7 = (((int16_t)coeffs[19]) << 8) | ((int16_t)coeffs[18]);
    BMP280_calib.P8 = (((int16_t)coeffs[21]) << 8) | ((int16_t)coeffs[20]);
    BMP280_calib.P9 = (((int16_t)coeffs[23]) << 8) | ((int16_t)coeffs[22]);

    return 1;
}

uint8_t BMP280_set_acquisition(const unsigned char os_t, const unsigned char os_p, const unsigned char mode) {
    if (!TWI_start())
        return 0;

    if (!TWI_slave_write(BMP280_I2C_ADDR))
        return 0;

    if (!TWI_write_byte(BMP280_CTRL_MEAS_ADDR))
        return 0;

    if (!TWI_write_byte((os_t << 5) | (os_p << 2) | mode))
        return 0;

    TWI_stop();

    return 1;
}

uint8_t BMP280_read_status(unsigned char *status) {
    unsigned char s;

    if (!TWI_start())
        return 0;

    if (!TWI_slave_write(BMP280_I2C_ADDR))
        return 0;

    if (!TWI_write_byte(BMP280_STATUS_ADDR))
        return 0;

    if (!TWI_rstart())
        return 0;

    if (!TWI_slave_read(BMP280_I2C_ADDR))
        return 0;

    if (!TWI_read_byte(&s))
        return 0;

    TWI_stop();

    *status = s & 0x09;

    return 1;
}

uint8_t BMP280_read_TP(void) {
    unsigned char data[BMP280_TP_LENGTH];

    if (!TWI_start())
        return 0;

    if (!TWI_slave_write(BMP280_I2C_ADDR))
        return 0;

    if (!TWI_write_byte(BMP280_TP_START_ADDR))
        return 0;

    if (!TWI_rstart())
        return 0;

    if (!TWI_slave_read(BMP280_I2C_ADDR))
        return 0;

    if (!TWI_read_nbytes(data, BMP280_TP_LENGTH))
        return 0;

    TWI_stop();

    BMP280_tp.t0 = data[3];
    BMP280_tp.t1 = data[4];
    BMP280_tp.t2 = data[5];

    BMP280_tp.p0 = data[0];
    BMP280_tp.p1 = data[1];
    BMP280_tp.p2 = data[2];

    return 1;
}

void BMP280_compensate(int16_t *final_T, uint32_t *final_P, unsigned char *alarm_T, unsigned char *alarm_P) {
    int32_t t_raw = (int32_t)((((int32_t)BMP280_tp.t0) << 12) | (((int32_t)BMP280_tp.t1) << 4) | (((int32_t)BMP280_tp.t2) >> 4));
    int32_t var1, var2, t_fine, T;
    var1 = (((t_raw >> 3) - (((int32_t)BMP280_calib.T1) << 1)) * ((int32_t)BMP280_calib.T2)) >> 11;
    var2 = (((((t_raw >> 4) - ((int32_t)BMP280_calib.T1)) * ((t_raw >> 4) - ((int32_t)BMP280_calib.T1))) >> 12) * ((int32_t)BMP280_calib.T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;

    if ((T < BMP280_TEMP_LOW) || (T > BMP280_TEMP_HIGH)) {
        *alarm_T = 1;

        if (T < BMP280_TEMP_LOW)
            *final_T = BMP280_TEMP_LOW;
        else
            *final_T = BMP280_TEMP_HIGH;
    }
    else {
        *alarm_T = 0;
        *final_T = T;
    }

    int32_t p_raw = (int32_t)((((uint32_t)BMP280_tp.p0) << 12) | (((uint32_t)BMP280_tp.p1) << 4) | (((uint32_t)BMP280_tp.p2) >> 4));
    uint32_t P;
    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)BMP280_calib.P6);
    var2 = var2 + ((var1 * ((int32_t)BMP280_calib.P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)BMP280_calib.P4) << 16);
    var1 = (((BMP280_calib.P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)BMP280_calib.P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)BMP280_calib.P1)) >> 15);
    P = (((uint32_t)(((int32_t)1048576) - p_raw) - (var2 >> 12))) * 3125;

    if (var1 != 0) {
        if (P < 0x80000000)
            P = (P << 1) / ((uint32_t)var1);
        else
            P = (P / ((uint32_t)var1)) * 2;

        var1 = (((int32_t)BMP280_calib.P9) * ((int32_t) (((P >> 3) * (P >> 3)) >> 13))) >> 12;
        var2 = (((int32_t)(P >> 2)) * ((int32_t)BMP280_calib.P8)) >> 13;
        P = (uint32_t)((int32_t)P + ((var1 + var2 + BMP280_calib.P7) >> 4));
    }
    else
        P = 0;

    if ((P < BMP280_PRESS_LOW) || (P > BMP280_PRESS_HIGH)) {
        *alarm_P = 1;

        if (P < BMP280_PRESS_LOW)
            *final_P = BMP280_PRESS_LOW;
        else
            *final_P = BMP280_PRESS_HIGH;
    }
    else {
        *alarm_P = 0;
        *final_P = P;
    }
}
