#include "bme280.h"
#include "twi.h"

BME280_coeffs_t BME280_calib;
BME280_data_t BME280_tph;

uint8_t BME280_read_id(unsigned char *id) {
    if (!TWI_start())
        return 0;

    if (!TWI_slave_write(BME280_I2C_ADDR))
        return 0;

    if (!TWI_write_byte(BME280_ID_ADDR))
        return 0;

    if (!TWI_rstart())
        return 0;

    if (!TWI_slave_read(BME280_I2C_ADDR))
        return 0;

    if (!TWI_read_byte(id))
        return 0;

    TWI_stop();

    return 1;
}

uint8_t BME280_reset(void) {
    if (!TWI_start())
        return 0;

    if (!TWI_slave_write(BME280_I2C_ADDR))
        return 0;

    if (!TWI_write_byte(BME280_RESET_ADDR))
        return 0;

    if (!TWI_write_byte(BME280_RESET_CMD))
        return 0;

    TWI_stop();

    return 1;
}

uint8_t BME280_read_calibration(void) {
    unsigned char tp_coeffs[BME280_TP_CALIB_LENGTH];
    unsigned char h1_coeff;
    unsigned char h_coeffs[BME280_HUM_REST_CALIB_LENGTH];

    // read T and P calibration coefficients
    if (!TWI_start())
        return 0;

    if (!TWI_slave_write(BME280_I2C_ADDR))
        return 0;

    if (!TWI_write_byte(BME280_TP_CALIB_START_ADDR))
        return 0;

    if (!TWI_rstart())
        return 0;

    if (!TWI_slave_read(BME280_I2C_ADDR))
        return 0;

    if (!TWI_read_nbytes(tp_coeffs, BME280_TP_CALIB_LENGTH))
        return 0;

    // read H1 calibration coefficient
    if (!TWI_rstart())
        return 0;

    if (!TWI_slave_write(BME280_I2C_ADDR))
        return 0;

    if (!TWI_write_byte(BME280_HUM_H1_CALIB_START_ADDR))
        return 0;

    if (!TWI_rstart())
        return 0;

    if (!TWI_slave_read(BME280_I2C_ADDR))
        return 0;

    if (!TWI_read_byte(&h1_coeff))
        return 0;

    // read H2-H6 calibration coefficients
    if (!TWI_rstart())
        return 0;

    if (!TWI_slave_write(BME280_I2C_ADDR))
        return 0;

    if (!TWI_write_byte(BME280_HUM_REST_CALIB_START_ADDR))
        return 0;

    if (!TWI_rstart())
        return 0;

    if (!TWI_slave_read(BME280_I2C_ADDR))
        return 0;

    if (!TWI_read_nbytes(h_coeffs, BME280_HUM_REST_CALIB_LENGTH))
        return 0;

    TWI_stop();

    BME280_calib.T1 = (((uint16_t)tp_coeffs[1]) << 8) | ((uint16_t)tp_coeffs[0]);
    BME280_calib.T2 = (((int16_t)tp_coeffs[3]) << 8) | ((int16_t)tp_coeffs[2]);
    BME280_calib.T3 = (((int16_t)tp_coeffs[5]) << 8) | ((int16_t)tp_coeffs[4]);

    BME280_calib.P1 = (((uint16_t)tp_coeffs[7]) << 8) | ((uint16_t)tp_coeffs[6]);
    BME280_calib.P2 = (((int16_t)tp_coeffs[9]) << 8) | ((int16_t)tp_coeffs[8]);
    BME280_calib.P3 = (((int16_t)tp_coeffs[11]) << 8) | ((int16_t)tp_coeffs[10]);
    BME280_calib.P4 = (((int16_t)tp_coeffs[13]) << 8) | ((int16_t)tp_coeffs[12]);
    BME280_calib.P5 = (((int16_t)tp_coeffs[15]) << 8) | ((int16_t)tp_coeffs[14]);
    BME280_calib.P6 = (((int16_t)tp_coeffs[17]) << 8) | ((int16_t)tp_coeffs[16]);
    BME280_calib.P7 = (((int16_t)tp_coeffs[19]) << 8) | ((int16_t)tp_coeffs[18]);
    BME280_calib.P8 = (((int16_t)tp_coeffs[21]) << 8) | ((int16_t)tp_coeffs[20]);
    BME280_calib.P9 = (((int16_t)tp_coeffs[23]) << 8) | ((int16_t)tp_coeffs[22]);

    BME280_calib.H1 = (unsigned char)h1_coeff;
    BME280_calib.H2 = (((int16_t)h_coeffs[1]) << 8) | ((int16_t)h_coeffs[0]);
    BME280_calib.H3 = (unsigned char)h_coeffs[2];
    BME280_calib.H4 = (((int16_t)h_coeffs[3]) << 4) | ((int16_t)(h_coeffs[4] & 0x0F));
    BME280_calib.H5 = (((int16_t)h_coeffs[5]) << 4) | ((int16_t)(h_coeffs[4] >> 4));
    BME280_calib.H6 = (char)h_coeffs[6];

    return 1;
}

uint8_t BME280_set_acquisition(const unsigned char os_t, const unsigned char os_p, const unsigned char os_h, const unsigned char mode) {
    // write setting for humidity oversampling first
    if (!TWI_start())
        return 0;

    if (!TWI_slave_write(BME280_I2C_ADDR))
        return 0;

    if (!TWI_write_byte(BME280_CTRL_HUM_ADDR))
        return 0;

    if (!TWI_write_byte(os_h))
        return 0;

    // then write the rest
    if (!TWI_rstart())
        return 0;

    if (!TWI_slave_write(BME280_I2C_ADDR))
        return 0;

    if (!TWI_write_byte(BME280_CTRL_MEAS_ADDR))
        return 0;

    if (!TWI_write_byte((os_t << 5) | (os_p << 2) | mode))
        return 0;

    TWI_stop();

    return 1;
}

uint8_t BME280_read_status(unsigned char *status) {
    unsigned char s;

    if (!TWI_start())
        return 0;

    if (!TWI_slave_write(BME280_I2C_ADDR))
        return 0;

    if (!TWI_write_byte(BME280_STATUS_ADDR))
        return 0;

    if (!TWI_rstart())
        return 0;

    if (!TWI_slave_read(BME280_I2C_ADDR))
        return 0;

    if (!TWI_read_byte(&s))
        return 0;

    TWI_stop();

    *status = s & 0x09;

    return 1;
}

uint8_t BME280_read_TPH(void) {
    unsigned char data[BME280_TPH_LENGTH];

    if (!TWI_start())
        return 0;

    if (!TWI_slave_write(BME280_I2C_ADDR))
        return 0;

    if (!TWI_write_byte(BME280_TPH_START_ADDR))
        return 0;

    if (!TWI_rstart())
        return 0;

    if (!TWI_slave_read(BME280_I2C_ADDR))
        return 0;

    if (!TWI_read_nbytes(data, BME280_TPH_LENGTH))
        return 0;

    TWI_stop();

    BME280_tph.t0 = data[3];
    BME280_tph.t1 = data[4];
    BME280_tph.t2 = data[5];

    BME280_tph.p0 = data[0];
    BME280_tph.p1 = data[1];
    BME280_tph.p2 = data[2];

    BME280_tph.h0 = data[6];
    BME280_tph.h1 = data[7];

    return 1;
}

void BME280_compensate(int16_t *final_T, uint32_t *final_P, uint32_t *final_H, unsigned char *alarm_T, unsigned char *alarm_P, unsigned char *alarm_H) {
    int32_t t_raw = (int32_t)((((int32_t)BME280_tph.t0) << 12) | (((int32_t)BME280_tph.t1) << 4) | (((int32_t)BME280_tph.t2) >> 4));
    int32_t var1, var2, t_fine, T;
    var1 = (((t_raw >> 3) - (((int32_t)BME280_calib.T1) << 1)) * ((int32_t)BME280_calib.T2)) >> 11;
    var2 = (((((t_raw >> 4) - ((int32_t)BME280_calib.T1)) * ((t_raw >> 4) - ((int32_t)BME280_calib.T1))) >> 12) * ((int32_t)BME280_calib.T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;

    if ((T <= BME280_TEMP_LOW) || (T >= BME280_TEMP_HIGH)) {
        *alarm_T = 1;

        if (T <= BME280_TEMP_LOW)
            *final_T = BME280_TEMP_LOW;
        else
            *final_T = BME280_TEMP_HIGH;
    }
    else {
        *alarm_T = 0;
        *final_T = T;
    }

    int32_t p_raw = (int32_t)((((uint32_t)BME280_tph.p0) << 12) | (((uint32_t)BME280_tph.p1) << 4) | (((uint32_t)BME280_tph.p2) >> 4));
    uint32_t P;
    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)BME280_calib.P6);
    var2 = var2 + ((var1 * ((int32_t)BME280_calib.P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)BME280_calib.P4) << 16);
    var1 = (((BME280_calib.P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)BME280_calib.P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)BME280_calib.P1)) >> 15);
    P = (((uint32_t)(((int32_t)1048576) - p_raw) - (var2 >> 12))) * 3125;

    if (var1 != 0) {
        if (P < 0x80000000)
            P = (P << 1) / ((uint32_t)var1);
        else
            P = (P / ((uint32_t)var1)) * 2;

        var1 = (((int32_t)BME280_calib.P9) * ((int32_t) (((P >> 3) * (P >> 3)) >> 13))) >> 12;
        var2 = (((int32_t)(P >> 2)) * ((int32_t)BME280_calib.P8)) >> 13;
        P = (uint32_t)((int32_t)P + ((var1 + var2 + BME280_calib.P7) >> 4));
    }
    else
        P = 0;

    if ((P <= BME280_PRESS_LOW) || (P >= BME280_PRESS_HIGH)) {
        *alarm_P = 1;

        if (P < BME280_PRESS_LOW)
            *final_P = BME280_PRESS_LOW;
        else
            *final_P = BME280_PRESS_HIGH;
    }
    else {
        *alarm_P = 0;
        *final_P = P;
    }

    int32_t h_raw = (int32_t)((((int32_t)BME280_tph.h0) << 8) | ((int32_t)BME280_tph.h1));
    int32_t var3, var4, var5;
    uint32_t H;
    var1 = (int32_t)t_fine - (int32_t)76800;
    var2 = h_raw << 14;
    var3 = ((int32_t)BME280_calib.H4) << 20;
    var4 = ((int32_t)BME280_calib.H5) * var1;
    var5 = ((var2 - var3 - var4 + (int32_t)16384) >> 15);
    var2 = (var1 * ((int32_t)BME280_calib.H6)) >> 10;
    var3 = (var1 * ((int32_t)BME280_calib.H3)) >> 11;
    var4 = ((var2 * (var3 + (int32_t)32768)) >> 10) + (int32_t)2097152;
    var2 = (var4 * ((int32_t)BME280_calib.H2) + 8192) >> 14;
    var3 = var5 * var2;
    var4 = ((var3 >> 15) * (var3 >> 15)) >> 7;
    var5 = var3 - ((var4 * ((int32_t)BME280_calib.H1)) >> 4);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    H = (uint32_t)(var5 >> 12);

    if ((H <= BME280_HUM_LOW) || (H >= BME280_HUM_HIGH)) {
        *alarm_H = 1;

        if (H <= BME280_HUM_LOW)
            *final_H = BME280_HUM_LOW;
        else
            *final_H = BME280_HUM_HIGH;
    }
    else {
        *alarm_H = 0;
        *final_H = H;
    }
}
