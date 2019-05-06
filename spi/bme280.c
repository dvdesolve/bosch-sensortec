#include <avr/io.h>
#include "bme280.h"

spi_rxtx BME280_spi;

unsigned char BME280_read_ID(void) {
    BME280_PORT &= ~(1 << BME280_SS); // pick up SPI bus

    (*BME280_spi)(BME280_ID_ADDR); // read from [id]
    unsigned char id = (*BME280_spi)(BME280_DUMB_CMD); // push ID from SPI I/O buffer

    BME280_PORT |= (1 << BME280_SS); // release SPI bus

    return id;
}

void BME280_reset(void) {
    BME280_PORT &= ~(1 << BME280_SS); // pick up SPI bus

    (*BME280_spi)(BME280_RESET_ADDR & ~0x80); // write to [reset]
    (*BME280_spi)(BME280_RESET_CMD); // do soft-reset

    BME280_PORT |= (1 << BME280_SS); // release SPI bus
}

char BME280_measuring(void) {
    BME280_PORT &= ~(1 << BME280_SS); // pick up SPI bus

    (*BME280_spi)(BME280_STATUS_ADDR); // read from [status]
    char measuring = (*BME280_spi)(BME280_DUMB_CMD) & 0x08; // get only fourth bit

    BME280_PORT |= (1 << BME280_SS); // release SPI bus

    return measuring;
}

void BME280_set_acquisition(const unsigned char os_t, const unsigned char os_p, const unsigned char os_h, const unsigned char mode) {
    BME280_PORT &= ~(1 << BME280_SS); // pick up SPI bus

    (*BME280_spi)(BME280_CTRL_HUM_ADDR & ~0x80); // write to [ctrl_hum] first
    (*BME280_spi)(os_h); // write config

    (*BME280_spi)(BME280_CTRL_MEAS_ADDR & ~0x80); // write to [ctrl_meas]
    (*BME280_spi)((os_t << 5) | (os_p << 2) | mode); // write combined config

    BME280_PORT |= (1 << BME280_SS); // release SPI bus
}

void BME280_set_config(const unsigned char t_sb, const unsigned char filter, const unsigned char spi3) {
    BME280_PORT &= ~(1 << BME280_SS); // pick up SPI bus

    (*BME280_spi)(BME280_CONFIG_ADDR & ~0x80); // write to [config]
    (*BME280_spi)((t_sb << 5) | (filter << 2) | spi3); // write combined config

    BME280_PORT |= (1 << BME280_SS); // release SPI bus
}

void BME280_read_calibration(unsigned char *tp_coeff, unsigned char *h_coeff) {
    BME280_PORT &= ~(1 << BME280_SS); // pick up SPI bus

    (*BME280_spi)(BME280_TP_CALIB_START_ADDR); // read from [calib00]..[calib23]

    for (uint8_t i = 0; i < BME280_TP_CALIB_LENGTH; i++)
        tp_coeff[i] = (*BME280_spi)(BME280_DUMB_CMD);

    BME280_PORT |= (1 << BME280_SS); // release SPI bus

    BME280_PORT &= ~(1 << BME280_SS); // pick up SPI bus

    (*BME280_spi)(BME280_HUM_H1_CALIB_START_ADDR); // read [calib24]
    h_coeff[0] = (*BME280_spi)(BME280_DUMB_CMD);

    BME280_PORT |= (1 << BME280_SS); // release SPI bus

    BME280_PORT &= ~(1 << BME280_SS); // pick up SPI bus

    (*BME280_spi)(BME280_HUM_REST_CALIB_START_ADDR); // read from [calib25]..[calib32]

    for (uint8_t i = 0; i < BME280_HUM_REST_CALIB_LENGTH; i++)
        h_coeff[i + 1] = (*BME280_spi)(BME280_DUMB_CMD);

    BME280_PORT |= (1 << BME280_SS); // release SPI bus
}

void BME280_read_pressure(unsigned char *data) {
    BME280_PORT &= ~(1 << BME280_SS); // pick up SPI bus

    (*BME280_spi)(BME280_PRESS_START_ADDR); // read from [press_msb], [press_lsb], [press_xlsb]

    for (uint8_t i = 0; i < BME280_PRESS_LENGTH; i++)
        data[i] = (*BME280_spi)(BME280_DUMB_CMD);

    BME280_PORT |= (1 << BME280_SS); // release SPI bus
}

void BME280_read_temperature(unsigned char *data) {
    BME280_PORT &= ~(1 << BME280_SS); // pick up SPI bus

    (*BME280_spi)(BME280_TEMP_START_ADDR); // read from [temp_msb], [temp_lsb], [temp_xlsb]

    for (uint8_t i = 0; i < BME280_TEMP_LENGTH; i++)
        data[i] = (*BME280_spi)(BME280_DUMB_CMD);

    BME280_PORT |= (1 << BME280_SS); // release SPI bus
}

void BME280_read_humidity(unsigned char *data) {
    BME280_PORT &= ~(1 << BME280_SS); // pick up SPI bus

    (*BME280_spi)(BME280_HUM_START_ADDR); // read from [hum_msb], [hum_lsb]

    for (uint8_t i = 0; i < BME280_HUM_LENGTH; i++)
        data[i] = (*BME280_spi)(BME280_DUMB_CMD);

    BME280_PORT |= (1 << BME280_SS); // release SPI bus
}

void BME280_read_TPH(unsigned char *data) {
    BME280_PORT &= ~(1 << BME280_SS); // pick up SPI bus

    (*BME280_spi)(BME280_TPH_START_ADDR); // read from [press_msb], [press_lsb], [press_xlsb], [temp_msb], [temp_lsb], [temp_xlsb], [hum_msb], [hum_lsb]

    for (uint8_t i = 0; i < BME280_TPH_LENGTH; i++)
        data[i] = (*BME280_spi)(BME280_DUMB_CMD);

    BME280_PORT |= (1 << BME280_SS); // release SPI bus
}
