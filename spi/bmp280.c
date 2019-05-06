#include <avr/io.h>
#include "bmp280.h"

spi_rxtx BMP280_spi;

unsigned char BMP280_read_ID(void) {
    BMP280_PORT &= ~(1 << BMP280_SS); // pick up SPI bus

    (*BMP280_spi)(BMP280_ID_ADDR); // read from [id]
    unsigned char id = (*BMP280_spi)(BMP280_DUMB_CMD); // push ID from SPI I/O buffer

    BMP280_PORT |= (1 << BMP280_SS); // release SPI bus

    return id;
}

void BMP280_reset(void) {
    BMP280_PORT &= ~(1 << BMP280_SS); // pick up SPI bus

    (*BMP280_spi)(BMP280_RESET_ADDR & ~0x80); // write to [reset]
    (*BMP280_spi)(BMP280_RESET_CMD); // do soft-reset

    BMP280_PORT |= (1 << BMP280_SS); // release SPI bus
}

char BMP280_measuring(void) {
    BMP280_PORT &= ~(1 << BMP280_SS); // pick up SPI bus

    (*BMP280_spi)(BMP280_STATUS_ADDR); // read from [status]
    char measuring = (*BMP280_spi)(BMP280_DUMB_CMD) & 0x08; // get only fourth bit

    BMP280_PORT |= (1 << BMP280_SS); // release SPI bus

    return measuring;
}

void BMP280_set_acquisition(const unsigned char os_t, const unsigned char os_p, const unsigned char mode) {
    BMP280_PORT &= ~(1 << BMP280_SS); // pick up SPI bus

    (*BMP280_spi)(BMP280_CTRL_MEAS_ADDR & ~0x80); // write to [ctrl_meas]
    (*BMP280_spi)((os_t << 5) | (os_p << 2) | mode); // write combined config

    BMP280_PORT |= (1 << BMP280_SS); // release SPI bus
}

void BMP280_set_config(const unsigned char t_sb, const unsigned char filter, const unsigned char spi3) {
    BMP280_PORT &= ~(1 << BMP280_SS); // pick up SPI bus

    (*BMP280_spi)(BMP280_CONFIG_ADDR & ~0x80); // write to [config]
    (*BMP280_spi)((t_sb << 5) | (filter << 2) | spi3); // write combined config

    BMP280_PORT |= (1 << BMP280_SS); // release SPI bus
}

void BMP280_read_calibration(unsigned char *coeff) {
    BMP280_PORT &= ~(1 << BMP280_SS); // pick up SPI bus

    (*BMP280_spi)(BMP280_CALIB_START_ADDR); // read from [calib00]..[calib23]

    for (uint8_t i = 0; i < BMP280_CALIB_LENGTH; i++)
        coeff[i] = (*BMP280_spi)(BMP280_DUMB_CMD);

    BMP280_PORT |= (1 << BMP280_SS); // release SPI bus
}

void BMP280_read_pressure(unsigned char *data) {
    BMP280_PORT &= ~(1 << BMP280_SS); // pick up SPI bus

    (*BMP280_spi)(BMP280_PRESS_START_ADDR); // read from [press_msb], [press_lsb], [press_xlsb]

    for (uint8_t i = 0; i < BMP280_PRESS_LENGTH; i++)
        data[i] = (*BMP280_spi)(BMP280_DUMB_CMD);

    BMP280_PORT |= (1 << BMP280_SS); // release SPI bus
}

void BMP280_read_temperature(unsigned char *data) {
    BMP280_PORT &= ~(1 << BMP280_SS); // pick up SPI bus

    (*BMP280_spi)(BMP280_TEMP_START_ADDR); // read from [temp_msb], [temp_lsb], [temp_xlsb]

    for (uint8_t i = 0; i < BMP280_TEMP_LENGTH; i++)
        data[i] = (*BMP280_spi)(BMP280_DUMB_CMD);

    BMP280_PORT |= (1 << BMP280_SS); // release SPI bus
}

void BMP280_read_TP(unsigned char *data) {
    BMP280_PORT &= ~(1 << BMP280_SS); // pick up SPI bus

    (*BMP280_spi)(BMP280_TP_START_ADDR); // read from [press_msb], [press_lsb], [press_xlsb], [temp_msb], [temp_lsb], [temp_xlsb]

    for (uint8_t i = 0; i < BMP280_TP_LENGTH; i++)
        data[i] = (*BMP280_spi)(BMP280_DUMB_CMD);

    BMP280_PORT |= (1 << BMP280_SS); // release SPI bus
}
