#include <util/twi.h>

#include "twi.h"

void TWI_setup(void) {
    /* final SCL frequency is:
     * F_SCL = F_CPU / (16 + 2 * 4^TWSR * TWBR)
     * (100 kHz in our case) */

    TWSR = 0;
    TWBR = 32; 
}

uint8_t TWI_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

    while (!(TWCR & (1 << TWINT)));

    return ((TWSR & 0xF8) == TW_START);
}

uint8_t TWI_rstart(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

    while (!(TWCR & (1 << TWINT)));

    return ((TWSR & 0xF8) == TW_REP_START);
}

uint8_t TWI_slave_write(const uint8_t addr) {
    TWDR = (addr << 1) | TWI_WRITE;
    TWCR = (1 << TWINT) | (1 << TWEN);

    while (!(TWCR & (1 << TWINT)));

    return ((TWSR & 0xF8) == TW_MT_SLA_ACK);
}

uint8_t TWI_slave_read(const uint8_t addr) {
    TWDR = (addr << 1) | TWI_READ;
    TWCR = (1 << TWINT) | (1 << TWEN);

    while (!(TWCR & (1 << TWINT)));

    return ((TWSR & 0xF8) == TW_MR_SLA_ACK);
}

uint8_t TWI_write_byte(const unsigned char byte) {
    TWDR = byte;
    TWCR = (1 << TWINT) | (1 << TWEN);

    while (!(TWCR & (1 << TWINT)));

    return ((TWSR & 0xF8) == TW_MT_DATA_ACK);
}

uint8_t TWI_read_byte(unsigned char *byte) {
    TWCR = (1 << TWINT) | (1 << TWEN);

    while (!(TWCR & (1 << TWINT)));

    if ((TWSR & 0xF8) != TW_MR_DATA_NACK)
        return 0;

    *byte = TWDR;
    return 1;
}

uint8_t TWI_read_nbytes(unsigned char *buf, const uint8_t n) {
    for (uint8_t i = 0; i < (n - 1); i++) {
        TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);

        while (!(TWCR & (1 << TWINT)));

        if ((TWSR & 0xF8) != TW_MR_DATA_ACK)
            return 0;

        *(buf + i) = TWDR;
    }

    TWCR = (1 << TWINT) | (1 << TWEN);

    while (!(TWCR & (1 << TWINT)));

    if ((TWSR & 0xF8) != TW_MR_DATA_NACK)
        return 0;

    *(buf + n - 1) = TWDR;

    return 1;
}

void TWI_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}
