#ifndef _TWI_H
#define _TWI_H

#include <stdint.h>

#define TWI_WRITE   0
#define TWI_READ    1

extern void TWI_setup(void);
extern uint8_t TWI_start(void);
extern uint8_t TWI_rstart(void);
extern uint8_t TWI_slave_write(const uint8_t addr);
extern uint8_t TWI_slave_read(const uint8_t addr);
extern uint8_t TWI_write_byte(const unsigned char byte);
extern uint8_t TWI_read_byte(unsigned char *byte);
extern uint8_t TWI_read_nbytes(unsigned char *buf, const uint8_t n);
extern void TWI_stop(void);

#endif
