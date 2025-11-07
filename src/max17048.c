#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>

#define I2C_PORT       i2c0
#define I2C_SDA_PIN    4      // change if you wired differently
#define I2C_SCL_PIN    5
#define MAX1704X_ADDR  0x36 // Equal to 54 in base 10

// Page 7 (Registers)
// https://www.analog.com/media/en/technical-documentation/data-sheets/max17043-max17044.pdf
#define REG_VCELL_MSB   0x02
#define REG_SOC_MSB     0x04
#define REG_MODE_MSB    0x06
#define REG_VERSION_MSB 0x08
#define REG_CONFIG_MSB  0x0C
#define REG_COMMAND_MSB 0xFE

// TODO: FINISH THIS
#define QUICKSTART_VALUE 0x4000
#define POWERONRST_VALUE 0x0054

int i2c_read16(uint8_t reg_msb, uint16_t *out) {
    uint8_t reg = reg_msb; // copy so we dont overwrite argument
    if (i2c_write_blocking(I2C_PORT, MAX1704X_ADDR, &reg, 1, true) != 1) {
        return -1; // -1 == error 
    }
    uint8_t buf[2] = {0};
    int bytes_read = i2c_read_blocking(I2C_PORT, MAX1704X_ADDR, buf, 2, false);
    if (bytes_read != 2) {
        return -1;
    }
    *out = ((uint16_t)buf[0] << 8) | buf[1];
    return 0; // zero == good
}

int i2c_write16(uint8_t reg_msb, uint16_t val) {
    uint8_t to_write[3];
    to_write[0] = reg_msb;
    to_write[1] = (uint8_t)(val >> 8);   // MSB
    to_write[2] = (uint8_t)(val & 0xFF); // LSB
    int bytes_written = i2c_write_blocking(I2C_PORT, MAX1704X_ADDR, to_write, 3, false);
    return (bytes_written == 3) ? 0 : -1;
}

float read_voltage() {
    // TODO
}

float read_soc() {
    // TODO
    // Read state of charge
}



// no main()