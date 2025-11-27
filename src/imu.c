#include "imu.h"

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <math.h>
#include <string.h>

// ==============================
//  Hardware / wiring config
// ==============================
//
// This code assumes the IMU is connected via SPI.
//
// The pin assignments below are just placeholders.
//
// Typical SPI0 mapping on the Pico (just an example):
//   SCK  = GPIO18
//   MOSI = GPIO19
//   MISO = GPIO16
//   CS   = GPIO17
//

#define IMU_SPI_PORT   spi0
#define IMU_PIN_SCK    18
#define IMU_PIN_MOSI   19
#define IMU_PIN_MISO   16
#define IMU_PIN_CS     17

// LSM6DS3TR-C SPI mode:
#define IMU_SPI_BAUD   (10 * 1000 * 1000)  // 10 MHz

// ==============================
//  LSM6DS3TR-C register map
// ==============================

#define LSM6DS3_REG_FUNC_CFG_ACCESS  0x01
#define LSM6DS3_REG_WHO_AM_I         0x0F
#define LSM6DS3_REG_CTRL1_XL         0x10
#define LSM6DS3_REG_CTRL2_G          0x11
#define LSM6DS3_REG_CTRL3_C          0x12
#define LSM6DS3_REG_CTRL8_XL         0x17
#define LSM6DS3_REG_STATUS_REG       0x1E

#define LSM6DS3_REG_OUTX_L_G         0x22
#define LSM6DS3_REG_OUTX_H_G         0x23
#define LSM6DS3_REG_OUTY_L_G         0x24
#define LSM6DS3_REG_OUTY_H_G         0x25
#define LSM6DS3_REG_OUTZ_L_G         0x26
#define LSM6DS3_REG_OUTZ_H_G         0x27

#define LSM6DS3_REG_OUTX_L_XL        0x28
#define LSM6DS3_REG_OUTX_H_XL        0x29
#define LSM6DS3_REG_OUTY_L_XL        0x2A
#define LSM6DS3_REG_OUTY_H_XL        0x2B
#define LSM6DS3_REG_OUTZ_L_XL        0x2C
#define LSM6DS3_REG_OUTZ_H_XL        0x2D

#define LSM6DS3_WHO_AM_I_VALUE       0x69

// ==============================
//  Step detection / history config
// ==============================
//
// Assumption: imu_update() is called at a fairly fixed rate
//             (something like 50–100 Hz works fine).
// The numbers below are just first guesses; they really should be tuned
// with actual motion data from your setup.

#define IMU_ACCEL_LSB_2G             (0.000061f)  // ≈0.061 mg/LSB → 0.000061 g/LSB
#define IMU_STEP_THRESHOLD_G         0.35f        // high-pass magnitude threshold in g
#define IMU_STEP_MIN_INTERVAL_MS     350          // ignore steps closer than this in time
#define IMU_HISTORY_MINUTES          60
#define IMU_STEP_GOAL_PER_HOUR       250

// Simple low-pass filter tracking the 1g baseline (used to get a high-pass signal)
#define IMU_MAG_LP_ALPHA             0.01f        // 0 < alpha <= 1

// ==============================
//  Internal state
// ==============================

static bool     s_initialized          = false;

// Last raw accelerometer reading (LSB)
static int16_t  s_raw_ax               = 0;
static int16_t  s_raw_ay               = 0;
static int16_t  s_raw_az               = 0;

// Last converted accel sample in g (no extra filtering for now)
static float    s_filt_ax              = 0.0f;
static float    s_filt_ay              = 0.0f;
static float    s_filt_az              = 0.0f;

// Magnitude baseline / high-pass
static bool     s_mag_lp_initialized   = false;
static float    s_mag_lp               = 0.0f;   // low-pass of |a|
static float    s_mag_hp               = 0.0f;   // high-pass: |a| - low-pass

// Step counters
static uint32_t s_total_steps          = 0;
static uint32_t s_last_step_ms         = 0;

// Per-minute history for the last hour
static uint16_t s_steps_per_min[IMU_HISTORY_MINUTES];
static uint8_t  s_curr_min_idx         = 0;
static uint32_t s_curr_bucket_start_ms = 0;
static uint32_t s_steps_last_hour_sum  = 0;

// ==============================
//  SPI helpers
// ==============================

static void imu_spi_init_pins(void)
{
    // Hook the pins up to the SPI peripheral
    gpio_set_function(IMU_PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(IMU_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(IMU_PIN_MISO, GPIO_FUNC_SPI);

    // CS is a regular GPIO, active low
    gpio_init(IMU_PIN_CS);
    gpio_set_dir(IMU_PIN_CS, GPIO_OUT);
    gpio_put(IMU_PIN_CS, 1);
}

static inline void imu_select(void)
{
    gpio_put(IMU_PIN_CS, 0);
}

static inline void imu_deselect(void)
{
    gpio_put(IMU_PIN_CS, 1);
}

// Write a single register over SPI
static void imu_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    // For a write, MSB=0. Auto-increment is not needed for a single register.
    tx[0] = reg & 0x3F;  // clear R/W and auto-inc bits just in case
    tx[1] = value;

    imu_select();
    spi_write_blocking(IMU_SPI_PORT, tx, 2);
    imu_deselect();
}

// Read a single register over SPI
static uint8_t imu_read_reg(uint8_t reg)
{
    uint8_t tx = reg | 0x80; // MSB=1 → read, auto-inc off
    uint8_t rx = 0;

    imu_select();
    spi_write_blocking(IMU_SPI_PORT, &tx, 1);
    spi_read_blocking(IMU_SPI_PORT, 0xFF, &rx, 1);
    imu_deselect();

    return rx;
}

// Read multiple consecutive registers (auto-increment)
static void imu_read_regs(uint8_t start_reg, uint8_t *buf, size_t len)
{
    uint8_t tx = start_reg | 0xC0; // MSB=1 (read), bit6=1 (auto-increment)

    imu_select();
    spi_write_blocking(IMU_SPI_PORT, &tx, 1);
    spi_read_blocking(IMU_SPI_PORT, 0xFF, buf, len);
    imu_deselect();
}

// Grab a 3-axis accelerometer sample (raw LSB units)
static void imu_read_accel_raw_internal(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t raw[6];
    imu_read_regs(LSM6DS3_REG_OUTX_L_XL, raw, 6);

    int16_t x = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t y = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t z = (int16_t)((raw[5] << 8) | raw[4]);

    if (ax) *ax = x;
    if (ay) *ay = y;
    if (az) *az = z;
}

// ==============================
//  Time bucket / history helpers
// ==============================

static void imu_history_reset(uint32_t now_ms)
{
    memset(s_steps_per_min, 0, sizeof(s_steps_per_min));
    s_curr_min_idx         = 0;
    s_curr_bucket_start_ms = now_ms;
    s_steps_last_hour_sum  = 0;
}

// Advance the "current minute" bucket based on now_ms
static void imu_history_advance_buckets(uint32_t now_ms)
{
    if (s_curr_bucket_start_ms == 0) {
        // First time we get called: initialise starting point
        imu_history_reset(now_ms);
        return;
    }

    // Move forward in 60-second chunks until the buckets are up to date
    while ((now_ms - s_curr_bucket_start_ms) >= 60000u) {
        s_curr_bucket_start_ms += 60000u;
        s_curr_min_idx = (uint8_t)((s_curr_min_idx + 1) % IMU_HISTORY_MINUTES);

        // Drop the oldest bucket from the running sum and clear it
        s_steps_last_hour_sum -= s_steps_per_min[s_curr_min_idx];
        s_steps_per_min[s_curr_min_idx] = 0;
    }
}

// ==============================
//  Public API
// ==============================

bool imu_init(void)
{
    // One-time SPI peripheral setup
    static bool spi_initialized = false;
    if (!spi_initialized) {
        imu_spi_init_pins();
        spi_init(IMU_SPI_PORT, IMU_SPI_BAUD);

        // 8 bits per frame, SPI mode 3, MSB first
        spi_set_format(IMU_SPI_PORT,
                       8,
                       SPI_CPOL_1,
                       SPI_CPHA_1,
                       SPI_MSB_FIRST);

        spi_initialized = true;
    }

    // Give the sensor some time to power up
    sleep_ms(20);

    // Confirm we're talking to the correct device
    uint8_t whoami = imu_read_reg(LSM6DS3_REG_WHO_AM_I);
    if (whoami != LSM6DS3_WHO_AM_I_VALUE) {
        // Optional: printf for debugging if stdio is enabled
        // printf("IMU WHO_AM_I mismatch: 0x%02X (expected 0x%02X)\n", whoami, LSM6DS3_WHO_AM_I_VALUE);
        s_initialized = false;
        return false;
    }

    // Basic control register setup
    // CTRL3_C: enable auto-increment (IF_INC) and block data update (BDU)
    // Bit2 = IF_INC, Bit3 = BDU
    uint8_t ctrl3_c = (1u << 2) | (1u << 3);
    imu_write_reg(LSM6DS3_REG_CTRL3_C, ctrl3_c);

    // CTRL1_XL: accelerometer configuration
    // ODR_XL = 104 Hz (0b0100 << 4 = 0x40), FS_XL = ±2g (00)
    imu_write_reg(LSM6DS3_REG_CTRL1_XL, 0x40);

    // CTRL2_G: gyroscope configuration
    // ODR_G = 104 Hz, FS_G = ±245 dps
    imu_write_reg(LSM6DS3_REG_CTRL2_G, 0x40);

    // CTRL8_XL etc. left at default for now (no extra filtering here)

    // Reset runtime state
    s_raw_ax = s_raw_ay = s_raw_az = 0;
    s_filt_ax = s_filt_ay = s_filt_az = 0.0f;
    s_mag_lp_initialized = false;
    s_mag_lp = 0.0f;
    s_mag_hp = 0.0f;
    s_total_steps = 0;
    s_last_step_ms = 0;
    imu_history_reset(0); // will be re-aligned on the first update() call

    s_initialized = true;
    return true;
}

void imu_update(uint32_t now_ms)
{
    if (!s_initialized) {
        return;
    }

    // 1) Update the per-minute buckets according to the current time
    imu_history_advance_buckets(now_ms);

    // 2) Read raw accelerometer data (LSB)
    int16_t ax, ay, az;
    imu_read_accel_raw_internal(&ax, &ay, &az);
    s_raw_ax = ax;
    s_raw_ay = ay;
    s_raw_az = az;

    // 3) Convert to g units (assuming ±2g full-scale)
    float ax_g = (float)ax * IMU_ACCEL_LSB_2G;
    float ay_g = (float)ay * IMU_ACCEL_LSB_2G;
    float az_g = (float)az * IMU_ACCEL_LSB_2G;

    s_filt_ax = ax_g;
    s_filt_ay = ay_g;
    s_filt_az = az_g;

    // 4) Compute magnitude and apply a crude high-pass to remove gravity
    float mag = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

    if (!s_mag_lp_initialized) {
        // First sample seeds the low-pass
        s_mag_lp = mag;
        s_mag_lp_initialized = true;
    } else {
        s_mag_lp += IMU_MAG_LP_ALPHA * (mag - s_mag_lp);
    }

    s_mag_hp = mag - s_mag_lp;

    // 5) Very simple step detector:
    //    - look for high-pass magnitude above threshold
    //    - enforce a minimum time interval between step events
    if (s_mag_hp > IMU_STEP_THRESHOLD_G) {
        uint32_t dt = now_ms - s_last_step_ms;
        if (dt > IMU_STEP_MIN_INTERVAL_MS) {
            s_last_step_ms = now_ms;
            s_total_steps++;

            // Count step into the current minute bucket
            s_steps_per_min[s_curr_min_idx]++;
            s_steps_last_hour_sum++;
        }
    }
}

void imu_get_accel_raw(int16_t *ax, int16_t *ay, int16_t *az)
{
    if (ax) *ax = s_raw_ax;
    if (ay) *ay = s_raw_ay;
    if (az) *az = s_raw_az;
}

void imu_get_accel_filtered(float *ax, float *ay, float *az)
{
    if (ax) *ax = s_filt_ax;
    if (ay) *ay = s_filt_ay;
    if (az) *az = s_filt_az;
}

uint32_t imu_get_total_steps(void)
{
    return s_total_steps;
}

uint16_t imu_get_steps_last_hour(void)
{
    if (s_steps_last_hour_sum > 0xFFFFu) {
        // Clamp in case of overflow (shouldn't realistically happen)
        return 0xFFFFu;
    }
    return (uint16_t)s_steps_last_hour_sum;
}

bool imu_step_goal_reached(void)
{
    return imu_get_steps_last_hour() >= IMU_STEP_GOAL_PER_HOUR;
}

// Very rough activity classification based on steps in the last hour.
// 0: almost no movement
// 1: light activity
// 2: around the hourly goal
// 3: well above the goal (very active)
uint8_t imu_get_activity_level(void)
{
    uint16_t steps = imu_get_steps_last_hour();

    if (steps < 50) {
        return 0;
    } else if (steps < IMU_STEP_GOAL_PER_HOUR) {
        return 1;
    } else if (steps < (IMU_STEP_GOAL_PER_HOUR * 2)) {
        return 2;
    } else {
        return 3;
    }
}
