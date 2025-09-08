#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ADS1256 register map
#define ADS1256_REG_STATUS 0x00
#define ADS1256_REG_MUX    0x01
#define ADS1256_REG_ADCON  0x02
#define ADS1256_REG_DRATE  0x03
#define ADS1256_REG_IO     0x04
#define ADS1256_REG_OFC0   0x05
#define ADS1256_REG_OFC1   0x06
#define ADS1256_REG_OFC2   0x07
#define ADS1256_REG_FSC0   0x08
#define ADS1256_REG_FSC1   0x09
#define ADS1256_REG_FSC2   0x0A

// ADS1256 commands
#define ADS1256_CMD_WAKEUP 0x00
#define ADS1256_CMD_RDATA  0x01
#define ADS1256_CMD_RDATAC 0x03
#define ADS1256_CMD_SDATAC 0x0F
#define ADS1256_CMD_RREG   0x10
#define ADS1256_CMD_WREG   0x50
#define ADS1256_CMD_SELFCAL 0xF0
#define ADS1256_CMD_SELFOCAL 0xF1
#define ADS1256_CMD_SELFGCAL 0xF2
#define ADS1256_CMD_SYSOCAL 0xF3
#define ADS1256_CMD_SYSGCAL 0xF4
#define ADS1256_CMD_SYNC   0xFC
#define ADS1256_CMD_STANDBY 0xFD
#define ADS1256_CMD_RESET  0xFE

// Data rate codes (see datasheet)
// These are typical codes; mapping to SPS is in ads1256.c

typedef struct {
    int spi_fd;
    int gpiochip_index;     // for libgpiod
    int line_drdy;          // BCM number
    int line_reset;         // BCM number or -1
    void* gpiod_chip;       // opaque to callers
    void* gpiod_line_drdy;  // opaque to callers
    void* gpiod_line_reset; // opaque to callers
    int pga_gain;           // 1,2,4,8,16,32,64
    double vref;            // volts
    uint8_t drate_code;     // ADS1256 DRATE register code
} ads1256_t;

// Single frame of 8 channels
typedef struct {
    int32_t ch[8];      // raw 24-bit sign-extended
    uint64_t t_ns;      // timestamp (monotonic) when frame finished
} ads1256_frame_t;

// Circular buffer for frames
typedef struct {
    ads1256_frame_t* buf;
    size_t capacity;    // number of frames capacity
    size_t head;        // next write index
    size_t count;       // current number of valid frames
    // internal lock
    void* mtx;
} ads1256_ring_t;

// API
int ads1256_open(ads1256_t* dev, const char* spi_path, int spi_speed_hz,
                 int gpiochip_index, int drdy_bcm, int reset_bcm);
void ads1256_close(ads1256_t* dev);

int ads1256_configure(ads1256_t* dev, int pga_gain, double vref_volts, int drate_sps);
int ads1256_self_calibrate(ads1256_t* dev);

int ads1256_read_one_shot(ads1256_t* dev, uint8_t ainp, uint8_t ainm, int32_t* out);
int ads1256_read_8_single_ended(ads1256_t* dev, int32_t out_ch[8]);

// Ring helpers
int ads1256_ring_init(ads1256_ring_t* r, size_t capacity);
void ads1256_ring_free(ads1256_ring_t* r);
void ads1256_ring_push(ads1256_ring_t* r, const ads1256_frame_t* f);
int ads1256_ring_pop(ads1256_ring_t* r, ads1256_frame_t* out); // 0 if none

#ifdef __cplusplus
}
#endif
