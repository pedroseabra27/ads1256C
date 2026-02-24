#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Mapa de registradores do ADS1256
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

// Comandos do ADS1256
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

// Códigos de taxa de dados (ver datasheet)
// Estes são códigos típicos; mapeamento para SPS está em ads1256.c

// Estrutura representando o dispositivo ADS1256
typedef struct {
    int spi_fd;                // File descriptor do SPI
    int gpiochip_index;        // Índice do gpiochip para libgpiod
    int line_drdy;             // Número BCM para DRDY
    int line_reset;            // Número BCM para RESET ou -1
    void* gpiod_chip;          // Ponteiro opaco para chip gpiod
    void* gpiod_line_drdy;     // Ponteiro opaco para linha DRDY
    void* gpiod_line_reset;    // Ponteiro opaco para linha RESET
    int pga_gain;              // Ganho PGA: 1,2,4,8,16,32,64
    double vref;               // Tensão de referência em volts
    uint8_t drate_code;        // Código do registrador DRATE do ADS1256
} ads1256_t;

// Um único frame de 8 canais
typedef struct {
    int32_t ch[8];      // Valores brutos 24-bit com extensão de sinal
    uint64_t t_ns;      // Timestamp (monotônico) quando o frame terminou
} ads1256_frame_t;

// Buffer circular para frames
typedef struct {
    ads1256_frame_t* buf;  // Buffer de frames
    size_t capacity;       // Capacidade em número de frames
    size_t head;           // Próximo índice de escrita
    size_t count;          // Número atual de frames válidos
    // Lock interno
    void* mtx;             // Mutex para thread-safety
} ads1256_ring_t;

// API do ADS1256

// Abre e inicializa o dispositivo ADS1256
int ads1256_open(ads1256_t* dev, const char* spi_path, int spi_speed_hz,
                 int gpiochip_index, int drdy_bcm, int reset_bcm);

// Fecha e libera recursos do dispositivo
void ads1256_close(ads1256_t* dev);

// Configura PGA, Vref e DRATE
int ads1256_configure(ads1256_t* dev, int pga_gain, double vref_volts, int drate_sps);

// Executa calibração automática offset/gain
int ads1256_self_calibrate(ads1256_t* dev);

// Lê uma conversão one-shot de um canal
int ads1256_read_one_shot(ads1256_t* dev, uint8_t ainp, uint8_t ainm, int32_t* out);

// Lê os 8 canais single-ended
int ads1256_read_8_single_ended(ads1256_t* dev, int32_t out_ch[8]);

// Funções auxiliares do buffer circular

// Inicializa o buffer circular
int ads1256_ring_init(ads1256_ring_t* r, size_t capacity);

// Libera o buffer circular
void ads1256_ring_free(ads1256_ring_t* r);

// Empurra um frame no buffer (sobrescreve se cheio)
void ads1256_ring_push(ads1256_ring_t* r, const ads1256_frame_t* f);

// Pop um frame do buffer (retorna 0 se vazio)
int ads1256_ring_pop(ads1256_ring_t* r, ads1256_frame_t* out); // 0 se nenhum

#ifdef __cplusplus
}
#endif
