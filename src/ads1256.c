#include "ads1256.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include <linux/spi/spidev.h>

// libgpiod opcional para controle eficiente de GPIOs
#ifdef USE_GPIOD
#include <gpiod.h>
#endif

// Macro para verificações de debug
#define CHECK(x) do { if(!(x)) { fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__, #x); } while(0)

// Configurações padrão do SPI para ADS1256
static uint8_t spi_mode = SPI_MODE_1; // ADS1256 requer modo 1 (CPOL=0, CPHA=1)
static uint8_t spi_bits = 8;
static uint32_t spi_speed = 1500000; // 1.5 MHz padrão
static uint16_t spi_delay = 0;

// Função auxiliar para dormir em nanosegundos
static inline void nsleep(long ns) {
    struct timespec ts; ts.tv_sec = ns/1000000000L; ts.tv_nsec = ns%1000000000L; nanosleep(&ts, NULL);
}

// Função para obter timestamp atual em nanosegundos
static uint64_t now_ns(void) {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return (uint64_t)ts.tv_sec*1000000000ull + ts.tv_nsec;
}

// Função para transação SPI básica (envia e recebe dados)
static int spi_txrx(int fd, const uint8_t* tx, uint8_t* rx, size_t len) {
    struct spi_ioc_transfer tr = {0};
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = len;
    tr.delay_usecs = 0;
    tr.speed_hz = spi_speed;
    tr.bits_per_word = spi_bits;
    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    return ret < 1 ? -1 : 0;
}

// Envia um comando SPI simples
static int spi_write_cmd(int fd, uint8_t cmd) {
    return spi_txrx(fd, &cmd, NULL, 1);
}

// Escreve em um registrador do ADS1256
static int ads1256_wreg(int fd, uint8_t reg, uint8_t value) {
    uint8_t buf[3] = { ADS1256_CMD_WREG | (reg & 0x0F), 0x00, value };
    return spi_txrx(fd, buf, NULL, sizeof(buf));
}

// Lê de um registrador do ADS1256
static int ads1256_rreg(int fd, uint8_t reg, uint8_t* value) {
    uint8_t tx[3] = { ADS1256_CMD_RREG | (reg & 0x0F), 0x00, 0xFF };
    uint8_t rx[3] = {0};
    int rc = spi_txrx(fd, tx, rx, sizeof(tx));
    if (rc==0) *value = rx[2];
    return rc;
}

// Espera pelo sinal DRDY (Data Ready) do ADS1256
static int ads1256_wait_drdy(ads1256_t* dev, int timeout_ms) {
#ifdef USE_GPIOD
    if (dev->gpiod_line_drdy) {
        // Polling para DRDY baixo (ativo)
        const int step_us = 200; // 0.2ms
        int remaining_us = timeout_ms * 1000;
        while (remaining_us > 0) {
            int v = gpiod_line_get_value((struct gpiod_line*)dev->gpiod_line_drdy);
            if (v == 0) return 0; // ativo baixo
            usleep(step_us);
            remaining_us -= step_us;
        }
        return -1;
    }
#endif
    // Fallback com sleep simples
    (void)dev;
    int waited = 0;
    const int step = 1; // ms
    while (waited < timeout_ms) { usleep(step*1000); waited += step; }
    return 0;
}

// Lê 24 bits de dados do ADS1256 após comando RDATA
static int ads1256_read_data24(int fd, int32_t* out) {
    // Envia RDATA, espera t6 (~50 tCLK) antes de ler 3 bytes
    uint8_t cmd = ADS1256_CMD_RDATA;
    if (spi_txrx(fd, &cmd, NULL, 1) != 0) return -1;
    nsleep(10*1000); // ~10us
    uint8_t tx[3] = {0xFF, 0xFF, 0xFF};
    uint8_t rx[3] = {0};
    if (spi_txrx(fd, tx, rx, sizeof(tx)) != 0) return -1;
    int32_t raw = ((int32_t)rx[0] << 16) | ((int32_t)rx[1] << 8) | rx[2];
    if (raw & 0x800000) raw |= 0xFF000000; // extensão de sinal 24-bit
    *out = raw;
    return 0;
}

// Mapeia SPS (samples per second) para código DRATE do ADS1256
static uint8_t drate_code_from_sps(int sps) {
    // Mapeamento simplificado; ajuste se necessário.
    // Códigos por datasheet Table 9 (típico): 30000..2.5 SPS
    struct { int sps; uint8_t code; } map[] = {
        {30000, 0xF0}, {15000, 0xE0}, {7500, 0xD0}, {3750, 0xC0}, {2000, 0xB0}, {1000, 0xA1},
        {500, 0x92}, {100, 0x82}, {60, 0x72}, {50, 0x63}, {30, 0x53}, {25, 0x43}, {15, 0x33}, {10, 0x23}, {5, 0x13}, {2, 0x03}
    };
    int best = 0; int best_err = 1e9;
    for (size_t i=0;i<sizeof(map)/sizeof(map[0]);++i) {
        int err = abs(map[i].sps - sps);
        if (err < best_err) { best_err = err; best = (int)i; }
    }
    return map[best].code;
}

// Abre e inicializa o dispositivo ADS1256
int ads1256_open(ads1256_t* dev, const char* spi_path, int spi_speed_hz,
                 int gpiochip_index, int drdy_bcm, int reset_bcm) {
    memset(dev, 0, sizeof(*dev));
    dev->spi_fd = open(spi_path, O_RDWR);
    if (dev->spi_fd < 0) { perror("open spidev"); return -1; }

    spi_speed = (spi_speed_hz>0)? (uint32_t)spi_speed_hz : spi_speed;

    if (ioctl(dev->spi_fd, SPI_IOC_WR_MODE, &spi_mode) == -1) { perror("SPI_IOC_WR_MODE"); return -1; }
    if (ioctl(dev->spi_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits) == -1) { perror("SPI_IOC_WR_BITS_PER_WORD"); return -1; }
    if (ioctl(dev->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) == -1) { perror("SPI_IOC_WR_MAX_SPEED_HZ"); return -1; }

    // Configuração libgpiod
    dev->gpiochip_index = gpiochip_index;
    dev->line_drdy = drdy_bcm;
    dev->line_reset = reset_bcm;

#ifdef USE_GPIOD
    if (drdy_bcm >= 0) {
        dev->gpiod_chip = gpiod_chip_open_by_number(gpiochip_index);
        if (!dev->gpiod_chip) { perror("gpiod_chip_open_by_number"); return -1; }
        dev->gpiod_line_drdy = gpiod_chip_get_line((struct gpiod_chip*)dev->gpiod_chip, drdy_bcm);
        if (!dev->gpiod_line_drdy) { fprintf(stderr, "Failed to get DRDY line %d\n", drdy_bcm); return -1; }
        if (gpiod_line_request_input((struct gpiod_line*)dev->gpiod_line_drdy, "ads1256") < 0) {
            perror("gpiod_line_request_input"); return -1; }
    }
    if (reset_bcm >= 0) {
        if (!dev->gpiod_chip) dev->gpiod_chip = gpiod_chip_open_by_number(gpiochip_index);
        dev->gpiod_line_reset = gpiod_chip_get_line((struct gpiod_chip*)dev->gpiod_chip, reset_bcm);
        if (!dev->gpiod_line_reset) { fprintf(stderr, "Failed to get RESET line %d\n", reset_bcm); return -1; }
        if (gpiod_line_request_output((struct gpiod_line*)dev->gpiod_line_reset, "ads1256", 1) < 0) { perror("gpiod_line_request_output"); return -1; }
        // Pulso de reset por hardware
        gpiod_line_set_value((struct gpiod_line*)dev->gpiod_line_reset, 0);
        nsleep(10*1000*1000);
        gpiod_line_set_value((struct gpiod_line*)dev->gpiod_line_reset, 1);
        nsleep(50*1000*1000);
    } else {
        // Envia comando RESET se não há GPIO reset
        spi_write_cmd(dev->spi_fd, ADS1256_CMD_RESET);
        nsleep(50*1000*1000);
    }
#else
    // Sem libgpiod ou desabilitado: apenas envia RESET e dorme
    (void)gpiochip_index; (void)drdy_bcm; (void)reset_bcm;
    spi_write_cmd(dev->spi_fd, ADS1256_CMD_RESET);
    nsleep(50*1000*1000);
#endif

    // Para qualquer conversão contínua
    spi_write_cmd(dev->spi_fd, ADS1256_CMD_SDATAC);

    return 0;
}

// Fecha e libera recursos do dispositivo ADS1256
void ads1256_close(ads1256_t* dev) {
    if (!dev) return;
    if (dev->spi_fd > 0) close(dev->spi_fd);
    #ifdef USE_GPIOD
    if (dev->gpiod_line_drdy) { gpiod_line_release((struct gpiod_line*)dev->gpiod_line_drdy); dev->gpiod_line_drdy = NULL; }
    if (dev->gpiod_line_reset) { gpiod_line_release((struct gpiod_line*)dev->gpiod_line_reset); dev->gpiod_line_reset = NULL; }
    if (dev->gpiod_chip) { gpiod_chip_close((struct gpiod_chip*)dev->gpiod_chip); dev->gpiod_chip = NULL; }
    #endif
}

// Configura o ADS1256 com PGA, Vref e DRATE
int ads1256_configure(ads1256_t* dev, int pga_gain, double vref_volts, int drate_sps) {
    if (pga_gain<=0) pga_gain=1;
    dev->pga_gain = pga_gain;
    dev->vref = vref_volts>0? vref_volts:2.5;
    dev->drate_code = drate_code_from_sps(drate_sps>0?drate_sps:1000);

    // STATUS: MSB first, Auto-Calibration desabilitado, BUFEN=1 recomendado
    uint8_t status = 0x00; // 0b00000000: ORDER=0 (MSB), ACAL=0, BUFEN=0
    // habilita buffer para single-ended
    status |= 0x02; // BUFEN=1
    ads1256_wreg(dev->spi_fd, ADS1256_REG_STATUS, status);

    // ADCON: CLKOUT off, Sensor Detect off, PGA
    uint8_t pga_bits = 0;
    switch(pga_gain){
        case 1: pga_bits=0; break; case 2: pga_bits=1; break; case 4: pga_bits=2; break; case 8: pga_bits=3; break;
        case 16: pga_bits=4; break; case 32: pga_bits=5; break; case 64: pga_bits=6; break; default: pga_bits=0; break;
    }
    uint8_t adcon = (0<<5) | (0<<3) | (pga_bits & 0x07);
    ads1256_wreg(dev->spi_fd, ADS1256_REG_ADCON, adcon);

    // DRATE
    ads1256_wreg(dev->spi_fd, ADS1256_REG_DRATE, dev->drate_code);

    // MUX padrão para AIN0-AINCOM
    ads1256_wreg(dev->spi_fd, ADS1256_REG_MUX, (0<<4) | 0x08);

    return 0;
}

// Executa calibração offset/gain do sistema
int ads1256_self_calibrate(ads1256_t* dev) {
    // Calibração offset/gain do sistema
    spi_write_cmd(dev->spi_fd, ADS1256_CMD_SYSOCAL);
    if (ads1256_wait_drdy(dev, 1000) != 0) return -1;
    spi_write_cmd(dev->spi_fd, ADS1256_CMD_SYSGCAL);
    if (ads1256_wait_drdy(dev, 1000) != 0) return -1;
    return 0;
}

// Configura o multiplexador MUX para canal específico
static int set_mux(ads1256_t* dev, uint8_t ainp, uint8_t ainm) {
    uint8_t val = ((ainp & 0x0F) << 4) | (ainm & 0x0F);
    return ads1256_wreg(dev->spi_fd, ADS1256_REG_MUX, val);
}

// Lê uma conversão one-shot de um canal single-ended
int ads1256_read_one_shot(ads1256_t* dev, uint8_t ainp, uint8_t ainm, int32_t* out) {
    set_mux(dev, ainp, ainm);
    // Sequência SYNC + WAKEUP para settling
    spi_write_cmd(dev->spi_fd, ADS1256_CMD_SYNC);
    nsleep(4*1000); // 4us min
    spi_write_cmd(dev->spi_fd, ADS1256_CMD_WAKEUP);
    // Espera DRDY
    if (ads1256_wait_drdy(dev, 1000) != 0) return -1;
    // RDATA
    return ads1256_read_data24(dev->spi_fd, out);
}

// Lê todos os 8 canais single-ended (AIN0..AIN7 vs AINCOM)
int ads1256_read_8_single_ended(ads1256_t* dev, int32_t out_ch[8]) {
    for (int ch=0; ch<8; ++ch) {
        if (ads1256_read_one_shot(dev, (uint8_t)ch, 0x08, &out_ch[ch]) != 0) return -1; // AINCOM=8
    }
    return 0;
}

// Implementação do buffer circular

typedef struct { pthread_mutex_t m; } _mtx_t;

// Inicializa o buffer circular
int ads1256_ring_init(ads1256_ring_t* r, size_t capacity) {
    memset(r, 0, sizeof(*r));
    r->buf = (ads1256_frame_t*)calloc(capacity, sizeof(ads1256_frame_t));
    if (!r->buf) return -1;
    r->capacity = capacity;
    _mtx_t* m = (_mtx_t*)malloc(sizeof(_mtx_t));
    if (!m) { free(r->buf); r->buf=NULL; return -1; }
    pthread_mutex_init(&m->m, NULL);
    r->mtx = m;
    return 0;
}

// Libera o buffer circular
void ads1256_ring_free(ads1256_ring_t* r) {
    if (!r) return;
    if (r->buf) free(r->buf);
    if (r->mtx) { pthread_mutex_destroy(&(((_mtx_t*)r->mtx)->m)); free(r->mtx); }
    memset(r, 0, sizeof(*r));
}

// Empurra um frame no buffer (sobrescreve se cheio)
void ads1256_ring_push(ads1256_ring_t* r, const ads1256_frame_t* f) {
    _mtx_t* m = (_mtx_t*)r->mtx;
    pthread_mutex_lock(&m->m);
    r->buf[r->head] = *f;
    r->head = (r->head + 1) % r->capacity;
    if (r->count < r->capacity) r->count++; // sobrescreve quando cheio
    pthread_mutex_unlock(&m->m);
}

// Pop um frame do buffer (retorna 0 se vazio)
int ads1256_ring_pop(ads1256_ring_t* r, ads1256_frame_t* out) {
    _mtx_t* m = (_mtx_t*)r->mtx;
    pthread_mutex_lock(&m->m);
    if (r->count == 0) { pthread_mutex_unlock(&m->m); return 0; }
    size_t tail = (r->head + r->capacity - r->count) % r->capacity;
    *out = r->buf[tail];
    r->count--;
    pthread_mutex_unlock(&m->m);
    return 1;
}
