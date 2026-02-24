#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include "ads1256.h"

#ifdef __cplusplus
extern "C" {
#endif

// Estrutura representando o sampler, que gerencia o dispositivo ADS1256 e o buffer circular
typedef struct {
    ads1256_t dev;         // Dispositivo ADS1256
    ads1256_ring_t ring;   // Buffer circular para frames
    int running;           // Flag indicando se o sampler está rodando
    pthread_t th;          // Thread do sampler
    double vref;           // Tensão de referência
    int pga;               // Ganho PGA
} sampler_t;

// Inicia o sampler: abre dispositivo, configura, calibra, inicializa buffer e cria thread
int sampler_start(sampler_t* s, const char* spi, int speed, int gpiochip, int drdy, int reset,
                  int pga, double vref, int drate_sps, size_t ring_capacity);

// Para o sampler: sinaliza parada, aguarda thread, libera recursos
void sampler_stop(sampler_t* s);

#ifdef __cplusplus
}
#endif
