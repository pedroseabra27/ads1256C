#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include "ads1256.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    ads1256_t dev;
    ads1256_ring_t ring;
    int running;
    pthread_t th;
    double vref;
    int pga;
} sampler_t;

int sampler_start(sampler_t* s, const char* spi, int speed, int gpiochip, int drdy, int reset,
                  int pga, double vref, int drate_sps, size_t ring_capacity);
void sampler_stop(sampler_t* s);

#ifdef __cplusplus
}
#endif
