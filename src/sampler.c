#include "sampler.h"
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

static void* sampler_thread(void* arg) {
    sampler_t* s = (sampler_t*)arg;
    int32_t ch[8];
    while (s->running) {
        if (ads1256_read_8_single_ended(&s->dev, ch) == 0) {
            ads1256_frame_t f = {0};
            for (int i=0;i<8;++i) f.ch[i]=ch[i];
            f.t_ns = 0; // filled by ads1256.c if desired; here we keep 0
            ads1256_ring_push(&s->ring, &f);
        } else {
            // small backoff on error
            usleep(1000);
        }
    }
    return NULL;
}

int sampler_start(sampler_t* s, const char* spi, int speed, int gpiochip, int drdy, int reset,
                  int pga, double vref, int drate_sps, size_t ring_capacity) {
    memset(s, 0, sizeof(*s));
    if (ads1256_open(&s->dev, spi, speed, gpiochip, drdy, reset) != 0) return -1;
    if (ads1256_configure(&s->dev, pga, vref, drate_sps) != 0) return -1;
    if (ads1256_self_calibrate(&s->dev) != 0) fprintf(stderr, "Warning: calibration failed or timed out\n");
    if (ads1256_ring_init(&s->ring, ring_capacity?ring_capacity:128) != 0) return -1;
    s->running = 1;
    s->pga = pga; s->vref = vref;
    if (pthread_create(&s->th, NULL, sampler_thread, s) != 0) { perror("pthread_create"); return -1; }
    return 0;
}

void sampler_stop(sampler_t* s) {
    if (!s) return;
    s->running = 0;
    if (s->th) pthread_join(s->th, NULL);
    ads1256_ring_free(&s->ring);
    ads1256_close(&s->dev);
}
