#include "sampler.h"
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

// Função da thread do sampler: lê frames continuamente e os empurra no buffer circular
static void* sampler_thread(void* arg) {
    sampler_t* s = (sampler_t*)arg;
    int32_t ch[8]; // Array para armazenar valores dos 8 canais
    while (s->running) { // Loop enquanto o sampler estiver rodando
        if (ads1256_read_8_single_ended(&s->dev, ch) == 0) { // Lê os 8 canais
            ads1256_frame_t f = {0}; // Cria um novo frame
            for (int i=0;i<8;++i) f.ch[i]=ch[i]; // Copia os valores dos canais
            f.t_ns = 0; // Timestamp preenchido por ads1256.c se desejado; aqui mantemos 0
            ads1256_ring_push(&s->ring, &f); // Empurra o frame no buffer circular
        } else {
            // Pequeno atraso em caso de erro
            usleep(1000);
        }
    }
    return NULL;
}

// Inicia o sampler: abre dispositivo, configura, calibra, inicializa buffer e cria thread
int sampler_start(sampler_t* s, const char* spi, int speed, int gpiochip, int drdy, int reset,
                  int pga, double vref, int drate_sps, size_t ring_capacity) {
    memset(s, 0, sizeof(*s)); // Zera a estrutura do sampler
    if (ads1256_open(&s->dev, spi, speed, gpiochip, drdy, reset) != 0) return -1; // Abre o dispositivo ADS1256
    if (ads1256_configure(&s->dev, pga, vref, drate_sps) != 0) return -1; // Configura PGA, Vref, DRATE
    if (ads1256_self_calibrate(&s->dev) != 0) fprintf(stderr, "Warning: calibration failed or timed out\n"); // Calibração automática
    if (ads1256_ring_init(&s->ring, ring_capacity?ring_capacity:128) != 0) return -1; // Inicializa buffer circular
    s->running = 1; // Marca como rodando
    s->pga = pga; s->vref = vref; // Armazena configurações
    if (pthread_create(&s->th, NULL, sampler_thread, s) != 0) { perror("pthread_create"); return -1; } // Cria thread do sampler
    return 0;
}

// Para o sampler: sinaliza parada, aguarda thread, libera buffer e fecha dispositivo
void sampler_stop(sampler_t* s) {
    if (!s) return; // Verifica se ponteiro é válido
    s->running = 0; // Sinaliza para parar a thread
    if (s->th) pthread_join(s->th, NULL); // Aguarda a thread terminar
    ads1256_ring_free(&s->ring); // Libera o buffer circular
    ads1256_close(&s->dev); // Fecha o dispositivo ADS1256
}
//