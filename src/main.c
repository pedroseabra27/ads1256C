#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <getopt.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>

#include "sampler.h"

// Estrutura do header do pacote: 32 bytes
typedef struct {
    uint64_t timestamp_ns;  // Timestamp em nanosegundos
    uint32_t packet_counter; // Contador sequencial
    uint32_t machine_id;    // ID da máquina
    uint8_t padding[16];    // Padding para 32 bytes
} packet_header_t;

// Contador global de pacotes
static uint32_t global_packet_counter = 0;

// Imprime o uso do programa
static void print_usage(const char* prog){
    printf("Usage: %s [options]\n", prog);
    printf("  --spi PATH        Dispositivo SPI (padrão /dev/spidev0.0)\n");
    printf("  --speed HZ        Velocidade SPI (padrão 1500000)\n");
    printf("  --drdy N          GPIO BCM para DRDY (padrão 17)\n");
    printf("  --reset N         GPIO BCM para RESET (-1 para desabilitar, padrão 18)\n");
    printf("  --chip N          Índice do gpiochip (padrão 0)\n");
    printf("  --vref V          Tensão de referência em volts (padrão 2.5)\n");
    printf("  --pga N           Ganho PGA (1,2,4,8,16,32,64) padrão 1\n");
    printf("  --drate SPS       SPS alvo (por conversão) padrão 1000\n");
    printf("  --frames N        Capturar N frames e sair (0=indefinidamente) padrão 10\n");
    printf("  --udp-host IP     IP do host UDP para enviar (padrão 127.0.0.1)\n");
    printf("  --port N          Porta UDP para enviar (padrão 12345)\n");
    printf("  --burst N         Enviar N frames por pacote (padrão 1)\n");
}

int main(int argc, char** argv){
    // Valores padrão
    const char* spi = "/dev/spidev0.0";
    int speed = 1500000;
    int drdy = 17;
    int reset = 18;
    int chip = 0;
    double vref = 1.0;
    int pga = 1;
    int drate = 1000;
    int frames = 10;
    const char* udp_host = "127.0.0.1";
    int port = 12345;
    int burst = 1;

    // Opções longas para getopt
    static struct option long_opts[] = {
        {"spi", required_argument, 0, 0},
        {"speed", required_argument, 0, 0},
        {"drdy", required_argument, 0, 0},
        {"reset", required_argument, 0, 0},
        {"chip", required_argument, 0, 0},
        {"vref", required_argument, 0, 0},
        {"pga", required_argument, 0, 0},
        {"drate", required_argument, 0, 0},
        {"frames", required_argument, 0, 0},
        {"burst", required_argument, 0, 0},
        {"udp-host", required_argument, 0, 0},
        {"port", required_argument, 0, 0},
        {0,0,0,0}
    };

    // Parsing dos argumentos
    while (1) {
        int opt_index = 0;
        int c = getopt_long(argc, argv, "", long_opts, &opt_index);
        if (c == -1) break;
        if (c == '?') { print_usage(argv[0]); return 1; }
        if (c == 0) {
            const char* name = long_opts[opt_index].name;
            if (!strcmp(name, "spi")) spi = optarg;
            else if (!strcmp(name, "speed")) speed = atoi(optarg);
            else if (!strcmp(name, "drdy")) drdy = atoi(optarg);
            else if (!strcmp(name, "reset")) reset = atoi(optarg);
            else if (!strcmp(name, "chip")) chip = atoi(optarg);
            else if (!strcmp(name, "vref")) vref = atof(optarg);
            else if (!strcmp(name, "pga")) pga = atoi(optarg);
            else if (!strcmp(name, "drate")) drate = atoi(optarg);
            else if (!strcmp(name, "frames")) frames = atoi(optarg);
            else if (!strcmp(name, "burst")) burst = atoi(optarg);
            else if (!strcmp(name, "udp-host")) udp_host = optarg;
            else if (!strcmp(name, "port")) port = atoi(optarg);
        }
    }

    // Iniciar o sampler
    sampler_t s;
    if (sampler_start(&s, spi, speed, chip, drdy, reset, pga, vref, drate, 100) != 0) {
        fprintf(stderr, "Falha ao iniciar sampler. SPI e libgpiod disponíveis?\n");
        return 1;
    }

    // Criar socket UDP
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket");
        sampler_stop(&s);
        return 1;
    }
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    if (inet_pton(AF_INET, udp_host, &server_addr.sin_addr) <= 0) {
        perror("inet_pton");
        close(sock);
        sampler_stop(&s);
        return 1;
    }
    printf("Socket UDP pronto para enviar para %s:%d\n", udp_host, port);

    // Agrupar 20 frames por pacote (20 x 16 bytes = 320 bytes de dados)
    const int FRAMES_PER_PACKET = 20;
    const int HEADER_SIZE = sizeof(packet_header_t);
    size_t data_bytes = (size_t)FRAMES_PER_PACKET * 8 * sizeof(int16_t);
    size_t packet_bytes = HEADER_SIZE + data_bytes;
    int remaining = frames;

    int running = 1;
    // Alocar buffer para header + amostras convertidas (20 frames)
    uint8_t *packet_buf = malloc(packet_bytes);
    if (!packet_buf) {
        fprintf(stderr, "Falha ao alocar buffer de pacote\n");
        close(sock);
        sampler_stop(&s);
        return 1;
    }

    int frames_in_buffer = 0; // rastrear frames acumulados

    while (frames == 0 || remaining > 0) {
        // Coletar 1 frame por vez
        ads1256_frame_t f;
        // Esperar por um frame
        int ok = 0;
        for (int i=0;i<100;i++) { // esperar até ~1s
            if (ads1256_ring_pop(&s.ring, &f)) { ok = 1; break; }
            usleep(10000);
        }
        if (!ok) { fprintf(stderr, "Timeout esperando frame\n"); running = 0; break; }

        // Converter e armazenar no buffer de pacote (após header)
        int16_t *data_ptr = (int16_t*)(packet_buf + HEADER_SIZE);   
        for (int ch = 0; ch < 8; ch++) {
            int16_t v16 = (int16_t)(f.ch[ch] >> 8); // converter 24-bit para 16-bit
            data_ptr[(frames_in_buffer*8) + ch] = htons(v16);
        }
        frames_in_buffer++;
        if (frames != 0) remaining--;

        // Se buffer cheio (20 frames = 320 bytes de dados), enviar pacote UDP
        if (frames_in_buffer >= FRAMES_PER_PACKET) {
            // Preencher header
            packet_header_t *hdr = (packet_header_t*)packet_buf;
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            hdr->timestamp_ns = (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
            hdr->packet_counter = global_packet_counter++;
            hdr->machine_id = 1; // ID da máquina fixo
            memset(hdr->padding, 0, sizeof(hdr->padding));

            ssize_t to_send = (ssize_t)packet_bytes;
            if (sendto(sock, packet_buf, to_send, 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
                perror("sendto");
                break;
            }
            frames_in_buffer = 0; // resetar buffer
        }

        if (!running) break;
    }

    // Enviar frames restantes se buffer não vazio
    if (frames_in_buffer > 0) {
        // Preencher header para pacote parcial
        packet_header_t *hdr = (packet_header_t*)packet_buf;
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        hdr->timestamp_ns = (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
        hdr->packet_counter = global_packet_counter++;
        hdr->machine_id = 1;
        memset(hdr->padding, 0, sizeof(hdr->padding));

        ssize_t to_send = HEADER_SIZE + (ssize_t)(frames_in_buffer * 8 * sizeof(int16_t));
        if (sendto(sock, packet_buf, to_send, 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            perror("sendto");
        }
    }

    free(packet_buf);

    close(sock);
    sampler_stop(&s);
    return 0;
}
