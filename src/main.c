#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <getopt.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "sampler.h"

static void print_usage(const char* prog){
    printf("Usage: %s [options]\n", prog);
    printf("  --spi PATH        SPI device (default /dev/spidev0.0)\n");
    printf("  --speed HZ        SPI speed (default 1500000)\n");
    printf("  --drdy N          BCM GPIO for DRDY (default 17)\n");
    printf("  --reset N         BCM GPIO for RESET (-1 to disable, default 18)\n");
    printf("  --chip N          gpiochip index (default 0)\n");
    printf("  --vref V          reference voltage in volts (default 2.5)\n");
    printf("  --pga N           PGA gain (1,2,4,8,16,32,64) default 1\n");
    printf("  --drate SPS       target samples per second (per conversion) default 1000\n");
    printf("  --frames N        capture N frames then exit (0=forever) default 10\n");
    printf("  --host IP         PC host IP to connect (default 127.0.0.1)\n");
    printf("  --port N          PC port to connect (default 12345)\n");
    printf("  --burst N         send N frames per packet (default 1)\n");
}

int main(int argc, char** argv){
    const char* spi = "/dev/spidev0.0";
    int speed = 1500000;
    int drdy = 17;
    int reset = 18;
    int chip = 0;
    double vref = 2.5;
    int pga = 1;
    int drate = 1000;
    int frames = 10;
    const char* host = "127.0.0.1";
    int port = 12345;
    int burst = 1;

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
        {"host", required_argument, 0, 0},
        {"port", required_argument, 0, 0},
        {0,0,0,0}
    };

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
            else if (!strcmp(name, "host")) host = optarg;
            else if (!strcmp(name, "port")) port = atoi(optarg);
        }
    }

    sampler_t s;
    if (sampler_start(&s, spi, speed, chip, drdy, reset, pga, vref, drate, 256) != 0) {
        fprintf(stderr, "Failed to start sampler. Are SPI and libgpiod available?\n");
        return 1;
    }

    // Connect to PC
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("socket");
        sampler_stop(&s);
        return 1;
    }
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    if (inet_pton(AF_INET, host, &server_addr.sin_addr) <= 0) {
        perror("inet_pton");
        close(sock);
        sampler_stop(&s);
        return 1;
    }
    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("connect");
        close(sock);
        sampler_stop(&s);
        return 1;
    }
    printf("Connected to %s:%d\n", host, port);

    // Burst sending: send 'burst' frames per packet. burst==1 => original behavior (one frame per packet)
    if (burst < 1) burst = 1;
    size_t samples_per_packet = (size_t)burst * 8;
    size_t packet_bytes = samples_per_packet * sizeof(int16_t);
    int remaining = frames;

    int running = 1;
    // allocate buffer for converted samples
    int16_t *packet_buf = malloc(packet_bytes);
    if (!packet_buf) {
        fprintf(stderr, "Failed to allocate packet buffer\n");
        close(sock);
        sampler_stop(&s);
        return 1;
    }

    while (frames == 0 || remaining > 0) {
        int to_collect = burst;
        if (frames != 0 && remaining < burst) to_collect = remaining;

        // collect 'to_collect' frames
        int collected = 0;
        for (int fidx = 0; fidx < to_collect; fidx++) {
            ads1256_frame_t f;
            // wait for a frame
            int ok = 0;
            for (int i=0;i<100;i++) { // wait up to ~1s
                if (ads1256_ring_pop(&s.ring, &f)) { ok = 1; break; }
                usleep(10000);
            }
            if (!ok) { fprintf(stderr, "Timeout waiting for frame\n"); running = 0; break; }

            // convert and store into packet buffer
            for (int ch = 0; ch < 8; ch++) {
                int16_t v16 = (int16_t)(f.ch[ch] >> 8); // keep existing truncation behavior
                packet_buf[(fidx*8) + ch] = htons(v16);
            }
            collected++;
            if (frames != 0) remaining--;
        }

        if (collected == 0) break;

        // send collected samples (collected * 8 * 2 bytes)
        ssize_t to_send = (ssize_t)(collected * 8 * sizeof(int16_t));
        if (send(sock, packet_buf, to_send, 0) < 0) {
            perror("send");
            break;
        }

        // wait 10ms per frame collected to preserve original cadence
        usleep(10000 * collected);
        if (!running) break;
    }

    free(packet_buf);

    close(sock);
    sampler_stop(&s);
    return 0;
}
