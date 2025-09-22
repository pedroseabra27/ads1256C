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

    int remaining = frames;
    while (frames == 0 || remaining-- > 0) {
        ads1256_frame_t f;
        // wait for a frame
        int ok = 0;
        for (int i=0;i<100;i++) { // wait up to ~1s
            if (ads1256_ring_pop(&s.ring, &f)) { ok = 1; break; }
            usleep(10000);
        }
        if (!ok) { fprintf(stderr, "Timeout waiting for frame\n"); break; }
        
        // Convert to 16-bit and send
        int16_t buffer[8];
        for (int i = 0; i < 8; i++) {
            buffer[i] = htons((int16_t)(f.ch[i] >> 8));  // Convert 24-bit to 16-bit, network byte order
        }
        if (send(sock, buffer, sizeof(buffer), 0) < 0) {
            perror("send");
            break;
        }
        
        // Wait 10ms
        usleep(10000);
    }

    close(sock);
    sampler_stop(&s);
    return 0;
}
