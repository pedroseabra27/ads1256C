# ADS1256 Sampler for Raspberry Pi 5 (C)

This is a small C project to sample 8 single-ended channels from the ADS1256 (e.g., Waveshare High-Precision AD/DA Board) on a Raspberry Pi 5 using:

- SPI via `/dev/spidev*` (kernel chip select)
- DRDY and optional RESET using `libgpiod` events
- A sampling thread that scans CH0..CH7 each frame
- A lock-protected circular buffer of frames (8 channels + timestamp)

## Hardware assumptions

- Raspberry Pi SPI enabled (CE0 used for ADS1256 CS)
- DRDY -> BCM17 (GPIO17), RESET -> BCM18 (GPIO18) [adjustable]
- AINCOM used as negative input for single-ended mode
- Default SPI: `/dev/spidev0.0`, mode 1, 1.5 MHz

## Build (on Raspberry Pi)

Install dependencies:

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libgpiod-dev
```

Enable SPI (if not yet):

```bash
sudo raspi-config  # Interface Options -> SPI -> Enable
```

Build:

```bash
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j
```

## Run

Basic run (prints a few frames then exits):

```bash
./ads1256_sampler --frames 10
```

Options:

- `--spi /dev/spidev0.0` SPI device
- `--speed 1500000` SPI speed (Hz)
- `--drdy-chip 0` libgpiod chip index (0 usually maps to gpiochip0)
- `--drdy 17` BCM line for DRDY
- `--reset 18` BCM line for RESET (optional; pass -1 to disable)
- `--vref 2.500` Reference voltage (V) for conversion to volts
- `--pga 1` PGA gain: 1,2,4,8,16,32,64
- `--drate 1000` SPS target (approx) [choices mapped internally]
- `--frames N` How many frames to capture before exit (default 0 = run forever)

## Notes

- The ADS1256 does not have hardware channel scanning; we switch MUX per channel and use SYNC+WAKEUP and wait for DRDY each time. Effective per-channel rate is limited by selected DRATE and settling.
- Code adapts to `libgpiod` v1 or v2 at compile time.
- For best performance, pin the sampling thread and increase priority (not enabled by default).
