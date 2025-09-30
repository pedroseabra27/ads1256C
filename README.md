# ADS1256 Sampler para Raspberry Pi 5 (C)

Este é um pequeno projeto em C para amostrar 8 canais single-ended do ADS1256 (ex.: placa Waveshare High-Precision AD/DA Board) em uma Raspberry Pi 5, utilizando:

- SPI via `/dev/spidev*` (chip select do kernel)
- DRDY e RESET opcional usando eventos `libgpiod`
- Uma thread de amostragem que varre CH0..CH7 em cada frame
- Um buffer circular protegido por lock de frames (8 canais + timestamp)

## Funcionalidades Implementadas

- **Amostragem Contínua:** Captura frames de 8 canais em tempo real.
- **Conversão de Dados:** Valores 24-bit signed do ADS1256 são convertidos para 16-bit signed (truncamento ou escala opcional).
- **Transmissão TCP:** Envia buffers de dados via socket TCP para um PC, permitindo aquisição remota.
- **Modos de Transmissão:**
  - Streaming: 1 frame por pacote a cada 10 ms.
  - Burst: N frames por pacote (ex.: 10 frames = 160 bytes), enviados em intervalos maiores.
- **Configurações Ajustáveis:** Taxa de amostragem, ganho PGA, tensão de referência, etc.

## Suposições de Hardware

- Raspberry Pi SPI habilitado (CE0 usado para CS do ADS1256)
- DRDY -> BCM17 (GPIO17), RESET -> BCM18 (GPIO18) [ajustável]
- AINCOM usado como entrada negativa para modo single-ended
- SPI padrão: `/dev/spidev0.0`, modo 1, 1.5 MHz

## Dependências

Instale as dependências na Raspberry Pi:

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libgpiod-dev
```

Habilite SPI (se ainda não estiver):

```bash
sudo raspi-config  # Interface Options -> SPI -> Enable
```

## Construção (na Raspberry Pi)

```bash
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j
```

## Execução

Execução básica (imprime alguns frames e sai):

```bash
./ads1256_sampler --frames 10
```

### Opções de Linha de Comando

- `--spi /dev/spidev0.0` Dispositivo SPI
- `--speed 1500000` Velocidade SPI (Hz)
- `--drdy-chip 0` Índice do chip libgpiod (0 geralmente mapeia para gpiochip0)
- `--drdy 17` Linha BCM para DRDY
- `--reset 18` Linha BCM para RESET (opcional; passe -1 para desabilitar)
- `--vref 2.500` Tensão de referência (V) para conversão em volts
- `--pga 1` Ganho PGA: 1,2,4,8,16,32,64
- `--drate 1000` SPS alvo (aprox.) [escolhas mapeadas internamente]
- `--frames N` Quantos frames capturar antes de sair (padrão 0 = rodar indefinidamente)
- `--host IP` IP do PC para conectar (padrão 127.0.0.1)
- `--port N` Porta do PC para conectar (padrão 12345)
- `--burst N` Enviar N frames por pacote (padrão 1)

### Exemplos de Uso

- **Teste básico (imprimir no terminal):**
  ```bash
  ./ads1256_sampler --frames 5
  ```

- **Transmissão para PC (streaming):**
  ```bash
  ./ads1256_sampler --host 192.168.18.8 --port 12345 --frames 0 --burst 1
  ```

- **Transmissão burst (10 frames por pacote):**
  ```bash
  ./ads1256_sampler --host 192.168.18.8 --port 12345 --frames 0 --burst 10
  ```

## Teste com Receptor Python

Para testar a transmissão no PC, use o script `receive.py` incluído:

1. No PC, execute:
   ```bash
   python receive.py
   ```

2. Na Raspberry Pi, execute o sampler com `--host` apontando para o IP do PC.

O receptor imprimirá os valores recebidos (ex.: 8 valores int16 por frame).

## Frequências e Taxas

- **Taxa do ADC:** Configurável (padrão 1000 SPS por conversão). Com 8 canais varridos sequencialmente, taxa efetiva por canal ≈ 125 SPS.
- **Transmissão:** Pacotes enviados a 100 Hz (burst=10) ou 1000 Hz (burst=1), preservando cadência de 10 ms por frame.
- **Conversão:** Valores 24-bit (±8.388.607) convertidos para 16-bit (±32.767) via truncamento (shift right 8). Opção para escala proporcional se necessário.

## Notas

- O ADS1256 não tem varredura de canais por hardware; alternamos MUX por canal e usamos SYNC+WAKEUP e esperamos DRDY cada vez. A taxa efetiva por canal é limitada pela DRATE selecionada e tempo de settling.
- O código se adapta a `libgpiod` v1 ou v2 em tempo de compilação.
- Para melhor performance, fixe a thread de amostragem e aumente a prioridade (não habilitado por padrão).
- Formato de transmissão: int16 em big-endian (ordem de rede). Burst=N envia N×8 int16 por pacote.

## Próximos Passos

- Integrar com software de aquisição (ex.: IBA) para visualização em tempo real.
- Adicionar header com timestamp/seq se necessário.
- Melhorar robustez (reconexão TCP, tratamento de erros).
