# Freenove ESP32-S3 WROOM — Pinout & Wiring

![Pinout Diagram](ESP32S3_Pinout.png)

## Left Header (top → bottom)

| Pin | GPIO | Camera | ADC | Other |
|-----|-------|--------|-----|-------|
| 1 | **3V3** | — | — | Power |
| 2 | **RST** | — | — | Reset |
| 3 | GPIO4 | CAM_SIOD | ADC1_CH3 | T4 |
| 4 | GPIO5 | CAM_SIOC | ADC1_CH4 | T5 |
| 5 | GPIO6 | CAM_VSYNC | ADC1_CH5 | T6 |
| 6 | GPIO7 | CAM_HREF | ADC1_CH6 | T7 |
| 7 | GPIO15 | CAM_XCLK | ADC2_CH4 | U0RTS |
| 8 | GPIO16 | CAM_Y9 | ADC2_CH5 | U0CTS |
| 9 | GPIO17 | CAM_Y8 | ADC2_CH6 | U1TXD |
| 10 | GPIO18 | CAM_Y7 | ADC2_CH7 | U1RXD |
| 11 | GPIO8 | CAM_Y4 | ADC1_CH7 | T8 |
| 12 | GPIO3 | — | ADC1_CH2 | T3, JTAG_EN |
| 13 | GPIO46 | — | — | LOG (strap) |
| 14 | GPIO9 | CAM_Y3 | ADC1_CH8 | T9 |
| 15 | GPIO10 | CAM_Y5 | ADC1_CH9 | T10 |
| 16 | GPIO11 | CAM_Y2 | ADC2_CH0 | T11 |
| 17 | GPIO12 | CAM_Y6 | ADC2_CH1 | T12 |
| 18 | GPIO13 | CAM_PCLK | ADC2_CH2 | T13 |
| 19 | GPIO14 | — | ADC2_CH3 | T14 |
| 20 | **5V** | — | — | Power |

## Right Header (top → bottom)

| Pin | GPIO | Function | Other |
|-----|-------|----------|-------|
| 1 | GPIO43 | U0TXD | LED TX |
| 2 | GPIO44 | U0RXD | LED RX |
| 3 | GPIO1 | **free** | ADC1_CH0, T1 |
| 4 | GPIO2 | **free** | ADC1_CH1, T2, LED ON |
| 5 | GPIO42 | **free** | MTMS |
| 6 | GPIO41 | **free** | MTDI |
| 7 | GPIO40 | SD_DATA | MTDO |
| 8 | GPIO39 | SD_CLK | MTCK |
| 9 | GPIO38 | SD_CMD | — |
| 10 | GPIO37 | PSRAM | — |
| 11 | GPIO36 | PSRAM | — |
| 12 | GPIO35 | PSRAM | — |
| 13 | GPIO0 | Boot | strap |
| 14 | GPIO45 | VSPI | strap |
| 15 | GPIO48 | WS2812 | on-board LED |
| 16 | GPIO47 | **free** | — |
| 17 | GPIO21 | **free** | — |
| 18 | GPIO20 | USB D− | ADC2_CH9, U1CTS |
| 19 | GPIO19 | USB D+ | ADC2_CH8, U1RTS |
| 20 | **GND** | Ground | — |

## Available GPIOs

Not used by camera, PSRAM, USB, or UART0:

| GPIO | Notes | Good for |
|------|-------|----------|
| **1** | ADC1_CH0, touch | I2C SDA (ToF) |
| **2** | ADC1_CH1, touch, "LED ON" label | I2C SCL (ToF) |
| **3** | ADC, touch, JTAG_EN | digital I/O |
| **14** | ADC2_CH3, touch | PIR sensor |
| **21** | clean GPIO | digital I/O |
| **41** | JTAG MTDI | digital I/O |
| **42** | JTAG MTMS | digital I/O |
| **47** | clean GPIO | digital I/O |
| **38–40** | SD card pins | usable if no SD card |

## Peripheral Wiring

### TOF050C (VL53L0X I2C ToF sensor)

| TOF050C | ESP32 | Notes |
|---------|-------|-------|
| VIN | 3.3V | |
| GND | GND | |
| SDA | GPIO47 | Wire1 bus |
| SCL | GPIO21 | Wire1 bus |
| SHUT | 3.3V | tie high (always enabled) |
| INT | — | leave unconnected |

### PIR Motion Sensor (3-pin)

| PIR | ESP32 | Notes |
|-----|-------|-------|
| VCC | 3.3V (or 5V if module requires) | |
| GND | GND | |
| OUT | GPIO14 | HIGH on motion |
