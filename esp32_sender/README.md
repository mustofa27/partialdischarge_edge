# ESP32 MQTT Sender

This project receives data from the STM32 over UART and publishes it to the configured MQTT topic.

## Wiring

- STM32 PA9 (TX) -> ESP32 GPIO16 (RX)
- STM32 PA10 (RX) -> ESP32 GPIO17 (TX)
- GND -> GND
- 3.3V -> 3.3V

## Notes

- The ESP32 uses UART2:
  - RX = GPIO16
  - TX = GPIO17
- The STM32 should send newline-terminated strings like:
  - `TEMP=31.50`
  - `TEMP=32.10`
