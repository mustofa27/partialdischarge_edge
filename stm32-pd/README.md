# stm32-pd — ADC 2,4 MSPS → USART2 (PA2)

Firmware pengganti untuk **STM32F407G-DISC1** yang membaca ADC 12-bit pada laju
maksimum dan mengalirkan hasilnya sebagai angka desimal ASCII ke **PA2**, siap
dibaca `esp-sender-wifi-bt.ino` di GPIO18 LilyGO.

Dibangun bare-metal (tanpa HAL, CMSIS, atau libc) — hanya `arm-none-eabi-gcc`.

## Sambungan

| STM32F407G-DISC1 | LilyGO T-SIM7600G |
|---|---|
| **PA2** (USART2_TX, header P1) | **GPIO18** |
| **GND** | **GND** |

Sensor PD disambungkan ke **PA1** (ADC1_IN1), tegangan harus dalam 0–3,3 V.
Untuk memakai pin lain, ubah `ADC_CHANNEL` dan `ADC_PIN` di `src/main.c`.

Tidak perlu level shifter — keduanya 3,3 V.

Di sisi ESP, `DATA_FROM_USB` harus `0` (sudah begitu sekarang).

## Angka yang penting

| | |
|---|---|
| SYSCLK | 144 MHz |
| ADCCLK | 36 MHz (batas maksimum F407) |
| Laju ADC | **2.400.000 sampel/detik** — terukur 2.398.957 |
| Laju keluaran UART | **244,0 angka/detik** — terukur 244,0 |
| Rasio desimasi | 9836 sampel per angka |
| Format | desimal ASCII 0–4095 + `\r\n` |
| Baud | 115200 8N1 |
| Beban CPU | ~17 % (ISR reduksi) |
| Ukuran | 1392 B flash, 4636 B RAM |

### Kenapa 144 MHz, bukan 168 MHz

ADCCLK maksimum F407 adalah 36 MHz, dan pembaginya hanya /2, /4, /6, /8 dari APB2:

- SYSCLK 168 → APB2 84 → /4 = 21 MHz → **1,40 MSPS**
- SYSCLK 144 → APB2 72 → /2 = 36 MHz → **2,40 MSPS**

Menurunkan clock inti justru menaikkan laju sampling 1,7×.

### Kenapa ada reduksi peak-hold

ADC menghasilkan 2,4 juta angka/detik; UART 115200 hanya sanggup ~1.600. Selisih
~1500×, dan tidak ada pengaturan yang bisa menghilangkannya.

Jadi tiap 9836 sampel diringkas jadi satu angka berisi **nilai tertinggi** jendela
itu. Untuk partial discharge inilah yang benar: pulsa PD berdurasi mikrodetik
tertangkap justru karena ADC berlari 2,4 MSPS, lalu amplitudonya diteruskan utuh
ke jalur lambat. Mode `REDUCE_MEAN` dan `REDUCE_RAW` tersedia di `src/main.c`.

### Kenapa 244 Hz

Menyamai firmware lama yang terukur 244,25 sampel/detik. `PD_BUFFER_SAMPLES = 300`
di sketch ESP disetel dari angka itu agar publish MQTT tidak lebih rapat dari
1 detik (300 ÷ 244 = 1,23 s). **Menaikkan `OUT_RATE_HZ` tanpa menaikkan
`PD_BUFFER_SAMPLES` akan membuat ESP nge-publish jauh lebih sering.**

Batas atasnya: UART ~1.600 angka/detik, dan jalur BLE hanya ~1.000–2.000
sampel/detik. Jadi ~1.500 Hz adalah langit-langit rantai ini, bukan 2,4 juta.

## Perintah

```sh
make            # build -> build/pd-adc-uart.bin
make flash      # tulis ke board lewat ST-LINK
make backup     # cadangkan isi flash board ke backup/firmware-lama.bin
make restore    # kembalikan firmware PD yang lama
make clean
```

Butuh `arm-none-eabi-gcc` dan `stlink` (`brew install arm-none-eabi-gcc stlink`).

## Firmware lama

`backup/firmware-lama.bin` — 1 MB utuh, dibaca dua kali dan byte-identik.

```
sha256  c724e9e133fe701cbf8639c42c76cfc7f904fd8999f21e7c60697968ef83784b
```

Kembalikan kapan saja dengan `make restore`.

## Catatan

- **Tanpa banner teks saat boot**, sengaja: parser ESP mengambil digit desimal
  apa pun, jadi teks yang mengandung angka akan terbaca sebagai sampel palsu.
- **LED hijau PD12** berkedip sekali per angka terkirim. Pada 244 Hz mata melihatnya
  menyala redup terus-menerus, bukan berkedip — itu tanda normal.
- **VCP ST-LINK (`/dev/cu.usbmodem*`) tidak tersambung ke PA2** pada board DISC1,
  jadi keluaran tidak bisa dipantau dari kabel ST-LINK. Pakai USB-TTL ke PA2, atau
  langsung lihat penghitung `rx` di log ESP.
- `total_raw` dan `total_out` di RAM adalah pencacah monoton untuk mengukur laju
  lewat debugger. `acc_count` dan `DMA_SxNDTR` **tidak bisa** dipakai untuk itu —
  keduanya berulang tiap 4 ms dan 853 µs.
