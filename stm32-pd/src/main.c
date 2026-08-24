/*
 * main.c — pembacaan ADC 12-bit laju maksimum + keluaran ASCII ke USART2 (PA2)
 *
 * Target : STM32F407VGT6 pada board STM32F407G-DISC1
 * Sumber : ADC1, satu kanal, DMA sirkular, laju konversi 2,4 MSPS (maksimum
 *          yang dijamin datasheet untuk satu ADC pada F407)
 * Keluar : USART2 TX = PA2, 115200 8N1, angka desimal + "\r\n"
 *
 * ---------------------------------------------------------------------------
 * KENAPA ADA REDUKSI DI TENGAH (bagian terpenting file ini)
 * ---------------------------------------------------------------------------
 * ADC berjalan 2.400.000 sampel/detik. UART 115200 baud hanya sanggup 11.520
 * byte/detik; satu angka desimal + CRLF memakan ~7 byte, jadi jalur keluar
 * maksimum sekitar 1.600 angka/detik. Selisihnya ~1500x. Tidak ada pengaturan
 * apa pun yang bisa membuat 2,4 juta angka/detik lewat kabel ini.
 *
 * Jadi dua laju itu dipisah dan dijembatani:
 *
 *   ADC  ──2,4 MSPS──> buffer DMA ──reduksi tiap OUT_DECIM sampel──> UART
 *
 * Mode reduksi bawaan adalah PEAK: tiap angka yang dikirim adalah nilai
 * TERTINGGI dari jendela sampel di belakangnya. Untuk partial discharge ini
 * yang benar — pulsa PD berdurasi mikrodetik, dan justru karena ADC berlari
 * 2,4 MSPS pulsa itu tertangkap; peak-hold lalu meneruskan amplitudonya ke
 * jalur lambat tanpa hilang. Bandingkan dengan sekadar mengambil tiap sampel
 * ke-9836 (mode RAW), yang hampir pasti melewatkan pulsanya.
 *
 * ---------------------------------------------------------------------------
 * KECOCOKAN DENGAN SISI PENERIMA
 * ---------------------------------------------------------------------------
 * Format keluaran sengaja dibuat identik dengan yang lama ("912\r\n") supaya
 * esp-sender-wifi-bt.ino tidak perlu diubah: parser di sana mengambil digit
 * desimal dan memakai karakter non-digit apa pun sebagai pemisah.
 *
 * OUT_RATE_HZ bawaan 244 Hz dipilih menyamai laju firmware lama yang terukur
 * 244,25 sampel/detik. Itu penting: PD_BUFFER_SAMPLES = 300 di sketch ESP
 * disetel dari angka tersebut supaya publish MQTT tidak lebih rapat dari 1
 * detik (300/244 = 1,23 s). Menaikkan OUT_RATE_HZ tanpa menaikkan
 * PD_BUFFER_SAMPLES akan membuat ESP nge-publish jauh lebih sering.
 *
 * TIDAK ADA banner teks saat boot — sengaja. Teks apa pun yang mengandung
 * angka akan terbaca parser ESP sebagai sampel palsu.
 */

#include "types.h"

/* ==========================================================================
 *  1. Konfigurasi — hanya bagian ini yang biasanya perlu disentuh
 * ========================================================================== */

/* Kanal ADC1 dan pin PORT A yang dipakai. Keduanya harus cocok.
 * Pin ADC yang BEBAS di F407 Discovery (tidak dipakai periferal onboard):
 *   PA1 = IN1   <- bawaan, bersebelahan dengan PA2 di header P1
 *   PA3 = IN3   (bebas kalau USART2_RX tidak dipakai)
 *   PC4 = IN14, PC5 = IN15   (perlu ganti GPIOA -> GPIOC di adc_init)
 * Terpakai onboard, JANGAN dipakai: PA0 tombol B1, PA4 audio, PA5/6/7
 * akselerometer, PA9..PA12 USB, PA13/14/15 SWD. */
#define ADC_CHANNEL     1
#define ADC_PIN         1

/* Laju angka yang DIKIRIM ke UART. ADC tetap 2,4 MSPS berapa pun nilai ini. */
#define OUT_RATE_HZ     244

#define UART_BAUD       115200

/* Cara meringkas satu jendela sampel menjadi satu angka keluaran. */
#define REDUCE_PEAK     0   /* nilai tertinggi  — untuk PD (bawaan)          */
#define REDUCE_MEAN     1   /* rata-rata        — menekan derau, buta pulsa  */
#define REDUCE_RAW      2   /* sampel terakhir  — aliasing, untuk uji saja   */
#define REDUCE_MODE     REDUCE_PEAK

/* ==========================================================================
 *  2. Turunan — jangan diubah manual
 * ========================================================================== */

#define SYSCLK_HZ       144000000u   /* HSE 8 MHz * 288 / 8 / 2              */
#define APB1_HZ         36000000u    /* SYSCLK / 4  — USART2 ada di sini     */
#define APB2_HZ         72000000u    /* SYSCLK / 2                            */
#define ADCCLK_HZ       (APB2_HZ / 2)          /* 36 MHz = batas maksimum    */
#define ADC_SPS         (ADCCLK_HZ / 15u)      /* 12 bit + 3 siklus sampling */
#define OUT_DECIM       (ADC_SPS / OUT_RATE_HZ)

/* Buffer DMA sirkular. Diproses separuh-separuh (ping-pong): tiap 1024 sampel
 * pada 2,4 MSPS = 427 us, jadi ~2340 interupsi/detik. Ringan. */
#define ADC_BUF_LEN     2048u
#define ADC_HALF        (ADC_BUF_LEN / 2u)

#define TX_BUF_LEN      512u         /* pangkat dua, wajib                   */

/* ==========================================================================
 *  3. Register — didefinisikan langsung supaya tidak butuh CMSIS/HAL
 * ========================================================================== */

#define REG(a)          (*(volatile uint32_t *)(a))
#define REG16(a)        (*(volatile uint16_t *)(a))

#define RCC_BASE        0x40023800u
#define RCC_CR          REG(RCC_BASE + 0x00)
#define RCC_PLLCFGR     REG(RCC_BASE + 0x04)
#define RCC_CFGR        REG(RCC_BASE + 0x08)
#define RCC_AHB1ENR     REG(RCC_BASE + 0x30)
#define RCC_APB1ENR     REG(RCC_BASE + 0x40)
#define RCC_APB2ENR     REG(RCC_BASE + 0x44)

#define FLASH_ACR       REG(0x40023C00u)
#define PWR_CR          REG(0x40007000u)

#define GPIOA_BASE      0x40020000u
#define GPIOD_BASE      0x40020C00u
#define GPIO_MODER(p)   REG((p) + 0x00)
#define GPIO_OSPEEDR(p) REG((p) + 0x08)
#define GPIO_PUPDR(p)   REG((p) + 0x0C)
#define GPIO_ODR(p)     REG((p) + 0x14)
#define GPIO_AFRL(p)    REG((p) + 0x20)

#define ADC1_BASE       0x40012000u
#define ADC1_SR         REG(ADC1_BASE + 0x00)
#define ADC1_CR1        REG(ADC1_BASE + 0x04)
#define ADC1_CR2        REG(ADC1_BASE + 0x08)
#define ADC1_SMPR1      REG(ADC1_BASE + 0x0C)
#define ADC1_SMPR2      REG(ADC1_BASE + 0x10)
#define ADC1_SQR1       REG(ADC1_BASE + 0x2C)
#define ADC1_SQR3       REG(ADC1_BASE + 0x34)
#define ADC1_DR_ADDR    (ADC1_BASE + 0x4C)
#define ADC_CCR         REG(0x40012300u)

#define DMA2_BASE       0x40026400u
#define DMA2_LISR       REG(DMA2_BASE + 0x00)
#define DMA2_LIFCR      REG(DMA2_BASE + 0x08)
#define DMA2_S0CR       REG(DMA2_BASE + 0x10)
#define DMA2_S0NDTR     REG(DMA2_BASE + 0x14)
#define DMA2_S0PAR      REG(DMA2_BASE + 0x18)
#define DMA2_S0M0AR     REG(DMA2_BASE + 0x1C)
#define DMA2_S0FCR      REG(DMA2_BASE + 0x24)

#define USART2_BASE     0x40004400u
#define USART2_SR       REG(USART2_BASE + 0x00)
#define USART2_DR       REG(USART2_BASE + 0x04)
#define USART2_BRR      REG(USART2_BASE + 0x08)
#define USART2_CR1      REG(USART2_BASE + 0x0C)

#define NVIC_ISER1      REG(0xE000E104u)   /* IRQ 32..63 */

#define IRQ_USART2      38u
#define IRQ_DMA2_S0     56u

/* ==========================================================================
 *  4. Clock: HSE 8 MHz -> PLL -> SYSCLK 144 MHz
 *
 *  144 MHz dipilih, bukan 168 MHz yang lazim, justru DEMI laju ADC. ADCCLK
 *  maksimum F407 adalah 36 MHz, dan ia hanya bisa dibagi 2/4/6/8 dari APB2:
 *      SYSCLK 168 -> APB2 84 -> /4 = 21 MHz -> 1,40 MSPS
 *      SYSCLK 144 -> APB2 72 -> /2 = 36 MHz -> 2,40 MSPS  <- ini
 *  Jadi menurunkan clock inti justru menaikkan laju sampling 1,7x.
 * ========================================================================== */
static void clock_init(void)
{
    /* Latency harus dinaikkan SEBELUM clock naik. 144 MHz @3,3 V = 4 wait state. */
    FLASH_ACR = (1u << 10) | (1u << 9) | (1u << 8) | 4u;  /* DCEN ICEN PRFTEN */

    RCC_APB1ENR |= (1u << 28);          /* PWREN                              */
    PWR_CR      |= (1u << 14);          /* VOS = Scale 1                      */

    RCC_CR |= (1u << 16);               /* HSEON                              */
    while (!(RCC_CR & (1u << 17))) { }  /* HSERDY                             */

    /* M=8 (8->1 MHz), N=288 (VCO 288 MHz), P=2 (144 MHz), Q=6 (48 MHz), HSE */
    RCC_PLLCFGR = 8u | (288u << 6) | (0u << 16) | (1u << 22) | (6u << 24);

    RCC_CR |= (1u << 24);               /* PLLON                              */
    while (!(RCC_CR & (1u << 25))) { }  /* PLLRDY                             */

    /* AHB /1 = 144, APB1 /4 = 36 (maks 42), APB2 /2 = 72 (maks 84) */
    RCC_CFGR = (0u << 4) | (5u << 10) | (4u << 13);
    RCC_CFGR |= 2u;                     /* SW = PLL                           */
    while (((RCC_CFGR >> 2) & 3u) != 2u) { }
}

/* ==========================================================================
 *  5. Antrean kirim UART — ring buffer + interupsi TXE
 *
 *  Reduksi ADC berjalan di dalam ISR DMA yang berdenyut 2340x/detik. Ia tidak
 *  boleh menunggu UART. Jadi angka hasil reduksi hanya dijejalkan ke ring
 *  buffer, dan ISR USART2 yang menguras satu byte per TXE.
 * ========================================================================== */
static volatile uint8_t  tx_buf[TX_BUF_LEN];
static volatile uint16_t tx_head, tx_tail;
static volatile uint32_t tx_overflow;   /* berguna saat debug lewat debugger  */

static void tx_push(uint8_t b)
{
    uint16_t next = (uint16_t)((tx_head + 1u) & (TX_BUF_LEN - 1u));
    if (next == tx_tail) { tx_overflow++; return; }  /* penuh: buang, jangan blokir */
    tx_buf[tx_head] = b;
    tx_head = next;
}

/* Kirim satu angka desimal diakhiri CRLF. Tanpa div/mod berulang di jalur
 * panas: nilai ADC selalu 0..4095, jadi maksimum 4 digit. */
static void tx_number(uint16_t v)
{
    uint8_t d[5];
    uint8_t n = 0;

    if (v == 0) {
        d[n++] = '0';
    } else {
        while (v > 0) { d[n++] = (uint8_t)('0' + (v % 10u)); v /= 10u; }
    }
    while (n > 0) { tx_push(d[--n]); }
    tx_push('\r');
    tx_push('\n');

    USART2_CR1 |= (1u << 7);            /* TXEIE — bangunkan penguras         */
}

void USART2_IRQHandler(void)
{
    if (USART2_SR & (1u << 7)) {        /* TXE                                */
        if (tx_tail == tx_head) {
            USART2_CR1 &= ~(1u << 7);   /* kosong: matikan, jangan spin       */
        } else {
            USART2_DR = tx_buf[tx_tail];
            tx_tail = (uint16_t)((tx_tail + 1u) & (TX_BUF_LEN - 1u));
        }
    }
}

/* ==========================================================================
 *  6. Reduksi: 2,4 MSPS -> OUT_RATE_HZ
 * ========================================================================== */
static volatile uint16_t adc_buf[ADC_BUF_LEN];

static uint32_t acc_count;
static uint32_t acc_sum;
static uint16_t acc_peak;
static uint16_t acc_last;

/* Pencacah monoton, murni untuk diagnosa lewat debugger. acc_count dan
 * DMA_SxNDTR tidak bisa dipakai mengukur laju karena keduanya berulang
 * (tiap 4 ms dan 853 us); dua pencacah ini tidak, jadi selisihnya pada dua
 * waktu berbeda memberi laju yang sebenarnya.
 *   total_raw: sampel ADC terproses — berulang tiap ~30 menit @2,4 MSPS
 *   total_out: angka terkirim       — berulang tiap ~200 hari @244 Hz     */
static volatile uint32_t total_raw;
static volatile uint32_t total_out;

static void led_toggle(void)
{
    GPIO_ODR(GPIOD_BASE) ^= (1u << 12);  /* LED hijau: satu kedip per angka   */
}

/* Diproses per separuh buffer. Hitungan sampel dibuat EKSAK (emit bisa jatuh
 * di tengah blok) supaya OUT_RATE_HZ benar-benar tercapai, bukan dibulatkan
 * ke kelipatan ukuran blok. */
static void reduce_block(const volatile uint16_t *p, uint32_t n)
{
    uint32_t cnt  = acc_count;
    uint32_t sum  = acc_sum;
    uint16_t peak = acc_peak;
    uint16_t last = acc_last;
    uint32_t i;

    for (i = 0; i < n; i++) {
        uint16_t v = (uint16_t)(p[i] & 0x0FFFu);

        last = v;
        sum += v;
        if (v > peak) { peak = v; }
        cnt++;

        if (cnt >= OUT_DECIM) {
#if   REDUCE_MODE == REDUCE_PEAK
            tx_number(peak);
#elif REDUCE_MODE == REDUCE_MEAN
            tx_number((uint16_t)(sum / cnt));
#else
            tx_number(last);
#endif
            cnt = 0; sum = 0; peak = 0;
            total_out++;
            led_toggle();
        }
    }

    total_raw += n;
    acc_count = cnt;
    acc_sum   = sum;
    acc_peak  = peak;
    acc_last  = last;
}

void DMA2_Stream0_IRQHandler(void)
{
    uint32_t s = DMA2_LISR;

    if (s & (1u << 4)) {                /* HTIF0 — separuh awal siap          */
        DMA2_LIFCR = (1u << 4);
        reduce_block(&adc_buf[0], ADC_HALF);
    }
    if (s & (1u << 5)) {                /* TCIF0 — separuh akhir siap         */
        DMA2_LIFCR = (1u << 5);
        reduce_block(&adc_buf[ADC_HALF], ADC_HALF);
    }
    if (s & (1u << 3)) {                /* TEIF0 — galat transfer             */
        DMA2_LIFCR = (1u << 3);
    }
}

/* ==========================================================================
 *  7. Inisialisasi periferal
 * ========================================================================== */
static void uart_init(void)
{
    RCC_AHB1ENR |= (1u << 0);           /* GPIOA                              */
    RCC_APB1ENR |= (1u << 17);          /* USART2                             */

    /* PA2 -> alternate function 7 (USART2_TX) */
    GPIO_MODER(GPIOA_BASE)   &= ~(3u << (2u * 2u));
    GPIO_MODER(GPIOA_BASE)   |=  (2u << (2u * 2u));      /* AF                */
    GPIO_OSPEEDR(GPIOA_BASE) |=  (3u << (2u * 2u));      /* very high speed   */
    GPIO_AFRL(GPIOA_BASE)    &= ~(0xFu << (4u * 2u));
    GPIO_AFRL(GPIOA_BASE)    |=  (7u << (4u * 2u));      /* AF7               */

    USART2_BRR = (APB1_HZ + (UART_BAUD / 2u)) / UART_BAUD;
    USART2_CR1 = (1u << 13) | (1u << 3);                 /* UE | TE           */

    NVIC_ISER1 = (1u << (IRQ_USART2 - 32u));
}

static void led_init(void)
{
    RCC_AHB1ENR |= (1u << 3);                            /* GPIOD             */
    GPIO_MODER(GPIOD_BASE) &= ~(3u << (2u * 12u));
    GPIO_MODER(GPIOD_BASE) |=  (1u << (2u * 12u));       /* PD12 output       */
}

static void adc_init(void)
{
    RCC_AHB1ENR |= (1u << 0) | (1u << 22);   /* GPIOA, DMA2                   */
    RCC_APB2ENR |= (1u << 8);                /* ADC1                          */

    /* Pin masukan -> mode analog, tanpa pull */
    GPIO_MODER(GPIOA_BASE) |= (3u << (2u * ADC_PIN));
    GPIO_PUPDR(GPIOA_BASE) &= ~(3u << (2u * ADC_PIN));

    /* ---- DMA2 Stream0 Channel0 = ADC1, sirkular, 16-bit ---- */
    DMA2_S0CR = 0;
    while (DMA2_S0CR & 1u) { }
    DMA2_LIFCR  = 0x3Fu;                     /* bersihkan semua flag stream 0 */
    DMA2_S0PAR  = ADC1_DR_ADDR;
    DMA2_S0M0AR = (uint32_t)adc_buf;
    DMA2_S0NDTR = ADC_BUF_LEN;
    DMA2_S0FCR  = 0;                         /* direct mode                   */
    DMA2_S0CR   = (0u << 25)   /* CHSEL 0                                     */
                | (3u << 16)   /* PL very high                                */
                | (1u << 13)   /* MSIZE 16-bit                                */
                | (1u << 11)   /* PSIZE 16-bit                                */
                | (1u << 10)   /* MINC                                        */
                | (1u << 8)    /* CIRC                                        */
                | (0u << 6)    /* DIR periph -> mem                           */
                | (1u << 4)    /* TCIE                                        */
                | (1u << 3)    /* HTIE                                        */
                | (1u << 2);   /* TEIE                                        */
    DMA2_S0CR  |= 1u;                        /* EN                            */

    NVIC_ISER1 = (1u << (IRQ_DMA2_S0 - 32u));

    /* ---- ADC1 ---- */
    ADC_CCR    = (0u << 16);                 /* ADCPRE /2 -> 36 MHz           */
    ADC1_CR1   = 0;                          /* 12-bit, scan mati             */
    ADC1_SMPR1 = 0;
    ADC1_SMPR2 = 0;                          /* semua kanal 3 siklus          */
    ADC1_SQR1  = 0;                          /* L = 1 konversi                */
    ADC1_SQR3  = ADC_CHANNEL;

    ADC1_CR2 = (1u << 9)    /* DDS  — DMA terus jalan setelah transfer habis  */
             | (1u << 8)    /* DMA                                            */
             | (1u << 1)    /* CONT — konversi bersambung, laju maksimum      */
             | (1u << 0);   /* ADON                                           */

    /* tSTAB: ADC butuh ~3 us setelah ADON sebelum konversi pertama sah */
    for (volatile uint32_t i = 0; i < 2000u; i++) { }

    ADC1_CR2 |= (1u << 30);                  /* SWSTART                       */
}

/* ==========================================================================
 *  8. main
 * ========================================================================== */
int main(void)
{
    clock_init();
    led_init();
    uart_init();
    adc_init();

    /* Semua pekerjaan ada di ISR. Inti tidur di antara interupsi. */
    for (;;) {
        __asm volatile ("wfi");
    }
}
