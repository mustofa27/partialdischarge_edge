/*
 * PD Sender WiFi+BT — LilyGO T-SIM7600G (ESP32-WROVER-E)
 *
 * Gabungan dari esp-sender-bt.ino: data yang sama dikirim ke DUA tujuan
 * sekaligus, dan keduanya berjalan mandiri.
 *
 * Alur:
 *   1. Angka desimal masuk lewat USB-C (UART0)  -> "520\n"
 *   2. Ditampung di buffer 100 sampel
 *   3. Begitu penuh, dibungkus SEKALI jadi {"data":[520,600,...]} lalu:
 *        - dikirim lewat BLE  (Nordic UART Service, dipecah per MTU)
 *        - dikirim lewat MQTT (WiFi + TLS, satu publish)
 *      Frame baru dilepas setelah kedua jalur selesai. Kalau salah satu
 *      sedang putus, jalur yang lain TIDAK ikut tertahan.
 *   4. SSID/password WiFi diatur lewat web server kecil di board ini
 *      (tanpa WiFiManager), tersimpan di NVS.
 *
 * Cara mengatur WiFi:
 *   Saat belum ada kredensial tersimpan — atau setelah 3 kali gagal
 *   menyambung — board menyalakan access point sendiri:
 *       SSID     : PD-Sender-Setup
 *       Password : pdsender123
 *   Sambungkan HP/laptop ke AP itu; halaman pengaturan biasanya terbuka
 *   sendiri (captive portal). Kalau tidak, buka http://192.168.4.1
 *   Setelah tersambung ke WiFi rumah/kantor, halaman yang sama bisa dibuka
 *   lewat alamat IP board di jaringan itu (lihat log atau layar router).
 *
 * PENTING — WiFi dan BLE berbagi satu radio 2,4 GHz dan satu heap:
 *   - Throughput keduanya turun dibanding kalau hanya salah satu yang aktif.
 *     Untuk data 100 sampel tiap beberapa ratus milidetik ini tidak masalah.
 *   - Heap jadi ketat: stack BLE + WiFi + TLS mbedTLS semuanya mengambil RAM.
 *     Baris [STAT] menampilkan sisa heap — kalau turun mendekati ~20 KB,
 *     handshake TLS bisa gagal. Matikan salah satu jalur lewat ENABLE_BLE /
 *     ENABLE_MQTT di bawah kalau itu terjadi.
 *
 * Pustaka yang harus dipasang lewat Library Manager:
 *   - PubSubClient   by Nick O'Leary  (>= 2.8)
 *   - NimBLE-Arduino by h2zero        (>= 2.5)
 *   (WebServer, DNSServer, Preferences sudah bawaan core ESP32.)
 *
 * BLE memakai NimBLE, bukan Bluedroid bawaan core — lihat catatan di dekat
 * #include, ini keharusan memori supaya TLS bisa jalan berbarengan.
 *
 * Setting Arduino IDE (sudah diukur langsung di board ini):
 *   Board       : "ESP32 Dev Module"
 *   Flash Size  : 16MB (128Mb)
 *   PSRAM       : Enabled
 *   Partition   : "Huge APP (3MB No OTA/1MB SPIFFS)"   <-- WAJIB, sketch besar
 *   Upload Speed: 115200        <-- JANGAN 921600, board ini gagal di baud tinggi
 *
 * File ini harus berada di folder "esp-sender-wifi-bt".
 */

// ===========================================================================
//  1. Jalur mana yang diaktifkan  (harus di atas #include)
// ===========================================================================
// Kalau heap masih terlalu ketat, matikan salah satu (set ke 0) — sisa sketch
// tetap jalan tanpa perubahan lain.
#define ENABLE_BLE  1
#define ENABLE_MQTT 1

#include <Arduino.h>
#include <Ticker.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <esp_heap_caps.h>

// BLE memakai NimBLE, BUKAN Bluedroid bawaan core.
//
// Alasannya bukan selera: Bluedroid menyisakan heap ~46 KB saat WiFi juga
// hidup, sedangkan satu handshake TLS ke broker perlu 2 x 16 KB buffer isi
// plus alokasi untuk mem-parsing rantai 4 sertifikat Let's Encrypt milik
// broker. Hasilnya mbedTLS gagal dengan "X509 - Allocation of memory failed".
// NimBLE mengerjakan peran yang sama dengan heap jauh lebih hemat.
//
// Pustaka: "NimBLE-Arduino" by h2zero (>= 2.5) lewat Library Manager.
#if ENABLE_BLE
  #include <NimBLEDevice.h>
#endif

// ===========================================================================
//  2. Pin map LilyGO T-SIM7600G
// ===========================================================================
#define BOARD_LED_PIN     12      // LED onboard (GPIO12 strapping pin, jangan di-pull-up)
#define BOARD_LED_ACTIVE_HIGH 1

// Modul ESP32-WROVER memakai GPIO16 & GPIO17 untuk PSRAM, jadi Serial2
// TIDAK boleh memakai pin default 16/17. Di-remap ke 18/19.
#define UART2_RX_PIN      18
#define UART2_TX_PIN      19
#define LED_BLUE_PIN      22      // status WiFi
#define LED_RED_PIN       21      // status MQTT

// ===========================================================================
//  3. Konfigurasi
// ===========================================================================
// Sumber data masuk: 1 = Serial (USB-C/UART0, pengirim PC), 0 = Serial2 (pin)
#define DATA_FROM_USB 0

// Log status ke pin Serial2 (butuh USB-TTL kedua). 0 = mati.
// Log selalu tersedia lewat BLE dan lewat halaman web, jadi ini jarang perlu.
#define LOG_TARGET 0

#if DATA_FROM_USB
  #define SerialData Serial
#else
  #define SerialData Serial2
#endif
#if !DATA_FROM_USB && LOG_TARGET == 2
  #error "LOG_TARGET 2 bentrok: Serial2 sudah dipakai untuk data."
#endif

// --- MQTT (identik dengan esp-sender.ino) ---
static constexpr const char* MQTT_HOST      = "mqtt.icminovasi.my.id";
static constexpr const char* MQTT_USERNAME  = "partial_discharge";
static constexpr const char* MQTT_PASSWORD  = "PartialDischarge@2026";
static constexpr const char* MQTT_DEVICE_ID = "bnd-9bf3";
static constexpr const char* MQTT_TOPIC     = "partial_discharge/bnd-9bf3/pd_signal";
#define WIFI_USE_TLS 1
static constexpr uint16_t MQTT_PORT_TLS   = 8883;
static constexpr uint16_t MQTT_PORT_PLAIN = 1883;
static constexpr uint16_t MQTT_PORT = WIFI_USE_TLS ? MQTT_PORT_TLS : MQTT_PORT_PLAIN;

// --- Access point untuk pengaturan ---
static constexpr const char* AP_SSID = "PD-Sender-Setup";
static constexpr const char* AP_PASS = "pdsender123";     // min 8 karakter
static constexpr uint8_t     WIFI_FAILS_BEFORE_AP = 3;
static constexpr uint32_t    WIFI_CONNECT_MS = 15000;     // tunggu saat boot
static constexpr uint32_t    WIFI_RETRY_MS   = 15000;     // jeda antar percobaan
// Jeda percobaan STA saat AP pengaturan sedang menyala. Dibuat panjang supaya
// AP tidak terus terganggu perpindahan kanal dan tetap terlihat di HP.
static constexpr uint32_t    WIFI_RETRY_AP_MS = 60000;

// --- Buffer sampel ---
typedef uint16_t pd_sample_t;
static constexpr uint32_t PD_SAMPLE_MAX     = 65535;
// Ukuran buffer dipilih dari laju sumber yang diukur langsung dari STM32:
// 244,25 sampel/detik rata-rata, dengan jitter 240..250 per detik.
//
// Syaratnya: publish MQTT tidak boleh lebih rapat dari 1 detik (server berat).
//   N=250 -> 1,00 detik pada detik tercepat (250/detik) = TEPAT di batas, ditolak
//   N=300 -> 1,20 detik pada detik tercepat, 1,23 detik rata-rata  <- dipakai
// Konsekuensi lain di N=300: payload 1.210 B (data 3 digit) atau 1.810 B
// (kalau ada pulsa 5 digit), RAM total ~4,3 KB, overhead header MQTT 3,6%.
static constexpr uint32_t PD_BUFFER_SAMPLES = 300;

// Jaring pengaman kalau sumber data suatu saat dipercepat: MQTT tidak akan
// pernah publish lebih rapat dari ini, berapa pun cepatnya buffer terisi.
// Frame yang belum boleh dikirim hanya menunggu — datanya tidak dibuang.
static constexpr uint32_t MQTT_MIN_INTERVAL_MS = 1000;

// Batas waktu sebuah frame boleh menggantung. Tanpa ini, satu tujuan yang macet
// akan menghentikan penyerapan UART selamanya dan ring buffer meluap diam-diam.
static constexpr uint32_t FRAME_TIMEOUT_MS = 5000;

// {"data":[ + 100 angka 5 digit + 99 koma + ]} + '\n' + NUL + kelonggaran
static constexpr size_t OUT_CAP =
    9 + PD_BUFFER_SAMPLES * 5 + (PD_BUFFER_SAMPLES - 1) + 3 + 1 + 8;

// --- Serial data masuk ---
static constexpr uint32_t DATA_BAUD   = 115200;
static constexpr size_t   DATA_RX_BUF = 16384;
static constexpr uint32_t LOG_BAUD    = 115200;

// --- BLE ---
static constexpr const char* BLE_DEVICE_NAME = "PD-Sender-BT";
static constexpr size_t   BLE_MAX_CHUNK = 244;
static constexpr uint32_t BLE_TX_GAP_MS = 5;
#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // DATA
#define NUS_LOG_UUID     "6E4000FF-B5A3-F393-E0A9-E50E24DCCA9E"  // LOG

// ===========================================================================
//  4. State global
// ===========================================================================
static Preferences prefs;
static WebServer   web(80);
static DNSServer   dns;
static String      g_ssid, g_pass;
static bool        g_apMode   = false;
static uint8_t     g_wifiFail = 0;

#if WIFI_USE_TLS
static WiFiClientSecure netClient;
#else
static WiFiClient       netClient;
#endif
static PubSubClient mqtt(netClient);

#if ENABLE_BLE
// NimBLE membuat descriptor CCCD (0x2902) sendiri, jadi tidak ada objek BLE2902
// untuk ditanya. Status subscribe justru datang lewat callback onSubscribe —
// lebih tepat daripada membaca nilai descriptor, karena kita diberi tahu tepat
// saat client menyalakan atau mematikan notification.
static NimBLEServer*         g_server  = nullptr;
static NimBLECharacteristic* g_chData  = nullptr;
static NimBLECharacteristic* g_chLog   = nullptr;
static volatile bool         g_bleConn = false;
static volatile bool         g_dataSub = false;   // client subscribe ke DATA
static volatile bool         g_logSub  = false;   // client subscribe ke LOG
static volatile uint16_t     g_mtu     = 23;      // sampai dinegosiasikan ulang
#endif

static pd_sample_t g_buf[PD_BUFFER_SAMPLES];
static uint32_t    g_len = 0;

// Satu frame teks, dipakai bersama oleh kedua jalur.
static char   g_out[OUT_CAP];
static size_t g_jsonLen = 0;   // panjang JSON saja (untuk MQTT)
static size_t g_outLen  = 0;   // JSON + '\n' (untuk BLE); 0 = slot kosong
static size_t g_bleSent = 0;
static bool   g_bleWant = false, g_mqttWant = false, g_mqttDone = false;

static uint32_t g_frameAt = 0;      // kapan frame yang sedang terbang dibuat
static uint32_t g_stuck   = 0;      // frame yang dipaksa pensiun karena macet
static uint32_t g_frames = 0, g_dropped = 0;
static uint32_t g_bleFrames = 0, g_mqttFrames = 0, g_mqttFail = 0;
static uint32_t g_totalSamples = 0;

// ===========================================================================
//  5. Log — satu sumber, tiga muara (pin serial, BLE, halaman web)
// ===========================================================================
static constexpr size_t LOG_RING = 2048;
static char   g_ring[LOG_RING];
static size_t g_ringHead = 0, g_ringLen = 0;

static char   g_logLine[BLE_MAX_CHUNK];
static size_t g_logFill = 0;

static bool logBleReady() {
#if ENABLE_BLE
  return g_bleConn && g_chLog && g_logSub;
#else
  return false;
#endif
}

class LogSink : public Print {
 public:
  size_t write(uint8_t c) override {
#if LOG_TARGET == 2
    Serial2.write(c);
#endif
    // ring untuk halaman web (menyimpan riwayat sejak boot)
    g_ring[g_ringHead] = (char)c;
    g_ringHead = (g_ringHead + 1) % LOG_RING;
    if (g_ringLen < LOG_RING) g_ringLen++;

#if ENABLE_BLE
    if (c != '\r') {
      if (g_logFill < sizeof(g_logLine)) g_logLine[g_logFill++] = (char)c;
      if (c == '\n' || g_logFill == sizeof(g_logLine)) {
        if (logBleReady()) g_chLog->notify((const uint8_t*)g_logLine, g_logFill);
        g_logFill = 0;
      }
    }
#endif
    return 1;
  }
  size_t write(const uint8_t* b, size_t n) override {
    for (size_t i = 0; i < n; i++) write(b[i]);
    return n;
  }
};
static LogSink logSink;
#define SerialLog logSink

// ===========================================================================
//  5b. Reset kredensial WiFi tanpa program ulang
// ===========================================================================
// Tombol "Lupakan WiFi" di halaman web tidak menolong justru saat paling
// dibutuhkan: kalau jaringan tersimpan hilang, halaman itu tidak terjangkau.
// Karena itu ada dua jalan keluar yang tidak butuh komputer sama sekali:
//
//   1. Tekan tombol RST dua kali, jeda kurang dari 6 detik.
//   2. Kirim teks "FORGET" (atau "RESET") ke karakteristik BLE 6E400002
//      dari nRF Connect / LightBlue.
//
// Cara 1 memakai RTC memory, yang isinya bertahan saat reset tombol tapi
// hilang saat listrik benar-benar putus — persis sifat yang dibutuhkan untuk
// membedakan "ditekan dua kali" dari "baru dinyalakan". Dua kata dipakai
// (nilai + komplemennya) karena RTC memory berisi sampah acak saat cold boot,
// dan sampah itu bisa saja kebetulan sama dengan satu nilai penanda.
static constexpr uint32_t DRD_WINDOW_MS = 6000;
#define DRD_MAGIC 0xD00DF00Du
RTC_NOINIT_ATTR static uint32_t g_drdA;
RTC_NOINIT_ATTR static uint32_t g_drdB;

static volatile bool g_forgetReq = false;   // diminta lewat BLE, dieksekusi di loop()

static void credsClear(const char* why) {
  prefs.begin("pdsender", false);
  prefs.clear();
  prefs.end();
  g_ssid = "";
  g_pass = "";
  SerialLog.printf("[CFG ] kredensial WiFi DIHAPUS (%s)\n", why);
}

static String ringText() {
  String s;
  s.reserve(g_ringLen + 1);
  size_t tail = (g_ringHead + LOG_RING - g_ringLen) % LOG_RING;
  for (size_t i = 0; i < g_ringLen; i++) s += g_ring[(tail + i) % LOG_RING];
  return s;
}

// ===========================================================================
//  6. LED
// ===========================================================================
//   biru    : kedip = WiFi belum tersambung, nyala tetap = tersambung
//   merah   : kedip = MQTT belum tersambung, nyala tetap = tersambung
//   onboard : kedip cepat selama ADA data serial masuk, padam kalau sepi
static Ticker            ledTicker;
static volatile bool     g_wifiUp = false, g_mqttUp = false;

static void ledTick() {
  static bool phase = false;
  phase = !phase;
  digitalWrite(LED_BLUE_PIN, g_wifiUp ? HIGH : (phase ? HIGH : LOW));
  digitalWrite(LED_RED_PIN,  g_mqttUp ? HIGH : (phase ? HIGH : LOW));
}

// --- LED onboard = ada tidaknya data serial masuk -------------------------
// Sengaja TIDAK dikedipkan per byte: pada 1.220 B/detik hasilnya akan terlihat
// menyala terus, tidak bisa dibedakan dari lampu mati-hidup biasa. Yang
// dilakukan: tiap RX_BLINK_MS diperiksa apakah ADA byte masuk sejak periksa
// terakhir — kalau ada, LED dibalik keadaannya; kalau sepi, LED dimatikan.
// Hasilnya kedipan mantap ~6 kali/detik selama data mengalir, dan padam total
// begitu aliran berhenti.
static constexpr uint32_t RX_BLINK_MS = 80;
static uint32_t g_rxSinceBlink = 0;     // byte masuk sejak pemeriksaan terakhir
static uint32_t g_rxTotal      = 0;     // total byte masuk sejak boot
static bool     g_rxLedOn      = false;

static void boardLed(bool on) {
#if BOARD_LED_ACTIVE_HIGH
  digitalWrite(BOARD_LED_PIN, on ? HIGH : LOW);
#else
  digitalWrite(BOARD_LED_PIN, on ? LOW : HIGH);
#endif
}

static void rxLedService() {
  static uint32_t last = 0;
  if (millis() - last < RX_BLINK_MS) return;
  last = millis();

  if (g_rxSinceBlink) {
    g_rxLedOn = !g_rxLedOn;     // ada aliran -> kedip
    g_rxSinceBlink = 0;
  } else {
    g_rxLedOn = false;          // sepi -> padam
  }
  boardLed(g_rxLedOn);
}

// ===========================================================================
//  7. Buffer -> frame teks
// ===========================================================================
static bool bleDataReady() {
#if ENABLE_BLE
  return g_bleConn && g_chData && g_dataSub;
#else
  return false;
#endif
}
static bool mqttReady() {
#if ENABLE_MQTT
  return mqtt.connected();
#else
  return false;
#endif
}

// Bungkus isi g_buf jadi {"data":[...]}\n. JSON tanpa '\n' dikirim ke MQTT,
// versi dengan '\n' dikirim ke BLE (pemisah frame untuk penerima).
static void encodeFrame() {
  size_t p = 0;
  p += (size_t)snprintf(g_out + p, sizeof(g_out) - p, "{\"data\":[");
  for (uint32_t i = 0; i < g_len; i++) {
    p += (size_t)snprintf(g_out + p, sizeof(g_out) - p,
                          i ? ",%u" : "%u", (unsigned)g_buf[i]);
  }
  p += (size_t)snprintf(g_out + p, sizeof(g_out) - p, "]}");
  g_jsonLen = p;
  g_out[p++] = '\n';
  g_out[p]   = '\0';
  g_outLen   = p;

  g_bleSent  = 0;
  g_bleWant  = bleDataReady();
  g_mqttWant = mqttReady();
  g_mqttDone = false;
  g_frameAt  = millis();
  g_len      = 0;
  g_frames++;
  g_totalSamples += PD_BUFFER_SAMPLES;
}

static void pushSample(pd_sample_t v) {
  if (g_len < PD_BUFFER_SAMPLES) g_buf[g_len++] = v;
  if (g_len < PD_BUFFER_SAMPLES) return;

  // Buffer penuh. Frame baru hanya dibuat kalau slot kirim kosong DAN ada
  // minimal satu tujuan yang siap menerima. Kalau tidak, isi buffer dibuang —
  // lebih baik kehilangan data lama daripada mengirim data basi nanti.
  if (g_outLen == 0 && (bleDataReady() || mqttReady())) {
    encodeFrame();
  } else {
    g_dropped += g_len;
    g_len = 0;
  }
}

// Parser stream: ambil angka desimal, pemisah apa pun (\n, \r, koma, spasi)
static void drainSerialData() {
  static uint32_t acc = 0;
  static bool     has = false;
  while (SerialData.available()) {
    int c = SerialData.read();
    g_rxSinceBlink++;                 // menggerakkan LED onboard
    g_rxTotal++;
    if (c >= '0' && c <= '9') {
      acc = acc * 10 + (uint32_t)(c - '0');
      if (acc > PD_SAMPLE_MAX) acc = PD_SAMPLE_MAX;
      has = true;
    } else {
      if (has) pushSample((pd_sample_t)acc);
      acc = 0;
      has = false;
    }
  }
}

// Frame dilepas hanya setelah SEMUA tujuan yang diminati selesai. Tujuan yang
// putus di tengah jalan ditandai selesai supaya tidak menahan yang lain.
static void retireIfDone() {
  if (g_outLen == 0) return;
  bool bleOk  = !g_bleWant  || g_bleSent >= g_outLen;
  bool mqttOk = !g_mqttWant || g_mqttDone;

  // Katup pengaman: selama frame menggantung, penyerapan UART berhenti. Kalau
  // sebuah tujuan macet, frame harus tetap dilepas — kehilangan satu frame yang
  // tercatat jauh lebih baik daripada ring buffer UART meluap tanpa jejak.
  bool timedOut = (millis() - g_frameAt) > FRAME_TIMEOUT_MS;
  if (timedOut && !(bleOk && mqttOk)) {
    g_stuck++;
    g_dropped += PD_BUFFER_SAMPLES;
    SerialLog.printf("[FRAME] macet >%lu ms (ble=%d mqtt=%d), dipaksa lepas\n",
                     (unsigned long)FRAME_TIMEOUT_MS, (int)bleOk, (int)mqttOk);
  }

  if ((bleOk && mqttOk) || timedOut) {
    g_outLen = g_jsonLen = g_bleSent = 0;
    g_bleWant = g_mqttWant = g_mqttDone = false;
  }
}

// ===========================================================================
//  8. Jalur BLE
// ===========================================================================
#if ENABLE_BLE
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
    g_bleConn = true;
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    g_bleConn = false;
    g_dataSub = false;
    g_logSub  = false;
    g_mtu     = 23;
    // advertiseOnDisconnect(true) sudah menyalakan advertising lagi sendiri.
  }
  void onMTUChange(uint16_t mtu, NimBLEConnInfo&) override {
    g_mtu = mtu;
  }
};

// subValue bit 0 = notification, bit 1 = indication.
class DataSubCallbacks : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic*, NimBLEConnInfo&, uint16_t subValue) override {
    g_dataSub = (subValue & 0x0001) != 0;
  }
};
class LogSubCallbacks : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic*, NimBLEConnInfo&, uint16_t subValue) override {
    g_logSub = (subValue & 0x0001) != 0;
  }
};

// Perintah dari HP lewat karakteristik RX. Hanya menaikkan bendera — kerja
// beratnya (hapus NVS + restart) dilakukan loop(), supaya tidak memanggil
// balik stack BLE dari dalam callback-nya sendiri.
class RxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
    std::string v = c->getValue();
    for (auto& ch : v) ch = (char)toupper((unsigned char)ch);
    if (v.find("FORGET") != std::string::npos || v.find("RESET") != std::string::npos) {
      g_forgetReq = true;
    }
  }
};

static void bleBegin() {
  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setMTU(247);

  g_server = NimBLEDevice::createServer();
  g_server->setCallbacks(new ServerCallbacks());
  g_server->advertiseOnDisconnect(true);

  NimBLEService* svc = g_server->createService(NUS_SERVICE_UUID);

  g_chData = svc->createCharacteristic(NUS_TX_UUID, NIMBLE_PROPERTY::NOTIFY);
  g_chData->setCallbacks(new DataSubCallbacks());

  g_chLog = svc->createCharacteristic(NUS_LOG_UUID, NIMBLE_PROPERTY::NOTIFY);
  g_chLog->setCallbacks(new LogSubCallbacks());

  // RX dipakai untuk satu hal: perintah menghapus kredensial WiFi dari HP,
  // saat halaman web sudah tidak terjangkau.
  NimBLECharacteristic* rx =
      svc->createCharacteristic(NUS_RX_UUID,
                                NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rx->setCallbacks(new RxCallbacks());
  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  adv->enableScanResponse(true);
  NimBLEDevice::startAdvertising();
}

// NimBLE menyalakan advertising sendiri setelah client putus; yang tersisa
// hanya melepas frame yang tertahan supaya jalur MQTT tidak ikut menunggu.
static void bleServiceAdv() {
  if (!g_bleConn && g_bleWant) g_bleWant = false;
}

// Satu notify per pemanggilan supaya loop() tidak pernah terblokir.
static void bleTxPump() {
  if (g_outLen == 0 || !g_bleWant || g_bleSent >= g_outLen) return;
  if (!bleDataReady()) { g_bleWant = false; return; }

  static uint32_t lastTx = 0;
  if (millis() - lastTx < BLE_TX_GAP_MS) return;
  lastTx = millis();

  size_t chunk = (g_mtu > 23) ? (size_t)(g_mtu - 3) : 20;
  if (chunk > BLE_MAX_CHUNK) chunk = BLE_MAX_CHUNK;

  size_t n = g_outLen - g_bleSent;
  if (n > chunk) n = chunk;
  g_chData->notify((const uint8_t*)g_out + g_bleSent, n);
  g_bleSent += n;
  if (g_bleSent >= g_outLen) g_bleFrames++;
}
#else
static void bleBegin() {}
static void bleServiceAdv() {}
static void bleTxPump() {}
#endif

// ===========================================================================
//  9. Jalur MQTT
// ===========================================================================
#if ENABLE_MQTT
// rc=-2 (MQTT_CONNECT_FAILED) hanya berarti "koneksi di bawah MQTT gagal" —
// belum sampai ke pemeriksaan user/password. Penyebabnya bisa tiga hal yang
// sangat berbeda, jadi ketiganya diuji terpisah supaya tidak menebak:
//   1. DNS tidak bisa menerjemahkan nama broker
//   2. TCP ke port-nya ditolak/diblokir jaringan
//   3. handshake TLS gagal (paling sering: kehabisan memori)
static void mqttDiagnose() {
  SerialLog.println("[DIAG] --- menelusuri penyebab gagal connect ---");

  size_t heapAll  = ESP.getFreeHeap();
  size_t biggest  = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  SerialLog.printf("[DIAG] heap %u B, blok internal terbesar %u B, PSRAM %u B\n",
                   (unsigned)heapAll, (unsigned)biggest, (unsigned)ESP.getFreePsram());
  // Satu sesi TLS meminta 2 x 16 KB (MBEDTLS_SSL_MAX_CONTENT_LEN=16384).
  if (biggest < 20000) {
    SerialLog.println("[DIAG] blok bersambung < 20 KB — handshake TLS kemungkinan besar "
                      "gagal karena memori, bukan karena jaringan.");
  }

  IPAddress ip;
  if (!WiFi.hostByName(MQTT_HOST, ip)) {
    SerialLog.printf("[DIAG] DNS GAGAL untuk %s — cek koneksi internet WiFi ini\n", MQTT_HOST);
    return;
  }
  SerialLog.printf("[DIAG] DNS ok: %s -> %s\n", MQTT_HOST, ip.toString().c_str());

  WiFiClient probe;                       // TCP polos, tanpa TLS
  probe.setTimeout(8);
  if (probe.connect(ip, MQTT_PORT)) {
    SerialLog.printf("[DIAG] TCP ke port %u ok — jaringan tidak memblokir. "
                     "Berarti yang gagal adalah TLS.\n", MQTT_PORT);
    probe.stop();
  } else {
    SerialLog.printf("[DIAG] TCP ke port %u DITOLAK — port diblokir jaringan/firewall, "
                     "atau broker tidak mendengarkan di situ.\n", MQTT_PORT);
  }
  SerialLog.println("[DIAG] ---------------------------------------");
}

static bool mqttEnsure() {
  if (WiFi.status() != WL_CONNECTED) return false;
  if (mqtt.connected()) return true;

  static uint32_t lastTry = 0;
  if (millis() - lastTry < 5000) return false;
  lastTry = millis();

  SerialLog.printf("[MQTT] connect %s:%u (heap %u) ...\n",
                   MQTT_HOST, MQTT_PORT, (unsigned)ESP.getFreeHeap());
  if (mqtt.connect(MQTT_DEVICE_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
    SerialLog.println("[MQTT] tersambung");
    return true;
  }

  int rc = mqtt.state();
#if WIFI_USE_TLS
  char tls[128] = { 0 };
  int  te = netClient.lastError(tls, sizeof(tls));
  SerialLog.printf("[MQTT] gagal rc=%d | TLS err=%d %s\n", rc, te, tls[0] ? tls : "(tidak ada)");
#else
  SerialLog.printf("[MQTT] gagal rc=%d\n", rc);
#endif

  // Telusuri saat kegagalan pertama, lalu sesekali saja supaya tidak berisik.
  static uint8_t fails = 0;
  if (fails < 200) fails++;
  if (fails == 1 || fails % 10 == 0) mqttDiagnose();
  return false;
}

// Satu frame = satu publish; payload ~600 byte jadi tidak perlu streaming.
static void mqttPump() {
  if (g_outLen == 0 || !g_mqttWant || g_mqttDone) return;
  if (!mqtt.connected()) { g_mqttWant = false; return; }

  // Jaga jarak minimum antar publish. Frame tidak dibuang, hanya menunggu —
  // data yang masuk selama menunggu tertampung di ring buffer UART.
  static uint32_t lastPub = 0;
  if (lastPub && millis() - lastPub < MQTT_MIN_INTERVAL_MS) return;

  if (mqtt.publish(MQTT_TOPIC, (const uint8_t*)g_out, g_jsonLen, false)) {
    lastPub = millis();
    g_mqttDone = true;
    g_mqttFrames++;
  } else {
    g_mqttFail++;
    g_mqttWant = false;                 // jangan tahan frame, coba lagi frame berikutnya
    SerialLog.println("[MQTT] publish gagal");
  }
}
#else
static bool mqttEnsure() { return false; }
static void mqttPump() {}
#endif

// ===========================================================================
// 10. Halaman web pengaturan
// ===========================================================================
static const char PAGE_HTML[] PROGMEM = R"HTML(<!doctype html><html lang="id"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>PD Sender</title><style>
:root{color-scheme:light dark;--bg:#fcfcfb;--card:#fff;--line:#dededa;--tx:#0b0b0b;--tx2:#52514e;--ac:#2a78d6;--ok:#0ca30c;--no:#d03b3b}
@media(prefers-color-scheme:dark){:root{--bg:#1a1a19;--card:#232322;--line:#383835;--tx:#fff;--tx2:#c3c2b7;--ac:#3987e5}}
*{box-sizing:border-box}body{margin:0;padding:16px;background:var(--bg);color:var(--tx);
font:14px/1.5 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
.w{max-width:560px;margin:0 auto}h1{font-size:17px;margin:0 0 14px}
.c{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:14px;margin-bottom:14px}
h2{font-size:13px;margin:0 0 10px;color:var(--tx2);text-transform:uppercase;letter-spacing:.05em}
label{display:block;font-size:12px;color:var(--tx2);margin:10px 0 4px}
input,select{width:100%;padding:9px;border:1px solid var(--line);border-radius:7px;
background:var(--bg);color:var(--tx);font:inherit}
button{margin-top:12px;padding:9px 16px;border:0;border-radius:7px;background:var(--ac);
color:#fff;font:inherit;cursor:pointer}button.g{background:transparent;color:var(--tx2);border:1px solid var(--line)}
table{width:100%;border-collapse:collapse}td{padding:4px 0;font-size:13px;vertical-align:top}
td:first-child{color:var(--tx2);width:42%}td:last-child{text-align:right;font-variant-numeric:tabular-nums}
.p{display:inline-block;padding:1px 8px;border-radius:99px;font-size:12px;border:1px solid var(--line)}
.on{color:var(--ok);border-color:var(--ok)}.off{color:var(--no);border-color:var(--no)}
pre{background:var(--bg);border:1px solid var(--line);border-radius:7px;padding:10px;
max-height:220px;overflow:auto;font-size:11px;white-space:pre-wrap;margin:0;color:var(--tx2)}
</style></head><body><div class="w">
<h1>PD Sender &mdash; WiFi + Bluetooth</h1>

<div class="c"><h2>Status</h2><table id="st"><tr><td>memuat&hellip;</td><td></td></tr></table></div>

<div class="c"><h2>Pengaturan WiFi</h2>
<form method="POST" action="/save">
<label>Jaringan</label>
<select id="ssidsel" onchange="document.getElementById('ssid').value=this.value"></select>
<label>SSID</label><input id="ssid" name="ssid" required maxlength="32">
<label>Password (kosongkan kalau jaringan terbuka)</label>
<input name="pass" type="password" maxlength="63">
<button type="submit">Simpan &amp; hubungkan</button>
<button type="button" class="g" onclick="scan()">Pindai ulang</button>
</form></div>

<div class="c"><h2>Log</h2><pre id="lg">memuat&hellip;</pre></div>

<div class="c"><h2>Lain-lain</h2>
<form method="POST" action="/forget" onsubmit="return confirm('Hapus kredensial WiFi dan restart?')">
<button class="g" type="submit">Lupakan WiFi &amp; restart</button></form>
<p style="font-size:12px;color:var(--tx2);margin:12px 0 0">Kalau halaman ini tidak
terjangkau (jaringan tersimpan hilang), kredensial tetap bisa dihapus tanpa komputer:
tekan tombol <b>RST</b> di board dua kali dengan jeda di bawah 6 detik, atau kirim teks
<b>FORGET</b> ke karakteristik BLE <code>6E400002</code> lewat nRF Connect.</p></div>

</div><script>
function pill(b){return '<span class="p '+(b?'on':'off')+'">'+(b?'ya':'tidak')+'</span>'}
async function st(){try{const r=await fetch('/status'),d=await r.json();
document.getElementById('st').innerHTML=
'<tr><td>Mode</td><td>'+d.mode+'</td></tr>'+
'<tr><td>SSID</td><td>'+(d.ssid||'&mdash;')+'</td></tr>'+
'<tr><td>IP</td><td>'+d.ip+'</td></tr>'+
'<tr><td>RSSI</td><td>'+(d.rssi?d.rssi+' dBm':'&mdash;')+'</td></tr>'+
'<tr><td>WiFi</td><td>'+pill(d.wifi)+'</td></tr>'+
'<tr><td>MQTT</td><td>'+pill(d.mqtt)+'</td></tr>'+
'<tr><td>BLE tersambung</td><td>'+pill(d.ble)+'</td></tr>'+
'<tr><td>Topik</td><td style="font-size:11px">'+d.topic+'</td></tr>'+
'<tr><td>Frame dibuat</td><td>'+d.frames+'</td></tr>'+
'<tr><td>Terkirim MQTT</td><td>'+d.mqttFrames+'</td></tr>'+
'<tr><td>Terkirim BLE</td><td>'+d.bleFrames+'</td></tr>'+
'<tr><td>Sampel drop</td><td>'+d.dropped+'</td></tr>'+
'<tr><td>Isi buffer</td><td>'+d.fill+' / '+d.cap+'</td></tr>'+
'<tr><td>Heap bebas</td><td>'+d.heap.toLocaleString('id-ID')+' B</td></tr>'+
'<tr><td>Waktu aktif</td><td>'+d.up+'</td></tr>';
}catch(e){}}
async function lg(){try{const r=await fetch('/log');const t=await r.text();
const p=document.getElementById('lg');p.textContent=t||'(kosong)';p.scrollTop=p.scrollHeight;}catch(e){}}
async function scan(){const s=document.getElementById('ssidsel');
s.innerHTML='<option>memindai&hellip;</option>';
try{const r=await fetch('/scan'),d=await r.json();
s.innerHTML='<option value="">— pilih —</option>'+d.map(n=>
'<option value="'+n.s+'">'+n.s+' ('+n.r+' dBm)'+(n.e?' 🔒':'')+'</option>').join('');
}catch(e){s.innerHTML='<option>gagal memindai</option>'}}
st();lg();scan();setInterval(st,2000);setInterval(lg,3000);
</script></body></html>)HTML";

static String uptimeStr() {
  uint32_t s = millis() / 1000;
  char b[16];
  snprintf(b, sizeof(b), "%02lu:%02lu:%02lu",
           (unsigned long)(s / 3600), (unsigned long)((s / 60) % 60), (unsigned long)(s % 60));
  return String(b);
}

static void handleRoot()   { web.send_P(200, "text/html", PAGE_HTML); }

static void handleStatus() {
  bool up = WiFi.status() == WL_CONNECTED;
  String j = "{";
  j += "\"mode\":\""    + String(g_apMode ? "Access Point (pengaturan)" : "Station") + "\",";
  j += "\"ssid\":\""    + (up ? WiFi.SSID() : g_ssid) + "\",";
  j += "\"ip\":\""      + (g_apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString()) + "\",";
  j += "\"rssi\":"      + String(up ? WiFi.RSSI() : 0) + ",";
  j += "\"wifi\":"      + String(up ? "true" : "false") + ",";
  j += "\"mqtt\":"      + String(mqttReady() ? "true" : "false") + ",";
  j += "\"ble\":"       + String(bleDataReady() ? "true" : "false") + ",";
  j += "\"topic\":\""   + String(MQTT_TOPIC) + "\",";
  j += "\"frames\":"    + String(g_frames) + ",";
  j += "\"mqttFrames\":"+ String(g_mqttFrames) + ",";
  j += "\"bleFrames\":" + String(g_bleFrames) + ",";
  j += "\"dropped\":"   + String(g_dropped) + ",";
  j += "\"fill\":"      + String(g_len) + ",";
  j += "\"cap\":"       + String(PD_BUFFER_SAMPLES) + ",";
  j += "\"heap\":"      + String(ESP.getFreeHeap()) + ",";
  j += "\"up\":\""      + uptimeStr() + "\"}";
  web.send(200, "application/json", j);
}

static void handleLog()  { web.send(200, "text/plain; charset=utf-8", ringText()); }

// Pemindaian memblokir ~2 detik; data masuk tertampung di buffer UART 16 KB.
static void handleScan() {
  int n = WiFi.scanNetworks();
  String j = "[";
  for (int i = 0; i < n && i < 20; i++) {
    if (i) j += ",";
    String s = WiFi.SSID(i);
    s.replace("\\", "\\\\");
    s.replace("\"", "\\\"");
    j += "{\"s\":\"" + s + "\",\"r\":" + String(WiFi.RSSI(i)) +
         ",\"e\":" + String(WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "false" : "true") + "}";
  }
  WiFi.scanDelete();
  web.send(200, "application/json", j + "]");
}

static void handleSave() {
  String ssid = web.arg("ssid");
  String pass = web.arg("pass");
  if (!ssid.length()) { web.send(400, "text/plain", "SSID kosong"); return; }

  prefs.begin("pdsender", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();

  SerialLog.printf("[CFG] SSID disimpan: %s — restart\n", ssid.c_str());
  web.send(200, "text/html",
           "<meta charset=utf-8><body style='font:15px system-ui;padding:24px'>"
           "Tersimpan. Board restart dan mencoba menyambung ke <b>" + ssid + "</b>.<br><br>"
           "Kalau berhasil, halaman ini pindah ke alamat IP board di jaringan tersebut. "
           "Kalau gagal, AP <b>" + String(AP_SSID) + "</b> akan menyala lagi.</body>");
  delay(400);
  ESP.restart();
}

static void handleForget() {
  credsClear("tombol di halaman web");
  web.send(200, "text/html",
           "<meta charset=utf-8><body style='font:15px system-ui;padding:24px'>"
           "Kredensial dihapus. Board restart ke mode pengaturan.</body>");
  delay(400);
  ESP.restart();
}

// Captive portal: alamat apa pun di mode AP diarahkan ke halaman pengaturan.
static void handleNotFound() {
  if (g_apMode) {
    web.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/", true);
    web.send(302, "text/plain", "");
  } else {
    web.send(404, "text/plain", "404");
  }
}

static void webBegin() {
  web.on("/",       HTTP_GET,  handleRoot);
  web.on("/status", HTTP_GET,  handleStatus);
  web.on("/log",    HTTP_GET,  handleLog);
  web.on("/scan",   HTTP_GET,  handleScan);
  web.on("/save",   HTTP_POST, handleSave);
  web.on("/forget", HTTP_POST, handleForget);
  web.onNotFound(handleNotFound);
  web.begin();
}

// ===========================================================================
// 11. WiFi
// ===========================================================================
static void startAp() {
  if (g_apMode) return;

  // Selama STA berstatus "connecting", driver WiFi ESP32 MENOLAK setiap
  // perubahan konfigurasi — gejalanya baris "wifi:sta is connecting, cannot
  // set config" dan softAP() yang gagal tanpa membuat error terlihat. Jadi
  // usaha menyambung dihentikan dulu. Argumen (false,false) = jangan matikan
  // radio, jangan hapus kredensial tersimpan.
  WiFi.disconnect(false, false);
  delay(150);

  WiFi.mode(WIFI_AP_STA);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  delay(200);
  if (!ok) {
    // Jangan set g_apMode: biar dicoba lagi di putaran berikutnya, bukan
    // dianggap sudah menyala padahal tidak.
    SerialLog.println("[AP  ] GAGAL menyalakan access point, akan dicoba lagi");
    return;
  }

  g_apMode = true;
  dns.start(53, "*", WiFi.softAPIP());
  SerialLog.printf("[AP  ] \"%s\" pass \"%s\" -> http://%s\n",
                   AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
}

static void stopAp() {
  if (!g_apMode) return;
  g_apMode = false;
  dns.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  SerialLog.println("[AP  ] dimatikan, sudah tersambung ke WiFi");
}

static void wifiBegin() {
  prefs.begin("pdsender", true);
  g_ssid = prefs.getString("ssid", "");
  g_pass = prefs.getString("pass", "");
  prefs.end();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                   // jangan tidur, MQTT jadi lebih responsif

  if (!g_ssid.length()) {
    SerialLog.println("[WIFI] belum ada kredensial -> mode pengaturan");
    startAp();
    return;
  }

  SerialLog.printf("[WIFI] menyambung ke \"%s\"...\n", g_ssid.c_str());
  WiFi.begin(g_ssid.c_str(), g_pass.c_str());
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_CONNECT_MS) delay(200);

  if (WiFi.status() == WL_CONNECTED) {
    SerialLog.printf("[WIFI] tersambung, IP %s (RSSI %d dBm)\n",
                     WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    SerialLog.println("[WIFI] gagal -> mode pengaturan");
    startAp();
  }
}

static bool wifiEnsure() {
  if (WiFi.status() == WL_CONNECTED) {
    g_wifiFail = 0;
    if (g_apMode) stopAp();               // sudah dapat WiFi, AP tidak perlu lagi
    return true;
  }
  if (!g_ssid.length()) { startAp(); return false; }

  static uint32_t lastTry = 0;
  // Saat AP menyala, percobaan STA dijarangkan. AP dan STA berbagi satu radio
  // dan harus sekanal: STA yang sedang memindai menyeret kanal AP ikut pindah,
  // sehingga AP hilang-timbul dari daftar scan HP persis ketika Anda sedang
  // berusaha menyambunginya untuk mengganti setelan.
  uint32_t gap = g_apMode ? WIFI_RETRY_AP_MS : WIFI_RETRY_MS;
  if (millis() - lastTry < gap) return false;
  lastTry = millis();

  if (g_wifiFail < 255) g_wifiFail++;

  // AP dinyalakan LEBIH DULU, sebelum WiFi.begin() membuat STA sibuk. Kalau
  // urutannya terbalik, driver menolak konfigurasi AP dan portal pengaturan
  // tidak pernah muncul — tepat yang terjadi sebelum perbaikan ini.
  if (g_wifiFail >= WIFI_FAILS_BEFORE_AP) startAp();

  SerialLog.printf("[WIFI] mencoba lagi (%u)...\n", g_wifiFail);
  WiFi.begin(g_ssid.c_str(), g_pass.c_str());
  return false;
}

// ===========================================================================
// 12. setup / loop
// ===========================================================================
void setup() {
#if LOG_TARGET == 2
  Serial2.begin(LOG_BAUD, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);
#endif
#if DATA_FROM_USB
  Serial.setRxBufferSize(DATA_RX_BUF);
  Serial.begin(DATA_BAUD);
#else
  Serial2.setRxBufferSize(DATA_RX_BUF);
  Serial2.begin(DATA_BAUD, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);
#endif
  delay(200);

  pinMode(LED_BLUE_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(BOARD_LED_PIN, OUTPUT);
  digitalWrite(LED_BLUE_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);
  boardLed(false);
  ledTicker.attach_ms(300, ledTick);

  bleBegin();

  SerialLog.println("\n=== PD Sender WiFi+BT - LilyGO T-SIM7600G ===");

  // Deteksi tekan-RST-dua-kali. Harus dijalankan SEBELUM wifiBegin(), karena
  // di sanalah kredensial dibaca dari NVS.
  if (g_drdA == DRD_MAGIC && g_drdB == ~DRD_MAGIC) {
    g_drdA = 0;
    g_drdB = 0;
    credsClear("tombol RST ditekan dua kali");
    for (int i = 0; i < 6; i++) {          // kedipan tanda supaya terlihat
      digitalWrite(LED_RED_PIN, HIGH); digitalWrite(LED_BLUE_PIN, HIGH); delay(120);
      digitalWrite(LED_RED_PIN, LOW);  digitalWrite(LED_BLUE_PIN, LOW);  delay(120);
    }
  } else {
    g_drdA = DRD_MAGIC;
    g_drdB = ~DRD_MAGIC;
    SerialLog.printf("[CFG ] tekan RST sekali lagi dalam %lu detik untuk menghapus WiFi\n",
                     (unsigned long)(DRD_WINDOW_MS / 1000));
  }
#if ENABLE_BLE
  SerialLog.printf("[BLE ] advertising sebagai \"%s\"\n", BLE_DEVICE_NAME);
#endif
#if DATA_FROM_USB
  SerialLog.printf("[IN  ] sumber data: USB-C / UART0 @ %lu baud\n", (unsigned long)DATA_BAUD);
#else
  SerialLog.printf("[IN  ] sumber data: pin GPIO%d (UART2) @ %lu baud\n",
                   UART2_RX_PIN, (unsigned long)DATA_BAUD);
#endif
  SerialLog.printf("[BUF ] %lu sampel per frame\n", (unsigned long)PD_BUFFER_SAMPLES);

  wifiBegin();
  webBegin();
  SerialLog.println("[WEB ] server pengaturan aktif di port 80");

#if ENABLE_MQTT
#if WIFI_USE_TLS
  netClient.setInsecure();        // ganti setCACert() bila ingin verifikasi sertifikat
#endif
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(OUT_CAP + 128);
  mqtt.setKeepAlive(60);
  mqtt.setSocketTimeout(15);
  SerialLog.printf("[MQTT] %s:%u topik %s\n", MQTT_HOST, MQTT_PORT, MQTT_TOPIC);
#endif
}

void loop() {
  // Lewat jendela deteksi -> penanda dilucuti, jadi reset berikutnya dianggap
  // boot biasa, bukan tekan-dua-kali.
  static bool drdArmed = true;
  if (drdArmed && millis() > DRD_WINDOW_MS) {
    drdArmed = false;
    g_drdA = 0;
    g_drdB = 0;
  }

  // Perintah FORGET dari BLE (dieksekusi di sini, bukan di dalam callback).
  if (g_forgetReq) {
    g_forgetReq = false;
    credsClear("perintah FORGET lewat BLE");
    delay(300);
    ESP.restart();
  }

  if (g_apMode) dns.processNextRequest();
  web.handleClient();

  bleServiceAdv();

  // Selama sebuah frame masih terbang, UART SENGAJA tidak diserap. Byte yang
  // masuk menumpuk di ring buffer driver (DATA_RX_BUF = 16384 B = 3.276 sampel
  // = 13,4 detik pada laju 244/detik), lalu terbaca utuh setelah frame lepas.
  // Ini menggantikan perilaku lama yang membuang sampel saat slot kirim sibuk.
  if (g_outLen == 0) drainSerialData();

  g_wifiUp = wifiEnsure();
  g_mqttUp = mqttEnsure();
#if ENABLE_MQTT
  if (g_mqttUp) mqtt.loop();
#endif

  if (g_outLen == 0) drainSerialData();
  bleTxPump();
  mqttPump();
  retireIfDone();
  rxLedService();

  static uint32_t lastLog = 0;
  if (millis() - lastLog >= 5000) {
    lastLog = millis();
    SerialLog.printf("[STAT] wifi=%d mqtt=%d ble=%d | rx %lu B | isi %lu/%lu | antre %u B | "
                     "frame %lu (mqtt %lu, ble %lu) | drop %lu | macet %lu | heap %lu\n",
                     (int)g_wifiUp, (int)mqttReady(), (int)bleDataReady(),
                     (unsigned long)g_rxTotal,
                     (unsigned long)g_len, (unsigned long)PD_BUFFER_SAMPLES,
                     (unsigned)SerialData.available(),
                     (unsigned long)g_frames, (unsigned long)g_mqttFrames,
                     (unsigned long)g_bleFrames, (unsigned long)g_dropped,
                     (unsigned long)g_stuck,
                     (unsigned long)ESP.getFreeHeap());
  }
}
