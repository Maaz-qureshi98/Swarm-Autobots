// ESP-NOW broadcast loss test (measures the p and L of the Gilbert-Elliott model on your robots).
// Flash the same sketch to two or more ESP32s. Set ROLE to 1 on the sender, 0 on receivers.
// The sender broadcasts a 20-byte frame with a sequence number at 10 Hz (like the leader heartbeat).
// Each receiver prints "seq,rssi,millis" over serial; log it with:  python3 analyze_loss.py port_or_file
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define ROLE 1                // 1 = sender, 0 = receiver
#define RATE_HZ 10
static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct __attribute__((packed)) Frame { uint8_t id; uint16_t seq; uint8_t pad[17]; };  // 20 B, as LeaderState
static_assert(sizeof(Frame) == 20, "frame must be 20 bytes");
volatile int last_rssi = 0;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (info && info->rx_ctrl) last_rssi = info->rx_ctrl->rssi;
#else
void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
#endif
  if (len != sizeof(Frame)) return;
  Frame f; memcpy(&f, data, sizeof f);
  Serial.printf("%u,%d,%lu\n", f.seq, last_rssi, millis());
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) { Serial.println("esp_now_init failed"); while (true) delay(1000); }
  if (ROLE == 1) {
    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, BCAST, 6); p.channel = 1; p.encrypt = false;
    esp_now_add_peer(&p);
  } else {
    esp_now_register_recv_cb(onRecv);
  }
  Serial.println("ready");
}

void loop() {
  if (ROLE != 1) { delay(10); return; }
  static uint16_t seq = 0;
  Frame f = {}; f.id = 0; f.seq = seq++;
  esp_now_send(BCAST, (const uint8_t*)&f, sizeof f);
  delay(1000 / RATE_HZ);
}
