#include <esp_now.h>
#include <WiFi.h>
#include "M5Atom.h"
#include <FastLED.h>

// LED settings
#define LED_PIN 26
#define NUM_LEDS 31
CRGB leds[NUM_LEDS];

// 受信するデータの構造体を定義
struct DataPacket {
  uint8_t hue;
  uint8_t sat;
};

// 受信するデータのインスタンスを作成
DataPacket data;

// 前回受信した周波数を記録する変数
float previousFrequency = 0.0;

// 現在の色相を保持する変数
uint8_t targetHue = 0;
uint8_t targetSat = 0;


// データ受信時のコールバック関数
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    if (len == sizeof(DataPacket)) {
        memcpy(&data, incomingData, sizeof(DataPacket));

        targetSat = data.sat;
        targetHue = data.hue;
    } else {
        Serial.println("Received data of unexpected length");
    }
}

void setup() {
    M5.begin(true, false, false);  // M5Atomの初期化

    // シリアルモニタの初期化
    Serial.begin(9600);
    delay(5000);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() == ESP_OK) {
        Serial.println("ESPNow Init Success");
    } else {
        Serial.println("ESPNow Init Failed");
        ESP.restart();
    }

    // 自身のMACアドレスを取得して表示
    Serial.println("MAC Address:");
    Serial.println(WiFi.macAddress());

    esp_now_register_recv_cb(OnDataRecv);

    // LED set up
    FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
    FastLED.clear();
    FastLED.show();
}

uint8_t hue = 0;
uint8_t sat = 0;
void loop() {
  // LEDの色と輝度を設定
  if (hue != targetHue) {
      hue = (targetHue > hue) ? hue + 1 : hue - 1;
  }

  if (sat != targetSat) {
      sat = (targetSat > sat) ? sat + 1 : sat - 1;
  }

  for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CHSV(hue, sat, 255);
  }
  FastLED.show();
  delay(8);
}
