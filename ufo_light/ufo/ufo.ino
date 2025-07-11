#include <esp_now.h>
#include <WiFi.h>
#include "M5Atom.h"
#include <FastLED.h>

// Laser settings
#define LASER_PIN 32

// LED settings
#define LED_PIN 25
#define NUM_LEDS 24
CRGB leds[NUM_LEDS];


// Mode variables
uint8_t currentMode = 1; // 1: 青, 2: 赤, 3: 緑
bool buttonPressed = false;
unsigned long lastButtonTime = 0;
unsigned long buttonDebounceTime = 200; // ボタンのデバウンス時間（ms）

struct DataPacket {
  uint8_t hue;
  uint8_t sat;
};

uint8_t targetHue = 0;
uint8_t targetSat = 2;
DataPacket data;

// データ受信時のコールバック関数
void OnDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingData, int len) {
     if (len == sizeof(DataPacket)) {
        memcpy(&data, incomingData, sizeof(DataPacket));

        targetSat = data.sat;
        targetHue = data.hue;
    } else {
        Serial.println("Received data of unexpected length");
    }
}

void setup() {
    M5.begin(true, false, true);  // M5Atomの初期化
    // シリアルモニタの初期化
    Serial.begin(9600);
    delay(5000);
    Serial.println("Begin");

    // T0: PIN32番をレーザー用に初期化してOFFにする
    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, LOW); // レーザーをOFF

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() == ESP_OK) {
        Serial.println("ESPNow Init Success");
    } else {
        Serial.println("ESPNow Init Failed");
        ESP.restart();
    }

    esp_now_register_recv_cb(OnDataRecv);
    // 自身のMACアドレスを取得して表示
    Serial.println("MAC Address:");
    Serial.println(WiFi.macAddress());



    // T0: LED set up - LEDをOFFで初期化
    FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
    FastLED.clear();
    FastLED.show();
    
    // 初期モードを表示
    Serial.print("Current Mode: ");
    Serial.println(currentMode);
    displayModeColor();
}

// T2用の変数
uint8_t ledPositions[4] = {0, 6, 12, 18}; // LEDの位置を4つに設定
unsigned long lastShiftTime = 0;

void handleMode1(bool isRandomColor = true, unsigned long shiftInterval = 1000);
void handleMode2(unsigned long shiftInterval = 1000);
void handleMode3();

void loop() {
    M5.update();
    // T1: ボタンによるモード切り替え
    if (M5.Btn.wasPressed()) {
        unsigned long currentTime = millis();
        if (currentTime - lastButtonTime > buttonDebounceTime) {
            currentMode++;
            if (currentMode > 3) {
                currentMode = 1;
            }
            Serial.print("Mode changed to: ");
            Serial.println(currentMode);
            displayModeColor();
            lastButtonTime = currentTime;
            // モード変更時にLEDをクリア
            FastLED.clear();
            digitalWrite(LASER_PIN, LOW); // レーザーをOFF
        }
    }

    // モードごとに関数で処理
    switch (currentMode) {
        case 1:
            handleMode1();
            break;
        case 2:
            handleMode2();
            break;
        case 3:
            handleMode3();
        default:
            break;
    }

    M5.update(); // M5Atomのボタン状態を更新
    delay(20);
}

// T2: モード1のLED点灯処理
void handleMode1(bool isRandomColor, unsigned long shiftInterval) {
    unsigned long now = millis();
    if (now - lastShiftTime >= shiftInterval) {
        Serial.println(shiftInterval);
        // 位置を1つずつ進める
        for (int i = 0; i < 4; i++) {
            ledPositions[i] = (ledPositions[i] + 1) % NUM_LEDS;
        }
        Serial.println(ledPositions[0]);
        lastShiftTime = now;
        // LED全消灯
        FastLED.clear();
        // 3つのLEDをランダム色・最大輝度で点灯
        for (int i = 0; i < 4; i++) {
            if (isRandomColor) {
                leds[ledPositions[i]] = CHSV(random8(), 255, 255);
            } else {
                leds[ledPositions[i]] = CHSV(255, 255, 255);
            }
        }
        FastLED.show();
    }
}

// T3: モード2 LED+レーザーモード
void handleMode2(unsigned long shiftInterval ) {
    static unsigned long mode2StartTime = 0;
    static bool ledPhase = true; // true: LED点灯, false: レーザー点灯
    static bool firstEntry = true;
    unsigned long now = millis();

    if (firstEntry) {
        mode2StartTime = now;
        ledPhase = true;
        firstEntry = false;
    }

    if (ledPhase) {
        // LED点灯フェーズ（10秒）
        handleMode1(true, shiftInterval); // LEDの光り方はモード1と同じ
        digitalWrite(LASER_PIN, LOW); // レーザーOFF
        if (now - mode2StartTime >= 8000) {
            ledPhase = false;
            mode2StartTime = now;
        }
    } else {
        // レーザー点灯フェーズ（2秒）
        FastLED.clear();
        FastLED.show();
        digitalWrite(LASER_PIN, HIGH); // レーザーON
        if (now - mode2StartTime >= 3000) {
            ledPhase = true;
            mode2StartTime = now;
        }
    }
}

unsigned long mode3Interval = 1000; // モード3の間隔（ミリ秒）
void handleMode3() {
    if (targetSat > 30) {
        handleMode2(mode3Interval);
        // 間隔を調整
        if (mode3Interval <= 30) {
            mode3Interval = 20;
        } else {
            mode3Interval = mode3Interval - 2; 
        }
    } else {
        handleMode1(true, mode3Interval);
        if (mode3Interval >= 1500) {
            mode3Interval = 1500;
        } else {
            mode3Interval = mode3Interval + 1; 
        }
    }
}

// T1: モードに応じた色をM5 Atom自身のLEDで表示する関数
void displayModeColor() {
    uint32_t modeColor;
    
    switch (currentMode) {
        case 1: // モード1: 青
            modeColor = 0x0000ff; // 青色
            break;
        case 2: // モード2: 赤
            modeColor = 0xff0000; // 赤色
            break;
        case 3: // モード3: 緑
            modeColor = 0x00ff00; // 緑色
            break;
        default:
            modeColor = 0x000000; // 黒色（消灯）
            break;
    }
    
    // M5 Atom自身のLEDでモード色を表示
    M5.dis.drawpix(0, modeColor);
    delay(1000); // 1秒間表示
    
    // M5 AtomのLEDを消灯
    M5.dis.drawpix(0, 0x000000);
}



