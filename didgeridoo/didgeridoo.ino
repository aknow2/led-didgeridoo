#include <driver/i2s.h>
#include "M5Atom.h"
#include "arduinoFFT.h"
#include <FastLED.h>
#include <WiFi.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define CONFIG_I2S_BCK_PIN     -1
#define CONFIG_I2S_LRCK_PIN    5
#define CONFIG_I2S_DATA_PIN    -1
#define CONFIG_I2S_DATA_IN_PIN 19

#define SPEAK_I2S_NUMBER I2S_NUM_0

#define MODE_MIC 0
#define MODE_SPK 1

#define SAMPLES 1024             // 1024サンプルを使ってFFTを計算
#define SAMPLING_FREQUENCY 16000 // サンプリング周波数16kHz

#define NUM_PEAKS 2 // ピーク周波数の数

// LED settings
#define LED_PIN 26
#define NUM_LEDS 144
CRGB leds[NUM_LEDS];

ArduinoFFT<double> FFT = ArduinoFFT<double>();


double vReal[SAMPLES];
double vImag[SAMPLES];


double prevMaxAmplitude = 0; // 前回の最大振幅を保存
double prevAverageFrequency = 0; // 前回の平均周波数を保存
double amplitudeChange = 0.0; // 振幅の変化量
double frequencyChange = 0.0; // 周波数の変化量

// ESP NOW setting
esp_now_peer_info_t peerInfo;
struct DataPacket {
    uint8_t sat;
    uint8_t hue;
};

bool InitI2SSpeakOrMic(int mode) {
    esp_err_t err = ESP_OK;

    i2s_driver_uninstall(SPEAK_I2S_NUMBER);
    i2s_config_t i2s_config = {
        .mode        = (i2s_mode_t)(I2S_MODE_MASTER),
        .sample_rate = SAMPLING_FREQUENCY,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 1, 0)
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
#else
        .communication_format = I2S_COMM_FORMAT_I2S,
#endif
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count    = 6,
        .dma_buf_len      = 60,
    };
    if (mode == MODE_MIC) {
        i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
    } else {
        i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
        i2s_config.use_apll = false;
        i2s_config.tx_desc_auto_clear = true;
    }

    //Serial.println("Init i2s_driver_install");

    err += i2s_driver_install(SPEAK_I2S_NUMBER, &i2s_config, 0, NULL);
    i2s_pin_config_t tx_pin_config;

#if (ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 3, 0))
    tx_pin_config.mck_io_num = I2S_PIN_NO_CHANGE;
#endif

    tx_pin_config.bck_io_num   = CONFIG_I2S_BCK_PIN;
    tx_pin_config.ws_io_num    = CONFIG_I2S_LRCK_PIN;
    tx_pin_config.data_out_num = CONFIG_I2S_DATA_PIN;
    tx_pin_config.data_in_num  = CONFIG_I2S_DATA_IN_PIN;

    //Serial.println("Init i2s_set_pin");
    err += i2s_set_pin(SPEAK_I2S_NUMBER, &tx_pin_config);
    //Serial.println("Init i2s_set_clk");
    err += i2s_set_clk(SPEAK_I2S_NUMBER, SAMPLING_FREQUENCY, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);

    return true;
}

uint8_t microphonedata0[SAMPLES * 2];
size_t byte_read = 0;
bool samplingActive = true; // サンプリングがアクティブかどうかを示すフラグ

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Deli_Success" : "Deli_Fail");
}

#define DATA_SIZE 1024

// ピーク周波数を取得する関数
void getPeaks(double *peaks, double *peakAmplitudes, int numPeaks) {
    for (int i = 0; i < numPeaks; i++) {
        peaks[i] = 0;
        peakAmplitudes[i] = 0;
    }

    for (int i = 2; i < (SAMPLES / 2); i++) {
        double amplitude = vReal[i];
        for (int j = 0; j < numPeaks; j++) {
            if (amplitude > peakAmplitudes[j]) {
                for (int k = numPeaks - 1; k > j; k--) {
                    peakAmplitudes[k] = peakAmplitudes[k - 1];
                    peaks[k] = peaks[k - 1];
                }
                peakAmplitudes[j] = amplitude;
                peaks[j] = (i * (SAMPLING_FREQUENCY / SAMPLES));
                break;
            }
        }
    }
}

// 平均周波数と平均振幅を取得する関数
void getAverageFrequencyAndAmplitude(double &averageFrequency, double &averageAmplitude, double *peaks, double *peakAmplitudes, int numPeaks) {
    double sumFrequency = 0;
    double sumAmplitude = 0;
    for (int i = 0; i < numPeaks; i++) {
        sumFrequency += peaks[i];
        sumAmplitude += peakAmplitudes[i];
    }
    averageFrequency = sumFrequency / numPeaks;
    averageAmplitude = sumAmplitude / numPeaks;
}

// FFT結果を取得する関数
void getFFTResults(double &averageFrequency, double &averageAmplitude) {
    uint32_t data_offset = 0;
    size_t bytes_read = 0;

    // サンプリングを行う
    while (data_offset < sizeof(microphonedata0)) {
        i2s_read(SPEAK_I2S_NUMBER, (char *)(microphonedata0 + data_offset), DATA_SIZE, &bytes_read, (100 / portTICK_RATE_MS));
        data_offset += bytes_read;
    }

    for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = ((microphonedata0[i * 2 + 1] << 8) | (microphonedata0[i * 2] & 0xFF)) / 32768.0; // 16ビットデータを変換
        vImag[i] = 0.0; // 虚数部分は0
    }

    FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD); // ハミング窓を適用
    FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD); // FFTを計算
    FFT.complexToMagnitude(vReal, vImag, SAMPLES); // 結果を振幅に変換

    double peaks[NUM_PEAKS];
    double peakAmplitudes[NUM_PEAKS];
    getPeaks(peaks, peakAmplitudes, NUM_PEAKS);
    getAverageFrequencyAndAmplitude(averageFrequency, averageAmplitude, peaks, peakAmplitudes, NUM_PEAKS);
}

void setColorBasedOnChange(double amplitudeChange, double frequencyChange) {
    // 振幅の変化率と周波数の変化を組み合わせて色を設定
    double changeMetric = amplitudeChange + frequencyChange; // 単純に足すか、他の組み合わせ方も検討
    uint8_t hue = map(changeMetric, -200, 200, 0, 255); // 変化の指標を色相にマッピング
    leds[0] = CHSV(hue, 255, 255);
}
void shiftLEDs() {
    for (int i = NUM_LEDS - 1; i > 0; i--) {
        leds[i] = leds[i - 1];
    }
}

void dimLED(CRGB &led) {
    led.fadeToBlackBy(3);
}

void FFTTask(void *parameter) {
    while (1) {
        double averageFrequency;
        double averageAmplitude;

        getFFTResults(averageFrequency, averageAmplitude);

        amplitudeChange = averageAmplitude - prevMaxAmplitude;
        frequencyChange = averageFrequency - prevAverageFrequency;
        prevMaxAmplitude = averageAmplitude;
        prevAverageFrequency = averageFrequency;



        vTaskDelay(10 / portTICK_PERIOD_MS); // 10ms待機
    }
}

double amlitudeThd = 50;
void LEDTask(void *parameter) {
  while (1) {
    if (prevMaxAmplitude > amlitudeThd) {
        setColorBasedOnChange(amplitudeChange, frequencyChange);
    } else {
        dimLED(leds[0]);
    }
    shiftLEDs();
    FastLED.show();
    DataPacket data;
    CHSV hsvColor = rgb2hsv_approximate(leds[0]);
    data.hue = hsvColor.hue;
    data.sat = map(prevMaxAmplitude, 0, amlitudeThd, 0, 255);
    esp_now_send(NULL, (uint8_t *)&data, sizeof(data));
    vTaskDelay(10 / portTICK_PERIOD_MS); // 10ms待機
  }
}


void setup() {
  M5.begin(true, false, true);
  M5.dis.clear();

  Serial.begin(9600);
  Serial.println("Init Microphone");
  InitI2SSpeakOrMic(MODE_MIC);
  delay(100);


  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();

  
  // ペアのMACアドレスを登録（ここでは例としてデバイスのMACアドレスを指定）
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }

  memset(&peerInfo, 0, sizeof(peerInfo));
  // E8:6B:EA:30:A5:68
  uint8_t peerMacAddress[] = {0xE8, 0x6B, 0xEA, 0x30, 0xA5, 0x68};

  memcpy(peerInfo.peer_addr, peerMacAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Peer情報の追加
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
  } 
  esp_now_register_send_cb(OnDataSent);  

  xTaskCreate(FFTTask, "FFTTask", 4096, NULL, 1, NULL);
  xTaskCreate(LEDTask, "LEDTask", 2048, NULL, 1, NULL);
}

void loop() {
    M5.update();
    if (M5.Btn.wasPressed()) {
      amlitudeThd -= 10;
      if (amlitudeThd < 0) {
        amlitudeThd = 100;
      }
    }
   vTaskDelay(100 / portTICK_PERIOD_MS);
}
