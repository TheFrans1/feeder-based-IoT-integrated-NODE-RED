#include "esp_camera.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <base64.h>

// Pilih model kamera yang sesuai
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// Konfigurasi WiFi
const char* ssid = "sile";
const char* password = "sembiring1";
const char* mqtt_server = "broker.mqtt-dashboard.com";
const int mqtt_port = 1883;
const char* mqtt_topic = "ESPCAM/gambar";

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long previousMillis = 0;
const long interval = 20000;

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Menghubungkan ke ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi terhubung");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Menghubungkan ke MQTT...");
    if (client.connect("ESP32CAMClient")) {
      Serial.println("terhubung");
    } else {
      Serial.print("gagal, rc=");
      Serial.print(client.state());
      Serial.println(" coba lagi dalam 5 detik");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  // Atur buffer MQTT yang lebih besar
  client.setBufferSize(1024 * 50); // 50KB buffer
  // Konfigurasi kamera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  // Resolusi lebih rendah untuk menghemat memori
  config.frame_size = FRAMESIZE_QVGA; // 800x600
  config.jpeg_quality = 10; // 0-63 (lebih rendah = kualitas lebih baik)
  config.fb_count = 1;
  // Inisialisasi kamera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Kamera init gagal dengan error 0x%x", err);
    ESP.restart();
  }
  // Cek sensor kamera
  sensor_t *s = esp_camera_sensor_get();
  if (s == NULL) {
    Serial.println("Gagal memuat sensor kamera");
    ESP.restart();
  }
  s->set_vflip(s, 1);   // 1 = balik vertikal, 0 = normal
  s->set_hmirror(s, 0); // 1 = mirror horizontal, 0 = normal
}

void captureAndSend() {
  // Ambil gambar
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Gagal mengambil gambar");
    return;
  }
  Serial.printf("Gambar diambil! Ukuran: %d bytes\n", fb->len);
  // Konversi ke base64
  String imageBase64 = base64::encode(fb->buf, fb->len);
  Serial.printf("Ukuran base64: %d bytes\n", imageBase64.length());
  // Kirim via MQTT
  if (client.publish(mqtt_topic, imageBase64.c_str(), imageBase64.length())) {
    Serial.println("Gambar berhasil dikirim ke MQTT");
  } else {
    Serial.print("Gagal mengirim gambar, state: ");
    Serial.println(client.state());
  }
  // Lepaskan memori gambar
  esp_camera_fb_return(fb);
}

 void loop() {
  // Reconnect WiFi jika terputus
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi terputus! Mencoba menyambung kembali...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    unsigned long wifiReconnectStart = millis();
    // Tunggu maksimal 10 detik
    while (WiFi.status() != WL_CONNECTED && millis() - wifiReconnectStart < 10000) {
      delay(500);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("");
      Serial.println("WiFi berhasil terhubung kembali!");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("");
      Serial.println("Gagal menyambung WiFi!");
      return; // keluar dari loop() dulu, coba lagi nanti
    }
  }
  // Reconnect MQTT jika terputus
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  // Jalankan capture tiap 20 detik
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    captureAndSend();
  }
}
