  #include <WiFi.h>
  #include <PubSubClient.h>
  #include <Wire.h>
  #include <RTClib.h>
  #include <EEPROM.h>
  #include <ESP32Servo.h>
  #include <LiquidCrystal_I2C.h>
  #include <OneWire.h>
  #include <DallasTemperature.h>

  /* ---------- WiFi & MQTT ---------- */
  const char* ssid        = "sile";
  const char* password    = "sembiring1";
  const char* mqtt_server = "broker.mqtt-dashboard.com";
  const int   mqtt_port   = 1883;

  /* ---------- Topik MQTT ---------- */
  #define TOPIC_EEPROM     "/jadwal/data"
  #define TOPIC_WAKTU      "/jadwal/waktu"
  #define TOPIC_STATUS     "/jadwal/status"
  #define TOPIC_CMD        "/W1/W2/D"
  #define TOPIC_LEVEL      "/pellet/level"
  #define TOPIC_TEMP       "/suhu/kolam"
  #define TOPIC_TURBIDITY  "/kolam/kekeruhan"
  #define TOPIC_ESP_WIFI_STATUS "/esp32/status"

  /* ---------- Hardware PIN ---------- */
  #define TRIG_PIN 26
  #define ECHO_PIN 27
  #define ONE_WIRE_BUS 5
  const int servoPin1     = 14;
  const int servoPin2     = 13;
  const int sensorPinTurb = 36;

  /* ---------- Servo ---------- */
  int kecepatanServo = 70;
  Servo servo1, servo2;
  bool servoAktif = false;
  unsigned long servoStart = 0;
  int eksekusiKe = 0;

  /* ---------- RTC & EEPROM ---------- */
  #define EEPROM_SIZE 64
  RTC_DS3231 rtc;
  String waktu1 = "";
  String waktu2 = "";
  int durasi = 0;

  /* ---------- LCD ---------- */
  LiquidCrystal_I2C lcd(0x27, 16, 2);
  byte lcdScreen = 0;
  unsigned long lcdLast = 0;
  const unsigned long lcdInterval = 2000;

  /* ---------- HC-SR04 ---------- */
  #define MIN_DISTANCE 1
  #define MAX_DISTANCE 14
  #define NUM_READINGS 10
  const unsigned long ULTRA_INTERVAL = 10000;
  unsigned long ultraLast = 0;
  int pelletPercent = 0;

  /* ---------- DS18B20 ---------- */
  OneWire oneWire(ONE_WIRE_BUS);
  DallasTemperature sensors(&oneWire);
  float waterTemp = 0.0;
  const unsigned long TEMP_INTERVAL = 10000;
  unsigned long tempLast = 0;

  /* ---------- Turbidity ---------- */
  unsigned long turbLast = 0;
  const unsigned long TURB_INTERVAL = 10000; 
  int turbidityPct = 0;
  String turbidityClass = "UNKNOWN";

  /* ---------- WiFi + MQTT ---------- */
  WiFiClient espClient;
  PubSubClient client(espClient);
  unsigned long lastReconnectAttempt = 0;
  const unsigned long RECONNECT_INTERVAL = 10000;
  unsigned long lastWiFiStatusSent = 0;

  /* ---------- Deklarasi fungsi ---------- */
  void publishEEPROMData();
  void publishWaktuSekarang(const String& t);
  void publishStatus(const char* s);
  void updateLCD();
  void readAndPublishTemp();
  void checkPelletLevel();
  void checkTurbidity();
  void publishWiFiStatus(const char* status); 

  /* EEPROM */
  void simpanKeEEPROM() {
    EEPROM.writeString(0, waktu1);
    EEPROM.writeString(10, waktu2);
    EEPROM.writeInt(20, durasi);
    EEPROM.commit();
    delay(100);
    publishEEPROMData();
  }

  void bacaDariEEPROM() {
    waktu1 = EEPROM.readString(0);
    waktu2 = EEPROM.readString(10);
    durasi = EEPROM.readInt(20);
  }

  /* MQTT */
  void callback(char* topic, byte* payload, unsigned int length) {
    String message;
    for (unsigned int i = 0; i < length; i++) message += (char)payload[i];
    if (message.startsWith("Sukses mengatur")) {
      int i1 = message.indexOf("Waktu 1 = ");
      int i2 = message.indexOf(", Waktu 2 = ");
      int i3 = message.indexOf(", Durasi: ");
      int i4 = message.indexOf(" s", i3);
      if (i1 != -1 && i2 != -1 && i3 != -1 && i4 != -1) {
        waktu1 = message.substring(i1 + 10, i2);
        waktu2 = message.substring(i2 + 12, i3);
        durasi = message.substring(i3 + 10, i4).toInt();
        simpanKeEEPROM();
      }
    }
  }

  void publishWiFiStatus(const char* status) {
    client.publish(TOPIC_ESP_WIFI_STATUS, status);
  }

  void setup_wifi() {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) delay(500);
    publishWiFiStatus("online");
  }

  void reconnectNonBlocking() {
    if (!client.connected()) {
      unsigned long now = millis();
      if (now - lastReconnectAttempt > RECONNECT_INTERVAL) {
        lastReconnectAttempt = now;
        if (WiFi.status() == WL_CONNECTED) {
          String cid = "ESP32Client-" + String(random(0xffff), HEX);
          if (client.connect(cid.c_str())) {
            client.subscribe(TOPIC_CMD);
            publishEEPROMData();
            publishWiFiStatus("online");
          }
        } else {
          publishWiFiStatus("offline");
        }
      }
    }
  }

  /* Setup */
  void setup() {
    Serial.begin(115200);
    analogReadResolution(10);
    analogSetAttenuation(ADC_11db);
    Wire.begin();
    Wire.setClock(400000);
    rtc.begin();
    if (rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    EEPROM.begin(EEPROM_SIZE);
    bacaDariEEPROM();
    servo1.attach(servoPin1);
    servo2.attach(servoPin2);
    servo1.write(90); servo2.write(90);
    lcd.init(); lcd.backlight(); lcd.print("BOOTING...");
    pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
    sensors.begin();
    setup_wifi();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    publishEEPROMData();
  }

  /* Servo */
  void putarServo() {
    servo1.write(kecepatanServo);
    servo2.write(kecepatanServo);
    servoAktif = true;
    servoStart = millis();
  }

  void stopServoJikaPerlu() {
    if (servoAktif && millis() - servoStart >= (unsigned long)durasi * 1000UL) {
      servo1.write(90); servo2.write(90);
      servoAktif = false;
      publishStatus(eksekusiKe == 1 ? "Pakan1, Telah Diberikan" : "Pakan2, Telah Diberikan");
      eksekusiKe = 0;
    }
  }

  /* Sensor */
  void checkPelletLevel() {
    if (millis() - ultraLast < ULTRA_INTERVAL) return;
    ultraLast = millis();
    float total = 0; int valid = 0;
    for (int i = 0; i < NUM_READINGS; i++) {
      digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
      digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);
      long d = pulseIn(ECHO_PIN, HIGH, 25000);
      if (d > 0) { total += (d * 0.0343) / 2.0; valid++; }
      delay(50);
    }
    if (!valid) return;
    float avg = total / valid;
    float pct = ((MAX_DISTANCE - avg) / (MAX_DISTANCE - MIN_DISTANCE)) * 100.0;
    pct = constrain(pct, 0, 100);
    pelletPercent = round(pct);
    char buf[8]; sprintf(buf, "%d", pelletPercent);
    client.publish(TOPIC_LEVEL, buf);
  }

  void readAndPublishTemp() {
    if (millis() - tempLast < TEMP_INTERVAL) return;
    tempLast = millis();
    sensors.requestTemperatures();
    waterTemp = sensors.getTempCByIndex(0);
    if (waterTemp == DEVICE_DISCONNECTED_C) return;
    char buf[10]; dtostrf(waterTemp, 4, 1, buf);
    client.publish(TOPIC_TEMP, buf);
  }

  void checkTurbidity() {
    if (millis() - turbLast < TURB_INTERVAL) return;
    turbLast = millis();
    int total = 0;
    for (int i = 0; i < 10; i++) { total += analogRead(sensorPinTurb); delay(10); }
    int sensorValue = total / 10;
    sensorValue = constrain(sensorValue, 600, 1100);
    turbidityPct = map(sensorValue, 600, 1100, 0, 100);
    if (turbidityPct > 80) turbidityClass = "JERNIH";
    else if (turbidityPct > 50) turbidityClass = "AGAK KERUH";
    else turbidityClass = "KERUH";  
    client.publish(TOPIC_TURBIDITY, turbidityClass.c_str(), false);
  }

  /* LCD */
  void updateLCD() {
    if (millis() - lcdLast < lcdInterval) return;
    lcdLast = millis();
    lcd.clear();
    switch (lcdScreen) {
      case 0:
        lcd.print("WiFi:"); lcd.setCursor(0, 1);
        lcd.print(WiFi.status() == WL_CONNECTED ? "OK " + WiFi.localIP().toString() : "DISCONNECT"); break;
      case 1:
        lcd.print("MQTT:"); lcd.setCursor(0, 1);
        lcd.print(client.connected() ? "Connected" : "DISCONNECT"); break;
      case 2: {
        DateTime now = rtc.now(); char buf[17];
        snprintf(buf, sizeof(buf), "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
        lcd.print("Time:"); lcd.setCursor(0, 1); lcd.print(buf); break;
      }
      case 3:
        lcd.print("W1: "); lcd.print(waktu1); lcd.setCursor(0, 1); lcd.print("W2: "); lcd.print(waktu2); break;
      case 4:
        lcd.print("Durasi:"); lcd.setCursor(0, 1); lcd.print(durasi); lcd.print(" s"); break;
      case 5:
        lcd.print("Pelet: "); lcd.print(pelletPercent); lcd.print("%"); break;
      case 6:
        lcd.print("Suhu: "); lcd.print(waterTemp, 1); lcd.print("C"); break;
      case 7:
        lcd.print("Kondisi Air:"); lcd.setCursor(0, 1); lcd.print(turbidityClass); break;
    }
    lcdScreen = (lcdScreen + 1) % 8;
  }

  /* Publish */
  void publishEEPROMData() {
    String msg = "Waktu 1 = " + waktu1 + ", Waktu 2 = " + waktu2 + ", Durasi: " + String(durasi) + " s";
    client.publish(TOPIC_EEPROM, msg.c_str(), true);
  }

  void publishWaktuSekarang(const String& t) { client.publish(TOPIC_WAKTU, t.c_str()); }

  void publishStatus(const char* s) { client.publish(TOPIC_STATUS, s); }

  /* Loop */
  void loop() {
    reconnectNonBlocking();
    client.loop();
    DateTime now = rtc.now();
    static int lastMinute = -1;
    if (now.minute() != lastMinute) {
      lastMinute = now.minute();
      String waktuRTC = (now.hour() < 10 ? "0" : "") + String(now.hour()) + ":" +
                        (now.minute() < 10 ? "0" : "") + String(now.minute());
      publishWaktuSekarang(waktuRTC);
      if (waktuRTC == waktu1) { eksekusiKe = 1; putarServo(); }
      else if (waktuRTC == waktu2) { eksekusiKe = 2; putarServo(); }
    }
    stopServoJikaPerlu();
    checkPelletLevel();
    readAndPublishTemp();
    checkTurbidity();
    updateLCD();

    if (millis() - lastWiFiStatusSent >= 15000) {
      lastWiFiStatusSent = millis();
      if (WiFi.status() == WL_CONNECTED && client.connected()) {
        publishWiFiStatus("online");
      }
    }
  }
