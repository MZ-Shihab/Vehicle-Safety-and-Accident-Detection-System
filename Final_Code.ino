/*
  ESP32 WROOM-32 Accident Detection & Emergency System v2.3.6 (with OTA)
  - Pinout and connections based on schematic
  - Compatible with Arduino IDE 2.x (ESP32 board selected)
  - For ESP32 only (not AVR/Uno/Nano!)
  - Includes OTA upload support via WiFi

  Components:
    - MPU6050 (I2C)
    - Neo-6M GPS (UART)
    - SIM800L GSM (UART)
    - RFID RC522 (SPI)
    - IR Sensor (Digital Input)
    - 0.96" OLED (I2C, SSD1306)
    - Buzzer (Digital Output)

  ESP32 WROOM-32 Accident Detection & Emergency System v2.3.6 (with OTA & Telegram)
  MODIFIED: 
    - Alerts via SIM & Telegram
    - RFID whitelist single card
    - OLED status always updated
    - Initial GPS fix 30s only
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include <MPU6050.h>
#include <MFRC522.h>
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>     // <-- MOD: For Telegram HTTP request
#include <ArduinoOTA.h>

// ------------ Pin Definitions (ESP32 Only!) -------------
#define BUZZER_PIN      27   // GPIO27
#define IR_SENSOR_PIN   34   // GPIO14

// OLED display - I2C
#define OLED_SDA        21
#define OLED_SCL        22
#define OLED_RESET_PIN  -1
#define SCREEN_ADDRESS  0x3C

// RFID RC522 - SPI
#define RFID_SS_PIN     5    // GPIO5 (SDA/SS)
#define RFID_RST_PIN    4    // GPIO4 (RST)
#define RFID_MOSI_PIN   23   // GPIO23 (MOSI)
#define RFID_MISO_PIN   19   // GPIO19 (MISO)
#define RFID_SCK_PIN    18   // GPIO18 (SCK)

// MPU6050 - I2C (shared with OLED)
#define MPU6050_SDA     21
#define MPU6050_SCL     22

// GPS (NEO-6M) - UART2
#define GPS_RX_PIN      16   // ESP32 RX2 <- GPS TX (GPIO16)
#define GPS_TX_PIN      17   // ESP32 TX2 -> GPS RX (GPIO17)

// ----------- WiFi Credentials -------------
#define WIFI_SSID     "512"
#define WIFI_PASS     "password"

// ----------- Telegram BOT Config -------------
// MOD: Enter your bot token and chat id here
#define TELEGRAM_BOT_TOKEN "8367869574:AAFUhywzLwY2bWh5RsxrUCz9m1dY0H1xP4"
#define TELEGRAM_CHAT_ID   "591116478"

// ----------- Global Objects -------------
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET_PIN);
TinyGPSPlus gps;
MPU6050 mpu;
MFRC522 mfrc522(RFID_SS_PIN, RFID_RST_PIN);

HardwareSerial GPS_Serial(2);   // 2 = UART2 (pins 16, 17)
HardwareSerial &sim800l = Serial;

// ---------- Configurable Data -----------
String phone1 = "+88018********";
String phone2 = "+88013********";
String overspeedPhone1 = "+88018********";
String overspeedPhone2 = "+88017********";
String vehicleNumber = "Dhaka Metro GA-11-1234";
unsigned int speedLimit = 100.0; // km/h
float wheelDiameter = 0.04; // meters

// MOD: Whitelisted RFID card UID (example)
byte allowedUid[4] = {0xDE, 0xAD, 0xBE, 0xEF}; // <-- Change this (example UID)
String allowedUidStr = "deadbeef";             // Lowercase hex string

// ----------- State Variables ------------
double initLat = 23.78, initLng = 90.28;
double currLat = 0.0, currLng = 0.0;
bool gpsReady = false, simReady = false;
bool buzzerActive = false;
unsigned long buzzerEndTime = 0;
int overSpeedCount = 0;
float currSpeed = 0.0;
float ax = 0.0, ay = 0.0, az = 0.0, gx = 0.0, gy = 0.0, gz = 0.0;

// ----------- Speed Measurement Variables ------------
volatile int irPulseCount = 0;     // Number of dark pulses in 2s window
unsigned long irWindowStart = 0;   // 2s window start time
float lastMeasuredSpeed = 0.0;     // Last computed speed (km/h)

// ----------- Function Prototypes --------
void initializeSystem();
void beep(int times, int duration, int pause);
void sendSMS(const String &number, const String &message);
void sendTelegram(const String &message);          // MOD: Forward to Telegram
void makeCall(const String &number);
void fetchGPS();
void processAccident();
void processRFID();
void processSpeed();
void showOLED(const String &str1 = "", const String &str2 = "", const String &str3 = "",
              const String &str4 = "", const String &str5 = "", const String &str6 = "", const String &str7 = "");
String generateCaseNumber();
String formatLatLng();
void updateSpeedMeasurement();
float getCurrentSpeed(bool inKmh = false);
void setupIRSpeedInterrupt();

void printOledStatus();   // MOD: Regular status to OLED

// ----------- OTA Setup -------------
void setupOTA() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  showOLED("Connecting WiFi...", WIFI_SSID, "", "", "", "", "");
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    showOLED("WiFi Failed!", "Retrying...", "", "", "", "", "");
    delay(3000);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
  showOLED("WiFi Connected!", WiFi.localIP().toString(), "", "", "", "", "");
  delay(1000);

  ArduinoOTA.setHostname("esp32-accident-system");
  ArduinoOTA.onStart([]() {
    showOLED("OTA Update", "Start...", "", "", "", "", "");
  });
  ArduinoOTA.onEnd([]() {
    showOLED("OTA Update", "End", "", "", "", "", "");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    showOLED("OTA Error!", String(error), "", "", "", "", "");
  });
  ArduinoOTA.begin();
}

// ----------- Setup ----------------------
void setup() {
  Serial.begin(9600); // SIM800L on Serial0

  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // IR Sensor
  pinMode(IR_SENSOR_PIN, INPUT_PULLUP);
  setupIRSpeedInterrupt();

  // OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();
  display.display();
  showOLED("System Initiating...", "OLED Ready", "", "", "", "", "");
  delay(500);

  // RFID - SPI
  SPI.begin(RFID_SCK_PIN, RFID_MISO_PIN, RFID_MOSI_PIN, RFID_SS_PIN);
  mfrc522.PCD_Init();
  showOLED("System Initiating...", "OLED Ready", "RFID Ready", "", "", "", "");
  delay(500);

  // MPU6050
  mpu.initialize();
  showOLED("System Initiating...", "OLED Ready", "RFID Ready", "MPU6050 Ready", "", "", "");
  delay(500);

  // NEO-6M GPS on UART2
  GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // OTA WiFi
  setupOTA();

  initializeSystem();
}

// ----------- Main Loop ------------------
void loop() {
  ArduinoOTA.handle(); // --- OTA Handler ---

  // --------- 1. Continue reading GPS ---------
  while (GPS_Serial.available()) {
    gps.encode(GPS_Serial.read());
    if (gps.location.isUpdated()) {
      currLat = gps.location.lat();
      currLng = gps.location.lng();
      gpsReady = true;
    }
  }
  String latStr = "Lat: " + String(currLat, 6);
  String lngStr = "Lng: " + String(currLng, 6);

  // --------- 2. Speed Measuring via IR ---------
  updateSpeedMeasurement();
  currSpeed = getCurrentSpeed(true); // km/h
  String speedStr = "Speed = " + String(currSpeed, 2) + " km/h";

  // --------- 3. Position Reading ---------
  int16_t ax_raw, ay_raw, az_raw;
  int16_t gx_raw, gy_raw, gz_raw;
  mpu.getAcceleration(&ax_raw, &ay_raw, &az_raw);
  mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);

  ax = ax_raw / 16384.0;
  ay = ay_raw / 16384.0;
  az = az_raw / 16384.0;
  gx = gx_raw / 131.0;
  gy = gy_raw / 131.0;
  gz = gz_raw / 131.0;

  // --------- OLED Status ----------------
  printOledStatus(); // MOD: Regular status update

  // --------- 4. Accident Detection ---------
  if ((abs(ax) > 15 && abs(ay) > 15 && abs(az) > 15) || abs(gx) > 90 || abs(gy) > 90 || abs(gz) > 90) {
    processAccident();
    delay(1000);
  }

  // --------- 5. Speed Monitoring ---------
  processSpeed();

  // --------- 6. RFID Emergency Trigger ---------
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    processRFID();
    delay(1000);
  }

  // --------- 7. Buzzer handling ---------
  if (buzzerActive && millis() > buzzerEndTime) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerActive = false;
  }

  delay(10);
}

// ----------- Initialization -------------
void initializeSystem() {
  simReady = false;
  sim800l.println("AT");
  delay(500);
  if (sim800l.find("OK")) {
    simReady = true;
    showOLED("System Initiating...", "SIM800L Ready", "", "", "", "", "");
    delay(500);
  } else {
    showOLED("System Initiating...", "SIM800L error...", "", "", "", "", "");
    delay(1000);
  }

  // --- GPS INIT WITH 30s COUNTDOWN --- MODIFIED
  uint32_t startMs = millis();
  bool gotFix = false;

  while (millis() - startMs < 30000) {  // MOD: Only 30 seconds
    while (GPS_Serial.available()) {
      gps.encode(GPS_Serial.read());
      if (gps.location.isValid() && gps.location.age() < 5000) {
        currLat = gps.location.lat();
        currLng = gps.location.lng();
        initLat = currLat;
        initLng = currLng;
        gpsReady = true;
        gotFix = true;
        break;
      }
    }
    int secondsLeft = 30 - ((millis() - startMs) / 1000);
    if (secondsLeft < 0) secondsLeft = 0;
    char msg[22];
    snprintf(msg, sizeof(msg), "GPS Wait: %2ds", secondsLeft);
    showOLED("Waiting for GPS...", msg, "", "", "", "", "");
    if (gotFix) break;
    delay(200);
  }

  String latStr = "Lat: " + String(currLat, 6);
  String lngStr = "Lng: " + String(currLng, 6);

  if (gotFix) {
    showOLED("GPS Ready", latStr, lngStr, "", "", "", "");
    Serial.println("GPS Ready");
    Serial.println(latStr);
    Serial.println(lngStr);
  } else {
    currLat = initLat;
    currLng = initLng;
    showOLED("GPS not Ready", latStr, lngStr, "", "", "", "");
    Serial.println("GPS not Ready. Using default location.");
    Serial.println(latStr);
    Serial.println(lngStr);
  }

  beep(2, 100, 100);
  showOLED("System Ready", latStr, lngStr, "", "", "", "");
}

// ----------- Beep Function --------------
void beep(int times, int duration, int pause) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(duration);
    digitalWrite(BUZZER_PIN, LOW);
    delay(pause);
  }
}

// ----------- Send SMS -------------------
void sendSMS(const String &number, const String &message) {
  sim800l.println("AT+CMGF=1");
  delay(100);
  sim800l.print("AT+CMGS=\"");
  sim800l.print(number);
  sim800l.println("\"");
  delay(100);
  sim800l.print(message);
  sim800l.write(26); // CTRL+Z
  delay(3000);
}

// ----------- Send Telegram -------------
// MOD: Forward alert message to Telegram via HTTP POST
void sendTelegram(const String &message) {
  if (WiFi.status() != WL_CONNECTED) return;
  HTTPClient http;
  // Set up URL (Telegram sendMessage endpoint)
  String url = "https://api.telegram.org/bot" TELEGRAM_BOT_TOKEN "/sendMessage?chat_id=" TELEGRAM_CHAT_ID "&text=";
  url += urlencode(message);

  http.begin(url);
  int httpResponseCode = http.GET();
  http.end();
}

// Utility for safe URL encoding in Telegram message (basic)
String urlencode(const String &msg) {
  String encodedMsg = "";
  char c;
  char code0;
  char code1;
  for (int i = 0; i < msg.length(); i++) {
    c = msg.charAt(i);
    if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
      encodedMsg += c;
    } else if (c == ' ') {
      encodedMsg += '+';
    } else {
      code1 = (c & 0xf) + '0';
      if ((c & 0xf) > 9) code1 = (c & 0xf) - 10 + 'A';
      c = (c >> 4)&0xf;
      code0 = c + '0';
      if (c > 9) code0 = c - 10 + 'A';
      encodedMsg += '%';
      encodedMsg += code0;
      encodedMsg += code1;
    }
  }
  return encodedMsg;
}

// ----------- Make Call ------------------
void makeCall(const String &number) {
  sim800l.print("ATD");
  sim800l.print(number);
  sim800l.println(";");
  delay(20000); // 20 seconds
  sim800l.println("ATH"); // Hang up
}

// ----------- Fetch GPS ------------------
void fetchGPS() {
  unsigned long start = millis();
  bool gotGPS = false;
  while (millis() - start < 3000) {
    while (GPS_Serial.available()) {
      gps.encode(GPS_Serial.read());
      if (gps.location.isUpdated()) {
        currLat = gps.location.lat();
        currLng = gps.location.lng();
        initLat = currLat;
        initLng = currLng;
        gotGPS = true;
      }
    }
    if (gotGPS) break;
    delay(10);
  }
  if (!gotGPS && !gpsReady) {
    currLat = initLat;
    currLng = initLng;
  }
}

String formatLatLng() {
  String out = String(currLat, 6) + "," + String(currLng, 6);
  return out;
}

// -------- OLED Display Helper ------------
void showOLED(const String &str1, const String &str2, const String &str3,
              const String &str4, const String &str5, const String &str6, const String &str7) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  auto centerX = [](const String &s) { return (128 - (s.length() * 6)) / 2; };
  auto rightX  = [](const String &s) { return 128 - (s.length() * 6) - 2; };
  auto leftX   = [](const String &) { return 2; };

  int y = 0;
  if (str1.length()) { 
    display.setCursor(centerX(str1), y); 
    display.print(str1); 
  }
  y += 16;
  if (str2.length()) { 
    display.setCursor(leftX(str2), y); 
    display.print(str2); 
  }
  y += 8;
  if (str3.length()) { 
    display.setCursor(leftX(str3), y); 
    display.print(str3); 
  }
  y += 8;
  if (str4.length()) { 
    display.setCursor(leftX(str4), y); 
    display.print(str4); 
  }
  y += 8;
  if (str5.length()) { 
    display.setCursor(rightX(str5), y); 
    display.print(str5); 
  }
  y += 8;
  if (str6.length()) { 
    display.setCursor(rightX(str6), y); 
    display.print(str6); 
  }
  y += 8;
  if (str7.length()) { 
    display.setCursor(rightX(str7), y); 
    display.print(str7); 
  }

  display.display();
  Serial.println(str1);
}

// ------- REGULAR OLED STATUS --------
// MOD: Regularly update OLED with current readings
void printOledStatus() {
  String latStr = "Lat: " + String(currLat, 6);
  String lngStr = "Lng: " + String(currLng, 6);
  String speedStr = "Speed = " + String(currSpeed, 2) + " km/h";
  String x = "x = " + String(gx, 3);
  String y = "y = " + String(gy, 3);
  String z = "z = " + String(gz, 3);
  showOLED("Monitoring...", latStr, lngStr, speedStr, x, y, z);
}

// ----------- Accident Handler -----------
void processAccident() {
  fetchGPS();
  String latStr = "Lat: " + String(currLat, 6);
  String lngStr = "Lng: " + String(currLng, 6);
  String SpeedStr = "Speed = " + String(currSpeed, 2) + " km/h";

  String alertText = "URGENT: Accident detected. Check driver. Location: https://maps.google.com/?q=" + formatLatLng();

  showOLED("Accident Detected", latStr, lngStr, SpeedStr, "", "", "");
  digitalWrite(BUZZER_PIN, HIGH);
  buzzerActive = true;
  buzzerEndTime = millis() + 10000;
  delay(1000);

  // Send Alerts via SMS and Telegram
  showOLED("Sending SMS...", latStr, lngStr, SpeedStr, "", "", "");
  sendSMS(phone1, alertText);
  sendSMS(phone2, alertText);
  sendTelegram(alertText); // MOD

  delay(1000);

  showOLED("Calling...", latStr, lngStr, SpeedStr, "", "", "");
  makeCall(phone1);
  makeCall(phone2);

  showOLED("Alert Sent", latStr, lngStr, SpeedStr, "", "", "");
  delay(1000);
}

// ----------- RFID Handler ---------------
void processRFID() {
  String uid = "";
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    if (mfrc522.uid.uidByte[i] < 0x10) uid += "0";
    uid += String(mfrc522.uid.uidByte[i], HEX);
  }
  uid.toLowerCase();

  // MOD: Only allow specific card UID
  if (uid != allowedUidStr) {
    showOLED("Unauthorized Card", uid, "", "", "", "", "");
    beep(2, 150, 60);
    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();
    delay(1000);
    return; // <--- REJECT other cards
  }

  fetchGPS();
  String latStr = "Lat: " + String(currLat, 6);
  String lngStr = "Lng: " + String(currLng, 6);
  String SpeedStr = "Speed = " + String(currSpeed, 2) + " km/h";

  showOLED(uid + " ID Detected", latStr, lngStr, SpeedStr, "", "", "");
  delay(1000);

  String rfidSMS = "WARNING: Driver triggered emergency. Location: https://maps.google.com/?q=" + formatLatLng();
  showOLED("Sending SMS...", latStr, lngStr, SpeedStr, "", "", "");
  sendSMS(phone1, rfidSMS);
  sendSMS(phone2, rfidSMS);
  sendTelegram(rfidSMS); // MOD
  delay(1000);

  showOLED("Calling...", latStr, lngStr, SpeedStr, "", "", "");
  makeCall(phone1);
  makeCall(phone2);
  delay(1000);

  showOLED("RFID Alert Sent", latStr, lngStr, SpeedStr, "", "", "");
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
  delay(1000);
}

// ----------- Speed Handler --------------
void processSpeed() {
  if (currSpeed > speedLimit) {
    overSpeedCount++;
    String latStr = "Lat: " + String(currLat, 6);
    String lngStr = "Lng: " + String(currLng, 6);
    String SpeedStr = "Speed = " + String(currSpeed, 2) + " km/h";

    if (overSpeedCount < 3) {
      showOLED("Overspeed Warning!!", latStr, lngStr, SpeedStr, "Warn = " + String(overSpeedCount), "", "");
      beep(1, 1000, 200);
      String sms2 = "CAUTION: Overspeed Detected.\nSpeed: " + String(currSpeed, 1) + " km/h";
      sendSMS(overspeedPhone2, sms2);
      sendTelegram(sms2); // MOD
    } 
    else if (overSpeedCount == 3) {
      showOLED("Overspeed Report", latStr, lngStr, SpeedStr, "Warn = " + String(overSpeedCount), "", "");
      fetchGPS();
      String caseNum = generateCaseNumber();
      String sms1 = "NOTICE: Overspeed case filed.\nSpeed: " + String(currSpeed, 1) +
                    " km/h.\nCase#: " + caseNum + ".\nLocation: https://maps.google.com/?q=" + formatLatLng();
      sendSMS(overspeedPhone1, sms1);
      sendTelegram(sms1); // MOD
    }
  }
}

// ----------- Speed Measurement with Interrupt --------------
void IRAM_ATTR irISR() {
  irPulseCount++;
}

void setupIRSpeedInterrupt() {
  attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), irISR, FALLING);
  irWindowStart = millis();
}

void updateSpeedMeasurement() {
  unsigned long now = millis();
  if (now - irWindowStart >= 2000) { // 2s window elapsed
    float distance = irPulseCount * (PI * wheelDiameter / 2.0f); // meters
    float speed_ms = distance / 2.0f;        // m/s
    lastMeasuredSpeed = speed_ms * 3.6f;     // km/h
    irPulseCount = 0;
    irWindowStart = now;
  }
}

float getCurrentSpeed(bool inKmh) {
  if (inKmh) return lastMeasuredSpeed;
  return lastMeasuredSpeed / 3.6f;
}

// ----------- Random Case Number ---------
String generateCaseNumber() {
  String s = "";
  for (int i = 0; i < 13; i++) s += String(random(0, 10));
  return s;
}

/*
 * ------- HOW TO EDIT SMS, NUMBERS, VEHICLE -------
 * - To update phone numbers, edit phone1, phone2, overspeedPhone1, overspeedPhone2.
 * - To update message contents, edit the SMS string variables.
 * - To update vehicle number, edit vehicleNumber.
 * - To change speed limit, edit speedLimit.
 * - To set WiFi for OTA, edit WIFI_SSID and WIFI_PASS.
 * - To allow a specific RFID, set allowedUid/allowedUidStr.
 * - To set Telegram, set TELEGRAM_BOT_TOKEN and TELEGRAM_CHAT_ID.
 * 
 * NOTE: For ESP32 only! Select board: ESP32 Dev Module in Arduino IDE.
 * To use OTA: After uploading once by USB, use "Upload via Network" in Arduino IDE.
 */
