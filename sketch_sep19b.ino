#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <TinyGPSPlus.h>   // GPS library
#include <HardwareSerial.h>

// ===== CONFIG =====
const char* ssid = "creepy 69";
const char* password = "pass@123";

#define BOT_TOKEN "8050681360:AAEojANKlfOBEAtc64JQ--4sOY41bR1kG8k"
#define CHAT_ID   "123456789"   // numeric string

// ===== Objects =====
WiFiClientSecure client;
UniversalTelegramBot bot(BOT_TOKEN, client);
Adafruit_BME680 bme; // I2C

// ===== pH Sensor =====
#define PH_PIN 34   // Analog pin for pH sensor

// ===== Moisture Sensor =====
#define MOISTURE_PIN 35   // Analog pin for Moisture sensor

// ===== GPS =====
HardwareSerial GPSserial(2);   // use UART2
TinyGPSPlus gps;

// ===== State =====
int lastMessage = 0;
unsigned long lastAutoPost = 0;
const unsigned long autoPostInterval = 10UL * 60UL * 1000UL; // auto post every 10 minutes

// ===== pH Reading Function =====
float readPH() {
  int raw = analogRead(PH_PIN);                // 0–4095
  float voltage = raw * (3.3 / 4095.0);        // Convert to voltage
  float pHValue = 7 + ((2.5 - voltage) / 0.18); // Example formula (needs calibration!)
  return pHValue;
}

// ===== Moisture Reading Function =====
String readMoisture() {
  int raw = analogRead(MOISTURE_PIN);          // 0–4095
  int percent = map(raw, 4095, 0, 0, 100);     // Map to 0–100% (adjust calibration)
  String msg = "🌱 Moisture: " + String(percent) + "% (Raw: " + String(raw) + ")";
  return msg;
}

// ===== GPS Reading Function =====
String readGPS() {
  while (GPSserial.available() > 0) {
    gps.encode(GPSserial.read());
  }

  if (gps.location.isValid()) {
    String msg = "📍 Lat: " + String(gps.location.lat(), 6) +
                 " , Lng: " + String(gps.location.lng(), 6);
    if (gps.altitude.isValid()) {
      msg += "\n⛰ Alt: " + String(gps.altitude.meters(), 2) + " m";
    }
    if (gps.satellites.isValid()) {
      msg += "\n🛰 Sats: " + String(gps.satellites.value());
    }
    return msg;
  } else {
    return "❌ No GPS fix yet. Place outside with clear sky.";
  }
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  delay(100);

  // Connect WiFi
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi connected");
  Serial.print("IP: "); Serial.println(WiFi.localIP());

  // For Telegram HTTPS
  client.setInsecure();

  // Init BME680
  if (!bme.begin(0x76)) {
    if (!bme.begin(0x77)) {
      Serial.println("❌ Could not find BME680 sensor! Check wiring.");
      while (1) delay(10);
    }
  }
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setGasHeater(320, 150);

  // Init GPS
  GPSserial.begin(9600, SERIAL_8N1, 16, 17);  // RX=16, TX=17
  Serial.println("✅ GPS serial started at 9600 baud");

  // Announce to Telegram
  bot.sendMessage(CHAT_ID, "🤖 ESP32 Bot is online! (BME680 + pH + Moisture + GPS)", "");
}

// ===== Read helper =====
String readSensor(String type) {
  if (!bme.performReading()) return "❌ Sensor read failed";
  if (type == "temperature") return String(bme.temperature, 2) + " °C";
  if (type == "humidity")    return String(bme.humidity, 2) + " %";
  if (type == "pressure")    return String(bme.pressure/100.0, 2) + " hPa";
  if (type == "gas")         return String(bme.gas_resistance/1000.0, 2) + " KΩ";
  return "Unknown";
}

// ===== Send All Readings =====
void sendAllReadings(const char* chatId) {
  String msg = "🌡 Temp: " + readSensor("temperature") + "\n";
  msg += "💧 Hum: " + readSensor("humidity") + "\n";
  msg += "🧭 Pres: " + readSensor("pressure") + "\n";
  msg += "⚡ Gas: " + readSensor("gas") + "\n";
  msg += "⚗ pH: " + String(readPH(), 2) + "\n";
  msg += readMoisture() + "\n";
  msg += readGPS();
  bot.sendMessage(chatId, msg, "");
}

// ===== Telegram handler =====
void handleTelegram() {
  int numNew = bot.getUpdates(lastMessage + 1);
  while (numNew) {
    for (int i = 0; i < numNew; i++) {
      String text = bot.messages[i].text;
      String chat_id = String(bot.messages[i].chat_id);
      lastMessage = bot.messages[i].update_id;

      Serial.println("📩 " + text);

      if (text == "/temperature") bot.sendMessage(chat_id, "🌡 Temperature: " + readSensor("temperature"), "");
      else if (text == "/humidity") bot.sendMessage(chat_id, "💧 Humidity: " + readSensor("humidity"), "");
      else if (text == "/pressure") bot.sendMessage(chat_id, "🧭 Pressure: " + readSensor("pressure"), "");
      else if (text == "/gas") bot.sendMessage(chat_id, "⚡ Gas: " + readSensor("gas"), "");
      else if (text == "/ph") bot.sendMessage(chat_id, "⚗ pH Value: " + String(readPH(), 2), "");
      else if (text == "/moisture") bot.sendMessage(chat_id, readMoisture(), "");
      else if (text == "/gps") bot.sendMessage(chat_id, readGPS(), "");
      else if (text == "/all") sendAllReadings(chat_id.c_str());
      else bot.sendMessage(chat_id, "Unknown. Use: /temperature /humidity /pressure /gas /ph /moisture /gps /all", "");
    }
    numNew = bot.getUpdates(lastMessage + 1);
  }
}

// ===== Loop =====
void loop() {
  handleTelegram();

  // Auto-post every interval
  if (millis() - lastAutoPost > autoPostInterval) {
    lastAutoPost = millis();
    sendAllReadings(CHAT_ID);
  }

  delay(1000);
}

