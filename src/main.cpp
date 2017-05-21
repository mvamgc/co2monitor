// #include <FS.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>

#include <SPI.h>
#include <Wire.h>
#include <BME280I2C.h>

#include <SimpleTimer.h>

#define BLYNK_PRINT Serial
#include <BlynkSimpleEsp8266.h>

#include <U8g2lib.h>

void drawMessage(char const *msg);
void drawMessageLF(char const *msg);
void draw(float temp, float humidity, float pressure);
void espInfo();

#define I2C_SDA 5
#define I2C_SCL 4

ADC_MODE(ADC_VCC);

char auth[] = "e02bfd9cc86e4c29805a87b707f9a409";
char server[] = "2010.io";

WiFiManager wifiManager;

BME280I2C bme;

WidgetTerminal terminal(V10);

bool metric = true;
uint8_t pressureUnit = 1; // unit: B000 = Pa, B001 = hPa, B010 = Hg, B011 = atm, B100 = bar, B101 = torr, B110 = N/m^2, B111 = psi

SimpleTimer timer;
float temp, hum, pres;
bool bmeReady = false;

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, I2C_SCL, I2C_SDA, U8X8_PIN_NONE);
byte x {0};
byte y {0};
bool screenOn = true;

void sendMeasurements() {
  bme.read(pres, temp, hum, metric, pressureUnit); // Parameters: (float& pressure, float& temp, float& humidity, bool celsius = false, uint8_t pressureUnit = 0x0)
  if(screenOn) {
    draw(temp, hum, pres);
  }
  // draw(temp, hum, pres);

  Serial.print("Mes: Temp: ");
  Serial.print(temp, 4);
  Serial.print(", Humidity: ");
  Serial.print(hum, 4);
  Serial.print(", Pressure: ");
  Serial.print(pres, 4);
  Serial.println("");
  Blynk.virtualWrite(1, temp);
  // Blynk.virtualWrite(6, temp);
  Blynk.virtualWrite(2, hum);
  Blynk.virtualWrite(3, pres);
}

void setup() {
  Serial.begin(9600);
  u8g2.begin();

  drawMessageLF("Start");

  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);
  delay(500);
  digitalWrite(BUILTIN_LED, HIGH);
  delay(500);
  digitalWrite(BUILTIN_LED, LOW);
  delay(500);

  wifiManager.autoConnect("AutoConnectAP", "esp8266");
  Serial.println("connected...yeey :)");
  drawMessage("WiFi Connected");

  while(!bme.begin(I2C_SDA, I2C_SCL)){
    Serial.println("Could not find BME280 sensor!");
    // Serial.println(BUILTIN_LED);
    delay(2000);
  }
  drawMessage("BME initialized");
  digitalWrite(BUILTIN_LED, HIGH);
  delay(500);
  digitalWrite(BUILTIN_LED, LOW);
  delay(500);
  digitalWrite(BUILTIN_LED, HIGH);

  Blynk.config(auth, server);

  drawMessage("Blynk configured");

  bmeReady = true;
  timer.setInterval(15000L, sendMeasurements);
  drawMessage("Timer started");
  sendMeasurements();
}

void loop() {
  Blynk.run();
  timer.run();
}

BLYNK_WRITE(V7) {
  screenOn = param[0].asInt() == 1;
  if(screenOn) {
    draw(temp, hum, pres);
  } else {
    u8g2.clear();
  }
}

BLYNK_WRITE(V10) {
  const char * cmdc = param[0].asStr();
  Serial.println("V10 cmd received: ");
  String cmd = String(cmdc);
  Serial.println(cmd);
  if(cmd == "scron") {
    screenOn = true;
    draw(temp, hum, pres);
  } else if(cmd == "scroff") {
    screenOn = false;
    u8g2.clear();
  }
}

void draw(float temp, float humidity, float pressure) {
  Serial.print("draw: ");
  u8g2.clearBuffer();

  if(screenOn) {
    const char degree {176};
    String measurement;

    u8g2.setFont(u8g2_font_9x18_mf);
    // u8g2.setFont(u8g2_font_inb19_mf);
    byte h = u8g2.getAscent() - u8g2.getDescent() + 2;
    char buf[12];

    measurement = "T:" + String(temp) + "" + degree + "C";
    measurement.toCharArray(buf, 12);
    Serial.println(buf);
    x = (128 - u8g2.getStrWidth(buf))/2;
    y = h;
    u8g2.drawStr(x, y, buf);

    measurement = "H:" + String(humidity) + "%";
    measurement.toCharArray(buf, 12);
    x = (128 - u8g2.getStrWidth(buf))/2;
    y += h;
    u8g2.drawStr(x, y, buf);

    measurement = "P:" + String(pres) + "hPa";
    measurement.toCharArray(buf, 12);
    x = (128 - u8g2.getStrWidth(buf))/2;
    y += h;
    u8g2.drawStr(x, y, buf);
  }
  u8g2.sendBuffer();
}

void drawMessageLF(char const *msg) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_inb19_mf);
  x = (128 - u8g2.getStrWidth(msg))/2;
  u8g2.drawStr(x, 30, msg);
  u8g2.sendBuffer();
}

void drawMessage(char const *msg) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_9x18_mf);
  x = (128 - u8g2.getStrWidth(msg))/2;
  u8g2.drawStr(x, 30, msg);
  u8g2.sendBuffer();
}
