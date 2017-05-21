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

void screen_init();
void draw(float temp, float humidity, float pressure);

#define I2C_SDA 5 // D1 Orange
#define I2C_SCL 4 // D2 Yellow

// char auth[] = "abe9ae65daa04aa2a6d71947b54eac44";
char auth[] = "e02bfd9cc86e4c29805a87b707f9a409";
char server[] = "2010.io";


// int counter = 0;

WiFiManager wifiManager;

BME280I2C bme;
bool metric = true;
uint8_t pressureUnit = 1; // unit: B000 = Pa, B001 = hPa, B010 = Hg, B011 = atm, B100 = bar, B101 = torr, B110 = N/m^2, B111 = psi

SimpleTimer timer;
float temp, hum, pres;
bool bmeReady = false;
int phonePres;
bool phonePresReady = false;

bool screenOn = true;
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, I2C_SCL, I2C_SDA, U8X8_PIN_NONE);
byte x {0};
byte y {0};

float calcAltitude(int phonePres, float pres) {
  return 283 - (phonePres - pres) / 0.12;
}

void sendMeasurements() {
  bme.read(pres, temp, hum, metric, pressureUnit); // Parameters: (float& pressure, float& temp, float& humidity, bool celsius = false, uint8_t pressureUnit = 0x0)
  if(screenOn) {
    draw(temp, hum, pres);
  } else {
    u8g2.clear();
  }

  Serial.print("Mes: Temp: ");
  Serial.print(temp, 4);
  Serial.print(", Humidity: ");
  Serial.print(hum, 4);
  Serial.print(", Pressure: ");
  Serial.print(pres, 4);
  if(phonePresReady) {
    Serial.print(", Phone Pressure: ");
    Serial.print(phonePres);
  }
  Serial.println("");
  Blynk.virtualWrite(1, temp);
  // Blynk.virtualWrite(6, temp);
  Blynk.virtualWrite(2, hum);
  Blynk.virtualWrite(3, pres);
  if(phonePresReady) {
    float alt = calcAltitude(phonePres, pres);
    Blynk.virtualWrite(4, phonePres);
    Blynk.virtualWrite(6, alt);
  }
}

void setup() {
  Serial.begin(9600);



  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);
  delay(500);
  digitalWrite(BUILTIN_LED, HIGH);
  delay(500);
  digitalWrite(BUILTIN_LED, LOW);
  delay(500);

  wifiManager.autoConnect("AutoConnectAP", "esp8266");
  Serial.println("connected...yeey :)");

  while(!bme.begin(I2C_SDA, I2C_SCL)){
    Serial.println("Could not find BME280 sensor!");
    // Serial.println(BUILTIN_LED);
    delay(2000);
  }
  digitalWrite(BUILTIN_LED, HIGH);
  delay(500);
  digitalWrite(BUILTIN_LED, LOW);
  delay(500);
  digitalWrite(BUILTIN_LED, HIGH);

  Blynk.config(auth, server);

  screen_init();

  bmeReady = true;
  timer.setInterval(10000L, sendMeasurements);
  sendMeasurements();
}

void loop() {
  Blynk.run();
  timer.run();
}

BLYNK_WRITE(V5) {
  phonePres = param[0].asInt();
  phonePresReady = true;
  // Serial.print("phonePres: ");
  // Serial.println(phonePres);
}

BLYNK_WRITE(V7) {
  screenOn = param[0].asInt() == 1;
  Serial.print("screenOn: ");
  Serial.println(screenOn);
  if(screenOn) {
    draw(temp, hum, pres);
  } else {
    u8g2.clear();
  }
}

// BLYNK_READ(V1) {
//     Blynk.virtualWrite(1, temp);
// }
//
// BLYNK_READ(V2) {
//     Blynk.virtualWrite(2, hum);
// }
//
// BLYNK_READ(V3) {
//     Blynk.virtualWrite(3, pres);
// }

/*
void loop1() {
  float temp, hum, pres;
  // Serial.print("Counter: ");
  // Serial.println(counter++);
  delay(1700);
  // digitalWrite(BUILTIN_LED, LOW);
  delay(300);
  // digitalWrite(BUILTIN_LED, HIGH);
  bme.read(pres, temp, hum, metric, pressureUnit); // Parameters: (float& pressure, float& temp, float& humidity, bool celsius = false, uint8_t pressureUnit = 0x0)
  Serial.print("Mes: Temp: ");
  Serial.print(temp, 4);
  Serial.print(", Humidity: ");
  Serial.print(hum, 4);
  Serial.print(", Pressure: ");
  Serial.print(pres, 4);
  Serial.println("");
}
*/

void screen_init() {
  u8g2.begin();
  Serial.println("screen initialized");
}

void draw(float temp, float humidity, float pressure) {
  Serial.print("draw: ");
  Serial.println(temp);
  // char tempa [8];
  // sprintf(tempa, "%5f", temp);
  // °
  // dtostrf(temp, 5, 1, tempa);
  // Serial.println(tempa);

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

  // u8g2.setFont(u8g2_font_inb19_mf);
  // x = (128 - u8g2.getStrWidth(tempa))/2;
  // y = u8g2.getAscent() - u8g2.getDescent();
  // u8g2.drawStr(x, y, tempa);
  u8g2.sendBuffer();
  Serial.println("draw finished");
}
