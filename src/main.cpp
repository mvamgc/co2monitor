// #include <FS.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>

#include <SPI.h>
#include <Wire.h>
#include <BME280I2C.h>

// SW Serial
#include <SoftwareSerial.h>

#include <SimpleTimer.h>

#include <U8g2lib.h>

#define BLYNK_PRINT Serial
#include <BlynkSimpleEsp8266.h>

#define I2C_SDA 5
#define I2C_SCL 4

#define SENSOR_SERIAL swSer

SoftwareSerial swSer(13, 15, false, 256); // GPIO15 (TX) and GPIO13 (RX)
byte cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};
unsigned char response[7];

WiFiManager wifiManager;

BME280I2C bme;

SimpleTimer readTimer;
SimpleTimer readCO2Timer;
SimpleTimer sendTimer;

// Blynk data
char auth[] = "e02bfd9cc86e4c29805a87b707f9a409";
char server[] = "2010.io";
WidgetTerminal terminal(V10);

// Measured data
unsigned int co2 = 0;
float temp, hum, pres;
bool bmeReady = false;
bool co2Ready = false;

float tempG[1280];

// u8g2
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, I2C_SCL, I2C_SDA, U8X8_PIN_NONE);
bool screenOn = true;

// ------------------------- Screen -------------------------

void draw(unsigned int co2, float temp, float humidity, float pressure) {
  Serial.print("draw: ");
  u8g2.clearBuffer();

  if(screenOn) {
    byte x {0};
    byte y {0};

    const char degree {176};
    String measurement;

    u8g2.setFont(u8g2_font_9x18_mf);
    // u8g2.setFont(u8g2_font_inb19_mf);
    byte h = u8g2.getAscent() - u8g2.getDescent() + 2;
    char buf[20];

    y = 0;

    if(co2Ready) {
      measurement = "CO2:" + String(co2) + " ppm";
    } else {
      measurement = "CO2:~" + String(co2) + " ppm";
    }

    measurement.toCharArray(buf, 20);
    Serial.println(buf);
    x = (128 - u8g2.getStrWidth(buf))/2;
    y += h;
    u8g2.drawStr(x, y, buf);

    measurement = "T:" + String(temp) + "" + degree + "C";
    measurement.toCharArray(buf, 12);
    x = (128 - u8g2.getStrWidth(buf))/2;
    y += h;
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
  byte x = (128 - u8g2.getStrWidth(msg))/2;
  u8g2.drawStr(x, 30, msg);
  u8g2.sendBuffer();
}

void drawMessage(char const *msg) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_9x18_mf);
  byte x = (128 - u8g2.getStrWidth(msg))/2;
  u8g2.drawStr(x, 30, msg);
  u8g2.sendBuffer();
}

void drawMeasurements() {
  if(screenOn) {
    draw(co2, temp, hum, pres);
  }
}

// ------------------------- Blynk -------------------------
void sendMeasurements() {
  if(bmeReady) {
    Blynk.virtualWrite(1, temp);
    Blynk.virtualWrite(2, hum);
    Blynk.virtualWrite(3, pres);
    Blynk.virtualWrite(9, co2);
    if(co2Ready) {
      Blynk.virtualWrite(8, co2);
    }
    Blynk.virtualWrite(17, millis());
  }
}

BLYNK_WRITE(V7) {
  screenOn = param[0].asInt() == 1;
  if(screenOn) {
    draw(co2, temp, hum, pres);
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
    draw(co2, temp, hum, pres);
    terminal.println("Screen on");
  } else if(cmd == "scroff") {
    screenOn = false;
    u8g2.clear();
    terminal.println("Screen off");
  } else {
    terminal.println("Supported commands:\nscron\nscroff");
  }
  terminal.flush();
}

void readCO2() {
  bool header_found {false};
  char tries {0};

  SENSOR_SERIAL.write(cmd, 9);
  memset(response, 0, 7);

    // Looking for packet start
  while(SENSOR_SERIAL.available() && (!header_found)) {
    if(SENSOR_SERIAL.read() == 0xff ) {
      if(SENSOR_SERIAL.read() == 0x86 ) {
        header_found = true;
      }
    }
  }

  if (header_found) {
    SENSOR_SERIAL.readBytes(response, 7);

    byte crc = 0x86;
    for (char i = 0; i < 6; i++) {
      crc+=response[i];
    }
    crc = 0xff - crc;
    crc++;

    if ( !(response[6] == crc) ) {
      Serial.println("CO2: CRC error: " + String(crc) + " / "+ String(response[6]));
    } else {
      unsigned int responseHigh = (unsigned int) response[0];
      unsigned int responseLow = (unsigned int) response[1];
      unsigned int ppm = (256*responseHigh) + responseLow;
      co2 = ppm;
      Serial.print("CO2: " + String(co2));
    }
  } else {
    Serial.println("CO2: Header not found");
  }
  if(!co2Ready && millis() > 3 * 60 * 1000) { //if(millis() > 3 * 60 * 1000 && co2 > 1 && co2 < 4999) {
    co2Ready = true;
  }
  Serial.print(", ready: ");
  Serial.println(co2Ready);
}

// ------------------------- BME280 -------------------------
void readMeasurements() {
  uint8_t pressureUnit = 1; // unit: B000 = Pa, B001 = hPa, B010 = Hg, B011 = atm, B100 = bar, B101 = torr, B110 = N/m^2, B111 = psi
  bme.read(pres, temp, hum, true, pressureUnit); // Parameters: (float& pressure, float& temp, float& humidity, bool celsius = false, uint8_t pressureUnit = 0x0)
  bmeReady = true;
  Serial.print(millis() / 1000);
  Serial.print(" - ");
  Serial.print("CO₂: ");
  Serial.print(co2);
  Serial.print(", Temp: ");
  Serial.print(temp, 4);
  Serial.print(", Humidity: ");
  Serial.print(hum, 4);
  Serial.print(", Pressure: ");
  Serial.print(pres, 4);
  Serial.println("");
  // sendMeasurements();
  drawMeasurements();
}

void setup() {
  SENSOR_SERIAL.begin(9600);
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
  readMeasurements();
  readCO2Timer.setInterval(30000L, readCO2);
  readTimer.setInterval(5000L, readMeasurements);
  sendTimer.setInterval(30000L, sendMeasurements);
  drawMessage("Timer started");
}

void loop() {
  Blynk.run();
  readTimer.run();
  readCO2Timer.run();
  sendTimer.run();
}
