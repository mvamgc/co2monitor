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

void sendMeasurements() {
  bme.read(pres, temp, hum, metric, pressureUnit); // Parameters: (float& pressure, float& temp, float& humidity, bool celsius = false, uint8_t pressureUnit = 0x0)
  Serial.print("Mes: Temp: ");
  Serial.print(temp, 4);
  Serial.print(", Humidity: ");
  Serial.print(hum, 4);
  Serial.print(", Pressure: ");
  Serial.print(pres, 4);
  Serial.println("");
  Blynk.virtualWrite(1, temp);
  Blynk.virtualWrite(6, temp);
  Blynk.virtualWrite(2, hum);
  Blynk.virtualWrite(3, pres);
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

  timer.setInterval(10000L, sendMeasurements);
  sendMeasurements();
}

void loop() {
  Blynk.run();
  timer.run();
}

BLYNK_WRITE(V5) {
  int phonePres = param[0].asInt();
  Blynk.virtualWrite(4, phonePres);
  Serial.print("phonePres: ");
  Serial.println(phonePres);
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
