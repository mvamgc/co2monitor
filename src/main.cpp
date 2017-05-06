// #include <FS.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>

#include <SPI.h>
#include <Wire.h>
#include <BME280I2C.h>

#define BLYNK_PRINT Serial
#include <BlynkSimpleEsp8266.h>

#define I2C_SDA 5 // D1 Orange
#define I2C_SCL 4 // D2 Yellow



// int counter = 0;

WiFiManager wifiManager;

BME280I2C bme;
bool metric = true;
uint8_t pressureUnit = 1; // unit: B000 = Pa, B001 = hPa, B010 = Hg, B011 = atm, B100 = bar, B101 = torr, B110 = N/m^2, B111 = psi

void setup() {
  Serial.begin(9600);
  pinMode(BUILTIN_LED, OUTPUT);

  wifiManager.autoConnect("AutoConnectAP", "esp8266");
  Serial.println("connected...yeey :)");

  while(!bme.begin(I2C_SDA, I2C_SCL)){
    Serial.println("Could not find BME280 sensor!");
    // Serial.println(BUILTIN_LED);
    delay(2000);
  }
  // Serial.println("Hello !");
  // Serial1.begin(57600);

// Get current baud rate
// int br = Serial1.baudRate();

// Will print "Serial is 57600 bps"
// Serial1.printf("Serial is %d bps", br);
  delay(2000);
}

void loop() {
  float temp, hum, pres;
  // Serial.print("Counter: ");
  // Serial.println(counter++);
  delay(1700);
  digitalWrite(BUILTIN_LED, LOW);
  delay(300);
  digitalWrite(BUILTIN_LED, HIGH);
  bme.read(pres, temp, hum, metric, pressureUnit); // Parameters: (float& pressure, float& temp, float& humidity, bool celsius = false, uint8_t pressureUnit = 0x0)
  Serial.print("Mes: Temp: ");
  Serial.print(temp, 4);
  Serial.print(", Humidity: ");
  Serial.print(hum, 4);
  Serial.print(", Pressure: ");
  Serial.print(pres, 4);
  Serial.println("");
}