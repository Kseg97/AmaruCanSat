 /****************************************************************
*  Sensors + NRF24L01+ + ESP32 Demo + JSON
*  
*  SHT21: https://github.com/markbeee/SHT21 | C++ files appended
*  MPU9250: https://github.com/hideakitai/MPU9250 | C++ files appended
*  RF24 (NRF24L01): https://medium.com/@anujdev11/communication-between-arduino-and-raspberry-pi-using-nrf24l01-818687f7f363 | Installed
*  BMP280: https://github.com/adafruit/Adafruit_BMP280_Library | C++ files appended (mod)
*  An example sketch that reads the GY-91 (MPU9250 + BMP280), SHT21, 
*  GUVA-S12SD to the NRF24L01+ module.
*  Measures Accel, Gyro, Pressure, Relative Humidity, Temperature, 
*  UV Radiation and Air Quality 
*  SHT21, MPU9250 --> 21: SDA, 22: SCL 
*  NRF24L01+ --> MOSI: 23, MISO: 22, SCK: 18, CSN: 5, CE: 15
*  
*  @TODO: Add capacitors and use a solid circuit board for all
*  @TODO: Round output data
*  @TODO: Improve performance
* 
***************************************************************/
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Wire.h>
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include "SHT21.h"
#include "MPU9250.h"
#include "Adafruit_BMP280.h"
#include <math.h>

RF24 radio(15,5); // CE, CSN
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL }; // Receiver / Transmitter

#define PIN_GUVA 4
#define BMP280_ADDRESS (0x76)

SHT21 SHT21;
MPU9250 mpu;
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

const int interval = 500; // ms

void setup() {
  SHT21.begin();
  
  Serial.begin(115200);
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1) delay(10);
  }
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  
  radio.begin();

  // enable dynamic payloads
  radio.enableDynamicPayloads();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(5,15);

  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);

  radio.startListening();

  radio.printDetails();

  mpu.setup();
  bmp_temp->printSensorDetails();

  delay(2000);
}

void loop() {
  static uint32_t prev_ms = millis();
  if ((millis() - prev_ms) > interval) {
    Serial.println("Sending packet: ");
  
    // First, stop listening so we can talk.
    radio.stopListening();

    // Read sensors
    mpu.update();

    sensors_event_t temp_event, pressure_event;
    bmp_temp->getEvent(&temp_event);
    bmp_pressure->getEvent(&pressure_event);

    // Take the time, and send it.  This will block until complete
    Serial.print(F("Sending"));

    String s = getJSON("tmp", String(SHT21.getTemperature())) + " ";
    char copy[32];
    s.toCharArray(copy, s.length());
    radio.write(copy, s.length());

    s = getJSON("hum", String(SHT21.getHumidity())) + " ";
    s.toCharArray(copy, s.length());
    radio.write(copy, s.length());
/*
    s = getJSON("tpi", String(temp_event.temperature)) + " ";
    s.toCharArray(copy, s.length());
    radio.write(copy, s.length());
*/
    s = getJSON("prs", String(bmp.readAltitude(1024.25))) + " ";
    s.toCharArray(copy, s.length());
    radio.write(copy, s.length());
/*
    s = getJSON("acc", "[" + String(round(mpu.getAcc(0))) + "," + String(round(mpu.getAcc(1))) + "," + String(round(mpu.getAcc(2))) + "]") + " ";
    s.toCharArray(copy, s.length());
    radio.write(copy, s.length());
*/
    /*s = getJSON("gyr", "[" + String(round(mpu.getGyro(0))) + "," + String(round(mpu.getGyro(1))) + "," + String(round(mpu.getGyro(2))) + "]") + " ";
    s.toCharArray(copy, s.length());
    radio.write(copy, s.length());

    s = getJSON("mag", "[" + String(round(mpu.getMag(0))) + "," + String(round(mpu.getMag(1))) + "," + String(round(mpu.getMag(2))) + "]") + " ";
    s.toCharArray(copy, s.length());
    radio.write(copy, s.length());*/

    s = getJSON("uv", String(analogRead(PIN_GUVA))) + " ";
    s.toCharArray(copy, s.length());
    radio.write(copy, s.length());

    /*
    doc["hum"] = SHT21.getHumidity(); // Humidity
    doc["acc"] = "[" + String(mpu.getAcc(0)) + "," + String(mpu.getAcc(1)) + "," + String(mpu.getAcc(2)) + "]";
    doc["gyr"] = "[" + String(mpu.getGyro(0)) + "," + String(mpu.getGyro(1)) + "," + String(mpu.getGyro(2)) + "]";
    doc["mag"] = "[" + String(mpu.getMag(0)) + "," + String(mpu.getMag(1)) + "," + String(mpu.getMag(2)) + "]";
    doc["rpy"] = "[" + String(mpu.getRoll()) + "," + String(mpu.getPitch()) + "," + String(mpu.getYaw()) + "]";
    doc["tmpi"] = temp_event.temperature;
    doc["prs"] = pressure_event.pressure;
    doc["uv"] = analogRead(PIN_GUVA);
    */
  
    //Serial.println("Packet sent");
    prev_ms = millis();
  } 
}

String getJSON(String _id, String _value) {
  String json;
  StaticJsonDocument<300> doc;

  doc[_id] = _value;
  
  serializeJson(doc, json);
  return json;
}
