/****************************************************************
*  Sensors + LoRa + ESP32 Demo + JSON
*  
*  SHT21: https://github.com/markbeee/SHT21 | C++ files appended
*  MPU9250: https://github.com/hideakitai/MPU9250 | C++ files appended
*  Arduino-LoRa: https://github.com/sandeepmistry/arduino-LoRa | Installed
*  BMP280: https://github.com/adafruit/Adafruit_BMP280_Library | C++ files appended (mod)
*  An example sketch that reads the GY-91 (MPU9250 + BMP280), SHT21, 
*  GUVA-S12SD and MQ135 (FC-22) to the LoRa SX1278 module.
*  Measures Accel, Gyro, Pressure, Relative Humidity, Temperature, 
*  UV Radiation and Air Quality 
*  SHT21, MPU9250 --> 21: SDA, 22: SCL 
*  LoRa SX1278 --> MOSI: 23, MISO: 22, SCK: 18, NSS: 5, DIO: 2, RST: 15
*  
*  @TODO: Add capacitors and use a solid circuit board for all
*  @TODO: Add LoRa Error handler / abort
*  @TODO: Round output data
*  @TODO: Improve performance
* 
***************************************************************/
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include "SHT21.h"
#include "MPU9250.h"
#include "Adafruit_BMP280.h"

//define the pins used by the LoRa transceiver module
#define ss 5
#define rst 15
#define dio0 2

#define PIN_MQ135 4

#define BMP280_ADDRESS (0x76)

#define BAND 915E6    //433E6 for Asia, 866E6 for Europe, 915E6 for North America

SHT21 SHT21;
MPU9250 mpu;
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

int counter = 0;
const int interval = 500; // ms

void setup() {
  LoRa.setPins(ss, rst, dio0); //setup LoRa transceiver module
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

  
  Wire.begin();
  
  while (!LoRa.begin(BAND) && counter < 10) {
    Serial.print(".");
    counter++;
    delay(500);
  }
  if (counter == 10) {
    Serial.println("Starting LoRa failed!"); 
  }
  

  mpu.setup();
  bmp_temp->printSensorDetails();

  delay(2000);
}

void loop() {
  static uint32_t prev_ms = millis();
  if ((millis() - prev_ms) > interval) {
    Serial.println("Sending packet: ");
  
    // send packet
    //LoRa.beginPacket();
    String s = getJSONSensorData(); //DELETE
    //LoRa.print(s);
    Serial.println(s);
    //LoRa.endPacket();
  
    //Serial.println("Packet sent");
  } 
}

String getJSONSensorData() {
  String json;
  StaticJsonDocument<300> doc;

  mpu.update();

  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
   
  doc["tmp"] = SHT21.getTemperature(); // Temperature
  doc["hum"] = SHT21.getHumidity(); // Humidity
  doc["acc"] = "[" + String(mpu.getAcc(0)) + "," + String(mpu.getAcc(1)) + "," + String(mpu.getAcc(2)) + "]";
  doc["gyr"] = "[" + String(mpu.getGyro(0)) + "," + String(mpu.getGyro(1)) + "," + String(mpu.getGyro(2)) + "]";
  doc["mag"] = "[" + String(mpu.getMag(0)) + "," + String(mpu.getMag(1)) + "," + String(mpu.getMag(2)) + "]";
  doc["rpy"] = "[" + String(mpu.getRoll()) + "," + String(mpu.getPitch()) + "," + String(mpu.getYaw()) + "]";
  doc["tmpi"] = temp_event.temperature;
  doc["prs"] = pressure_event.pressure;
  doc["aiq"] = analogRead(PIN_MQ135);
  
  serializeJson(doc, json);
  return json;
}
