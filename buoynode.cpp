#include <SPI.h>
#include <LoRa.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <AM2302-Sensor.h>
#include <ArduinoJson.h>
#include "FS.h"
#include "SD.h"

// Define LoRa SPI pins
#define LORA_SCK 18
#define LORA_MOSI 23
#define LORA_MISO 19
#define LORA_CS 5
#define LORA_RST 2
#define LORA_EN 26

// Define SD card SPI pins
#define SD_CS 15 // Chip Select pin for the SD card
#define SD_SCK 14
#define SD_MOSI 13
#define SD_MISO 27

SPIClass vspi(VSPI);
SPIClass hspi(HSPI);

#define ONE_WIRE_BUS 12 // SST waterproof sensor data wire
constexpr unsigned int SENSOR_PIN {25}; // Air temp and humidity sensor 

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire); // Pass oneWire reference to SST sensor
Adafruit_MMA8451 mma = Adafruit_MMA8451();
AM2302::AM2302_Sensor am2302{SENSOR_PIN};

#define MS5607_ADDRESS 0x76 // Define Altimeter address
#define GY271_ADDRESS 0x1E // Define Compass address

#define CMD_RESET 0x1E
#define CMD_CONVERT_D1_4096 0x48
#define CMD_CONVERT_D2_4096 0x58
#define CMD_READ_ADC 0x00

uint16_t C[6]; // Calibration coefficients

unsigned long lastPressureTime = 0;
unsigned long lastTemperatureTime = 0;
unsigned long lastAM2302Time = 0;
unsigned long lastAccelerometerTime = 0;
unsigned long lastMagnetometerTime = 0;

void resetMS5607() {
  Wire.beginTransmission(MS5607_ADDRESS);
  Wire.write(CMD_RESET);
  Wire.endTransmission();
  delay(10);
}

void readCalibrationCoefficients() {
  for (uint8_t i = 0; i < 6; i++) {
    Wire.beginTransmission(MS5607_ADDRESS);
    Wire.write(0xA2 + (i * 2));
    Wire.endTransmission();
    Wire.requestFrom(MS5607_ADDRESS, 2);
    C[i] = (Wire.read() << 8) | Wire.read();
  }
}

uint32_t readRawPressure() {
  Wire.beginTransmission(MS5607_ADDRESS);
  Wire.write(CMD_CONVERT_D1_4096);
  Wire.endTransmission();
  delay(10);  // Wait for conversion

  return readADC();
}

uint32_t readRawTemperature() {
  Wire.beginTransmission(MS5607_ADDRESS);
  Wire.write(CMD_CONVERT_D2_4096);
  Wire.endTransmission();
  delay(10);  // Wait for conversion

  return readADC();
}

uint32_t readADC() {
  Wire.beginTransmission(MS5607_ADDRESS);
  Wire.write(CMD_READ_ADC);
  Wire.endTransmission();

  Wire.requestFrom(MS5607_ADDRESS, 3);
  uint32_t result = 0;
  result = Wire.read();
  result = (result << 8) | Wire.read();
  result = (result << 8) | Wire.read();

  return result;
}

void readMagnetometer(int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(GY271_ADDRESS);
  Wire.write(0x03);
  Wire.endTransmission();

  Wire.requestFrom(GY271_ADDRESS, 6);
  if (Wire.available() == 6) {
    x = (Wire.read() << 8) | Wire.read();
    z = (Wire.read() << 8) | Wire.read();
    y = (Wire.read() << 8) | Wire.read();

    if (x > 32767) x -= 65536;
    if (y > 32767) y -= 65536;
    if (z > 32767) z -= 65536;
  }
}

void transmitData(const char* dataLabel, const char* jsonData) {
  LoRa.beginPacket();
  LoRa.print(jsonData);
  LoRa.endPacket();
  Serial.print(dataLabel);
  Serial.println(jsonData);

  // Log data to SD card
  logToSD(dataLabel, jsonData);
}

void logToSD(const char* dataLabel, const char* jsonData) {
  File file = SD.open("/data_log.txt", FILE_APPEND);
  if (file) {
    file.print(dataLabel);
    file.println(jsonData);
    file.close();
    Serial.println("Data logged to SD card");
  } else {
    Serial.println("Failed to open file for logging");
  }
}

void transmitSSTData() {
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  StaticJsonDocument<200> sstDoc;
  sstDoc["type"] = "sst";
  sstDoc["timestamp"] = millis(); // Placeholder timestamp
  sstDoc["value"] = tempC;
  char sstData[200];
  serializeJson(sstDoc, sstData, sizeof(sstData));
  if (tempC != DEVICE_DISCONNECTED_C) {
    transmitData("SST Data: ", sstData);
  } else {
    Serial.println("Error: Could not read SST data");
  }
}

void transmitAM2302Data() {
  float temperature = am2302.get_Temperature();
  float humidity = am2302.get_Humidity();
  
  Serial.print("AM2302 Temperature: ");
  Serial.println(temperature);
  Serial.print("AM2302 Humidity: ");
  Serial.println(humidity);

  // Transmit Temperature
  StaticJsonDocument<200> tempDoc;
  tempDoc["type"] = "temperature";
  tempDoc["timestamp"] = millis(); // Placeholder timestamp
  tempDoc["value"] = temperature;
  char tempData[200];
  serializeJson(tempDoc, tempData, sizeof(tempData));
  transmitData("Temperature Data: ", tempData);

  // Transmit Humidity
  StaticJsonDocument<200> humidityDoc;
  humidityDoc["type"] = "humidity";
  humidityDoc["timestamp"] = millis(); // Placeholder timestamp
  humidityDoc["value"] = humidity;
    char humidityData[200];
  serializeJson(humidityDoc, humidityData, sizeof(humidityData));
  transmitData("Humidity Data: ", humidityData);
}

void transmitPressureData() {
  uint32_t D1 = readRawPressure();
  uint32_t D2 = readRawTemperature();
  int32_t dT = D2 - ((uint32_t)C[4] << 8);
  int32_t TEMP = 2000 + ((int64_t)dT * C[5]) / 8388608;
  int64_t OFF = ((int64_t)C[1] << 16) + ((int64_t)C[3] * dT) / 128;
  int64_t SENS = ((int64_t)C[0] << 15) + ((int64_t)C[2] * dT) / 256;
  int32_t P = (((D1 * SENS) / 2097152) - OFF) / 32768;
  StaticJsonDocument<200> pressureDoc;
  pressureDoc["type"] = "pressure";
  pressureDoc["timestamp"] = millis(); // Placeholder timestamp
  pressureDoc["value"] = P;
  char pressureData[200];
  serializeJson(pressureDoc, pressureData, sizeof(pressureData));
  transmitData("Pressure Data: ", pressureData);
}

void transmitMagnetometerData() {
  int16_t x, y, z;
  readMagnetometer(x, y, z);
  StaticJsonDocument<200> magnetometerDoc;
  magnetometerDoc["type"] = "magnetometer";
  magnetometerDoc["timestamp"] = millis(); // Placeholder timestamp
  magnetometerDoc["value"]["x"] = x;
  magnetometerDoc["value"]["y"] = y;
  magnetometerDoc["value"]["z"] = z;
  char magnetometerData[200];
  serializeJson(magnetometerDoc, magnetometerData, sizeof(magnetometerData));
  transmitData("Magnetometer Data: ", magnetometerData);
}

void transmitAccelerometerData() {
  mma.read();
  sensors_event_t event;
  mma.getEvent(&event);
  uint8_t orientation = mma.getOrientation();
  String orientationStr;
  switch (orientation) {
    case MMA8451_PL_PUF: orientationStr = "Portrait Up Front"; break;
    case MMA8451_PL_PUB: orientationStr = "Portrait Up Back"; break;
    case MMA8451_PL_PDF: orientationStr = "Portrait Down Front"; break;
    case MMA8451_PL_PDB: orientationStr = "Portrait Down Back"; break;
    case MMA8451_PL_LRF: orientationStr = "Landscape Right Front"; break;
    case MMA8451_PL_LRB: orientationStr = "Landscape Right Back"; break;
    case MMA8451_PL_LLF: orientationStr = "Landscape Left Front"; break;
    case MMA8451_PL_LLB: orientationStr = "Landscape Left Back"; break;
    default: orientationStr = "Unknown"; break;
  }

  StaticJsonDocument<200> accelerometerDoc;
  accelerometerDoc["type"] = "accelerometer";
  accelerometerDoc["timestamp"] = millis(); // Placeholder timestamp
  accelerometerDoc["value"]["x"] = event.acceleration.x;
  accelerometerDoc["value"]["y"] = event.acceleration.y;
  accelerometerDoc["value"]["z"] = event.acceleration.z;
  accelerometerDoc["orientation"] = orientationStr;
  char accelerometerData[200];
  serializeJson(accelerometerDoc, accelerometerData, sizeof(accelerometerData));
  transmitData("Accelerometer Data: ", accelerometerData);
}

void setup() {
  Serial.begin(115200);

  // Initialize VSPI for LoRa
  vspi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setSPI(vspi);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_EN);

  // Initialize HSPI for SD card
  hspi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  SD.begin(SD_CS, hspi);

  pinMode(LORA_CS, OUTPUT);
  pinMode(SD_CS, OUTPUT);

  Serial.println("Initializing LoRa and SD card...");

  // Initialize LoRa module
  if (!LoRa.begin(433E6)) {
    Serial.println("Error: Starting LoRa failed!");
    while (1);
  } else {
    Serial.println("Starting LoRa!");
  }

  // Initialize SD card
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS, hspi, 4000000)) { // Reduced SPI speed to 4MHz
    Serial.println("Initialization failed!");
    return;
  } else {
    Serial.println("SD card initialized.");
  }

  Serial.println("Initialization done.");

  // List files on SD card
  listDir(SD, "/", 0);

  // Write dummy data to a file on SD card
  writeFile(SD, "/data_log.txt", "Hello, SD card!");

  // Start up the SST library
  sensors.begin();

  // Initialize the MMA8451 accelerometer
  if (!mma.begin()) {
    Serial.println("Error: MMA8451");
    while (1);
  }
  Serial.println("MMA8451 found!");
  mma.setRange(MMA8451_RANGE_2_G);

  Serial.print("Range = ");
  Serial.print(2 << mma.getRange());
  Serial.println("G");

  // Initialize the AM2302 sensor
  if (am2302.begin()) {
    // this delay is needed to receive valid data
    delay(3000);
    Serial.println("AM2302 found!");
  } else {
    while (true) {
      Serial.println("Error: AM2302");
      delay(10000);
    }
  }

  resetMS5607();
  delay(100);
  readCalibrationCoefficients();

  // Initialize GY-271 magnetometer
  Wire.beginTransmission(GY271_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x70); // 8-average, 15 Hz default, normal measurement
  Wire.endTransmission();

  Wire.beginTransmission(GY271_ADDRESS);
  Wire.write(0x01);
  Wire.write(0xA0); // Gain = 5
  Wire.endTransmission();

  Wire.beginTransmission(GY271_ADDRESS);
  Wire.write(0x02);
  Wire.write(0x00); // Continuous measurement mode
  Wire.endTransmission();
}

void loop() {
  unsigned long currentTime = millis();

  // Read SST data every 10 seconds
  if (currentTime - lastTemperatureTime >= 3000) {
    lastTemperatureTime = currentTime;
    transmitSSTData();
  }

  // Read AM2302 data every 10 seconds
  if (currentTime - lastAM2302Time >= 5000) {
    lastAM2302Time = currentTime;
    transmitAM2302Data();
  }

  // Read accelerometer data every 1 second
  if (currentTime - lastAccelerometerTime >= 2000) {
    lastAccelerometerTime = currentTime;
    transmitAccelerometerData();
  }

  // Read pressure data every 10 seconds
  if (currentTime - lastPressureTime >= 15000) {
    lastPressureTime = currentTime;
    transmitPressureData();
  }

  // Read magnetometer data every 1 second
  if (currentTime - lastMagnetometerTime >= 1000) {
    lastMagnetometerTime = currentTime;
    transmitMagnetometerData();
  }
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname); // Print directory name

  File root = fs.open(dirname); // Open the directory
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile(); // Opens the next file in the directory.
  // Loop through all files in the directory
  while (file) { 
    // Check if current file is directory or regular file
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

