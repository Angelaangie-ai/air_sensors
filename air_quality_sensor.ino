#include <Wire.h>
#include <WiFi.h>
#include <bme680.h>
#include <SoftwareSerial.h>

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// Pin definitions
#define PMS_RX 16  // GPIO16 for PMS5003 RX
#define PMS_TX 17  // GPIO17 for PMS5003 TX
#define LED_PIN 2  // Status LED

// BME680 I2C address
#define BME680_ADDR 0x77

// Global objects
SoftwareSerial pmsSensor(PMS_RX, PMS_TX);
Bme680 bme;

// Data structures
struct PmsData {
  uint16_t pm1_0;
  uint16_t pm2_5;
  uint16_t pm10_0;
} pmsData;

struct EnvData {
  float temperature;
  float humidity;
  float pressure;
  float gasResistance;
} envData;

void setup() {
  // Initialize serial communications
  Serial.begin(115200);
  pmsSensor.begin(9600);
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize BME680
  if (!bme.begin(BME680_ADDR)) {
    Serial.println("Could not find BME680 sensor!");
    while (1);
  }
  
  // Configure BME680
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320°C for 150 ms
  
  // Connect to WiFi
  setupWifi();
}

void loop() {
  // Read sensors
  readPmsSensor();
  readBme680();
  
  // Print data to Serial
  printData();
  
  // Send data to server/cloud
  sendData();
  
  // Blink LED to indicate successful reading
  blinkLed();
  
  // Wait before next reading
  delay(2000);
}

void setupWifi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void readPmsSensor() {
  uint8_t buffer[32];
  if (pmsSensor.available() >= 32) {
    // Read PMS5003 data
    pmsSensor.readBytes(buffer, 32);
    
    // Check data header
    if (buffer[0] == 0x42 && buffer[1] == 0x4d) {
      // Extract PM values (standard particle)
      pmsData.pm1_0 = (buffer[10] << 8) | buffer[11];
      pmsData.pm2_5 = (buffer[12] << 8) | buffer[13];
      pmsData.pm10_0 = (buffer[14] << 8) | buffer[15];
    }
  }
}

void readBme680() {
  if (bme.performMeasurement()) {
    envData.temperature = bme.readTemperature();
    envData.humidity = bme.readHumidity();
    envData.pressure = bme.readPressure() / 100.0; // Convert to hPa
    envData.gasResistance = bme.readGasResistance() / 1000.0; // Convert to kOhm
  }
}

void printData() {
  // Print PM data
  Serial.println("\n--- Air Quality Data ---");
  Serial.print("PM1.0: "); Serial.print(pmsData.pm1_0);
  Serial.println(" µg/m³");
  Serial.print("PM2.5: "); Serial.print(pmsData.pm2_5);
  Serial.println(" µg/m³");
  Serial.print("PM10: "); Serial.print(pmsData.pm10_0);
  Serial.println(" µg/m³");
  
  // Print environmental data
  Serial.println("\n--- Environmental Data ---");
  Serial.print("Temperature: "); Serial.print(envData.temperature);
  Serial.println(" °C");
  Serial.print("Humidity: "); Serial.print(envData.humidity);
  Serial.println(" %");
  Serial.print("Pressure: "); Serial.print(envData.pressure);
  Serial.println(" hPa");
  Serial.print("Gas Resistance: "); Serial.print(envData.gasResistance);
  Serial.println(" kΩ");
}

void sendData() {
  // Here you would implement your preferred method of sending data
  // Examples include: HTTP POST, MQTT, or other protocols
  // This is a placeholder for your specific implementation
}

void blinkLed() {
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
}
