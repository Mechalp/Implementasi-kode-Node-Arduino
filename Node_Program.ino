#include <SPI.h>
#include <LoRa.h>
#include "DHT.h"

#define SS_PIN 22
#define RST_PIN 15
#define DHT_PIN 14
#define MQ_PIN 27
#define VOLTAGE_PIN 13
#define DIO0_PIN 21

#define DHTTYPE DHT11

const int nodeID = 11;  // Set the unique node ID here
const float posX = 26.23;
const float posY = 67.35;
const float initialEnergy = 4.2;
const int sinkID = 99;  // Use a specific ID for Sink

unsigned long previousMillis = 0;
const long interval = 5000;  // Interval for sensor data transmission (5 seconds)
bool dataAcknowledged = false;  // Flag to check if data is acknowledged
bool phase2DataAcknowledged = false;  // Flag to check if phase 2 data is acknowledged
bool waitingForNextRound = false; // Flag to wait for the next round
unsigned long syncTime = 0;  // To store synchronized time from sink

DHT dht(DHT_PIN, DHTTYPE);

// Sensor values
float firstRead_t, firstRead_h, firstRead_ppm, firstRead_voltage;
float secondRead_t, secondRead_h, secondRead_ppm, secondRead_voltage;

struct LoRa_config {
  long Frequency;
  int SpreadingFactor;
  long SignalBandwidth;
  int CodingRate4;
  bool enableCrc;
  bool invertIQ;
  int SyncWord;
  int PreambleLength;
};

// Initialize the LoRa configuration
static LoRa_config nodeLoRa = {433E6, 7, 125E3, 5, true, false, 0x12, 8};

void configureLoRa(LoRa_config config) {
  LoRa.setSpreadingFactor(config.SpreadingFactor);
  LoRa.setSignalBandwidth(config.SignalBandwidth);
  LoRa.setCodingRate4(config.CodingRate4);
  if (config.enableCrc) {
    LoRa.enableCrc();
  } else {
    LoRa.disableCrc();
  }
  if (config.invertIQ) {
    LoRa.enableInvertIQ();
  } else {
    LoRa.disableInvertIQ();
  }
  LoRa.setSyncWord(config.SyncWord);
  LoRa.setPreambleLength(config.PreambleLength);
}

void setup() {
  Serial.begin(9600);
  while (!Serial);
  delay(1000);
  Serial.println("Starting Node Setup...");

  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);

  if (!LoRa.begin(nodeLoRa.Frequency)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  dht.begin();
  configureLoRa(nodeLoRa);
  Serial.println("LoRa Node Initialized");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String incomingData = "";
    while (LoRa.available()) {
      incomingData += (char) LoRa.read();
    }

    // Check if the incoming data is from the Sink
    if (incomingData.startsWith(String(sinkID) + ";")) {
      Serial.print("Received data: ");
      Serial.println(incomingData);

      // Remove the sinkID prefix before processing
      incomingData = incomingData.substring(incomingData.indexOf(';') + 1);

      if (incomingData.startsWith("SYNC")) {
        String syncMessage = String(sinkID) + ";TIME;" + String(millis());
        LoRa.beginPacket();
        LoRa.print(syncMessage);
        LoRa.endPacket();
        Serial.println("Sent SYNC response: " + syncMessage);
      } else if (incomingData.startsWith("TIME")) {
        syncTime = incomingData.substring(5).toInt();
        Serial.print("Synchronized time with sink: ");
        Serial.println(syncTime);
        // Ready to send phase 2 data after synchronization
        phase2DataAcknowledged = false;
        dataAcknowledged = true;  // Reset phase 1 acknowledgment
        waitingForNextRound = false;
      } else if (incomingData.startsWith("REQ")) {
        sendInitialData();
      } else if (incomingData.startsWith("ACK")) {
        int ackId = incomingData.substring(4).toInt();
        if (ackId == nodeID) {
          Serial.println("ACK received for initial data.");
          dataAcknowledged = true;
        }
      } else if (incomingData.startsWith("REQ2")) {
        if (dataAcknowledged) {  // Only send phase 2 data if initial data has been acknowledged
          sendSensorData();
        }
      } else if (incomingData.startsWith("ACK2")) {
        int ackId = incomingData.substring(5).toInt();
        if (ackId == nodeID) {
          Serial.println("ACK received for sensor data.");
          phase2DataAcknowledged = true;
          waitingForNextRound = true;
        }
      } else if (incomingData.startsWith("NEXT")) {
        Serial.println("Received start of next round.");
        dataAcknowledged = false;
        phase2DataAcknowledged = false;
        waitingForNextRound = false;
        sendInitialData(); // Reset and send initial data for the new round
      } else {
        Serial.println("Received unknown command.");
      }
    } else {
      Serial.println("Ignored data not from Sink.");
    }
  }

  if (!waitingForNextRound) {
    if (dataAcknowledged && !phase2DataAcknowledged) {
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        readFirstSensorData();
        delay(5000);  // Delay for 1 seconds
        readSecondSensorData();
        delay(5000);
        if (shouldSendData()) {
          sendSensorData();
        } else {
          Serial.println("----------------------------------------------");
          Serial.println("DATA SAMA TIDAK PERLU DIKIRIM, NAMUN TEGANGAN TETAP DIKIRIM");
          sendVoltageData(); // Tetap mengirimkan data tegangan
        }
      }
    } else if (!dataAcknowledged) {
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        sendInitialData();
      }
    }
  }
}

void readFirstSensorData() {
  firstRead_t = dht.readTemperature();
  firstRead_h = dht.readHumidity();
  firstRead_ppm = analogRead(MQ_PIN);
  firstRead_voltage = analogRead(VOLTAGE_PIN) * (4.2 / 2048.0);
  
  Serial.println("----------------------------------------------");
  Serial.println("First Sensor Read: ");
  Serial.print("Temperature: "); Serial.println(firstRead_t);
  Serial.print("Humidity: "); Serial.println(firstRead_h);
  Serial.print("Gas PPM: "); Serial.println(firstRead_ppm);
  Serial.print("Voltage: "); Serial.println(firstRead_voltage);
}

void readSecondSensorData() {
  secondRead_t = dht.readTemperature();
  secondRead_h = dht.readHumidity();
  secondRead_ppm = analogRead(MQ_PIN);
  secondRead_voltage = analogRead(VOLTAGE_PIN) * (4.2 / 2048.0);

  Serial.println("----------------------------------------------");
  Serial.println("Second Sensor Read: ");
  Serial.print("Temperature: "); Serial.println(secondRead_t);
  Serial.print("Humidity: "); Serial.println(secondRead_h);
  Serial.print("Gas PPM: "); Serial.println(secondRead_ppm);
  Serial.print("Voltage: "); Serial.println(secondRead_voltage);
}

bool shouldSendData() {
  float similarity_t = similarity(firstRead_t, secondRead_t);
  float similarity_h = similarity(firstRead_h, secondRead_h);
  float similarity_ppm = similarity(firstRead_ppm, secondRead_ppm);
  
  Serial.println("----------------------------------------------");
  Serial.println("Similarity:");
  Serial.print("Temperature: "); Serial.println(similarity_t);
  Serial.print("Humidity: "); Serial.println(similarity_h);
  Serial.print("Gas PPM: "); Serial.println(similarity_ppm);

  // Only send data if similarity is less than 0.8 for any sensor
  return similarity_t < 0.8 || similarity_h < 0.8 || similarity_ppm < 0.8;
}

float max_custom(float a, float b, float c) {
  float max_val = a;
  if (b > max_val) max_val = b;
  if (c > max_val) max_val = c;
  return max_val;
}

float similarity(float current, float previous) {
  return 1.0 - abs(current - previous) / max_custom(abs(current), abs(previous), 1.0);
}

void sendInitialData() {
  char buffer[128];
  snprintf(buffer, sizeof(buffer), "%d;INIT;%d;%.2f,%.2f;%.2f", sinkID, nodeID, posX, posY, initialEnergy);  // Menambahkan sinkID di awal pesan
  LoRa.beginPacket();
  LoRa.print(buffer);
  LoRa.endPacket();
  Serial.println("Mengirimkan Data Initial (ID, posisi, Energi Awal): " + String(buffer));
}

void sendSensorData() {
  float send_t = (similarity(firstRead_t, secondRead_t) < 0.8) ? secondRead_t : 0.0;
  float send_h = (similarity(firstRead_h, secondRead_h) < 0.8) ? secondRead_h : 0.0;
  float send_ppm = (similarity(firstRead_ppm, secondRead_ppm) < 0.8) ? secondRead_ppm : 0.0;
  float send_voltage = secondRead_voltage;

  char buffer[128];
  snprintf(buffer, sizeof(buffer), "%d;PHASE2;%d;%.2f;%.2f;%d;%.2f", sinkID, nodeID, send_t, send_h, (int)send_ppm, send_voltage);  // Menambahkan sinkID di awal pesan
  LoRa.beginPacket();
  LoRa.print(buffer);
  LoRa.endPacket();
  Serial.println("----------------------------------------------");
  Serial.println("DATA BERBEDA DAN DIKIRIM");
  Serial.println("Sent sensor data (ID, Suhu, Kelembaban, Gas, Tegangan): " + String(buffer));
}

void sendVoltageData() {
  char buffer[128];
  snprintf(buffer, sizeof(buffer), "%d;PHASE2;%d;%.2f;%.2f;%d;%.2f", sinkID, nodeID, 0.0, 0.0, 0, secondRead_voltage);  // Menambahkan sinkID di awal pesan
  LoRa.beginPacket();
  LoRa.print(buffer);
  LoRa.endPacket();
  Serial.println("Sent voltage data with zeros for other sensors (ID, Suhu=0, Kelembaban=0, Gas=0, Tegangan): " + String(buffer));
}
