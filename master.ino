#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

#define LORA_LOCAL_ADDRESS 0x93
#define LORA_GATEWAY_ADDRESS 0x92
#define LORA_BANDWIDTH_INDEX 6
#define LORA_SPREADING_FACTOR 12
#define LORA_CODING_RATE 8
#define LORA_TRANSMIT_POWER 2

#define USB_BAUD_RATE 9600
#define COMMAND_REGISTER byte(0)
#define REAL_RANGING_MODE_CM byte(81)
#define RANGE_HIGH_BYTE byte(2)
#define RANGE_LOW_BYTE byte(3)

#define MAX_BANDWIDTH 9
#define MIN_SPREADING_FACTOR 6
#define MIN_CODING_RATE 5
#define MIN_TRANSMIT_POWER 0

double bandwidth_kHz[10] = {7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
                            41.7E3, 62.5E3, 125E3, 250E3, 500E3 };

struct LoraConfig {
    byte deviceAddress;
    byte bandwidthIndex;
    byte spreadingFactor;
    byte codingRate;
    byte transmitPower;
};

LoraConfig LAST_CONFIG = {
    .deviceAddress = LORA_LOCAL_ADDRESS,
    .bandwidthIndex = LORA_BANDWIDTH_INDEX,
    .spreadingFactor = LORA_SPREADING_FACTOR,
    .codingRate = LORA_CODING_RATE,
    .transmitPower = LORA_TRANSMIT_POWER
};

LoraConfig LORA_CONFIG = {
        .deviceAddress = LORA_LOCAL_ADDRESS,
        .bandwidthIndex = LORA_BANDWIDTH_INDEX,
        .spreadingFactor = LORA_SPREADING_FACTOR,
        .codingRate = LORA_CODING_RATE,
        .transmitPower = LORA_TRANSMIT_POWER
    };

long lastMillis = 0;

volatile bool isSending = false;

bool confirmation = true;

volatile int frequency = 1;

bool optimized = false;


void setup() 
{
  Serial.begin(USB_BAUD_RATE);
  while (!Serial);
  delay(100);
  if (!init_PMIC()) {
    Serial.println("Initilization of BQ24195L failed!");
  }
  else {
    Serial.println("Initilization of BQ24195L succeeded!");
  }
  begin(LORA_CONFIG);
}

void loop()
{
  if (!optimized) findLowerTransmissionTime();
}

void begin(LoraConfig config)
{
    if (!LoRa.begin(868E6)) {      // Initicializa LoRa a 868 MHz
      Serial.println("LoRa init failed. Check your connections.");
      while (true);                
    }
    LoRa.setSignalBandwidth(long(bandwidth_kHz[config.bandwidthIndex]));
    LoRa.setSpreadingFactor(config.spreadingFactor);
    LoRa.setCodingRate4(config.codingRate);
    LoRa.setTxPower(config.transmitPower, PA_OUTPUT_PA_BOOST_PIN);    
    LoRa.setSyncWord(0x12);
    LoRa.setPreambleLength(8);
    LoRa.onTxDone(finishedSending);
    LoRa.onReceive(receiveMessage);
    LoRa.receive();
}

void findLowerTransmissionTime()
{
  confirmation = false;
  if (LORA_CONFIG.bandwidthIndex == MAX_BANDWIDTH &&
    LORA_CONFIG.spreadingFactor == MIN_SPREADING_FACTOR &&
    LORA_CONFIG.codingRate == MIN_CODING_RATE &&
    LORA_CONFIG.transmitPower == MIN_TRANSMIT_POWER) {
      optimized = true;
      return;
    }
  sendNewLoraConfig();
  delay(frequency * 1000);
  int i = 1;
  while (!confirmation && i < 6) {
    sendMessage(0x80 + i++);
    delay(frequency * 1000);
  }
  delay(1000);
  if (!confirmation) returnToLastConfig();
}

void sendMessage(byte content) 
{
  isSending = true;
  while (!LoRa.beginPacket()) {
    delay(10);
  }
  LoRa.write(LORA_GATEWAY_ADDRESS);
  LoRa.write(LORA_LOCAL_ADDRESS);
  LoRa.write(content);
  LoRa.endPacket(true);
  SerialUSB.println("Hemos enviado:");
  SerialUSB.print("0x");
  SerialUSB.println(content, HEX);
  SerialUSB.println("");
}

void sendNewLoraConfig()
{
  isSending = true;
  LAST_CONFIG = LORA_CONFIG;
  while (!LoRa.beginPacket()) {
    delay(10);
  }
  modifyLoraConfig();
  LoRa.write(LORA_GATEWAY_ADDRESS);
  LoRa.write(LORA_LOCAL_ADDRESS);
  LoRa.write(getFirstConfigByte());
  LoRa.write(getSecondConfigByte());
  LoRa.endPacket(false);
  SerialUSB.println("Hemos enviado:");
  SerialUSB.print("0x");
  SerialUSB.println(getFirstConfigByte(), HEX);
  SerialUSB.println("");
  SerialUSB.print("0x");
  SerialUSB.println(getSecondConfigByte(), HEX);
  SerialUSB.println("");
  delay(1000);
  updateLoraConfig(LORA_CONFIG);
}

byte getFirstConfigByte()
{
  byte result = LORA_CONFIG.bandwidthIndex << 3;
  result |= (LORA_CONFIG.spreadingFactor - 6);
  return result;
}

byte getSecondConfigByte()
{
  byte result = (LORA_CONFIG.codingRate - 5) << 5;
  result |= (LORA_CONFIG.transmitPower);
  return result;
}

void modifyLoraConfig()
{
  if (LORA_CONFIG.bandwidthIndex < MAX_BANDWIDTH) {
    LORA_CONFIG.bandwidthIndex++;
  } else if (LORA_CONFIG.spreadingFactor > MIN_SPREADING_FACTOR) {
    LORA_CONFIG.spreadingFactor--;
  } else if (LORA_CONFIG.codingRate > MIN_CODING_RATE) {
    LORA_CONFIG.codingRate--;
  } else if (LORA_CONFIG.transmitPower > MIN_TRANSMIT_POWER) {
    LORA_CONFIG.transmitPower--;
  }
}

void returnToLastConfig()
{
  SerialUSB.println("Estamos en returnToLastConfig");
  SerialUSB.println(LORA_CONFIG.bandwidthIndex);
  LORA_CONFIG = LAST_CONFIG;
  optimized = true;
  updateLoraConfig(LORA_CONFIG);
  for (int i = 0; i < 127; i += 2) {
    delay(3000);
    sendMessage(0x80 + i);
  }
}

void receiveMessage(int packetSize)
{
  SerialUSB.println("---------------------");
  SerialUSB.println("Estamos recibiendo:");
  if (isSending) return;
  if (packetSize == 0) return;
  const byte recipient = LoRa.read();
  const byte sender = LoRa.read();
  const byte content = LoRa.read();
  if (recipient != LORA_LOCAL_ADDRESS) return;
  if (sender != LORA_GATEWAY_ADDRESS) return;
  SerialUSB.print("0x");
  SerialUSB.println(content, HEX);
  SerialUSB.println("---------------------");
  SerialUSB.println("");
  proccessMessage(content);
}

void finishedSending()
{
  isSending = false;
  LoRa.receive();
  SerialUSB.println(
    "Finish sending"
  );
}

void proccessMessage(byte content)
{
  if (content == 0) {
    confirmation = true;
    return;
  }
  returnToLastConfig();
}

void updateLoraConfig(LoraConfig config) {
    LoRa.setSignalBandwidth(long(bandwidth_kHz[config.bandwidthIndex]));
    LoRa.setSpreadingFactor(config.spreadingFactor);
    LoRa.setCodingRate4(config.codingRate);
    LoRa.setTxPower(config.transmitPower, PA_OUTPUT_PA_BOOST_PIN);
    /*
    LoRa.setSyncWord(0x12);
    LoRa.setPreambleLength(8);
    LoRa.onTxDone(finishedSending);
    LoRa.onReceive(receiveMessage);
    */
    LoRa.receive();
}
