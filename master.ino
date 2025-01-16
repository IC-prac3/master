#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

#define LORA_LOCAL_ADDRESS 0x93
#define LORA_GATEWAY_ADDRESS 0x92
#define LORA_BANDWIDTH_INDEX 5
#define LORA_SPREADING_FACTOR 8
#define LORA_CODING_RATE 6
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

#define TX_LAPSE_MS 5000

double bandwidth_kHz[10] = { 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
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

volatile bool txDoneFlag = true;
volatile bool transmitting = false;

volatile uint32_t TxTime_ms;
volatile uint32_t txInterval_ms = TX_LAPSE_MS;
volatile uint32_t lastSendTime_ms = 0;
volatile uint32_t timeOut = TX_LAPSE_MS * 2;
uint32_t tx_begin_ms;

bool newConfigTimedOut = false;
volatile bool shouldOptimize = false;
volatile bool optimized = false;
volatile bool shouldSendValidation = false;
byte validationCount = 1;


void setup() {
  Serial.begin(USB_BAUD_RATE);
  while (!Serial)
    ;
  delay(100);
  if (!init_PMIC()) {
    Serial.println("Initilization of BQ24195L failed!");
  } else {
    Serial.println("Initilization of BQ24195L succeeded!");
  }
  begin(LORA_CONFIG);
  validationMessage(validationCount);
}

void loop() {

  if (!optimized) checkTimeout();

  if (!transmitting && ((millis() - lastSendTime_ms) > txInterval_ms)) {
    if (shouldOptimize) findLowerTransmissionTime();
    if (!optimized && shouldSendValidation) {
      SerialUSB.println("Sending validation");
      validationMessage(validationCount % 3 + 1);
      validationCount++;
      shouldSendValidation = false;
    }
    if (optimized && shouldSendValidation) {
      SerialUSB.println("Sending validation 2");
      validationMessage(validationCount++);
      shouldSendValidation = false;
    }
  }
  if (transmitting && txDoneFlag) {
    transmitting = false;
  }
}

void checkTimeout() {
  if (millis() - lastSendTime_ms > timeOut) {
    newConfigTimedOut = true;
    returnToLastConfig();
  }
}

void begin(LoraConfig config) {
  if (!LoRa.begin(868E6)) {  // Initicializa LoRa a 868 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true)
      ;
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

void findLowerTransmissionTime() {
  if (LORA_CONFIG.bandwidthIndex == MAX_BANDWIDTH && LORA_CONFIG.spreadingFactor == MIN_SPREADING_FACTOR && LORA_CONFIG.codingRate == MIN_CODING_RATE && LORA_CONFIG.transmitPower == MIN_TRANSMIT_POWER) {
    shouldOptimize = false;
    optimized = true;
    return;
  }
  transmitting = true;
  tx_begin_ms = millis();
  shouldOptimize = false;
  sendNewLoraConfig();
}

void validationMessage(int i) {
  transmitting = true;
  tx_begin_ms = millis();
  sendMessage(0x80 + i);
}

void adjustTxInterval(uint32_t tx_begin_ms, uint32_t TxTime_ms) {
  uint32_t lapse_ms = tx_begin_ms - lastSendTime_ms;
  lastSendTime_ms = tx_begin_ms;
  float duty_cycle = (100.0f * TxTime_ms) / lapse_ms;

  Serial.print("Duty cycle: ");
  Serial.print(duty_cycle, 1);
  Serial.println(" %\n");

  // Solo si el ciclo de trabajo es superior al 1% lo ajustamos
  if (duty_cycle > 1.0f) {
    txInterval_ms = TxTime_ms * 100;
    timeOut = txInterval_ms * 2;
  }
}

uint32_t getTransmissionTime(uint32_t tx_begin_ms) {
  TxTime_ms = millis() - tx_begin_ms;
  Serial.print("----> TX completed in ");
  Serial.print(TxTime_ms);
  Serial.println(" msecs");
  return TxTime_ms;
}

void sendMessage(byte content) {
  txDoneFlag = false;
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

void sendNewLoraConfig() {
  txDoneFlag = false;
  LAST_CONFIG = LORA_CONFIG;
  while (!LoRa.beginPacket()) {
    delay(10);
  }
  modifyLoraConfig();
  LoRa.write(LORA_GATEWAY_ADDRESS);
  LoRa.write(LORA_LOCAL_ADDRESS);
  LoRa.write(getFirstConfigByte());
  LoRa.write(getSecondConfigByte());
  LoRa.endPacket(true);
  SerialUSB.println("Hemos enviado:");
  SerialUSB.print("0x");
  SerialUSB.println(getFirstConfigByte(), HEX);
  SerialUSB.println("");
  SerialUSB.print("0x");
  SerialUSB.println(getSecondConfigByte(), HEX);
  SerialUSB.println("");
}

byte getFirstConfigByte() {
  byte result = LORA_CONFIG.bandwidthIndex << 3;
  result |= (LORA_CONFIG.spreadingFactor - 6);
  return result;
}

byte getSecondConfigByte() {
  byte result = (LORA_CONFIG.codingRate - 5) << 5;
  result |= (LORA_CONFIG.transmitPower);
  return result;
}

void modifyLoraConfig() {
  if (LORA_CONFIG.codingRate > MIN_CODING_RATE) {
    LORA_CONFIG.codingRate--;
  } else if (LORA_CONFIG.spreadingFactor > MIN_SPREADING_FACTOR) {
    LORA_CONFIG.spreadingFactor--;
  } else if (LORA_CONFIG.bandwidthIndex < MAX_BANDWIDTH) {
    LORA_CONFIG.bandwidthIndex++;
  } else if (LORA_CONFIG.transmitPower > MIN_TRANSMIT_POWER) {
    LORA_CONFIG.transmitPower--;
  }
}

void returnToLastConfig() {
  SerialUSB.println("Return To Last Config");
  LORA_CONFIG = LAST_CONFIG;
  newConfigTimedOut = true;
  optimized = true;
  shouldSendValidation = true;
  updateLoraConfig(LORA_CONFIG);
}

void receiveMessage(int packetSize) {
  SerialUSB.println("---------------------");
  SerialUSB.println("Estamos recibiendo:");
  if (!txDoneFlag) return;
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

void finishedSending() {

  LoRa.receive();
  txDoneFlag = true;
  TxTime_ms = getTransmissionTime(tx_begin_ms);
  adjustTxInterval(tx_begin_ms, TxTime_ms);
  SerialUSB.println("Finish sending\n");
}

void proccessMessage(byte content) {
  switch (content) {
    case 0:  // Slave received new config
      SerialUSB.println("Esclavo ha recibido nueva configuración");
      updateLoraConfig(LORA_CONFIG);
      shouldSendValidation = true;
      break;
    case 3:
      shouldOptimize = true;
      break;
    default:
      shouldSendValidation = true;
      break;
  }
}

void updateLoraConfig(LoraConfig config) {
  LoRa.setSignalBandwidth(long(bandwidth_kHz[config.bandwidthIndex]));
  LoRa.setSpreadingFactor(config.spreadingFactor);
  LoRa.setCodingRate4(config.codingRate);
  LoRa.setTxPower(config.transmitPower, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.receive();
}