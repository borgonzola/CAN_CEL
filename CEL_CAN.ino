/*
  Simple example of sending/receiving messages between the two CAN interfaces on the ESP32-CAN-X2 dev board.
  https://wiki.autosportlabs.com/ESP32-CAN-X2
  
  This example relies on the ESP32 supplied TWAI api for interfacing with CAN1, and the Longan Labs mcp_canbus library
  for interfacing with the MCP2515 on CAN2.
*/

#include <Arduino.h>
#include <SPI.h>
#include "mcp_canbus.h"
#include "driver/twai.h"
#define POLLING_RATE_MS 50
#define CANStart_ID  0x240
#define CANME_ID1  0x101
#define CANME_ID3  0x308

MCP_CAN CAN(CS);
bool EngineON = 0;
bool MEAuth = 0;
bool startSIG = 0;
bool CELstatus = 1;
int engineRPM = 0;
int startPIN = 2; // IO2, drop IO to get pin number. This scheme sucks
int CELPin = 4; // IO4

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(startPIN, INPUT);
  pinMode(CELPin, OUTPUT);

  Serial.println("Initializing builtin CAN peripheral");
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN1_TX, (gpio_num_t)CAN1_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
//  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_filter_config_t f_config = {
    .acceptance_code = 0x61002020, //This sucks a lot to calculate
    .acceptance_mask = 0x001F001F, // 0 is for care bits?
    .single_filter = false
  };
  if(twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("CAN1 Driver initialized");
  } else {
    Serial.println("Failed to initialze CAN1 driver");
    return;
  }

  if (twai_start() == ESP_OK) {
    Serial.println("CAN1 interface started");
  } else {
    Serial.println("Failed to start CAN1");
    return;
  }

  uint32_t alerts_to_enable = TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    Serial.println("CAN1 Alerts reconfigured");
  } else {
    Serial.println("Failed to reconfigure alerts");
    return;
  }

  if (CAN_OK == CAN.begin(CAN_500KBPS)) {
    Serial.println("CAN2 interface started");
  } else {
    Serial.println("Failed to start CAN2");
    while (1);
  }
}

static void sendCANstart() {
  // Send message
  // Configure message to transmit
  twai_message_t startHI;
  startHI.identifier = CANStart_ID;
  startHI.extd = 0;
  startHI.rtr = 0;
  startHI.data_length_code = 3;
  startHI.data[0] = 0x00;
  startHI.data[1] = 0x13;
  startHI.data[2] = 0x00;
  twai_message_t startLO;
  startLO.identifier = CANStart_ID;
  startLO.extd = 0;
  startLO.rtr = 0;
  startLO.data_length_code = 3;
  startLO.data[0] = 0x00;
  startLO.data[1] = 0x12;
  startLO.data[2] = 0x01;

  // Queue message for transmission
  if (twai_transmit(&startHI, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.println("CAN1: Start HI Message queued for transmission");
  } else {
    Serial.println("CAN1: Failed to queue message for transmission");
  }
  delay(200);
  if (twai_transmit(&startLO, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.println("CAN1: Start LO Message queued for transmission");
  } else {
    Serial.println("CAN1: Failed to queue message for transmission");
  }

}

static void readCAN1() {
  twai_message_t message;
  while (twai_receive(&message, 0) == ESP_OK) {
    if (message.identifier == CANME_ID1) {
      if (message.data[1] == 0x48){
        MEAuth = 1;
      }
      else {
        MEAuth = 0;
      }
    }
    else if (message.identifier == CANME_ID3) {
      if ((message.data[3] & 0x02) == 0x02){
        CELstatus = 1;
      }
      else if ((message.data[3] & 0x02) == 0x00) {
        CELstatus = 0;
      }
      engineRPM = message.data[1];
      engineRPM = engineRPM << 8;
      engineRPM = engineRPM + message.data[2];
      if (engineRPM > 400) {
        EngineON = 1;
      }
      else if (engineRPM < 350){
        EngineON = 0;
      }
    }

    Serial.print("CAN1: Received ");
    // Process received message
    if (message.extd) {
      Serial.print("extended ");
    } else {
      Serial.print("standard ");
    }

    if (message.rtr) {
      Serial.print("RTR ");
    }

    Serial.printf("packet with id 0x%x", message.identifier);

    if (message.rtr) {
      Serial.printf(" and requested length %d\n", message.data_length_code);
    } else {
      Serial.printf(" and length %d\n", message.data_length_code);
      Serial.printf("CAN1: Data: %.*s\n", message.data_length_code, message.data);
    }
  }
}

void loop() {
//  Serial.println("MAIN: Disable LED");
  digitalWrite(LED_BUILTIN, HIGH);

  // Check if alert happened
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
  twai_status_info_t twaistatus;
  twai_get_status_info(&twaistatus);

  // Handle alerts
  if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
    Serial.println("CAN1: Alert: TWAI controller has become error passive.");
  }
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
    Serial.println("CAN1: Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
    Serial.printf("CAN1: Bus error count: %d\n", twaistatus.bus_error_count);
  }
  if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
    Serial.println("CAN1: Alert: The Transmission failed.");
    Serial.printf("CAN1: TX buffered: %d\t", twaistatus.msgs_to_tx);
    Serial.printf("CAN1: TX error: %d\t", twaistatus.tx_error_counter);
    Serial.printf("CAN1: TX failed: %d\n", twaistatus.tx_failed_count);
  }
  if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
    Serial.println("CAN1: Alert: The RX queue is full causing a received frame to be lost.");
    Serial.printf("CAN1: RX buffered: %d\t", twaistatus.msgs_to_rx);
    Serial.printf("CAN1: RX missed: %d\t", twaistatus.rx_missed_count);
    Serial.printf("CAN1: RX overrun %d\n", twaistatus.rx_overrun_count);
  }
  if (alerts_triggered & TWAI_ALERT_TX_SUCCESS) {
    Serial.println("CAN1: Alert: The Transmission was successful.");
    Serial.printf("CAN1: TX buffered: %d\n", twaistatus.msgs_to_tx);
  }

  // Check if message is received
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    readCAN1();
  }
  startSIG = digitalRead(startPIN);
  if ((EngineON == 0)&&(MEAuth == 1)&&(startSIG == 1)) {
    sendCANstart();
  }
  digitalWrite(CELPin, CELstatus);
  delay(5);
}
