#include <TimerMs.h>

#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include "CRC8.h"

#include <MQTT.h>
#include <IotWebConf.h>
#include <IotWebConfUsing.h>

/** Settings **/
// Baud rate for serial port
static const int BAUD_RATE = 115200;
static const int rx_timeout = 1000;


// Iot Web Conf Setup
const char thing_name[] = "MirtekMQTTGateway";
const char wifi_initial_ap_password[] = "12345678";
const char config_version[] = "MirtekMQTTGateway_v1";
const int string_length = 128;

DNSServer dns_server;
WebServer server(80);
WiFiClient net;
MQTTClient mqtt_client;

char mqtt_server_value[string_length];
char mqtt_username_value[string_length];
char mqtt_password_value[string_length];
char mirtek_id_value[string_length];

IotWebConf iot_web_conf(thing_name, &dns_server, &server, wifi_initial_ap_password, config_version);

IotWebConfParameterGroup mqtt_group = IotWebConfParameterGroup("mqtt", "MQTT configuration");
IotWebConfTextParameter mqtt_server_param = IotWebConfTextParameter("MQTT server", "mqttServer", mqtt_server_value, string_length);
IotWebConfTextParameter mqtt_username_param = IotWebConfTextParameter("MQTT user", "mqttUser", mqtt_username_value, string_length);
IotWebConfPasswordParameter mqtt_password_param = IotWebConfPasswordParameter("MQTT password", "mqttPass", mqtt_password_value, string_length);

IotWebConfParameterGroup mirtek_group = IotWebConfParameterGroup("mirtek", "Mirket Configuration");
IotWebConfTextParameter mirtek_id_param = IotWebConfTextParameter("Mirtek Meter ID", "mirtekMeterId", mirtek_id_value, string_length);

// Meter id vars
static const int meter_id_size = 2;
static byte meter_id[meter_id_size];

// Board state vars
bool need_mqtt_connect = true;
bool need_reset = false;

// CC1101 Settings
byte rfSettings[] = {
  0x0D,  // IOCFG2              GDO2 Output Pin Configuration
  0x2E,  // IOCFG1              GDO1 Output Pin Configuration
  0x06,  // IOCFG0              GDO0 Output Pin Configuration
  0x4F,  // FIFOTHR             RX FIFO and TX FIFO Thresholds
  0xD3,  // SYNC1               Sync Word, High Byte
  0x91,  // SYNC0               Sync Word, Low Byte
  0x3C,  // PKTLEN              Packet Length
  0x00,  // PKTCTRL1            Packet Automation Control
  0x41,  // PKTCTRL0            Packet Automation Control
  0x00,  // ADDR                Device Address
  0x16,  // CHANNR              Channel Number
  // 0x0B,  // CHANNR              Channel Number
  0x0F,  // FSCTRL1             Frequency Synthesizer Control
  0x00,  // FSCTRL0             Frequency Synthesizer Control
  0x10,  // FREQ2               Frequency Control Word, High Byte
  0x8B,  // FREQ1               Frequency Control Word, Middle Byte
  0x54,  // FREQ0               Frequency Control Word, Low Byte
  0xD9,  // MDMCFG4             Modem Configuration
  0x83,  // MDMCFG3             Modem Configuration
  0x13,  // MDMCFG2             Modem Configuration
  0xD2,  // MDMCFG1             Modem Configuration
  0xAA,  // MDMCFG0             Modem Configuration
  0x31,  // DEVIATN             Modem Deviation Setting
  0x07,  // MCSM2               Main Radio Control State Machine Configuration
  0x0C,  // MCSM1               Main Radio Control State Machine Configuration
  0x08,  // MCSM0               Main Radio Control State Machine Configuration
  0x16,  // FOCCFG              Frequency Offset Compensation Configuration
  0x6C,  // BSCFG               Bit Synchronization Configuration
  0x03,  // AGCCTRL2            AGC Control
  0x40,  // AGCCTRL1            AGC Control
  0x91,  // AGCCTRL0            AGC Control
  0x87,  // WOREVT1             High Byte Event0 Timeout
  0x6B,  // WOREVT0             Low Byte Event0 Timeout
  0xF8,  // WORCTRL             Wake On Radio Control
  0x56,  // FREND1              Front End RX Configuration
  0x10,  // FREND0              Front End TX Configuration
  0xE9,  // FSCAL3              Frequency Synthesizer Calibration
  0x2A,  // FSCAL2              Frequency Synthesizer Calibration
  0x00,  // FSCAL1              Frequency Synthesizer Calibration
  0x1F,  // FSCAL0              Frequency Synthesizer Calibration
  0x41,  // RCCTRL1             RC Oscillator Configuration
  0x00,  // RCCTRL0             RC Oscillator Configuration
  0x59,  // FSTEST              Frequency Synthesizer Calibration Control
  0x59,  // PTEST               Production Test
  0x3F,  // AGCTEST             AGC Test
  0x81,  // TEST2               Various Test Settings
  0x35,  // TEST1               Various Test Settings
  //0x0B,  // TEST0               Various Test Settings
  0x09,  // TEST0               Various Test Settings
};

/**
 * Print buffer of specified size into serial.
 */
void print_buffer(const byte* buffer, const int bufferSize) {
  for (int i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");
}

/**
 * Receive packet from network.
 * Writes to buffer.
 * Waits for a specified timeout before giving up.
 * Returns lengths of read packet.
 */
int receive_packet(byte* buffer, const int timeout) {
  ELECHOUSE_cc1101.SetRx();
  unsigned long startTime = millis();  // Record start time
  // Wait for the receive flag or until timeout
  while (!ELECHOUSE_cc1101.CheckReceiveFlag()) {
    if (millis() - startTime > timeout) return 0;
  }

  // Get received Data and calculate length
  int len = ELECHOUSE_cc1101.ReceiveData(buffer);

  ELECHOUSE_cc1101.SpiStrobe(0x36);  // Exit RX / TX, turn off frequency synthesizer and exit
  ELECHOUSE_cc1101.SpiStrobe(0x3A);  // Flush the RX FIFO buffer
  ELECHOUSE_cc1101.SpiStrobe(0x3B);  // Flush the TX FIFO buffer
  delay(2);
  ELECHOUSE_cc1101.SpiStrobe(0x34);  // Enable RX

  return len;  // Return the length of the received data
}

/**
 * Transmits packet to the network.
 * Transmits data from specified buffer of specified size.
 * Returns length of written data.
 */
void transmit_packet(const byte* data, const int data_size) {
  ELECHOUSE_cc1101.SpiStrobe(0x33);  // Calibrate frequency synthesizer and turn it off
  delay(2);
  ELECHOUSE_cc1101.SpiStrobe(0x3B);                   // Flush the TX FIFO buffer
  ELECHOUSE_cc1101.SpiStrobe(0x36);                   // Exit RX / TX, turn off frequency synthesizer and exit
  ELECHOUSE_cc1101.SpiWriteReg(0x3e, 0xC0);           //выставляем мощность 10dB
  ELECHOUSE_cc1101.SendData((byte*)data, data_size);  // Send packet
  ELECHOUSE_cc1101.SpiStrobe(0x3A);                   // Flush the RX FIFO buffer
  ELECHOUSE_cc1101.SpiStrobe(0x34);                   // Enable RX
}

/**
 * Read 4 bytes from buffer as uint32_t
 */
uint32_t readUInt32FromBuffer(const byte* buffer) {
  uint32_t value = 0;
  value |= buffer[0];  // LSB
  value |= (uint32_t)buffer[1] << 8;
  value |= (uint32_t)buffer[2] << 16;
  value |= (uint32_t)buffer[3] << 24;  // MSB
  return value;
}

/**
 * Generate CRC for packet buffer.
 * Buffer must be already filled.
 * Returns CRC.
 */
byte generate_packet_crc(const byte* buffer) {
  CRC8 crc;
  crc.reset();
  crc.setPolynome(0xA9);
  crc.add(buffer + 3, buffer[0] - 1 - 3);
  return crc.getCRC();
}

/* Mirtek proto requests */
static const byte MT_REQUEST_KIND_SHORT = 0x20;
static const byte MT_REQUEST_KIND_LONG = 0x21;

/* Mirtek trailer byte */
static const byte MT_TRAILER = 0x55;

/* Mirket magic bytes */
#define __MT_MAGIC 0x73, 0x55
static const byte MT_MAGIC[2] = { __MT_MAGIC };

/* Mirtek basic request struct */
typedef struct {
  byte size;
  byte magic[2] = { __MT_MAGIC };
  byte kind;
  byte reserved_1 = 0x0;
  byte meter_id[2];
  byte magic2[2] = { 0x9, 0xff };
  byte cmd;
  byte pin[4] = { 0x0, 0x0, 0x0, 0x0 };
} MTRequestTemplate;


/**
 * Generate mirtek request into buffer
 * Returns length of the generated packet.
 */
int generate_mt_request(const byte* meter_id, byte kind, byte cmd, byte* buffer, const int buffer_size) {
  MTRequestTemplate init;
  init.kind = kind;
  memcpy(&init.meter_id, meter_id, meter_id_size);
  init.cmd = cmd;

  // Full size of packet
  static const int packet_size = sizeof(MTRequestTemplate) + 2 /*crc & trailer*/ + (kind == MT_REQUEST_KIND_LONG ? 1 : 0) /*extra*/;
  init.size = packet_size - 1;

  if (packet_size > buffer_size) {
    Serial.print("Failed to generate packet 1: buffer size ");
    Serial.print(buffer_size);
    Serial.print(" is less than packet size ");
    Serial.println(packet_size);
    return 0;
  }

  memcpy(buffer, &init, sizeof(MTRequestTemplate));
  int i = sizeof(MTRequestTemplate);
  if (kind == MT_REQUEST_KIND_LONG) {
    buffer[i] = 0x00;
    i++;
  }

  buffer[i] = generate_packet_crc(buffer);
  i += 1;

  buffer[i] = MT_TRAILER;

  return packet_size;
}

/**
 * Data from reply for type-5 packet.
 */
struct PacketReply5 {
  float sum;
  float t1;
  float t2;
};

/**
 * Parse packet-5 reply into struct `res`.
 * Returns non-zero value on error.
 */
int parse_packet_5_reply(const byte* meter_id, const byte* buffer, struct PacketReply5* res) {
  int packet_size = buffer[0];

  // Verify trailer & size
  if (buffer[packet_size] != MT_TRAILER) {
    Serial.println("Failed to parse type-5 packet: incorrect trailer.");
    return 1;
  }

  // Verify magic
  for (int i = 0; i < sizeof(MT_MAGIC); i++) {
    if (buffer[1 + i] != MT_MAGIC[i]) {
      Serial.println("Failed to parse type-5 packet: incorrect magic.");
      return 1;
    }
  }

  // Verify header
  if (buffer[3] != 0x1E) {
    Serial.println("Failed to parse type-5 packet: incorrect header.");
    return 1;
  }

  // Verify meter address
  for (int i = 0; i < meter_id_size; i++) {
    if (buffer[7 + i] != meter_id[i]) {
      Serial.println("Failed to parse type-5 packet: incorrect meter id.");
      return 1;
    }
  }

  // Parse values
  uint32_t sum = readUInt32FromBuffer(buffer + 24);
  uint32_t t1 = readUInt32FromBuffer(buffer + 28);
  uint32_t t2 = readUInt32FromBuffer(buffer + 32);

  // Verify values sanity
  if (t1 + t2 != sum) {
    Serial.println("Failed to parse type-5 packet: t1 + t2 does not equal sum.");
    Serial.print("Sum: ");
    Serial.println(sum);
    Serial.print("t1: ");
    Serial.println(t1);
    Serial.print("t2: ");
    Serial.println(t2);
    return 1;
  }

  res->sum = float(sum) / 100;
  res->t1 = float(t1) / 100;
  res->t2 = float(t2) / 100;

  return 0;
}

void loop_iot() {
  iot_web_conf.doLoop();
  mqtt_client.loop();

  if (need_mqtt_connect) {
    need_mqtt_connect = !connect_mqtt();
  } else if ((iot_web_conf.getState() == iotwebconf::OnLine) && (!mqtt_client.connected())) {
    Serial.println("MQTT reconnect");
    connect_mqtt();
  }

  if (need_reset) {
    Serial.println("Rebooting after 1 second.");
    iot_web_conf.delay(1000);
    ESP.restart();
  }
}

void loop() {
  loop_iot();

  {
    const int buffer_size = 50;
    byte buffer[buffer_size];
    int len = generate_mt_request(meter_id, MT_REQUEST_KIND_LONG, 0x05, buffer, buffer_size);
    Serial.print("Generated packet 1: ");
    print_buffer(buffer, len);
    transmit_packet(buffer, len);
  }

  // Get response
  // Try to read pack of 20 packets.
  for(int i = 0; i < 20; i++) {
    int buffer_size = 50;
    byte buffer[buffer_size];

    int len = receive_packet(buffer, 1000);

    if (len > 0) {
      Serial.println("");
      Serial.print("Received packet: ");
      print_buffer(buffer, len);
      struct PacketReply5 info;
      if (parse_packet_5_reply(meter_id, buffer, &info) == 0) {
        Serial.print("SUM: ");
        Serial.println(info.sum);
        Serial.print("T1: ");
        Serial.println(info.t1);
        Serial.print("T2: ");
        Serial.println(info.t2);

        mqtt_client.publish("mirtek/" + (String)mirtek_id_value + "/SUM", String(info.sum));
        mqtt_client.publish("mirtek/" + (String)mirtek_id_value + "/T1", String(info.t1));
        mqtt_client.publish("mirtek/" + (String)mirtek_id_value + "/T2", String(info.t2));
      }
    }
    else {
      break; // stop if no packet read
    }
  }
}

void setup() {
  ELECHOUSE_cc1101.setGDO0(2);
  Serial.begin(BAUD_RATE);
  Serial.println("Ready.");

  if (ELECHOUSE_cc1101.getCC1101()) {  // Check the CC1101 SPI connection.
    Serial.println("SPI Connection CC1101 OK");
  } else {
    Serial.println("SPI Connection CC1101 Error");
  }

  //Инициализируем cc1101
  ELECHOUSE_cc1101.Init();

  ELECHOUSE_cc1101.SpiStrobe(0x30);  //reset
  ELECHOUSE_cc1101.SpiWriteBurstReg(0x00, rfSettings, 0x2F);
  ELECHOUSE_cc1101.SpiStrobe(0x33);  //Calibrate frequency synthesizer and turn it off
  delay(1);
  ELECHOUSE_cc1101.SpiStrobe(0x3A);  // Flush the RX FIFO buffer
  ELECHOUSE_cc1101.SpiStrobe(0x3B);  // Flush the TX FIFO buffer
  ELECHOUSE_cc1101.SpiStrobe(0x34);  // Enable RX

  ELECHOUSE_cc1101.setCrc(0);
  ELECHOUSE_cc1101.setAdrChk(0);

  // Iot webconf
  mqtt_group.addItem(&mqtt_server_param);
  mqtt_group.addItem(&mqtt_username_param);
  mqtt_group.addItem(&mqtt_password_param);

  mirtek_group.addItem(&mirtek_id_param);

  iot_web_conf.addParameterGroup(&mqtt_group);
  iot_web_conf.addParameterGroup(&mirtek_group);
  iot_web_conf.setConfigSavedCallback(&config_saved);
  iot_web_conf.setWifiConnectionCallback(&wifi_connected);

  // Initializing the configuration.
  if (!iot_web_conf.init()) {
    mqtt_server_value[0] = '\0';
    mqtt_username_value[0] = '\0';
    mqtt_password_value[0] = '\0';
    mirtek_id_value[0] = '\0';
  }

  // Set up required URL handlers on the web server.
  server.on("/", handle_root);
  server.on("/config", [] {
    iot_web_conf.handleConfig();
  });
  server.onNotFound([]() {
    iot_web_conf.handleNotFound();
  });

  mqtt_client.begin(mqtt_server_value, net);

  meter_id[0] = atoi(mirtek_id_value) & 0xff;
  meter_id[1] = (atoi(mirtek_id_value) >> 8) & 0xff;
}

/**
 * Handle web requests to "/" path.
 */
void handle_root() {
  // -- Let IotWebConf test and handle captive portal requests.
  if (iot_web_conf.handleCaptivePortal()) {
    return;
  }

  String s = "<!DOCTYPE html><html lang=\"en\"><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/>";
  s += "<title>IotWebConf 06 MQTT App</title></head><body>Mirtek MQTT Gateway";
  s += "<ul>";
  s += "<li>MQTT server: ";
  s += mqtt_server_value;
  s += "</li>";
  s += "<li>Mirket meter id: ";
  s += mirtek_id_value;
  s += "</li>";
  s += "</ul>";
  s += "Go to <a href='config'>configure page</a> to change values.";
  s += "</body></html>\n";

  server.send(200, "text/html", s);
}

void wifi_connected() {
  need_mqtt_connect = true;
}

void config_saved() {
  Serial.println("Configuration was updated.");
  need_reset = true;
}

bool connect_mqtt() {
  static unsigned long last_mqtt_connection_attempt = 0;

  unsigned long now = millis();
  if (1000 > now - last_mqtt_connection_attempt) {
    // Do not repeat within 1 sec.
    return false;
  }
  Serial.println("Connecting to MQTT server...");
  if (!connect_mqtt_options()) {
    last_mqtt_connection_attempt = now;
    return false;
  }
  Serial.println("Connected!");

  return true;
}

bool connect_mqtt_options() {
  bool result;
  if (mqtt_password_value[0] != '\0') {
    result = mqtt_client.connect(iot_web_conf.getThingName(), mqtt_username_value, mqtt_password_value);
  } else if (mqtt_username_value[0] != '\0') {
    result = mqtt_client.connect(iot_web_conf.getThingName(), mqtt_username_value);
  } else {
    result = mqtt_client.connect(iot_web_conf.getThingName());
  }
  return result;
}
