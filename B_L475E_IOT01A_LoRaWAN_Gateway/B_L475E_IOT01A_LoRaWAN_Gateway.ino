/*
 * B_L475E_IOT01A_LoRaWAN_Gateway
 * 
 * This project aims to create a single channel LoRaWAN gateway that 
 * formward the LoRaWAN messages received from LoRaWAN nodes to The Things Network
 * European Server ("router.eu.thethings.network") using UDP protocol.
 * 
 * The gateway has been made with the following devices:
 * 1) B-L475E-IOT01A2 Discovery kit 
 * 2) Dragino LoRa/GPS Hat
 * 
 * This project is based on "single_chan_pkt_fwd" made by Thomas Telkamp
 *
 * NOTE: This implementation does not support downstream messages processing,
 *       so only ABP (Activation By Personalization) is supported.
 *       OTAA (Over the Air Activation) is not supported yet.
 *
 * Copyright (c) 2018 Biagio Montaruli 
 */

// Include necessary libraries
#include <SPI.h>
#include <WiFiST.h>
#include <WiFiUdpST.h>
#include <time.h>
#include <Base64.h>
#include "loraConfig.h"

SPIClass SPI_3(PC12, PC11, PC10);
WiFiClass WiFi(&SPI_3, PE0, PE1, PE8, PB13);

int wifiStatus = WL_IDLE_STATUS;

// UDP instance to connect to TTN and to NTP Server
WiFiUDP Udp;

#define SERIAL_DEBUG 1

#define LORA_MAX_PACKET_SIZE 256

// Message received from LoRaWAN nodes
char receivedMessage[LORA_MAX_PACKET_SIZE];

bool sx1272 = true;

uint16_t receivedBytes;

uint32_t cp_nb_rx_rcv = 0;    // counter for received LoRa packet
uint32_t cp_nb_rx_ok = 0;     // counter for successful received LoRa packet
uint32_t cp_nb_rx_bad = 0;    // counter for bad received LoRa packet
uint32_t cp_nb_rx_nocrc = 0;  // counter for received packet withouth CRC enabled
uint32_t cp_up_pkt_fwd = 0;   // counter for forwarded packet

enum sf_t { SF7 = 7, SF8 = 8, SF9 = 9, SF10 = 10, SF11 = 11, SF12 = 12 };

#define PACKET_HEADER_LEN 12

// NTP time stamp is in the first 48 bytes of the message
const int NTP_PACKET_SIZE = 48;
//buffer to hold incoming and outgoing packets
char packetBuffer[NTP_PACKET_SIZE];
// IP Address of the NTP server: 188.213.165.209 => it.pool.ntp.org
IPAddress timeServer(188, 213, 165, 209);
unsigned long epoch = 0;
// Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
const unsigned long seventyYears = 2208988800UL;

uint32_t lastTimeSec;

// LoRaWAN Gateway ID
byte gwID[8];

/* --- CONFIGURE THE FOLLOWING VALUES ACCORDING TO YOUR CONFIGURATION --- */

char ssid[] = "your-ssid";   // your network SSID
char pass[] = "your-pass";   // your network password

// SX1276 - Arduino connections
int ssPin = 10;
int dio0Pin = 2;
int rstPin  = 9;

// Set spreading factor (SF7 - SF12)
sf_t sf = SF7;

// Set center frequency
uint32_t freq = 868100000;
float frequency = 868.100;
uint8_t channel = 0;
uint8_t rfChannel = 0;
uint16_t bandWidth = 125;
String codingRate = "4/5";

// Set location
float lat = 41.10948630;
float lon = 16.87783100;
int   alt = 13;

// Gateway info fields
String gatewayInfo = "B-L475E-IOT01A2 Single Channel Gateway";
String email = "";
String gwDescription = "B-L475E-IOT01A2 LoRa Gateway";

// IP Address and host name of the Network Server
// The Things Network Europe Server: router.eu.thethings.network
IPAddress TTN_Server_IP(52, 169, 76, 203);
String TTNServer = "router.eu.thethings.network";
// The port on which to send data through UDP
#define PORT 1700

/* ----------------- LoRaWAN fuctions -------------------------- */

void selectLoRaModule() {
  digitalWrite(ssPin, LOW);
}

void unselectLoRaModule() {
  digitalWrite(ssPin, HIGH);
}

byte readRegister(byte addr) {
  uint8_t response;

  selectLoRaModule();
  SPI.beginTransaction(SPISettings(8E6, MSBFIRST, SPI_MODE0));
  SPI.transfer(addr & 0x7F);
  response = SPI.transfer(0x00);
  SPI.endTransaction();
  unselectLoRaModule();

  return response;
}

void writeRegister(byte addr, byte value) {
  uint8_t response;

  selectLoRaModule();
  SPI.beginTransaction(SPISettings(8E6, MSBFIRST, SPI_MODE0));
  SPI.transfer(addr | 0x80);
  response = SPI.transfer(value);
  SPI.endTransaction();

  unselectLoRaModule();
}

boolean getLoRaMessage(char *payload) {
  // clear rxDone
  writeRegister(REG_IRQ_FLAGS, 0x40);

  int irqflags = readRegister(REG_IRQ_FLAGS);

  cp_nb_rx_rcv++;

  //  check payload crc: 0x20
  if ((irqflags & 0x20) == 0x20) {
    Serial.println("CRC error");
    writeRegister(REG_IRQ_FLAGS, 0x20);
    return false;
  }
  else {
    
    cp_nb_rx_ok++;

    byte currentAddr = readRegister(REG_FIFO_RX_CURRENT_ADDR);
    byte receivedCount = readRegister(REG_RX_NB_BYTES);
    receivedBytes = receivedCount;
    #if SERIAL_DEBUG == 1
    Serial.print("Received ");
    Serial.print(receivedBytes);
    Serial.println(" bytes");
    #endif

    writeRegister(REG_FIFO_ADDR_PTR, currentAddr);

    for (int i = 0; i < receivedBytes; i++) {
      payload[i] = (char)readRegister(REG_FIFO);
    }
  }
  return true;
}

void setupLoRaModule() {
  // setup pins
  pinMode(dio0Pin, INPUT);

  pinMode(ssPin, OUTPUT);
  // set SS pin high
  digitalWrite(ssPin, HIGH);

  pinMode(rstPin, OUTPUT);

  // perform reset
  digitalWrite(rstPin, LOW);
  delay(10);
  digitalWrite(rstPin, HIGH);
  delay(10);

  // start SPI
  SPI.begin();

  byte version = readRegister(REG_VERSION);

  // check if radio module is sx1276
  if (version == 0x22) {
    // sx1272 radio module detected
    Serial.println("SX1272 radio module detected");
    sx1272 = true;
  }
  else { // check if radio module is sx1276
    digitalWrite(rstPin, LOW);
    delay(10);
    digitalWrite(rstPin, HIGH);
    delay(10);
    version = readRegister(REG_VERSION);
    if (version == 0x12) {
      // sx1276 detected
      Serial.println("SX1276 radio module detected");
      sx1272 = false;
    }
    else {
      Serial.println("Unrecognized radio module.");
      while (true) ;
    }
  }

  writeRegister(REG_OPMODE, SX72_MODE_SLEEP);

  // set frequency
  uint64_t frf = ((uint64_t)freq << 19) / 32000000;
  writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16) );
  writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8) );
  writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0) );

  writeRegister(REG_SYNC_WORD, 0x34); // LoRaWAN public sync word

  if (sx1272) {
    if (sf == SF11 || sf == SF12) {
      writeRegister(REG_MODEM_CONFIG, 0x0B);
    }
    else {
      writeRegister(REG_MODEM_CONFIG, 0x0A);
    }
    writeRegister(REG_MODEM_CONFIG2, (sf << 4) | 0x04);
  }
  else {
    if (sf == SF11 || sf == SF12) {
      writeRegister(REG_MODEM_CONFIG3, 0x0C);
    }
    else {
      writeRegister(REG_MODEM_CONFIG3, 0x04);
    }
    writeRegister(REG_MODEM_CONFIG, 0x72);
    writeRegister(REG_MODEM_CONFIG2, (sf << 4) | 0x04);
  }

  if (sf == SF10 || sf == SF11 || sf == SF12) {
    writeRegister(REG_SYMB_TIMEOUT_LSB, 0x05);
  }
  else {
    writeRegister(REG_SYMB_TIMEOUT_LSB, 0x08);
  }
  writeRegister(REG_MAX_PAYLOAD_LENGTH, 0x80);
  writeRegister(REG_PAYLOAD_LENGTH, PAYLOAD_LENGTH);
  writeRegister(REG_HOP_PERIOD, 0xFF);
  writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_BASE_AD));

  // Set Continous Receive Mode
  writeRegister(REG_LNA, LNA_MAX_GAIN);  // max lna gain
  writeRegister(REG_OPMODE, SX72_MODE_RX_CONTINUOS);

}

/* ------------------- Network functions ------------------------*/

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress& serverIP) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);

  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;            // Stratum, or type of clock
  packetBuffer[2] = 6;            // Polling Interval
  packetBuffer[3] = 0xEC;         // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  while(Udp.beginPacket(serverIP, 123) == 0) {  // NTP requests are to port 123
    Serial.println("Failed to start UDP connection to NTP Server. Retrying in 2 seconds...");
    delay(2000);
  }
  
  size_t pktSentLen;
  while((pktSentLen = Udp.write(packetBuffer, NTP_PACKET_SIZE)) == 0) {
    Serial.println("Failed to send the UDP packet. Retrying in 3 seconds...");
    delay(3000);
  }
  
  Serial.print("Length of the packet sent to NTP Server: ");
  Serial.println(pktSentLen);
}

void printWifiStatus() {
  // print the SSID of the network you're connected to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the BSSID of the network you're connected to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  
  Serial.print("BSSID: ");
  for (uint8_t i = 0; i < 6; i++) {
    if (bssid[i] < 0x10) {
      Serial.print("0");
    }
    
    Serial.print(bssid[i], HEX);
    
    if (i != 5) {
      Serial.print(":");
    }
    else {
      Serial.println();
    }
  }

  // print your WiFi device's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the subnet mask of the WiFi network you're connected to:
  IPAddress subMask = WiFi.subnetMask();
  Serial.print("Subnet Mask: ");
  Serial.println(subMask);

  // print your WiFi device's MAC address
  byte macAddr[6];
  WiFi.macAddress(macAddr);

  Serial.print("MAC Address: ");
  for (uint8_t i = 0; i < 6; i++) {
    if (macAddr[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(macAddr[i], HEX);

    if (i != 5) {
      Serial.print(":");
    }
    else {
      Serial.println();
    }
  }

  // build the Gateway ID and print it in the Serial Monitor
  gwID[0] = macAddr[0];
  gwID[1] = macAddr[1];
  gwID[2] = macAddr[2];
  gwID[3] = 0xFF;
  gwID[4] = 0xFE;
  gwID[5] = macAddr[3];
  gwID[6] = macAddr[4];
  gwID[7] = macAddr[5];
  
  Serial.print("LoRaWAN Gateway ID: ");
  for (uint8_t i = 0; i < 8; i++) {
    if (gwID[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(gwID[i], HEX);

    if (i != 7) {
      Serial.print(":");
    }
    else {
      Serial.println();
    }
  }

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI): ");
  Serial.print(rssi);
  Serial.println(" dBm");

  // print encryption type:
  Serial.print("Encryption type: ");
  printEncryptionType(WiFi.encryptionType());
}

void printEncryptionType(int thisType) {
  // read the encryption type and print out the name:
  switch (thisType) {
    case ES_WIFI_SEC_OPEN:
      Serial.println("OPEN");
      break;
    case ES_WIFI_SEC_WEP:
      Serial.println("WEP");
      break;
    case ES_WIFI_SEC_WPA:
      Serial.println("WPA");
      break;
    case ES_WIFI_SEC_WPA2:
      Serial.println("WPA2");
      break;
    case ES_WIFI_SEC_WPA_WPA2:
      Serial.println("WPA_WPA2");
      break;
     case ES_WIFI_SEC_WPA2_TKIP:
      Serial.println("WPA_TKIP");
      break;
     case ES_WIFI_SEC_UNKNOWN:
      Serial.println("UNKNOW");
      break;
  }
}

String getFullFormattedTime(unsigned long epoch, unsigned long timeOffset) {
  time_t rawtime = epoch + timeOffset;
  struct tm * ti;
  ti = localtime(&rawtime);

  uint16_t year = ti->tm_year + 1900;
  String yearStr = String(year);

  uint8_t month = ti->tm_mon + 1;
  String monthStr = month < 10 ? "0" + String(month) : String(month);

  uint8_t day = ti->tm_mday;
  String dayStr = day < 10 ? "0" + String(day) : String(day);

  uint8_t hours = ti->tm_hour;
  String hoursStr = hours < 10 ? "0" + String(hours) : String(hours);

  uint8_t minutes = ti->tm_min;
  String minuteStr = minutes < 10 ? "0" + String(minutes) : String(minutes);

  uint8_t seconds = ti->tm_sec;
  String secondStr = seconds < 10 ? "0" + String(seconds) : String(seconds);

  return yearStr + "-" + monthStr + "-" + dayStr + " " +
         hoursStr + ":" + minuteStr + ":" + secondStr;
}

void updateNTP() {
  // send an NTP packet to a NTP server
  sendNTPpacket(timeServer);

  // wait to see if a reply is available
  delay(1000);

  // read the packet into the buffer
  int len = Udp.read(packetBuffer, NTP_PACKET_SIZE);
  if (len > 0) {

    #if SERIAL_DEBUG == 1
    Serial.println("New NTP packet received");
    #endif

    // the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    // now convert NTP time into everyday time subtracting seventy years:
    epoch = secsSince1900 - seventyYears;

    #if SERIAL_DEBUG == 1
    Serial.print("Seconds since Jan 1 1900 = ");
    Serial.println(secsSince1900);
    // print Unix time:
    Serial.print("Unix time: ");
    Serial.println(epoch);
    // print time and date
    String NTPTime = getFullFormattedTime(epoch, 3600);
    Serial.print("NTP Time: ");
    Serial.println(NTPTime);
    #endif

    Udp.stop();
    delay(1000);
  }
}

void sendUdp(const char *msg, uint16_t len) {
  while(Udp.beginPacket(TTNServer.c_str(), PORT) != 1) {
    Serial.println("Failed to start UDP connection to TTN. Retrying in 2 seconds...");
    delay(2000);
  }
  size_t sentPktLen;
  while((sentPktLen = Udp.write((const uint8_t *)msg, len)) == 0) {
    Serial.println("Failed to send the UDP packet. Retrying in 3 seconds...");
    delay(3000);
  }
  delay(1000);
  Udp.stop();
  delay(1000);

  Serial.print("Length of packet sent to TTN: ");
  Serial.println(sentPktLen);
}

/* -------------------- Gateway functions ---------------------- */

void parseLoRaPacket() {

  long snr;
  byte rssiCorrection;
  int rssi;

  memset(receivedMessage, 0, LORA_MAX_PACKET_SIZE);
  
  // save the new LoRaWAN packet received and parse it
  if (getLoRaMessage(receivedMessage)) {
    byte value = readRegister(REG_PKT_SNR_VALUE);

    // The SNR sign bit is 1
    if (value & 0x80) {
      // Invert and divide by 4
      value = ((~value + 1) & 0xFF) >> 2;
      snr = -value;
    }
    else {
      // Divide by 4
      snr = (value & 0xFF) >> 2;
    }

    if (sx1272 == true) {
      rssiCorrection = 139;
    }
    else {
      rssiCorrection = 157;
    }

    rssi = readRegister(0x1A) - rssiCorrection;

    #if SERIAL_DEBUG == 1
    Serial.println("Packet RSSI: " + String(rssi));
    Serial.println("RSSI: " + String(readRegister(0x1B) - rssiCorrection));
    Serial.println("SNR: " + String(snr));
    Serial.println("Length: " + String(receivedBytes));
    #endif

    #if SERIAL_DEBUG == 1
    Serial.print("LoRaWAN packet payload: ");
    for (uint16_t i = 0; i < receivedBytes; i++) {
      Serial.print(receivedMessage[i]);
    }
    Serial.println();
    #endif

    uint16_t encodedLen = Base64.encodedLength(receivedBytes);
    char encodedMessage[encodedLen];
    uint16_t len = Base64.encode(encodedMessage, receivedMessage, receivedBytes);

    if (len == encodedLen && len != 0) {
      
      #if SERIAL_DEBUG == 1
      Serial.println("Encoded msg OK");

      Serial.print("LoRa Data encoded: ");
      Serial.println(encodedMessage);
      #endif

      uint8_t packetHeader[12];
      
      // start composing the UDP packet
      uint8_t token_h = (uint8_t)rand();  // random token
      uint8_t token_l = (uint8_t)rand();  // random token

      // pre-fill the UDP packet header with fixed fields
      packetHeader[0] = PROTOCOL_VERSION;
      packetHeader[1] = token_h;
      packetHeader[2] = token_l;
      packetHeader[3] = PKT_PUSH_DATA;

      packetHeader[4] = gwID[0];
      packetHeader[5] = gwID[1];
      packetHeader[6] = gwID[2];
      packetHeader[7] = gwID[3];
      packetHeader[8] = gwID[4];
      packetHeader[9] = gwID[5];
      packetHeader[10] = gwID[6];
      packetHeader[11] = gwID[7];

      String udpPacket;
      for (uint8_t i = 0; i < PACKET_HEADER_LEN; i++) {
        udpPacket.concat((char)packetHeader[i]);
      }
      
      updateNTP();
      delay(3000);

      // start of JSON structure
      udpPacket += "{\"rxpk\":[{";
      // add timestamp
      udpPacket += "\"tmst\":";
      udpPacket += String(epoch);
      // add channel
      udpPacket += ",\"chan\":";
      udpPacket += String(channel);
      // add rf channel
      udpPacket += ",\"rfch\":";
      udpPacket += String(rfChannel);
      // add frequency
      udpPacket += ",\"freq\":";
      udpPacket += String(frequency, 2);
      // enable statistics
      udpPacket += ",\"stat\":1";
      // set modulation: LORA
      udpPacket += ",\"modu\":\"LORA\"";
      // Set Spreading Factor and data rate
      switch (sf) {
        case SF7:
          udpPacket += ",\"datr\":\"SF7";
          break;
        case SF8:
          udpPacket += ",\"datr\":\"SF8";
          break;
        case SF9:
          udpPacket += ",\"datr\":\"SF9";
          break;
        case SF10:
          udpPacket += ",\"datr\":\"SF10";
          break;
        case SF11:
          udpPacket += ",\"datr\":\"SF11";
          break;
        case SF12:
          udpPacket += ",\"datr\":\"SF12";
          break;
        default:
          udpPacket += ",\"datr\":\"SF?";
      }
      
      udpPacket += "BW";
      udpPacket += String(bandWidth);
      // set Coding Rate (CR)
      udpPacket += "\",\"codr\":\"";
      udpPacket += codingRate;
      // set SNR (Signal to Noise Ratio)
      udpPacket += "\",\"lsnr\":";
      udpPacket += String(snr);
      udpPacket += ",\"rssi\":";
      udpPacket += String(rssi);
      // add payload size
      udpPacket += ",\"size\":";
      udpPacket += String(receivedBytes);
      // add payload data encoded (Base64)
      udpPacket += ",\"data\":\"";
      udpPacket += encodedMessage;
      udpPacket += "\"";
      // End of packet serialization
      udpPacket += "}";
      udpPacket += "]";
      // end of JSON datagram payload
      udpPacket += "}";

      uint16_t packetLen = udpPacket.length();
      // send the UDP packet
      sendUdp(udpPacket.c_str(), packetLen);

      #if SERIAL_DEBUG == 1
      Serial.println("Packet sent: ");
      Serial.println(udpPacket);
      #endif
    }
    else {
      #if SERIAL_DEBUG == 1
      Serial.println("Error encoding the LoRaWAN received message");
      Serial.print("len: ");
      Serial.println(len);
      Serial.print("encodedLen: ");
      Serial.println(encodedLen);
      #endif
    }
  }
}

void sendStats() {

  uint8_t packetHeader[12];
  uint8_t token_h = (uint8_t)rand(); // random token
  uint8_t token_l = (uint8_t)rand(); // random token

  // pre-fill the udp packet buffer with the header fields
  packetHeader[0] = PROTOCOL_VERSION;
  packetHeader[1] = token_h;
  packetHeader[2] = token_l;
  packetHeader[3] = PKT_PUSH_DATA;

  packetHeader[4] = gwID[0];
  packetHeader[5] = gwID[1];
  packetHeader[6] = gwID[2];
  packetHeader[7] = gwID[3];
  packetHeader[8] = gwID[4];
  packetHeader[9] = gwID[5];
  packetHeader[10] = gwID[6];
  packetHeader[11] = gwID[7];

  String gwStatPacket;
  for (uint8_t i = 0; i < 12; i++) {
    gwStatPacket.concat((char)packetHeader[i]);
  }

  // get timestamp for statistics
  updateNTP();
  delay(3000);
  
  gwStatPacket += "{\"stat\":{\"time\":\"";
  gwStatPacket += getFullFormattedTime(epoch, 3600);
  gwStatPacket += " GMT\",\"lati\":";
  gwStatPacket += String(lat, 5);
  gwStatPacket += ",\"long\":";
  gwStatPacket += String(lon, 5);
  gwStatPacket += ",\"alti\":";
  gwStatPacket += String(alt);
  gwStatPacket += ",\"rxnb\":";
  gwStatPacket += String(cp_nb_rx_rcv);
  gwStatPacket += ",\"rxok\":";
  gwStatPacket += String(cp_nb_rx_ok);
  gwStatPacket += ",\"rxfw\":";
  gwStatPacket += String(cp_up_pkt_fwd);
  gwStatPacket += ",\"ackr\":";
  gwStatPacket += String(0);
  gwStatPacket += ",\"dwnb\":";
  gwStatPacket += String(0);
  gwStatPacket += ",\"txnb\":";
  gwStatPacket += String(0);
  gwStatPacket += ",\"pfrm\":\"";
  gwStatPacket += gatewayInfo;
  gwStatPacket += "\",\"mail\":\"";
  gwStatPacket += email;
  gwStatPacket += "\",\"desc\":\"";
  gwStatPacket += gwDescription;
  gwStatPacket += "\"}}";

  uint16_t packetLen = gwStatPacket.length();
  #if SERIAL_DEBUG == 1
  Serial.print("Length of gateway statistics packet: ");
  Serial.println(packetLen);
  Serial.print("Payload of stat packet: ");
  // print stats in JSON format
  Serial.println(gwStatPacket);
  #endif

  // send statistics using UDP protocol
  sendUdp(gwStatPacket.c_str(), packetLen);
}

void renewConnection() {

  uint8_t packetHeader[12];
  
  uint8_t token_h = (uint8_t)rand(); // random token
  uint8_t token_l = (uint8_t)rand(); // random token

  // pre-fill the data buffer with fixed fields
  packetHeader[0] = PROTOCOL_VERSION;
  packetHeader[1] = token_h;
  packetHeader[2] = token_l;
  packetHeader[3] = PKT_PUSH_DATA;

  packetHeader[4] = gwID[0];
  packetHeader[5] = gwID[1];
  packetHeader[6] = gwID[2];
  packetHeader[7] = gwID[3];
  packetHeader[8] = gwID[4];
  packetHeader[9] = gwID[5];
  packetHeader[10] = gwID[6];
  packetHeader[11] = gwID[7];

  String udpPacket;
  
  for (uint8_t i = 0; i < PACKET_HEADER_LEN; i++) {
    udpPacket.concat((char)packetHeader[i]);
  }
  
  // send the udp packet
  sendUdp(udpPacket.c_str(), udpPacket.length());
}

/* -------------------- Sketch main functions --------------------------- */

void setup() {
  // Open serial communications and wait for Serial monitor to open:
  Serial.begin(115200);
  while (!Serial) ;

  // initialize the WiFi module:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi module not connected");
    // don't continue:
    while (true) ;
  }

  // print firmware version
  String fv = WiFi.firmwareVersion();
  Serial.print("Firwmare version: ");
  Serial.println(fv);

  // attempt to connect to WiFi network:
  while (wifiStatus != WL_CONNECTED) {
    Serial.print("Attempting to connect to WiFi network: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network.
    wifiStatus = WiFi.begin(ssid, pass);

    // wait 8 seconds before reconnecting:
    delay(8000);
  }

  Serial.println("WiFi connected.");
  
  printWifiStatus();

  setupLoRaModule();

  // add callback function "parseLoRaPacket" that parse LoRa messages received from LoRa Nodes
  // and send them to TTN using UDP protocol 
  attachInterrupt(digitalPinToInterrupt(dio0Pin), parseLoRaPacket, RISING);
  
  Serial.println("Listening at SF" + String(sf) + " on " + String(frequency, 2) + " Mhz.");
  Serial.println("------------------------------------------\n");

  lastTimeSec = millis() / 1000;
}

void loop() {

  uint32_t currentTimeSec = millis() / 1000;

  if (currentTimeSec - lastTimeSec >= 30) {
    lastTimeSec = currentTimeSec;
    sendStats();
  }
}
