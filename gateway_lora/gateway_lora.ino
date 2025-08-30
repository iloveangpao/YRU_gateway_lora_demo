// ========= Gateway: ESP32 + RAK4270 (P2P Receiver) + Blynk =========
#include <Arduino.h>

/************ Fill these ************/
#define BLYNK_TEMPLATE_ID "TMPL61ElWjg6z"
#define BLYNK_TEMPLATE_NAME "My Demo Gateway"
#define BLYNK_AUTH_TOKEN "mVX08bu8RE5QYtQIbCvcBFLMiPVPPrTS"

const char* WIFI_SSID = "Yanrui";
const char* WIFI_PASS = "12345678";

/************ RAK4270 P2P params (must match your sender) ************/
const char* P2P_FREQ_HZ   = "923200000"; // AS923 example
const int   P2P_SF        = 10;          // 7..12
const int   P2P_BW        = 0;           // 0=125k,1=250k,2=500k
const int   P2P_CR        = 1;           // 1=4/5..4=4/8
const int   P2P_PREAMBLE  = 8;
const int   P2P_PWR_DBM   = 14;

/************ RAK UART on SoftwareSerial (ESP8266) ************/
// ESP RX <- RAK TX on D6 (GPIO12)
// ESP TX -> RAK RX on D5 (GPIO14)
#define RAK_RX_PIN D1
#define RAK_TX_PIN D2
static const uint32_t RAK_TARGET_BAUD = 57600; // keep SW serial ≤ ~57600 for reliability

/************ Includes ************/
#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SoftwareSerial.h>       // EspSoftwareSerial (ESP8266-ready)
#include <vector>

BlynkTimer timer;
SoftwareSerial RAK(RAK_RX_PIN, RAK_TX_PIN, false);  // RX, TX

/************ Logging helper ************/
#define LOG(tag, fmt, ...) do {                      \
  Serial.printf("[%lu] %s: ", millis(), tag);        \
  Serial.printf(fmt, ##__VA_ARGS__);                 \
  Serial.print("\r\n");                               \
} while (0)

// --- add near globals ---
volatile bool needBlynkConnect = false;
unsigned long lastConnectAttempt = 0;

// helper: try Blynk connect safely (non-blocking, with SW-serial paused)
bool tryBlynkConnect(uint32_t timeout_ms = 4000) {
  // pause SoftwareSerial RX during network/TLS handshakes (prevents WDT)
  RAK.enableRx(false);                    // <-- requires EspSoftwareSerial
  bool ok = Blynk.connect(timeout_ms/1000); // connect with explicit timeout (sec)
  RAK.enableRx(true);
  return ok;
}

/************ Node cache (multi-node) ************/
struct Node {
  bool     used = false;
  uint16_t id   = 0;
  uint16_t cnt  = 0;
  float    tC   = NAN;
  float    hP   = NAN;   // %RH
  int      rssi = 0;
  uint32_t seenMs = 0;
};
static const int MAX_NODES = 12;
Node nodes[MAX_NODES];

int findOrAllocNode(uint16_t id) {
  for (int i=0;i<MAX_NODES;i++) if (nodes[i].used && nodes[i].id==id) return i;
  int freeIdx = -1, lruIdx = 0; uint32_t lruSeen = UINT32_MAX;
  for (int i=0;i<MAX_NODES;i++) {
    if (!nodes[i].used) { freeIdx = i; break; }
    if (nodes[i].seenMs < lruSeen) { lruSeen = nodes[i].seenMs; lruIdx = i; }
  }
  int idx = (freeIdx>=0) ? freeIdx : lruIdx;
  nodes[idx] = Node{};
  nodes[idx].used = true; nodes[idx].id = id;
  return idx;
}
int countNodes() { int n=0; for (int i=0;i<MAX_NODES;i++) if (nodes[i].used) n++; return n; }

/************ Helpers: CRC & parsing ************/
uint16_t crc16_ibm(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i=0;i<len;i++) {
    crc ^= data[i];
    for (int b=0;b<8;b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}
bool hexToBytes(const String& hex, std::vector<uint8_t>& out) {
  if (hex.length()%2) return false;
  out.clear(); out.reserve(hex.length()/2);
  auto toNib = [](char c)->uint8_t{ c=toupper(c); return (c>='A')?(c-'A'+10):(c-'0'); };
  for (size_t i=0;i<hex.length(); i+=2) {
    char a=hex[i], b=hex[i+1];
    if (!isxdigit(a) || !isxdigit(b)) return false;
    out.push_back((toNib(a)<<4) | toNib(b));
  }
  return true;
}
// Accept: at+recv=<RSSI>,<SNR>,<LEN>:<HEX>   (RAK4270 prints this in P2P RX)
bool parseRecvLine(const String& line, int& rssi, int& snr, int& len, String& hex) {
  int eq = line.indexOf('=');
  if (eq < 0) return false;
  String head = line.substring(0, eq); head.toLowerCase(); head.trim();
  if (head != "at+recv") return false;
  int colon = line.indexOf(':', eq + 1);
  if (colon < 0) return false;
  String meta = line.substring(eq + 1, colon); meta.trim();
  int c1 = meta.indexOf(','); if (c1<0) return false;
  int c2 = meta.indexOf(',', c1+1); if (c2<0) return false;
  rssi = meta.substring(0, c1).toInt();
  snr  = meta.substring(c1+1, c2).toInt();
  len  = meta.substring(c2+1).toInt();
  hex = line.substring(colon + 1); hex.trim();
  return true;
}
/************ Line reader ************/
bool readLineFrom(Stream& s, String& out, unsigned long timeout_ms=5) {
  unsigned long t0 = millis(); out="";
  while (millis()-t0 < timeout_ms) {
    while (s.available()) {
      char c = (char)s.read();
      if (c=='\n') return out.length()>0;
      if (c!='\r') out += c;
    }
    delay(1);
  }
  return false;
}

/************ Blynk wiring ************/
void blynkSummary(uint16_t nodeId, float tC, float hP, int rssi) {
  Blynk.virtualWrite(V0, countNodes()); // total nodes
  Blynk.virtualWrite(V1, (int)nodeId);  // last node id
  Blynk.virtualWrite(V2, tC);           // last temp
  Blynk.virtualWrite(V3, hP);           // last humidity
  Blynk.virtualWrite(V4, rssi);         // last RSSI
}

/************ YOUR handler (kept verbatim) ************/
void handleRecvLine(const String& line) {
  int rssi=0, snr=0, len=0; String hex;
  if (!parseRecvLine(line, rssi, snr, len, hex)) return;

  std::vector<uint8_t> bytes;
  if (!hexToBytes(hex, bytes) || (int)bytes.size()!=len) { Serial.println("[RX] Bad HEX/LEN"); return; }
  if (bytes.size()<11 || bytes[0]!=0xA5) { Serial.println("[RX] Bad frame prefix/size"); return; }

  uint16_t nodeId = (bytes[1]<<8)|bytes[2];
  uint16_t cnt    = (bytes[3]<<8)|bytes[4];
  int16_t  tC100  = (int16_t)((bytes[5]<<8)|bytes[6]);
  int16_t  hP100  = (int16_t)((bytes[7]<<8)|bytes[8]);
  uint16_t rxCRC  = (bytes[9]<<8)|bytes[10];
  uint16_t calc   = crc16_ibm(bytes.data(), 9);
  if (calc != rxCRC) { Serial.println("[RX] CRC mismatch"); return; }

  float tC = tC100/100.0f;
  float hP = hP100/100.0f;

  int idx = findOrAllocNode(nodeId);
  nodes[idx].cnt   = cnt;
  nodes[idx].tC    = tC;
  nodes[idx].hP    = hP;
  nodes[idx].rssi  = rssi;
  nodes[idx].seenMs= millis();

  Serial.printf("[RX] node=0x%04X cnt=%u temp=%.2fC hum=%.2f%% RSSI=%d SNR=%d\n",
                nodeId, cnt, tC, hP, rssi, snr);

  blynkSummary(nodeId, tC, hP, rssi);
}

/************ RAK UART helpers ************/
bool sendAT(const String& cmd, unsigned long to=2000) {
  LOG("RAK>", "%s", cmd.c_str());
  RAK.print(cmd); RAK.print("\r\n");
  String acc; unsigned long t0 = millis();
  while (millis()-t0 < to) {
    while (RAK.available()) {
      char c=(char)RAK.read();
      if (c=='\n') {
        if (acc.startsWith("OK")) { LOG("RAK<", "%s", acc.c_str()); return true; }
        if (acc.startsWith("ERROR")) { LOG("RAK<", "%s", acc.c_str()); return false; }
        LOG("RAK<", "%s", acc.c_str());
        acc="";
      } else if (c!='\r') acc+=c;
    }
    delay(1);
  }
  return false;
}
bool probeOK(unsigned long to=500) {
  RAK.flush(); RAK.print("AT\r\n");
  unsigned long t0 = millis(); String acc="";
  while (millis()-t0 < to) {
    while (RAK.available()) {
      char c=(char)RAK.read();
      if (c=='\n') { bool ok = acc.startsWith("OK"); acc=""; if (ok) return true; }
      else if (c!='\r') acc+=c;
    }
  }
  return false;
}
// Try target baud first; if not, try 115200 then reconfigure down to target.
bool initRakBaud() {
  RAK.begin(RAK_TARGET_BAUD);
  delay(80);
  if (probeOK()) { LOG("RAK", "Linked at %lu", (unsigned long)RAK_TARGET_BAUD); return true; }

  LOG("RAK", "No response at %lu, trying 115200...", (unsigned long)RAK_TARGET_BAUD);
  RAK.end(); delay(20);
  RAK.begin(115200);
  delay(120);
  if (!probeOK()) { LOG("RAK", "No response at 115200 either"); return false; }

  // Set both UARTs to target (RAK4270 supports DEVICE:UART:<idx>:<baud>)
  sendAT(String("AT+SET_CONFIG=DEVICE:UART:1:") + RAK_TARGET_BAUD);
  sendAT(String("AT+SET_CONFIG=DEVICE:UART:2:") + RAK_TARGET_BAUD);

  RAK.end(); delay(20);
  RAK.begin(RAK_TARGET_BAUD);
  delay(80);
  bool ok = probeOK();
  LOG("RAK", "Re-linked at %lu: %s", (unsigned long)RAK_TARGET_BAUD, ok?"OK":"FAIL");
  return ok;
}
void rakSetupP2P() {
  // Switch to P2P and set params; then RX transfer mode
  sendAT("AT+SET_CONFIG=LORA:WORK_MODE:1"); // 1=P2P
  char buf[128];
  snprintf(buf,sizeof(buf),"AT+SET_CONFIG=LORAP2P:%s:%d:%d:%d:%d:%d",
           P2P_FREQ_HZ, P2P_SF, P2P_BW, P2P_CR, P2P_PREAMBLE, P2P_PWR_DBM);
  sendAT(buf, 3000);
  sendAT("AT+SET_CONFIG=LORAP2P:TRANSFER_MODE:1"); // receiver
}

/************ Wi-Fi/Blynk events ************/
WiFiEventHandler onGotIP, onDisc;

void startWifi() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  onGotIP = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP& e){
    LOG("WIFI", "Got IP: %s", e.ip.toString().c_str());
    needBlynkConnect = true;            // defer actual connect to loop()
  });

  onDisc = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected& e){
    LOG("WIFI", "Disconnected, reason=%d", e.reason);
    // no delay() here; just trigger reconnects from loop()
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    needBlynkConnect = false;           // will try again after we regain IP
  });
  LOG("WIFI", "Connecting to %s ...", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

/************ RAK receive pump ************/
void pumpRak() {
  while (RAK.available()) {
    String line;
    if (!readLineFrom(RAK, line, 1)) break;
    if (line.length()==0) continue;
    handleRecvLine(line); // <-- your parser & Blynk update
  }
}

/************ setup/loop ************/
void setup() {
  Serial.begin(115200);
  delay(200);
  LOG("BOOT", "ESP8266 RAK P2P → Blynk");

  Blynk.config(BLYNK_AUTH_TOKEN);   // non-blocking
  startWifi();

  if (!initRakBaud()) {
    LOG("RAK", "Failed to init UART link; check wiring & power");
  } else {
    rakSetupP2P();
  }

  timer.setInterval(5000, [](){ Blynk.virtualWrite(V0, countNodes()); });
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!Blynk.connected()) {
      // rate-limit connect attempts to avoid repeated blocking
      if (needBlynkConnect && millis() - lastConnectAttempt > 5000UL) {
        lastConnectAttempt = millis();
        LOG("BLYNK", "Connecting...");
        bool ok = tryBlynkConnect(4000);
        LOG("BLYNK", "%s", ok ? "Connected" : "Connect timeout");
        if (!ok) {
          // skip Blynk.run() when disconnected to avoid hidden retries blocking loop
        }
      }
    } else {
      Blynk.run();     // only when connected
    }
  }

  timer.run();
  pumpRak(); // keep SW serial drained frequently
}
