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

/************ RAK UART for ESP32 ************/
// ESP32 RX2 (GPIO16) <- RAK4270 TX2
// ESP32 TX2 (GPIO17) -> RAK4270 RX2
#define RAK_RX_PIN 16
#define RAK_TX_PIN 17
static const uint32_t RAK_TARGET_BAUD = 115200;

/************ Includes ************/
#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <vector>

/************ Constants ************/
#define RESPONSE_TIMEOUT_MS 1000  // Wait up to 1s for response
#define POLL_INTERVAL_MS 5000     // Poll every 5 seconds
#define MAX_RETRY_COUNT 3         // Maximum number of retries per node

// List of nodes to poll (add your node IDs here)
const uint16_t EDGE_NODES[] = {0x1234, 0x0001, 0x0002}; // Match edge node IDs
const int NUM_EDGE_NODES = sizeof(EDGE_NODES) / sizeof(EDGE_NODES[0]);

BlynkTimer timer;

// Using HardwareSerial for ESP32
HardwareSerial RAK(2);

/************ Logging helper ************/
#define LOG(tag, fmt, ...) do {                      \
  Serial.printf("[%lu] %s: ", millis(), tag);        \
  Serial.printf(fmt, ##__VA_ARGS__);                 \
  Serial.print("\r\n");                               \
} while (0)

// --- add near globals ---
volatile bool needBlynkConnect = false;
unsigned long lastConnectAttempt = 0;

// helper: try Blynk connect safely (non-blocking)
bool tryBlynkConnect(uint32_t timeout_ms = 4000) {
  // No need to pause HardwareSerial - it's hardware buffered
  bool ok = Blynk.connect(timeout_ms/1000); // connect with explicit timeout (sec)
  return ok;
}

/************ Request/Response Frame Handling ************/
// Send request to specific node
bool sendRequest(uint16_t targetNode) {
  // Switch to TX mode first (use same AT command format as edge node)
  if (!sendAT("at+set_config=lorap2p:transfer_mode:2", 1000)) {  // 2 = TX mode
    LOG("RAK", "Failed to switch to TX mode");
    return false;
  }
  
  // Format: [A1][target_node(2)][CRC16(2)]
  uint8_t frame[5];
  frame[0] = 0xA1;
  frame[1] = (targetNode >> 8) & 0xFF;
  frame[2] = targetNode & 0xFF;
  uint16_t crc = crc16_ibm(frame, 3);
  frame[3] = (crc >> 8) & 0xFF;
  frame[4] = crc & 0xFF;

  // Convert to hex string
  char hexStr[11];
  for(int i = 0; i < 5; i++) {
    sprintf(hexStr + (i * 2), "%02X", frame[i]);
  }
  
  LOG("RAK>", "Sending request to node 0x%04X: %s", targetNode, hexStr);
  
  // Send using RAK P2P transmit command (match edge node format)
  String cmd = String("at+send=lorap2p:") + hexStr;
  bool ok = sendAT(cmd, 3000);
  
  // Switch to RX mode after sending
  if (ok) {
    ok = sendAT("at+set_config=lorap2p:transfer_mode:1", 1000);  // 1 = RX mode
    if (!ok) {
      LOG("RAK", "Failed to switch to RX mode");
    }
  }
  
  return ok;
}

// Wait for and process response
bool waitForResponse(uint32_t timeout_ms, uint16_t expectedNodeId, SensorData &outData) {
  unsigned long startTime = millis();
  String line;
  
  while (millis() - startTime < timeout_ms) {
    if (readLineFrom(RAK, line, 100)) {
      LOG("RAK<", "Received: %s", line.c_str());
      
      // Check for both possible response formats
      if (line.startsWith("+RCV=") || line.startsWith("at+recv=")) {
        // Parse the response and extract data
        int rssi=0, snr=0, len=0; 
        String hex;
        
        if (line.startsWith("+RCV=")) {
          // Format: +RCV=<data>
          hex = line.substring(5);
          len = hex.length() / 2;
          rssi = -999; // Unknown RSSI for this format
          snr = 0;
        } else if (parseRecvLine(line, rssi, snr, len, hex)) {
          // Standard format handled by parseRecvLine
        } else {
          continue;
        }
        
        // Parse the response frame
        if (parseResponseFrame(hex, outData)) {
          if (outData.nodeId == expectedNodeId) {
            LOG("POLL", "Valid response from node 0x%04X", outData.nodeId);
            
            // Update node cache
            int idx = findOrAllocNode(outData.nodeId);
            nodes[idx].cnt = outData.count;
            nodes[idx].tC = outData.temperature;
            nodes[idx].hP = outData.humidity;
            nodes[idx].rssi = rssi;
            nodes[idx].seenMs = millis();
            
            // Update Blynk
            blynkSummary(outData.nodeId, outData.temperature, outData.humidity, rssi);
            
            return true;
          } else {
            LOG("POLL", "Response from wrong node (expected 0x%04X, got 0x%04X)", 
                expectedNodeId, outData.nodeId);
          }
        } else {
          LOG("POLL", "Failed to parse response frame");
        }
      }
    }
    yield(); // Allow other tasks to run
  }
  return false;
}

// Parse received response frame
bool parseResponseFrame(const String &hexData, SensorData &outData) {
  std::vector<uint8_t> bytes;
  if (!hexToBytes(hexData, bytes) || bytes.size() != 11 || bytes[0] != 0xA5) {
    return false;
  }

  uint16_t nodeId = (bytes[1] << 8) | bytes[2];
  uint16_t count = (bytes[3] << 8) | bytes[4];
  int16_t temp = (bytes[5] << 8) | bytes[6];
  int16_t hum = (bytes[7] << 8) | bytes[8];
  uint16_t rxCRC = (bytes[9] << 8) | bytes[10];
  uint16_t calcCRC = crc16_ibm(bytes.data(), 9);

  if (calcCRC != rxCRC) {
    return false;
  }

  outData.nodeId = nodeId;
  outData.count = count;
  outData.temperature = temp / 100.0f;
  outData.humidity = hum / 100.0f;
  
  return true;
}

// Handle complete request/response cycle
bool pollNode(uint16_t nodeId, SensorData &outData) {
  LOG("POLL", "Polling node 0x%04X", nodeId);
  
  // Send request
  if (!sendRequest(nodeId)) {
    LOG("POLL", "Failed to send request to node 0x%04X", nodeId);
    return false;
  }

  // Wait for response with proper data capture
  if (!waitForResponse(RESPONSE_TIMEOUT_MS, nodeId, outData)) {
    LOG("POLL", "No valid response from node 0x%04X (timeout)", nodeId);
    return false;
  }

  return true;
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

/************ Data Structures ************/
struct SensorData {
  uint16_t nodeId;
  uint16_t count;
  float temperature;
  float humidity;
};

/************ Response Handler ************/
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
// Enhanced AT command sending with better error handling
bool sendAT(const String& cmd, unsigned long to=2000) {
  LOG("RAK>", "%s", cmd.c_str());
  RAK.print(cmd); RAK.print("\r\n");
  String acc; unsigned long t0 = millis();
  while (millis()-t0 < to) {
    while (RAK.available()) {
      char c=(char)RAK.read();
      if (c=='\n') {
        acc.trim();
        if (acc.length() > 0) {
          LOG("RAK<", "%s", acc.c_str());
          if (acc.startsWith("OK")) return true;
          if (acc.startsWith("ERROR")) return false;
        }
        acc="";
      } else if (c!='\r') acc+=c;
    }
    yield(); // Allow other tasks to run
  }
  LOG("RAK", "Command timeout for: %s", cmd.c_str());
  return false;
}

// Verify radio mode
bool verifyRadioMode(int expectedMode) {
  // This would need specific RAK4270 commands to verify mode
  // For now, we'll trust our mode switches worked
  return true;
}

bool probeOK(unsigned long to=500) {
  RAK.flush(); RAK.print("AT\r\n");
  unsigned long t0 = millis(); String acc="";
  while (millis()-t0 < to) {
    while (RAK.available()) {
      char c=(char)RAK.read();
      if (c=='\n') { 
        acc.trim();
        bool ok = acc.startsWith("OK"); 
        acc=""; 
        if (ok) return true; 
      }
      else if (c!='\r') acc+=c;
    }
    yield();
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
  // Switch to P2P and set params (use same format as edge node)
  sendAT("at+set_config=lora:work_mode:1"); // 1=P2P
  delay(600); // Allow module to restart
  
  // Configure P2P parameters
  char buf[128];
  snprintf(buf,sizeof(buf),"at+set_config=lorap2p:%s:%d:%d:%d:%d:%d",
           P2P_FREQ_HZ, P2P_SF, P2P_BW, P2P_CR, P2P_PREAMBLE, P2P_PWR_DBM);
  sendAT(buf, 3000);
  
  // Start in TX mode (will toggle between TX/RX during operation)
  sendAT("at+set_config=lorap2p:transfer_mode:2"); // 2 = TX mode
}

/************ Wi-Fi/Blynk events ************/
void startWifi() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  
  // ESP32 WiFi event handling
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info){
    LOG("WIFI", "Got IP: %s", WiFi.localIP().toString().c_str());
    needBlynkConnect = true;            // defer actual connect to loop()
  }, ARDUINO_EVENT_WIFI_STA_GOT_IP);

  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info){
    LOG("WIFI", "Disconnected, reason=%d", info.wifi_sta_disconnected.reason);
    // no delay() here; just trigger reconnects from loop()
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    needBlynkConnect = false;           // will try again after we regain IP
  }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  
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
  LOG("BOOT", "ESP32 RAK P2P Gateway");

  // Initialize RAK UART on ESP32's HardwareSerial2
  RAK.begin(RAK_TARGET_BAUD, SERIAL_8N1, RAK_RX_PIN, RAK_TX_PIN);
  delay(200);

  Blynk.config(BLYNK_AUTH_TOKEN);   // non-blocking
  startWifi();

  if (!probeOK()) {
    LOG("RAK", "Failed to init UART link; check wiring & power");
  } else {
    LOG("RAK", "RAK module connected successfully");
    rakSetupP2P();
  }

  timer.setInterval(5000, [](){ Blynk.virtualWrite(V0, countNodes()); });
}

void loop() {
  static unsigned long lastPollTime = 0;
  static int currentNodeIndex = 0;
  static int retryCount = 0;
  static unsigned long lastRadioCheck = 0;
  
  // Handle Blynk connection
  if (WiFi.status() == WL_CONNECTED) {
    if (!Blynk.connected()) {
      if (needBlynkConnect && millis() - lastConnectAttempt > 5000UL) {
        lastConnectAttempt = millis();
        LOG("BLYNK", "Connecting...");
        bool ok = tryBlynkConnect(4000);
        LOG("BLYNK", "%s", ok ? "Connected" : "Connect timeout");
      }
    } else {
      Blynk.run();
    }
  }

  timer.run();

  // Periodic radio health check (every 30 seconds)
  if (millis() - lastRadioCheck > 30000) {
    lastRadioCheck = millis();
    if (!probeOK(1000)) {
      LOG("RAK", "Radio health check failed - attempting recovery");
      rakSetupP2P(); // Try to reinitialize
    }
  }

  // Poll nodes periodically
  if (millis() - lastPollTime >= POLL_INTERVAL_MS) {
    lastPollTime = millis();
    
    // Ensure we have nodes to poll
    if (NUM_EDGE_NODES == 0) {
      LOG("POLL", "No edge nodes configured");
      return;
    }
    
    // Get current node to poll
    uint16_t targetNode = EDGE_NODES[currentNodeIndex];
    
    SensorData data;
    if (pollNode(targetNode, data)) {
      LOG("POLL", "✓ Node: 0x%04X, Count: %u, Temp: %.2f°C, Humidity: %.2f%%",
          data.nodeId, data.count, data.temperature, data.humidity);
          
      // Reset retry count on success
      retryCount = 0;
      
      // Move to next node
      currentNodeIndex = (currentNodeIndex + 1) % NUM_EDGE_NODES;
      
    } else {
      LOG("POLL", "✗ Failed to poll node 0x%04X (attempt %d/%d)", 
          targetNode, retryCount + 1, MAX_RETRY_COUNT);
          
      // Increment retry count
      retryCount++;
      
      // If we've tried enough times, move to next node
      if (retryCount >= MAX_RETRY_COUNT) {
        retryCount = 0;
        currentNodeIndex = (currentNodeIndex + 1) % NUM_EDGE_NODES;
        LOG("POLL", "Moving to next node after %d failed attempts", MAX_RETRY_COUNT);
      }
    }
  }

  // Process any other incoming messages
  pumpRak();
